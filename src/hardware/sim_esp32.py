"""
Virtual ESP32 — a software stand-in for the real microcontroller + hardware.

The real robot's motors, servo, IMU and ToF sensors all live on the ESP32 and
are reached from the Pi over the serial link. When that hardware isn't working
(or isn't attached), this class plays the ESP32's role: it speaks the exact
same line protocol (CMD/EMG/PING/ACK in, STATUS/TEL/ACK out), plus the local
`EVT,START_BUTTON_PRESSED` test hook, so nothing above the serial layer can
tell the difference. The control loop, logging and dashboard all behave as if
a fully working robot were connected.

It also runs a deliberately simple motion model so the telemetry it returns is
*plausible and reactive* rather than static:

        * A forward/backward command produces forward acceleration; the IMU's ax
            reflects it and az sits at ~1 g (gravity), like a real accelerometer.
        * A steering command produces a yaw rate on gz and rotates the heading.
        * Two ToF sensors (left, right) report distance to virtual side walls in a
            ~1 m wide corridor; steering changes those distances the way walls would
            appear to approach as the car angles toward them.

This is not a physics engine — it is just enough dynamics that the numbers move
sensibly when you drive, which is what makes the dashboard and scenario runs
worth watching.
"""
from __future__ import annotations

import math
import random
import threading
import time

from utils.logger import get_logger
from utils.telemetry_hub import get_hub

logger = get_logger(__name__)


class SimulatedESP32:
    """A virtual microcontroller that reads command packets and writes telemetry."""

    # Corridor / geometry assumptions for the ToF model (millimetres).
    CORRIDOR_WIDTH_MM = 1000.0
    TOF_MAX_MM = 2000.0
    TELEMETRY_PERIOD_S = 0.05  # 20 Hz, matches the real firmware's cadence

    def __init__(self, noise: float = 0.02):
        self._hub = get_hub()
        self._noise = noise

        # The CommandDispatcher thread now calls write_line() while the control
        # loop calls read_line(); both touch command/physics state, so guard
        # them. Command re-sends are value-identical, so this only prevents a
        # torn read, never changes the simulated motion.
        self._io_lock = threading.Lock()

        # --- command state (what the "firmware" was last told to do) ----------
        self._action = "STOP"
        self._speed = 0        # 0..255 as sent by the Pi
        self._steer = 0        # -90..90
        self._emergency = False

        # --- simulated physical state -----------------------------------------
        self._velocity = 0.0        # signed, m/s (+forward)
        self._heading = 0.0         # radians, 0 = straight down the corridor
        self._lateral = 0.0         # m offset from corridor centre (+ = right)
        self._last_step = time.time()
        self._last_tel = 0.0

        # Outbound line buffer: lines the "ESP32" has produced for the Pi to read.
        self._outbox: list[str] = []

        logger.info("Virtual ESP32 online (simulated motors, servo, IMU, dual ToF).")
        # Mirror the real firmware's boot STATUS line so packet_parser sees it.
        self._outbox.append("STATUS,Serial Initialised (SIMULATED)")

    # ================================================================= inbound
    def write_line(self, line: str) -> None:
        """Handle one packet from the Pi. Mirrors serial_protocol.cpp parsing."""
        text = (line or "").strip()
        if not text:
            return

        parts = text.split(",")
        tag = parts[0].strip().upper()

        with self._io_lock:
            if tag == "CMD" and len(parts) == 5:
                self._speed = _safe_int(parts[1])
                self._steer = _safe_int(parts[2])
                self._action = parts[3].strip().upper()
                self._emergency = False
            elif tag == "EMG":
                self._speed = 0
                self._steer = 0
                self._action = "STOP"
                self._emergency = True
                self._outbox.append("STATUS,EMG ack")
            elif tag == "EVT" and len(parts) == 2:
                name = parts[1].strip().upper()
                if name == "START_BUTTON_PRESSED":
                    self._outbox.append("EVT,START_BUTTON_PRESSED")
                else:
                    self._outbox.append(f"STATUS,ERR unsupported EVT: {name}")
            elif tag == "PING":
                self._outbox.append("ACK,PING")
            elif tag == "ACK":
                pass  # Pi acknowledged something; nothing to do
            else:
                self._outbox.append(f"STATUS,ERR unknown packet: {text}")

    # ================================================================ outbound
    def read_line(self):
        """Return one queued line for the Pi, or None. Mirrors a serial read."""
        with self._io_lock:
            self._advance()
            if self._outbox:
                return self._outbox.pop(0)
            return None

    # ================================================================= physics
    def _advance(self) -> None:
        """Integrate the motion model and, at 20 Hz, queue a TEL packet."""
        now = time.time()
        dt = now - self._last_step
        if dt <= 0:
            return
        self._last_step = now
        dt = min(dt, 0.1)  # clamp so a paused process doesn't teleport the model

        # Map the 0..255 speed byte to a modest real-world velocity target.
        target_speed_ms = (self._speed / 255.0) * 1.2  # up to ~1.2 m/s
        if self._action == "FORWARD":
            target_v = target_speed_ms
        elif self._action == "BACKWARD":
            target_v = -target_speed_ms
        else:
            target_v = 0.0

        # First-order approach to target velocity (motor + inertia lag).
        self._velocity += (target_v - self._velocity) * min(1.0, dt * 4.0)

        # Steering -> yaw rate. Only meaningful while actually moving.
        steer_rad = math.radians(self._steer)
        yaw_rate = steer_rad * self._velocity * 2.0  # rad/s
        self._heading += yaw_rate * dt
        self._heading = max(-math.pi / 2, min(math.pi / 2, self._heading))

        # Lateral drift within the corridor from the current heading.
        self._lateral += self._velocity * math.sin(self._heading) * dt
        half = self.CORRIDOR_WIDTH_MM / 2000.0  # metres to each wall
        self._lateral = max(-half, min(half, self._lateral))

        if now - self._last_tel >= self.TELEMETRY_PERIOD_S:
            self._last_tel = now
            self._outbox.append(self._build_telemetry(target_v, yaw_rate))

    def _build_telemetry(self, target_v: float, yaw_rate: float) -> str:
        """Compose a TEL packet consistent with parse_incoming's field order:
        TEL,ax,ay,az,gx,gy,gz,tof1_mm,tof2_mm
        """
        # Longitudinal acceleration ~ how fast we're chasing the target velocity.
        ax = (target_v - self._velocity) * 4.0 + self._n()
        ay = self._velocity * yaw_rate + self._n()  # centripetal-ish
        az = 9.81 + self._n()                        # gravity, m/s^2

        gx = self._n(0.5)
        gy = self._n(0.5)
        gz = math.degrees(yaw_rate) + self._n(1.0)   # deg/s about vertical

        # ToF: distance to left/right walls given lateral offset + heading.
        half_mm = self.CORRIDOR_WIDTH_MM / 2.0
        lateral_mm = self._lateral * 1000.0
        # Angling toward a wall shortens the beam to it (cheap cos model).
        angle_factor = max(0.2, math.cos(self._heading))
        left = (half_mm + lateral_mm) / angle_factor + self._n(5.0)
        right = (half_mm - lateral_mm) / angle_factor + self._n(5.0)
        left = max(20.0, min(self.TOF_MAX_MM, left))
        right = max(20.0, min(self.TOF_MAX_MM, right))

        return (
            f"TEL,{ax:.3f},{ay:.3f},{az:.3f},"
            f"{gx:.3f},{gy:.3f},{gz:.3f},"
            f"{left:.1f},{right:.1f}"
        )

    def _n(self, scale: float = 1.0) -> float:
        """Gaussian sensor noise scaled by the configured noise level."""
        return random.gauss(0.0, self._noise * scale)


def _safe_int(value: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0
