"""
IMU interface.

OWNERSHIP: the IMU is physical hardware wired to the ESP32's I2C bus (see
firmware/esp_controller/imu.cpp). The Pi NEVER opens an I2C bus or reads a
register itself -- doing so here would be wrong, since the Pi has no wire to
that sensor at all. The ESP32 is the only thing that reads raw accel/gyro
registers; it streams the result up as a TEL packet, which
communication/packet_parser.py decodes and publishes to the TelemetryHub via
hub.telemetry(). This class only ever reads that published value back out
(see _latest() below) -- real hardware and hardware/sim_esp32.py's simulated
telemetry are indistinguishable from here, which is exactly the point.

The one thing this class DOES compute on the Pi -- integrating gz (yaw rate)
into a running heading estimate -- is a CONTROL computation over data the
ESP32 already sent, not a hardware read. That distinction matters: it's what
lets control/steering_control.py's heading-hold PID live on the Pi (where the
rest of the decision-making already is) without the Pi pretending to own the
sensor. If you ever find yourself adding smbus/board.SCL/RPi.GPIO imports to
this file, stop -- that reading belongs in firmware/esp_controller/imu.cpp.
"""

from __future__ import annotations

import time
from typing import Optional

from utils.logger import get_logger
from utils.telemetry_hub import get_hub, CH_TELEMETRY

logger = get_logger(__name__)


class IMU:
    def __init__(self, simulated: bool = False):
        self._hub = get_hub()
        self.simulated = simulated
        self._heading_deg = 0.0
        self._last_ts: Optional[float] = None
        source = "SIMULATED" if simulated else "ESP32 telemetry"
        logger.info(f"IMU interface ready (source: {source}).")

    def _latest(self) -> Optional[dict]:
        rec = self._hub.snapshot().get(CH_TELEMETRY)
        return rec["values"] if rec else None

    def read(self) -> dict:
        """Return the latest IMU reading, integrating heading from yaw rate.

        Returns a dict with ax, ay, az, gx, gy, gz and a derived heading_deg.
        Falls back to zeros (level, stationary) before any telemetry arrives.
        """
        values = self._latest()
        if not values:
            return {"ax": 0.0, "ay": 0.0, "az": 9.81,
                    "gx": 0.0, "gy": 0.0, "gz": 0.0, "heading_deg": self._heading_deg}

        now = time.time()
        if self._last_ts is not None:
            dt = now - self._last_ts
            # gz is deg/s; integrate to a heading estimate.
            self._heading_deg += values.get("gz", 0.0) * dt
            self._heading_deg = (self._heading_deg + 180.0) % 360.0 - 180.0
        self._last_ts = now

        return {**values, "heading_deg": self._heading_deg}

    @property
    def heading(self) -> float:
        return self._heading_deg

    def reset_heading(self) -> None:
        self._heading_deg = 0.0
        self._last_ts = None
        logger.info("IMU heading reset to 0.")

    def healthy(self) -> bool:
        """True once at least one telemetry packet has been seen."""
        return self._latest() is not None
