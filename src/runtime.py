"""
Runtime — the orchestrator that ties every subsystem together into one loop.

Both entry points use it:
    * main.py runs a Runtime directly (real or simulated hardware), and
    * the monitoring dashboard runs a Runtime in a background thread and pokes
      events into it while you watch.

Responsibilities:
    * build the hardware/comms/perception objects (real or simulated),
    * run the 20 Hz control loop (inputs -> command -> serial -> telemetry),
    * accept injected state-machine events (from the keyboard, the dashboard,
      or a scenario file) and feed them through the state machine, and
    * publish a per-tick status snapshot to the TelemetryHub for the dashboard.

Everything it does is logged, so a run produces a complete narrative of what
the robot decided and did — with real or simulated hardware, identically.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from utils.logger import get_logger
from utils.telemetry_hub import get_hub
from config.robot_config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT

from communication.protocol import serialize_command, get_emergency_packet
from communication.serial_link import SerialLink
from communication.packet_parser import parse_incoming

from control.drive_command import drive_command
from state_machine.manager import StateMachine
from state_machine.event_queue import EventQueue
from state_machine.events import EventType, make_event, EVENT_PRIORITY_MAP

from hardware.buttons import KeyboardOverrideListener
from hardware.imu import IMU
from hardware.tof import ToFArray
from hardware.diagnostics import run_startup_diagnostics

logger = get_logger("Runtime")


class Runtime:
    def __init__(self, simulated: bool = False, use_keyboard: bool = True,
                 use_camera: bool = True, loop_hz: float = 20.0):
        self.simulated = simulated
        self.loop_interval = 1.0 / loop_hz
        self._hub = get_hub()

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._events = EventQueue()

        # --- comms: real serial or a virtual ESP32 backend --------------------
        backend = None
        if simulated:
            from hardware.sim_esp32 import SimulatedESP32
            backend = SimulatedESP32()
        self.link = SerialLink(SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, backend=backend)
        self.serial_ok = self.link.connect()

        # --- sensors ----------------------------------------------------------
        self.imu = IMU(simulated=simulated)
        self.tof = ToFArray(simulated=simulated)

        # --- camera -----------------------------------------------------------
        self.camera = None
        if use_camera:
            try:
                from perception.camera import Camera
                self.camera = Camera(simulated=simulated)
            except Exception as e:
                logger.warning(f"Camera unavailable: {e}")
        camera_source = self.camera.source if self.camera else "none"

        # --- manual driving ---------------------------------------------------
        # Always create the listener so the dashboard can drive remotely even
        # with no physical keyboard; only attach the pynput hook if asked.
        self.listener = KeyboardOverrideListener()
        if use_keyboard:
            try:
                self.listener.start()
            except Exception as e:
                logger.warning(f"Keyboard listener unavailable: {e}")

        # --- decision core ----------------------------------------------------
        self.state_machine = StateMachine()

        run_startup_diagnostics(simulated, self.serial_ok, camera_source)
        self._mode_flag = 0

    # ============================================================ event inject
    def inject_event(self, event_type_name: str, source: str = "manual") -> bool:
        """Queue a state-machine event by name (e.g. 'START_BUTTON_PRESSED')."""
        try:
            etype = EventType[event_type_name]
        except KeyError:
            logger.error(f"Unknown event type: {event_type_name}")
            return False
        self._events.push(make_event(etype))
        self._hub.event(etype.name, EVENT_PRIORITY_MAP[etype].name, source=source)
        logger.info(f"EVENT injected: {etype.name} (source: {source})")
        return True

    def _drain_events(self) -> None:
        """Feed all queued events through the state machine, highest first."""
        for event in self._events.drain():
            old = self.state_machine.current_state
            self.state_machine.handle_event(event)
            new = self.state_machine.current_state
            if new != old:
                self._hub.state(old.name, new.name, via=event.type.name)
            self._post_event_effects(event)

    def _post_event_effects(self, event) -> None:
        """Bookkeeping the bare state machine doesn't do: record which pillar
        we're avoiding (so drive_command can steer around it), count laps, and
        auto-raise 'three laps complete'.
        """
        ctx = self.state_machine.context

        # drive_command() steers around a pillar based on context.last_pillar_color,
        # but nothing set it until now — so avoidance was always straight-ahead.
        # Latch the colour on detection and clear it once the obstacle is passed.
        if event.type == EventType.PILLAR_DETECTED_RED:
            ctx.last_pillar_color = "RED"
        elif event.type == EventType.PILLAR_DETECTED_GREEN:
            ctx.last_pillar_color = "GREEN"
        elif event.type == EventType.OBSTACLE_CLEARED:
            ctx.last_pillar_color = None

        if event.type == EventType.LAP_MARKER_DETECTED:
            ctx.increment_lap()
            if ctx.lap_count >= ctx.target_laps:
                self._events.push(make_event(EventType.THREE_LAPS_COMPLETE))
                logger.info("Target laps reached — raising THREE_LAPS_COMPLETE.")

    # =================================================================== driving
    def _resolve_command(self):
        """Return (speed, steer, action, mode_flag) from manual or autonomous."""
        manual = self.listener.is_manual_mode_active() if self.listener else False
        if manual:
            speed, steer, action = self.listener.get_manual_target()
            return speed, steer, action, 1
        speed, steer, action = drive_command(
            self.state_machine.current_state, self.state_machine.context
        )
        return speed, steer, action, 0

    def tick(self) -> None:
        """Run exactly one control cycle. Safe to call from a loop or a test."""
        self._drain_events()

        speed, steer, action, mode_flag = self._resolve_command()
        if mode_flag != self._mode_flag:
            self._hub.mode(bool(mode_flag))
            self._mode_flag = mode_flag

        # Record what we're commanding, in structured form and as a human line.
        source = "manual" if mode_flag else "auto"
        self._hub.command(speed, steer, action, source)
        ctx = self.state_machine.context
        ctx.speed, ctx.steering = speed, steer

        # Send to the ESP32 (real or virtual). comms is logged inside the link.
        packet = serialize_command(speed, steer, action, mode_flag)
        self.link.send(packet)

        # Drain any telemetry / status coming back. Loop so we don't fall behind
        # the 20 Hz telemetry stream from the (virtual) ESP32.
        for _ in range(8):
            incoming = self.link.read_line()
            if not incoming:
                break
            parse_incoming(incoming)

        # Fold IMU heading into the shared context.
        imu = self.imu.read()
        ctx.heading = imu["heading_deg"]

        # One tidy status record the dashboard renders as the "current state".
        tof = self.tof.read()
        self._hub.publish("status", {
            "state": self.state_machine.current_state.name,
            "mode": "MANUAL" if mode_flag else "AUTO",
            "speed": speed, "steer": steer, "action": action,
            "lap": ctx.lap_count, "target_laps": ctx.target_laps,
            "heading_deg": round(imu["heading_deg"], 1),
            "imu": {k: round(v, 3) for k, v in imu.items() if k != "heading_deg"},
            "tof": {"left_mm": round(tof["left_mm"], 1), "right_mm": round(tof["right_mm"], 1)},
            "simulated": self.simulated,
        })

    # ==================================================================== loops
    def run_forever(self) -> None:
        """Blocking run loop (used by main.py)."""
        self._running = True
        logger.info(
            "\n\n==============================================="
            "\nHermes Autonomous Vehicle Software Starting..."
            f"\nMode: {'SIMULATION' if self.simulated else 'REAL HARDWARE'}"
            "\n==============================================="
        )
        logger.info("Manual controls: w/s drive, a/d steer, space stop, m toggle.")
        try:
            while self._running:
                start = time.time()
                self.tick()
                elapsed = time.time() - start
                time.sleep(max(0.0, self.loop_interval - elapsed))
        except KeyboardInterrupt:
            logger.info("Shutdown requested.")
        except Exception as run_err:
            logger.error(f"Runtime error: {run_err}", exc_info=True)
        finally:
            self.shutdown()

    def start_background(self) -> None:
        """Run the loop in a daemon thread (used by the dashboard)."""
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._bg_loop, daemon=True, name="RuntimeLoop")
        self._thread.start()
        logger.info("Runtime loop started in background.")

    def _bg_loop(self) -> None:
        try:
            while self._running:
                start = time.time()
                self.tick()
                elapsed = time.time() - start
                time.sleep(max(0.0, self.loop_interval - elapsed))
        except Exception as e:
            logger.error(f"Background runtime error: {e}", exc_info=True)

    def stop(self) -> None:
        self._running = False

    def shutdown(self) -> None:
        logger.warning("Triggering safety shutdown.")
        self._running = False
        try:
            self.link.send_emergency(get_emergency_packet(mode=1))
        except Exception as e:
            logger.error(f"Emergency send failed: {e}")
        if self.listener:
            self.listener.stop()
        if self.camera:
            self.camera.close()
        logger.info("Shutdown completed.")
