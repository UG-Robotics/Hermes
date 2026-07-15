"""
Central telemetry hub — the single nervous system of the robot software.

Every subsystem (perception, state machine, control, communication, hardware)
publishes structured records here. Two kinds of consumer read them back:

    * the file/console logger (so everything is persisted), and
    * the monitoring dashboard (so everything is visible live).

The hub is deliberately hardware-agnostic. It works identically whether the
underlying hardware is real or simulated — which is the whole point: the logs
and the dashboard behave the same way on the bench with no motors as they do
on a fully working robot.

Design notes
------------
* Thread-safe. The control loop, the serial reader and the Flask/SSE workers
  all touch it from different threads.
* Never logs anything itself (that would recurse through the logging bridge).
* `snapshot()` returns the latest value seen on every channel — this is what a
  freshly-connected dashboard renders before the live stream catches up.
* `subscribe()` hands out an independent queue; each dashboard client gets one.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from queue import Queue, Full
from typing import Any, Dict, List, Optional


# Channels are just strings, but we name the well-known ones so producers and
# consumers agree. Anything may be published; these are the ones the dashboard
# knows how to render.
CH_LOG = "log"                # a log record (level, module, message)
CH_STATE = "state"           # state-machine state changed
CH_MODE = "mode"             # manual/auto mode changed
CH_COMMAND = "command"       # a drive command was produced (speed/steer/action)
CH_ACTION = "action"         # a human-readable action the bot is taking
CH_COMMS = "comms"           # a serial packet crossed the Pi<->ESP32 wire
CH_TELEMETRY = "telemetry"   # IMU + ToF readings arrived
CH_EVENT = "event"           # a state-machine event was raised/injected
CH_CAMERA = "camera"         # camera frame metadata (not the frame bytes)


class TelemetryHub:
    """Process-wide singleton. Use :func:`get_hub` to obtain it."""

    def __init__(self, log_buffer_size: int = 500):
        self._lock = threading.Lock()
        self._subscribers: List["Queue[Dict[str, Any]]"] = []
        self._snapshot: Dict[str, Dict[str, Any]] = {}
        self._log_buffer: "deque[Dict[str, Any]]" = deque(maxlen=log_buffer_size)
        self._seq = 0

    # ------------------------------------------------------------------ publish
    def publish(self, channel: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Publish a record on a channel. Returns the enriched record.

        Enrichment adds a monotonically increasing ``seq`` and a wall-clock
        ``ts`` so consumers can order and timestamp records without guessing.
        """
        with self._lock:
            self._seq += 1
            record = {
                "seq": self._seq,
                "ts": time.time(),
                "channel": channel,
                **data,
            }

            # Latest-value snapshot, keyed by channel.
            self._snapshot[channel] = record

            # Keep a rolling window of log records for late-joining dashboards.
            if channel == CH_LOG:
                self._log_buffer.append(record)

            # Fan out to every live subscriber. Drop (rather than block) if a
            # slow client's queue is full — the control loop must never stall
            # because a browser tab fell behind.
            dead: List["Queue[Dict[str, Any]]"] = []
            for q in self._subscribers:
                try:
                    q.put_nowait(record)
                except Full:
                    dead.append(q)
            for q in dead:
                # A perpetually-full queue means the client is gone/stuck.
                try:
                    self._subscribers.remove(q)
                except ValueError:
                    pass

            return record

    # --------------------------------------------------------------- subscribe
    def subscribe(self, maxsize: int = 1000) -> "Queue[Dict[str, Any]]":
        """Register a new consumer and return its private queue."""
        q: "Queue[Dict[str, Any]]" = Queue(maxsize=maxsize)
        with self._lock:
            self._subscribers.append(q)
        return q

    def unsubscribe(self, q: "Queue[Dict[str, Any]]") -> None:
        with self._lock:
            try:
                self._subscribers.remove(q)
            except ValueError:
                pass

    # ---------------------------------------------------------------- snapshot
    def snapshot(self) -> Dict[str, Dict[str, Any]]:
        """Return the latest record seen on every channel."""
        with self._lock:
            return dict(self._snapshot)

    def recent_logs(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        with self._lock:
            logs = list(self._log_buffer)
        if limit is not None:
            return logs[-limit:]
        return logs

    # ------------------------------------------------------- typed convenience
    # These are thin helpers so call sites read well. They all funnel into
    # publish() so there is exactly one code path.
    def state(self, old: str, new: str, **extra: Any) -> None:
        self.publish(CH_STATE, {"old": old, "new": new, **extra})

    def mode(self, manual: bool) -> None:
        self.publish(CH_MODE, {"manual": manual, "label": "MANUAL" if manual else "AUTO"})

    def command(self, speed: int, steer: int, action: str, source: str) -> None:
        self.publish(
            CH_COMMAND,
            {"speed": speed, "steer": steer, "action": action, "source": source},
        )

    def action(self, message: str, **extra: Any) -> None:
        self.publish(CH_ACTION, {"message": message, **extra})

    def comms(self, direction: str, packet: str, **extra: Any) -> None:
        # direction: "tx" (Pi->ESP32) or "rx" (ESP32->Pi)
        self.publish(CH_COMMS, {"direction": direction, "packet": packet.strip(), **extra})

    def telemetry(self, values: Dict[str, float], source: str = "esp32") -> None:
        self.publish(CH_TELEMETRY, {"values": values, "source": source})

    def event(self, event_type: str, priority: str, source: str = "system") -> None:
        self.publish(CH_EVENT, {"event_type": event_type, "priority": priority, "source": source})

    def camera(self, width: int, height: int, fps: float, source: str) -> None:
        self.publish(
            CH_CAMERA,
            {"width": width, "height": height, "fps": round(fps, 1), "source": source},
        )


_hub: Optional[TelemetryHub] = None
_hub_lock = threading.Lock()


def get_hub() -> TelemetryHub:
    """Return the process-wide TelemetryHub, creating it on first use."""
    global _hub
    if _hub is None:
        with _hub_lock:
            if _hub is None:
                _hub = TelemetryHub()
    return _hub
