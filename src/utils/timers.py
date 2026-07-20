"""
Small timing helpers.

The runtime's own control loop uses time.time() directly (its cadence and the
start/parking clocks all live in runtime.py), but these two utilities are the
reusable primitives behind that pattern for anywhere else that needs "how long
since X" or "do something every N seconds" without re-implementing the
bookkeeping. Pure standard library -- safe to import on a bare Python install.
"""

from __future__ import annotations

import time


class Stopwatch:
    """Monotonic elapsed-time counter. Reset it at an event, ask elapsed()
    later. Uses time.monotonic() so a system clock change can't make time
    appear to run backwards mid-run."""

    def __init__(self, start: bool = True):
        self._start: float | None = time.monotonic() if start else None

    def reset(self) -> None:
        self._start = time.monotonic()

    def stop(self) -> None:
        self._start = None

    @property
    def running(self) -> bool:
        return self._start is not None

    def elapsed(self) -> float:
        """Seconds since the last reset()/construction, or 0.0 if stopped."""
        return 0.0 if self._start is None else (time.monotonic() - self._start)

    def expired(self, seconds: float) -> bool:
        """True once `seconds` have elapsed since the last reset."""
        return self.running and self.elapsed() >= seconds


class PeriodicTimer:
    """Fires at most once per `period` seconds. `ready()` returns True and
    re-arms when the period has elapsed, False otherwise -- a rate limiter for
    things like throttled log lines or telemetry that shouldn't run every
    20 Hz tick."""

    def __init__(self, period: float):
        self.period = period
        self._last = time.monotonic()

    def ready(self) -> bool:
        now = time.monotonic()
        if now - self._last >= self.period:
            self._last = now
            return True
        return False

    def reset(self) -> None:
        self._last = time.monotonic()
