"""
Small signal-shaping helpers for the control loop.

Currently just a slew-rate limiter, used to keep servo commands smooth (no
sudden snaps) even when the heading-hold PID wants a big correction in one
tick — see control/steering_control.py.
"""

from __future__ import annotations


class SlewRateLimiter:
    """Limits how much a value can change per step, regardless of the target."""

    def __init__(self, max_delta_per_step: float, initial: float = 0.0):
        self.max_delta_per_step = abs(max_delta_per_step)
        self._value = initial

    def reset(self, value: float = 0.0) -> None:
        self._value = value

    def step(self, target: float) -> float:
        delta = target - self._value
        if delta > self.max_delta_per_step:
            delta = self.max_delta_per_step
        elif delta < -self.max_delta_per_step:
            delta = -self.max_delta_per_step
        self._value += delta
        return self._value

    @property
    def value(self) -> float:
        return self._value
