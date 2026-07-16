"""
Generic PID controller.

Used by control/steering_control.py to turn an IMU heading error into a
steering correction, but deliberately has no knowledge of steering, IMUs, or
degrees vs. anything else — it's a plain, reusable PID so it can also back
speed control or lane-centering later without duplicating this logic.
"""

from __future__ import annotations


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -float("inf"), output_max: float = float("inf"),
                 integral_min: float = -float("inf"), integral_max: float = float("inf")):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max

        self._integral = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        """Clear accumulated state. Call whenever the target changes abruptly
        (e.g. a new heading is locked) so old error history doesn't cause a
        derivative/integral kick."""
        self._integral = 0.0
        self._prev_error = None

    def update(self, error: float, dt: float) -> float:
        """Advance the controller by one tick and return the control output.

        error: setpoint - measurement, in whatever units the caller defines.
        dt:    seconds since the previous update() call. Must be > 0 for the
               I and D terms to be meaningful; dt <= 0 is treated as "skip
               integration this tick" so a stalled loop can't inject a huge
               derivative spike.
        """
        if dt <= 0:
            return max(self.output_min, min(self.output_max, self.kp * error))

        self._integral += error * dt
        self._integral = max(self.integral_min, min(self.integral_max, self._integral))

        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        return max(self.output_min, min(self.output_max, output))
