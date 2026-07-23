"""
IMU heading-hold steering controller.

This is the "IMU acts as a PID, always checking the orientation of the bot"
layer from the spec: it runs every tick, compares the IMU's current heading
against a target heading, and outputs a smoothed steering correction.

How the target heading gets set (by runtime.py):
    * hold_straight(heading)  - "go straight from here" - locked once on
      entry to FOLLOW_TRACK and again whenever a pillar is cleared, so drift
      doesn't accumulate across a whole lap.
    * turn_by(heading, delta) - "you need to turn `delta` degrees from here"
      - used when the pillar-avoidance planner (perception/pillar_detection.py)
      decides a steering angle: instead of firing that angle open-loop, we
      convert it into a target heading and let the PID *drive the servo to
      hold that heading*, which is what makes the turn come out smooth and
      accurate regardless of wheel slip, battery sag, etc.

The output of compute() is what actually gets serialized and sent to the
ESP32 as the CMD packet's steer field — this is how "the IMU communicates
with the servo": every steer value that reaches communication/protocol.py in
autonomous mode has already been corrected against the IMU.

OWNERSHIP: this is the one Pi-side computation that's allowed to touch IMU
data. It never reads the sensor (hardware/imu.py's module docstring covers
that boundary) — it only consumes the heading value hardware/imu.py already
integrated from ESP32 telemetry, and turns it into a steering correction.
That's a control decision, not hardware access, which is why it belongs
here on the Pi rather than on the ESP32.
"""

from __future__ import annotations

from control.pid import PIDController
from control.filters import SlewRateLimiter
from config.pid_config import (
    STEER_PID_KP, STEER_PID_KI, STEER_PID_KD,
    STEER_PID_OUTPUT_MIN, STEER_PID_OUTPUT_MAX,
    STEER_PID_INTEGRAL_MIN, STEER_PID_INTEGRAL_MAX,
    STEER_SLEW_MAX_DEG_PER_TICK,
)
from config.robot_config import STEER_MIN, STEER_MAX
from utils.logger import get_logger

logger = get_logger(__name__)


def _wrap_deg(angle: float) -> float:
    """Wrap an angle to (-180, 180], so heading error never takes the long
    way around through 180/-180."""
    return (angle + 180.0) % 360.0 - 180.0


class SteeringController:
    def __init__(self):
        self._pid = PIDController(
            kp=STEER_PID_KP, ki=STEER_PID_KI, kd=STEER_PID_KD,
            output_min=STEER_PID_OUTPUT_MIN, output_max=STEER_PID_OUTPUT_MAX,
            integral_min=STEER_PID_INTEGRAL_MIN, integral_max=STEER_PID_INTEGRAL_MAX,
        )
        self._slew = SlewRateLimiter(STEER_SLEW_MAX_DEG_PER_TICK)
        self._target_heading: float | None = None

    # ------------------------------------------------------------- targeting
    def hold_straight(self, current_heading_deg: float) -> None:
        """Lock the current heading as 'straight ahead'."""
        self._target_heading = _wrap_deg(current_heading_deg)
        self._pid.reset()

    def turn_by(self, current_heading_deg: float, delta_deg: float) -> None:
        """Lock a new target heading, `delta_deg` away from current heading.
        Positive delta = turn right, matching the steer sign convention."""
        self._target_heading = _wrap_deg(current_heading_deg + delta_deg)
        self._pid.reset()

    def nudge_target(self, delta_deg: float) -> None:
        """Incrementally shift the already-locked target heading by
        `delta_deg`, WITHOUT resetting the PID.

        This is the continuous-correction counterpart to turn_by(): turn_by()
        is for a one-shot manoeuvre (a fresh pillar detection) where
        resetting the PID's integral/derivative history is exactly right.
        Calling turn_by() every tick instead (e.g. from
        planning/wall_centering.py, which computes a new nudge every tick)
        would reset that history every tick and turn the heading-hold loop
        jittery. nudge_target() just drags the aim point a little and lets
        the PID's existing state keep smoothing the servo toward it.

        No-op if hold_straight()/turn_by() haven't locked a target yet (the
        next compute() call will lock one from the current heading, same as
        before this method existed).
        """
        if self._target_heading is None:
            return
        self._target_heading = _wrap_deg(self._target_heading + delta_deg)

    @property
    def target_heading_deg(self) -> float | None:
        return self._target_heading

    # -------------------------------------------------------------- control
    def compute(self, current_heading_deg: float, dt: float) -> tuple[int, float]:
        """Run one control tick. Returns (steer_command, heading_error_deg).

        If no target has been locked yet, holds the current heading (so the
        very first tick after boot doesn't try to correct against a bogus
        0.0 default) and returns steer=0.
        """
        if self._target_heading is None:
            self.hold_straight(current_heading_deg)
            return 0, 0.0

        error = _wrap_deg(self._target_heading - current_heading_deg)
        correction = self._pid.update(error, dt)
        smoothed = self._slew.step(correction)
        steer = int(max(STEER_MIN, min(STEER_MAX, round(smoothed))))
        return steer, error

    def reset(self) -> None:
        """Full reset — used on ERROR->INIT recovery or manual->auto handoff
        so stale integral/slew state from before doesn't leak into the next run."""
        self._target_heading = None
        self._pid.reset()
        self._slew.reset(0.0)
