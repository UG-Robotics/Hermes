"""
ToF + IMU corridor centering.

Replaces the old camera-based lane centering (perception/track_detection.py +
planning/lane_centering.py, both removed): the forward camera's field of view
is too narrow to see both walls reliably, so corridor centering now comes from
the two side ToF sensors instead. The camera is left to do only what nothing
else can -- spot the coloured pillars, the orange/blue corner markers, and the
magenta parking zone.

The ToFs give perpendicular distance to the left and right walls
(hardware/tof.py, ESP32-owned). This module turns those two distances into the
same thing the camera version produced: a small per-tick heading NUDGE
(control.steering_control.SteeringController.nudge_target) that drags the IMU
heading-hold target toward the middle of the lane. The IMU still does all the
yaw stabilisation; the ToF only corrects lateral position. They compose:

    * IMU heading-hold  -> keeps the car pointed straight (yaw drift),
    * ToF centering     -> keeps the car in the middle (lateral offset).

Three regimes, chosen by how many walls are actually in range:

    * BOTH walls seen -> differential centering: steer to equalise the left
      and right distances (optionally biased toward the inner wall in an OPEN
      run). This is the "stabilise the heading when we're a good distance from
      both walls and essentially in the middle of the lane" case.
    * ONE wall seen   -> wall-follow: hold a fixed target clearance
      (TOF_WALL_FOLLOW_TARGET_MM) to the single wall in range -- corners, wall
      gaps, and the wide edge of an OPEN corridor where the far wall is beyond
      ToF range.
    * NO walls seen   -> no correction; coast on the locked IMU heading.

Sign convention matches everywhere else (config/robot_config.py): +nudge =
steer right. We track a signed lateral `offset_mm` where + means "displaced
toward the RIGHT wall" (the right wall reads closer); the correction is always
nudge = -KP * offset, i.e. displaced-right -> steer left, displaced-left ->
steer right.

Step (v) -- sending anything to the ESP32 -- is not done here; runtime.py owns
the serial link, the same boundary the rest of planning/ keeps.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from config.thresholds import (
    TOF_WALL_SEEN_MAX_MM,
    TOF_CENTER_KP,
    TOF_CENTER_DEADBAND_MM,
    TOF_CENTER_MAX_NUDGE_DEG,
    TOF_WALL_FOLLOW_TARGET_MM,
)


@dataclass
class CenteringObservation:
    """Result of one compute_centering_nudge() call."""
    valid: bool
    nudge_deg: float = 0.0
    mode: str = "NONE"                  # "BOTH" | "LEFT_WALL" | "RIGHT_WALL" | "NONE"
    offset_mm: Optional[float] = None   # signed lateral offset, + = toward RIGHT wall (pre-bias)
    left_seen: bool = False
    right_seen: bool = False


def _wall_seen(distance_mm: Optional[float]) -> bool:
    """A ToF reading counts as a wall only if it's in range. At/above
    TOF_WALL_SEEN_MAX_MM the sensor is effectively saying 'no wall this side'
    (the far side of a wide corridor, a corner opening, or its out-of-range
    sentinel), not 'a wall very far away' we should steer relative to."""
    return distance_mm is not None and distance_mm < TOF_WALL_SEEN_MAX_MM


def compute_centering_nudge(left_mm: Optional[float], right_mm: Optional[float],
                            bias_mm: float = 0.0) -> CenteringObservation:
    """Turn the two side ToF distances into a heading nudge (degrees, + right).

    bias_mm shifts the target lateral position in BOTH-walls mode only, in the
    same +right convention as offset_mm: +bias => sit that many mm closer to
    the RIGHT wall, -bias => closer to the LEFT. runtime.py sets it from the
    OPEN-run inner-wall side (see config/thresholds.py INNER_WALL_BIAS_MM);
    0.0 (the default, and always in an OBSTACLE run) is pure centre-hold so the
    car can pass pillars on either side.
    """
    left_seen = _wall_seen(left_mm)
    right_seen = _wall_seen(right_mm)

    if left_seen and right_seen:
        # Half the difference so offset_mm is an actual lateral displacement in
        # mm (move x mm right and left grows by x while right shrinks by x, so
        # left-right changes by 2x). Halving keeps TOF_CENTER_KP meaning "deg
        # per mm of offset" and makes the BOTH <-> one-wall handover
        # gain-continuous. + = displaced toward the right wall.
        offset = (left_mm - right_mm) / 2.0
        biased = offset - bias_mm
        mode = "BOTH"
    elif left_seen:
        # Only the left wall in range: hold TOF_WALL_FOLLOW_TARGET_MM to it.
        # Farther than target from the left wall == displaced right == +offset.
        offset = left_mm - TOF_WALL_FOLLOW_TARGET_MM
        biased = offset
        mode = "LEFT_WALL"
    elif right_seen:
        # Only the right wall: nearer than target to it == displaced right.
        offset = TOF_WALL_FOLLOW_TARGET_MM - right_mm
        biased = offset
        mode = "RIGHT_WALL"
    else:
        return CenteringObservation(valid=False, mode="NONE")

    if abs(biased) <= TOF_CENTER_DEADBAND_MM:
        # Centred enough; don't hunt over sensor noise.
        return CenteringObservation(
            valid=True, nudge_deg=0.0, mode=mode, offset_mm=offset,
            left_seen=left_seen, right_seen=right_seen,
        )

    # Displaced toward the right wall (+offset) -> steer left (-nudge).
    raw_nudge = -TOF_CENTER_KP * biased
    nudge = max(-TOF_CENTER_MAX_NUDGE_DEG, min(TOF_CENTER_MAX_NUDGE_DEG, raw_nudge))
    return CenteringObservation(
        valid=True, nudge_deg=nudge, mode=mode, offset_mm=offset,
        left_seen=left_seen, right_seen=right_seen,
    )
