"""
Continuous camera-based lane (corridor) centering.

Turns perception.track_detection's per-frame LaneObservation into a small
heading nudge that keeps the IMU heading-hold loop's locked target pointed
at the corridor's actual centre, instead of just "wherever we happened to
be pointed when FOLLOW_TRACK started" (control/steering_control.py's
hold_straight(), currently only re-locked once at START_BUTTON_PRESSED and
again whenever a pillar is cleared). Gyro-only heading drifts over a lap;
this closes that loop using the one absolute reference the Pi actually
has -- the walls themselves, via the camera.

This is deliberately NOT a second PID fighting the heading-hold PID. It
only ever nudges *where the target heading points*
(control.steering_control.SteeringController.nudge_target), a small,
clamped, deadbanded amount each tick -- so it looks like the same one
steering loop slowly dragging its aim point toward the corridor centre. The
heading-hold PID still does all the actual servo-smoothing work, the same
way it already does for the once-per-detection pillar-avoidance angle in
control/drive_command.py's AVOID_OBSTACLE branch.

Step (v) -- actually sending anything to the ESP32 -- is not done here.
runtime.py is the only place that talks to communication/*, consistent with
perception/pillar_detection.py's module docstring.
"""

from __future__ import annotations

from perception.track_detection import LaneObservation
from config.thresholds import (
    LANE_STEER_KP, LANE_MAX_NUDGE_DEG, LANE_MIN_CONFIDENCE, LANE_OFFSET_DEADBAND_PX,
)


def compute_heading_nudge(observation: LaneObservation, frame_width: int,
                          bias_px: float = 0.0) -> float:
    """Return a small heading-correction nudge in degrees (+ = turn right),
    or 0.0 if this observation isn't trustworthy enough to act on.

    frame_width is accepted (not currently used beyond being available for
    future width-normalised gains) to keep this function's signature
    self-describing at call sites, mirroring
    perception.pillar_detection.compute_steer_angle(frame_width, blob).

    bias_px shifts the corridor-centre target the car aims to hold, in the
    same +right/-left pixel convention as offset_px. runtime.py uses it to
    hug the INNER wall during the OPEN challenge (+ = bias toward the right
    wall, - = toward the left); 0.0 (the default) is pure centre-hold, which
    is what the Obstacle Challenge always uses so the car can pass pillars on
    either side. See config/thresholds.py's INNER_WALL_BIAS_PX.
    """
    if not observation.valid or observation.confidence < LANE_MIN_CONFIDENCE:
        return 0.0

    # A positive bias means "sit further right of corridor centre", i.e. act
    # as though the corridor centre were bias_px further right than measured.
    offset = observation.offset_px + bias_px
    if abs(offset) <= LANE_OFFSET_DEADBAND_PX:
        return 0.0

    # + offset = corridor centre is right of image centre -> the bot has
    # drifted left of centre -> nudge right (+) to come back. Same
    # +steer=right convention as everywhere else (config/robot_config.py).
    raw_nudge = LANE_STEER_KP * offset * observation.confidence
    return max(-LANE_MAX_NUDGE_DEG, min(LANE_MAX_NUDGE_DEG, raw_nudge))
