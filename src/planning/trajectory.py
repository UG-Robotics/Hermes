"""
Trajectory helpers (shared planning geometry).

Hermes generates its motion targets close to the point of use rather than as
one monolithic path:

    * FOLLOW_TRACK corridor centering -> planning/wall_centering.py turns the
      two side ToF distances into a small per-tick heading nudge,
    * AVOID_OBSTACLE pillar passing  -> planning/obstacle_planner.py caps the
      pillar-avoidance angle against the side ToF walls,
    * PARK parallel parking          -> planning/parking_planner.py runs a
      staged, IMU/time-driven maneuver.

Each of those consumes the IMU heading-hold controller
(control/steering_control.py) as its "trajectory follower", so there is no
separate global path object to store here yet. This module is the home for
any geometry shared across those planners if one is later needed (e.g. an
arc/turn-radius model once a wheel encoder is added for real odometry). It is
intentionally light rather than empty so the planning layer's file list in
src/README.md stays accurate.
"""

from __future__ import annotations


def clamp(value: float, low: float, high: float) -> float:
    """Clamp `value` into [low, high]. Shared by planners that need a plain
    numeric clamp without pulling in numpy."""
    return max(low, min(high, value))
