"""
Speed scaling.

Turns a 0..1 "how cautious should we be right now" scale -- produced by
planning/obstacle_planner.py from ToF wall clearance -- into an actual
speed command, applied on top of whichever base speed
control/drive_command.py picked for the current state.

Deliberately tiny and generic, same spirit as control/pid.py: no knowledge
of ToF, pillars, or lanes here, just "take a base speed and a 0..1 scale,
return a valid speed command."
"""

from __future__ import annotations

from config.robot_config import SPEED_MIN, SPEED_MAX
from config.thresholds import SPEED_SCALE_MIN


def apply_speed_scale(base_speed: int, scale: float) -> int:
    """Scale base_speed by `scale`, clamped to [SPEED_SCALE_MIN, 1.0] and
    then to the valid [SPEED_MIN, SPEED_MAX] command range."""
    scale = max(SPEED_SCALE_MIN, min(1.0, scale))
    return int(max(SPEED_MIN, min(SPEED_MAX, round(base_speed * scale))))
