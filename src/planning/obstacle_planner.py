"""
Obstacle-avoidance planning: reconciles the camera's pillar-avoidance
decision (perception/pillar_detection.py) with the ESP32's two side ToF
sensors (hardware/tof.py) so an avoidance manoeuvre never trades "clear the
pillar on the correct side" for "drive into the wall we're steering
toward." Also derives a general wall-clearance speed scale usable outside
active avoidance (e.g. FOLLOW_TRACK cornering).

Per the WRO rules, the obstacle challenge's corridor is a fixed 1000mm --
tight enough that swinging the full pillar-avoidance angle (up to
PILLAR_MAX_STEER_DEG, see config/camera_config.py) while a side ToF is
already reading close range is exactly the failure mode this module exists
to prevent: touching/moving a wall (a scored penalty) while chasing a
pillar (also scored, but recoverable).

Ownership: pure decision logic over already-published data (a computed
pillar steer angle/direction and the latest ToF reading) -- no hardware or
serial access, exactly like perception/pillar_detection.py's module
docstring describes for its own step (v). runtime.py is the only caller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from config.thresholds import (
    TOF_WALL_WARNING_MM, TOF_WALL_CRITICAL_MM, TOF_MAX_VALID_MM,
    AVOIDANCE_STEER_CAP_NEAR_WALL_DEG, AVOIDANCE_WALL_NUDGE_DEG,
    SPEED_SCALE_WALL_WARNING, SPEED_SCALE_WALL_CRITICAL, SPEED_SCALE_MIN,
)
from config.robot_config import STEER_MIN, STEER_MAX
from utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class AvoidancePlan:
    steer_angle: int                  # possibly-capped avoidance steer (turn-by intent, degrees)
    speed_scale: float                # multiply the base speed by this, 0..1
    wall_nudge_deg: float = 0.0       # extra away-from-wall heading nudge for THIS tick, or 0
    wall_side: Optional[str] = None   # "LEFT" | "RIGHT" | None -- which wall triggered caution


def _clearance_side(direction: str) -> str:
    """Which side wall we risk clipping while avoiding a pillar passed on
    this side. Passing RIGHT of a red pillar swings the car toward the
    RIGHT wall as it cuts across; LEFT for a green pillar."""
    return "RIGHT" if direction == "RIGHT" else "LEFT"


def adjust_avoidance_steer(steer_angle: int, direction: str,
                            tof_left_mm: Optional[float], tof_right_mm: Optional[float]) -> AvoidancePlan:
    """Cap/soften a pillar-avoidance steer angle using the side ToF reading
    for the wall we're steering toward, and derive a speed scale from
    whichever side wall is currently closest (both walls matter for speed;
    only the avoidance-direction wall matters for capping steer).
    """
    side = _clearance_side(direction)
    side_mm = tof_right_mm if side == "RIGHT" else tof_left_mm

    capped = steer_angle
    wall_nudge = 0.0
    warned_side = None

    if side_mm is not None and side_mm < TOF_MAX_VALID_MM:
        if side_mm < TOF_WALL_CRITICAL_MM:
            # Too close to safely commit to the requested avoidance angle --
            # soften it hard and nudge back toward centre, away from that wall.
            capped = _cap_toward_zero(steer_angle, AVOIDANCE_STEER_CAP_NEAR_WALL_DEG // 2)
            wall_nudge = -AVOIDANCE_WALL_NUDGE_DEG if side == "RIGHT" else AVOIDANCE_WALL_NUDGE_DEG
            warned_side = side
            logger.warning(
                f"[OBSTACLE] {side} wall critical ({side_mm:.0f}mm) -- capping avoidance steer "
                f"{steer_angle:+d} -> {capped:+d}, nudging {wall_nudge:+.1f} deg."
            )
        elif side_mm < TOF_WALL_WARNING_MM:
            capped = _cap_toward_zero(steer_angle, AVOIDANCE_STEER_CAP_NEAR_WALL_DEG)
            warned_side = side

    speed_scale = _speed_scale_for_clearance(tof_left_mm, tof_right_mm)

    steer = int(max(STEER_MIN, min(STEER_MAX, capped)))
    return AvoidancePlan(steer_angle=steer, speed_scale=speed_scale,
                          wall_nudge_deg=wall_nudge, wall_side=warned_side)


def general_speed_scale(tof_left_mm: Optional[float], tof_right_mm: Optional[float]) -> float:
    """Speed scale for driving outside active pillar avoidance (e.g.
    FOLLOW_TRACK cornering) -- wall proximity only, no steer capping."""
    return _speed_scale_for_clearance(tof_left_mm, tof_right_mm)


def _speed_scale_for_clearance(tof_left_mm: Optional[float], tof_right_mm: Optional[float]) -> float:
    readings = [d for d in (tof_left_mm, tof_right_mm) if d is not None and d < TOF_MAX_VALID_MM]
    if not readings:
        return 1.0

    closest = min(readings)
    if closest < TOF_WALL_CRITICAL_MM:
        scale = SPEED_SCALE_WALL_CRITICAL
    elif closest < TOF_WALL_WARNING_MM:
        # Linear taper across the warning..critical band, rather than a
        # single step, so the bot doesn't visibly lurch as it crosses the
        # threshold mid-corner.
        span = TOF_WALL_WARNING_MM - TOF_WALL_CRITICAL_MM
        t = (closest - TOF_WALL_CRITICAL_MM) / span if span > 0 else 0.0
        scale = SPEED_SCALE_WALL_CRITICAL + t * (SPEED_SCALE_WALL_WARNING - SPEED_SCALE_WALL_CRITICAL)
    else:
        scale = 1.0

    return max(SPEED_SCALE_MIN, min(1.0, scale))


def _cap_toward_zero(value: int, cap: int) -> int:
    """Clamp `value` to [-cap, cap] without flipping its sign (a positive
    steer intent capped near zero should stay >= 0, never overshoot past
    zero into a turn the other way)."""
    cap = abs(cap)
    if value > 0:
        return min(value, cap)
    if value < 0:
        return max(value, -cap)
    return value
