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

import time

from config.thresholds import (
    TOF_WALL_WARNING_MM, TOF_WALL_CRITICAL_MM, TOF_MAX_VALID_MM,
    TOF_MIN_PLAUSIBLE_SUM_MM,
    AVOIDANCE_STEER_CAP_NEAR_WALL_DEG, AVOIDANCE_WALL_NUDGE_DEG,
    SPEED_SCALE_WALL_WARNING, SPEED_SCALE_WALL_CRITICAL, SPEED_SCALE_MIN,
)
from config.camera_config import PILLAR_MAX_STEER_DEG
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


# ------------------------------------------------------ ToF sanity cross-check
_last_spurious_log = 0.0


def _spurious_close(near_mm: float, opposite_mm: Optional[float]) -> bool:
    """True when a critically-close `near_mm` reading is physically contradicted
    by the opposite side and is therefore a lying/stuck sensor, not a real wall.

    The ONLY case flagged (so a real wall is never suppressed): both sides are
    in range yet sum to less than TOF_MIN_PLAUSIBLE_SUM_MM -- two opposite walls
    that close together can't bound any legal corridor, so at least one reading
    is bogus. When the opposite side is out of range (a corner/opening) there's
    nothing to cross-check against, so we do NOT flag it -- a genuine close wall
    at a corner is taken at face value.

    NOTE: this can only catch *geometrically impossible* readings (a dead sensor
    reading ~0mm, crosstalk pulling both sides low, a swapped/garbage value). A
    single sensor stuck close whose partner is consistent with actually hugging
    that wall (e.g. right=79mm, left=900mm) is indistinguishable from reality
    using the two ToFs alone -- that needs a sensor recalibration / mount fix.
    """
    if opposite_mm is None or opposite_mm >= TOF_MAX_VALID_MM:
        return False
    return (near_mm + opposite_mm) < TOF_MIN_PLAUSIBLE_SUM_MM


def _sanitize_walls(tof_left_mm: Optional[float], tof_right_mm: Optional[float]):
    """Cross-check the two side ToFs and neutralise a critically-close reading
    the opposite side proves impossible (see _spurious_close), returning
    (left, right) with any such reading replaced by TOF_MAX_VALID_MM ('no wall
    in range') so it can't trigger braking or steer-capping downstream. Real
    close walls (opposite side far, or out of range at a corner) pass through
    untouched. Log is throttled to ~1 Hz so a persistently-stuck sensor doesn't
    flood the tick-rate loop."""
    left, right = tof_left_mm, tof_right_mm
    left_bad = (left is not None and left < TOF_WALL_CRITICAL_MM
                and _spurious_close(left, right))
    right_bad = (right is not None and right < TOF_WALL_CRITICAL_MM
                 and _spurious_close(right, left))

    if left_bad or right_bad:
        global _last_spurious_log
        now = time.time()
        if now - _last_spurious_log >= 1.0:
            _last_spurious_log = now
            logger.warning(
                f"[OBSTACLE] Implausible ToF pair left={left}mm right={right}mm "
                f"(sum < {TOF_MIN_PLAUSIBLE_SUM_MM:.0f}mm -- opposite walls can't "
                f"both be this close). Ignoring the critical reading, not braking/capping."
            )
        if left_bad:
            left = TOF_MAX_VALID_MM
        if right_bad:
            right = TOF_MAX_VALID_MM
    return left, right


def adjust_avoidance_steer(steer_angle: int, direction: str,
                            tof_left_mm: Optional[float], tof_right_mm: Optional[float]) -> AvoidancePlan:
    """Cap/soften a pillar-avoidance steer angle using the side ToF reading
    for the wall we're steering toward, and derive a speed scale from
    whichever side wall is currently closest (both walls matter for speed;
    only the avoidance-direction wall matters for capping steer).
    """
    # Cross-check the two sensors first so a lying/stuck 'critically close'
    # reading can't trigger a hard steer-cap + away-nudge on a phantom wall.
    tof_left_mm, tof_right_mm = _sanitize_walls(tof_left_mm, tof_right_mm)

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

    # Clamp to the pillar-avoidance ceiling (PILLAR_MAX_STEER_DEG), NOT the
    # heading-hold PID's ±STEER_MAX correction authority. This value is a
    # turn-by *intent* (runtime.py converts it into a target heading via
    # SteeringController.turn_by); the PID then drives the servo to hold that
    # heading, and its own output is separately clamped to ±STEER_MAX inside
    # control/steering_control.py. Clamping the intent to ±45 here would have
    # silently capped every avoidance swing below the 60° the pillar planner
    # is allowed to ask for.
    steer = int(max(-PILLAR_MAX_STEER_DEG, min(PILLAR_MAX_STEER_DEG, capped)))
    return AvoidancePlan(steer_angle=steer, speed_scale=speed_scale,
                          wall_nudge_deg=wall_nudge, wall_side=warned_side)


def general_speed_scale(tof_left_mm: Optional[float], tof_right_mm: Optional[float]) -> float:
    """Speed scale for driving outside active pillar avoidance (e.g.
    FOLLOW_TRACK cornering) -- wall proximity only, no steer capping."""
    tof_left_mm, tof_right_mm = _sanitize_walls(tof_left_mm, tof_right_mm)
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
