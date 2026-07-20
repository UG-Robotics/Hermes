"""
Parallel-parking maneuver (Obstacle Challenge finish).

WRO 2026 Future Engineers ends the obstacle round with a parallel park into
the magenta-walled parking lot. This module owns the low-level maneuver:
given the current IMU heading and side ToF readings, it produces the
(speed, steer, action) command for THIS tick and reports when the park is
finished, so runtime.py can raise PARKING_COMPLETE.

It is a staged, reverse-in parallel park (see config/parking_config.py for
the full stage breakdown and every tunable constant). Because this platform
has no wheel encoder/odometry -- only the IMU and two side ToF sensors -- the
stages are driven primarily by TIME, with the integrated IMU heading and the
lot-side ToF used as secondary completion/safety guards. Nothing here talks
to the serial link or the camera: it is pure motion logic over already-known
state, mirroring the perception -> planning -> control -> communication split
the rest of the stack uses (runtime.py is the only caller and the only place
commands are actually sent).

Usage (runtime.py):
    park = ParkingManeuver()
    park.begin(park_side, heading_deg)         # once, on entry to PARK
    speed, steer, action = park.step(dt, heading_deg, tof_left_mm, tof_right_mm)
    if park.done:
        ... raise PARKING_COMPLETE ...
"""

from __future__ import annotations

from enum import Enum, auto
from typing import Tuple

from config.parking_config import (
    PARK_REVERSE_SPEED, PARK_FORWARD_SPEED, PARK_STEER_DEG,
    PARK_SETTLE_S, PARK_REVERSE_IN_S, PARK_REVERSE_STRAIGHTEN_S,
    PARK_FORWARD_SETTLE_S,
    PARK_USE_HEADING_GUARD, PARK_TURN_IN_DEG, PARK_ALIGN_TOLERANCE_DEG,
    PARK_MIN_SIDE_CLEARANCE_MM,
)
from config.robot_config import SPEED_STOP
from utils.logger import get_logger

logger = get_logger(__name__)


def _wrap_deg(angle: float) -> float:
    """Wrap to (-180, 180], same convention as control/steering_control.py."""
    return (angle + 180.0) % 360.0 - 180.0


class ParkStage(Enum):
    SETTLE = auto()
    REVERSE_IN = auto()
    REVERSE_STRAIGHTEN = auto()
    FORWARD_SETTLE = auto()
    DONE = auto()


class ParkingManeuver:
    """Stateful driver for one parallel-park attempt.

    step() returns a (speed, steer, action) command every tick and advances
    an internal stage machine; `done` flips True once the car is parked.
    Re-usable across runs via reset().
    """

    def __init__(self):
        self._stage = ParkStage.SETTLE
        self._elapsed = 0.0                 # seconds in the current stage
        self._park_side = "LEFT"            # "LEFT" | "RIGHT" (lot side)
        self._ref_heading: float | None = None  # heading at park start
        self._started = False

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        self._stage = ParkStage.SETTLE
        self._elapsed = 0.0
        self._ref_heading = None
        self._started = False

    def begin(self, park_side: str, heading_deg: float) -> None:
        """Lock the lot side and the reference (pre-park) heading. Idempotent:
        only the first call per park attempt takes effect, so runtime.py can
        call it every PARK tick without re-arming the maneuver."""
        if self._started:
            return
        self.reset()
        self._park_side = "RIGHT" if str(park_side).upper() == "RIGHT" else "LEFT"
        self._ref_heading = _wrap_deg(heading_deg)
        self._started = True
        logger.info(f"[PARK] begin: lot on {self._park_side}, ref heading {self._ref_heading:+.1f} deg.")

    @property
    def done(self) -> bool:
        return self._stage == ParkStage.DONE

    @property
    def stage_name(self) -> str:
        return self._stage.name

    # --------------------------------------------------------------- signs
    def _reverse_in_steer(self) -> int:
        """Reversing INTO the spot: cut the wheels toward the lot. Steering
        left (-) while reversing swings the tail left (into a LEFT lot);
        mirror for a RIGHT lot."""
        return -PARK_STEER_DEG if self._park_side == "LEFT" else PARK_STEER_DEG

    def _straighten_steer(self) -> int:
        """Opposite lock to bring the heading back parallel to the wall."""
        return PARK_STEER_DEG if self._park_side == "LEFT" else -PARK_STEER_DEG

    def _heading_delta(self, heading_deg: float) -> float:
        if self._ref_heading is None:
            return 0.0
        return abs(_wrap_deg(heading_deg - self._ref_heading))

    def _lot_side_mm(self, tof_left_mm: float, tof_right_mm: float) -> float:
        return tof_left_mm if self._park_side == "LEFT" else tof_right_mm

    # ---------------------------------------------------------------- step
    def step(self, dt: float, heading_deg: float,
             tof_left_mm: float, tof_right_mm: float) -> Tuple[int, int, str]:
        """Advance the maneuver by `dt` seconds and return (speed, steer,
        action) for this tick. Safe to keep calling after `done` -- it just
        commands a full stop."""
        if not self._started:
            # Defensive: begin() should have been called, but never move if
            # it wasn't (a stationary car is always the safe default).
            return SPEED_STOP, 0, "STOP"

        if self._stage == ParkStage.DONE:
            return SPEED_STOP, 0, "STOP"

        self._elapsed += max(0.0, dt)
        lot_mm = self._lot_side_mm(tof_left_mm, tof_right_mm)
        too_close = (PARK_MIN_SIDE_CLEARANCE_MM > 0.0 and lot_mm < PARK_MIN_SIDE_CLEARANCE_MM)

        if self._stage == ParkStage.SETTLE:
            if self._elapsed >= PARK_SETTLE_S:
                self._advance(ParkStage.REVERSE_IN)
            return SPEED_STOP, 0, "STOP"

        if self._stage == ParkStage.REVERSE_IN:
            turned_enough = (PARK_USE_HEADING_GUARD and
                             self._heading_delta(heading_deg) >= PARK_TURN_IN_DEG)
            if self._elapsed >= PARK_REVERSE_IN_S or turned_enough or too_close:
                self._advance(ParkStage.REVERSE_STRAIGHTEN)
                return SPEED_STOP, 0, "STOP"
            return PARK_REVERSE_SPEED, self._reverse_in_steer(), "BACKWARD"

        if self._stage == ParkStage.REVERSE_STRAIGHTEN:
            aligned = (PARK_USE_HEADING_GUARD and
                       self._heading_delta(heading_deg) <= PARK_ALIGN_TOLERANCE_DEG)
            if self._elapsed >= PARK_REVERSE_STRAIGHTEN_S or aligned or too_close:
                self._advance(ParkStage.FORWARD_SETTLE)
                return SPEED_STOP, 0, "STOP"
            return PARK_REVERSE_SPEED, self._straighten_steer(), "BACKWARD"

        if self._stage == ParkStage.FORWARD_SETTLE:
            if self._elapsed >= PARK_FORWARD_SETTLE_S:
                self._advance(ParkStage.DONE)
                logger.info("[PARK] complete.")
                return SPEED_STOP, 0, "STOP"
            return PARK_FORWARD_SPEED, 0, "FORWARD"

        return SPEED_STOP, 0, "STOP"

    def _advance(self, stage: ParkStage) -> None:
        logger.info(f"[PARK] {self._stage.name} -> {stage.name} (after {self._elapsed:.2f}s).")
        self._stage = stage
        self._elapsed = 0.0


def park_side_for_direction(race_direction: str | None) -> str:
    """Map run direction to the side the parking lot is on. Falls back to
    LEFT when the direction is unknown (a benign default -- the maneuver is
    still a valid parallel park, just possibly mirrored; PARK_SIDE_OVERRIDE
    in config/parking_config.py is the escape hatch if the geometry differs)."""
    from config.parking_config import (
        PARK_SIDE_OVERRIDE, PARK_SIDE_FOR_CLOCKWISE, PARK_SIDE_FOR_COUNTER_CLOCKWISE,
    )
    if PARK_SIDE_OVERRIDE in ("LEFT", "RIGHT"):
        return PARK_SIDE_OVERRIDE
    if race_direction == "CLOCKWISE":
        return PARK_SIDE_FOR_CLOCKWISE
    if race_direction == "COUNTER_CLOCKWISE":
        return PARK_SIDE_FOR_COUNTER_CLOCKWISE
    return "LEFT"
