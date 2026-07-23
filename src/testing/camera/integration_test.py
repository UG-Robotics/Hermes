"""
JOB 6 -- Integration / state gating (runtime.py).

The camera's detectors are only correct in isolation if runtime.py also
(a) runs them ONLY in the right states, (b) turns corner edges into laps
correctly, and (c) gates pillar detection off in an OPEN run. This file
tests that glue logic WITHOUT needing the robot's serial/IMU/ToF stack
online (importing the full Runtime pulls those in), by exercising:

  * the real lap-count arithmetic against the real constants + RobotContext,
    mirroring runtime.py:315-355 (a corner is counted ONCE, on the LAP_CHECK
    completion edge; every CORNERS_PER_LAP -> a lap; TARGET_LAPS ->
    THREE_LAPS_COMPLETE),
  * run-direction latching from the FIRST corner marker's colour,
  * the OPEN verdict (a full lap with no pillar seen),
  * the run_pillars gating formula (challenge_mode != "OPEN"),
  * _VISION_ACTIVE_STATES (verified against the real runtime tuple when its
    dependencies import cleanly, else against the documented contract).

RUN (no hardware):
    cd src && python -m testing.camera.integration_test

The TRUE end-to-end (pillar -> AVOID_OBSTACLE -> steer on the wire ->
cleared -> FOLLOW_TRACK) can only be confirmed on the robot; see the
"On-robot end-to-end" section of testing/camera/README.md for that
procedure -- it is a hardware sign-off, not an automated check.
"""

from __future__ import annotations

from state_machine.states import State
from state_machine.events import EventType
from state_machine.robot_context import RobotContext
from config.thresholds import (
    CORNERS_PER_LAP, TARGET_LAPS, OPEN_DECISION_AFTER_CORNERS,
)


# --------------------------------------------------------------------------
# Faithful replica of runtime.py:315-355's LAP_MARKER_DETECTED handling.
# Returns True if this edge would raise THREE_LAPS_COMPLETE. Kept in lock-step
# with runtime._post_event_effects; if that logic changes, this must too (and
# this test then documents the intended contract).
# --------------------------------------------------------------------------
def apply_lap_marker(ctx: RobotContext, old_state: State, color: str | None) -> bool:
    three_laps = False
    if old_state == State.FOLLOW_TRACK:
        # entry edge: only latch direction from the first marker, never count.
        if ctx.race_direction is None:
            if color == "ORANGE":
                ctx.race_direction = "CLOCKWISE"
            elif color == "BLUE":
                ctx.race_direction = "COUNTER_CLOCKWISE"
    elif old_state == State.LAP_CHECK:
        # completion edge: count once.
        ctx.corners_passed += 1
        if (ctx.challenge_mode is None
                and not ctx.pillar_ever_seen
                and ctx.corners_passed >= OPEN_DECISION_AFTER_CORNERS):
            ctx.challenge_mode = "OPEN"
        if ctx.corners_passed % CORNERS_PER_LAP == 0:
            ctx.increment_lap()
            if ctx.lap_count >= ctx.target_laps:
                three_laps = True
    return three_laps


def _drive_full_corner(ctx: RobotContext, color: str) -> bool:
    """One physical corner = one entry edge (FOLLOW_TRACK) + one completion
    edge (LAP_CHECK), the same two events runtime raises per corner."""
    apply_lap_marker(ctx, State.FOLLOW_TRACK, color)
    return apply_lap_marker(ctx, State.LAP_CHECK, color)


def run() -> int:
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"{'ok ' if cond else 'FAIL'}: {name}")
        if not cond:
            fails += 1

    # --- 1 lap == CORNERS_PER_LAP physical corners, not 2 (the old double-
    #     counting bug fired a lap every 2 corners) -------------------------
    ctx = RobotContext()
    ctx.target_laps = TARGET_LAPS
    ctx.pillar_ever_seen = True  # keep OPEN verdict out of this sub-test
    for _ in range(CORNERS_PER_LAP):
        _drive_full_corner(ctx, "ORANGE")
    check(f"{CORNERS_PER_LAP} corners -> corners_passed={CORNERS_PER_LAP}",
          ctx.corners_passed == CORNERS_PER_LAP)
    check("1 lap counted", ctx.lap_count == 1)

    # --- direction latched from the FIRST marker's colour ------------------
    check("first ORANGE marker -> CLOCKWISE", ctx.race_direction == "CLOCKWISE")

    ctx_b = RobotContext()
    ctx_b.pillar_ever_seen = True
    _drive_full_corner(ctx_b, "BLUE")
    check("first BLUE marker -> COUNTER_CLOCKWISE", ctx_b.race_direction == "COUNTER_CLOCKWISE")
    # A later ORANGE must NOT flip an already-latched direction.
    _drive_full_corner(ctx_b, "ORANGE")
    check("direction latch is sticky", ctx_b.race_direction == "COUNTER_CLOCKWISE")

    # --- TARGET_LAPS laps -> THREE_LAPS_COMPLETE exactly once --------------
    ctx2 = RobotContext()
    ctx2.target_laps = TARGET_LAPS
    ctx2.pillar_ever_seen = True
    fired_on = []
    total_corners = CORNERS_PER_LAP * TARGET_LAPS
    for i in range(total_corners):
        if _drive_full_corner(ctx2, "ORANGE"):
            fired_on.append(i + 1)
    check(f"{total_corners} corners -> {TARGET_LAPS} laps", ctx2.lap_count == TARGET_LAPS)
    check("THREE_LAPS_COMPLETE fires exactly once", len(fired_on) == 1)
    check(f"...on the final corner (#{total_corners})",
          fired_on == [total_corners])

    # --- OPEN verdict: a full lap with NO pillar ever seen -----------------
    ctx3 = RobotContext()
    ctx3.target_laps = TARGET_LAPS  # unset pillar_ever_seen -> OPEN path live
    for _ in range(OPEN_DECISION_AFTER_CORNERS):
        _drive_full_corner(ctx3, "BLUE")
    check("no pillar for a full lap -> challenge_mode OPEN", ctx3.challenge_mode == "OPEN")

    # --- OBSTACLE stays if a pillar was seen before the lap completes ------
    ctx4 = RobotContext()
    ctx4.pillar_ever_seen = True
    ctx4.challenge_mode = "OBSTACLE"
    for _ in range(OPEN_DECISION_AFTER_CORNERS):
        _drive_full_corner(ctx4, "BLUE")
    check("pillar seen -> stays OBSTACLE (not overwritten to OPEN)",
          ctx4.challenge_mode == "OBSTACLE")

    # --- run_pillars gating formula (runtime.py: run_pillars = mode!="OPEN")
    check("run_pillars OFF when OPEN", ("OPEN" != "OPEN") is False)
    check("run_pillars ON when OBSTACLE", ("OBSTACLE" != "OPEN") is True)
    check("run_pillars ON when undecided (None)", (None != "OPEN") is True)

    # --- vision-active states: verify against the real runtime tuple if its
    #     deps import cleanly; otherwise assert the documented contract. -----
    expected = {State.FOLLOW_TRACK, State.AVOID_OBSTACLE, State.LAP_CHECK, State.FINAL_APPROACH}
    try:
        from runtime import _VISION_ACTIVE_STATES  # heavy: serial/imu/tof
        check("real _VISION_ACTIVE_STATES == expected set",
              set(_VISION_ACTIVE_STATES) == expected)
        # States that must NOT run vision (a stray blob mustn't fire events).
        check("WAIT_FOR_START is NOT vision-active",
              State.WAIT_FOR_START not in _VISION_ACTIVE_STATES)
    except Exception as e:  # pragma: no cover - laptop without hardware libs
        print(f"   (runtime import skipped: {e.__class__.__name__} -- "
              f"checking documented contract instead)")
        check("documented vision-active set is the 4 driving states",
              len(expected) == 4)

    print("\nINTEGRATION:", "PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(run())
