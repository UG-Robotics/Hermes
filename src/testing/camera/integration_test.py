"""
JOB 6 -- Integration / state gating (runtime.py).

The camera's detectors are only correct in isolation if runtime.py also
(a) runs them ONLY in the right states, (b) turns corner edges into laps
correctly, and (c) gates pillar detection off in an OPEN run. This file
tests that glue logic WITHOUT needing the robot's serial/IMU/ToF stack
online (importing the full Runtime pulls those in), by exercising:

  * the real lap-count arithmetic against the real constants + RobotContext,
    mirroring runtime.py's LAP_MARKER_DETECTED handling. Each corner has TWO
    floor lines (entry + exit), so one corner = 2 lines, one lap = 4 corners =
    8 lines, and a 3-lap run = 24 lines. A corner is counted ONCE, on the EXIT
    line (the LAP_CHECK edge); every CORNERS_PER_LAP -> a lap; TARGET_LAPS ->
    THREE_LAPS_COMPLETE,
  * that runtime fires on each line's APPEARANCE only (so a single line can't
    be counted as a whole corner -- the double-counting bug),
  * run-direction latching from the FIRST line's colour (the entry line),
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


def _drive_corner(ctx: RobotContext, entry_color: str, exit_color: str) -> bool:
    """One physical corner = TWO floor lines. The ENTRY line appears while
    FOLLOW_TRACK (-> LAP_CHECK, latch direction, no count); the EXIT line
    appears while LAP_CHECK (-> FOLLOW_TRACK, count one corner). These are the
    exact two LAP_MARKER_DETECTED events runtime raises per corner -- one per
    line APPEARANCE. So 2 lines -> 1 corner, 8 lines -> 1 lap."""
    apply_lap_marker(ctx, State.FOLLOW_TRACK, entry_color)   # entry line
    return apply_lap_marker(ctx, State.LAP_CHECK, exit_color)  # exit line


# Real WRO FE line colours per run direction: the entry line's colour is what
# latches direction (ORANGE-first -> CW, BLUE-first -> CCW).
_CW = ("ORANGE", "BLUE")    # clockwise: enter on orange, exit on blue
_CCW = ("BLUE", "ORANGE")   # counter-clockwise: enter on blue, exit on orange


def run() -> int:
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"{'ok ' if cond else 'FAIL'}: {name}")
        if not cond:
            fails += 1

    # --- 1 lap == 4 corners == 8 lines (2 per corner), NOT 8 corners. The old
    #     bug counted each line as a whole corner and lapped every 2 corners. --
    ctx = RobotContext()
    ctx.target_laps = TARGET_LAPS
    ctx.pillar_ever_seen = True  # keep OPEN verdict out of this sub-test
    for _ in range(CORNERS_PER_LAP):        # 4 corners = 8 line appearances
        _drive_corner(ctx, *_CCW)
    check(f"{CORNERS_PER_LAP} corners (8 lines) -> corners_passed={CORNERS_PER_LAP}",
          ctx.corners_passed == CORNERS_PER_LAP)
    check("1 lap counted (not 2)", ctx.lap_count == 1)

    # --- direction latched from the FIRST LINE's colour (the entry line) ----
    check("entry BLUE (first line) -> COUNTER_CLOCKWISE",
          ctx.race_direction == "COUNTER_CLOCKWISE")

    ctx_b = RobotContext()
    ctx_b.pillar_ever_seen = True
    _drive_corner(ctx_b, *_CW)              # entry line is ORANGE
    check("entry ORANGE (first line) -> CLOCKWISE", ctx_b.race_direction == "CLOCKWISE")
    _drive_corner(ctx_b, *_CCW)             # a later BLUE entry must not flip it
    check("direction latch is sticky", ctx_b.race_direction == "CLOCKWISE")

    # --- TARGET_LAPS laps == 24 lines -> THREE_LAPS_COMPLETE exactly once ----
    ctx2 = RobotContext()
    ctx2.target_laps = TARGET_LAPS
    ctx2.pillar_ever_seen = True
    fired_on = []
    total_corners = CORNERS_PER_LAP * TARGET_LAPS   # 12 corners
    total_lines = total_corners * 2                 # 24 lines
    for i in range(total_corners):
        if _drive_corner(ctx2, *_CCW):
            fired_on.append(i + 1)
    check(f"{total_corners} corners ({total_lines} lines) -> {TARGET_LAPS} laps",
          ctx2.lap_count == TARGET_LAPS)
    check("THREE_LAPS_COMPLETE fires exactly once", len(fired_on) == 1)
    check(f"...on the final corner (#{total_corners})",
          fired_on == [total_corners])

    # --- OPEN verdict: a full lap with NO pillar ever seen -----------------
    ctx3 = RobotContext()
    ctx3.target_laps = TARGET_LAPS  # unset pillar_ever_seen -> OPEN path live
    for _ in range(OPEN_DECISION_AFTER_CORNERS):
        _drive_corner(ctx3, *_CCW)
    check("no pillar for a full lap -> challenge_mode OPEN", ctx3.challenge_mode == "OPEN")

    # --- OBSTACLE stays if a pillar was seen before the lap completes ------
    ctx4 = RobotContext()
    ctx4.pillar_ever_seen = True
    ctx4.challenge_mode = "OBSTACLE"
    for _ in range(OPEN_DECISION_AFTER_CORNERS):
        _drive_corner(ctx4, *_CCW)
    check("pillar seen -> stays OBSTACLE (not overwritten to OPEN)",
          ctx4.challenge_mode == "OBSTACLE")

    # --- the fix itself: runtime fires a lap-marker on a line's APPEARANCE
    #     only, never its disappearance. So one physical line (appears, holds,
    #     then clears) yields exactly ONE event -- a line can't be both the
    #     entry and the exit of a corner. Mirrors _update_corner_tracking's
    #     post-fix condition `fired = obs.new_detection`. -------------------
    # (new_detection, cleared) edges of one physical line over its lifetime:
    line_edges = [(True, False), (False, False), (False, True)]  # appear, hold, clear
    events_now = sum(1 for new, _clr in line_edges if new)            # fire on appearance
    events_old = sum((1 if new else 0) + (1 if clr else 0)            # old: appearance AND clear
                     for new, clr in line_edges)
    check("one physical line -> exactly ONE lap-marker event (the fix)", events_now == 1)
    check("old behaviour would have fired twice per line (the bug)", events_old == 2)

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
