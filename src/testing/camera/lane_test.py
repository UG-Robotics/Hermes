"""
JOB 5 -- Lane / wall centering (perception/track_detection.py + the
planning/lane_centering.py nudge it feeds).

Covers: black-wall mask (white mat excluded), both-wall vs one-wall vs
no-wall cases and their confidences, offset sign, the ROI band, and that
compute_heading_nudge turns the offset into a correction in the right
direction.

LIVE (camera on the robot, in a real corridor between black walls):
    cd src && python -m testing.camera.lane_test

    Telemetry, e.g.:
      valid off=+34 conf=1.0  L=118 R=286  nudge=+2.0   (drifted LEFT, correct right)
      valid off=-40 conf=0.5  L=None R=232 nudge=-2.4   (one wall only)
      INVALID (no walls)
    Sign rule to eyeball: physically shift the robot toward the LEFT wall ->
    off should go POSITIVE and nudge POSITIVE (turn right, back to centre).
    Annotated frames (detected wall columns drawn) -> _debug_out/job5_lane.png.

SELF-CHECK (no hardware):
    cd src && python -m testing.camera.lane_test --selfcheck
"""

from __future__ import annotations

import sys

from testing.camera import _bench as B
from config.camera_config import FRAME_WIDTH, FRAME_HEIGHT
from config.thresholds import LANE_MIN_CONFIDENCE
from perception.track_detection import detect_lane
from planning.lane_centering import compute_heading_nudge


def run_live():
    B.require_cv2()
    import cv2

    def step(frame, idx):
        if frame is None:
            return "-- (no frame)", None
        obs = detect_lane(frame, FRAME_WIDTH, FRAME_HEIGHT)
        nudge = compute_heading_nudge(obs, FRAME_WIDTH)
        if not obs.valid:
            return "INVALID (no walls)", frame
        line = (f"valid off={obs.offset_px:+d} conf={obs.confidence:.1f} "
                f"L={obs.left_wall_px} R={obs.right_wall_px} nudge={nudge:+.1f}")
        out = frame.copy()
        if obs.left_wall_px is not None:
            cv2.line(out, (obs.left_wall_px, 0), (obs.left_wall_px, FRAME_HEIGHT), (0, 255, 255), 1)
        if obs.right_wall_px is not None:
            cv2.line(out, (obs.right_wall_px, 0), (obs.right_wall_px, FRAME_HEIGHT), (0, 255, 255), 1)
        cv2.line(out, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), (255, 0, 0), 1)
        return line, out

    B.live_loop(step, hz=10.0, save_name="job5_lane.png")


def _walls(left_band=None, right_band=None):
    """Build a frame with black vertical wall bands spanning the full height
    (so they clear the ROI black-fraction threshold). Bands are (x0,x1)."""
    f = B.blank_frame()
    # White floor so the black-vs-floor mask has something to reject.
    f[:] = (240, 240, 240)
    if left_band:
        B.draw_wall(f, left_band[0], left_band[1], 0, FRAME_HEIGHT)
    if right_band:
        B.draw_wall(f, right_band[0], right_band[1], 0, FRAME_HEIGHT)
    return f


def run_selfcheck() -> int:
    B.require_cv2()
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"{'ok ' if cond else 'FAIL'}: {name}")
        if not cond:
            fails += 1

    # White-only frame: no wall pixels -> invalid (mat excluded from mask).
    white = B.blank_frame()
    white[:] = (240, 240, 240)
    check("all-white mat -> INVALID (mask excludes floor)",
          not detect_lane(white, FRAME_WIDTH, FRAME_HEIGHT).valid)

    # Both walls, corridor centre RIGHT of image centre -> positive offset,
    # confidence 1.0, and a POSITIVE (turn-right) nudge.
    a = _walls(left_band=(120, 150), right_band=(290, 315))
    oa = detect_lane(a, FRAME_WIDTH, FRAME_HEIGHT)
    check("both walls -> confidence 1.0", oa.valid and oa.confidence == 1.0)
    check("corridor-centre-right -> offset > 0", oa.offset_px > 0)
    na = compute_heading_nudge(oa, FRAME_WIDTH)
    check(f"positive offset -> positive nudge ({na:+.1f})", na > 0)

    # Mirror: corridor centre LEFT of image centre -> negative offset + nudge.
    bfr = _walls(left_band=(5, 30), right_band=(170, 200))
    ob = detect_lane(bfr, FRAME_WIDTH, FRAME_HEIGHT)
    check("mirror -> offset < 0", ob.valid and ob.offset_px < 0)
    check("negative offset -> negative nudge", compute_heading_nudge(ob, FRAME_WIDTH) < 0)

    # One wall only -> confidence 0.5 (the assumed-half-width guess); still
    # >= LANE_MIN_CONFIDENCE, so a one-wall detection is trusted enough to steer.
    one = _walls(left_band=(120, 150))
    oo = detect_lane(one, FRAME_WIDTH, FRAME_HEIGHT)
    check("one wall -> confidence 0.5", oo.valid and oo.confidence == 0.5)
    check("0.5 >= LANE_MIN_CONFIDENCE so one-wall still steers",
          0.5 >= LANE_MIN_CONFIDENCE)

    # No walls -> invalid. Note: the "no wall" scene is an all-WHITE floor;
    # an all-black frame is the opposite degenerate case (everything reads as
    # wall), which never occurs on the real white mat.
    no_wall = B.blank_frame()
    no_wall[:] = (240, 240, 240)
    check("no walls (white floor) -> INVALID",
          not detect_lane(no_wall, FRAME_WIDTH, FRAME_HEIGHT).valid)

    print("\nSELF-CHECK:", "PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--selfcheck" in sys.argv:
        raise SystemExit(run_selfcheck())
    run_live()
