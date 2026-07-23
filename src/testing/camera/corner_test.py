"""
JOB 3 -- Corner-marker detection (perception/corner_detection.py).

The camera's job here is ONLY: detect an orange/blue floor line and report
the new_detection (entered corner) / cleared (exited corner) edges. The
lap-count ARITHMETIC that consumes those edges lives in runtime.py and is
tested separately in testing/camera/integration_test.py.

Covers: orange vs blue classification, the lower area bar
(MIN_CORNER_MARKER_AREA), new/cleared edges, and lost-frame hysteresis.

LIVE (camera pointed at an orange or blue floor line):
    cd src && python -m testing.camera.corner_test

    Telemetry, e.g.:
      ORANGE new=1 clr=0   (marker just entered view)
      ORANGE new=0 clr=0   (still in view)
      --                    (marker gone; after 4 lost frames -> clr=1)
    Sweep the marker in and out of the bottom of the frame: you should get
    exactly ONE new=1 as it enters and ONE clr=1 a few frames after it
    leaves -- not a burst of either. Annotated frames -> _debug_out/job3_corner.png.

SELF-CHECK (no hardware):
    cd src && python -m testing.camera.corner_test --selfcheck
"""

from __future__ import annotations

import math
import sys

from testing.camera import _bench as B
from config.camera_config import (
    FRAME_WIDTH, FRAME_HEIGHT, MIN_CORNER_MARKER_AREA, CORNER_MARKER_LOST_FRAMES,
)
from perception.corner_detection import CornerTracker, find_corner_marker


def run_live():
    B.require_cv2()
    import cv2
    tracker = CornerTracker()

    def step(frame, idx):
        if frame is None:
            return "-- (no frame)", None
        obs = tracker.update(frame, FRAME_WIDTH)
        blob = find_corner_marker(frame)
        if obs.present and obs.color:
            line = f"{obs.color:<6} new={int(obs.new_detection)} clr={int(obs.cleared)}"
        elif obs.cleared:
            line = f"CLEARED ({obs.color})"
        else:
            line = "--"
        out = frame.copy()
        if blob is not None:
            col = (255, 120, 0) if blob.color == "ORANGE" else (0, 0, 255)
            cv2.circle(out, (blob.cx, blob.cy), 8, col, 2)
        return line, out

    B.live_loop(step, hz=10.0, save_name="job3_corner.png")


def run_selfcheck() -> int:
    B.require_cv2()
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"{'ok ' if cond else 'FAIL'}: {name}")
        if not cond:
            fails += 1

    # Markers sit low in the frame (floor lines); draw them there.
    yb = int(FRAME_HEIGHT * 0.8)
    f_orange = B.draw_box(B.blank_frame(), "ORANGE", FRAME_WIDTH // 2, yb, half_w=40, half_h=8)
    f_blue = B.draw_box(B.blank_frame(), "BLUE", FRAME_WIDTH // 2, yb, half_w=40, half_h=8)
    empty = B.blank_frame()

    ob, bb = find_corner_marker(f_orange), find_corner_marker(f_blue)
    check("orange marker classified", ob is not None and ob.color == "ORANGE")
    check("blue marker classified", bb is not None and bb.color == "BLUE")
    check("empty -> no marker", find_corner_marker(empty) is None)

    # Lower area bar: a blob under MIN_CORNER_MARKER_AREA is noise.
    tiny_h = max(1, int(math.sqrt(MIN_CORNER_MARKER_AREA) / 2) - 2)
    f_tiny = B.draw_box(B.blank_frame(), "ORANGE", FRAME_WIDTH // 2, yb,
                        half_w=tiny_h, half_h=tiny_h)
    check(f"sub-threshold marker ignored (half={tiny_h}px)",
          find_corner_marker(f_tiny) is None)

    # Edge behaviour: exactly one new_detection on entry.
    t = CornerTracker()
    e1 = t.update(f_orange, FRAME_WIDTH)
    e2 = t.update(f_orange, FRAME_WIDTH)
    check("new_detection once on entry", e1.new_detection and not e2.new_detection)

    # single dropped frame does NOT clear; CORNER_MARKER_LOST_FRAMES does.
    drop1 = t.update(empty, FRAME_WIDTH)
    check("single dropped frame does NOT clear", not drop1.cleared)
    cleared = False
    for _ in range(CORNER_MARKER_LOST_FRAMES + 2):
        if t.update(empty, FRAME_WIDTH).cleared:
            cleared = True
            break
    check(f"cleared after {CORNER_MARKER_LOST_FRAMES} lost frames", cleared)

    print("\nSELF-CHECK:", "PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--selfcheck" in sys.argv:
        raise SystemExit(run_selfcheck())
    run_live()
