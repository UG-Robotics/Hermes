"""
JOB 4 -- Parking-zone detection (perception/finish_detection.py).

Covers: magenta detection (HSV_MAGENTA + MIN_PARKING_MARKER_AREA), and the
confirm-then-latch behaviour (PARKING_CONFIRM_FRAMES consecutive sightings
before confirmed=True, then never fires again this run).

LIVE (camera pointed at the magenta parking-lot line):
    cd src && python -m testing.camera.parking_test

    Telemetry, e.g.:
      seen cx=160 area= 640  consec-> (building)
      seen cx=161 area= 655  consec->
      CONFIRMED cx=160 area=655           <- fires ONCE, on the 5th consecutive
      -- (latched; nothing more this run)
    Cover/uncover the marker: a brief flash (< 5 frames) must NOT confirm.
    Annotated frames -> _debug_out/job4_parking.png.

SELF-CHECK (no hardware):
    cd src && python -m testing.camera.parking_test --selfcheck
"""

from __future__ import annotations

import math
import sys

from testing.camera import _bench as B
from config.camera_config import (
    FRAME_WIDTH, FRAME_HEIGHT, MIN_PARKING_MARKER_AREA, PARKING_CONFIRM_FRAMES,
)
from perception.finish_detection import ParkingZoneTracker, find_parking_marker


def run_live():
    B.require_cv2()
    import cv2
    tracker = ParkingZoneTracker()

    def step(frame, idx):
        if frame is None:
            return "-- (no frame)", None
        obs = tracker.update(frame)
        found = find_parking_marker(frame)
        if obs.confirmed:
            line = f"CONFIRMED cx={obs.cx} area={(obs.area or 0):.0f}  (latched)"
        elif obs.present:
            line = f"seen cx={obs.cx} area={(obs.area or 0):.0f}  consec->"
        else:
            line = "--"
        out = frame.copy()
        if found is not None:
            cv2.circle(out, (found[0], FRAME_HEIGHT // 2), 8, (255, 0, 255), 2)
        return line, out

    B.live_loop(step, hz=10.0, save_name="job4_parking.png")


def run_selfcheck() -> int:
    B.require_cv2()
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"{'ok ' if cond else 'FAIL'}: {name}")
        if not cond:
            fails += 1

    f_mag = B.draw_box(B.blank_frame(), "MAGENTA", FRAME_WIDTH // 2, FRAME_HEIGHT // 2,
                       half_w=30, half_h=15)
    empty = B.blank_frame()

    check("magenta marker found", find_parking_marker(f_mag) is not None)
    check("empty -> no marker", find_parking_marker(empty) is None)

    tiny_h = max(1, int(math.sqrt(MIN_PARKING_MARKER_AREA) / 2) - 2)
    f_tiny = B.draw_box(B.blank_frame(), "MAGENTA", FRAME_WIDTH // 2, FRAME_HEIGHT // 2,
                        half_w=tiny_h, half_h=tiny_h)
    check(f"sub-threshold marker ignored (half={tiny_h}px)",
          find_parking_marker(f_tiny) is None)

    # A flash shorter than PARKING_CONFIRM_FRAMES must NOT confirm.
    t = ParkingZoneTracker()
    flashed = False
    for _ in range(PARKING_CONFIRM_FRAMES - 1):
        if t.update(f_mag).confirmed:
            flashed = True
        t.update(empty)  # break the streak
    check(f"flash < {PARKING_CONFIRM_FRAMES} frames does NOT confirm", not flashed)

    # PARKING_CONFIRM_FRAMES consecutive -> confirm ONCE, then latched.
    t2 = ParkingZoneTracker()
    confirms = 0
    for _ in range(PARKING_CONFIRM_FRAMES + 5):
        if t2.update(f_mag).confirmed:
            confirms += 1
    check(f"confirms exactly once after {PARKING_CONFIRM_FRAMES} consec", confirms == 1)
    check("latched: no re-fire while still visible", not t2.update(f_mag).confirmed)

    print("\nSELF-CHECK:", "PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--selfcheck" in sys.argv:
        raise SystemExit(run_selfcheck())
    run_live()
