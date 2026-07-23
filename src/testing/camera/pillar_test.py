"""
JOB 2 -- Pillar detection + avoidance (perception/pillar_detection.py).

Covers: presence, area threshold, RED/GREEN classification, pass-side
decision, steer sign/magnitude, distance estimate, new_detection fires once,
cleared fires (both paths), and lost-frame hysteresis.

LIVE (camera pointed at a real red/green pillar):
    cd src && python -m testing.camera.pillar_test

    Telemetry per tick, e.g.:
      RED    cx=178 area=  920 -> pass RIGHT steer=+27 ~430mm  new=1 clr=0
      GREEN  cx=140 area= 1500 -> pass LEFT  steer=-31 ~350mm  new=1 clr=0
      --     (no pillar)
    Slide the pillar LEFT->RIGHT across the frame: |steer| must stay >= 18
    and the sign must match (RED positive, GREEN negative). Walk the pillar
    toward the camera: ~mm must shrink. Annotated frames with the detected
    blob boxed land in _debug_out/job2_pillar.png.

SELF-CHECK (no hardware):
    cd src && python -m testing.camera.pillar_test --selfcheck
"""

from __future__ import annotations

import sys

from testing.camera import _bench as B
from config.camera_config import (
    FRAME_WIDTH, FRAME_HEIGHT, MIN_PILLAR_AREA,
    PILLAR_MIN_STEER_DEG, PILLAR_MAX_STEER_DEG, PILLAR_LOST_FRAMES,
)
from perception.pillar_detection import (
    PillarDetector, find_largest_pillar, decide_direction,
    compute_steer_angle, _estimate_distance_mm, PillarBlob,
)


def _annotate(frame_rgb, blob):
    import cv2
    out = frame_rgb.copy()
    if blob is not None:
        col = (255, 0, 0) if blob.color == "RED" else (0, 255, 0)
        cv2.circle(out, (blob.cx, blob.cy), 8, col, 2)
        cv2.line(out, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT),
                 (255, 255, 0), 1)
    return out


def run_live():
    B.require_cv2()
    det = PillarDetector()

    def step(frame, idx):
        if frame is None:
            return "-- (no frame)", None
        obs = det.update(frame, FRAME_WIDTH)
        blob = find_largest_pillar(frame)
        if obs.present and obs.color:
            line = (f"{obs.color:<6} cx={obs.cx} area={(obs.area or 0):6.0f} "
                    f"-> pass {obs.direction or '?':<5} steer={obs.steer_angle:+d} "
                    f"~{(obs.distance_mm or 0):.0f}mm  new={int(obs.new_detection)} "
                    f"clr={int(obs.cleared)}")
        elif obs.cleared:
            line = f"CLEARED ({obs.color})"
        else:
            line = "--     (no pillar)"
        return line, _annotate(frame, blob)

    B.live_loop(step, hz=10.0, save_name="job2_pillar.png")


def run_selfcheck() -> int:
    B.require_cv2()
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"{'ok ' if cond else 'FAIL'}: {name}")
        if not cond:
            fails += 1

    # presence + None
    empty = B.blank_frame()
    check("empty frame -> no pillar", find_largest_pillar(empty) is None)
    f_red = B.draw_box(B.blank_frame(), "RED", FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
    red_blob = find_largest_pillar(f_red)
    check("red pillar present", red_blob is not None and red_blob.color == "RED")
    f_grn = B.draw_box(B.blank_frame(), "GREEN", FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
    grn_blob = find_largest_pillar(f_grn)
    check("green pillar present", grn_blob is not None and grn_blob.color == "GREEN")

    # area threshold: a box below MIN_PILLAR_AREA must be ignored. A blob of
    # half-width h has area ~ (2h)^2; pick h so (2h)^2 < MIN_PILLAR_AREA.
    import math
    tiny_h = max(1, int(math.sqrt(MIN_PILLAR_AREA) / 2) - 2)
    f_tiny = B.draw_box(B.blank_frame(), "RED", FRAME_WIDTH // 2, FRAME_HEIGHT // 2,
                        half_w=tiny_h, half_h=tiny_h)
    check(f"sub-threshold blob ignored (half={tiny_h}px)",
          find_largest_pillar(f_tiny) is None)

    # pass-side
    check("RED -> RIGHT", decide_direction("RED") == "RIGHT")
    check("GREEN -> LEFT", decide_direction("GREEN") == "LEFT")

    # steer sign + floor, swept across the frame
    for cx in (30, FRAME_WIDTH // 2, FRAME_WIDTH - 30):
        rb = PillarBlob("RED", cx, 120, 1000, 40)
        gb = PillarBlob("GREEN", cx, 120, 1000, 40)
        rs, gs = compute_steer_angle(FRAME_WIDTH, rb), compute_steer_angle(FRAME_WIDTH, gb)
        check(f"RED steer>0 @cx={cx} ({rs:+d})", rs > 0)
        check(f"GREEN steer<0 @cx={cx} ({gs:+d})", gs < 0)
        check(f"|steer|>=MIN @cx={cx}", abs(rs) >= PILLAR_MIN_STEER_DEG and abs(gs) >= PILLAR_MIN_STEER_DEG)
        check(f"|steer|<=MAX @cx={cx}", abs(rs) <= PILLAR_MAX_STEER_DEG and abs(gs) <= PILLAR_MAX_STEER_DEG)

    # distance monotonic: wider blob (closer) -> smaller mm
    check("distance shrinks as blob widens",
          _estimate_distance_mm(80) < _estimate_distance_mm(40))

    # new_detection fires once
    det = PillarDetector()
    o1 = det.update(f_red, FRAME_WIDTH)
    o2 = det.update(f_red, FRAME_WIDTH)
    check("new_detection true on 1st frame", o1.new_detection)
    check("new_detection false on 2nd frame", (not o2.new_detection) and o2.present)

    # cleared via lost frames + single-frame drop does NOT clear (hysteresis)
    det2 = PillarDetector()
    det2.update(f_grn, FRAME_WIDTH)
    one_drop = det2.update(empty, FRAME_WIDTH)  # 1 lost frame
    check("single dropped frame does NOT clear", not one_drop.cleared)
    cleared = False
    for _ in range(PILLAR_LOST_FRAMES + 2):
        if det2.update(empty, FRAME_WIDTH).cleared:
            cleared = True
            break
    check(f"cleared after {PILLAR_LOST_FRAMES} lost frames", cleared)

    print("\nSELF-CHECK:", "PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--selfcheck" in sys.argv:
        raise SystemExit(run_selfcheck())
    run_live()
