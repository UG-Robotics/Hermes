"""
CALIBRATION -- HSV colour bounds (config/camera_config.py).

The default HSV ranges are Pi-Cam-V2-under-generic-light guesses; the config
itself says they WILL need an on-mat pass. This tool samples the real colour
under your real lighting and prints ready-to-paste bounds.

HOW TO USE (on the Pi):
    cd src
    python -m testing.camera.calibrate_hsv --color RED

    1. Fill the CENTRE box (drawn on the saved preview) with ONE sample of the
       target colour -- a red pillar, a green pillar, an orange/blue floor
       line, or the magenta parking line.
    2. Hold it steady; the tool averages ~40 frames.
    3. It prints the measured H/S/V spread and SUGGESTED bounds with margin.
    4. Paste those into config/camera_config.py and re-run the matching
       testing/camera/*_test.py to confirm the colour now classifies.

    --color one of: RED GREEN ORANGE BLUE MAGENTA
    Repeat per colour. RED wraps the hue circle -- see the note it prints.

    (Walls are no longer detected by the camera -- corridor centering moved to
    the side ToFs + IMU, see planning/wall_centering.py -- so there is no
    black-wall colour to calibrate here.)

A preview with the sample box is saved to _debug_out/calib_<color>.png so you
can confirm the box actually sat on the colour.
"""

from __future__ import annotations

import sys

from testing.camera import _bench as B
from config.camera_config import FRAME_WIDTH, FRAME_HEIGHT

VALID = ("RED", "GREEN", "ORANGE", "BLUE", "MAGENTA")


def _arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default


def run():
    B.require_cv2()
    import cv2
    import numpy as np

    color = (_arg("--color", "RED") or "RED").upper()
    if color not in VALID:
        raise SystemExit(f"--color must be one of {VALID}")
    n = int(_arg("--frames", "40"))

    # Sample box: centre of the frame for pillars/parking, lower-centre for
    # the orange/blue floor markers (that's where they actually appear).
    if color in ("ORANGE", "BLUE"):
        cy = int(FRAME_HEIGHT * 0.8)
    else:
        cy = FRAME_HEIGHT // 2
    cx = FRAME_WIDTH // 2
    bw, bh = 24, 16
    x0, x1, y0, y1 = cx - bw, cx + bw, cy - bh, cy + bh

    cam = B.open_camera(simulated=False)
    print(f"Sampling {color} from box x[{x0}:{x1}] y[{y0}:{y1}] over {n} frames...")
    if cam.source == "synthetic":
        print("!! synthetic source -- point a REAL camera at the colour; these "
              "numbers are meaningless otherwise.")

    hs, ss, vs = [], [], []
    preview = None
    for _ in range(n):
        frame = cam.get_frame()
        if frame is None:
            continue
        patch = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(patch, cv2.COLOR_RGB2HSV).reshape(-1, 3)
        hs.append(hsv[:, 0]); ss.append(hsv[:, 1]); vs.append(hsv[:, 2])
        preview = frame.copy()
        cv2.rectangle(preview, (x0, y0), (x1, y1), (255, 255, 0), 1)

    cam.close()
    if not hs:
        raise SystemExit("No frames captured.")

    H = np.concatenate(hs); S = np.concatenate(ss); V = np.concatenate(vs)

    def rng(a):
        # robust spread: 5th..95th percentile ignores a few stray edge pixels.
        return float(np.percentile(a, 5)), float(np.percentile(a, 50)), float(np.percentile(a, 95))

    h5, h50, h95 = rng(H)
    s5, s50, s95 = rng(S)
    v5, v50, v95 = rng(V)
    print(f"\nMeasured (5th / median / 95th pct):")
    print(f"  H: {h5:.0f} / {h50:.0f} / {h95:.0f}")
    print(f"  S: {s5:.0f} / {s50:.0f} / {s95:.0f}")
    print(f"  V: {v5:.0f} / {v50:.0f} / {v95:.0f}")

    hm, sm, vm = 8, 40, 40  # margins added around the measured spread

    def clamp(x, lo, hi):
        return int(max(lo, min(hi, x)))

    low = (clamp(h5 - hm, 0, 179), clamp(s5 - sm, 0, 255), clamp(v5 - vm, 0, 255))
    high = (clamp(h95 + hm, 0, 179), clamp(s95 + sm, 0, 255), clamp(v95 + vm, 0, 255))
    prefix = {"RED": "HSV_RED", "GREEN": "HSV_GREEN", "ORANGE": "HSV_ORANGE",
              "BLUE": "HSV_BLUE", "MAGENTA": "HSV_MAGENTA"}[color]
    print("\nSUGGESTED (config/camera_config.py):")
    if color == "RED" and (h5 < hm or h95 > 179 - hm):
        print("  # RED wraps the 0/180 hue boundary -- keep BOTH ranges. This")
        print("  # tool measured only one side; widen whichever your sample hit:")
        print(f"  {prefix}_LOW1 = (0, {low[1]}, {low[2]})")
        print(f"  {prefix}_HIGH1 = ({clamp(h95 + hm, 0, 10)}, 255, 255)")
        print(f"  {prefix}_LOW2 = ({clamp(h5 - hm, 170, 179)}, {low[1]}, {low[2]})")
        print(f"  {prefix}_HIGH2 = (179, 255, 255)")
    else:
        print(f"  {prefix}_LOW = {low}")
        print(f"  {prefix}_HIGH = {high}")

    saved = B.save_rgb(preview, f"calib_{color}.png")
    if saved:
        print(f"\npreview saved {saved} -- confirm the yellow box sat ON the colour.")


if __name__ == "__main__":
    run()
