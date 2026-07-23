"""
CALIBRATION -- monocular focal length CAMERA_FOCAL_PX (config/camera_config.py).

Pillar distance is estimated with the pinhole model
(perception/pillar_detection._estimate_distance_mm):

    distance_mm = (PILLAR_REAL_DIAMETER_MM * CAMERA_FOCAL_PX) / width_px

CAMERA_FOCAL_PX=350 is a guess. Calibrate it once, on the real lens, by
turning the equation around at a KNOWN distance:

    CAMERA_FOCAL_PX = width_px * distance_mm / PILLAR_REAL_DIAMETER_MM

HOW TO USE (on the Pi):
    cd src
    # Put ONE pillar squarely in frame, measure lens->pillar with a tape.
    python -m testing.camera.calibrate_focal --distance-mm 1000

    It detects the pillar, averages its measured pixel width over ~40 frames,
    and prints the CAMERA_FOCAL_PX to paste into config/camera_config.py.
    Then re-run  python -m testing.camera.pillar_test  and confirm the
    reported ~mm matches your tape at a couple of distances.

Tips: use the SAME resolution you race at (FRAME_WIDTH/HEIGHT), good even
lighting, and a distance in the range you actually avoid pillars at
(~300-800mm) for best real-world accuracy.
"""

from __future__ import annotations

import sys

from testing.camera import _bench as B
from config.camera_config import (
    FRAME_WIDTH, PILLAR_REAL_DIAMETER_MM, CAMERA_FOCAL_PX, PILLAR_CLEAR_DISTANCE_MM,
)
from perception.pillar_detection import find_largest_pillar, _estimate_distance_mm


def _arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default


def run():
    B.require_cv2()
    import numpy as np

    dist_s = _arg("--distance-mm")
    if dist_s is None:
        raise SystemExit("Required: --distance-mm <measured lens-to-pillar distance>\n"
                         "  e.g. python -m testing.camera.calibrate_focal --distance-mm 1000")
    distance_mm = float(dist_s)
    n = int(_arg("--frames", "40"))

    cam = B.open_camera(simulated=False)
    if cam.source == "synthetic":
        print("!! synthetic source -- calibrate against a REAL pillar/camera.")

    widths = []
    color = None
    last_frame = None
    print(f"Measuring pillar width over {n} frames at {distance_mm:.0f}mm...")
    for _ in range(n):
        frame = cam.get_frame()
        if frame is None:
            continue
        last_frame = frame
        blob = find_largest_pillar(frame)
        if blob is not None:
            widths.append(blob.width_px)
            color = blob.color
    cam.close()

    if len(widths) < n // 2:
        raise SystemExit(f"Only detected a pillar in {len(widths)}/{n} frames -- "
                         "check the pillar is centred, lit, and within the colour "
                         "bounds (run calibrate_hsv.py first if it won't detect).")

    width_px = float(np.median(widths))
    focal = width_px * distance_mm / PILLAR_REAL_DIAMETER_MM

    print(f"\ncolour detected:      {color}")
    print(f"median pixel width:   {width_px:.1f}px  (spread "
          f"{min(widths)}..{max(widths)})")
    print(f"real diameter:        {PILLAR_REAL_DIAMETER_MM:.0f}mm")
    print(f"measured distance:    {distance_mm:.0f}mm")
    print(f"\nSUGGESTED (config/camera_config.py):")
    print(f"  CAMERA_FOCAL_PX = {focal:.0f}      # was {CAMERA_FOCAL_PX:.0f}")

    # Sanity: what the OLD constant would have reported at this real distance.
    old_est = _estimate_distance_mm(int(width_px))
    print(f"\nWith the OLD {CAMERA_FOCAL_PX:.0f}, this {distance_mm:.0f}mm pillar would read "
          f"~{old_est:.0f}mm ({old_est - distance_mm:+.0f}mm error).")
    print(f"PILLAR_CLEAR_DISTANCE_MM is {PILLAR_CLEAR_DISTANCE_MM:.0f}mm -- make sure your "
          "calibrated estimate crosses it about when you're actually alongside the pillar.")

    saved = B.save_rgb(last_frame, "calib_focal.png")
    if saved:
        print(f"preview saved {saved}")


if __name__ == "__main__":
    run()
