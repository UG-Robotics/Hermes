"""
JOB 1 -- Frame acquisition (perception/camera.py).

Verifies the pixels themselves before any detector is trusted:
  * which backend opened (picamera2 / opencv / synthetic),
  * frame shape is HEIGHT x WIDTH x 3,
  * measured FPS,
  * RGB channel order (the _swap_rb bug class -- if R and B are swapped
    here, EVERY colour detector downstream is wrong and the dashboard looks
    pink/magenta),
  * get_jpeg() returns real JPEG bytes,
  * synthetic fallback works with no camera.

LIVE (on the Pi, camera pointed at a KNOWN pure-RED object filling the
centre):
    cd src && python -m testing.camera.frame_source_test

    Expect:
      source: picamera2         <- NOT 'synthetic', NOT 'opencv' on the Pi
      shape:  (240, 320, 3)
      fps:    ~12-15
      centre patch  R=~200  G=low  B=low       <- R dominant == channels OK
      JPEG: 1234 bytes, magic ffd8 ffd9 OK
    A frame is saved to _debug_out/job1_frame.png -- open it: red must look
    red, not blue, and there must be no overall pink cast.

SELF-CHECK (no hardware):
    cd src && python -m testing.camera.frame_source_test --selfcheck
"""

from __future__ import annotations

import sys

from testing.camera import _bench as B
from config.camera_config import FRAME_WIDTH, FRAME_HEIGHT


def _analyse(frame_rgb):
    """Return (shape, centre R/G/B means) for an RGB frame."""
    h, w, _ = frame_rgb.shape
    cx0, cx1 = w // 2 - 20, w // 2 + 20
    cy0, cy1 = h // 2 - 20, h // 2 + 20
    patch = frame_rgb[cy0:cy1, cx0:cx1]
    return (frame_rgb.shape,
            float(patch[..., 0].mean()),
            float(patch[..., 1].mean()),
            float(patch[..., 2].mean()))


def run_live():
    B.require_cv2()
    import time
    cam = B.open_camera(simulated=False)
    print(f"source: {cam.source}")
    if cam.source == "synthetic":
        print("!! No real camera -- on the Pi this means picamera2/opencv "
              "failed to open. Check `libcamera-hello` / the ribbon cable.")

    # ---- shape + FPS over 30 frames -------------------------------------
    n = 30
    t0 = time.time()
    frame = None
    for _ in range(n):
        frame = cam.get_frame()
    dt = time.time() - t0
    fps = n / dt if dt > 0 else float("nan")

    if frame is None:
        print("!! get_frame() returned None -- no pixels at all. Stop here.")
        cam.close()
        return
    shape, rmean, gmean, bmean = _analyse(frame)
    print(f"shape:  {shape}   (expected ({FRAME_HEIGHT}, {FRAME_WIDTH}, 3))")
    print(f"fps:    {fps:.1f}")

    # ---- channel order --------------------------------------------------
    print(f"centre patch  R={rmean:.0f}  G={gmean:.0f}  B={bmean:.0f}")
    print("  -> point at a PURE RED object: R should dominate. If B "
          "dominates instead, red/blue are swapped (see BUGS in README).")

    # ---- JPEG stream ----------------------------------------------------
    jpeg = cam.get_jpeg()
    ok = jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"
    print(f"JPEG: {len(jpeg)} bytes, magic {'OK' if ok else 'BAD'}")

    saved = B.save_rgb(frame, "job1_frame.png")
    if saved:
        print(f"saved {saved}  -- open it and sanity-check the colours by eye.")
    cam.close()


def run_selfcheck() -> int:
    """No-hardware checks: synthetic backend produces a well-formed frame and
    a valid JPEG, and the RGB channel contract holds for a drawn red box."""
    B.require_cv2()
    fails = 0
    cam = B.open_camera(simulated=True)  # force synthetic

    frame = cam.get_frame()
    if frame is None:
        print("FAIL: synthetic get_frame() returned None (PIL/numpy missing?)")
        return 1
    if frame.shape != (FRAME_HEIGHT, FRAME_WIDTH, 3):
        print(f"FAIL: shape {frame.shape} != ({FRAME_HEIGHT},{FRAME_WIDTH},3)")
        fails += 1
    else:
        print(f"ok: synthetic frame shape {frame.shape}")

    jpeg = cam.get_jpeg()
    if jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9":
        print(f"ok: get_jpeg() valid JPEG ({len(jpeg)} bytes)")
    else:
        print("FAIL: get_jpeg() did not return JPEG-framed bytes")
        fails += 1

    # Channel-order contract: draw a red box on a blank RGB frame and confirm
    # index 0 (R) is the hot channel -- this is the invariant every detector
    # relies on.
    test = B.blank_frame()
    B.draw_box(test, "RED", FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
    r = float(test[FRAME_HEIGHT // 2, FRAME_WIDTH // 2, 0])
    b = float(test[FRAME_HEIGHT // 2, FRAME_WIDTH // 2, 2])
    if r > 200 and b < 50:
        print(f"ok: RGB channel contract (R={r:.0f} > B={b:.0f})")
    else:
        print(f"FAIL: channel contract R={r:.0f} B={b:.0f}")
        fails += 1

    cam.close()
    print("\nSELF-CHECK:", "PASS" if fails == 0 else f"{fails} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--selfcheck" in sys.argv:
        raise SystemExit(run_selfcheck())
    run_live()
