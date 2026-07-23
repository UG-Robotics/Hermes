"""
Shared helpers for the camera bench tests (testing/camera/*).

Design choice: every live test grabs pixels through the project's real
``perception.camera.Camera`` object, NOT a private ``Picamera2()`` opened
inside the test. That way each bench exercises the exact acquisition path
the runtime uses -- backend selection, the RGB<->BGR channel swap
(``Camera._swap_rb``), resolution -- so a green-light here means the real
pipeline gets the same pixels, not a look-alike that could drift from
production (the trap the old testing/vision/pillar_vision_test.py fell into
by re-opening the camera itself).

Run any bench from the ``src/`` directory, e.g.::

    cd src
    python -m testing.camera.pillar_test            # live, needs a camera
    python -m testing.camera.pillar_test --selfcheck  # synthetic, no camera

Both ``python -m testing.camera.<name>`` and a direct
``python testing/camera/<name>.py`` work (the sys.path shim below handles
the direct case).
"""

from __future__ import annotations

import pathlib
import sys
import time

# Allow ``python testing/camera/foo.py`` from src/ (parents[2] == src root,
# the dir that holds perception/, config/, ...).
_SRC_ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

try:
    import cv2
    import numpy as np
    HAVE_CV2 = True
except Exception:  # pragma: no cover - bench needs cv2 to draw/save anything
    cv2 = None
    np = None
    HAVE_CV2 = False

# Where live benches drop annotated debug frames for you to scp/view.
DEBUG_DIR = pathlib.Path(__file__).resolve().parent / "_debug_out"


# --------------------------------------------------------------------- camera
def open_camera(simulated: bool = False):
    """Open the project Camera. Falls back to synthetic frames on its own if
    no real backend is present (see perception/camera.py); use --selfcheck
    mode instead when you deliberately want no hardware."""
    from perception.camera import Camera
    return Camera(simulated=simulated)


def require_cv2() -> None:
    if not HAVE_CV2:
        raise SystemExit(
            "opencv-python + numpy are required for this test.\n"
            "  pip install opencv-python numpy   (on the Pi)"
        )


# ------------------------------------------------------------------ debug I/O
def save_rgb(frame_rgb, name: str) -> pathlib.Path | None:
    """Write an RGB frame to testing/camera/_debug_out/<name> as PNG.
    cv2.imwrite expects BGR, so flip on the way out."""
    if not HAVE_CV2 or frame_rgb is None:
        return None
    DEBUG_DIR.mkdir(exist_ok=True)
    out = DEBUG_DIR / name
    cv2.imwrite(str(out), frame_rgb[:, :, ::-1])
    return out


def hsv_stats(frame_rgb, x0: int, y0: int, x1: int, y1: int):
    """Mean H,S,V over a rectangular patch of an RGB frame -- the number you
    plug into config/camera_config.py when a colour won't classify."""
    require_cv2()
    patch = frame_rgb[y0:y1, x0:x1]
    hsv = cv2.cvtColor(patch, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (float(h.mean()), float(s.mean()), float(v.mean()))


# -------------------------------------------------------------- synthetic art
# Colours below are RGB values chosen to land squarely inside the default
# HSV bounds in config/camera_config.py, so the synthetic self-checks verify
# the *code path* independently of on-mat colour calibration.
SYNTH_RGB = {
    "RED": (255, 0, 0),      # -> HSV H~0
    "GREEN": (0, 255, 0),    # -> H~60
    "ORANGE": (255, 120, 0),  # -> H~14  (corner marker)
    "BLUE": (0, 0, 255),     # -> H~120 (corner marker)
    "MAGENTA": (255, 0, 255),  # -> H~150 (parking)
    "BLACK": (0, 0, 0),      # -> wall (low S, low V)
}


def blank_frame(width: int = 320, height: int = 240):
    require_cv2()
    return np.zeros((height, width, 3), dtype=np.uint8)


def draw_box(frame_rgb, color_name: str, cx: int, cy: int, half_w: int = 20, half_h: int = 20):
    """Draw a filled RGB rectangle 'blob' of a named colour, centred at
    (cx,cy). Returns the frame for chaining."""
    require_cv2()
    r, g, b = SYNTH_RGB[color_name]
    cv2.rectangle(
        frame_rgb,
        (cx - half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (r, g, b), -1,
    )
    return frame_rgb


def draw_wall(frame_rgb, x0: int, x1: int, y0: int, y1: int):
    """Draw a black vertical wall band (for lane tests)."""
    require_cv2()
    cv2.rectangle(frame_rgb, (x0, y0), (x1, y1), (0, 0, 0), -1)
    return frame_rgb


# ---------------------------------------------------------------- live driver
def live_loop(step_fn, hz: float = 10.0, save_every_s: float = 1.0,
              save_name: str = "frame.png", simulated: bool = False) -> None:
    """Open the camera and call ``step_fn(frame_rgb, frame_idx) -> (line, annotated_rgb)``
    every tick. Prints ``line``; saves ``annotated_rgb`` to _debug_out every
    ``save_every_s`` seconds. Ctrl-C to stop; the camera is always released.
    """
    cam = open_camera(simulated=simulated)
    print(f"Camera source: {cam.source}  ({cam.width}x{cam.height})")
    if cam.source == "synthetic" and not simulated:
        print("!! WARNING: no real camera detected -- you are looking at DRAWN "
              "frames, not the lens. Nothing below reflects the real world.")
    print("Press Ctrl-C to stop.\n")

    interval = 1.0 / hz if hz > 0 else 0.0
    last_save = 0.0
    idx = 0
    try:
        while True:
            t0 = time.time()
            frame = cam.get_frame()
            line, annotated = step_fn(frame, idx)
            if line:
                print(line)
            if annotated is not None and (t0 - last_save) >= save_every_s:
                p = save_rgb(annotated, save_name)
                last_save = t0
                if p:
                    print(f"   [saved {p}]")
            idx += 1
            dt = time.time() - t0
            if interval > dt:
                time.sleep(interval - dt)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.close()
