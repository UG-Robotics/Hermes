"""
Camera abstraction with real and simulated backends.

The rest of the software just calls ``camera.get_jpeg()`` (for the dashboard
stream) or ``camera.get_frame()`` (for perception). Which backend supplies the
pixels is decided once, at construction:

    * REAL:      Picamera2 (Pi Camera on a Pi 4) if available, else OpenCV's
                 VideoCapture(0) as a fallback for a USB/webcam.
    * SYNTHETIC: a drawn WRO-style corridor with coloured pillars that pans as
                 the (simulated) robot drives, so the monitoring feed is alive
                 even with no camera attached.

Backends are picked opportunistically: asking for REAL on a machine with no
camera and no Picamera2/OpenCV degrades to SYNTHETIC with a warning rather than
crashing, keeping the "runs anywhere" promise.
"""

from __future__ import annotations

import io
import math
import threading
import time

from utils.logger import get_logger
from utils.telemetry_hub import get_hub

logger = get_logger(__name__)

# PIL is optional: with it we draw rich synthetic frames; without it we fall
# back to a static placeholder JPEG so the MJPEG stream still works.
try:
    from PIL import Image, ImageDraw
    _PIL = True
except Exception:
    _PIL = False

# numpy is optional too, and only needed for get_frame()'s synthetic-array
# fallback below (perception consumes numpy arrays, not JPEGs/PIL Images).
try:
    import numpy as np
    _NUMPY = True
except Exception:
    _NUMPY = False


class Camera:
    def __init__(self, simulated: bool = False, width: int = 320, height: int = 240, fps: int = 15):
        self._hub = get_hub()
        self.width = width
        self.height = height
        self.fps = fps
        self.source = "synthetic"
        self._impl = None
        self._lock = threading.Lock()
        self._last_ts = time.time()
        self._frame_count = 0

        if not simulated:
            self._impl = self._open_real()
            if self._impl is not None:
                self.source = self._impl_source
        if self._impl is None:
            if not simulated:
                logger.warning("No real camera available — falling back to SYNTHETIC frames.")
            else:
                logger.info("Camera in SYNTHETIC mode (drawn frames).")
            self.source = "synthetic"

        logger.info(f"Camera ready ({self.width}x{self.height} @ {self.fps}fps, source: {self.source}).")

    # ---------------------------------------------------------------- real cam
    def _open_real(self):
        # Try Picamera2 first (native on Pi 4 / Bookworm).
        try:
            from picamera2 import Picamera2  # type: ignore
            cam = Picamera2()
            config = cam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            cam.configure(config)
            cam.start()
            self._impl_source = "picamera2"
            logger.info("Opened Pi Camera via Picamera2.")
            return ("picamera2", cam)
        except Exception as e:
            logger.debug(f"Picamera2 unavailable: {e}")

        # Fall back to OpenCV VideoCapture (USB webcam).
        try:
            import cv2  # type: ignore
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self._impl_source = "opencv"
                logger.info("Opened camera via OpenCV VideoCapture(0).")
                return ("opencv", cap)
            cap.release()
        except Exception as e:
            logger.debug(f"OpenCV camera unavailable: {e}")

        return None

    # ------------------------------------------------------------------ public
    def get_jpeg(self) -> bytes:
        """Return the current frame as JPEG bytes for the MJPEG stream."""
        with self._lock:
            self._frame_count += 1
            now = time.time()
            dt = now - self._last_ts
            self._last_ts = now
            fps = (1.0 / dt) if dt > 0 else float(self.fps)
            self._hub.camera(self.width, self.height, fps, self.source)

            if self._impl is None:
                return self._synthetic_jpeg()
            return self._real_jpeg()

    def get_frame(self):
        """Return a raw frame (numpy array) for perception.

        Returns a drawn SYNTHETIC frame (see _synthetic_frame()) when
        there's no real backend, instead of None -- get_jpeg()'s
        _real_jpeg() already had this "no camera -> draw one instead"
        fallback for the dashboard stream; perception (pillar/lane/corner/
        parking detection, all under perception/) needs the same fallback
        on the raw-array path it actually reads, or none of that code ever
        runs at all under Runtime(simulated=True) -- it silently no-ops on
        `if frame is None: return`, which looks like "ran and saw nothing"
        rather than "never ran". Can still return None: only if neither
        PIL nor numpy is installed (see _synthetic_frame()), or a real
        backend's read genuinely fails this tick.
        """
        if self._impl is None:
            return self._synthetic_frame()
        kind, dev = self._impl
        try:
            if kind == "picamera2":
                return dev.capture_array()
            if kind == "opencv":
                ok, frame = dev.read()
                return frame if ok else None
        except Exception as e:
            logger.error(f"Camera capture failed: {e}")
        return None

    def close(self) -> None:
        if self._impl is None:
            return
        kind, dev = self._impl
        try:
            if kind == "picamera2":
                dev.stop()
            elif kind == "opencv":
                dev.release()
        except Exception:
            pass

    # ---------------------------------------------------------------- encoding
    def _real_jpeg(self) -> bytes:
        frame = self.get_frame()
        if frame is None:
            return self._synthetic_jpeg()
        try:
            import cv2  # type: ignore
            ok, buf = cv2.imencode(".jpg", frame)
            if ok:
                return buf.tobytes()
        except Exception:
            pass
        # Last resort if cv2 isn't around to encode.
        return self._synthetic_jpeg()

    # --------------------------------------------------------------- synthetic
    def _draw_synthetic_scene(self):
        """Draw the corridor + pillars scene and return it as a PIL Image,
        or None if PIL isn't installed. Shared by _synthetic_jpeg() (JPEG
        bytes, for the dashboard stream) and _synthetic_frame() (numpy
        array, for perception) so the two can never drift out of sync --
        pillar/lane/corner/parking detection see exactly the scene the
        dashboard is showing.
        """
        if not _PIL:
            return None

        # Pan the scene by the robot's integrated heading (degrees), not the raw
        # gyro rate — heading is what actually tells us where the car is pointing,
        # so the corridor swings the way a real forward camera would when turning.
        heading = 0.0
        status = self._hub.snapshot().get("status")
        if status:
            heading = status.get("heading_deg", 0.0)

        w, h = self.width, self.height
        img = Image.new("RGB", (w, h), (30, 32, 38))
        d = ImageDraw.Draw(img)

        # Ground and horizon.
        horizon = h // 2
        d.rectangle([0, horizon, w, h], fill=(52, 56, 64))
        d.rectangle([0, 0, w, horizon], fill=(22, 24, 30))

        # Perspective corridor lines, shifted by heading so turning is visible.
        shift = int(max(-w // 4, min(w // 4, heading * 1.5)))
        cx = w // 2 - shift
        d.line([(0, h), (cx - 10, horizon)], fill=(120, 200, 255), width=2)
        d.line([(w, h), (cx + 10, horizon)], fill=(120, 200, 255), width=2)
        d.line([(cx, horizon), (cx, h)], fill=(90, 90, 110), width=1)

        # A red and a green pillar that bob with the frame counter, so the feed
        # is obviously live.
        t = self._frame_count
        gy = horizon + 10 + int(20 * (0.5 + 0.5 * math.sin(t * 0.15)))
        d.ellipse([cx - 60, gy, cx - 40, gy + 30], fill=(40, 200, 90))
        ry = horizon + 10 + int(20 * (0.5 + 0.5 * math.cos(t * 0.15)))
        d.ellipse([cx + 40, ry, cx + 60, ry + 30], fill=(220, 60, 60))

        d.text((6, 4), f"SYNTHETIC CAM  f{t}", fill=(230, 230, 230))
        d.text((6, h - 14), f"hdg {heading:+.0f} deg", fill=(180, 180, 190))

        return img

    def _synthetic_jpeg(self) -> bytes:
        """Draw a simple corridor + pillars that pans with the simulated pose."""
        img = self._draw_synthetic_scene()
        if img is None:
            return _PLACEHOLDER_JPEG

        out = io.BytesIO()
        img.save(out, format="JPEG", quality=70)
        return out.getvalue()

    def _synthetic_frame(self):
        """Numpy-array counterpart to _synthetic_jpeg(), for get_frame().

        Returns None (same "nothing available" contract a real backend's
        failed read already has) only if PIL and/or numpy aren't
        installed -- everywhere the rest of the perception code already
        null-checks get_frame()'s result, so this degrades the same way a
        disconnected real camera would, it just can't happen on a normal
        install where requirements.txt's opencv-python pulls numpy in
        anyway.
        """
        if not _NUMPY:
            logger.warning("numpy not installed -- get_frame() has no synthetic fallback, "
                            "perception will see no frames in simulated mode.")
            return None

        img = self._draw_synthetic_scene()
        if img is None:
            logger.warning("Pillow not installed -- get_frame() has no synthetic fallback, "
                            "perception will see no frames in simulated mode.")
            return None

        # PIL Images are RGB; every perception module (pillar/track/corner/
        # finish_detection) already assumes a native RGB array, same
        # convention documented in perception/pillar_detection.py.
        return np.array(img)


def _build_placeholder() -> bytes:
    """Build a minimal valid grey JPEG at import time (no deps)."""
    if _PIL:
        img = Image.new("RGB", (16, 16), (60, 60, 68))
        out = io.BytesIO()
        img.save(out, format="JPEG")
        return out.getvalue()
    # Without PIL we can't synthesise a JPEG; return a known-good 1x1 grey JPEG.
    return bytes.fromhex(
        "ffd8ffdb0043000302020302020303030304030304050805050404050a070706080c0a0c0c0b0a0b0b0d0e12100d"
        "0e11000b0b0d13111516151416181d201d1618191c1e28211c1c2422"
        "1e1d1c1c1e2226231f24291c1c2229ffc0000b080001000101011100ffc4001f0000010501010101010100000000"
        "00000000000102030405060708090a0bffc400b5100002010303020403050504040000017d01020300041105122131"
        "410613516107227114328191a1082342b1c11552d1f02433627282090a161718191a25262728292a3435363738393a43"
        "4445464748494a535455565758595a636465666768696a737475767778797a838485868788898a92939495969798999a"
        "a2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3"
        "f4f5f6f7f8f9faffda0008010100003f00fbfeffd9"
    )


_PLACEHOLDER_JPEG = _build_placeholder()