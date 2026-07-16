"""
Pillar detection + avoidance decision pipeline.

This module implements steps (i)-(iv) of the FOLLOW_TRACK pillar pipeline:

    i.   detect that *a* pillar is present in the frame
    ii.  classify its colour (RED or GREEN - magenta/parking is out of scope
         here and is explicitly ignored)
    iii. decide which side to pass on (pass RIGHT of RED, LEFT of GREEN)
    iv.  decide a steering angle and estimate distance/closeness so the
         caller knows when the pillar has been cleared

Step (v) - actually sending the steering angle to the ESP32 - is NOT done
here. This module is pure perception/decision logic with no knowledge of the
serial link; runtime.py is the only place that talks to communication/*, per
the existing architecture (perception -> planning/control -> communication).

Colour classification uses HSV thresholding (accounts for hue only, so
brightness/shadow variation on the mat doesn't flip a pillar's classification
the way RGB thresholding would). Red wraps the 0/180 hue boundary and needs
two ranges OR'd together; green does not.

Usage:
    detector = PillarDetector()
    obs = detector.update(frame_rgb, frame_width)  # once per tick
    if obs.new_detection:
        ... raise PILLAR_DETECTED_RED / PILLAR_DETECTED_GREEN ...
    if obs.cleared:
        ... raise OBSTACLE_CLEARED ...
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    import cv2
    import numpy as np
    _CV2_AVAILABLE = True
except Exception:  # pragma: no cover - exercised on bare-Python installs
    cv2 = None
    np = None
    _CV2_AVAILABLE = False

from config.camera_config import (
    HSV_RED_LOW1, HSV_RED_HIGH1, HSV_RED_LOW2, HSV_RED_HIGH2,
    HSV_GREEN_LOW, HSV_GREEN_HIGH,
    MIN_PILLAR_AREA, PILLAR_LOST_FRAMES,
    PILLAR_LATERAL_OFFSET_PX, PILLAR_STEER_KP,
    PILLAR_MIN_STEER_DEG, PILLAR_MAX_STEER_DEG,
    PILLAR_REAL_DIAMETER_MM, CAMERA_FOCAL_PX, PILLAR_CLEAR_DISTANCE_MM,
)
from config.robot_config import STEER_MIN, STEER_MAX
from utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class PillarBlob:
    """Raw result of steps (i)+(ii): the largest valid pillar blob, if any."""
    color: str          # "RED" | "GREEN"
    cx: int
    cy: int
    area: float
    width_px: int


@dataclass
class PillarObservation:
    """Result of a full detector.update() call - steps (i) through (iv)."""
    present: bool
    color: Optional[str] = None
    cx: Optional[int] = None
    area: Optional[float] = None
    direction: Optional[str] = None          # "RIGHT" | "LEFT" (pass side)
    steer_angle: int = 0                     # signed, -90..90 (+ = right)
    distance_mm: Optional[float] = None
    new_detection: bool = False              # first frame this pillar was confirmed
    cleared: bool = False                    # AVOID_OBSTACLE just finished


# ------------------------------------------------------------------ step i+ii
def find_largest_pillar(frame_rgb) -> Optional[PillarBlob]:
    """Steps (i) and (ii): find the largest red-or-green blob in the frame.

    Frame is expected as a native RGB array (e.g. Picamera2's capture_array()).
    Returns None if nothing large enough is found.
    """
    if frame_rgb is None or not _CV2_AVAILABLE:
        if frame_rgb is not None and not _CV2_AVAILABLE:
            logger.warning("opencv-python not installed, pillar detection disabled "
                            "(pip install opencv-python on the Pi to enable it).")
        return None

    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

    mask_red1 = cv2.inRange(hsv, np.array(HSV_RED_LOW1), np.array(HSV_RED_HIGH1))
    mask_red2 = cv2.inRange(hsv, np.array(HSV_RED_LOW2), np.array(HSV_RED_HIGH2))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, np.array(HSV_GREEN_LOW), np.array(HSV_GREEN_HIGH))

    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    best: Optional[PillarBlob] = None
    max_area = MIN_PILLAR_AREA

    for mask, color in ((mask_red, "RED"), (mask_green, "GREEN")):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area <= max_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            x, y, w, h = cv2.boundingRect(c)
            max_area = area
            best = PillarBlob(
                color=color,
                cx=int(M["m10"] / M["m00"]),
                cy=int(M["m01"] / M["m00"]),
                area=area,
                width_px=max(w, 1),
            )

    return best


def _estimate_distance_mm(width_px: int) -> float:
    """Pinhole-camera monocular distance estimate. Needs CAMERA_FOCAL_PX
    calibrated for the real lens; treat as a rough heuristic, not ground truth."""
    return (PILLAR_REAL_DIAMETER_MM * CAMERA_FOCAL_PX) / float(width_px)


# ------------------------------------------------------------------ step iii+iv
def decide_direction(color: str) -> str:
    """Step (iii): which side to pass the pillar on.

    Competition rule: pass RIGHT of RED pillars, LEFT of GREEN pillars.
    """
    return "RIGHT" if color == "RED" else "LEFT"


def compute_steer_angle(frame_width: int, blob: PillarBlob) -> int:
    """Step (iv): proportional steering angle to clear the pillar on the
    correct side.

    We steer toward an "aim point" offset from frame centre - RIGHT of centre
    to pass a RED pillar on its right, LEFT of centre to pass a GREEN pillar
    on its left - proportional to how far the pillar's centroid is from that
    aim point. A floor (PILLAR_MIN_STEER_DEG) guarantees we always commit to
    a real avoidance turn once a pillar is confirmed, even if it's currently
    dead-centre in frame (error ~ offset only).
    """
    center = frame_width // 2
    direction = decide_direction(blob.color)
    sign = 1 if direction == "RIGHT" else -1  # +steer = right, per STEER_MIN/MAX convention

    aim_x = center + sign * PILLAR_LATERAL_OFFSET_PX
    error_px = aim_x - blob.cx
    raw_steer = PILLAR_STEER_KP * error_px

    # Enforce the floor in the *correct* direction, then clamp to the ceiling.
    if sign > 0:
        steer = max(raw_steer, PILLAR_MIN_STEER_DEG)
        steer = min(steer, PILLAR_MAX_STEER_DEG)
    else:
        steer = min(raw_steer, -PILLAR_MIN_STEER_DEG)
        steer = max(steer, -PILLAR_MAX_STEER_DEG)

    return int(max(STEER_MIN, min(STEER_MAX, round(steer))))


# ------------------------------------------------------------------ detector
class PillarDetector:
    """Stateful wrapper: turns per-frame blob detections into discrete
    'a new pillar showed up' / 'we've cleared it' events with basic hysteresis
    so single-frame noise doesn't cause chattering state transitions.
    """

    def __init__(self):
        self._active_color: Optional[str] = None
        self._lost_frames = 0
        self._peak_area = 0.0

    def reset(self) -> None:
        self._active_color = None
        self._lost_frames = 0
        self._peak_area = 0.0

    def update(self, frame_rgb, frame_width: int) -> PillarObservation:
        blob = find_largest_pillar(frame_rgb)

        # ---- no pillar actively being tracked: only look for a NEW one -----
        if self._active_color is None:
            if blob is None:
                return PillarObservation(present=False)

            direction = decide_direction(blob.color)
            steer = compute_steer_angle(frame_width, blob)
            distance = _estimate_distance_mm(blob.width_px)

            self._active_color = blob.color
            self._lost_frames = 0
            self._peak_area = blob.area

            logger.info(
                f"[PILLAR] {blob.color} detected @ cx={blob.cx} area={blob.area:.0f} "
                f"-> pass {direction}, steer={steer:+d} deg, ~{distance:.0f}mm"
            )

            return PillarObservation(
                present=True, color=blob.color, cx=blob.cx, area=blob.area,
                direction=direction, steer_angle=steer, distance_mm=distance,
                new_detection=True,
            )

        # ---- a pillar is already being avoided: track it until cleared -----
        if blob is not None and blob.color == self._active_color:
            self._lost_frames = 0
            self._peak_area = max(self._peak_area, blob.area)
            distance = _estimate_distance_mm(blob.width_px)
            direction = decide_direction(blob.color)
            steer = compute_steer_angle(frame_width, blob)

            # Cleared once we're close alongside it AND its apparent size has
            # started shrinking again (i.e. we've passed the closest point).
            shrinking = blob.area < self._peak_area * 0.75
            close_enough = distance <= PILLAR_CLEAR_DISTANCE_MM
            if shrinking and close_enough:
                logger.info(f"[PILLAR] {self._active_color} cleared (shrinking + close).")
                self.reset()
                return PillarObservation(present=False, cleared=True)

            return PillarObservation(
                present=True, color=self._active_color, cx=blob.cx, area=blob.area,
                direction=direction, steer_angle=steer, distance_mm=distance,
            )

        # Blob missing (or changed colour, which we ignore mid-avoidance) -
        # count consecutive lost frames before declaring it cleared, so a
        # single dropped frame doesn't prematurely cut the avoidance manoeuvre.
        self._lost_frames += 1
        if self._lost_frames >= PILLAR_LOST_FRAMES:
            logger.info(f"[PILLAR] {self._active_color} cleared (out of view).")
            color = self._active_color
            self.reset()
            return PillarObservation(present=False, cleared=True, color=color)

        return PillarObservation(present=True, color=self._active_color)
