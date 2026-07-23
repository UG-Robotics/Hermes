"""
Corner-section marker detection (orange/blue floor lines) -> lap counting
and start-direction determination.

WRO Future Engineers track: 4 corner sections + 4 straight sections, and
each corner is marked by TWO coloured floor lines -- one where the car
ENTERS the corner and one where it EXITS. The two colours are orange and
blue; which one comes first depends on the run's (randomised) direction, and
the colour of the FIRST line the car ever sees IS the direction for the whole
run (ORANGE-first -> CLOCKWISE, BLUE-first -> COUNTER-CLOCKWISE), per the
standard WRO FE convention several public team repos document independently.
So one corner = 2 lines, one lap = 4 corners = 8 lines, and a 3-lap run = 24
lines (12 of each colour).

This module only does steps (i)-(ii) of that: detect that a corner LINE is
currently in view, and classify its colour. It does NOT count laps, decide
"clockwise" vs "counter-clockwise" semantics, or touch the state machine --
that bookkeeping lives in runtime.py's _post_event_effects (mirrors the
perception/planning split perception/pillar_detection.py already uses). Two
frame-to-frame edges are all this module reports, and they deliberately
mirror PillarObservation's new_detection/cleared fields:

    new_detection -- a corner LINE just appeared. This is the edge runtime.py
                      acts on: it raises LAP_MARKER_DETECTED, and the
                      FOLLOW_TRACK<->LAP_CHECK toggle pairs consecutive lines
                      into corners -- an entry line moves FOLLOW_TRACK ->
                      LAP_CHECK (slow through the corner), the next line
                      (the exit) moves LAP_CHECK -> FOLLOW_TRACK and counts one
                      corner. Every CORNERS_PER_LAP corners (8 lines) is a lap.
    cleared       -- the line just left view. Reported for completeness /
                      hysteresis, but runtime.py does NOT count on it (doing so
                      is what made a single line register as a whole corner and
                      double-counted this two-line-per-corner track).

Step (v) -- touching the serial link or the state machine -- is not done
here, same boundary perception/pillar_detection.py already documents.
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
    HSV_ORANGE_LOW, HSV_ORANGE_HIGH,
    HSV_BLUE_LOW, HSV_BLUE_HIGH,
    MIN_CORNER_MARKER_AREA, CORNER_MARKER_LOST_FRAMES,
)
from utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class CornerBlob:
    """Raw result of steps (i)+(ii): the largest valid corner-marker blob."""
    color: str   # "ORANGE" | "BLUE"
    cx: int
    cy: int
    area: float


@dataclass
class CornerObservation:
    """Result of one CornerTracker.update() call."""
    present: bool
    color: Optional[str] = None
    new_detection: bool = False   # marker just appeared this frame
    cleared: bool = False         # marker just disappeared this frame


def find_corner_marker(frame_rgb) -> Optional[CornerBlob]:
    """Steps (i)+(ii): find the largest orange-or-blue floor marker blob.

    Frame is expected as a native RGB array, same convention as
    perception/pillar_detection.find_largest_pillar().
    """
    if frame_rgb is None or not _CV2_AVAILABLE:
        if frame_rgb is not None and not _CV2_AVAILABLE:
            logger.warning("opencv-python not installed, corner detection disabled "
                            "(pip install opencv-python on the Pi to enable it).")
        return None

    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    mask_orange = cv2.inRange(hsv, np.array(HSV_ORANGE_LOW), np.array(HSV_ORANGE_HIGH))
    mask_blue = cv2.inRange(hsv, np.array(HSV_BLUE_LOW), np.array(HSV_BLUE_HIGH))

    kernel = np.ones((5, 5), np.uint8)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    best: Optional[CornerBlob] = None
    max_area = MIN_CORNER_MARKER_AREA

    for mask, color in ((mask_orange, "ORANGE"), (mask_blue, "BLUE")):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area <= max_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            max_area = area
            best = CornerBlob(
                color=color,
                cx=int(M["m10"] / M["m00"]),
                cy=int(M["m01"] / M["m00"]),
                area=area,
            )

    return best


class CornerTracker:
    """Stateful wrapper: turns per-frame marker detections into discrete
    new_detection/cleared edges with the same lost-frame hysteresis
    perception.pillar_detection.PillarDetector uses, so a single dropped
    frame doesn't cause a corner to be miscounted."""

    def __init__(self):
        self._active_color: Optional[str] = None
        self._lost_frames = 0

    def reset(self) -> None:
        self._active_color = None
        self._lost_frames = 0

    def update(self, frame_rgb, frame_width: int) -> CornerObservation:
        """frame_width is accepted, unused, purely so call sites match
        perception.pillar_detection.PillarDetector.update()'s signature."""
        blob = find_corner_marker(frame_rgb)

        if self._active_color is None:
            if blob is None:
                return CornerObservation(present=False)
            self._active_color = blob.color
            self._lost_frames = 0
            logger.info(f"[CORNER] {blob.color} marker detected @ cx={blob.cx}, area={blob.area:.0f}")
            return CornerObservation(present=True, color=blob.color, new_detection=True)

        if blob is not None and blob.color == self._active_color:
            self._lost_frames = 0
            return CornerObservation(present=True, color=self._active_color)

        # Marker missing (or changed colour mid-corner, which we ignore) --
        # count consecutive lost frames before declaring it cleared.
        self._lost_frames += 1
        if self._lost_frames >= CORNER_MARKER_LOST_FRAMES:
            logger.info(f"[CORNER] {self._active_color} marker cleared.")
            color = self._active_color
            self.reset()
            return CornerObservation(present=False, cleared=True, color=color)

        return CornerObservation(present=True, color=self._active_color)
