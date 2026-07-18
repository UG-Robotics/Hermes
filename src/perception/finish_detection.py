"""
Parking-zone (magenta) detection.

WRO Future Engineers obstacle challenge: the parking lot is bounded by
magenta/pink lines. This module finds that marker in-frame -- nothing
more; runtime.py decides when to actually look (FINAL_APPROACH, once three
laps are done) and what to do about it (currently just raises
PARKING_ZONE_DETECTED -> PARK; the actual parking manoeuvre is
planning/parking_planner.py's job, which is still an empty stub).

Note on FINISH_ZONE_DETECTED (the open-challenge equivalent -- no colour
marker involved): the rules place the finish line "on the extension of the
starting line segment", i.e. it's a *position* on the track, not a
uniquely-coloured marker -- there's nothing distinct to point a camera at.
perception/corner_detection.py already counts corner-section crossings to
count laps, so "we've returned to the start straight for the Nth time"
falls out of that same count for free. runtime.py raises
FINISH_ZONE_DETECTED directly from that lap-completion bookkeeping (see
its _post_event_effects) instead of duplicating a positional check here
that the corner tracker has effectively already made. This file
intentionally only covers the piece that genuinely needs its own
colour-based detector: parking.
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
    HSV_MAGENTA_LOW, HSV_MAGENTA_HIGH,
    MIN_PARKING_MARKER_AREA, PARKING_CONFIRM_FRAMES,
)
from utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class ParkingObservation:
    """Result of one ParkingZoneTracker.update() call."""
    present: bool
    cx: Optional[int] = None
    area: Optional[float] = None
    confirmed: bool = False   # PARKING_CONFIRM_FRAMES consecutive sightings reached


def find_parking_marker(frame_rgb):
    """Largest magenta blob in-frame as (cx, area), or None.

    Frame is expected as a native RGB array, same convention as
    perception/pillar_detection.find_largest_pillar().
    """
    if frame_rgb is None or not _CV2_AVAILABLE:
        if frame_rgb is not None and not _CV2_AVAILABLE:
            logger.warning("opencv-python not installed, parking-zone detection disabled "
                            "(pip install opencv-python on the Pi to enable it).")
        return None

    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(HSV_MAGENTA_LOW), np.array(HSV_MAGENTA_HIGH))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    max_area = MIN_PARKING_MARKER_AREA
    for c in contours:
        area = cv2.contourArea(c)
        if area <= max_area:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        max_area = area
        best = (int(M["m10"] / M["m00"]), area)

    return best


class ParkingZoneTracker:
    """Requires PARKING_CONFIRM_FRAMES consecutive sightings before
    reporting confirmed=True, then latches (won't fire again this run) --
    this is a one-shot "we've arrived" signal, not a tracked, moving
    object like a pillar, so a run of consecutive hits stands in for the
    hysteresis PillarDetector/CornerTracker get from lost-frame counting.
    """

    def __init__(self):
        self._consecutive = 0
        self._raised = False

    def reset(self) -> None:
        self._consecutive = 0
        self._raised = False

    def update(self, frame_rgb) -> ParkingObservation:
        if self._raised:
            return ParkingObservation(present=False)

        found = find_parking_marker(frame_rgb)
        if found is None:
            self._consecutive = 0
            return ParkingObservation(present=False)

        cx, area = found
        self._consecutive += 1
        if self._consecutive >= PARKING_CONFIRM_FRAMES:
            self._raised = True
            logger.info(f"[PARKING] zone confirmed @ cx={cx} area={area:.0f}")
            return ParkingObservation(present=True, cx=cx, area=area, confirmed=True)

        return ParkingObservation(present=True, cx=cx, area=area)
