"""
Lane (track-boundary) detection.

WRO Future Engineers track: the drivable surface is white; the boundary
walls (both the fixed outer perimeter and the randomly-placed inner walls)
are black and 100mm tall, so from the Pi camera's low, forward-facing mount
they show up as a dark band along the lower-middle of the frame on
whichever side(s) a wall is actually present. There are no painted centre
lane lines on this mat -- only 20mm orange/blue corner-vs-straight markers
on the floor, which this module intentionally ignores (see
config/thresholds.py's module docstring). "The lane" here means the free
corridor between whichever walls bound the track at the robot's current
position, and "lane detection" means finding the pixel columns where each
wall starts.

This module owns steps (i)-(ii) of the corridor-centering pipeline: find
the left and right wall edges nearest the robot, and turn that into a
signed pixel offset of the corridor's centre from the image centre
(+ = corridor centre is to the right of the image centre, i.e. the bot has
drifted left). It does not touch steering -- planning/lane_centering.py
turns this into a heading nudge, and control/steering_control.py is what
actually moves the servo. Mirrors the perception/planning split already
used by perception/pillar_detection.py.

Ownership: a Pi-side computation over camera pixels only. No knowledge of
the IMU or the ESP32 -- purely vision, same as pillar_detection.py.
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

from config.thresholds import (
    LANE_ROI_TOP_FRAC, LANE_ROI_BOTTOM_FRAC,
    LANE_BLACK_S_MAX, LANE_BLACK_V_MAX,
    LANE_SMOOTH_KERNEL, LANE_MIN_WALL_FRACTION,
    LANE_MAX_OFFSET_FRAC, LANE_DEFAULT_HALF_WIDTH_PX,
)
from utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class LaneObservation:
    """Result of one detect_lane() call."""
    valid: bool
    offset_px: int = 0                     # + = corridor centre is right of image centre
    left_wall_px: Optional[int] = None
    right_wall_px: Optional[int] = None
    left_present: bool = False
    right_present: bool = False
    # 1.0 = both walls seen (trustworthy), 0.5 = one wall + an assumed
    # corridor width (a guess), 0.0 = nothing usable (see detect_lane()).
    confidence: float = 0.0


def _black_wall_mask(roi_rgb):
    """HSV threshold isolating dark (wall) pixels within an ROI. Value/
    saturation only -- black has no meaningful hue, unlike the red/green
    pillar thresholds in perception/pillar_detection.py."""
    hsv = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2HSV)
    lower = np.array([0, 0, 0])
    upper = np.array([179, LANE_BLACK_S_MAX, LANE_BLACK_V_MAX])
    return cv2.inRange(hsv, lower, upper)


def _wall_column_profile(mask) -> "np.ndarray":
    """Per-column black-pixel counts across the ROI, smoothed to resist
    single-column noise (specular highlights on the wall top edge, etc.)."""
    counts = (mask.sum(axis=0) // 255).astype(float)
    if LANE_SMOOTH_KERNEL > 1:
        kernel = np.ones(LANE_SMOOTH_KERNEL) / LANE_SMOOTH_KERNEL
        counts = np.convolve(counts, kernel, mode="same")
    return counts


def _find_wall_edge(col_counts, start: int, direction: int, min_pixels: float) -> Optional[int]:
    """Scan outward from `start` toward `direction` (-1 left, +1 right) for
    the first column whose smoothed black-pixel count clears `min_pixels`.
    Returns that column index, or None if no wall was found on this side."""
    n = len(col_counts)
    x = start
    while 0 <= x < n:
        if col_counts[x] >= min_pixels:
            return x
        x += direction
    return None


def detect_lane(frame_rgb, frame_width: int, frame_height: int) -> LaneObservation:
    """Find the corridor centre offset for one frame.

    frame_rgb: native RGB array (e.g. Picamera2's capture_array(), same
    convention as perception/pillar_detection.find_largest_pillar()).
    """
    if frame_rgb is None or not _CV2_AVAILABLE:
        if frame_rgb is not None and not _CV2_AVAILABLE:
            logger.warning("opencv-python not installed, lane detection disabled "
                            "(pip install opencv-python on the Pi to enable it).")
        return LaneObservation(valid=False)

    y0 = int(frame_height * LANE_ROI_TOP_FRAC)
    y1 = int(frame_height * LANE_ROI_BOTTOM_FRAC)
    if y1 <= y0 or frame_width <= 0:
        return LaneObservation(valid=False)

    roi = frame_rgb[y0:y1, :, :]
    mask = _black_wall_mask(roi)
    col_counts = _wall_column_profile(mask)

    roi_height = y1 - y0
    min_pixels = roi_height * LANE_MIN_WALL_FRACTION

    center = frame_width // 2
    left_wall = _find_wall_edge(col_counts, center, -1, min_pixels)
    right_wall = _find_wall_edge(col_counts, center, 1, min_pixels)

    left_present = left_wall is not None
    right_present = right_wall is not None

    if not left_present and not right_present:
        return LaneObservation(valid=False)

    # Corridor-centre estimate:
    #  * both walls seen  -> true midpoint (high confidence).
    #  * one wall only    -> assume LANE_DEFAULT_HALF_WIDTH_PX of corridor on
    #    the free side of that wall -- the common case in a wide (1000mm)
    #    corridor or mid-corner, where the far wall is out of frame.
    if left_present and right_present:
        corridor_center = (left_wall + right_wall) / 2.0
        confidence = 1.0
    elif left_present:
        corridor_center = left_wall + LANE_DEFAULT_HALF_WIDTH_PX
        confidence = 0.5
    else:
        corridor_center = right_wall - LANE_DEFAULT_HALF_WIDTH_PX
        confidence = 0.5

    max_offset = int((frame_width / 2.0) * LANE_MAX_OFFSET_FRAC)
    offset = int(max(-max_offset, min(max_offset, corridor_center - center)))

    return LaneObservation(
        valid=True, offset_px=offset,
        left_wall_px=left_wall, right_wall_px=right_wall,
        left_present=left_present, right_present=right_present,
        confidence=confidence,
    )
