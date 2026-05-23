from dataclasses import dataclass
from typing import List, Tuple, Optional

from config.thresholds import PILLAR_HSV_RANGES, PILLAR_MIN_AREA


BBox = Tuple[int, int, int, int]


@dataclass
class PillarObservation:
	color: str
	bbox: BBox
	area: float
	confidence: float
	center: Tuple[int, int]


def detect_pillars_bgr(
	frame,
	hsv_ranges=None,
	min_area: int = PILLAR_MIN_AREA,
) -> List[PillarObservation]:
	"""Detect red/green pillars in a BGR frame using HSV thresholds."""

	if hsv_ranges is None:
		hsv_ranges = PILLAR_HSV_RANGES

	try:
		import cv2
		import numpy as np
	except ImportError as exc:
		raise ImportError("OpenCV and NumPy are required for pillar detection") from exc

	if frame is None:
		return []

	frame_area = max(frame.shape[0] * frame.shape[1], 1)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	detections: List[PillarObservation] = []

	for color, ranges in hsv_ranges.items():
		mask = None

		for lower, upper in ranges:
			lower_arr = np.array(lower, dtype=np.uint8)
			upper_arr = np.array(upper, dtype=np.uint8)
			current = cv2.inRange(hsv, lower_arr, upper_arr)
			mask = current if mask is None else cv2.bitwise_or(mask, current)

		if mask is None:
			continue

		contours_info = cv2.findContours(
			mask,
			cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE,
		)
		contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

		for contour in contours:
			area = float(cv2.contourArea(contour))
			if area < min_area:
				continue

			x, y, w, h = cv2.boundingRect(contour)
			center = (x + w // 2, y + h // 2)
			confidence = min(area / frame_area, 1.0)

			detections.append(
				PillarObservation(
					color=color,
					bbox=(x, y, w, h),
					area=area,
					confidence=confidence,
					center=center,
				)
			)

	detections.sort(key=lambda d: d.area, reverse=True)
	return detections


def best_pillar(frame, hsv_ranges=None, min_area: int = PILLAR_MIN_AREA) -> Optional[PillarObservation]:
	"""Return the largest pillar detection, if any."""

	detections = detect_pillars_bgr(frame, hsv_ranges=hsv_ranges, min_area=min_area)
	return detections[0] if detections else None
