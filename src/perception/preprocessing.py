import cv2
import numpy as np
from config.camera_config import (
    FRAME_WIDTH, HSV_RED_LOW1, HSV_RED_HIGH1,
    HSV_RED_LOW2, HSV_RED_HIGH2, HSV_GREEN_LOW, HSV_GREEN_HIGH, MIN_PILLAR_AREA
)

def detect_pillars(frame_rgb):
    """
    Detects the closest red or green pillar using native RGB frames from Picamera2.
    Returns:
        tuple: (string 'RED' or 'GREEN' or None, integer 'cx' or None)
    """
    if frame_rgb is None:
        return None, None

    # Convert native RGB Pi Cam array directly to HSV to avoid channel swapping bugs
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

    # 1. Red Masking (handling the hue wrap around the 0/180 boundary)
    mask_red1 = cv2.inRange(hsv, np.array(HSV_RED_LOW1), np.array(HSV_RED_HIGH1))
    mask_red2 = cv2.inRange(hsv, np.array(HSV_RED_LOW2), np.array(HSV_RED_HIGH2))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 2. Green Masking
    mask_green = cv2.inRange(hsv, np.array(HSV_GREEN_LOW), np.array(HSV_GREEN_HIGH))

    # Clean up image noise using Morphological Operations (Opening)
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    # Find Contours
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_type = None
    best_cx = None
    max_area = 0

    # Process Red Contours
    for c in contours_red:
        area = cv2.contourArea(c)
        if area > MIN_PILLAR_AREA and area > max_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                max_area = area
                best_cx = int(M["m10"] / M["m00"])
                best_type = 'RED'

    # Process Green Contours (prefer the largest visible object overall)
    for c in contours_green:
        area = cv2.contourArea(c)
        if area > MIN_PILLAR_AREA and area > max_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                max_area = area
                best_cx = int(M["m10"] / M["m00"])
                best_type = 'GREEN'

    return best_type, best_cx