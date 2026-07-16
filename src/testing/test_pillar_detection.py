import unittest

try:
    import numpy as np
    import cv2
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False

from perception.pillar_detection import (
    PillarDetector, decide_direction, compute_steer_angle, PillarBlob,
)
from config.camera_config import FRAME_WIDTH


def _synthetic_frame(color: str, cx: int, width: int = FRAME_WIDTH, height: int = 240):
    """Draw a solid-colour rectangle 'pillar' onto an RGB frame for detection tests."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    bgr = (0, 0, 255) if color == "RED" else (0, 255, 0)  # cv2 draws in BGR
    rgb = (bgr[2], bgr[1], bgr[0])
    cv2.rectangle(frame, (cx - 20, height // 2 - 20), (cx + 20, height // 2 + 20), rgb, -1)
    return frame


@unittest.skipUnless(_HAVE_CV2, "opencv-python not installed")
class TestPillarDetection(unittest.TestCase):
    def test_decide_direction(self):
        self.assertEqual(decide_direction("RED"), "RIGHT")
        self.assertEqual(decide_direction("GREEN"), "LEFT")

    def test_steer_sign_matches_direction(self):
        red_blob = PillarBlob(color="RED", cx=FRAME_WIDTH // 2, cy=120, area=1000, width_px=40)
        green_blob = PillarBlob(color="GREEN", cx=FRAME_WIDTH // 2, cy=120, area=1000, width_px=40)
        self.assertGreater(compute_steer_angle(FRAME_WIDTH, red_blob), 0)
        self.assertLess(compute_steer_angle(FRAME_WIDTH, green_blob), 0)

    def test_detector_raises_new_detection_once(self):
        detector = PillarDetector()
        frame = _synthetic_frame("RED", FRAME_WIDTH // 2)

        obs1 = detector.update(frame, FRAME_WIDTH)
        self.assertTrue(obs1.new_detection)
        self.assertEqual(obs1.color, "RED")
        self.assertGreater(obs1.steer_angle, 0)

        obs2 = detector.update(frame, FRAME_WIDTH)
        self.assertFalse(obs2.new_detection)
        self.assertTrue(obs2.present)

    def test_detector_clears_after_lost_frames(self):
        detector = PillarDetector()
        frame = _synthetic_frame("GREEN", FRAME_WIDTH // 2)
        detector.update(frame, FRAME_WIDTH)

        blank = np.zeros((240, FRAME_WIDTH, 3), dtype=np.uint8)
        obs = None
        for _ in range(10):
            obs = detector.update(blank, FRAME_WIDTH)
            if obs.cleared:
                break
        self.assertTrue(obs.cleared)


if __name__ == "__main__":
    unittest.main()
