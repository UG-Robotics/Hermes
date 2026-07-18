import unittest

try:
    import numpy as np
    _HAVE_NUMPY = True
except Exception:
    _HAVE_NUMPY = False

from perception.corner_detection import CornerTracker
from config.thresholds import CORNERS_PER_LAP  # noqa: F401 (documents the assumption below)

FRAME_WIDTH = 320
FRAME_HEIGHT = 240


def _orange_frame():
    frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 255, dtype=np.uint8)
    # OpenCV HSV orange ~H=18 -> a mid-saturation orange-ish RGB patch.
    frame[100:180, 100:220, :] = (255, 140, 0)
    return frame


def _blue_frame():
    frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 255, dtype=np.uint8)
    frame[100:180, 100:220, :] = (0, 60, 220)
    return frame


def _blank_frame():
    return np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 255, dtype=np.uint8)


@unittest.skipUnless(_HAVE_NUMPY, "numpy not installed")
class TestCornerTracker(unittest.TestCase):
    def test_no_marker_stays_absent(self):
        tracker = CornerTracker()
        obs = tracker.update(_blank_frame(), FRAME_WIDTH)
        self.assertFalse(obs.present)
        self.assertFalse(obs.new_detection)

    def test_orange_marker_appearing_is_new_detection(self):
        tracker = CornerTracker()
        obs = tracker.update(_orange_frame(), FRAME_WIDTH)
        self.assertTrue(obs.present)
        self.assertTrue(obs.new_detection)
        self.assertEqual(obs.color, "ORANGE")

    def test_blue_marker_appearing_is_new_detection(self):
        tracker = CornerTracker()
        obs = tracker.update(_blue_frame(), FRAME_WIDTH)
        self.assertTrue(obs.new_detection)
        self.assertEqual(obs.color, "BLUE")

    def test_marker_disappearing_after_hysteresis_reports_cleared(self):
        tracker = CornerTracker()
        tracker.update(_orange_frame(), FRAME_WIDTH)  # new_detection
        obs = None
        for _ in range(10):
            obs = tracker.update(_blank_frame(), FRAME_WIDTH)
            if obs.cleared:
                break
        self.assertTrue(obs.cleared)
        self.assertEqual(obs.color, "ORANGE")

    def test_single_dropped_frame_does_not_clear_immediately(self):
        tracker = CornerTracker()
        tracker.update(_orange_frame(), FRAME_WIDTH)
        obs = tracker.update(_blank_frame(), FRAME_WIDTH)  # one missed frame only
        self.assertFalse(obs.cleared)
        self.assertTrue(obs.present)


if __name__ == "__main__":
    unittest.main()
