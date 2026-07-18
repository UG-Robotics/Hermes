import unittest

try:
    import numpy as np
    _HAVE_NUMPY = True
except Exception:
    _HAVE_NUMPY = False

from perception.track_detection import detect_lane

FRAME_WIDTH = 320
FRAME_HEIGHT = 240


def _walled_frame(left_px=None, right_px=None, width=FRAME_WIDTH, height=FRAME_HEIGHT):
    """All-white frame with an optional solid black wall band on either side."""
    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    if left_px:
        frame[:, 0:left_px, :] = 0
    if right_px:
        frame[:, width - right_px:width, :] = 0
    return frame


@unittest.skipUnless(_HAVE_NUMPY, "numpy not installed")
class TestTrackDetection(unittest.TestCase):
    def test_no_frame_is_invalid(self):
        obs = detect_lane(None, FRAME_WIDTH, FRAME_HEIGHT)
        self.assertFalse(obs.valid)

    def test_no_walls_is_invalid(self):
        frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 255, dtype=np.uint8)
        obs = detect_lane(frame, FRAME_WIDTH, FRAME_HEIGHT)
        self.assertFalse(obs.valid)

    def test_symmetric_walls_give_zero_offset_full_confidence(self):
        frame = _walled_frame(left_px=60, right_px=60)
        obs = detect_lane(frame, FRAME_WIDTH, FRAME_HEIGHT)
        self.assertTrue(obs.valid)
        self.assertTrue(obs.left_present and obs.right_present)
        self.assertEqual(obs.confidence, 1.0)
        self.assertAlmostEqual(obs.offset_px, 0, delta=2)

    def test_wall_closer_on_right_gives_negative_offset(self):
        # Right wall intrudes further into frame than left -> corridor centre
        # shifts left of image centre -> offset should be negative.
        frame = _walled_frame(left_px=40, right_px=100)
        obs = detect_lane(frame, FRAME_WIDTH, FRAME_HEIGHT)
        self.assertTrue(obs.valid)
        self.assertLess(obs.offset_px, 0)

    def test_wall_closer_on_left_gives_positive_offset(self):
        frame = _walled_frame(left_px=100, right_px=40)
        obs = detect_lane(frame, FRAME_WIDTH, FRAME_HEIGHT)
        self.assertTrue(obs.valid)
        self.assertGreater(obs.offset_px, 0)

    def test_single_wall_gives_reduced_confidence(self):
        frame = _walled_frame(left_px=50)
        obs = detect_lane(frame, FRAME_WIDTH, FRAME_HEIGHT)
        self.assertTrue(obs.valid)
        self.assertTrue(obs.left_present)
        self.assertFalse(obs.right_present)
        self.assertEqual(obs.confidence, 0.5)


if __name__ == "__main__":
    unittest.main()
