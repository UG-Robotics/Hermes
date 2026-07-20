import unittest

from perception.track_detection import LaneObservation
from planning.lane_centering import compute_heading_nudge
from config.thresholds import LANE_MAX_NUDGE_DEG, LANE_MIN_CONFIDENCE, LANE_OFFSET_DEADBAND_PX

FRAME_WIDTH = 320


class TestLaneCentering(unittest.TestCase):
    def test_invalid_observation_gives_no_nudge(self):
        obs = LaneObservation(valid=False)
        self.assertEqual(compute_heading_nudge(obs, FRAME_WIDTH), 0.0)

    def test_low_confidence_gives_no_nudge(self):
        obs = LaneObservation(valid=True, offset_px=100, confidence=LANE_MIN_CONFIDENCE - 0.01)
        self.assertEqual(compute_heading_nudge(obs, FRAME_WIDTH), 0.0)

    def test_within_deadband_gives_no_nudge(self):
        obs = LaneObservation(valid=True, offset_px=LANE_OFFSET_DEADBAND_PX - 1, confidence=1.0)
        self.assertEqual(compute_heading_nudge(obs, FRAME_WIDTH), 0.0)

    def test_positive_offset_nudges_right(self):
        obs = LaneObservation(valid=True, offset_px=50, confidence=1.0)
        nudge = compute_heading_nudge(obs, FRAME_WIDTH)
        self.assertGreater(nudge, 0.0)

    def test_negative_offset_nudges_left(self):
        obs = LaneObservation(valid=True, offset_px=-50, confidence=1.0)
        nudge = compute_heading_nudge(obs, FRAME_WIDTH)
        self.assertLess(nudge, 0.0)

    def test_nudge_is_capped(self):
        obs = LaneObservation(valid=True, offset_px=10_000, confidence=1.0)
        nudge = compute_heading_nudge(obs, FRAME_WIDTH)
        self.assertLessEqual(abs(nudge), LANE_MAX_NUDGE_DEG)

    def test_positive_bias_pushes_a_centred_observation_rightward(self):
        # Perfectly centred corridor (offset 0), but a +bias (hug right/inner
        # wall) should still produce a rightward nudge.
        obs = LaneObservation(valid=True, offset_px=0, confidence=1.0)
        self.assertEqual(compute_heading_nudge(obs, FRAME_WIDTH), 0.0)
        biased = compute_heading_nudge(obs, FRAME_WIDTH, bias_px=40)
        self.assertGreater(biased, 0.0)

    def test_negative_bias_pushes_a_centred_observation_leftward(self):
        obs = LaneObservation(valid=True, offset_px=0, confidence=1.0)
        biased = compute_heading_nudge(obs, FRAME_WIDTH, bias_px=-40)
        self.assertLess(biased, 0.0)

    def test_bias_can_cancel_an_offset(self):
        # An offset of +30 with a -30 bias nets to zero -> inside deadband.
        obs = LaneObservation(valid=True, offset_px=30, confidence=1.0)
        self.assertEqual(compute_heading_nudge(obs, FRAME_WIDTH, bias_px=-30), 0.0)


if __name__ == "__main__":
    unittest.main()
