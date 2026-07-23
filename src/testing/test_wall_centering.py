import unittest

from planning.wall_centering import compute_centering_nudge, CenteringObservation
from config.thresholds import (
    TOF_WALL_SEEN_MAX_MM, TOF_CENTER_DEADBAND_MM, TOF_CENTER_MAX_NUDGE_DEG,
    TOF_WALL_FOLLOW_TARGET_MM,
)

# A distance the module treats as "no wall this side".
OUT = TOF_WALL_SEEN_MAX_MM + 500.0


class TestWallCentering(unittest.TestCase):
    # ------------------------------------------------------------- both walls
    def test_centered_no_nudge(self):
        obs = compute_centering_nudge(500.0, 500.0)
        self.assertTrue(obs.valid)
        self.assertEqual(obs.mode, "BOTH")
        self.assertEqual(obs.nudge_deg, 0.0)

    def test_displaced_toward_right_wall_steers_left(self):
        # right wall closer -> displaced right (+offset) -> steer left (-nudge).
        obs = compute_centering_nudge(700.0, 300.0)
        self.assertEqual(obs.mode, "BOTH")
        self.assertGreater(obs.offset_mm, 0)
        self.assertLess(obs.nudge_deg, 0)

    def test_displaced_toward_left_wall_steers_right(self):
        obs = compute_centering_nudge(300.0, 700.0)
        self.assertLess(obs.offset_mm, 0)
        self.assertGreater(obs.nudge_deg, 0)

    def test_nudge_clamped(self):
        obs = compute_centering_nudge(1100.0, 100.0)  # huge offset
        self.assertAlmostEqual(abs(obs.nudge_deg), TOF_CENTER_MAX_NUDGE_DEG)

    def test_small_offset_within_deadband(self):
        # offset = (left-right)/2; keep it under the deadband.
        left = 500.0 + (TOF_CENTER_DEADBAND_MM - 2)
        obs = compute_centering_nudge(left, 500.0)
        self.assertTrue(obs.valid)
        self.assertEqual(obs.nudge_deg, 0.0)

    # --------------------------------------------------------------- one wall
    def test_left_wall_only_follows_target(self):
        # left wall seen, right out of range. Farther than target from the
        # left wall == displaced right == steer left.
        obs = compute_centering_nudge(TOF_WALL_FOLLOW_TARGET_MM + 200, OUT)
        self.assertEqual(obs.mode, "LEFT_WALL")
        self.assertTrue(obs.left_seen and not obs.right_seen)
        self.assertLess(obs.nudge_deg, 0)

    def test_right_wall_only_follows_target(self):
        obs = compute_centering_nudge(OUT, TOF_WALL_FOLLOW_TARGET_MM + 200)
        self.assertEqual(obs.mode, "RIGHT_WALL")
        self.assertTrue(obs.right_seen and not obs.left_seen)
        self.assertGreater(obs.nudge_deg, 0)

    def test_one_wall_at_target_no_nudge(self):
        obs = compute_centering_nudge(TOF_WALL_FOLLOW_TARGET_MM + 2, OUT)
        self.assertEqual(obs.mode, "LEFT_WALL")
        self.assertEqual(obs.nudge_deg, 0.0)

    # ---------------------------------------------------------------- no wall
    def test_no_walls_invalid(self):
        obs = compute_centering_nudge(OUT, OUT)
        self.assertFalse(obs.valid)
        self.assertEqual(obs.mode, "NONE")
        self.assertEqual(obs.nudge_deg, 0.0)

    def test_seen_boundary_exclusive(self):
        # Exactly at the threshold counts as NOT seen (strict <).
        obs = compute_centering_nudge(TOF_WALL_SEEN_MAX_MM, TOF_WALL_SEEN_MAX_MM)
        self.assertFalse(obs.valid)

    # ------------------------------------------------------- inner-wall bias
    def test_positive_bias_hugs_right_wall(self):
        # +bias (CLOCKWISE, inner = right): even perfectly centred, steer
        # toward the right wall to hug the inner line.
        obs = compute_centering_nudge(500.0, 500.0, bias_mm=120.0)
        self.assertGreater(obs.nudge_deg, 0)

    def test_negative_bias_hugs_left_wall(self):
        obs = compute_centering_nudge(500.0, 500.0, bias_mm=-120.0)
        self.assertLess(obs.nudge_deg, 0)

    def test_bias_only_applies_to_both_walls(self):
        # One-wall mode ignores bias (it holds the fixed target clearance).
        at_target = compute_centering_nudge(TOF_WALL_FOLLOW_TARGET_MM, OUT, bias_mm=120.0)
        self.assertEqual(at_target.nudge_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
