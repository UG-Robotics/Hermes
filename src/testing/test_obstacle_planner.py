import unittest

from planning.obstacle_planner import adjust_avoidance_steer, general_speed_scale
from control.speed_controller import apply_speed_scale
from config.thresholds import (
    TOF_WALL_WARNING_MM, TOF_WALL_CRITICAL_MM, TOF_MAX_VALID_MM,
    AVOIDANCE_STEER_CAP_NEAR_WALL_DEG, SPEED_SCALE_MIN,
)


class TestAdjustAvoidanceSteer(unittest.TestCase):
    def test_far_walls_leave_steer_unchanged(self):
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=TOF_MAX_VALID_MM, tof_right_mm=TOF_MAX_VALID_MM)
        self.assertEqual(plan.steer_angle, 50)
        self.assertEqual(plan.wall_nudge_deg, 0.0)
        self.assertIsNone(plan.wall_side)

    def test_warning_range_caps_steer_toward_the_relevant_wall_only(self):
        # Steering RIGHT (passing a red pillar) cares about the RIGHT wall.
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=TOF_MAX_VALID_MM,
                                       tof_right_mm=TOF_WALL_WARNING_MM - 10)
        self.assertLessEqual(plan.steer_angle, AVOIDANCE_STEER_CAP_NEAR_WALL_DEG)
        self.assertEqual(plan.wall_side, "RIGHT")

    def test_left_wall_proximity_does_not_cap_a_rightward_avoidance(self):
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=10.0, tof_right_mm=TOF_MAX_VALID_MM)
        self.assertEqual(plan.steer_angle, 50)
        self.assertIsNone(plan.wall_side)

    def test_critical_range_caps_harder_and_nudges_away(self):
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=TOF_MAX_VALID_MM,
                                       tof_right_mm=TOF_WALL_CRITICAL_MM - 10)
        self.assertLess(plan.steer_angle, AVOIDANCE_STEER_CAP_NEAR_WALL_DEG)
        self.assertLess(plan.wall_nudge_deg, 0.0)  # nudge AWAY from the right wall = negative

    def test_capping_never_flips_steer_sign(self):
        plan = adjust_avoidance_steer(-50, "LEFT", tof_left_mm=TOF_WALL_CRITICAL_MM - 10,
                                       tof_right_mm=TOF_MAX_VALID_MM)
        self.assertLessEqual(plan.steer_angle, 0)
        self.assertGreater(plan.wall_nudge_deg, 0.0)  # nudge AWAY from the left wall = positive


class TestSpeedScale(unittest.TestCase):
    def test_far_walls_full_speed(self):
        self.assertEqual(general_speed_scale(TOF_MAX_VALID_MM, TOF_MAX_VALID_MM), 1.0)

    def test_critical_wall_floors_at_minimum_scale(self):
        scale = general_speed_scale(TOF_WALL_CRITICAL_MM - 10, TOF_MAX_VALID_MM)
        self.assertGreaterEqual(scale, SPEED_SCALE_MIN)
        self.assertLess(scale, 1.0)

    def test_apply_speed_scale_clamped(self):
        self.assertEqual(apply_speed_scale(200, 2.0), 200)   # scale clamped to 1.0
        self.assertGreater(apply_speed_scale(200, -1.0), 0)  # scale clamped to SPEED_SCALE_MIN


if __name__ == "__main__":
    unittest.main()
