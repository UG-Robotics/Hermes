import unittest

from planning.obstacle_planner import adjust_avoidance_steer, general_speed_scale
from control.speed_controller import apply_speed_scale
from config.thresholds import (
    TOF_WALL_WARNING_MM, TOF_WALL_CRITICAL_MM, TOF_MAX_VALID_MM,
    TOF_MIN_PLAUSIBLE_SUM_MM,
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


class TestTofCrossCheck(unittest.TestCase):
    """A critically-close reading the OPPOSITE side proves impossible (both
    walls too close to bound any legal corridor) is a lying sensor -- it must
    NOT trigger a hard steer-cap / brake. A close reading the opposite side is
    consistent with (real hug, or a corner where the far wall is out of range)
    must still be honoured."""

    def test_both_sides_impossibly_close_is_ignored(self):
        # left=right=79mm -> sum 158 < 300mm -> physically impossible -> phantom.
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=79.0, tof_right_mm=79.0)
        self.assertEqual(plan.steer_angle, 50)       # not capped
        self.assertEqual(plan.wall_nudge_deg, 0.0)   # not nudged
        self.assertIsNone(plan.wall_side)

    def test_genuine_hug_is_still_capped(self):
        # right=79mm with left=900mm -> sum 979 >= 300 -> consistent with really
        # hugging the right wall -> the cap/nudge MUST still fire.
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=900.0, tof_right_mm=79.0)
        self.assertEqual(plan.wall_side, "RIGHT")
        self.assertLess(plan.wall_nudge_deg, 0.0)
        self.assertLess(plan.steer_angle, AVOIDANCE_STEER_CAP_NEAR_WALL_DEG)

    def test_close_reading_at_a_corner_is_trusted(self):
        # Far wall out of range (a corner/opening): nothing to cross-check, so a
        # close near-wall reading is taken at face value.
        plan = adjust_avoidance_steer(50, "RIGHT", tof_left_mm=TOF_MAX_VALID_MM, tof_right_mm=79.0)
        self.assertEqual(plan.wall_side, "RIGHT")
        self.assertLess(plan.wall_nudge_deg, 0.0)

    def test_speed_not_scaled_on_phantom(self):
        # Impossible pair -> ignored -> full speed, no phantom slowdown.
        self.assertEqual(general_speed_scale(79.0, 79.0), 1.0)

    def test_speed_still_scaled_on_real_wall(self):
        # Real close wall (opposite far) still slows the car.
        self.assertLess(general_speed_scale(900.0, 79.0), 1.0)

    def test_boundary_sum_at_threshold_is_trusted(self):
        # Sum exactly at the threshold is NOT below it -> treated as real (the
        # check is strictly `< TOF_MIN_PLAUSIBLE_SUM_MM`). Split it across two
        # sub-critical readings so both are in the critical band.
        a = TOF_WALL_CRITICAL_MM - 1                      # 79
        b = TOF_MIN_PLAUSIBLE_SUM_MM - a                  # 221 (>critical, in range)
        # Here only `a` is critical; b is a mid-range wall. a+b == threshold, so
        # a is NOT flagged and the cap on the a-side still applies.
        plan = adjust_avoidance_steer(50, "LEFT", tof_left_mm=a, tof_right_mm=b)
        self.assertEqual(plan.wall_side, "LEFT")


if __name__ == "__main__":
    unittest.main()
