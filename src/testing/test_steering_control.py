import unittest

from control.steering_control import SteeringController, _wrap_deg
from control.pid import PIDController


class TestWrapDeg(unittest.TestCase):
    def test_wraps_into_range(self):
        self.assertAlmostEqual(_wrap_deg(200), -160)
        self.assertAlmostEqual(_wrap_deg(-200), 160)
        self.assertAlmostEqual(_wrap_deg(10), 10)


class TestPIDController(unittest.TestCase):
    def test_zero_error_zero_output(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        self.assertEqual(pid.update(0.0, 0.05), 0.0)

    def test_output_clamped(self):
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_min=-5.0, output_max=5.0)
        self.assertEqual(pid.update(100.0, 0.05), 5.0)
        self.assertEqual(pid.update(-100.0, 0.05), -5.0)


class TestSteeringController(unittest.TestCase):
    def test_first_compute_locks_current_heading(self):
        sc = SteeringController()
        steer, error = sc.compute(current_heading_deg=15.0, dt=0.05)
        self.assertEqual(steer, 0)
        self.assertEqual(sc.target_heading_deg, 15.0)

    def test_holds_straight_corrects_drift(self):
        sc = SteeringController()
        sc.hold_straight(0.0)
        # Bot has drifted 10 degrees right of target -> correction should steer left (negative).
        steer, error = sc.compute(current_heading_deg=10.0, dt=0.05)
        self.assertLess(steer, 0)
        self.assertAlmostEqual(error, -10.0, places=3)

    def test_turn_by_sets_offset_target(self):
        sc = SteeringController()
        sc.turn_by(current_heading_deg=0.0, delta_deg=30.0)
        self.assertAlmostEqual(sc.target_heading_deg, 30.0)
        steer, error = sc.compute(current_heading_deg=0.0, dt=0.05)
        self.assertGreater(steer, 0)  # need to turn right to reach +30

    def test_nudge_target_shifts_without_resetting_pid(self):
        sc = SteeringController()
        sc.hold_straight(0.0)
        sc.compute(current_heading_deg=5.0, dt=0.05)  # build up some PID state
        integral_before = sc._pid._integral
        sc.nudge_target(2.0)
        self.assertAlmostEqual(sc.target_heading_deg, 2.0)
        # nudge_target must not reset accumulated PID state (unlike turn_by).
        self.assertEqual(sc._pid._integral, integral_before)

    def test_nudge_target_is_noop_before_any_target_locked(self):
        sc = SteeringController()
        sc.nudge_target(5.0)
        self.assertIsNone(sc.target_heading_deg)


if __name__ == "__main__":
    unittest.main()
