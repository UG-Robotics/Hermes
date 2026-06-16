import unittest
from state_machine.robot_context import RobotContext

class TestRobotContext(unittest.TestCase):
    def test_default_values(self):
        ctx = RobotContext()
        self.assertEqual(
            ctx.lap_count,
            0
        )
        self.assertEqual(
            ctx.current_state,
            "INIT"
        )

    def test_increment_lap(self):
        ctx = RobotContext()
        ctx.increment_lap()
        self.assertEqual(
            ctx.lap_count,
            1
        )

    def test_state_update(self):
        ctx = RobotContext()
        ctx.update_state(
            "FOLLOW_TRACK"
        )
        self.assertEqual(
            ctx.current_state,
            "FOLLOW_TRACK"
        )
        self.assertEqual(
            ctx.previous_state,
            "INIT"
        )

    def test_error(self):
        ctx = RobotContext()
        ctx.set_error(
            "Camera Failure"
        )
        self.assertTrue(
            ctx.error_flag
        )
        ctx.clear_error()
        self.assertFalse(
            ctx.error_flag
        )

    def test_reset(self):
        ctx = RobotContext()
        ctx.increment_lap()
        ctx.update_state(
            "PARK"
        )
        ctx.reset()
        self.assertEqual(
            ctx.lap_count,
            0
        )
        self.assertEqual(
            ctx.current_state,
            "INIT"
        )


if __name__ == "__main__":
    unittest.main()