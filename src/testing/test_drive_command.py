import unittest
from state_machine.states import State
from state_machine.robot_context import RobotContext
from control.drive_command import drive_command

class TestDriveCommand(unittest.TestCase):

    def test_follow_track_goes_forward(self):
        ctx = RobotContext()
        speed, steer, action = drive_command(State.FOLLOW_TRACK, ctx)
        self.assertEqual(action, "FORWARD")
        self.assertGreater(speed, 0)

    def test_avoid_red_pillar_steers_left(self):
        ctx = RobotContext()
        ctx.last_pillar_color = "RED"
        _, steer, action = drive_command(State.AVOID_OBSTACLE, ctx)
        self.assertLess(steer, 0)
        self.assertEqual(action, "FORWARD")

    def test_avoid_green_pillar_steers_right(self):
        ctx = RobotContext()
        ctx.last_pillar_color = "GREEN"
        _, steer, action = drive_command(State.AVOID_OBSTACLE, ctx)
        self.assertGreater(steer, 0)

    def test_stop_state_halts(self):
        ctx = RobotContext()
        speed, _, action = drive_command(State.STOP, ctx)
        self.assertEqual(speed, 0)
        self.assertEqual(action, "STOP")

    def test_error_state_halts(self):
        ctx = RobotContext()
        speed, _, action = drive_command(State.ERROR, ctx)
        self.assertEqual(speed, 0)
        self.assertEqual(action, "STOP")

if __name__ == "__main__":
    unittest.main()