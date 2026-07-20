import unittest

from planning.parking_planner import ParkingManeuver, park_side_for_direction
from config.parking_config import PARK_SETTLE_S, PARK_STEER_DEG

FAR = 2000.0  # ToF "no wall" so the side-clearance guard never trips

# Hold heading at a fixed 20 deg from the ref (0.0) throughout: that's above
# PARK_ALIGN_TOLERANCE_DEG (so REVERSE_STRAIGHTEN doesn't end instantly) and
# below PARK_TURN_IN_DEG (so REVERSE_IN doesn't end instantly) -- i.e. neither
# IMU heading guard fires and both reversing stages run on their timers, giving
# a deterministic command sequence to assert on.
HELD_HEADING = 20.0


def _drive_to_completion(park, heading=HELD_HEADING, dt=0.1, max_ticks=500):
    """Step the maneuver with a fixed dt until done (or give up). Returns the
    sequence of (speed, steer, action) commands seen."""
    cmds = []
    for _ in range(max_ticks):
        cmds.append(park.step(dt, heading, FAR, FAR))
        if park.done:
            break
    return cmds


class TestParkSideForDirection(unittest.TestCase):
    def test_clockwise_parks_left(self):
        self.assertEqual(park_side_for_direction("CLOCKWISE"), "LEFT")

    def test_counter_clockwise_parks_right(self):
        self.assertEqual(park_side_for_direction("COUNTER_CLOCKWISE"), "RIGHT")

    def test_unknown_direction_defaults_left(self):
        self.assertEqual(park_side_for_direction(None), "LEFT")


class TestParkingManeuver(unittest.TestCase):
    def test_does_not_move_before_begin(self):
        park = ParkingManeuver()
        speed, steer, action = park.step(0.1, 0.0, FAR, FAR)
        self.assertEqual((speed, steer, action), (0, 0, "STOP"))

    def test_begin_is_idempotent(self):
        park = ParkingManeuver()
        park.begin("LEFT", 0.0)
        park.begin("RIGHT", 90.0)  # should be ignored
        self.assertEqual(park._park_side, "LEFT")
        self.assertEqual(park._ref_heading, 0.0)

    def test_settle_then_reverses(self):
        park = ParkingManeuver()
        park.begin("LEFT", 0.0)
        # During SETTLE the car is stopped.
        speed, steer, action = park.step(PARK_SETTLE_S / 2, 0.0, FAR, FAR)
        self.assertEqual(action, "STOP")
        self.assertEqual(park.stage_name, "SETTLE")
        # After the settle time it advances into REVERSE_IN on the next step.
        park.step(PARK_SETTLE_S, 0.0, FAR, FAR)
        self.assertEqual(park.stage_name, "REVERSE_IN")

    def test_left_park_reverses_with_left_lock_then_right_lock(self):
        park = ParkingManeuver()
        park.begin("LEFT", 0.0)
        cmds = _drive_to_completion(park)  # heading held -> pure timing
        actions = [a for _, _, a in cmds]
        self.assertIn("BACKWARD", actions)
        self.assertIn("FORWARD", actions)   # final settle nudge
        self.assertTrue(park.done)
        # Reversing into a LEFT spot cuts the wheels left first (negative steer).
        reverse_steers = [s for _, s, a in cmds if a == "BACKWARD"]
        self.assertIn(-PARK_STEER_DEG, reverse_steers)
        self.assertIn(PARK_STEER_DEG, reverse_steers)   # straighten stage flips lock

    def test_right_park_mirrors_the_steer_signs(self):
        park = ParkingManeuver()
        park.begin("RIGHT", 0.0)
        cmds = _drive_to_completion(park)
        reverse_steers = [s for _, s, a in cmds if a == "BACKWARD"]
        # Mirror of the LEFT case: first lock is right (+), straighten is left (-).
        self.assertIn(PARK_STEER_DEG, reverse_steers)
        self.assertIn(-PARK_STEER_DEG, reverse_steers)

    def test_completes_and_holds_stop(self):
        park = ParkingManeuver()
        park.begin("LEFT", 0.0)
        _drive_to_completion(park)
        self.assertTrue(park.done)
        self.assertEqual(park.step(0.1, 0.0, FAR, FAR), (0, 0, "STOP"))

    def test_reset_allows_reuse(self):
        park = ParkingManeuver()
        park.begin("LEFT", 0.0)
        _drive_to_completion(park)
        self.assertTrue(park.done)
        park.reset()
        self.assertFalse(park.done)
        park.begin("RIGHT", 10.0)
        self.assertEqual(park._park_side, "RIGHT")


if __name__ == "__main__":
    unittest.main()
