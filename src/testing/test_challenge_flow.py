"""
Integration tests for the run-level behaviour that lives in runtime.py:
single-button challenge auto-detection, corner/lap counting (the double-count
fix), and reaching a proper finish in both challenges.

These build a real Runtime against simulated hardware (no camera, no
keyboard) and drive its state machine by pushing the same events the vision
pipeline would raise, then draining them exactly like tick() does. That
exercises runtime._post_event_effects / _update_final_approach /
_resolve_parking end to end without needing real time or real sensors.
"""

import unittest

from runtime import Runtime
from state_machine.states import State
from state_machine.events import EventType, make_event
from config.thresholds import CORNERS_PER_LAP, TARGET_LAPS


def _headless_runtime():
    return Runtime(simulated=True, use_keyboard=False, use_camera=False, auto_start=False)


def _start(rt):
    rt.inject_event("START_BUTTON_PRESSED")
    rt._drain_events()


def _corners(rt, n, color="ORANGE"):
    """Simulate n full corner traversals: each corner raises LAP_MARKER_DETECTED
    twice (appear -> FOLLOW_TRACK->LAP_CHECK, disappear -> LAP_CHECK->FOLLOW_TRACK),
    mirroring perception/corner_detection.py. Drained in one batch."""
    for _ in range(n):
        rt._events.push(make_event(EventType.LAP_MARKER_DETECTED, metadata={"color": color}))
        rt._events.push(make_event(EventType.LAP_MARKER_DETECTED, metadata={"color": color}))
    rt._drain_events()


class TestLapCounting(unittest.TestCase):
    def test_starts_in_follow_track_after_button(self):
        rt = _headless_runtime()
        self.assertEqual(rt.state_machine.current_state, State.WAIT_FOR_START)
        _start(rt)
        self.assertEqual(rt.state_machine.current_state, State.FOLLOW_TRACK)

    def test_corner_is_not_double_counted(self):
        rt = _headless_runtime()
        _start(rt)
        # Two physical corners must NOT already be a lap (the old bug counted
        # both the entry and exit edge, so 2 corners == 1 "lap").
        _corners(rt, 2)
        self.assertEqual(rt.state_machine.context.corners_passed, 2)
        self.assertEqual(rt.state_machine.context.lap_count, 0)

    def test_one_lap_is_exactly_four_corners(self):
        rt = _headless_runtime()
        _start(rt)
        _corners(rt, CORNERS_PER_LAP)
        self.assertEqual(rt.state_machine.context.lap_count, 1)

    def test_three_laps_is_twelve_corners_and_enters_final_approach(self):
        rt = _headless_runtime()
        _start(rt)
        _corners(rt, CORNERS_PER_LAP * TARGET_LAPS)
        self.assertEqual(rt.state_machine.context.lap_count, TARGET_LAPS)
        # THREE_LAPS_COMPLETE was queued during that drain; process it.
        rt._drain_events()
        self.assertEqual(rt.state_machine.current_state, State.FINAL_APPROACH)

    def test_direction_latched_from_first_corner_colour(self):
        rt = _headless_runtime()
        _start(rt)
        _corners(rt, 1, color="BLUE")
        self.assertEqual(rt.state_machine.context.race_direction, "COUNTER_CLOCKWISE")


class TestChallengeDetection(unittest.TestCase):
    def test_pillar_marks_obstacle_challenge(self):
        rt = _headless_runtime()
        _start(rt)
        rt._events.push(make_event(EventType.PILLAR_DETECTED_RED, metadata={"steer_angle": 30}))
        rt._drain_events()
        self.assertEqual(rt.state_machine.context.challenge_mode, "OBSTACLE")
        self.assertTrue(rt.state_machine.context.pillar_ever_seen)

    def test_full_lap_without_pillar_marks_open_challenge(self):
        rt = _headless_runtime()
        _start(rt)
        self.assertIsNone(rt.state_machine.context.challenge_mode)
        _corners(rt, CORNERS_PER_LAP)
        self.assertEqual(rt.state_machine.context.challenge_mode, "OPEN")


class TestTermination(unittest.TestCase):
    def test_open_run_finishes_via_timer_to_stop(self):
        rt = _headless_runtime()
        _start(rt)
        _corners(rt, CORNERS_PER_LAP * TARGET_LAPS)
        rt._drain_events()  # THREE_LAPS_COMPLETE -> FINAL_APPROACH
        self.assertEqual(rt.state_machine.current_state, State.FINAL_APPROACH)
        self.assertEqual(rt.state_machine.context.challenge_mode, "OPEN")

        # Force the finish clock to have elapsed, then run the finish check.
        rt._final_approach_ts -= 100.0
        rt._update_final_approach()      # raises FINISH_ZONE_DETECTED
        rt._drain_events()               # FINAL_APPROACH -> PARK
        self.assertEqual(rt.state_machine.current_state, State.PARK)

        # PARK for an OPEN run just stops and raises PARKING_COMPLETE.
        rt._resolve_parking(0.1, rt.state_machine.context)
        rt._drain_events()               # PARK -> STOP
        self.assertEqual(rt.state_machine.current_state, State.STOP)

    def test_obstacle_run_parks_to_stop(self):
        rt = _headless_runtime()
        _start(rt)
        # Make it an obstacle run.
        rt._events.push(make_event(EventType.PILLAR_DETECTED_RED, metadata={"steer_angle": 30}))
        rt._drain_events()
        rt._events.push(make_event(EventType.OBSTACLE_CLEARED))
        rt._drain_events()
        _corners(rt, CORNERS_PER_LAP * TARGET_LAPS)
        rt._drain_events()  # -> FINAL_APPROACH
        self.assertEqual(rt.state_machine.current_state, State.FINAL_APPROACH)

        # Magenta parking zone confirmed -> PARK.
        rt._events.push(make_event(EventType.PARKING_ZONE_DETECTED, metadata={"cx": 160, "area": 500}))
        rt._drain_events()
        self.assertEqual(rt.state_machine.current_state, State.PARK)

        # Run the parking maneuver to completion (fixed dt, walls far away).
        for _ in range(400):
            rt._resolve_parking(0.1, rt.state_machine.context)
            if rt.parking.done:
                break
        self.assertTrue(rt.parking.done)
        rt._drain_events()  # PARKING_COMPLETE -> STOP
        self.assertEqual(rt.state_machine.current_state, State.STOP)


if __name__ == "__main__":
    unittest.main()
