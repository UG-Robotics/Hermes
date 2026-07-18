import unittest

try:
    import numpy as np
    _HAVE_NUMPY = True
except Exception:
    _HAVE_NUMPY = False

from perception.finish_detection import ParkingZoneTracker
from config.camera_config import PARKING_CONFIRM_FRAMES

FRAME_WIDTH = 320
FRAME_HEIGHT = 240


def _magenta_frame():
    frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 255, dtype=np.uint8)
    frame[120:200, 80:240, :] = (230, 0, 200)  # magenta-ish patch
    return frame


def _blank_frame():
    return np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 255, dtype=np.uint8)


@unittest.skipUnless(_HAVE_NUMPY, "numpy not installed")
class TestParkingZoneTracker(unittest.TestCase):
    def test_no_marker_never_confirms(self):
        tracker = ParkingZoneTracker()
        for _ in range(PARKING_CONFIRM_FRAMES + 2):
            obs = tracker.update(_blank_frame())
        self.assertFalse(obs.confirmed)

    def test_confirms_after_enough_consecutive_sightings(self):
        tracker = ParkingZoneTracker()
        obs = None
        for _ in range(PARKING_CONFIRM_FRAMES):
            obs = tracker.update(_magenta_frame())
        self.assertTrue(obs.confirmed)

    def test_not_confirmed_before_threshold_reached(self):
        tracker = ParkingZoneTracker()
        obs = None
        for _ in range(PARKING_CONFIRM_FRAMES - 1):
            obs = tracker.update(_magenta_frame())
        self.assertFalse(obs.confirmed)
        self.assertTrue(obs.present)

    def test_latches_and_does_not_refire(self):
        tracker = ParkingZoneTracker()
        for _ in range(PARKING_CONFIRM_FRAMES):
            obs = tracker.update(_magenta_frame())
        self.assertTrue(obs.confirmed)
        obs2 = tracker.update(_magenta_frame())
        self.assertFalse(obs2.present)
        self.assertFalse(obs2.confirmed)

    def test_dropped_frame_resets_the_streak(self):
        tracker = ParkingZoneTracker()
        for _ in range(PARKING_CONFIRM_FRAMES - 1):
            tracker.update(_magenta_frame())
        tracker.update(_blank_frame())  # streak broken
        obs = tracker.update(_magenta_frame())
        self.assertFalse(obs.confirmed)  # streak restarted, only 1 hit so far


if __name__ == "__main__":
    unittest.main()
