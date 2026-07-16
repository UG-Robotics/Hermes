"""
Backward-compatible thin wrapper around perception.pillar_detection.

detect_pillars() is kept with its original signature/return contract
(color, cx) because testing/vision/pillar_vision_test.py already depends on
it for the standalone bench-test loop. The actual detection logic now lives
in one place, perception/pillar_detection.find_largest_pillar(), so the
runtime pipeline and this standalone tester can never drift apart.
"""

from perception.pillar_detection import find_largest_pillar


def detect_pillars(frame_rgb):
    """
    Detects the closest red or green pillar using native RGB frames from Picamera2.
    Returns:
        tuple: (string 'RED' or 'GREEN' or None, integer 'cx' or None)
    """
    blob = find_largest_pillar(frame_rgb)
    if blob is None:
        return None, None
    return blob.color, blob.cx
