"""
Region-of-Interest (ROI) helpers.

The pillar/corner/parking detectors currently search the whole frame rather
than a cropped ROI. These helpers are the shared, reusable version of a
fraction -> pixel-slice crop, so a module that later wants to restrict
detection to a band of the frame (e.g. only look for corner markers in the
lower half where the floor is) doesn't re-derive the arithmetic. Pure indexing
over a native RGB numpy array -- no OpenCV dependency.
"""

from __future__ import annotations


def horizontal_band(frame, top_frac: float, bottom_frac: float):
    """Return the horizontal slice of `frame` between top_frac and bottom_frac
    of its height (both in [0, 1]). Returns None for an empty/degenerate band
    so callers can null-check exactly like they already null-check a missing
    camera frame."""
    if frame is None:
        return None
    height = frame.shape[0]
    y0 = int(height * top_frac)
    y1 = int(height * bottom_frac)
    if y1 <= y0:
        return None
    return frame[y0:y1, :, :]


def center_column_band(frame, width_frac: float = 0.5):
    """Return the central vertical strip of `frame`, `width_frac` of the full
    width. Handy for 'is the path ahead clear' style checks that only care
    about what's directly in front of the car."""
    if frame is None:
        return None
    width = frame.shape[1]
    keep = max(1, int(width * width_frac))
    x0 = (width - keep) // 2
    return frame[:, x0:x0 + keep, :]
