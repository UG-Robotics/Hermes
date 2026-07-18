# Thresholds for lane (wall) detection, TOF wall-safety, and speed scaling.
#
# Ties back to the WRO 2026 Future Engineers field spec (game field + rules
# PDFs): the track surface is white, both the outer perimeter wall and the
# movable inner walls are black and 100mm tall, and there are NO painted
# centre lane lines on the mat -- only 20mm orange/blue corner-vs-straight
# markers on the floor, which perception/track_detection.py intentionally
# ignores. So "the lane" is the free corridor between whichever walls
# currently bound the track at the robot's position, and "lane detection"
# here means finding where those black walls start in the camera frame, not
# following a painted centre line.
#
# Corridor width is 600-1000mm (+/-100mm) in the Open Challenge and a fixed
# 1000mm (+/-10mm) in the Obstacle Challenge (see rules doc section 8).

from __future__ import annotations

# --------------------------------------------------------------------- lane
# ROI: a horizontal band of the frame, expressed as fractions of frame
# height, where we look for wall pixels. Starts below the horizon (top of
# the corridor recedes into the distance -> noisy/irrelevant) and stops
# short of the very bottom (robot's own chassis/bumper can appear there).
LANE_ROI_TOP_FRAC = 0.45
LANE_ROI_BOTTOM_FRAC = 0.95

# HSV upper bounds for "this pixel is wall, not track floor". Black has no
# meaningful hue, so only S (saturation) and V (value/brightness) are used;
# hue is left unconstrained (0..179).
LANE_BLACK_S_MAX = 90
LANE_BLACK_V_MAX = 70

# Column-count smoothing: average adjacent columns' black-pixel counts with
# a rolling window this wide before thresholding, so a single noisy column
# doesn't flip a wall edge on/off frame-to-frame.
LANE_SMOOTH_KERNEL = 5

# A column counts as "wall" once at least this fraction of the ROI band's
# height is black. 0.35 tolerates a wall that's partially occluded/angled
# without false-triggering on shadows or the odd stray dark pixel.
LANE_MIN_WALL_FRACTION = 0.35

# Cap how far from image centre we'll ever report a corridor-centre offset,
# as a fraction of the half-frame-width. Keeps a single bad detection from
# demanding a wild nudge.
LANE_MAX_OFFSET_FRAC = 0.9

# When only ONE wall is visible (very common: 1000mm corridors and corners
# routinely put the far wall out of frame), assume the corridor centre is
# this many pixels in from that wall. This is a rough, uncalibrated guess
# proportional to a typical ~1000mm corridor at FRAME_WIDTH=320 -- retune
# on the mat alongside CAMERA_FOCAL_PX in camera_config.py.
LANE_DEFAULT_HALF_WIDTH_PX = 110

# ---------------------------------------------------------- lane centering
# Proportional gain: pixels of corridor-centre offset -> degrees of heading
# nudge. Deliberately gentle -- this runs every tick as a continuous trim on
# top of the heading-hold PID (see planning/lane_centering.py), not a single
# corrective manoeuvre like the pillar-avoidance turn_by() angle.
LANE_STEER_KP = 0.06

# Per-tick nudge cap, in degrees. Bounds how fast the heading-hold PID's
# locked target can be dragged around by vision alone.
LANE_MAX_NUDGE_DEG = 4.0

# Ignore observations below this confidence (see LaneObservation.confidence
# in perception/track_detection.py) -- e.g. a single-wall guess right after
# a corner, before we trust it enough to steer off of it.
LANE_MIN_CONFIDENCE = 0.45

# Deadband, in pixels: offsets smaller than this are treated as "centred
# enough", so the loop doesn't hunt over sub-pixel noise.
LANE_OFFSET_DEADBAND_PX = 8

# --------------------------------------------------------------- TOF walls
# Both ToF sensors live on the ESP32 (firmware/esp_controller/tof.cpp) and
# are read Pi-side as a pure pass-through (hardware/tof.py). These
# thresholds are what turn "a wall got close" into a driving decision in
# planning/obstacle_planner.py.
#
# Matches hardware/tof.py's ToFArray.OUT_OF_RANGE_MM -- readings at/above
# this are treated as "no wall in range", not "wall is 2m away".
TOF_MAX_VALID_MM = 2000.0

# Below this: worth being cautious (soften avoidance turns, ease off speed).
TOF_WALL_WARNING_MM = 150.0

# Below this: imminent contact risk -- cap avoidance steer hard and nudge
# away from the wall regardless of what the pillar-avoidance angle wanted.
TOF_WALL_CRITICAL_MM = 80.0

# When AVOID_OBSTACLE's steer angle needs capping because the wall we're
# steering toward is within TOF_WALL_WARNING_MM, this is the ceiling
# (degrees) it gets capped to instead of PILLAR_MAX_STEER_DEG.
AVOIDANCE_STEER_CAP_NEAR_WALL_DEG = 35

# Extra one-tick heading nudge (degrees) away from a critically-close wall
# during avoidance, applied via SteeringController.nudge_target() alongside
# the (now-capped) turn_by() target -- see runtime.py.
AVOIDANCE_WALL_NUDGE_DEG = 8.0

# Nominal corridor widths from the rules doc (section 8) -- kept here as
# reference constants for tuning/telemetry, not consumed by a formula yet.
CORRIDOR_WIDTH_OPEN_MM = 800.0       # open challenge: 600-1000mm (+/-100mm)
CORRIDOR_WIDTH_OBSTACLE_MM = 1000.0  # obstacle challenge: fixed 1000mm (+/-10mm)

# ------------------------------------------------------------------- speed
# Multiplier floor: speed is never scaled down past this, however close the
# walls get -- we want caution, not a stall mid-corridor.
SPEED_SCALE_MIN = 0.5

SPEED_SCALE_WALL_WARNING = 0.75
SPEED_SCALE_WALL_CRITICAL = 0.5

# --------------------------------------------------------------- lap count
# The FE track is 4 corner sections + 4 straight sections (rules doc,
# field layout) -- one full lap is 4 corner-marker traversals. Used by
# runtime.py's _post_event_effects to turn perception/corner_detection.py's
# per-corner events into a real lap count.
CORNERS_PER_LAP = 4
