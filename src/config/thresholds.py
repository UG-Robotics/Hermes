# Thresholds for ToF corridor centering, TOF wall-safety, and speed scaling.
#
# Ties back to the WRO 2026 Future Engineers field spec (game field + rules
# PDFs): the track surface is white, both the outer perimeter wall and the
# movable inner walls are black and 100mm tall. Corridor centering is done
# from the two SIDE ToF sensors + the IMU (see planning/wall_centering.py),
# NOT the camera -- the forward camera's FOV is too narrow to see both walls
# reliably, so it's left to spot the coloured pillars, the 20mm orange/blue
# corner markers, and the magenta parking zone. "The lane" is the free
# corridor between whichever walls currently bound the track at the robot's
# position; "centering" means equalising the two side ToF distances.
#
# Corridor width is 600-1000mm (+/-100mm) in the Open Challenge and a fixed
# 1000mm (+/-10mm) in the Obstacle Challenge (see rules doc section 8).

from __future__ import annotations

# ----------------------------------------------------- ToF corridor centering
# All distances are millimetres, matching hardware/tof.py's readings. See
# planning/wall_centering.py for how these turn two side distances into a
# per-tick heading nudge on top of the IMU heading-hold loop.

# A side reading at/above this is treated as "no wall on this side" (the far
# side of a wide corridor, a corner opening, or the OUT_OF_RANGE_MM sentinel),
# not a real wall to centre against. Must exceed the widest half-corridor the
# far wall can sit at when the car is hard against the near wall (~1000mm), so
# both walls still register when off-centre.
TOF_WALL_SEEN_MAX_MM = 1200.0

# Proportional gain: degrees of heading nudge per mm of lateral offset from
# the corridor centre. Deliberately gentle -- this runs every tick as a
# continuous trim on top of the heading-hold PID (planning/wall_centering.py),
# not a single corrective manoeuvre like the pillar-avoidance turn_by() angle.
# At 0.05, an 80mm offset saturates the per-tick cap below.
TOF_CENTER_KP = 0.05

# Deadband, in mm: lateral offsets smaller than this are "centred enough", so
# the loop doesn't hunt over ToF noise / a slightly asymmetric mount.
TOF_CENTER_DEADBAND_MM = 20.0

# Per-tick nudge cap, in degrees. Bounds how fast the heading-hold PID's
# locked target can be dragged around by the ToF centering loop alone.
TOF_CENTER_MAX_NUDGE_DEG = 4.0

# Target clearance to hold to the single visible wall when only ONE wall is in
# range (wall-following at corners / open edges). ~Half a nominal 1000mm
# corridor. Retune on the mat next to TOF_WALL_SEEN_MAX_MM.
TOF_WALL_FOLLOW_TARGET_MM = 500.0

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

# Sensor cross-check: two OPPOSITE side walls cannot both be closer than this
# combined -- they'd bound a corridor narrower than the 600mm minimum (rules
# doc section 8), even for a zero-width robot. So when both ToFs are in range
# but sum below this, at least one is lying/stuck, and a "critically close"
# reading from the pair is treated as spurious rather than a real wall (see
# planning/obstacle_planner._spurious_close): the car won't brake or cap
# steering on a phantom. Deliberately well below the real minimum clearance so
# a genuine close wall is NEVER suppressed. At a corner (one side out of range)
# there's nothing to cross-check, so a close reading is taken at face value.
TOF_MIN_PLAUSIBLE_SUM_MM = 300.0

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

# Total laps required to finish either challenge (rules doc, section 6/7).
TARGET_LAPS = 3

# ---------------------------------------------------- challenge separation
# WRO FE allows exactly ONE start button (rules doc, section 5), so the car
# can't be told which challenge it's running -- it has to work it out itself.
# The rule Hermes uses: a red/green traffic-sign pillar is UNIQUE to the
# Obstacle Challenge (the Open Challenge mat has none), so the first pillar
# ever seen latches challenge_mode = "OBSTACLE" (runtime.py's
# _post_event_effects). If a full lap is completed without ever seeing one,
# it's latched "OPEN" instead. See RobotContext.challenge_mode.
#
# Number of completed corners after which, if no pillar has been seen, the
# run is declared OPEN. One full lap (CORNERS_PER_LAP) is the natural point:
# by then any obstacle-challenge run would almost certainly have shown a
# pillar, and we still have two laps left to act on the OPEN decision
# (inner-wall bias, perception gating).
OPEN_DECISION_AFTER_CORNERS = CORNERS_PER_LAP

# ---------------------------------------------------------- inner-wall bias
# OPEN-challenge only: once we know the run direction (from the first corner
# marker's colour) we also know which side the INNER wall is on -- CLOCKWISE
# hugs the RIGHT wall, COUNTER_CLOCKWISE the LEFT. Biasing the ToF-centering
# target a little toward that inner wall makes the racing line tighter and,
# more importantly, keeps the car reliably away from the outer wall on the
# wide/variable open-challenge corridor. Obstacle runs are NEVER biased (the
# car must stay centred to pass pillars on either side).
#
# Millimetres to shift the target lateral position toward the inner wall (in
# planning/wall_centering.py's BOTH-walls mode). 0 disables the bias entirely.
# Retune on the mat. Applied with +sign toward the RIGHT wall, matching
# CenteringObservation.offset_mm's convention.
INNER_WALL_BIAS_MM = 120.0

# ------------------------------------------------------------- start delay
# Seconds to stay stopped in FOLLOW_TRACK after START_BUTTON_PRESSED before
# the car actually rolls, so the person pressing the button can withdraw
# their hand first (a real, if minor, WRO scoring/interference concern).
# runtime.py enforces this; drive_command stays a pure state->command map.
# Set to 0.0 to launch instantly.
START_MOVE_DELAY_S = 0.75

# ---------------------------------------------------------- open finish
# OPEN challenge has no magenta parking lot to aim at -- the finish line is
# just a POSITION on the start straight (rules doc: "on the extension of the
# starting line"). After the 3rd lap completes the car enters FINAL_APPROACH
# and drives straight for this long to clear the finish line, then stops.
# Purely time-based because there is no encoder/odometry on this platform
# (only IMU + side ToF) -- retune on the mat so the car halts just inside the
# start section.
FINAL_APPROACH_OPEN_DURATION_S = 1.2

# ------------------------------------------------------ sensor-health warnings
# runtime.py logs a WARNING (to logs/hermes.log + dashboard) when the ESP32's
# telemetry that carries the IMU + ToF readings goes stale, and when the ToF
# sensors stop producing valid ranges. Both are latched (one line down, one on
# recovery) so a persistent fault doesn't spam at the loop rate.
#
# TELEMETRY_STALE_S: how long with NO new TEL packet before we call the whole
# IMU+ToF stream dead. The ESP sends telemetry every TELEMETRY_INTERVAL (100ms
# / ~10 Hz in config.h), so this is several missed packets, not a hiccup.
TELEMETRY_STALE_S = 0.75
# TOF_DEAD_S: how long BOTH ToF sensors must continuously read the out-of-range
# sentinel (while telemetry is otherwise flowing) before we warn they've stopped
# ranging. Sustained on purpose so briefly facing open space (both walls far)
# doesn't trip it -- only sensors that actually dropped off the bus do.
TOF_DEAD_S = 2.0
