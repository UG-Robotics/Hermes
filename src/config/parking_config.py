# Parallel-parking maneuver constants (planning/parking_planner.py).
#
# WRO 2026 Future Engineers obstacle challenge ends with a PARALLEL park into
# the magenta-walled parking lot on the outer wall of the start section (rules
# doc, section 7 + playfield PDF). This platform has NO wheel encoder or other
# odometry -- only the IMU (integrated heading) and the two side ToF sensors --
# so the maneuver is staged by TIME, with the IMU heading and side ToF used as
# secondary guards. Every duration/angle below is a starting point that MUST be
# tuned on the real mat with the real car; they are intentionally conservative
# (slower, wider) so the first on-mat runs fail safe rather than clipping a
# wall.
#
# The maneuver is a textbook reverse-in parallel park, mirrored to whichever
# side the parking lot is on (see PARK_SIDE_* below):
#
#   SETTLE          brief full stop so the car is stationary before reversing
#   REVERSE_IN      reverse with the wheels cut TOWARD the lot; the tail swings
#                   into the spot and the heading rotates ~PARK_TURN_IN_DEG
#   REVERSE_STRAIGHTEN
#                   reverse with the wheels cut the OTHER way to bring the
#                   heading back parallel to the wall, seating the car in the
#                   spot
#   FORWARD_SETTLE  small forward nudge, wheels centred, to sit off the rear
#                   wall and finish square
#   DONE            stopped; runtime raises PARKING_COMPLETE
#
# Steering sign convention matches everywhere else: + = right, - = left
# (config/robot_config.py). Speeds are 0..255 command units.

from __future__ import annotations

# --- which side the parking lot is on -------------------------------------
# The lot sits on the OUTER wall of the start section. The outer wall is on
# the car's LEFT when the run direction is CLOCKWISE (the car turns right each
# corner, so track-centre -- and the inner wall -- is on its right), and on
# the RIGHT when COUNTER_CLOCKWISE. runtime.py derives the park side from
# RobotContext.race_direction using this mapping, unless PARK_SIDE_OVERRIDE
# is set. If on-mat testing shows the lot is actually reached from the other
# side, flip PARK_SIDE_OVERRIDE rather than editing logic.
PARK_SIDE_FOR_CLOCKWISE = "LEFT"
PARK_SIDE_FOR_COUNTER_CLOCKWISE = "RIGHT"
# None = derive from race_direction (normal). "LEFT"/"RIGHT" = force that side.
PARK_SIDE_OVERRIDE = None

# --- stage speeds (0..255) -------------------------------------------------
PARK_REVERSE_SPEED = 110    # gentle reverse for both reversing stages
PARK_FORWARD_SPEED = 90     # even gentler forward settle

# --- stage steering magnitude (degrees, sign applied by park side) ---------
# How hard the wheels are cut during the two reversing stages. Near full lock
# so the car rotates into a tight spot within the stage durations below.
PARK_STEER_DEG = 70

# --- stage durations (seconds) ---------------------------------------------
# Tune these first on the mat -- they dominate where the car ends up.
PARK_SETTLE_S = 0.35
PARK_REVERSE_IN_S = 1.30
PARK_REVERSE_STRAIGHTEN_S = 1.20
PARK_FORWARD_SETTLE_S = 0.45

# --- IMU heading guards (degrees) ------------------------------------------
# Secondary completion checks layered on top of the timers so the maneuver
# adapts a little to battery sag / surface grip instead of trusting time
# alone. REVERSE_IN ends when EITHER its timer expires OR the heading has
# rotated at least this much from the pre-park heading; REVERSE_STRAIGHTEN
# ends when the heading has come back to within PARK_ALIGN_TOLERANCE_DEG of
# the original. Set PARK_USE_HEADING_GUARD = False to fall back to pure
# timing if the IMU proves unreliable in the spot.
PARK_USE_HEADING_GUARD = True
PARK_TURN_IN_DEG = 42.0
PARK_ALIGN_TOLERANCE_DEG = 10.0

# --- side ToF safety guard (mm) --------------------------------------------
# During reversing, if the ToF on the lot side reads closer than this the
# rear is about to touch the lot's back/side wall -- cut the current stage
# short. 0 disables the guard (e.g. if the ToF can't see the low lot walls).
PARK_MIN_SIDE_CLEARANCE_MM = 60.0
