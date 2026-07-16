# PID gains for the IMU heading-hold controller (control/steering_control.py).
#
# This is the loop that keeps the bot pointed the right way: it compares the
# IMU's integrated heading against a target heading (locked to "go straight"
# in FOLLOW_TRACK, or offset by the avoidance angle in AVOID_OBSTACLE) and
# outputs a steering correction in degrees, added on top of whatever the
# planner asked for.

# --- heading-hold PID (output: steering correction, degrees) --------------
STEER_PID_KP = 1.6
STEER_PID_KI = 0.02
STEER_PID_KD = 0.35

# Correction authority: the PID alone should never be able to slam the servo
# to full lock — it's a *correction*, not the whole steering command.
STEER_PID_OUTPUT_MIN = -45.0
STEER_PID_OUTPUT_MAX = 45.0

# Anti-windup clamp on the accumulated integral term (degrees * seconds).
STEER_PID_INTEGRAL_MIN = -20.0
STEER_PID_INTEGRAL_MAX = 20.0

# --- servo smoothing --------------------------------------------------------
# Maximum change in commanded steering angle per control tick (20 Hz loop),
# in degrees. Keeps corrections from snapping the servo — "smooth" per spec.
STEER_SLEW_MAX_DEG_PER_TICK = 12.0
