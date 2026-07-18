import os

# ---------------------------------------------------------------------------
# Simulation / hardware substitution
# ---------------------------------------------------------------------------
# When SIMULATION is True the software runs against simulated hardware:
#   * the serial link talks to a virtual ESP32 instead of a real one, and
#   * the camera produces synthetic frames instead of reading a Pi Camera.
# Everything else (logging, state machine, dashboard) behaves identically, so
# the whole stack can be exercised on a laptop with no robot attached.
#
# Toggle without editing code:  HERMES_SIM=1 python main.py --mode run
# The --mode sim / --dashboard entry points force this on regardless.
SIMULATION = os.environ.get("HERMES_SIM", "0").lower() in ("1", "true", "yes", "on")

# Serial Communication Settings
# /dev/serial0 is the Raspberry Pi's stable alias for the UART header; the
# runtime also falls back to common USB/ACM device names if that alias is not
# the active one on the current setup.
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_PORT_FALLBACKS = ["/dev/serial0", "/dev/ttyACM0", "/dev/ttyAMA0", "/dev/ttyS0"]
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # seconds

# Motor Control Constants
SPEED_MIN = 0
SPEED_MAX = 255
SPEED_DEFAULT_FORWARD = 150
SPEED_DEFAULT_BACKWARD = 100
SPEED_STOP = 0

# Steering Constants (MG90S Servo Calibration)
# STEER_MIN/STEER_MAX cap the autonomous PID's correction authority (it should
# never be able to slam the servo to full lock on its own -- see
# control/steering_control.py). They are NOT the wire-protocol range -- see
# STEER_COMMAND_MIN/MAX below for that.
STEER_MIN = -45
STEER_MAX = 45
STEER_CENTER_DEGREE = 32  # 90 degrees is the center calibration for the servo

# Full command range the ESP32 actually expects on the wire (esp_controller.ino
# maps this -90..90 onto SERVO_LEFT..SERVO_RIGHT). Used by protocol.py so
# manual driving -- and anything else that wants full lock -- isn't silently
# clamped down to the PID's half-authority STEER_MIN/STEER_MAX above.
STEER_COMMAND_MIN = -90
STEER_COMMAND_MAX = 90

# Keyboard Mapping (laptop keyboard manual driving)
KEY_TOGGLE_MANUAL = 'm'   # toggle manual <-> autonomous
KEY_FORWARD = 'w'         # drive forward (hold)
KEY_BACKWARD = 's'        # drive backward (hold)
KEY_LEFT = 'a'            # steer left (hold)
KEY_RIGHT = 'd'           # steer right (hold)
KEY_STOP = ' '            # spacebar: immediate stop / straighten

# How hard manual steering turns, in the same -90..90 units as
# STEER_COMMAND_MIN/MAX (full lock -- manual is operator-controlled, so there's
# no reason to cap it below the PID's correction-authority limit).
STEER_MANUAL_DEGREE = 90

# Arduino Pin Mappings (Hardcoded compliance on the board)
PIN_START_BUTTON = 2

# Rear Driving Motor (H-Bridge L298N pins)
PIN_MOTOR_PWM = 23
PIN_MOTOR_FORWARD = 18
PIN_MOTOR_BACKWARD = 19

# Front Steering Servo
PIN_STEERING_SERVO = 14
# ESP32 Sensor Telemetry
IMU_UPDATE_INTERVAL_MS = 20
TOF_UPDATE_INTERVAL_MS = 20

IMU_PACKET_PREFIX = "IMU"
TOF_PACKET_PREFIX = "TOF"