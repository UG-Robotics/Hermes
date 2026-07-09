# Serial Communication Settings
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # seconds

# Motor Control Constants
SPEED_MIN = 0
SPEED_MAX = 255
SPEED_DEFAULT_FORWARD = 150
SPEED_DEFAULT_BACKWARD = 100
SPEED_STOP = 0

# Steering Constants (MG90S Servo Calibration)
STEER_MIN = -90
STEER_MAX = 90
STEER_CENTER_DEGREE = 90  # 90 degrees is the center calibration for the servo

# Keyboard Mapping
KEY_TOGGLE_MANUAL = 'm'
KEY_FORWARD = 'w'
KEY_BACKWARD = 's'

# Arduino Pin Mappings (Hardcoded compliance on the board)
PIN_START_BUTTON = 2

# Rear Driving Motor (H-Bridge L298N pins)
PIN_MOTOR_PWM = 5
PIN_MOTOR_FORWARD = 3
PIN_MOTOR_BACKWARD = 4

# Front Steering Servo
PIN_STEERING_SERVO = 6

# ESP32 Sensor Telemetry
IMU_UPDATE_INTERVAL_MS = 20
TOF_UPDATE_INTERVAL_MS = 20

IMU_PACKET_PREFIX = "IMU"
TOF_PACKET_PREFIX = "TOF"