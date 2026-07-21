#pragma once

// I2C Configuration
const int IMU_I2C_ADDRESS = 0x6A;
//const int TOF_I2C_ADDRESS = 0x7E;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

const int IMU_SAMPLE_RATE_HZ = 104;

// ---- ToF (VL53L1X x2) ------------------------------------------------
// VL53L1X boots at a fixed default address (0x29) with no way to strap a
// different one in hardware, so two on one bus need the standard XSHUT
// dance: hold both in reset, bring the LEFT sensor up alone and re-address
// it off 0x29, THEN bring the RIGHT sensor up (which is free to stay at
// the now-vacated default). See firmware/esp_controller/tof.cpp.
const int PIN_TOF_LEFT_XSHUT = 35;
const int PIN_TOF_RIGHT_XSHUT = 33;
const int TOF_DEFAULT_I2C_ADDRESS = 0x29;   // VL53L1X power-on default
const int TOF_LEFT_I2C_ADDRESS = 0x30;      // re-addressed at boot, see tof.cpp
const int TOF_RIGHT_I2C_ADDRESS = TOF_DEFAULT_I2C_ADDRESS;

// Documents the sensor's expected ranging cycle time; not written to the
// sensor directly (kept out of the timing-budget API to avoid depending on
// a specific Pololu VL53L1X library version) -- it just explains why
// TELEMETRY_INTERVAL below isn't tighter than this.
const int TOF_TIMING_BUDGET_MS = 20;

// Motor pins
const int PIN_MOTOR_IN1 = 18;
const int PIN_MOTOR_IN2 = 19;
const int PIN_MOTOR_PWM = 23;  // ena

const int MOTOR_PWM_CHANNEL = 0; // unused in 3.0, apparently ledcAttach() uses the pin number directly now
const int MOTOR_PWM_FREQ = 1000;
const int MOTOR_PWM_RESOLUTION = 8;

// Steering servo
const int PIN_STEERING_SERVO = 14;
const int SERVO_CENTER = 32;
const int SERVO_LEFT = 62;
const int SERVO_RIGHT = 2;

// Start button
const int PIN_START_BUTTON = 2;

// LED (status indicator)
const int PIN_LED = 25;

// serial
const int long BAUD_RATE = 115200;

// telemetry
const unsigned long TELEMETRY_INTERVAL = 100;

// IMU mounting: flip this to -1 on the bench if a clockwise (rightward) spin
// of the chassis produces a DECREASING gz reading instead of increasing --
// the Pi integrates heading += gz*dt and expects +gz = turning right, to
// match the steer sign convention used everywhere else (+ = right).
const int IMU_GZ_SIGN = 1;