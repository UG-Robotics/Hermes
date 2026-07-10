#pragma once

// I2C Configuration
const int IMU_I2C_ADDRESS = 0x6A;
const int TOF_I2C_ADDRESS = 0x7E;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

const int IMU_SAMPLE_RATE_HZ = 104;
const int TOF_TIMING_BUDGET_MS = 20;

// Motor pins
const int PIN_MOTOR_IN1 = 18;
const int PIN_MOTOR_IN2 = 19;
const int PIN_MOTOR_PWM = 23;  // ena

const int MOTOR_PWM_CHANNEL = 0; // unused in 3.0, apparently ledcAttach() uses the pin number directly now
const int MOTOR_PWM_FREQ = 1000;
const int MOTOR_PWM_RESOLUTION = 8;

// Steering servo
const int PIN_STEERING_SERVO = 12;
const int SERVO_CENTER = 90;
const int SERVO_LEFT = 45;
const int SERVO_RIGHT = 135;

// Start button
const int PIN_START_BUTTON = 2;

// LED (status indicator)
const int PIN_LED = 25;

// serial
const int long BAUD_RATE = 115200;

// telemetry
const unsigned long TELEMETRY_INTERVAL = 100;