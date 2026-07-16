#pragma once

#include "robot_data.h"

// 6-axis IMU driver (LSM6DS-family register map: LSM6DS3 / LSM6DSOX /
// ISM330DHCX are all register-compatible for the subset used here). Config.h
// already defines IMU_I2C_ADDRESS (0x6A) and IMU_SAMPLE_RATE_HZ (104).
//
// Fills the same RobotTelemetry struct that serial_protocol.cpp sends to the
// Pi as a TEL,... packet, so the Pi-side IMU (hardware/imu.py) starts seeing
// real accel/gyro data instead of the zeros it got before this existed.

bool initIMU();                          // returns false if WHO_AM_I doesn't match / bus error
void readIMU(RobotTelemetry &telemetry); // fills ax,ay,az (m/s^2) and gx,gy,gz (deg/s)
bool imuHealthy();
