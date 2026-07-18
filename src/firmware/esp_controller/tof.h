#pragma once

#include "robot_data.h"

// Dual VL53L0X time-of-flight distance sensor driver.
//
// OWNERSHIP: both ToF sensors are physical hardware wired to the ESP32
// (config.h: PIN_TOF_LEFT_XSHUT/PIN_TOF_RIGHT_XSHUT, shared I2C bus with the
// IMU). The Pi NEVER talks to these sensors directly -- see
// hardware/tof.py's module docstring on the Pi side, which only ever reads
// tof1_mm/tof2_mm back out of the TEL packet this file fills in
// RobotTelemetry, exactly like imu.h/imu.cpp does for accel/gyro.
//
// Convention (matches hardware/tof.py and communication/packet_parser.py's
// TEL_FIELD_NAMES): tof1_mm = LEFT sensor, tof2_mm = RIGHT sensor. If the
// physical wiring ever ends up mirrored, fix it in readTOF() below, not on
// the Pi side.
//
// Requires the "Adafruit VL53L0X" library (Library Manager) and its
// "Adafruit BusIO" dependency.

bool initTOF();                          // false if either sensor fails to come up on I2C
void readTOF(RobotTelemetry &telemetry); // fills tof1_mm (left) / tof2_mm (right)
bool tofHealthy();                       // true only once BOTH sensors initialised OK
