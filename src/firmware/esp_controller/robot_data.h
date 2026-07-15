#pragma once

#include <Arduino.h>

struct RobotCommand
{
    int speed = 0;
    int steer = 0;
    String action = "STOP";
    int mode = 1;
    bool valid = false;
};

struct RobotTelemetry
{
    float ax = 0;
    float ay = 0;
    float az = 0;
    float gx = 0;
    float gy = 0;
    float gz = 0;
    float tof1_mm = 0;
    float tof2_mm = 0;
};