#pragma once

struct RobotCommand
{
    int speed = 0;          // -100 to 100
    int steering = 0;       // -45 to +45 degrees from centre
};

struct RobotTelemetry
{
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;

    int leftToF = -1;
    int rightToF = -1;

    bool imuConnected = false;
};

extern RobotCommand command;
extern RobotTelemetry telemetry;