#include <Arduino.h>

#include "serial_protocol.h"
#include "robot_data.h"
#include "config.h"

RobotCommand command;
RobotTelemetry telemetry;

void initSerial()
{
    Serial.begin(BAUD_RATE);

    while (!Serial)
    {
        delay(10);
    }

    Serial.println("Serial Initialised");
}

void processSerial()
{
    while (Serial.available())
    {
        Serial.read();
    }
}

