#include <Arduino.h>

#include "telemetry.h"
#include "config.h"

void sendTelemetry()
{
    static unsigned long lastSend = 0;

    if (millis() - lastSend >= TELEMETRY_INTERVAL)
    {
        lastSend = millis();

        Serial.println("STATUS,OK");
    }
}