#include <Arduino.h>

#include "config.h"
#include "servo_control.h"

namespace
{
    constexpr int SERVO_PWM_FREQUENCY = 50;
    constexpr int SERVO_PWM_RESOLUTION = 16;
    constexpr int SERVO_MIN_PULSE_US = 1000;
    constexpr int SERVO_MAX_PULSE_US = 2000;

    int currentSteeringAngle = SERVO_CENTER;

    int angleToDuty(int angleDegrees)
    {
        angleDegrees = constrain(angleDegrees, 0, 180);
        int pulseWidthUs = map(angleDegrees, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        long maxDuty = (1L << SERVO_PWM_RESOLUTION) - 1;
        return static_cast<int>((pulseWidthUs * maxDuty) / 20000L);
    }
}

void initServo()
{
    ledcAttach(PIN_STEERING_SERVO, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);
    setSteeringAngle(SERVO_CENTER);
}

void setSteeringAngle(int angleDegrees)
{
    currentSteeringAngle = constrain(angleDegrees, SERVO_LEFT, SERVO_RIGHT);
    ledcWrite(PIN_STEERING_SERVO, angleToDuty(currentSteeringAngle));
}

void updateServo()
{
    (void)currentSteeringAngle;
}