#include <Arduino.h>

#include "motor.h"
#include "config.h"


int currentMotorSpeed = 0;

void initMotor()
{
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);

    ledcAttach(
        PIN_MOTOR_PWM,
        MOTOR_PWM_FREQ,
        MOTOR_PWM_RESOLUTION
    );

    setMotorSpeed(0);
}

void setMotorSpeed(int speed)
{
    speed = constrain(speed, -100, 100);

    currentMotorSpeed = speed;

    if (speed > 0)
    {
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
    }
    else
    {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, LOW);
    }

    int pwm = map(abs(speed), 0, 100, 0, 255);

    ledcWrite(
        PIN_MOTOR_PWM,
        pwm
    );
}

void updateMotor()
{
    // Reserved for acceleration limiting later.
}