#include <Arduino.h>
#include <ESP32Servo.h>  // pulls in ESP32PWM

#include "motor.h"
#include "config.h"

// The ENA (speed) pin is driven through ESP32PWM -- the SAME LEDC wrapper the
// steering servo uses (servo_control.cpp / ESP32Servo). Previously this file
// used the core's raw ledcAttach()/ledcWrite() while the servo used ESP32Servo;
// the two allocate LEDC timers through different bookkeeping and were colliding,
// which left the ENA pin without a valid PWM signal -- so the motor sat dead in
// BOTH directions even though IN1/IN2 toggled correctly. Routing both PWMs
// through ESP32PWM with a dedicated timer each (servo = timer 0, reserved in
// initServo which runs first; motor = timer 1, reserved below) keeps them from
// ever sharing/reconfiguring the same timer.
namespace
{
    ESP32PWM motorPwm;
}

int currentMotorSpeed = 0;

void initMotor()
{
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);

    // Reserve a timer distinct from the servo's (timer 0) and attach the ENA
    // pin at the motor's own frequency/resolution.
    ESP32PWM::allocateTimer(1);
    motorPwm.attachPin(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

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

    // MOTOR_PWM_RESOLUTION is 8-bit, so duty range is 0..255.
    int duty = map(abs(speed), 0, 100, 0, 255);
    motorPwm.write(duty);
}

void updateMotor()
{
    // Reserved for acceleration limiting later.
}
