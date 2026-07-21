#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"
#include "servo_control.h"

// Drives the steering servo with the ESP32Servo library -- the SAME method as
// testing/firmware/servo_test/servo_test.ino, which is known to move the servo
// correctly on the bench.
//
// WHY NOT raw ledcAttach/ledcWrite (the previous approach): the main firmware
// also drives the motor PWM with its own ledcAttach() (motor.cpp, 1000 Hz /
// 8-bit), initialised BEFORE the servo. Standing up a second raw LEDC channel
// at a very different config (50 Hz / 16-bit) alongside it makes servo motion
// depend on how the two share the chip's four LEDC timers -- which is exactly
// why the servo moved in the standalone test (no motor LEDC present) but not in
// the full firmware. ESP32Servo manages its own timer allocation and coexists
// cleanly with the motor's channel, so the servo behaves identically to the
// isolated test.
namespace
{
    Servo steeringServo;
    int currentSteeringAngle = SERVO_CENTER;
}

void initServo()
{
    // Reserve a dedicated LEDC timer for the servo BEFORE attaching, so its
    // 50 Hz frame can't be disturbed by the motor's own ledcAttach() (1000 Hz)
    // sharing the same timer. A shared/reconfigured timer would skew the pulse
    // width the servo actually sees, which shows up as the physical center
    // sitting off from the calibrated SERVO_CENTER even though we write 158.
    // (esp_controller.ino setup() calls initServo() BEFORE initMotor() so this
    // reservation happens first.)
    ESP32PWM::allocateTimer(0);

    // Match the working bench test exactly: 50 Hz frame, 500-2500 us pulse.
    steeringServo.setPeriodHertz(50);
    steeringServo.attach(PIN_STEERING_SERVO, 500, 2500);
    setSteeringAngle(SERVO_CENTER);
}

void setSteeringAngle(int angleDegrees)
{
    // Clamp to the calibrated physical travel (SERVO_LEFT..SERVO_RIGHT in
    // either order) so a bad command can't drive the linkage into a hard stop.
    int minAngle = min(SERVO_LEFT, SERVO_RIGHT);
    int maxAngle = max(SERVO_LEFT, SERVO_RIGHT);

    currentSteeringAngle = constrain(angleDegrees, minAngle, maxAngle);

    steeringServo.write(currentSteeringAngle);
}

void updateServo()
{
    (void)currentSteeringAngle;
}
