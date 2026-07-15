// // I2C Configuration
// #define IMU_I2C_ADDRESS 0x6A
// #define TOF_I2C_ADDRESS 0x7E
// #define I2C_SDA_PIN 21
// #define I2C_SCL_PIN 22

// #define IMU_SAMPLE_RATE_HZ 104
// #define TOF_TIMING_BUDGET_MS 20

// // Motor pins
// #define PIN_MOTOR_IN1 18
// #define PIN_MOTOR_IN2 19
// #define PIN_MOTOR_PWM 23  


// // Steering servo
// #define PIN_STEERING_SERVO 12

// // Start button
// #define PIN_START_BUTTON 2

// // LED (status indicator)
// #define PIN_LED 25


// void setup()
// {
//     initSerial();
//     initMotor();
//     initServo();
//     initIMU();
// }

// void loop()
// {
//     processSerial();

//     readIMU();

//     updateMotor();

//     updateServo();

//     sendTelemetry();
// }

#include "serial_protocol.h"
#include "config.h"
#include "motor.h"
#include "servo_control.h"

namespace
{
    RobotCommand activeCommand;

    int speedToMotorDuty(const RobotCommand &command)
    {
        int speed = constrain(command.speed, 0, 255);
        return map(speed, 0, 255, 0, 100);
    }

    void applyCommand(const RobotCommand &command)
    {
        if (command.action == "STOP")
        {
            setMotorSpeed(0);
            setSteeringAngle(SERVO_CENTER);
            return;
        }

        int motorSpeed = speedToMotorDuty(command);
        if (command.action == "BACKWARD")
        {
            motorSpeed = -motorSpeed;
        }
        else if (command.action != "FORWARD")
        {
            motorSpeed = 0;
        }

        setMotorSpeed(motorSpeed);
        setSteeringAngle(map(constrain(command.steer, -90, 90), -90, 90, SERVO_LEFT, SERVO_RIGHT));
    }
}

void setup() {
    initSerial();
    initMotor();
    initServo();
    activeCommand.valid = true;
    applyCommand(activeCommand);
}

void loop() {
    processSerial();

    if (commandAvailable()) {
        activeCommand = getLatestCommand();
    }

    if (emergencyActive()) {
        setMotorSpeed(0);
        setSteeringAngle(SERVO_CENTER);
        clearEmergency();
    }
    else if (activeCommand.valid) {
        applyCommand(activeCommand);
    }

    updateMotor();
    updateServo();

    RobotTelemetry telemetry;
    sendTelemetry(telemetry);
}