#include "serial_protocol.h"
#include "config.h"
#include "motor.h"
#include "servo_control.h"
#include "imu.h"
#include "start_button.h"

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
            sendStatus("APPLY STOP motor=0 steer=90");
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
        // NOTE: this is the ONE place a steer value crosses from the wire
        // protocol into the physical servo. Whatever the Pi sent -- whether
        // that's raw manual input or the output of the IMU heading-hold PID
        // (control/steering_control.py) -- lands here identically.
        int servoAngle = map(constrain(command.steer, -90, 90), -90, 90, SERVO_LEFT, SERVO_RIGHT);
        setSteeringAngle(servoAngle);
        sendStatus("APPLY CMD motor=" + String(motorSpeed) + " steer=" + String(servoAngle) + " action=" + command.action);
    }
}

void setup() {
    initSerial();
    initMotor();
    initServo();
    initStartButton();
    initIMU(); // failure is non-fatal: readIMU() falls back to level/stationary
               // defaults and the STATUS,ERR line already told the Pi why.

    activeCommand.valid = true;
    applyCommand(activeCommand);
}

void loop() {
    processSerial();
    updateStartButton();

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
    readIMU(telemetry); // fills ax..gz; tof1_mm/tof2_mm remain 0.0 until a
                         // ToF driver is wired up (out of scope here)
    sendTelemetry(telemetry);
}
