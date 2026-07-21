#include "serial_protocol.h"
#include "config.h"
#include "motor.h"
#include "servo_control.h"
#include "imu.h"
#include "tof.h"
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
        // Motor: STOP (and any non-drive action) means "don't drive"; only
        // FORWARD/BACKWARD move the wheels. Steering is handled AFTER this,
        // unconditionally, so it applies to STOP too (see below).
        if (command.action == "STOP")
        {
            setMotorSpeed(0);
        }
        else
        {
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
        }

        // Steering runs for EVERY action, STOP included. The Pi sends a real
        // steer value alongside a STOP when the operator holds a/d without a
        // drive key ("pre-aim the wheels before moving" -- see the Pi's
        // hardware/buttons.py get_manual_target). Forcing SERVO_CENTER on STOP
        // here would throw that away and the wheels would never turn until a
        // drive key was also held. (Emergency stop still hard-centers via the
        // emergencyActive() branch in loop(), which does not call this.)
        //
        // NOTE: this is the ONE place a steer value crosses from the wire
        // protocol into the physical servo. Whatever the Pi sent -- whether
        // that's raw manual input or the output of the IMU heading-hold PID
        // (control/steering_control.py) -- lands here identically.
        //
        // The Pi sends an abstract steer in [-90, +90] (+ = right, - = left);
        // we map it to physical servo degrees. Mapping is PIECEWISE around
        // SERVO_CENTER instead of a single map(LEFT..RIGHT) because the
        // steering geometry is asymmetric (SERVO_CENTER is NOT the midpoint of
        // SERVO_LEFT/SERVO_RIGHT). A single linear map would put steer=0 at the
        // midpoint, biasing "straight ahead" off the calibrated center; the
        // split makes steer=0 land exactly on SERVO_CENTER and lets each side
        // use its own travel.
        int steerCmd = constrain(command.steer, -90, 90);
        int servoAngle = (steerCmd <= 0)
            ? map(steerCmd, -90, 0, SERVO_LEFT, SERVO_CENTER)
            : map(steerCmd,   0, 90, SERVO_CENTER, SERVO_RIGHT);
        setSteeringAngle(servoAngle);
    }
}

void setup() {
    initSerial();
    initMotor();
    initServo();
    initStartButton();
    initIMU(); // failure is non-fatal: readIMU() falls back to level/stationary
               // defaults and the STATUS,ERR line already told the Pi why.
    initTOF(); // same deal: non-fatal, readTOF() holds 0.0 for any sensor
               // that failed to come up and the STATUS,ERR line says which.

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
    readIMU(telemetry); // fills ax..gz
    readTOF(telemetry); // fills tof1_mm (left) / tof2_mm (right)
    sendTelemetry(telemetry);
}
