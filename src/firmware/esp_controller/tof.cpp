#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "tof.h"
#include "config.h"

// -----------------------------------------------------------------------
// Two VL53L1X sensors share the same I2C bus as the IMU (config.h:
// I2C_SDA_PIN/I2C_SCL_PIN). Every VL53L1X boots at the same fixed address
// (0x29) with no hardware address-select pin, so bringing up a second one
// on the same bus needs the standard XSHUT re-addressing sequence:
//
//   1. Hold BOTH sensors in hardware reset (XSHUT low).
//   2. Release only the LEFT sensor's XSHUT, initialise it while it's the
//      only device answering at 0x29, then move it off 0x29 onto
//      TOF_LEFT_I2C_ADDRESS so it stops colliding with the next sensor.
//   3. Release the RIGHT sensor's XSHUT and initialise it at the now-free
//      default address (TOF_RIGHT_I2C_ADDRESS == TOF_DEFAULT_I2C_ADDRESS).
//
// Wire.begin() itself is NOT called here -- imu.cpp's initIMU() already
// brings the shared bus up (Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) and is
// called first in esp_controller.ino's setup(), so this file just reuses
// that bus.
//
// Each sensor's init() carries its own setTimeout() (see initTOF()), so a
// wiring fault on ONE sensor can only cost that sensor's timeout window --
// it can't hang the other sensor's bring-up or block esp_controller.ino's
// setup() indefinitely.
// -----------------------------------------------------------------------
namespace {
    VL53L1X tofLeft;
    VL53L1X tofRight;

    bool leftHealthy = false;
    bool rightHealthy = false;

    // Mirrors hardware/tof.py's ToFArray.OUT_OF_RANGE_MM. A sensor that
    // never came up (or one bad reading, see readOne()) reports THIS, not
    // 0.0 -- 0.0 would look to planning/obstacle_planner.py exactly like a
    // wall pressed against the bumper and trigger a hard avoidance nudge
    // for a sensor that's simply unplugged.
    constexpr float OUT_OF_RANGE_MM = 2000.0f;

    // Last-known-good readings, in mm. Held across a transient read failure
    // (see readTOF()) rather than snapping to a sentinel, mirroring
    // imu.cpp's "keep the last-known-good sample" behaviour for a dropped
    // IMU read.
    float lastLeftMm = OUT_OF_RANGE_MM;
    float lastRightMm = OUT_OF_RANGE_MM;

    // Reads one sensor. Returns true and fills `outMm` on a valid range;
    // false (leaving `outMm` untouched) on a failed/timed-out/out-of-range
    // read so the caller can decide whether to hold the last value.
    bool readOne(VL53L1X &sensor, float &outMm) {
        uint16_t dist = sensor.read();
        if (sensor.timeoutOccurred() || dist == 0 || dist > 2000) {
            return false;
        }
        outMm = (float)dist;
        return true;
    }
}

bool initTOF() {
    pinMode(PIN_TOF_LEFT_XSHUT, OUTPUT);
    pinMode(PIN_TOF_RIGHT_XSHUT, OUTPUT);

    // Step 1: both sensors held in reset.
    digitalWrite(PIN_TOF_LEFT_XSHUT, LOW);
    digitalWrite(PIN_TOF_RIGHT_XSHUT, LOW);
    delay(10);

    // Step 2: bring up LEFT alone at the default address, then re-address it.
    digitalWrite(PIN_TOF_LEFT_XSHUT, HIGH);
    delay(10);
    tofLeft.setTimeout(500);
    leftHealthy = tofLeft.init();
    if (leftHealthy) {
        leftHealthy = tofLeft.setAddress(TOF_LEFT_I2C_ADDRESS);
    }
    if (leftHealthy) {
        tofLeft.setDistanceMode(VL53L1X::Long);
        tofLeft.setMeasurementTimingBudget(50000);
        tofLeft.startContinuous(50);
    } else {
        Serial.println("STATUS,ERR ToF LEFT: init/address failed");
    }

    // Step 3: bring up RIGHT at the now-vacated default address.
    digitalWrite(PIN_TOF_RIGHT_XSHUT, HIGH);
    delay(10);
    tofRight.setTimeout(500);
    rightHealthy = tofRight.init();
    if (rightHealthy) {
        tofRight.setDistanceMode(VL53L1X::Long);
        tofRight.setMeasurementTimingBudget(50000);
        tofRight.startContinuous(50);
    } else {
        Serial.println("STATUS,ERR ToF RIGHT: init failed");
    }

    lastLeftMm = OUT_OF_RANGE_MM;
    lastRightMm = OUT_OF_RANGE_MM;

    if (leftHealthy && rightHealthy) {
        Serial.println("STATUS,ToF ready (left+right)");
    }

    return leftHealthy && rightHealthy;
}

void readTOF(RobotTelemetry &telemetry) {
    if (leftHealthy) {
        float mm;
        if (readOne(tofLeft, mm)) {
            lastLeftMm = mm;
        }
        // else: out of range / bad read this cycle -- hold lastLeftMm, same
        // "don't let one dropped sample look like the wall vanished"
        // reasoning imu.cpp uses for a dropped IMU read.
    }
    if (rightHealthy) {
        float mm;
        if (readOne(tofRight, mm)) {
            lastRightMm = mm;
        }
    }

    // tof1 = LEFT, tof2 = RIGHT -- must match hardware/tof.py's convention
    // and communication/packet_parser.py's TEL_FIELD_NAMES order.
    telemetry.tof1_mm = lastLeftMm;
    telemetry.tof2_mm = lastRightMm;
}

bool tofHealthy() {
    return leftHealthy && rightHealthy;
}
