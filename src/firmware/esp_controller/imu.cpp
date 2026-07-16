#include <Arduino.h>
#include <Wire.h>

#include "imu.h"
#include "config.h"

// -----------------------------------------------------------------------
// LSM6DS-family register map (LSM6DS3 / LSM6DSOX / ISM330DHCX all share the
// registers used here). If your specific breakout is a different part with
// a different map, this is the one file to change -- readIMU()'s output
// contract (m/s^2, deg/s) must stay the same since serial_protocol.cpp and
// the whole Pi-side heading-hold PID (control/steering_control.py) depend on
// it.
// -----------------------------------------------------------------------
namespace {
    constexpr uint8_t REG_WHO_AM_I  = 0x0F;
    constexpr uint8_t REG_CTRL1_XL  = 0x10; // accelerometer ODR/scale
    constexpr uint8_t REG_CTRL2_G   = 0x11; // gyroscope ODR/scale
    constexpr uint8_t REG_CTRL3_C   = 0x12; // block-data-update / auto-increment
    constexpr uint8_t REG_OUTX_L_G  = 0x22; // gyro output start (6 bytes: gx,gy,gz)
    constexpr uint8_t REG_OUTX_L_XL = 0x28; // accel output start (6 bytes: ax,ay,az)

    // CTRL1_XL = 0x60 -> 416 Hz ODR (close enough to IMU_SAMPLE_RATE_HZ), +-4g
    constexpr uint8_t CTRL1_XL_416HZ_4G = 0x68;
    // CTRL2_G  = 0x60 -> 416 Hz ODR, +-1000 dps (matches typical WRO turn rates)
    constexpr uint8_t CTRL2_G_416HZ_1000DPS = 0x68;
    // CTRL3_C: IF_INC (register auto-increment) + BDU (block data update, no
    // torn reads while a sample is being written).
    constexpr uint8_t CTRL3_C_BDU_IFINC = 0x44;

    // Sensitivity for the scales configured above (datasheet constants).
    constexpr float ACCEL_SENSITIVITY_G_PER_LSB = 0.122f / 1000.0f;   // +-4g range
    constexpr float GYRO_SENSITIVITY_DPS_PER_LSB = 35.0f / 1000.0f;   // +-1000dps range
    constexpr float G_TO_MS2 = 9.80665f;

    bool healthy = false;

    bool writeReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool readReg(uint8_t reg, uint8_t &value) {
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            return false;
        }
        if (Wire.requestFrom((int)IMU_I2C_ADDRESS, 1) != 1) {
            return false;
        }
        value = Wire.read();
        return true;
    }

    // Reads `len` bytes starting at `reg` (auto-increment must be enabled via
    // CTRL3_C, which initIMU() does).
    bool readBlock(uint8_t reg, uint8_t *buf, size_t len) {
        Wire.beginTransmission(IMU_I2C_ADDRESS);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            return false;
        }
        if (Wire.requestFrom((int)IMU_I2C_ADDRESS, (int)len) != (int)len) {
            return false;
        }
        for (size_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
        return true;
    }

    int16_t toInt16(uint8_t lo, uint8_t hi) {
        return (int16_t)((hi << 8) | lo);
    }
}

bool initIMU() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    uint8_t whoAmI = 0;
    if (!readReg(REG_WHO_AM_I, whoAmI)) {
        Serial.println("STATUS,ERR IMU: no response on I2C bus");
        healthy = false;
        return false;
    }
    // NOTE: WHO_AM_I varies by exact part (LSM6DS3=0x69, LSM6DSOX=0x6C,
    // ISM330DHCX=0x6B). We log it rather than hard-reject on a mismatch, so
    // this driver keeps working if the breakout gets swapped for a
    // register-compatible sibling part.
    Serial.print("STATUS,IMU WHO_AM_I=0x");
    Serial.println(whoAmI, HEX);

    bool ok = true;
    ok &= writeReg(REG_CTRL1_XL, CTRL1_XL_416HZ_4G);
    ok &= writeReg(REG_CTRL2_G, CTRL2_G_416HZ_1000DPS);
    ok &= writeReg(REG_CTRL3_C, CTRL3_C_BDU_IFINC);

    if (!ok) {
        Serial.println("STATUS,ERR IMU: config write failed");
        healthy = false;
        return false;
    }

    delay(20); // let the first sample settle
    healthy = true;
    Serial.println("STATUS,IMU ready");
    return true;
}

void readIMU(RobotTelemetry &telemetry) {
    if (!healthy) {
        telemetry.ax = telemetry.ay = 0.0f;
        telemetry.az = G_TO_MS2; // level, stationary default
        telemetry.gx = telemetry.gy = telemetry.gz = 0.0f;
        return;
    }

    uint8_t g[6];
    uint8_t a[6];
    bool okG = readBlock(REG_OUTX_L_G, g, 6);
    bool okA = readBlock(REG_OUTX_L_XL, a, 6);

    if (!okG || !okA) {
        // Transient I2C glitch: keep the last-known-good values rather than
        // zeroing them out, so a single dropped read doesn't look like the
        // bot suddenly went weightless to the Pi-side heading-hold PID.
        return;
    }

    telemetry.gx = toInt16(g[0], g[1]) * GYRO_SENSITIVITY_DPS_PER_LSB;
    telemetry.gy = toInt16(g[2], g[3]) * GYRO_SENSITIVITY_DPS_PER_LSB;
    telemetry.gz = IMU_GZ_SIGN * toInt16(g[4], g[5]) * GYRO_SENSITIVITY_DPS_PER_LSB;

    telemetry.ax = toInt16(a[0], a[1]) * ACCEL_SENSITIVITY_G_PER_LSB * G_TO_MS2;
    telemetry.ay = toInt16(a[2], a[3]) * ACCEL_SENSITIVITY_G_PER_LSB * G_TO_MS2;
    telemetry.az = toInt16(a[4], a[5]) * ACCEL_SENSITIVITY_G_PER_LSB * G_TO_MS2;
}

bool imuHealthy() {
    return healthy;
}
