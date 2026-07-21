// Single-sensor VL53L1X bring-up test, using the Adafruit_VL53L1X library
// (ST's official driver underneath). We use this instead of the Pololu VL53L1X
// library because Pololu's init() divides by a calibration value it reads from
// the sensor and, on this sensor, that value comes back 0 -> IntegerDivideByZero
// -> ESP32 crash. Adafruit's begin() returns a clean status code instead.
//
// Correct Adafruit_VL53L1X API (NOT the Pololu one):
//   vl53.begin(addr, &Wire)   -> bool
//   vl53.startRanging()       -> bool
//   vl53.setTimingBudget(ms)  -> 15/20/33/50/100/200/500
//   vl53.dataReady()          -> bool
//   vl53.distance()           -> int16_t mm, -1 on error
//   vl53.clearInterrupt()
//   vl53.vl_status            -> last status code

#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

#define TOF_ADDR 0x29

// No XSHUT / IRQ pin wired for this single-sensor test.
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
bool sensorOK = false;

// Reads a 16-bit big-endian register straight over I2C (no library), so we can
// SEE what the sensor reports. Returns 0xFFFF if the sensor doesn't even ACK.
uint16_t readReg16(uint8_t addr, uint16_t reg) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    if (Wire.endTransmission() != 0) {
        return 0xFFFF;  // no ACK -- sensor not on the bus
    }
    Wire.requestFrom(addr, (uint8_t)2);
    if (Wire.available() < 2) {
        return 0xFFFF;
    }
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    return (hi << 8) | lo;
}

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    // 100 kHz (standard mode). Bump up only once everything is stable.
    Wire.setClock(100000);

    // Let the sensor's firmware finish booting before we touch it.
    delay(100);

    Serial.println("Initializing VL53L1X (Adafruit library)...");

    // Direct probe first: IDENTIFICATION__MODEL_ID (0x010F) reads 0xEACC on a
    // healthy VL53L1X. RETRY it, because a single probe can miss a sensor that
    // boots slowly on a cold power-up or has a marginal connection. If it reads
    // 0xEACC on ANY attempt, the sensor is reachable; if all attempts read
    // 0xFFFF, it is genuinely not answering -> physical wiring/power, not code.
    const int MAX_PROBES = 20;
    uint16_t modelId = 0xFFFF;
    for (int attempt = 1; attempt <= MAX_PROBES; attempt++) {
        modelId = readReg16(TOF_ADDR, 0x010F);
        Serial.print("  probe ");
        Serial.print(attempt);
        Serial.print("/");
        Serial.print(MAX_PROBES);
        Serial.print(": Model ID @0x29 = 0x");
        Serial.println(modelId, HEX);
        if (modelId == 0xEACC) {
            break;  // sensor answered -- stop retrying
        }
        delay(200);
    }

    if (modelId != 0xEACC) {
        Serial.println("Sensor never ACKed after all retries -- it is not reliably on the bus. "
                        "This is wiring/power, not software: reseat VIN/GND/SDA/SCL, reflow the "
                        "breakout header, and check VIN is a steady 3.3V. Skipping begin().");
    } else if (!vl53.begin(TOF_ADDR, &Wire)) {
        Serial.print("begin() failed, status = ");
        Serial.println(vl53.vl_status);
        Serial.println("Sensor answered its address but ST init did not complete -- "
                        "likely a clone/blank-calibration board or unstable power.");
    } else {
        Serial.println("VL53L1X begin() OK.");
        vl53.setTimingBudget(50);
        if (vl53.startRanging()) {
            sensorOK = true;
            Serial.println("Ranging started.");
        } else {
            Serial.print("startRanging() failed, status = ");
            Serial.println(vl53.vl_status);
        }
    }

    Serial.println();
    Serial.println("Starting measurements...");
    Serial.println();
}

void loop() {
    if (!sensorOK) {
        Serial.println("TOF NOT DETECTED");
        delay(500);
        return;
    }

    if (vl53.dataReady()) {
        int16_t dist = vl53.distance();
        if (dist == -1) {
            Serial.print("Range error, status = ");
            Serial.println(vl53.vl_status);
        } else {
            Serial.print("Distance: ");
            Serial.print(dist);
            Serial.println(" mm");
        }
        vl53.clearInterrupt();
    }

    delay(50);
}
