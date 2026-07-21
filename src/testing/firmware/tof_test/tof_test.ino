// #include <Wire.h>
// #include <VL53L1X.h>

// // pins
// #define SDA_PIN 21
// #define SCL_PIN 22

// #define LEFT_XSHUT 35
// #define RIGHT_XSHUT 33

// // addresses (7-bit). 0x29 is the sensor's power-on default.
// #define DEFAULT_ADDR 0x29
// #define LEFT_ADDR    0x30
// #define RIGHT_ADDR   0x29

// VL53L1X leftSensor;
// VL53L1X rightSensor;

// bool leftOK = false;
// bool rightOK = false;

// void setup() {
//     Serial.begin(115200);

//     Wire.begin(SDA_PIN, SCL_PIN);
//     Wire.setClock(400000);

//     pinMode(LEFT_XSHUT, OUTPUT);
//     pinMode(RIGHT_XSHUT, OUTPUT);

//     // Hold both off
//     digitalWrite(LEFT_XSHUT, LOW);
//     digitalWrite(RIGHT_XSHUT, LOW);
//     delay(100);

//     // left
//     digitalWrite(LEFT_XSHUT, HIGH);
//     delay(20);

//     leftSensor.setTimeout(500);
//     if (leftSensor.init()) {

//         leftSensor.setAddress(LEFT_ADDR);
//         leftSensor.setDistanceMode(VL53L1X::Long);
//         leftSensor.setMeasurementTimingBudget(50000);
//         leftSensor.startContinuous(50);
//         leftOK = true;
//         Serial.println("LEFT OK");

//     } else {
//         Serial.println("LEFT init failed (timed out talking to sensor -- check wiring/power/address)");
//     }

//     // right
//     digitalWrite(RIGHT_XSHUT, HIGH);
//     delay(20);

//     rightSensor.setTimeout(500);
//     if (rightSensor.init()) {

//         rightSensor.setDistanceMode(VL53L1X::Long);
//         rightSensor.setMeasurementTimingBudget(50000);
//         rightSensor.startContinuous(50);
//         rightOK = true;
//         Serial.println("RIGHT OK");

//     } else {
//         Serial.println("RIGHT init failed (timed out talking to sensor -- check wiring/power/address)");
//     }

//     Serial.println();
//     Serial.println("Starting measurements...");
//     Serial.println();
// }

// void loop() {

//     // left
//     if (leftOK) {

//         uint16_t dist = leftSensor.read();
//         Serial.print("LEFT : ");

//         if (leftSensor.timeoutOccurred()) {
//             Serial.print("TIMEOUT");
//         } else {
//             Serial.print(dist);
//             Serial.print(" mm");
//         }

//     } else {

//         Serial.print("LEFT : NOT DETECTED");

//     }

//     Serial.print("     ");

//     // right
//     if (rightOK) {

//         uint16_t dist = rightSensor.read();
//         Serial.print("RIGHT : ");

//         if (rightSensor.timeoutOccurred()) {
//             Serial.print("TIMEOUT");
//         } else {
//             Serial.print(dist);
//             Serial.print(" mm");
//         }

//     } else {

//         Serial.print("RIGHT : NOT DETECTED");

//     }

//     Serial.println();

//     delay(200);
// }

#include <Wire.h>
#include <VL53L1X.h>

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

#define TOF_ADDR 0x29

VL53L1X tof;
bool sensorOK = false;

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    Serial.println("Initializing VL53L1X...");

    // Pololu's init() honors this timeout instead of blocking forever if the
    // sensor never ACKs on I2C (bad wiring/power/address).
    tof.setTimeout(500);

    if (tof.init()) {
        sensorOK = true;
        tof.setDistanceMode(VL53L1X::Long);
        tof.setMeasurementTimingBudget(50000);
        tof.startContinuous(50);
        Serial.println("TOF OK");
    } else {
        Serial.println("TOF init failed (timed out talking to sensor -- check wiring/power/address)");
    }

    Serial.println();
    Serial.println("Starting measurements...");
    Serial.println();
}

void loop() {

    if (sensorOK) {

        uint16_t dist = tof.read();

        Serial.print("Distance: ");

        if (tof.timeoutOccurred()) {
            Serial.println("TIMEOUT");
        } else {
            Serial.print(dist);
            Serial.println(" mm");
        }

    } else {

        Serial.println("TOF NOT DETECTED");

    }

    delay(200);
}
