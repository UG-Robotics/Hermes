// #include <Wire.h>
// #include <Adafruit_VL53L0X.h>

// // pins
// #define SDA_PIN 21
// #define SCL_PIN 22

// #define LEFT_XSHUT 32
// #define RIGHT_XSHUT 33

// // addresses
// #define DEFAULT_ADDR 0x29
// #define LEFT_ADDR    0x30
// #define RIGHT_ADDR   0x29

// Adafruit_VL53L0X leftSensor;
// Adafruit_VL53L0X rightSensor;

// bool leftOK = false;
// bool rightOK = false;

// void setup() {
//     Serial.begin(115200);

//     Wire.begin(SDA_PIN, SCL_PIN);

//     pinMode(LEFT_XSHUT, OUTPUT);
//     pinMode(RIGHT_XSHUT, OUTPUT);

//     // Hold both off
//     digitalWrite(LEFT_XSHUT, LOW);
//     digitalWrite(RIGHT_XSHUT, LOW);
//     delay(100);

//     // left
//     digitalWrite(LEFT_XSHUT, HIGH);
//     delay(20);

//     if (leftSensor.begin(DEFAULT_ADDR, false, &Wire)) {

//         if (leftSensor.setAddress(LEFT_ADDR)) {
//             leftOK = true;
//             Serial.println("LEFT OK");
//         } else {
//             Serial.println("LEFT address failed");
//         }

//     } else {
//         Serial.println("LEFT init failed");
//     }

//     // right
//     digitalWrite(RIGHT_XSHUT, HIGH);
//     delay(20);

//     if (rightSensor.begin(RIGHT_ADDR, false, &Wire)) {
//         rightOK = true;
//         Serial.println("RIGHT OK");
//     } else {
//         Serial.println("RIGHT init failed");
//     }

//     Serial.println();
//     Serial.println("Starting measurements...");
//     Serial.println();
// }

// void loop() {

//     // left
//     if (leftOK) {

//         VL53L0X_RangingMeasurementData_t measure;
//         leftSensor.rangingTest(&measure, false);

//         Serial.print("LEFT : ");

//         if (measure.RangeStatus == 4) {
//             Serial.print("OUT OF RANGE");
//         } else {
//             Serial.print(measure.RangeMilliMeter);
//             Serial.print(" mm");
//         }

//     } else {

//         Serial.print("LEFT : NOT DETECTED");

//     }

//     Serial.print("     ");

//     // right
//     if (rightOK) {

//         VL53L0X_RangingMeasurementData_t measure;
//         rightSensor.rangingTest(&measure, false);

//         Serial.print("RIGHT : ");

//         if (measure.RangeStatus == 4) {
//             Serial.print("OUT OF RANGE");
//         } else {
//             Serial.print(measure.RangeMilliMeter);
//             Serial.print(" mm");
//         }

//     } else {

//         Serial.print("RIGHT : NOT DETECTED");

//     }

//     Serial.println();

//     delay(200);
// }

#include <Wire.h>

// Change these if your SDA/SCL pins are different
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32 I2C Scanner");
  Serial.println("=================================");
}

void loop() {
  byte count = 0;

  Serial.println("\nScanning...");

  for (byte address = 1; address < 127; address++) {

    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" device(s).");
  }

  Serial.println("-------------------------");
  delay(3000);
}