
// #include <Wire.h>

// // Change these if your SDA/SCL pins are different
// #define SDA_PIN 21
// #define SCL_PIN 22

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   Wire.begin(SDA_PIN, SCL_PIN);

//   Serial.println();
//   Serial.println("=================================");
//   Serial.println("ESP32 I2C Scanner");
//   Serial.println("=================================");
// }

// void loop() {
//   byte count = 0;

//   Serial.println("\nScanning...");

//   for (byte address = 1; address < 127; address++) {

//     Wire.beginTransmission(address);
//     byte error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("Found device at 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//       count++;
//     }
//     else if (error == 4) {
//       Serial.print("Unknown error at 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }

//   if (count == 0) {
//     Serial.println("No I2C devices found.");
//   } else {
//     Serial.print("Found ");
//     Serial.print(count);
//     Serial.println(" device(s).");
//   }

//   Serial.println("-------------------------");
//   delay(3000);
// }

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define LEFT_XSHUT   32
#define RIGHT_XSHUT  33

#define DEFAULT_ADDR 0x29
#define LEFT_ADDR    0x30
#define RIGHT_ADDR   0x29

Adafruit_VL53L0X leftSensor;
Adafruit_VL53L0X rightSensor;

void scanI2C() {
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
  }

  if (count == 0)
    Serial.println("No devices found.");
  else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" device(s).");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);

  // Turn both sensors off
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  delay(100);

  // ---------------- LEFT ----------------
  digitalWrite(LEFT_XSHUT, HIGH);
  delay(20);

  if (leftSensor.begin(DEFAULT_ADDR, false, &Wire)) {
    if (leftSensor.setAddress(LEFT_ADDR))
      Serial.println("LEFT initialized -> 0x30");
    else
      Serial.println("LEFT address change FAILED");
  } else {
    Serial.println("LEFT init FAILED");
  }

  // ---------------- RIGHT ----------------
  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(20);

  if (rightSensor.begin(DEFAULT_ADDR, false, &Wire))
    Serial.println("RIGHT initialized -> 0x29");
  else
    Serial.println("RIGHT init FAILED");

  delay(100);

  scanI2C();
}

void loop() {
}