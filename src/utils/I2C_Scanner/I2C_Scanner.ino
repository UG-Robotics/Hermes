
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
#include <Adafruit_VL53L1X.h>

// Adafruit_VL53L1X (ST's official driver), NOT Adafruit_VL53L0X -- our
// hardware is the VL53L1X. VL53L0X's API doesn't match: begin() takes
// (addr, bool, TwoWire*) and has a separate setAddress(); VL53L1X's begin()
// takes (addr, TwoWire*) and does the address reassignment ITSELF as part of
// that call -- there is no separate setAddress() method on this class.

#define SDA_PIN 21
#define SCL_PIN 22

#define LEFT_XSHUT   32
#define RIGHT_XSHUT  33

#define DEFAULT_ADDR 0x29
#define LEFT_ADDR    0x30
#define RIGHT_ADDR   0x29

Adafruit_VL53L1X leftSensor;
Adafruit_VL53L1X rightSensor;

// Adafruit's begin() has NO internal timeout -- if a sensor never reaches the
// "boot ready" state it expects, begin() blocks FOREVER, which is why a dead
// RIGHT sensor previously hung the sketch right after LEFT succeeded and
// nothing after that line ever printed. So: probe the raw ID register first
// (bounded retries) and only call begin() if something actually answers.
// Returns true if the sensor ACKs with the correct VL53L1X model ID.
bool probePresent(const char *label, uint8_t addr, int retries = 10) {
  for (int i = 1; i <= retries; i++) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x01);  // IDENTIFICATION__MODEL_ID high byte (0x010F)
    Wire.write((uint8_t)0x0F);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(addr, (uint8_t)2);
      if (Wire.available() >= 2) {
        uint16_t id = (Wire.read() << 8) | Wire.read();
        if (id == 0xEACC) {
          return true;
        }
      }
    }
    delay(50);
  }
  Serial.print(label);
  Serial.println(": not detected after retries -- skipping begin() so it can't hang.");
  return false;
}

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
  // Only LEFT is awake right now (RIGHT is still held in XSHUT reset), so
  // begin(LEFT_ADDR, &Wire) finds it at the sensor's power-on default (0x29)
  // and reassigns it to LEFT_ADDR as part of this same call.
  digitalWrite(LEFT_XSHUT, HIGH);
  delay(100);  // let firmware boot before we probe

  if (probePresent("LEFT", DEFAULT_ADDR)) {
    if (leftSensor.begin(LEFT_ADDR, &Wire)) {
      Serial.println("LEFT initialized -> 0x30");
    } else {
      Serial.print("LEFT init FAILED, status = ");
      Serial.println(leftSensor.vl_status);
    }
  }

  // ---------------- RIGHT ----------------
  // LEFT has already moved off 0x29 (or was skipped), so RIGHT can come up at
  // the default. Runs REGARDLESS of what happened to LEFT.
  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(100);

  if (probePresent("RIGHT", DEFAULT_ADDR)) {
    if (rightSensor.begin(RIGHT_ADDR, &Wire)) {
      Serial.println("RIGHT initialized -> 0x29");
    } else {
      Serial.print("RIGHT init FAILED, status = ");
      Serial.println(rightSensor.vl_status);
    }
  }

  delay(100);

  scanI2C();
}

void loop() {
}