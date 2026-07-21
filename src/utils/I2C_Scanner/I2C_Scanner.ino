#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// I2C bus pins (ESP32 defaults).
#define SDA_PIN 21
#define SCL_PIN 22

// Time-of-flight sensor shutdown (XSHUT) pin.
#define XSHUT_PIN 32

#define DEFAULT_ADDR 0x29   // VL53L1X power-on address
#define SENSOR_ADDR  0x30   // address we re-assign the sensor to

// We drive XSHUT ourselves, so the constructor gets NO shutdown pin (that keeps
// begin() from doing its own 5 ms reset pulse, which is too short to let a cold
// sensor boot before we talk to it).
Adafruit_VL53L1X sensor = Adafruit_VL53L1X();
bool sensorOK = false;

// Raw 16-bit big-endian register read, no library involved, so we can SEE
// whether the sensor is physically on the bus. Returns 0xFFFF if it doesn't
// even ACK its address.
uint16_t readReg16(uint8_t addr, uint16_t reg) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission() != 0) return 0xFFFF;  // no ACK -- not on the bus
  Wire.requestFrom(addr, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
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
      Serial.print(address, HEX);
      if (address == SENSOR_ADDR) Serial.print("  <- VL53L1X");
      Serial.println();
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
  delay(500);  // let USB-serial come up so the banner below isn't missed
  Serial.println("\n=== VL53L1X bring-up ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Bring the sensor out of reset ourselves and give it time to boot.
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(20);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(50);  // VL53L1X firmware boot

  // Is the sensor physically answering at its default address? Retry, because a
  // cold sensor can boot slowly. 0xEACC = healthy VL53L1X model ID (reg 0x010F).
  uint16_t modelId = 0xFFFF;
  for (int i = 1; i <= 10; i++) {
    modelId = readReg16(DEFAULT_ADDR, 0x010F);
    Serial.print("  probe ");
    Serial.print(i);
    Serial.print("/10 @0x29 model id = 0x");
    Serial.println(modelId, HEX);
    if (modelId == 0xEACC) break;
    delay(100);
  }

  if (modelId != 0xEACC) {
    Serial.println("Sensor not answering at 0x29 -- this is WIRING/POWER, not the");
    Serial.println("address assignment. Check VIN(3.3V)/GND/SDA(21)/SCL(22)/XSHUT(32)");
    Serial.println("and that SDA & SCL have pull-up resistors. Skipping begin().");
  } else if (!sensor.begin(SENSOR_ADDR, &Wire)) {
    // Sensor ACKed at 0x29 but ST init/re-address did not complete.
    Serial.print("begin() failed after sensor ACKed, status = ");
    Serial.println(sensor.vl_status);
  } else {
    Serial.print("VL53L1X initialized, re-addressed to 0x");
    Serial.println(SENSOR_ADDR, HEX);
    sensor.setTimingBudget(50);
    if (sensor.startRanging()) {
      sensorOK = true;
      Serial.println("Ranging started.");
    } else {
      Serial.print("startRanging() failed, status = ");
      Serial.println(sensor.vl_status);
    }
  }
}

void loop() {
  if (sensorOK && sensor.dataReady()) {
    int16_t dist = sensor.distance();
    if (dist == -1) {
      Serial.print("Range error, status = ");
      Serial.println(sensor.vl_status);
    } else {
      Serial.print("Distance @0x");
      Serial.print(SENSOR_ADDR, HEX);
      Serial.print(": ");
      Serial.print(dist);
      Serial.println(" mm");
    }
    sensor.clearInterrupt();
  }

  scanI2C();
  delay(1000);
}
