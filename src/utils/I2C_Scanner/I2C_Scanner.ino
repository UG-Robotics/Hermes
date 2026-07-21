#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// I2C bus pins (ESP32 defaults).
#define SDA_PIN 21
#define SCL_PIN 22

// Time-of-flight sensor shutdown (XSHUT) pin.
#define XSHUT_PIN 32

// Address the sensor is re-assigned to, off its 0x29 power-on default so it
// won't collide with a second sensor sharing this bus. begin(SENSOR_ADDR)
// moves the sensor onto this address during bring-up (see setup()).
#define SENSOR_ADDR 0x30

// Passing XSHUT_PIN to the constructor lets begin() pulse it to hardware-reset
// the sensor before re-addressing it.
Adafruit_VL53L1X sensor = Adafruit_VL53L1X(XSHUT_PIN);
bool sensorOK = false;

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
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  // begin(SENSOR_ADDR, &Wire) resets the sensor via XSHUT_PIN, then moves it
  // off the 0x29 default onto SENSOR_ADDR and initialises it there.
  if (sensor.begin(SENSOR_ADDR, &Wire)) {
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
  } else {
    Serial.print("VL53L1X init FAILED, status = ");
    Serial.println(sensor.vl_status);
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
