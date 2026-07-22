// // DUAL VL53L1X bring-up test -- BOTH sensors active simultaneously, matching
// // real competition conditions (this mirrors what firmware/esp_controller/tof.cpp
// // does on the actual robot: LEFT on XSHUT=33, RIGHT on XSHUT=34, both ranging
// // continuously every loop).
// //
// // Library: Adafruit_VL53L1X (ST's official driver underneath), not Pololu's.
// // Pololu's init() divides by a calibration value read from the sensor, and on
// // a marginal connection that value can come back 0 -> IntegerDivideByZero ->
// // hard ESP32 crash. Adafruit's begin() returns a clean status code instead, so
// // a bad/disconnected sensor degrades to "not detected" rather than taking the
// // whole board down.
// //
// // ADDRESS REASSIGNMENT: Adafruit_VL53L1X has NO separate setAddress() method
// // (that's a Pololu-only API -- calling it on an Adafruit_VL53L1X is a compile
// // error, "has no member named 'setAddress'"). Instead, begin(new_addr, &Wire)
// // does the reassignment ITSELF: while only ONE sensor is awake on the bus (the
// // other held in reset via XSHUT), begin() talks to it at the sensor's power-on
// // default (0x29), commands the address change to new_addr, and continues setup
// // there. That's why the LEFT sensor below is brought up alone first and told to
// // become 0x30, before the RIGHT sensor (which stays at the default 0x29) is
// // ever released from reset.
// //
// // CRASH RESILIENCE: a hardware task watchdog reboots the ESP32 if anything
// // hangs (e.g. begin() waiting on a sensor that drops off the bus mid-init).
// // Combined with setup() re-running the full bring-up on every boot, this test
// // keeps retrying indefinitely no matter what either sensor does.
// //
// // Adafruit_VL53L1X API used here:
// //   begin(addr, &Wire)        -> bool  (also sets the device's address to `addr`)
// //   startRanging()            -> bool
// //   setTimingBudget(ms)       -> 15/20/33/50/100/200/500
// //   dataReady()               -> bool
// //   distance()                -> int16_t mm, -1 on error
// //   clearInterrupt()
// //   vl_status                 -> last status code

// #include <Wire.h>
// #include <Adafruit_VL53L1X.h>
// #include <esp_task_wdt.h>

// // I2C pins
// #define SDA_PIN 21
// #define SCL_PIN 22

// // XSHUT (shutdown) control pins -- both output-capable GPIOs.
// #define XSHUT_LEFT   33
// #define XSHUT_RIGHT  34

// // Sensor's power-on-default address, and the address we move LEFT to so both
// // can coexist on the bus at once (matches config.h's TOF_LEFT/RIGHT_I2C_ADDRESS
// // on the real firmware).
// #define DEFAULT_ADDR 0x29
// #define LEFT_ADDR    0x30
// #define RIGHT_ADDR   0x29

// #define WDT_TIMEOUT_S 12
// #define ID_PROBES 20

// Adafruit_VL53L1X leftSensor;
// Adafruit_VL53L1X rightSensor;
// bool leftOK = false;
// bool rightOK = false;

// // Reads a 16-bit big-endian register straight over I2C (no library), so we can
// // SEE what a sensor reports at a given address. 0xFFFF => no ACK.
// uint16_t readReg16(uint8_t addr, uint16_t reg) {
//     Wire.beginTransmission(addr);
//     Wire.write((uint8_t)(reg >> 8));
//     Wire.write((uint8_t)(reg & 0xFF));
//     if (Wire.endTransmission() != 0) {
//         return 0xFFFF;
//     }
//     Wire.requestFrom(addr, (uint8_t)2);
//     if (Wire.available() < 2) {
//         return 0xFFFF;
//     }
//     uint16_t hi = Wire.read();
//     uint16_t lo = Wire.read();
//     return (hi << 8) | lo;
// }

// // Confirms a sensor is really answering at `addr` before we hand off to the
// // driver -- retries because a single probe can miss a sensor that boots slowly
// // or has a marginal connection. IDENTIFICATION__MODEL_ID (0x010F) reads 0xEACC
// // on a healthy VL53L1X.
// bool probeUntilPresent(const char *label, uint8_t addr) {
//     for (int attempt = 1; attempt <= ID_PROBES; attempt++) {
//         uint16_t modelId = readReg16(addr, 0x010F);
//         Serial.print("  [");
//         Serial.print(label);
//         Serial.print("] probe ");
//         Serial.print(attempt);
//         Serial.print(": Model ID @0x");
//         Serial.print(addr, HEX);
//         Serial.print(" = 0x");
//         Serial.println(modelId, HEX);
//         if (modelId == 0xEACC) {
//             return true;
//         }
//         delay(200);
//         esp_task_wdt_reset();
//     }
//     return false;
// }

// void setup() {
//     Serial.begin(115200);

//     // --- Watchdog: reboot (and re-run this whole setup) if any pass hangs ---
//     esp_task_wdt_config_t wdtCfg;
//     wdtCfg.timeout_ms = WDT_TIMEOUT_S * 1000;
//     wdtCfg.idle_core_mask = 0;
//     wdtCfg.trigger_panic = true;
//     if (esp_task_wdt_reconfigure(&wdtCfg) != ESP_OK) {
//         esp_task_wdt_init(&wdtCfg);
//     }
//     esp_task_wdt_add(NULL);
//     esp_task_wdt_reset();

//     pinMode(XSHUT_LEFT, OUTPUT);
//     pinMode(XSHUT_RIGHT, OUTPUT);

//     // Hold BOTH sensors in hardware reset so neither answers on the bus yet.
//     digitalWrite(XSHUT_LEFT, LOW);
//     digitalWrite(XSHUT_RIGHT, LOW);
//     delay(50);

//     Wire.begin(SDA_PIN, SCL_PIN);
//     Wire.setClock(100000);  // 100 kHz standard mode -- more tolerant of marginal wiring than 400 kHz

//     Serial.println();
//     Serial.println("=== Dual VL53L1X test -- LEFT XSHUT=33, RIGHT XSHUT=34 ===");

//     // ---------------- LEFT: bring up alone at 0x29, move it to 0x30 ----------------
//     digitalWrite(XSHUT_LEFT, HIGH);
//     delay(100);  // let its firmware finish booting before we touch it
//     esp_task_wdt_reset();

//     Serial.println("Initializing LEFT sensor...");
//     if (!probeUntilPresent("LEFT", DEFAULT_ADDR)) {
//         Serial.println("[LEFT] Sensor never ACKed at 0x29 -- wiring/power issue. Will retry on next boot.");
//     } else if (!leftSensor.begin(LEFT_ADDR, &Wire)) {
//         // begin(LEFT_ADDR, ...) finds it at the 0x29 default and reassigns it
//         // to LEFT_ADDR (0x30) as part of setup -- no separate setAddress() call.
//         Serial.print("[LEFT] begin() failed, status = ");
//         Serial.println(leftSensor.vl_status);
//     } else {
//         leftSensor.setTimingBudget(50);
//         if (leftSensor.startRanging()) {
//             leftOK = true;
//             Serial.println("[LEFT] OK, now at 0x30, ranging.");
//         } else {
//             Serial.print("[LEFT] startRanging() failed, status = ");
//             Serial.println(leftSensor.vl_status);
//         }
//     }
//     esp_task_wdt_reset();

//     // ---------------- RIGHT: bring up at the now-vacated 0x29 ----------------
//     digitalWrite(XSHUT_RIGHT, HIGH);
//     delay(100);
//     esp_task_wdt_reset();

//     Serial.println("Initializing RIGHT sensor...");
//     if (!probeUntilPresent("RIGHT", RIGHT_ADDR)) {
//         Serial.println("[RIGHT] Sensor never ACKed at 0x29 -- wiring/power issue. Will retry on next boot.");
//     } else if (!rightSensor.begin(RIGHT_ADDR, &Wire)) {
//         Serial.print("[RIGHT] begin() failed, status = ");
//         Serial.println(rightSensor.vl_status);
//     } else {
//         rightSensor.setTimingBudget(50);
//         if (rightSensor.startRanging()) {
//             rightOK = true;
//             Serial.println("[RIGHT] OK, at 0x29, ranging.");
//         } else {
//             Serial.print("[RIGHT] startRanging() failed, status = ");
//             Serial.println(rightSensor.vl_status);
//         }
//     }

//     Serial.println();
//     Serial.print("Setup done. LEFT=");
//     Serial.print(leftOK ? "OK" : "FAIL");
//     Serial.print("  RIGHT=");
//     Serial.println(rightOK ? "OK" : "FAIL");
//     Serial.println();
// }

// void loop() {
//     esp_task_wdt_reset();  // pet the dog every pass

//     if (!leftOK && !rightOK) {
//         // Neither came up. Rather than spin forever printing into the void,
//         // let the watchdog's next silent window reboot us into a fresh
//         // full bring-up -- this delay is short enough not to trip it.
//         Serial.println("No sensors detected. Will keep retrying / rebooting via watchdog.");
//         delay(1000);
//         return;
//     }

//     if (leftOK && leftSensor.dataReady()) {
//         int16_t d = leftSensor.distance();
//         Serial.print("LEFT : ");
//         if (d < 0) {
//             Serial.print("ERROR status=");
//             Serial.print(leftSensor.vl_status);
//         } else {
//             Serial.print(d);
//             Serial.print(" mm");
//         }
//         leftSensor.clearInterrupt();
//     } else if (!leftOK) {
//         Serial.print("LEFT : ---");
//     }

//     Serial.print("    ");

//     if (rightOK && rightSensor.dataReady()) {
//         int16_t d = rightSensor.distance();
//         Serial.print("RIGHT : ");
//         if (d < 0) {
//             Serial.print("ERROR status=");
//             Serial.print(rightSensor.vl_status);
//         } else {
//             Serial.print(d);
//             Serial.print(" mm");
//         }
//         rightSensor.clearInterrupt();
//     } else if (!rightOK) {
//         Serial.print("RIGHT : ---");
//     }

//     Serial.println();
//     delay(1000);
// }


// ============================================================================
// TRIPLE VL53L1X test -- three sensors on XSHUT 34, 32, 33.
//
// Pololu VL53L1X library (same as the working 2-sensor test). Pololu's init()
// and read() honour setTimeout(), so a missing/dead sensor makes them RETURN
// (after ~500ms) instead of hanging -- that's what lets this keep going and
// test the rest even when one sensor is broken or absent. Each sensor is
// independent: init failures are logged and skipped, and the loop reads
// whatever came up.
//
// ORDER MATTERS because GPIO34 is INPUT-ONLY: its XSHUT line CANNOT be driven,
// so that sensor can't be held in reset -- it powers up on its own at 0x29.
// It must therefore be the FIRST sensor brought up and moved off 0x29, before
// the two controllable sensors (32, 33) are released from reset. So the array
// below lists the pin-34 sensor first.
//
// See the CAVEAT at the bottom of this file re: GPIO34 being input-only.
// ============================================================================
#include <Wire.h>
#include <VL53L1X.h>

// ===================== I2C =====================
#define SDA_PIN 21
#define SCL_PIN 22

// ===================== Sensors (input-only pin 34 listed FIRST) =====================
#define NUM_SENSORS 3

const char*   NAME[NUM_SENSORS]      = { "FRONT(34)", "LEFT(32)", "RIGHT(33)" };
const uint8_t XSHUT_PIN[NUM_SENSORS] = { 34,          32,         33 };
const uint8_t NEW_ADDR[NUM_SENSORS]  = { 0x30,        0x31,       0x29 };  // last stays at default
const bool    REASSIGN[NUM_SENSORS]  = { true,        true,       false };

VL53L1X sensor[NUM_SENSORS];
bool ok[NUM_SENSORS] = { false, false, false };

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== Triple VL53L1X Test =====");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Hold every controllable XSHUT LOW so those sensors stay in reset. (GPIO34
  // is input-only, so this write is a no-op there and that sensor stays on --
  // which is why it must be brought up / re-addressed first, below.)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT);
    digitalWrite(XSHUT_PIN[i], LOW);
  }
  delay(50);

  // Bring sensors up one at a time, in list order. At each step only THIS
  // sensor is answering at 0x29 (earlier ones re-addressed away, later ones
  // still held in reset), so init() + setAddress() can't collide.
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(XSHUT_PIN[i], HIGH);   // power this one on (no-op on 34, already on)
    delay(20);

    sensor[i].setTimeout(500);          // Pololu: gives up instead of hanging
    if (sensor[i].init()) {
      if (REASSIGN[i]) {
        sensor[i].setAddress(NEW_ADDR[i]);
      }
      sensor[i].setDistanceMode(VL53L1X::Long);
      sensor[i].setMeasurementTimingBudget(50000);
      sensor[i].startContinuous(50);
      ok[i] = true;
      Serial.print("  OK   ");
      Serial.print(NAME[i]);
      Serial.print(" @ 0x");
      Serial.println(NEW_ADDR[i], HEX);
    } else {
      ok[i] = false;                    // <-- keep going; do NOT stop
      Serial.print("  FAIL ");
      Serial.print(NAME[i]);
      Serial.println(" -- init timed out (absent/flaky). Skipping, continuing with the rest.");
    }
  }

  Serial.println("Initialization complete.");
  Serial.println();
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(NAME[i]);
    Serial.print(": ");

    if (!ok[i]) {
      Serial.print("OFFLINE");
    } else if (sensor[i].dataReady()) {
      // Non-blocking: only pull a sample when one is ready, so a sensor that
      // drops out mid-run can't stall the whole print loop waiting on it.
      uint16_t d = sensor[i].read(false);
      if (sensor[i].timeoutOccurred()) {
        Serial.print("TIMEOUT");
      } else {
        Serial.print(d);
        Serial.print("mm");
      }
    } else {
      Serial.print("...");  // no fresh sample this pass
    }

    if (i < NUM_SENSORS - 1) {
      Serial.print("   |   ");
    }
  }
  Serial.println();
  delay(100);
}

// ============================================================================
// CAVEAT -- GPIO34 is INPUT-ONLY on the ESP32.
//
// pinMode(34, OUTPUT) / digitalWrite(34, ...) do nothing, so the FRONT sensor's
// XSHUT can't actually be controlled. This test works around it by bringing
// that sensor up FIRST and re-addressing it off 0x29. But if the FRONT sensor
// is PRESENT yet fails to re-address (flaky), it stays at 0x29 and collides
// with LEFT/RIGHT when they power up. For a rock-solid 3-sensor rig, move the
// FRONT XSHUT wire to an output-capable pin (e.g. 25, 26, or 27) and update
// XSHUT_PIN[0] above -- then all three can be held in reset independently.
// ============================================================================