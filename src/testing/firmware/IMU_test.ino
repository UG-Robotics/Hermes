#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM6DSOX lsm6ds;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();   // Or Wire.begin(SDA, SCL); default = 21, 22

  Serial.println("\n==============================================");
  Serial.println("        LSM6DSOX IMU SENSOR TEST");
  Serial.println("==============================================");

  if (!lsm6ds.begin_I2C()) {
    Serial.println("ERROR: LSM6DSOX not detected.");
    while (1) delay(100);
  }

  Serial.println("SUCCESS: Sensor detected successfully!\n");

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  Serial.println("---------------------------------------------------------------");
  Serial.printf("%-15s %-12s %-12s %-12s\n", "Measurement", "X", "Y", "Z");
  Serial.println("---------------------------------------------------------------");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  // Clear previous table
  Serial.print("\033[2J\033[H");

  Serial.println("==============================================");
  Serial.println("            LSM6DSOX LIVE READINGS");
  Serial.println("==============================================");
  Serial.println();

  Serial.println("+----------------+------------+------------+------------+");
  Serial.printf("| %-14s | %10.2f | %10.2f | %10.2f |\n",
                "Acceleration",
                accel.acceleration.x,
                accel.acceleration.y,
                accel.acceleration.z);

  Serial.printf("| %-14s | %10.2f | %10.2f | %10.2f |\n",
                "Rotation",
                gyro.gyro.x,
                gyro.gyro.y,
                gyro.gyro.z);

  Serial.println("+----------------+------------+------------+------------+");

  Serial.printf("\nTemperature : %.2f °C\n\n", temp.temperature);

  // Simple explanation
  Serial.println("Meaning:");
  Serial.println("Acceleration:");
  Serial.println("  X = Left ↔ Right");
  Serial.println("  Y = Forward ↔ Backward");
  Serial.println("  Z = Up ↕ Down (≈9.8 when lying flat)");
  Serial.println();
  Serial.println("Rotation:");
  Serial.println("  X = Tilting forward/back");
  Serial.println("  Y = Rolling left/right");
  Serial.println("  Z = Spinning left/right");
  Serial.println();
  Serial.println("Tip: Move the board slowly and watch which numbers change.");

  delay(500);
}