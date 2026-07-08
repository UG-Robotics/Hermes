#include <ESP32Servo.h>

const int SERVO_PIN = 12;

Servo steering;

void setup() {
  Serial.begin(115200);

  steering.attach(SERVO_PIN);

  Serial.println("=================================");
  Serial.println("        SERVO TEST");
  Serial.println("=================================");

  steering.write(90);
  delay(1000);
}

void loop() {

  Serial.println("------------------------------");

  Serial.println("Center (90°)");
  steering.write(90);
  delay(2000);

  Serial.println("Left (0°)");
  steering.write(0);
  delay(2000);

  Serial.println("Center (90°)");
  steering.write(90);
  delay(2000);

  Serial.println("Right (180°)");
  steering.write(180);
  delay(2000);
}