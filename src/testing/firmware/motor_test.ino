#include <Arduino.h>

// Motor pins
const int IN1 = 18;
const int IN2 = 19;
const int ENA = 23;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  stopMotor();

  Serial.println("=================================");
  Serial.println("        MOTOR TEST");
  Serial.println("=================================");
}

void loop() {

  Serial.println("Forward (50% speed)");
  forward(128);
  delay(3000);

  Serial.println("Stop");
  stopMotor();
  delay(2000);

  Serial.println("Forward (100% speed)");
  forward(255);
  delay(3000);

  Serial.println("Stop");
  stopMotor();
  delay(2000);

  Serial.println("Backward (50% speed)");
  backward(128);
  delay(3000);

  Serial.println("Stop");
  stopMotor();
  delay(2000);

  Serial.println("Backward (100% speed)");
  backward(255);
  delay(3000);

  Serial.println("Stop");
  stopMotor();
  delay(4000);
}

void forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void backward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}