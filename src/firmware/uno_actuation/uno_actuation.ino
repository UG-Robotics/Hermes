#include <Servo.h>

const int PIN_START_BUTTON = 2;
const int PIN_MOTOR_PWM = 5;
const int PIN_MOTOR_FORWARD = 3;
const int PIN_MOTOR_BACKWARD = 4;
const int PIN_STEERING_SERVO = 6;
const int SERVO_CENTER_DEGREE = 90;
const int PIN_LED = 13;
const unsigned long SERIAL_WATCHDOG_TIMEOUT = 1000;

enum SystemState {
  WAIT_FOR_START,
  RUNNING
};

SystemState currentState = WAIT_FOR_START;
Servo steeringServo;
unsigned long lastPacketReceivedTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_BACKWARD, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  steeringServo.attach(PIN_STEERING_SERVO);
  steeringServo.write(SERVO_CENTER_DEGREE);
  stopActuation();
}

void loop() {
  switch (currentState) {
    case WAIT_FOR_START:
      if (digitalRead(PIN_START_BUTTON) == LOW) {
        delay(50); // debounce
        if (digitalRead(PIN_START_BUTTON) == LOW) {
          currentState = RUNNING;
          lastPacketReceivedTime = millis();
          digitalWrite(PIN_LED, HIGH);
          Serial.println("System initialized: RUNNING state activated.");
        }
      }
      stopActuation();
      break;

    case RUNNING:
      if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        parseAndExecute(input);
        lastPacketReceivedTime = millis();
      }
      
      // Watchdog Safety check
      if (millis() - lastPacketReceivedTime > SERIAL_WATCHDOG_TIMEOUT) {
        stopActuation();
        digitalWrite(PIN_LED, LOW);
      }
      break;
  }
}

void parseAndExecute(const String& packet) {
  if (packet.length() == 0) return;

  int firstComma = packet.indexOf(',');
  int secondComma = packet.indexOf(',', firstComma + 1);
  int thirdComma = packet.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || thirdComma == -1) return;

  int speed = packet.substring(0, firstComma).toInt();
  int steerOffset = packet.substring(firstComma + 1, secondComma).toInt();
  String actionStr = packet.substring(secondComma + 1, thirdComma);
  int mode = packet.substring(thirdComma + 1).toInt();

  actionStr.trim();
  speed = constrain(speed, 0, 255);

  int targetServoAngle = constrain(SERVO_CENTER_DEGREE + steerOffset, 0, 180);
  steeringServo.write(targetServoAngle);

  // Indicate override status on onboard LED
  if (mode == 1) {
    digitalWrite(PIN_LED, (millis() / 100) % 2);
  } else {
    digitalWrite(PIN_LED, HIGH);
  }

  // Set motor speeds and H-Bridge pin configuration
  if (actionStr.equalsIgnoreCase("FORWARD")) {
    digitalWrite(PIN_MOTOR_FORWARD, HIGH);
    digitalWrite(PIN_MOTOR_BACKWARD, LOW);
    analogWrite(PIN_MOTOR_PWM, speed);
  } else if (actionStr.equalsIgnoreCase("BACKWARD")) {
    digitalWrite(PIN_MOTOR_FORWARD, LOW);
    digitalWrite(PIN_MOTOR_BACKWARD, HIGH);
    analogWrite(PIN_MOTOR_PWM, speed);
  } else {
    stopActuation();
  }
}

void stopActuation() {
  analogWrite(PIN_MOTOR_PWM, 0);
  digitalWrite(PIN_MOTOR_FORWARD, LOW);
  digitalWrite(PIN_MOTOR_BACKWARD, LOW);
  steeringServo.write(SERVO_CENTER_DEGREE);
}
