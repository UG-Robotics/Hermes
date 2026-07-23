// #include <Arduino.h>

// // Motor pins
// const int IN1 = 18;
// const int IN2 = 19;
// const int ENA = 23;

// void setup() {
//   Serial.begin(115200);

//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(ENA, OUTPUT);

//   stopMotor();

//   Serial.println("=================================");
//   Serial.println("        MOTOR TEST");
//   Serial.println("=================================");
// }

// void loop() {

//   // Serial.println("Forward (50% speed)");
//   // forward(128);
//   // delay(3000);

//   // Serial.println("Stop");
//   // stopMotor();
//   // delay(2000);

//   Serial.println("Forward (100% speed)");
//   forward(255);
//   delay(4000);

//   Serial.println("Stop");
//   stopMotor();
//   delay(2000);

//   // Serial.println("Backward (50% speed)");
//   // backward(128);
//   // delay(3000);

//   // Serial.println("Stop");
//   // stopMotor();
//   // delay(2000);

//   Serial.println("Backward (100% speed)");
//   backward(255);
//   delay(4000);

//   Serial.println("Stop");
//   stopMotor();
//   delay(2000);
// }

// void forward(int speed) {
//   digitalWrite(IN1, HIGH);
//   digitalWrite(IN2, LOW);
//   analogWrite(ENA, speed);
// }

// void backward(int speed) {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   analogWrite(ENA, speed);
// }

// void stopMotor() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, LOW);
//   analogWrite(ENA, 0);
// }

// // // isolate pins
// // #include <Arduino.h>

// // const int IN1 = 18;

// // void setup() {
// //   Serial.begin(115200);
// //   pinMode(IN1, OUTPUT);
// // }

// // void loop() {
// //   digitalWrite(IN1, HIGH);
// //   Serial.println("IN1 HIGH");
// //   delay(1000);

// //   digitalWrite(IN1, LOW);
// //   Serial.println("IN1 LOW");
// //   delay(1000);
// // }

// // // isolate GPIO 18 PWM test
// // #include <Arduino.h>

// // const int IN1 = 18;

// // const int PWM_FREQ = 1000;       // 1 kHz
// // const int PWM_RESOLUTION = 8;    // 0-255


// // void setup() {
// //   Serial.begin(115200);

// //   ledcAttach(IN1, PWM_FREQ, PWM_RESOLUTION);

// //   Serial.println("GPIO 18 PWM TEST");
// // }


// // void loop() {

// //   Serial.println("PWM 100%");
// //   ledcWrite(IN1, 255);
// //   delay(2000);


// //   Serial.println("PWM 50%");
// //   ledcWrite(IN1, 128);
// //   delay(2000);


// //   Serial.println("PWM 25%");
// //   ledcWrite(IN1, 64);
// //   delay(2000);


// //   Serial.println("PWM OFF");
// //   ledcWrite(IN1, 0);
// //   delay(2000);
// // }

#include <Arduino.h>

// TB6612FNG Channel A
const int AIN1 = 18;
const int AIN2 = 25;
const int PWMA = 23;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8; // 0-255


void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);

  Serial.println("============================");
  Serial.println("TB6612FNG MOTOR TEST");
  Serial.println("============================");
}


void forward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  ledcWrite(PWMA, speed);
}


void backward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  ledcWrite(PWMA, speed);
}


void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  ledcWrite(PWMA, 0);
}


void loop() {

  Serial.println("FORWARD");
  forward(255);
  delay(3000);


  Serial.println("STOP");
  stopMotor();
  delay(2000);


  Serial.println("BACKWARD");
  backward(255);
  delay(3000);


  Serial.println("STOP");
  stopMotor();
  delay(2000);
}

// #include <Arduino.h>

// const int LED_PIN = 2;

// void setup() {
//   pinMode(LED_PIN, OUTPUT);

//   Serial.begin(115200);
//   Serial.println("ESP32 onboard LED test");
// }

// void loop() {
//   digitalWrite(LED_PIN, HIGH);
//   Serial.println("LED ON");
//   delay(1000);

//   digitalWrite(LED_PIN, LOW);
//   Serial.println("LED OFF");
//   delay(1000);
// }