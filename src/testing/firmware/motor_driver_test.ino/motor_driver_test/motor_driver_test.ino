// ======================================================
// L298N SINGLE MOTOR RANDOM MOTION TEST
// ESP32 - Channel A
// ======================================================

const int ENA = 23;   // PWM speed control
const int IN1 = 18;   // Direction
const int IN2 = 19;   // Direction

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8; // 0-255


void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);

  randomSeed(micros());

  Serial.println("L298N Random Motion Test");
}


void forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  ledcWrite(ENA, speed);
}


void reverse(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  ledcWrite(ENA, speed);
}


void brake() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);

  ledcWrite(ENA, 255);
}


void coast() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  ledcWrite(ENA, 0);
}


void loop() {

  int action = random(0, 4);
  int speed = random(100, 256);
  int duration = random(500, 2500);


  Serial.printf("IN1=%d IN2=%d\n",
                digitalRead(IN1),
                digitalRead(IN2));


  switch(action) {

    case 0:
      Serial.printf("FORWARD speed=%d time=%d\n",
                    speed, duration);
      forward(speed);
      break;


    case 1:
      Serial.printf("REVERSE speed=%d time=%d\n",
                    speed, duration);
      reverse(speed);
      break;


    case 2:
      Serial.println("BRAKE");
      brake();
      break;


    case 3:
      Serial.println("COAST");
      coast();
      break;
  }


  delay(duration);

  coast();
  delay(300);
}