// actual test
// #include <ESP32Servo.h>

// const int SERVO_PIN = 14;

// const int CENTER = 90;
// const int LEFT = 60;
// const int RIGHT = 120;

// Servo steering;

// void setup() {
//   Serial.begin(115200);

//   steering.attach(SERVO_PIN);

//   Serial.println("=================================");
//   Serial.println("        SERVO TEST");
//   Serial.println("=================================");

//   steering.write(CENTER);
//   delay(1000);
// }

// void loop() {

//   Serial.println("------------------------------");

//   Serial.println("LEFT (60DEG)");
//   steering.write(LEFT);
//   delay(2000);

//   Serial.println("RIGHT (120DEG)");
//   steering.write(RIGHT);
//   delay(2000);

//   Serial.println("CENTER (90DEG)");
//   steering.write(CENTER);
//   delay(2000);

//   Serial.println("RIGHT (120DEG)");
//   steering.write(RIGHT);
//   delay(2000);
// }


// serial monitor test
#include <Arduino.h>
#include <ESP32Servo.h>

Servo steeringServo;

// Change this to the GPIO your servo signal wire is connected to.
constexpr int SERVO_PIN = 14;

void setup() {
    Serial.begin(115200);

    steeringServo.setPeriodHertz(50);      // Standard servo frequency
    steeringServo.attach(SERVO_PIN, 500, 2500);

    steeringServo.write(90);               // Start centered

    Serial.println();
    Serial.println("=== Servo Test ===");
    Serial.println("Enter an angle from 0-180 and press Enter.");
    Serial.println("Example: 32");
    Serial.println();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        int angle = input.toInt();

        if (angle >= 0 && angle <= 180) {
            steeringServo.write(angle);

            Serial.print("Moved to ");
            Serial.print(angle);
            Serial.println(" degrees.");
        } else {
            Serial.println("Invalid angle. Enter a value between 0 and 180.");
        }
    }
}