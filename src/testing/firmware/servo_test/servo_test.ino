// // // actual test
// // #include <ESP32Servo.h>

// // const int SERVO_PIN = 14;

// // const int CENTER = 90;
// // const int LEFT = 60;
// // const int RIGHT = 120;

// // Servo steering;

// // void setup() {
// //   Serial.begin(115200);

// //   steering.attach(SERVO_PIN);

// //   Serial.println("=================================");
// //   Serial.println("        SERVO TEST");
// //   Serial.println("=================================");

// //   steering.write(CENTER);
// //   delay(1000);
// // }

// // void loop() {

// //   Serial.println("------------------------------");

// //   Serial.println("LEFT (60DEG)");
// //   steering.write(LEFT);
// //   delay(2000);

// //   Serial.println("RIGHT (120DEG)");
// //   steering.write(RIGHT);
// //   delay(2000);

// //   Serial.println("CENTER (90DEG)");
// //   steering.write(CENTER);
// //   delay(2000);

// //   Serial.println("RIGHT (120DEG)");
// //   steering.write(RIGHT);
// //   delay(2000);
// // }


// // serial monitor test
// #include <Arduino.h>
// #include <ESP32Servo.h>

// Servo steeringServo;

// // Change this to the GPIO your servo signal wire is connected to.
// constexpr int SERVO_PIN = 14;

// void setup() {
//     Serial.begin(115200);

//     steeringServo.setPeriodHertz(50);      // Standard servo frequency
//     steeringServo.attach(SERVO_PIN, 500, 2500);

//     steeringServo.write(90);               // Start centered

//     Serial.println();
//     Serial.println("=== Servo Test ===");
//     Serial.println("Enter an angle from 0-180 and press Enter.");
//     Serial.println("Example: 32");
//     Serial.println();
// }

// void loop() {
//     if (Serial.available()) {
//         String input = Serial.readStringUntil('\n');
//         input.trim();

//         // Require every character to be a digit so line noise (stray/NUL
//         // bytes, which trim() does NOT strip -- it only removes whitespace)
//         // can't silently parse as a valid angle via toInt()'s "stop at the
//         // first non-digit" behaviour.
//         bool isNumeric = input.length() > 0;
//         for (unsigned int i = 0; i < input.length(); i++) {
//             if (!isDigit(input.charAt(i))) {
//                 isNumeric = false;
//                 break;
//             }
//         }

//         if (!isNumeric) {
//             return; // not real typed input -- ignore silently
//         }

//         int angle = input.toInt();

//         if (angle >= 0 && angle <= 180) {
//             steeringServo.write(angle);

//             Serial.print("Moved to ");
//             Serial.print(angle);
//             Serial.println(" degrees.");
//         } else {
//             Serial.println("Invalid angle. Enter a value between 0 and 180.");
//         }
//     }
// }

#include <Arduino.h>
#include <ESP32Servo.h>

Servo steeringServo;

constexpr int SERVO_PIN = 14;

constexpr int MIN_ANGLE = 0;
constexpr int MAX_ANGLE = 180;
constexpr int STEP = 2;          // Degrees per step
constexpr int STEP_DELAY = 1000;   // ms between steps

void setup() {
    Serial.begin(115200);

    steeringServo.setPeriodHertz(50);
    steeringServo.attach(SERVO_PIN, 500, 2500);

    Serial.println("=== Servo Sweep Test ===");
}

void loop() {

    // Sweep left -> right
    // for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += STEP) {
    //     steeringServo.write(angle);

    //     Serial.print("Angle: ");
    //     Serial.println(angle);

    //     delay(STEP_DELAY);
    // }

        steeringServo.write(160);
        Serial.println("CENTER: 160deg");
        delay(2000);

        steeringServo.write(90);
        Serial.println("LEFT: 90deg");
        delay(2000);

        steeringServo.write(250);
        Serial.println("RIGHT1: 250deg"); //-110
        delay(2000);

        // steeringServo.write(-110);
        // Serial.println("RIGHT2: -110deg");
        // delay(2000);

    delay(1000);

    // Sweep right -> left
    // for (int angle = MAX_ANGLE; angle >= MIN_ANGLE; angle -= STEP) {
    //     steeringServo.write(angle);

    //     Serial.print("Angle: ");
    //     Serial.println(angle);

    //     delay(STEP_DELAY);
    // }

    // delay(1000);
}