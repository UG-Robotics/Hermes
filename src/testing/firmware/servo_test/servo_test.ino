// =====================================================================
//                    STEERING SERVO CALIBRATION
// =====================================================================
// Goal: find the three absolute servo angles that make the wheels point
// FORWARD (center), full LEFT, and full RIGHT -- then copy them into
// firmware/esp_controller/config.h (SERVO_CENTER / SERVO_LEFT / SERVO_RIGHT).
//
// KEY IDEA: steeringServo.write(a) commands an ABSOLUTE position, a in
// [0,180]. The servo has no notion of "center" -- center is just wherever
// the wheels happen to point straight given how the horn is clocked on the
// spline. So we don't COMPUTE left/right as center +/- X; we JOG the servo
// by eye and CAPTURE each real position.
//
// WHY CENTER MIGHT NOT BE ~90: if the horn was pressed on rotated, "straight"
// can land anywhere in 0..180. If it lands near an end (e.g. 5 or 175) you
// physically CAN'T steer far one way -- you hit 0 or 180 first. This tool
// prints your left/right travel from center and WARNS when it's lopsided so
// you know to re-seat the horn nearer the middle before trusting the numbers.
//
// ---------------------------------------------------------------------
// SERIAL COMMANDS (type, then Enter):
//   0..180   jog to that absolute angle and hold
//   +  / -   nudge current angle by 1 degree
//   +N / -N  nudge current angle by N degrees   (e.g. +5, -3)
//   c        capture current angle as CENTER (forward / wheels straight)
//   l        capture current angle as LEFT
//   r        capture current angle as RIGHT
//   s        show the captured values + travel summary
//   t        sweep through captured center->left->center->right->center
//   ?        reprint this help
// ---------------------------------------------------------------------
#include <Arduino.h>
#include <ESP32Servo.h>

Servo steeringServo;

// Change this to the GPIO your servo signal wire is connected to.
constexpr int SERVO_PIN = 14;

constexpr int MIN_ANGLE = 0;
constexpr int MAX_ANGLE = 180;

// Current commanded position, and the captured calibration points.
// -1 means "not captured yet".
int currentAngle = 90;
int centerAngle  = -1;
int leftAngle    = -1;
int rightAngle   = -1;

void printHelp() {
    Serial.println();
    Serial.println("=== Steering Servo Calibration ===");
    Serial.println("  0..180   jog to absolute angle");
    Serial.println("  + / -    nudge by 1 deg   (or +N / -N for N deg)");
    Serial.println("  c        capture CENTER (wheels straight)");
    Serial.println("  l        capture LEFT");
    Serial.println("  r        capture RIGHT");
    Serial.println("  s        show captured values + summary");
    Serial.println("  t        sweep through captured positions");
    Serial.println("  ?        show this help");
    Serial.println();
}

void applyAngle(int angle) {
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
    currentAngle = angle;
    steeringServo.write(angle);
    Serial.print("-> ");
    Serial.print(angle);
    Serial.println(" deg");
}

void showSummary() {
    Serial.println();
    Serial.println("---- captured values ----");
    Serial.print("  CENTER : "); Serial.println(centerAngle < 0 ? String("(not set)") : String(centerAngle));
    Serial.print("  LEFT   : "); Serial.println(leftAngle   < 0 ? String("(not set)") : String(leftAngle));
    Serial.print("  RIGHT  : "); Serial.println(rightAngle  < 0 ? String("(not set)") : String(rightAngle));

    if (centerAngle >= 0 && leftAngle >= 0 && rightAngle >= 0) {
        int leftTravel  = abs(leftAngle  - centerAngle);
        int rightTravel = abs(rightAngle - centerAngle);
        Serial.print("  travel : LEFT ");
        Serial.print(leftTravel);
        Serial.print(" deg from center, RIGHT ");
        Serial.print(rightTravel);
        Serial.println(" deg from center");

        // Warn if center sits so close to an end that one side is starved,
        // or if the two sides are badly asymmetric -- both mean "re-seat the
        // horn nearer the middle for symmetric steering".
        int marginLow  = centerAngle - MIN_ANGLE;
        int marginHigh = MAX_ANGLE - centerAngle;
        if (marginLow < 20 || marginHigh < 20) {
            Serial.println("  !! center is near an end of travel -- consider re-seating the horn ~90 deg");
        }
        int diff = abs(leftTravel - rightTravel);
        if (diff > 10) {
            Serial.println("  !! left/right travel is lopsided -- re-seat horn for symmetric steering");
        }

        Serial.println();
        Serial.println("  paste into config.h:");
        Serial.print("    const int SERVO_CENTER = "); Serial.print(centerAngle); Serial.println(";");
        Serial.print("    const int SERVO_LEFT   = "); Serial.print(leftAngle);   Serial.println(";");
        Serial.print("    const int SERVO_RIGHT  = "); Serial.print(rightAngle);  Serial.println(";");
    } else {
        Serial.println("  (capture all three with c / l / r to get config values)");
    }
    Serial.println("-------------------------");
    Serial.println();
}

void sweep() {
    if (centerAngle < 0 || leftAngle < 0 || rightAngle < 0) {
        Serial.println("Capture CENTER, LEFT and RIGHT first (c / l / r).");
        return;
    }
    Serial.println("Sweeping: center -> left -> center -> right -> center");
    applyAngle(centerAngle); delay(800);
    applyAngle(leftAngle);   delay(800);
    applyAngle(centerAngle); delay(800);
    applyAngle(rightAngle);  delay(800);
    applyAngle(centerAngle); delay(800);
    Serial.println("Sweep done.");
}

void handleCommand(String input) {
    input.trim();
    if (input.length() == 0) {
        return; // stray newline / line noise -- ignore
    }

    char c = input.charAt(0);

    // Relative nudge: "+", "-", "+5", "-3"
    if (c == '+' || c == '-') {
        String rest = input.substring(1);
        rest.trim();
        int delta = (rest.length() == 0) ? 1 : rest.toInt();
        applyAngle(currentAngle + (c == '-' ? -delta : delta));
        return;
    }

    // Absolute angle: all digits
    bool allDigits = true;
    for (unsigned int i = 0; i < input.length(); i++) {
        if (!isDigit(input.charAt(i))) { allDigits = false; break; }
    }
    if (allDigits) {
        int angle = input.toInt();
        if (angle < MIN_ANGLE || angle > MAX_ANGLE) {
            Serial.print("Out of range -- enter ");
            Serial.print(MIN_ANGLE); Serial.print("..");
            Serial.println(MAX_ANGLE);
            return;
        }
        applyAngle(angle);
        return;
    }

    // Single-letter commands
    switch (c) {
        case 'c': case 'C':
            centerAngle = currentAngle;
            Serial.print("CENTER captured = "); Serial.println(centerAngle);
            break;
        case 'l': case 'L':
            leftAngle = currentAngle;
            Serial.print("LEFT captured = "); Serial.println(leftAngle);
            break;
        case 'r': case 'R':
            rightAngle = currentAngle;
            Serial.print("RIGHT captured = "); Serial.println(rightAngle);
            break;
        case 's': case 'S':
            showSummary();
            break;
        case 't': case 'T':
            sweep();
            break;
        case '?':
            printHelp();
            break;
        default:
            Serial.print("Unknown command: ");
            Serial.println(input);
            Serial.println("Type ? for help.");
            break;
    }
}

void setup() {
    Serial.begin(115200);

    steeringServo.setPeriodHertz(50);          // standard 50 Hz servo frame
    steeringServo.attach(SERVO_PIN, 500, 2500); // 500-2500 us pulse range

    applyAngle(currentAngle);                   // start at a safe mid position
    printHelp();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        handleCommand(input);
    }
}
