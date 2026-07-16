#include <Arduino.h>

#include "start_button.h"
#include "config.h"

namespace {
    constexpr unsigned long DEBOUNCE_MS = 30;

    // Wired as INPUT_PULLUP: idle HIGH, pressed pulls LOW.
    int lastStableState = HIGH;
    int lastRawState = HIGH;
    unsigned long lastChangeMs = 0;
}

void emitStartButtonPressed() {
    Serial.println("EVT,START_BUTTON_PRESSED");
}

void initStartButton() {
    pinMode(PIN_START_BUTTON, INPUT_PULLUP);
    lastStableState = digitalRead(PIN_START_BUTTON);
    lastRawState = lastStableState;
    lastChangeMs = millis();
}

void updateStartButton() {
    int raw = digitalRead(PIN_START_BUTTON);

    if (raw != lastRawState) {
        lastRawState = raw;
        lastChangeMs = millis();
    }

    if ((millis() - lastChangeMs) >= DEBOUNCE_MS && raw != lastStableState) {
        lastStableState = raw;
        if (lastStableState == LOW) {
            // Falling edge = button just pressed (pull-up wiring).
            emitStartButtonPressed();
        }
    }
}
