#pragma once

// Physical start button (config.h: PIN_START_BUTTON). Debounced, edge-only:
// fires EVT,START_BUTTON_PRESSED exactly once per physical press, over
// serial, for runtime.py to forward into the state machine (only while it's
// actually in WAIT_FOR_START -- see Runtime._handle_hardware_event).
// emitStartButtonPressed() is the shared output path used by the physical
// button and by the serial test hook for hardware-less bring-up.

void initStartButton();
void updateStartButton(); // call every loop(); sends the EVT line on a fresh press
void emitStartButtonPressed();
