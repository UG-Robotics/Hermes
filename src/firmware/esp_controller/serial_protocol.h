// #pragma once

// void initSerial();
// void processSerial();

// #pragma once

#include <Arduino.h>

#include "robot_data.h"

// ---- Packet types exchanged with the Pi ---------------------------------
//
// Pi -> ESP32:   CMD,<speed>,<steer>,<action>,<mode>
//                EMG,<mode>
//                PING
//                ACK,<tag>
//                EVT,<name>  (local test hook; only START_BUTTON_PRESSED is
//                             supported, and it re-emits the real EVT line)
//
// ESP32 -> Pi:   STATUS,<message>
//                TEL,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<tof1_mm>,<tof2_mm>
//                ACK,<tag>
//                PING
// --------------------------------------------------------------------------

// Lifecycle
void initSerial();
void processSerial(); // call every loop(); reads available bytes, parses complete lines

// Inbound - endpoints the rest of the firmware reads from
bool commandAvailable();          // true if a new CMD/EMG arrived since last read
RobotCommand getLatestCommand();  // returns latest parsed command, clears commandAvailable()
bool emergencyActive();           // true after an EMG packet, until clearEmergency()
void clearEmergency();

// Outbound - endpoints the rest of the firmware sends through
void sendTelemetry(const RobotTelemetry &tel);
void sendStatus(const String &message);
void sendAck(const String &tag = "");
void sendPing();