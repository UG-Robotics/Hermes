// #pragma once

// void initSerial();
// void processSerial();

// #pragma once

#include <Arduino.h>

// ---- Packet types exchanged with the Pi ---------------------------------
//
// Pi -> ESP32:   CMD,<speed>,<steer>,<action>,<mode>
//                EMG,<mode>
//                PING
//                ACK,<tag>

// ESP32 -> Pi:   STATUS,<message>
//                TEL,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<tof1_mm>,<tof2_mm>
//                ACK,<tag>
//                PING
//
// NOTE: RobotCommand / RobotTelemetry are defined here. Your original
// serial_protocol.cpp included "robot_data.h" — I don't have that file's
// contents. If it already declares structs with these names, delete the
// two structs below and keep using yours instead (just make sure the field
// names line up with what this file reads/writes).
// --------------------------------------------------------------------------

struct RobotCommand
{
    int speed = 0;
    int steer = 0;
    String action = "STOP";
    int mode = 1;
    bool valid = false; // true once at least one CMD/EMG has been parsed
};

struct RobotTelemetry
{
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float tof1_mm = 0;
    float tof2_mm = 0;
};

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