// #include <Arduino.h>

// #include "serial_protocol.h"
// #include "robot_data.h"
// #include "config.h"

// RobotCommand command;
// RobotTelemetry telemetry;

// void initSerial()
// {
//     Serial.begin(BAUD_RATE);

//     while (!Serial)
//     {
//         delay(10);
//     }

//     Serial.println("Serial Initialised");
// }

// void processSerial()
// {
//     while (Serial.available())
//     {
//         Serial.read();
//     }
// }


#include "serial_protocol.h"
#include "config.h"
#include "start_button.h"

// ASSUMPTION: config.h defines BAUD_RATE (carried over from your original
// serial_protocol.cpp, which already relied on this).

namespace
{
    constexpr size_t LINE_BUF_MAX = 128;

    char lineBuf[LINE_BUF_MAX];
    size_t lineLen = 0;

    RobotCommand latestCommand;
    bool newCommandFlag = false;
    bool emergencyFlag = false;

    // Splits `text` on commas into up to maxParts substrings.
    // Returns the number of parts found.
    size_t splitFields(const String &text, String out[], size_t maxParts)
    {
        size_t count = 0;
        int start = 0;
        while (count < maxParts)
        {
            int comma = text.indexOf(',', start);
            if (comma == -1)
            {
                out[count++] = text.substring(start);
                break;
            }
            out[count++] = text.substring(start, comma);
            start = comma + 1;
        }
        return count;
    }

    void handleCmd(String fields[], size_t n)
    {
        // CMD,<speed>,<steer>,<action>,<mode>
        if (n != 5)
        {
            sendStatus("ERR malformed CMD: " + String(n) + " fields");
            return;
        }
        latestCommand.speed = fields[1].toInt();
        latestCommand.steer = fields[2].toInt();
        latestCommand.action = fields[3];
        latestCommand.action.toUpperCase();
        latestCommand.mode = fields[4].toInt();
        latestCommand.valid = true;
        newCommandFlag = true;
        emergencyFlag = false; // a fresh normal CMD clears a prior emergency latch
#if DEBUG_ECHO_CMD
        // Debug receipt confirmation: echo the ACCEPTED command back so the
        // monitor shows valid CMDs landing (they're otherwise silent, per the
        // NB below). Gated by DEBUG_ECHO_CMD in config.h -- turn it OFF before
        // racing, exactly for the flooding reason the NB describes.
        Serial.print("ECHO,CMD,");
        Serial.print(latestCommand.speed);  Serial.print(',');
        Serial.print(latestCommand.steer);  Serial.print(',');
        Serial.print(latestCommand.action); Serial.print(',');
        Serial.println(latestCommand.mode);
#endif
        // NB: do NOT echo every CMD back as STATUS here. The Pi streams a CMD
        // per tick (~20 Hz), and echoing 2 STATUS lines per CMD floods the
        // serial TX so hard it starves the TEL telemetry line (and can back up
        // the loop). Keep the wire quiet; errors below still report.
    }

    void handleEmg(String fields[], size_t n)
    {
        // EMG,<mode> — always forces a stop regardless of payload
        latestCommand.speed = 0;
        latestCommand.steer = 0;
        latestCommand.action = "STOP";
        latestCommand.mode = (n >= 2) ? fields[1].toInt() : 1;
        latestCommand.valid = true;
        newCommandFlag = true;
        emergencyFlag = true;
        sendStatus("EMG ack mode=" + String(latestCommand.mode));
    }

    void handleEvent(String fields[], size_t n)
    {
        // Serial-monitor test hook: EVT,START_BUTTON_PRESSED
        // This mirrors the real button's outgoing line so the Pi runtime can
        // exercise the full event pipeline without wired hardware.
        if (n != 2)
        {
            sendStatus("ERR malformed EVT: " + String(n) + " fields");
            return;
        }

        String name = fields[1];
        name.trim();
        name.toUpperCase();

        if (name == "START_BUTTON_PRESSED")
        {
            emitStartButtonPressed();
        }
        else
        {
            sendStatus("ERR unsupported EVT: " + name);
        }
    }

    void handleLine(const String &line)
    {
        if (line.length() == 0)
        {
            return;
        }

        String fields[6];
        size_t n = splitFields(line, fields, 6);
        String tag = fields[0];
        tag.trim();
        tag.toUpperCase();

        if (tag == "CMD")
        {
            handleCmd(fields, n);
        }
        else if (tag == "EMG")
        {
            handleEmg(fields, n);
        }
        else if (tag == "EVT")
        {
            handleEvent(fields, n);
        }
        else if (tag == "PING")
        {
            sendAck("PING");
        }
        else if (tag == "ACK")
        {
            // Pi acknowledged something we sent - nothing to do yet.
        }
        else
        {
            sendStatus("ERR unknown packet: " + line);
        }
    }
}

void initSerial()
{
    Serial.begin(BAUD_RATE);

    while (!Serial)
    {
        delay(10);
    }

    lineLen = 0;
    sendStatus("Serial Initialised");
}

void processSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            lineBuf[lineLen] = '\0';
            handleLine(String(lineBuf));
            lineLen = 0;
        }
        else if (c != '\r')
        {
            if (lineLen < LINE_BUF_MAX - 1)
            {
                lineBuf[lineLen++] = c;
            }
            else
            {
                // Line too long ? drop it and resync on the next newline
                // rather than silently misparsing garbage.
                lineLen = 0;
                sendStatus("ERR line overflow");
            }
        }
    }
}

bool commandAvailable()
{
    return newCommandFlag;
}

RobotCommand getLatestCommand()
{
    newCommandFlag = false;
    return latestCommand;
}

bool emergencyActive()
{
    return emergencyFlag;
}

void clearEmergency()
{
    emergencyFlag = false;
}

void sendTelemetry(const RobotTelemetry &tel)
{
    Serial.print("TEL,");
    Serial.print(tel.ax, 3); Serial.print(',');
    Serial.print(tel.ay, 3); Serial.print(',');
    Serial.print(tel.az, 3); Serial.print(',');
    Serial.print(tel.gx, 3); Serial.print(',');
    Serial.print(tel.gy, 3); Serial.print(',');
    Serial.print(tel.gz, 3); Serial.print(',');
    Serial.print(tel.tof1_mm, 1); Serial.print(',');
    Serial.println(tel.tof2_mm, 1);
}

void sendStatus(const String &message)
{
    Serial.print("STATUS,");
    Serial.println(message);
}

void sendAck(const String &tag)
{
    if (tag.length() > 0)
    {
        Serial.print("ACK,");
        Serial.println(tag);
    }
    else
    {
        Serial.println("ACK");
    }
}

void sendPing()
{
    Serial.println("PING");
}