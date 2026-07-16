#include <Arduino.h>
#include <ESP32Servo.h>

// Quick ESP32 test sketch for Pi <-> ESP motor/steering link validation.
// Packet format matches src/communication/protocol.py and
// src/firmware/esp_controller/serial_protocol.cpp.

const int PIN_MOTOR_IN1 = 18;
const int PIN_MOTOR_IN2 = 19;
const int PIN_MOTOR_PWM = 23;
const int PIN_STEERING_SERVO = 14;

const int SERVO_CENTER = 90;
const int SERVO_LEFT = 45;
const int SERVO_RIGHT = 135;

const int BAUD_RATE = 115200;
const int MOTOR_PWM_FREQ = 1000;
const int MOTOR_PWM_RESOLUTION = 8;
const int MAX_DUTY = (1 << MOTOR_PWM_RESOLUTION) - 1;

const int SPEED_MIN = 0;
const int SPEED_MAX = 255;
const int STEER_MIN = 0;
const int STEER_MAX = 180;

Servo steering;
String lineBuffer;
unsigned long lastHeartbeatMs = 0;

void stopMotor()
{
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  ledcWrite(PIN_MOTOR_PWM, 0);
}

void driveForward(int speed)
{
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  ledcWrite(PIN_MOTOR_PWM, constrain(speed, SPEED_MIN, SPEED_MAX));
}

void driveBackward(int speed)
{
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  ledcWrite(PIN_MOTOR_PWM, constrain(speed, SPEED_MIN, SPEED_MAX));
}

void setSteering(int steer)
{
  steering.write(constrain(steer, STEER_MIN, STEER_MAX));
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

int parseIntField(const String &value, int fallback)
{
  String trimmed = value;
  trimmed.trim();
  if (trimmed.length() == 0)
  {
    return fallback;
  }
  return trimmed.toInt();
}

void handleCommandLine(const String &line)
{
  String text = line;
  text.trim();
  if (text.length() == 0)
  {
    return;
  }

  String fields[5];
  int count = 0;
  int start = 0;
  while (count < 5)
  {
    int comma = text.indexOf(',', start);
    if (comma < 0)
    {
      fields[count++] = text.substring(start);
      break;
    }
    fields[count++] = text.substring(start, comma);
    start = comma + 1;
  }

  String tag = fields[0];
  tag.trim();
  tag.toUpperCase();

  if (tag == "CMD")
  {
    if (count != 5)
    {
      sendStatus("ERR malformed CMD");
      return;
    }

    int speed = constrain(parseIntField(fields[1], 0), SPEED_MIN, SPEED_MAX);
    int steer = constrain(parseIntField(fields[2], SERVO_CENTER), STEER_MIN, STEER_MAX);
    String action = fields[3];
    action.trim();
    action.toUpperCase();

    int mode = parseIntField(fields[4], 1);

    setSteering(steer);

    if (action == "FORWARD")
    {
      driveForward(speed);
    }
    else if (action == "BACKWARD")
    {
      driveBackward(speed);
    }
    else
    {
      stopMotor();
      action = "STOP";
    }

    Serial.print("STATUS,CMD OK,");
    Serial.print(speed);
    Serial.print(',');
    Serial.print(steer);
    Serial.print(',');
    Serial.print(action);
    Serial.print(',');
    Serial.println(mode);

    sendAck("CMD");
    return;
  }

  if (tag == "EMG")
  {
    stopMotor();
    setSteering(SERVO_CENTER);
    sendStatus("EMG STOP");
    sendAck("EMG");
    return;
  }

  if (tag == "PING")
  {
    sendAck("PING");
    return;
  }

  if (tag == "ACK")
  {
    return;
  }

  sendStatus("ERR unknown packet: " + text);
}

void setup()
{
  Serial.begin(BAUD_RATE);
  while (!Serial)
  {
    delay(10);
  }

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);

  ledcAttach(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcWrite(PIN_MOTOR_PWM, 0);

  steering.attach(PIN_STEERING_SERVO, 500, 2400);
  setSteering(SERVO_CENTER);

  stopMotor();
  sendStatus("ESP_PI_TEST_READY");
  sendPing();
}

void loop()
{
  while (Serial.available() > 0)
  {
    char c = static_cast<char>(Serial.read());
    if (c == '\n')
    {
      handleCommandLine(lineBuffer);
      lineBuffer = "";
    }
    else if (c != '\r')
    {
      lineBuffer += c;
      if (lineBuffer.length() > 128)
      {
        lineBuffer = "";
        sendStatus("ERR line overflow");
      }
    }
  }

  unsigned long now = millis();
  if (now - lastHeartbeatMs >= 2000)
  {
    lastHeartbeatMs = now;
    sendStatus("LINK_ALIVE");
  }
}