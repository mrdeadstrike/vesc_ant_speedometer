#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled! Please enable it in menuconfig."
#endif

BluetoothSerial SerialBT;

// --- Hardware configuration -------------------------------------------------
const char *DEVICE_NAME = "MirrorControl";

const int LEFT_SERVO_PIN = 25;
const int RIGHT_SERVO_PIN = 26;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

const int ANGLE_MIN = 60;
const int ANGLE_MAX = 150;

const int DEFAULT_LEFT_ANGLE = 120;
const int DEFAULT_RIGHT_ANGLE = 120;

// --- Internal state ---------------------------------------------------------
Servo leftServo;
Servo rightServo;

int currentLeftAngle = DEFAULT_LEFT_ANGLE;
int currentRightAngle = DEFAULT_RIGHT_ANGLE;

bool prevClientConnected = false;
String btInputBuffer;

// --- Helpers ----------------------------------------------------------------
int clampAngle(int value) {
  if (value < ANGLE_MIN) {
    return ANGLE_MIN;
  }
  if (value > ANGLE_MAX) {
    return ANGLE_MAX;
  }
  return value;
}

bool tryParseInt(const String &text, int &outValue) {
  if (text.length() == 0) {
    return false;
  }
  int startIndex = 0;
  if (text.charAt(0) == '-') {
    startIndex = 1;
  }
  if (startIndex >= text.length()) {
    return false;
  }
  for (int i = startIndex; i < text.length(); ++i) {
    if (!isDigit(text.charAt(i))) {
      return false;
    }
  }
  outValue = text.toInt();
  return true;
}

void sendAngle(const char side, int value) {
  SerialBT.print("ANGLE ");
  SerialBT.print(side);
  SerialBT.print(' ');
  SerialBT.println(value);
}

void setLeftAngle(int angle) {
  currentLeftAngle = clampAngle(angle);
  leftServo.write(currentLeftAngle);
  sendAngle('L', currentLeftAngle);
}

void setRightAngle(int angle) {
  currentRightAngle = clampAngle(angle);
  rightServo.write(currentRightAngle);
  sendAngle('R', currentRightAngle);
}

void setAngles(int leftAngle, int rightAngle) {
  currentLeftAngle = clampAngle(leftAngle);
  currentRightAngle = clampAngle(rightAngle);
  leftServo.write(currentLeftAngle);
  rightServo.write(currentRightAngle);
  sendAngle('L', currentLeftAngle);
  sendAngle('R', currentRightAngle);
}

void sendHello() {
  SerialBT.print("HELLO ");
  SerialBT.print(currentLeftAngle);
  SerialBT.print(' ');
  SerialBT.println(currentRightAngle);
}

void sendCurrentState() {
  sendAngle('L', currentLeftAngle);
  sendAngle('R', currentRightAngle);
}

void handleSetCommand(const String &payload) {
  String trimmed = payload;
  trimmed.trim();
  int spaceIndex = trimmed.indexOf(' ');
  if (spaceIndex <= 0) {
    SerialBT.println("ERR BAD_SET");
    return;
  }

  String sideToken = trimmed.substring(0, spaceIndex);
  sideToken.toUpperCase();
  String angleToken = trimmed.substring(spaceIndex + 1);
  angleToken.trim();

  int angleValue = 0;
  if (!tryParseInt(angleToken, angleValue)) {
    SerialBT.println("ERR BAD_VALUE");
    return;
  }

  if (sideToken == "L") {
    setLeftAngle(angleValue);
  } else if (sideToken == "R") {
    setRightAngle(angleValue);
  } else {
    SerialBT.println("ERR BAD_SIDE");
  }
}

void handleGetCommand(const String &payload) {
  String trimmed = payload;
  trimmed.trim();
  trimmed.toUpperCase();

  if (trimmed == "ALL") {
    sendCurrentState();
  } else if (trimmed == "L") {
    sendAngle('L', currentLeftAngle);
  } else if (trimmed == "R") {
    sendAngle('R', currentRightAngle);
  } else {
    SerialBT.println("ERR BAD_GET");
  }
}

void processCommand(const String &line) {
  String command = line;
  command.trim();
  if (command.length() == 0) {
    return;
  }

  int spaceIndex = command.indexOf(' ');
  String verb = (spaceIndex == -1) ? command : command.substring(0, spaceIndex);
  verb.toUpperCase();

  String payload = "";
  if (spaceIndex != -1) {
    payload = command.substring(spaceIndex + 1);
  }

  if (verb == "SET") {
    handleSetCommand(payload);
  } else if (verb == "GET") {
    handleGetCommand(payload);
  } else if (verb == "PING") {
    SerialBT.println("PONG");
  } else if (verb == "DELTA") {
    payload.trim();
    int spacePos = payload.indexOf(' ');
    if (spacePos <= 0) {
      SerialBT.println("ERR BAD_DELTA");
      return;
    }
    String sideToken = payload.substring(0, spacePos);
    sideToken.toUpperCase();
    String deltaToken = payload.substring(spacePos + 1);
    deltaToken.trim();
    int deltaValue = 0;
    if (!tryParseInt(deltaToken, deltaValue)) {
      SerialBT.println("ERR BAD_DELTA");
      return;
    }
    if (sideToken == "L") {
      setLeftAngle(currentLeftAngle + deltaValue);
    } else if (sideToken == "R") {
      setRightAngle(currentRightAngle + deltaValue);
    } else {
      SerialBT.println("ERR BAD_SIDE");
    }
  } else {
    SerialBT.println("ERR UNKNOWN");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting mirror control...");

  if (!SerialBT.begin(DEVICE_NAME)) {
    Serial.println("Bluetooth init failed!");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("Bluetooth ready");

  leftServo.setPeriodHertz(50);
  rightServo.setPeriodHertz(50);
  leftServo.attach(LEFT_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  rightServo.attach(RIGHT_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  setAngles(DEFAULT_LEFT_ANGLE, DEFAULT_RIGHT_ANGLE);
}

void loop() {
  bool clientConnected = SerialBT.hasClient();
  if (clientConnected && !prevClientConnected) {
    Serial.println("Client connected");
    sendHello();
    sendCurrentState();
  } else if (!clientConnected && prevClientConnected) {
    Serial.println("Client disconnected");
  }
  prevClientConnected = clientConnected;

  while (SerialBT.available()) {
    char ch = static_cast<char>(SerialBT.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      processCommand(btInputBuffer);
      btInputBuffer = "";
    } else {
      if (btInputBuffer.length() < 120) {
        btInputBuffer += ch;
      }
    }
  }

  delay(5);
}
