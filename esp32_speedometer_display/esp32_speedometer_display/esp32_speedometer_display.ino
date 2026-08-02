#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic is not enabled for this ESP32 board."
#endif

// Each 5643BW module uses one CLK/DIO pair. Temporary physical assignment:
// white speed display = 27/14, cell-voltage display = 26/25,
// temperature display = 32/33.
constexpr uint8_t SPEED_CLK_PIN = 27;
constexpr uint8_t SPEED_DIO_PIN = 14;
constexpr uint8_t CELL_CLK_PIN = 26;
constexpr uint8_t CELL_DIO_PIN = 25;
constexpr uint8_t TEMP_CLK_PIN = 32;
constexpr uint8_t TEMP_DIO_PIN = 33;
// Connect the toggle switch between GPIO23 and GND. GPIO23 supports the
// internal pull-up; GPIO34/35 do not.
constexpr uint8_t POWER_SWITCH_PIN = 23;

constexpr uint8_t DISPLAY_BRIGHTNESS = 7;
constexpr char BLUETOOTH_DEVICE_NAME[] = "SpeedDisplay";
constexpr uint32_t SPEED_SIGNAL_TIMEOUT_MS = 2000;
constexpr uint32_t TELEMETRY_SIGNAL_TIMEOUT_MS = 2000;
constexpr uint32_t LINK_ANIMATION_STEP_MS = 100;
constexpr uint32_t STATUS_ANIMATION_STEP_MS = 150;
constexpr uint32_t TEMP_PAGE_DURATION_MS = 3000;
constexpr uint32_t TEMP_ALERT_BLINK_HALF_PERIOD_MS = 500;
constexpr uint16_t CELL_ALERT_MILLIVOLTS = 3500;
constexpr int16_t BATTERY_ALERT_TEMP_C = 45;
constexpr int16_t CONTROLLER_ALERT_TEMP_C = 55;
constexpr int16_t MOTOR_ALERT_TEMP_C = 60;
constexpr uint8_t FINISH_SPEED_KMH = 60;
constexpr uint32_t MAX_ACCELERATION_MEASUREMENT_MS = 20000;
constexpr uint32_t RESULT_BLINK_HALF_PERIOD_MS = 300;
constexpr uint8_t RESULT_BLINK_COUNT = 5;
constexpr uint32_t POWER_SWITCH_DEBOUNCE_MS = 80;

// On this clock-style 5643BW module, bit 7 of address #1 controls the colon.
constexpr uint8_t COLON_DIGIT_INDEX = 1;
constexpr uint8_t SEGMENT_COLON = 0x80;
constexpr uint8_t SEGMENT_C = 0x39;
constexpr uint8_t SEGMENT_B_APPROXIMATION = 0x7C; // Seven-segment lowercase b.

BluetoothSerial SerialBT;

class TM1637Display {
public:
  TM1637Display(uint8_t clkPin, uint8_t dioPin) : clkPin_(clkPin), dioPin_(dioPin) {}

  void begin(uint8_t brightness) {
    pinMode(clkPin_, OUTPUT);
    pinMode(dioPin_, OUTPUT);
    digitalWrite(clkPin_, HIGH);
    digitalWrite(dioPin_, HIGH);
    setBrightness(brightness);
  }

  void setBrightness(uint8_t brightness) {
    brightness_ = constrain(brightness, 0, 7);
    command(0x88 | brightness_);
  }

  uint8_t digit(uint8_t value) const { return digitSegments(value); }

  void showSegments(const uint8_t segments[4]) { writeData(segments, 4); }

private:
  uint8_t clkPin_;
  uint8_t dioPin_;
  uint8_t brightness_ = 7;

  static uint8_t digitSegments(uint8_t digit) {
    static constexpr uint8_t DIGITS[] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66,
        0x6D, 0x7D, 0x07, 0x7F, 0x6F,
    };
    return digit < 10 ? DIGITS[digit] : 0x00;
  }

  void delaySignal() const { delayMicroseconds(5); }

  void start() {
    digitalWrite(dioPin_, HIGH);
    digitalWrite(clkPin_, HIGH);
    delaySignal();
    digitalWrite(dioPin_, LOW);
    delaySignal();
    digitalWrite(clkPin_, LOW);
  }

  void stop() {
    digitalWrite(clkPin_, LOW);
    digitalWrite(dioPin_, LOW);
    delaySignal();
    digitalWrite(clkPin_, HIGH);
    delaySignal();
    digitalWrite(dioPin_, HIGH);
    delaySignal();
  }

  void writeByte(uint8_t value) {
    for (uint8_t bit = 0; bit < 8; ++bit) {
      digitalWrite(clkPin_, LOW);
      digitalWrite(dioPin_, (value >> bit) & 0x01);
      delaySignal();
      digitalWrite(clkPin_, HIGH);
      delaySignal();
    }

    digitalWrite(clkPin_, LOW);
    pinMode(dioPin_, INPUT_PULLUP);
    delaySignal();
    digitalWrite(clkPin_, HIGH);
    delaySignal();
    digitalWrite(clkPin_, LOW);
    pinMode(dioPin_, OUTPUT);
  }

  void command(uint8_t value) {
    start();
    writeByte(value);
    stop();
  }

  void writeData(const uint8_t *segments, uint8_t count) {
    command(0x40);
    start();
    writeByte(0xC0);
    for (uint8_t index = 0; index < count; ++index) {
      writeByte(segments[index]);
    }
    stop();
    command(0x88 | brightness_);
  }
};

TM1637Display speedDisplay(SPEED_CLK_PIN, SPEED_DIO_PIN);
TM1637Display cellDisplay(CELL_CLK_PIN, CELL_DIO_PIN);
TM1637Display tempDisplay(TEMP_CLK_PIN, TEMP_DIO_PIN);

uint16_t receivedSpeedKmh = 0;
uint32_t lastSpeedReceivedMs = 0;
bool receivedSpeed = false;
bool previousBtClientConnected = false;
String btInputBuffer;

bool frontControllerReady = false;
bool rearControllerReady = false;
bool bmsReady = false;
uint16_t minimumCellMillivolts = 0;
int16_t maximumBatteryTempC = 0;
int16_t maximumControllerTempC = 0;
int16_t maximumMotorTempC = 0;
uint32_t lastTelemetryReceivedMs = 0;
bool receivedTelemetry = false;
bool allSourcesPreviouslyReady = false;
uint32_t allSourcesReadySinceMs = 0;

bool powerSwitchStableClosed = false;
bool powerSwitchCandidateClosed = false;
uint32_t powerSwitchCandidateSinceMs = 0;

uint32_t accelerationStartMs = 0;
uint32_t completedAccelerationMs = 0;
uint32_t resultStartMs = 0;
bool accelerationRunning = false;
bool resultActive = false;
bool resultShownThisRun = false;
bool measurementExpiredThisRun = false;

bool tryParseSpeed(const String &text, uint16_t &outSpeed) {
  if (text.length() == 0) {
    return false;
  }
  for (size_t index = 0; index < text.length(); ++index) {
    if (!isDigit(text.charAt(index))) {
      return false;
    }
  }
  const unsigned long parsed = text.toInt();
  outSpeed = parsed > 999 ? 999 : static_cast<uint16_t>(parsed);
  return true;
}

bool tryParseLinkState(const String &text, bool &frontReady, bool &rearReady) {
  int front = 0;
  int rear = 0;
  if (sscanf(text.c_str(), "%d %d", &front, &rear) != 2 ||
      front < 0 || front > 1 || rear < 0 || rear > 1) {
    return false;
  }
  frontReady = front == 1;
  rearReady = rear == 1;
  return true;
}

bool tryParseTelemetry(const String &text) {
  int front = 0;
  int rear = 0;
  int bms = 0;
  int cellMv = 0;
  int batteryTemp = 0;
  int controllerTemp = 0;
  int motorTemp = 0;
  if (sscanf(text.c_str(), "%d %d %d %d %d %d %d", &front, &rear, &bms,
             &cellMv, &batteryTemp, &controllerTemp, &motorTemp) != 7 ||
      front < 0 || front > 1 || rear < 0 || rear > 1 || bms < 0 || bms > 1 ||
      cellMv < 0 || cellMv > 6000 || batteryTemp < -99 || batteryTemp > 199 ||
      controllerTemp < -99 || controllerTemp > 199 || motorTemp < -99 || motorTemp > 199) {
    return false;
  }

  frontControllerReady = front == 1;
  rearControllerReady = rear == 1;
  bmsReady = bms == 1;
  minimumCellMillivolts = static_cast<uint16_t>(cellMv);
  maximumBatteryTempC = static_cast<int16_t>(batteryTemp);
  maximumControllerTempC = static_cast<int16_t>(controllerTemp);
  maximumMotorTempC = static_cast<int16_t>(motorTemp);
  lastTelemetryReceivedMs = millis();
  receivedTelemetry = true;
  return true;
}

void processBluetoothCommand(const String &line) {
  String command = line;
  command.trim();
  if (command.length() == 0) {
    return;
  }

  const int separator = command.indexOf(' ');
  String verb = separator < 0 ? command : command.substring(0, separator);
  verb.toUpperCase();
  String payload = separator < 0 ? "" : command.substring(separator + 1);
  payload.trim();

  if (verb == "SPEED") {
    uint16_t parsedSpeed = 0;
    if (tryParseSpeed(payload, parsedSpeed)) {
      receivedSpeedKmh = parsedSpeed;
      lastSpeedReceivedMs = millis();
      receivedSpeed = true;
    } else {
      SerialBT.println("ERR BAD_SPEED");
    }
  } else if (verb == "TELEM") {
    if (!tryParseTelemetry(payload)) {
      SerialBT.println("ERR BAD_TELEM");
    }
  } else if (verb == "LINK") {
    // Backward compatibility with an older Raspberry sender.
    if (!tryParseLinkState(payload, frontControllerReady, rearControllerReady)) {
      SerialBT.println("ERR BAD_LINK");
    }
  } else if (verb == "PING") {
    SerialBT.println("PONG");
  } else {
    SerialBT.println("ERR UNKNOWN");
  }
}

void pollBluetooth() {
  const bool clientConnected = SerialBT.hasClient();
  if (clientConnected && !previousBtClientConnected) {
    SerialBT.println("HELLO SPEED_DISPLAY");
  }
  previousBtClientConnected = clientConnected;

  while (SerialBT.available()) {
    const char ch = static_cast<char>(SerialBT.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      processBluetoothCommand(btInputBuffer);
      btInputBuffer = "";
    } else if (btInputBuffer.length() < 100) {
      btInputBuffer += ch;
    }
  }
}

void pollPowerSwitch() {
  const bool rawClosed = digitalRead(POWER_SWITCH_PIN) == LOW;
  const uint32_t now = millis();

  if (rawClosed != powerSwitchCandidateClosed) {
    powerSwitchCandidateClosed = rawClosed;
    powerSwitchCandidateSinceMs = now;
    return;
  }

  if (rawClosed == powerSwitchStableClosed ||
      now - powerSwitchCandidateSinceMs < POWER_SWITCH_DEBOUNCE_MS) {
    return;
  }

  const bool wasClosed = powerSwitchStableClosed;
  powerSwitchStableClosed = rawClosed;
  if (!wasClosed && powerSwitchStableClosed) {
    if (SerialBT.hasClient()) {
      SerialBT.println("SHUTDOWN");
      Serial.println("Power switch open->closed: SHUTDOWN sent");
    } else {
      Serial.println("Power switch open->closed: no Bluetooth client, ignored");
    }
  }
}

bool hasLiveSpeedSignal() {
  return SerialBT.hasClient() && receivedSpeed &&
         millis() - lastSpeedReceivedMs <= SPEED_SIGNAL_TIMEOUT_MS;
}

bool hasLiveTelemetrySignal() {
  return SerialBT.hasClient() && receivedTelemetry &&
         millis() - lastTelemetryReceivedMs <= TELEMETRY_SIGNAL_TIMEOUT_MS;
}

uint16_t currentSpeedKmh() {
  return hasLiveSpeedSignal() ? receivedSpeedKmh : 0;
}

void resetAccelerationMeasurement() {
  accelerationStartMs = 0;
  completedAccelerationMs = 0;
  resultStartMs = 0;
  accelerationRunning = false;
  resultActive = false;
  resultShownThisRun = false;
  measurementExpiredThisRun = false;
}

void showLinkAnimation(TM1637Display &target) {
  static constexpr uint8_t DIGIT_INDEX[] = {0, 1, 2, 3, 3, 3, 3, 2, 1, 0, 0, 0};
  static constexpr uint8_t SEGMENT_BIT[] = {
      0x01, 0x01, 0x01, 0x01,
      0x02, 0x04, 0x08,
      0x08, 0x08, 0x08,
      0x10, 0x20,
  };
  constexpr uint8_t PATH_LENGTH = sizeof(DIGIT_INDEX) / sizeof(DIGIT_INDEX[0]);

  uint8_t segments[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t first = (millis() / LINK_ANIMATION_STEP_MS) % PATH_LENGTH;
  for (uint8_t offset = 0; offset < 2; ++offset) {
    const uint8_t position = (first + offset) % PATH_LENGTH;
    segments[DIGIT_INDEX[position]] |= SEGMENT_BIT[position];
  }
  target.showSegments(segments);
}

void addLowerSquareAnimation(uint8_t segments[4], uint8_t digitIndex) {
  static constexpr uint8_t LOWER_SQUARE_BITS[] = {0x40, 0x04, 0x08, 0x10};
  const uint8_t phase = (millis() / STATUS_ANIMATION_STEP_MS) % 4;
  segments[digitIndex] |= LOWER_SQUARE_BITS[phase] |
                          LOWER_SQUARE_BITS[(phase + 1) % 4];
}

void addBmsCenterAnimation(uint8_t segments[4]) {
  // One shared lower rectangle spanning the two central digits.
  static constexpr uint8_t DIGIT_INDEX[] = {1, 2, 2, 2, 1, 1};
  static constexpr uint8_t SEGMENT_BIT[] = {0x40, 0x40, 0x04, 0x08, 0x08, 0x10};
  constexpr uint8_t PATH_LENGTH = sizeof(DIGIT_INDEX) / sizeof(DIGIT_INDEX[0]);
  const uint8_t first = (millis() / STATUS_ANIMATION_STEP_MS) % PATH_LENGTH;
  for (uint8_t offset = 0; offset < 2; ++offset) {
    const uint8_t position = (first + offset) % PATH_LENGTH;
    segments[DIGIT_INDEX[position]] |= SEGMENT_BIT[position];
  }
}

void updateAccelerationMeasurement(uint16_t speedKmh) {
  const uint32_t now = millis();

  if (speedKmh == 0) {
    accelerationRunning = false;
    if (!resultActive) {
      completedAccelerationMs = 0;
      resultShownThisRun = false;
      measurementExpiredThisRun = false;
    }
    return;
  }

  if (!accelerationRunning && !resultShownThisRun && !measurementExpiredThisRun) {
    accelerationStartMs = now;
    accelerationRunning = true;
  }

  if (accelerationRunning) {
    const uint32_t elapsedMs = now - accelerationStartMs;
    if (speedKmh >= FINISH_SPEED_KMH && elapsedMs <= MAX_ACCELERATION_MEASUREMENT_MS) {
      completedAccelerationMs = elapsedMs;
      accelerationRunning = false;
      resultStartMs = now;
      resultActive = true;
      resultShownThisRun = true;
    } else if (elapsedMs >= MAX_ACCELERATION_MEASUREMENT_MS) {
      accelerationRunning = false;
      measurementExpiredThisRun = true;
    }
  }
}

bool shouldShowResult() {
  if (!resultActive) {
    return false;
  }
  const uint32_t elapsedMs = millis() - resultStartMs;
  const uint32_t durationMs = RESULT_BLINK_COUNT * 2 * RESULT_BLINK_HALF_PERIOD_MS;
  if (elapsedMs >= durationMs) {
    resultActive = false;
    return false;
  }
  return (elapsedMs / RESULT_BLINK_HALF_PERIOD_MS) % 2 == 0;
}

void showSpeed(uint16_t speedKmh) {
  uint8_t segments[4] = {0x00, 0x00, 0x00, 0x00};
  if (speedKmh >= 100) {
    speedKmh = speedKmh > 999 ? 999 : speedKmh;
    segments[0] = speedDisplay.digit(speedKmh / 100);
    segments[1] = speedDisplay.digit((speedKmh / 10) % 10);
    segments[2] = speedDisplay.digit(speedKmh % 10);
  } else {
    segments[1] = speedDisplay.digit(speedKmh / 10);
    segments[2] = speedDisplay.digit(speedKmh % 10);
  }

  if (!frontControllerReady && speedKmh < 100) {
    addLowerSquareAnimation(segments, 0);
  }
  if (!rearControllerReady) {
    addLowerSquareAnimation(segments, 3);
  }
  speedDisplay.showSegments(segments);
}

void showAccelerationResult() {
  const uint32_t rawCentiseconds = completedAccelerationMs / 10;
  const uint32_t centiseconds = rawCentiseconds > 9999 ? 9999 : rawCentiseconds;
  const uint8_t seconds = centiseconds / 100;
  const uint8_t hundredths = centiseconds % 100;
  uint8_t segments[4] = {
      speedDisplay.digit(seconds / 10),
      speedDisplay.digit(seconds % 10),
      speedDisplay.digit(hundredths / 10),
      speedDisplay.digit(hundredths % 10),
  };
  segments[COLON_DIGIT_INDEX] |= SEGMENT_COLON;
  if (seconds < 10) {
    segments[0] = 0x00;
  }
  speedDisplay.showSegments(segments);
}

void showCellVoltage() {
  // 3450 mV becomes 3:45. This clock module has a colon, not an individually
  // controllable decimal point, so the colon represents the decimal separator.
  if (minimumCellMillivolts < CELL_ALERT_MILLIVOLTS &&
      (millis() / TEMP_ALERT_BLINK_HALF_PERIOD_MS) % 2 == 1) {
    const uint8_t blank[4] = {0x00, 0x00, 0x00, 0x00};
    cellDisplay.showSegments(blank);
    return;
  }

  const uint16_t roundedHundredths = (minimumCellMillivolts + 5) / 10;
  const uint16_t hundredths = roundedHundredths > 999 ? 999 : roundedHundredths;
  uint8_t segments[4] = {
      0x00,
      cellDisplay.digit((hundredths / 100) % 10),
      cellDisplay.digit((hundredths / 10) % 10),
      cellDisplay.digit(hundredths % 10),
  };
  segments[COLON_DIGIT_INDEX] |= SEGMENT_COLON;
  cellDisplay.showSegments(segments);
}

void showTemperatureWaiting() {
  uint8_t segments[4] = {0x00, 0x00, 0x00, 0x00};
  if (!frontControllerReady) {
    addLowerSquareAnimation(segments, 0);
  }
  if (!rearControllerReady) {
    addLowerSquareAnimation(segments, 3);
  }
  if (!bmsReady) {
    addBmsCenterAnimation(segments);
  }
  tempDisplay.showSegments(segments);
}

void showTemperatureValue(uint8_t page, int16_t rawValue) {
  const uint8_t value = static_cast<uint8_t>(constrain(rawValue, 0, 99));
  uint8_t segments[4] = {0x00, 0x00, 0x00, 0x00};
  if (page == 0) {
    segments[0] = SEGMENT_B_APPROXIMATION;
    segments[2] = tempDisplay.digit(value / 10);
    segments[3] = tempDisplay.digit(value % 10);
  } else if (page == 1) {
    segments[0] = SEGMENT_C;
    segments[2] = tempDisplay.digit(value / 10);
    segments[3] = tempDisplay.digit(value % 10);
  } else {
    // Motor/wheel temperature uses the two rightmost digits.
    segments[2] = tempDisplay.digit(value / 10);
    segments[3] = tempDisplay.digit(value % 10);
  }
  tempDisplay.showSegments(segments);
}

void showTemperaturePages() {
  const uint32_t elapsed = millis() - allSourcesReadySinceMs;
  const uint8_t page = (elapsed / TEMP_PAGE_DURATION_MS) % 3;
  int16_t value = maximumMotorTempC;
  int16_t alertThreshold = MOTOR_ALERT_TEMP_C;
  if (page == 0) {
    value = maximumBatteryTempC;
    alertThreshold = BATTERY_ALERT_TEMP_C;
  } else if (page == 1) {
    value = maximumControllerTempC;
    alertThreshold = CONTROLLER_ALERT_TEMP_C;
  }

  const bool alertBlank = value > alertThreshold &&
                          (millis() / TEMP_ALERT_BLINK_HALF_PERIOD_MS) % 2 == 1;
  if (alertBlank) {
    const uint8_t blank[4] = {0x00, 0x00, 0x00, 0x00};
    tempDisplay.showSegments(blank);
  } else {
    showTemperatureValue(page, value);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
  powerSwitchStableClosed = digitalRead(POWER_SWITCH_PIN) == LOW;
  powerSwitchCandidateClosed = powerSwitchStableClosed;
  powerSwitchCandidateSinceMs = millis();
  speedDisplay.begin(DISPLAY_BRIGHTNESS);
  cellDisplay.begin(DISPLAY_BRIGHTNESS);
  tempDisplay.begin(DISPLAY_BRIGHTNESS);
  if (!SerialBT.begin(BLUETOOTH_DEVICE_NAME)) {
    Serial.println("Bluetooth init failed");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("SpeedDisplay Bluetooth SPP ready (3 displays)");
}

void loop() {
  pollBluetooth();
  pollPowerSwitch();
  const bool liveSpeedSignal = hasLiveSpeedSignal();
  const bool liveTelemetrySignal = hasLiveTelemetrySignal();
  const uint16_t speedKmh = currentSpeedKmh();

  if (!liveTelemetrySignal) {
    frontControllerReady = false;
    rearControllerReady = false;
    bmsReady = false;
  }

  if (!liveSpeedSignal) {
    resetAccelerationMeasurement();
    allSourcesPreviouslyReady = false;
    showLinkAnimation(speedDisplay);
    showLinkAnimation(cellDisplay);
    showLinkAnimation(tempDisplay);
  } else {
    updateAccelerationMeasurement(speedKmh);
    if (shouldShowResult()) {
      showAccelerationResult();
    } else if (!resultActive) {
      showSpeed(speedKmh);
    } else {
      const uint8_t blank[4] = {0x00, 0x00, 0x00, 0x00};
      speedDisplay.showSegments(blank);
    }

    if (liveTelemetrySignal && bmsReady) {
      showCellVoltage();
    } else {
      showLinkAnimation(cellDisplay);
    }

    const bool allSourcesReady = liveTelemetrySignal && frontControllerReady &&
                                 rearControllerReady && bmsReady;
    if (allSourcesReady && !allSourcesPreviouslyReady) {
      allSourcesReadySinceMs = millis();
    }
    allSourcesPreviouslyReady = allSourcesReady;
    if (allSourcesReady) {
      showTemperaturePages();
    } else {
      showTemperatureWaiting();
    }
  }

  static uint16_t previousSpeed = 1000;
  if (speedKmh != previousSpeed) {
    Serial.printf("Speed: %u km/h%s\n", static_cast<unsigned>(speedKmh),
                  resultActive ? " (showing 0-60 result)" : "");
    previousSpeed = speedKmh;
  }
  delay(15);
}
