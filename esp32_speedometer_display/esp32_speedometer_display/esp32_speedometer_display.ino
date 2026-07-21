#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic is not enabled for this ESP32 board."
#endif

// 5643BW is one four-digit TM1637 display. Its two side connectors duplicate
// the same interface, so only one CLK/DIO pair must drive it.
// Pin order below is CLK first, then DIO.
constexpr uint8_t RIGHT_CLK_PIN = 27;
constexpr uint8_t RIGHT_DIO_PIN = 14;

// These pins are connected to the duplicate connector on the module. Keep
// them high-impedance so they cannot conflict with the real TM1637 bus.
constexpr uint8_t UNUSED_LEFT_CLK_PIN = 26;
constexpr uint8_t UNUSED_LEFT_DIO_PIN = 25;

constexpr uint8_t DISPLAY_BRIGHTNESS = 7; // 0 (dim) ... 7 (bright)
constexpr char BLUETOOTH_DEVICE_NAME[] = "SpeedDisplay";
constexpr uint32_t SPEED_SIGNAL_TIMEOUT_MS = 2000;
constexpr uint32_t LINK_ANIMATION_STEP_MS = 100;
constexpr uint32_t CONTROLLER_ANIMATION_STEP_MS = 150;
constexpr uint8_t FINISH_SPEED_KMH = 60;
constexpr uint32_t MAX_ACCELERATION_MEASUREMENT_MS = 20000;
constexpr uint32_t RESULT_BLINK_HALF_PERIOD_MS = 300;
constexpr uint8_t RESULT_BLINK_COUNT = 5;
// On this clock-style 5643BW module, bit 7 of TM1637 address #1 (the second
// digit from the left) switches the central colon. Address #0 does not.
constexpr uint8_t COLON_DIGIT_INDEX = 1;

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
    command(0x88 | brightness_); // Display on + brightness.
  }

  uint8_t digit(uint8_t value) const { return digitSegments(value); }

  void showSegments(const uint8_t segments[4]) {
    writeData(segments, 4);
  }

private:
  uint8_t clkPin_;
  uint8_t dioPin_;
  uint8_t brightness_ = 7;

  static uint8_t digitSegments(uint8_t digit) {
    // Segments: 0b0GFEDCBA. The tenth entry is a blank digit.
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

  void writeByte(uint8_t data) {
    for (uint8_t bit = 0; bit < 8; ++bit) {
      digitalWrite(clkPin_, LOW);
      digitalWrite(dioPin_, (data >> bit) & 0x01);
      delaySignal();
      digitalWrite(clkPin_, HIGH);
      delaySignal();
    }

    // ACK is not needed for this one-way test, but release the line so the
    // controller can acknowledge without fighting the ESP32 output.
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
    command(0x40); // Automatic address increment.
    start();
    writeByte(0xC0); // First digit address.
    for (uint8_t index = 0; index < count; ++index) {
      writeByte(segments[index]);
    }
    stop();
    command(0x88 | brightness_);
  }
};

TM1637Display display(RIGHT_CLK_PIN, RIGHT_DIO_PIN);

uint16_t receivedSpeedKmh = 0;
uint32_t lastSpeedReceivedMs = 0;
bool receivedSpeed = false;
bool previousBtClientConnected = false;
String btInputBuffer;
bool frontControllerReady = false;
bool rearControllerReady = false;

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
  const int separator = text.indexOf(' ');
  if (separator <= 0) {
    return false;
  }
  String frontToken = text.substring(0, separator);
  String rearToken = text.substring(separator + 1);
  frontToken.trim();
  rearToken.trim();

  uint16_t frontValue = 0;
  uint16_t rearValue = 0;
  if (!tryParseSpeed(frontToken, frontValue) || !tryParseSpeed(rearToken, rearValue) ||
      frontValue > 1 || rearValue > 1) {
    return false;
  }
  frontReady = frontValue == 1;
  rearReady = rearValue == 1;
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
  } else if (verb == "LINK") {
    bool parsedFrontReady = false;
    bool parsedRearReady = false;
    if (tryParseLinkState(payload, parsedFrontReady, parsedRearReady)) {
      frontControllerReady = parsedFrontReady;
      rearControllerReady = parsedRearReady;
    } else {
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
    } else if (btInputBuffer.length() < 40) {
      btInputBuffer += ch;
    }
  }
}

uint16_t currentSpeedKmh() {
  if (!SerialBT.hasClient() || !receivedSpeed || millis() - lastSpeedReceivedMs > SPEED_SIGNAL_TIMEOUT_MS) {
    return 0;
  }
  return receivedSpeedKmh;
}

bool hasLiveSpeedSignal() {
  return SerialBT.hasClient() && receivedSpeed && millis() - lastSpeedReceivedMs <= SPEED_SIGNAL_TIMEOUT_MS;
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

void showLinkAnimation() {
  // Two adjacent segments travel clockwise along the outer rim of the four
  // digits: top row -> right edge -> bottom row -> left edge.
  static constexpr uint8_t DIGIT_INDEX[] = {0, 1, 2, 3, 3, 3, 3, 2, 1, 0, 0, 0};
  static constexpr uint8_t SEGMENT_BIT[] = {
      0x01, 0x01, 0x01, 0x01, // top
      0x02, 0x04, 0x08,       // right and bottom-right corner
      0x08, 0x08, 0x08,       // bottom
      0x10, 0x20              // left edge
  };
  constexpr uint8_t PATH_LENGTH = sizeof(DIGIT_INDEX) / sizeof(DIGIT_INDEX[0]);

  uint8_t segments[4] = {0x00, 0x00, 0x00, 0x00};
  const uint8_t first = (millis() / LINK_ANIMATION_STEP_MS) % PATH_LENGTH;
  for (uint8_t offset = 0; offset < 2; ++offset) {
    const uint8_t position = (first + offset) % PATH_LENGTH;
    segments[DIGIT_INDEX[position]] |= SEGMENT_BIT[position];
  }
  display.showSegments(segments);
}

void updateAccelerationMeasurement(uint16_t speedKmh) {
  const uint32_t now = millis();

  if (speedKmh == 0) {
    accelerationRunning = false;
    // Let a captured result finish all five flashes even if the simulated
    // speed has already fallen back to zero. The reset is performed on the
    // next loop pass after the blinking interval ends.
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
      // This start was too slow. Do not restart measuring until speed returns
      // to zero, which marks the next independent acceleration attempt.
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
  const uint32_t resultDurationMs = RESULT_BLINK_COUNT * 2 * RESULT_BLINK_HALF_PERIOD_MS;
  if (elapsedMs >= resultDurationMs) {
    resultActive = false;
    return false;
  }

  return (elapsedMs / RESULT_BLINK_HALF_PERIOD_MS) % 2 == 0;
}

void showSpeed(uint16_t speedKmh) {
  uint8_t segments[4] = {0x00, 0x00, 0x00, 0x00};

  if (speedKmh >= 100) {
    // Three digits for 100...999. The fourth digit stays unused.
    speedKmh = (speedKmh > 999) ? 999 : speedKmh;
    segments[0] = display.digit(speedKmh / 100);
    segments[1] = display.digit((speedKmh / 10) % 10);
    segments[2] = display.digit(speedKmh % 10);
  } else {
    // Two digits for 0...99 in the two central positions.
    segments[1] = display.digit(speedKmh / 10);
    segments[2] = display.digit(speedKmh % 10);
  }

  // While a FarDriver is still establishing telemetry, two adjacent segments
  // run around the lower square of its outer digit. Left = front/slave,
  // right = rear/master. Do not corrupt the hundreds digit at 100+ km/h.
  static constexpr uint8_t LOWER_SQUARE_BITS[] = {0x40, 0x04, 0x08, 0x10};
  const uint8_t phase = (millis() / CONTROLLER_ANIMATION_STEP_MS) % 4;
  if (!frontControllerReady && speedKmh < 100) {
    segments[0] |= LOWER_SQUARE_BITS[phase] | LOWER_SQUARE_BITS[(phase + 1) % 4];
  }
  if (!rearControllerReady) {
    segments[3] |= LOWER_SQUARE_BITS[phase] | LOWER_SQUARE_BITS[(phase + 1) % 4];
  }
  display.showSegments(segments);
}

void showAccelerationResult() {
  // The display has a usable central colon but no separately controlled decimal
  // points. Use SS:CC, where SS is seconds and CC is hundredths of a second.
  const uint32_t rawCentiseconds = completedAccelerationMs / 10;
  const uint32_t centiseconds = (rawCentiseconds > 9999) ? 9999 : rawCentiseconds;
  const uint8_t seconds = centiseconds / 100;
  const uint8_t hundredths = centiseconds % 100;
  uint8_t allSegments[4] = {
      display.digit(seconds / 10),
      display.digit(seconds % 10),
      display.digit(hundredths / 10),
      display.digit(hundredths % 10),
  };
  allSegments[COLON_DIGIT_INDEX] |= 0x80;

  if (seconds < 10) {
    allSegments[0] = 0x00; // Blank leading zero.
  }
  display.showSegments(allSegments);
}

void setup() {
  Serial.begin(115200);
  if (!SerialBT.begin(BLUETOOTH_DEVICE_NAME)) {
    Serial.println("Bluetooth init failed");
    while (true) {
      delay(1000);
    }
  }
  pinMode(UNUSED_LEFT_CLK_PIN, INPUT);
  pinMode(UNUSED_LEFT_DIO_PIN, INPUT);
  display.begin(DISPLAY_BRIGHTNESS);
  Serial.println("SpeedDisplay Bluetooth SPP ready");
}

void loop() {
  pollBluetooth();
  const bool liveSpeedSignal = hasLiveSpeedSignal();
  const uint16_t speedKmh = currentSpeedKmh();

  if (!liveSpeedSignal) {
    // A reconnect starts a fresh 0-60 attempt; never use speed data captured
    // before a Bluetooth interruption.
    resetAccelerationMeasurement();
    showLinkAnimation();
  } else {
    updateAccelerationMeasurement(speedKmh);
    if (shouldShowResult()) {
      showAccelerationResult();
    } else if (!resultActive) {
      showSpeed(speedKmh);
    } else {
      const uint8_t blankSegments[4] = {0x00, 0x00, 0x00, 0x00};
      display.showSegments(blankSegments);
    }
  }

  static uint16_t previousSpeed = 1000;
  if (speedKmh != previousSpeed) {
    Serial.printf("Speed: %u km/h%s\\n", static_cast<unsigned>(speedKmh),
                  resultActive ? " (showing 0-60 result)" : "");
    previousSpeed = speedKmh;
  }

  delay(15);
}
