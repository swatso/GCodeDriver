#include <Arduino.h>

#include <cstring>

#include "vehicle_config.h"

#ifndef GCODE_USB_DEBUG
#define GCODE_USB_DEBUG 0
#endif

namespace {
constexpr int8_t kSpeedMin = -50;
constexpr int8_t kSpeedMax = 50;
constexpr uint16_t kVehicleSettleMs = 3000;
constexpr uint16_t kMaxGCodeRateMs = 1000;  // 1 Hz
constexpr int8_t kSpeedDeadbandMmPerSec = 3;  // speeds within [-deadband, +deadband] are treated as stopped
constexpr float kHeadingOnlyFeedrateDegPerSec = 30.0F;

// GPIO mappings.
constexpr uint8_t kSpeedEncoderA = 13;
constexpr uint8_t kSpeedEncoderB = 12;
constexpr uint8_t kBearingEncoderA = 4;
constexpr uint8_t kBearingEncoderB = 14;
constexpr uint8_t kStopButtonInput = 32;
constexpr uint8_t kForwardButtonInput = 21;
constexpr uint8_t kBackwardButtonInput = 18;
constexpr uint8_t kPrintButtonInput = 19;
constexpr uint8_t kCncSerialTx = 22;
constexpr uint8_t kCncSerialRx = 23;
constexpr uint8_t kVehicleInputs[kVehicleInputCount] = {16, 27, 17, 26, 25};
constexpr uint16_t kStopDecelIntervalMs = 100;
constexpr int8_t kStopDecelStepMmPerSec = 1;
constexpr uint32_t kUsbSerialBaudRate = 115200;
constexpr uint16_t kButtonDebounceMs = 50;
constexpr size_t kRecordedGCodeCapacity = 96;
constexpr size_t kRecordedGCodeLineLength = 80;

// Bed Size 265, 225
constexpr uint16_t kBedSizeX = 265;
constexpr uint16_t kBedSizeY = 225;

struct Encoder {
  uint8_t pinA;
  uint8_t pinB;
  uint8_t lastState;

  void begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    lastState = static_cast<uint8_t>((digitalRead(pinA) << 1) | digitalRead(pinB));
  }

  int8_t readDelta() {
    const uint8_t newState =
        static_cast<uint8_t>((digitalRead(pinA) << 1) | digitalRead(pinB));
    const uint8_t index = static_cast<uint8_t>((lastState << 2) | newState);
    static constexpr int8_t transitionTable[16] = {0,  -1, 1,  0, 1, 0,  0,  -1,
                                                    -1, 0,  0,  1, 0, 1,  -1, 0};
    lastState = newState;
    return transitionTable[index];
  }
};

struct DebouncedButton {
  uint8_t pin;
  bool stablePressed = false;
  bool lastRawPressed = false;
  uint32_t lastChangeMs = 0;

  explicit DebouncedButton(uint8_t buttonPin) : pin(buttonPin) {}

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    const bool rawPressed = isPressedRaw();
    stablePressed = rawPressed;
    lastRawPressed = rawPressed;
    lastChangeMs = millis();
  }

  bool isPressedRaw() const { return digitalRead(pin) == LOW; }

  bool wasPressed() {
    const uint32_t nowMs = millis();
    const bool rawPressed = isPressedRaw();

    if (rawPressed != lastRawPressed) {
      lastRawPressed = rawPressed;
      lastChangeMs = nowMs;
    }

    if ((nowMs - lastChangeMs) < kButtonDebounceMs || rawPressed == stablePressed) {
      return false;
    }

    stablePressed = rawPressed;
    return stablePressed;
  }
};

struct RecordedState {
  Pose vehiclePoses[kVehicleCount];
  uint8_t currentVehicle;
  int16_t bearingDeg;
};

struct RecordedGCodeLine {
  char line[kRecordedGCodeLineLength];
  RecordedState state;
};

Encoder speedEncoder{kSpeedEncoderA, kSpeedEncoderB, 0};
Encoder bearingEncoder{kBearingEncoderA, kBearingEncoderB, 0};
DebouncedButton forwardButton{kForwardButtonInput};
DebouncedButton backwardButton{kBackwardButtonInput};
DebouncedButton printButton{kPrintButtonInput};

Pose vehiclePoses[kVehicleCount];
uint8_t currentVehicle = 0;
uint8_t pendingVehicle = 0;
uint32_t pendingVehicleSinceMs = 0;

int8_t speedMmPerSec = 0;
int16_t bearingDeg = 0;

bool poseDirty = false;
uint32_t lastPoseUpdateMs = 0;
uint32_t lastGCodeSentMs = 0;
uint32_t lastStopDecelMs = 0;
constexpr size_t kCncRxBufferSize = 96;
char cncRxBuffer[kCncRxBufferSize] = {};
size_t cncRxLength = 0;
uint16_t cncOkResponsesPending = 0;

RecordedGCodeLine recordedGCode[kRecordedGCodeCapacity] = {};
size_t recordedGCodeCount = 0;
bool replayModeActive = false;
size_t replayIndex = 0;
bool printReplayActive = false;
size_t printReplayIndex = 0;
bool printReplayWaitingForOk = false;

float toRadians(float deg) { return deg * DEG_TO_RAD; }

int16_t normalizeBearing(int16_t value) {
  while (value < -20) {
    value += 360;
  }
  while (value > 400) {
    value -= 360;
  }
  return value;
}

int8_t clampSpeed(int32_t value) {
  // Clamping to int8_t range just in case, but the encoder deltas should prevent overflow.
  if (value < kSpeedMin) {
    return kSpeedMin;
  }
  if (value > kSpeedMax) {
    return kSpeedMax;
  }
  return static_cast<int8_t>(value);
}

RecordedState captureRecordedState() {
  RecordedState state{};
  for (uint8_t i = 0; i < kVehicleCount; ++i) {
    state.vehiclePoses[i] = vehiclePoses[i];
  }
  state.currentVehicle = currentVehicle;
  state.bearingDeg = bearingDeg;
  return state;
}

void restoreRecordedState(const RecordedState& state) {
  for (uint8_t i = 0; i < kVehicleCount; ++i) {
    vehiclePoses[i] = state.vehiclePoses[i];
  }
  currentVehicle = state.currentVehicle;
  bearingDeg = state.bearingDeg;
  pendingVehicle = currentVehicle;
  pendingVehicleSinceMs = millis();
  lastPoseUpdateMs = pendingVehicleSinceMs;
  lastGCodeSentMs = pendingVehicleSinceMs;
  lastStopDecelMs = pendingVehicleSinceMs;
}

void clearRecordedGCode() {
  recordedGCodeCount = 0;
  cncOkResponsesPending = 0;
  replayModeActive = false;
  replayIndex = 0;
  printReplayActive = false;
  printReplayIndex = 0;
  printReplayWaitingForOk = false;
}

void recordStreamedLine(const char* line) {
  if (recordedGCodeCount == kRecordedGCodeCapacity) {
    for (size_t i = 1; i < recordedGCodeCount; ++i) {
      recordedGCode[i - 1] = recordedGCode[i];
    }
    --recordedGCodeCount;
  }

  RecordedGCodeLine& entry = recordedGCode[recordedGCodeCount++];
  snprintf(entry.line, sizeof(entry.line), "%s", line);
  entry.state = captureRecordedState();
}

void streamLine(const char* line, bool shouldRecord = true) {
  Serial2.println(line);
#if GCODE_USB_DEBUG
  Serial.printf("Speed: %d mm/s, Bearing: %d deg -> ", speedMmPerSec, bearingDeg);
#endif
  Serial.println(line);

  if (shouldRecord) {
    recordStreamedLine(line);
  }
}

void streamLines(const char* const* lines, size_t count, bool shouldRecord = true) {
  for (size_t i = 0; i < count; ++i) {
    streamLine(lines[i], shouldRecord);
  }
}

void streamCurrentPoseGCode() {
  const uint32_t nowMs = millis();
  const float dtSec = static_cast<float>(nowMs - lastPoseUpdateMs) / 1000.0F;
  Pose& pose = vehiclePoses[currentVehicle];

  // Treat speed as zero when within the deadband, consistent with processPoseOutput().
  // This prevents tiny positional drift and ensures the heading-only feedrate is applied
  // correctly when the speed encoder oscillates near zero.
  const bool effectivelyStationary =
      (speedMmPerSec <= kSpeedDeadbandMmPerSec && speedMmPerSec >= -kSpeedDeadbandMmPerSec);
  const float effectiveSpeedMmPerSec = effectivelyStationary ? 0.0F : static_cast<float>(speedMmPerSec);

  const float distanceMm = effectiveSpeedMmPerSec * dtSec;
  const float headingRad = toRadians(static_cast<float>(bearingDeg));
  pose.x -= distanceMm * pose.forward * sinf(headingRad);
  if(pose.x < 0) {
    pose.x = 0;
  } else if(pose.x > kBedSizeX) {
    pose.x = kBedSizeX;
  }
  pose.y += distanceMm * pose.forward * cosf(headingRad);
  if(pose.y < 0) {
    pose.y = 0;
  } else if(pose.y > kBedSizeY) {
    pose.y = kBedSizeY;
  }
  const float previousHeading = pose.heading;
  pose.heading = static_cast<float>(bearingDeg);

  const float updatesPerSecond = 1000.0F / static_cast<float>(kMaxGCodeRateMs);
  const float distancePerUpdateMm = fabsf(effectiveSpeedMmPerSec) / updatesPerSecond;
  float feedrateMmPerMin = distancePerUpdateMm * updatesPerSecond * 60.0F;
  if (effectivelyStationary && previousHeading != pose.heading) {
    feedrateMmPerMin = kHeadingOnlyFeedrateDegPerSec * 60.0F;
  }

  char gcodeLine[64];
  snprintf(gcodeLine, sizeof(gcodeLine), "G1 X%.3f Y%.3f Z%.3f F%.1f", pose.x, pose.y,
           pose.heading, feedrateMmPerMin);
  streamLine(gcodeLine);

  lastPoseUpdateMs = nowMs;
  lastGCodeSentMs = nowMs;
}

bool isCncOkResponse(const char* line) { return strncmp(line, "ok", 2) == 0; }

void processCncSerialInput() {
  while (Serial2.available() > 0) {
    const char ch = static_cast<char>(Serial2.read());

    if (ch == '\r' || ch == '\n') {
      if (cncRxLength > 0) {
        if (isCncOkResponse(cncRxBuffer)) {
          ++cncOkResponsesPending;
        }
#if GCODE_USB_DEBUG
        Serial.print("Marlin:");
        Serial.println(cncRxBuffer);
#endif
        cncRxLength = 0;
        cncRxBuffer[0] = '\0';
      }
      continue;
    }

    if (cncRxLength < (kCncRxBufferSize - 1)) {
      cncRxBuffer[cncRxLength++] = ch;
      cncRxBuffer[cncRxLength] = '\0';
    }
  }
}

void applyVehicleSelection(uint8_t vehicle) {
  currentVehicle = vehicle;
  speedMmPerSec = 0;
  poseDirty = false;

  streamLines(kVehicleDeselectGCode[currentVehicle], 3);
  bearingDeg = normalizeBearing(static_cast<int16_t>(vehiclePoses[currentVehicle].heading));
  lastPoseUpdateMs = millis();
  lastGCodeSentMs = lastPoseUpdateMs - kMaxGCodeRateMs;
  streamCurrentPoseGCode();
  streamLines(kVehicleSelectGCode[currentVehicle], 3);
}

uint8_t readVehicleSelectionRaw() {
  uint8_t selected = 0;
  for (uint8_t i = 0; i < kVehicleInputCount; ++i) {
    if (digitalRead(kVehicleInputs[i]) == LOW) {
      if (selected != 0) {
        return 0;
      }
      selected = static_cast<uint8_t>(i + 1);
    }
  }
  return selected;
}

void processVehicleSelection() {
  const uint8_t rawVehicle = readVehicleSelectionRaw();
  const uint32_t nowMs = millis();

  if (rawVehicle != pendingVehicle) {
    pendingVehicle = rawVehicle;
    pendingVehicleSinceMs = nowMs;
  }

  if (pendingVehicle != currentVehicle &&
      (nowMs - pendingVehicleSinceMs) >= kVehicleSettleMs) {
    applyVehicleSelection(pendingVehicle);
  }
}

bool isStopButtonPressed() { return digitalRead(kStopButtonInput) == LOW; }

void resumeNormalStreamingFromReplay() {
  if (!replayModeActive) {
    return;
  }

  if (recordedGCodeCount > 0) {
    restoreRecordedState(recordedGCode[replayIndex].state);
    recordedGCodeCount = min(recordedGCodeCount, replayIndex + 1);
  }

  replayModeActive = false;
  printReplayActive = false;
  printReplayIndex = 0;
  printReplayWaitingForOk = false;
  speedMmPerSec = 0;
}

void processEncoders() {
  if (!isStopButtonPressed()) {
    const int8_t speedDelta = speedEncoder.readDelta();
    if (speedDelta != 0) {
      if (replayModeActive) {
        resumeNormalStreamingFromReplay();
      }
      int8_t newSpeed = clampSpeed(static_cast<int32_t>(speedMmPerSec) + speedDelta);
      // Snap to zero when moving towards zero and landing within the deadband.
      // This allows speed to be set to exactly zero despite encoder noise near zero.
      const bool movingTowardsZero = (abs(newSpeed) < abs(speedMmPerSec));
      const bool withinDeadband =
          (newSpeed <= kSpeedDeadbandMmPerSec && newSpeed >= -kSpeedDeadbandMmPerSec);
      if (movingTowardsZero && withinDeadband) {
        newSpeed = 0;
      }
      if (newSpeed != speedMmPerSec) {
        speedMmPerSec = newSpeed;
        poseDirty = true;
      }
    }
  }

  const int8_t bearingDelta = bearingEncoder.readDelta();
  if (bearingDelta != 0) {
    if (replayModeActive) {
      resumeNormalStreamingFromReplay();
    }
    const int16_t newBearing = normalizeBearing(static_cast<int16_t>(bearingDeg) + bearingDelta);
    // Snap bearing to zero when a step towards zero would land within one degree.
    // This allows heading to be set to exactly zero (north) despite encoder noise.
    const int16_t snappedBearing =
        (abs(newBearing) < abs(bearingDeg) && abs(newBearing) <= 1) ? 0 : newBearing;
    if (snappedBearing != bearingDeg) {
      bearingDeg = snappedBearing;
      poseDirty = true;
    }
  }
}

void processStopButton() {
  if (!replayModeActive && isStopButtonPressed() && speedMmPerSec == 0 &&
      recordedGCodeCount > 0) {
    replayModeActive = true;
    replayIndex = recordedGCodeCount - 1;
    restoreRecordedState(recordedGCode[replayIndex].state);
    speedMmPerSec = 0;
    poseDirty = false;
  }

  if (!isStopButtonPressed() || speedMmPerSec == 0) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - lastStopDecelMs) < kStopDecelIntervalMs) {
    return;
  }

  int16_t newSpeed = speedMmPerSec;
  if (newSpeed > 0) {
    newSpeed = max<int16_t>(0, newSpeed - kStopDecelStepMmPerSec);
  } else {
    newSpeed = min<int16_t>(0, newSpeed + kStopDecelStepMmPerSec);
  }

  if (newSpeed != speedMmPerSec) {
    speedMmPerSec = static_cast<int8_t>(newSpeed);
    poseDirty = true;
  }
  lastStopDecelMs = nowMs;
}

void replayRecordedLine(size_t index) {
  if (index >= recordedGCodeCount) {
    return;
  }

  replayIndex = index;
  restoreRecordedState(recordedGCode[index].state);
  speedMmPerSec = 0;
  poseDirty = false;
  streamLine(recordedGCode[index].line, false);
}

void processReplayButtons() {
  if (!replayModeActive || printReplayActive || speedMmPerSec != 0 || recordedGCodeCount == 0) {
    return;
  }

  if (backwardButton.wasPressed() && replayIndex > 0) {
    replayRecordedLine(replayIndex - 1);
  }

  if (forwardButton.wasPressed() && (replayIndex + 1) < recordedGCodeCount) {
    replayRecordedLine(replayIndex + 1);
  }

  if (printButton.wasPressed()) {
    printReplayActive = true;
    printReplayIndex = 0;
    printReplayWaitingForOk = false;
    cncOkResponsesPending = 0;
  }
}

void processPrintReplay() {
  if (!printReplayActive || recordedGCodeCount == 0) {
    return;
  }

  if (printReplayWaitingForOk) {
    if (cncOkResponsesPending == 0) {
      return;
    }
    --cncOkResponsesPending;
    printReplayWaitingForOk = false;
  }

  if (printReplayIndex >= recordedGCodeCount) {
    printReplayActive = false;
    return;
  }

  replayRecordedLine(printReplayIndex++);
  printReplayWaitingForOk = true;
}

void processPoseOutput() {
  if (currentVehicle == 0 || replayModeActive) {
    return;
  }

  const bool isMoving =
      (speedMmPerSec > kSpeedDeadbandMmPerSec || speedMmPerSec < -kSpeedDeadbandMmPerSec);

  if (!poseDirty && !isMoving) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - lastGCodeSentMs) < kMaxGCodeRateMs) {
    return;
  }

  streamCurrentPoseGCode();
  poseDirty = false;
}

}  // namespace

void setup() {
  Serial.begin(kUsbSerialBaudRate);
  Serial2.begin(250000, SERIAL_8N1, kCncSerialRx, kCncSerialTx);

  for (uint8_t i = 0; i < kVehicleCount; ++i) {
    vehiclePoses[i] = kInitialVehiclePoses[i];
  }

  clearRecordedGCode();
  streamLines(kInitGCode, sizeof(kInitGCode) / sizeof(kInitGCode[0]));

  speedEncoder.begin();
  bearingEncoder.begin();
  pinMode(kStopButtonInput, INPUT_PULLUP);
  forwardButton.begin();
  backwardButton.begin();
  printButton.begin();

  for (uint8_t pin : kVehicleInputs) {
    pinMode(pin, INPUT_PULLUP);
  }

  pendingVehicle = readVehicleSelectionRaw();
  pendingVehicleSinceMs = millis();
  lastPoseUpdateMs = millis();
  lastGCodeSentMs = lastPoseUpdateMs;
  lastStopDecelMs = lastPoseUpdateMs;
}

void loop() {
  processCncSerialInput();
  processVehicleSelection();
  processEncoders();
  processStopButton();
  processReplayButtons();
  processPrintReplay();
  processPoseOutput();
}
