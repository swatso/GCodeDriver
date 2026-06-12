#include <Arduino.h>

#include <cstring>

#include "vehicle_config.h"
#include "FileSystem.h"
#include "MQTTComms.h"
#include "NodeServices.h"
#include "WiFiManager.h"

#ifndef GCODE_USB_DEBUG
#define GCODE_USB_DEBUG 1
#endif

#ifndef MARLIN_USB_DEBUG
#define MARLIN_USB_DEBUG 1
#endif

namespace {
constexpr int8_t kSpeedMin = -30;
constexpr int8_t kSpeedMax = 30;
constexpr uint16_t kVehicleSettleMs = 3000;
constexpr uint16_t kMaxGCodeRateMs = 1000;  // 1 Hz
constexpr int8_t kSpeedDeadbandMmPerSec = 3;  // speeds within [-deadband, +deadband] are treated as stopped
constexpr float kHeadingOnlyFeedrateDegPerSec = 50.0F;

// GPIO mappings.
constexpr uint8_t kSpeedEncoderA = 13;          // Bit 00
constexpr uint8_t kSpeedEncoderB = 12;          // Bit 01
constexpr uint8_t kBearingEncoderA = 4;         // Bit 02
constexpr uint8_t kBearingEncoderB = 14;        // Bit 03
// GPIO 16 Bit 04  - use for vehicle selection
// GPIO 27 Bit 05  - use for vehicle selection
// GPIO 17 Bit 06  - use for vehicle selection
// GPIO 26 Bit 07  - use for vehicle selection
// GPIO 25 Bit 08
constexpr uint8_t kBackwardButtonInput = 18;    // Bit 09
constexpr uint8_t kPrintButtonInput = 19;       // Bit 0A
constexpr uint8_t kStopButtonInput = 32;        // Bit 0B
constexpr uint8_t kForwardButtonInput = 21;     // Bit 0C
constexpr uint8_t kGimbleLock = 22;              // Bit 0D
constexpr uint8_t kOrthogonal = 23;             // Bit 0E
// GPIO 33   Bit 0F



constexpr uint8_t kVehicleInputs[kVehicleInputCount] = {16, 27, 17, 26, 25};
constexpr uint16_t kStopDecelIntervalMs = 100;
constexpr int8_t kStopDecelStepMmPerSec = 30;
constexpr int8_t kSpeedEncoderTransitionsPerStep = 2;
constexpr int8_t kBearingEncoderTransitionsPerDegree = kSpeedEncoderTransitionsPerStep;
constexpr bool kBearingDirectionInverted = false;
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
  int8_t stepsPerDetent;  // raw quadrature transitions required to output ±1
  int8_t accumulator = 0;

  void configure(uint8_t newPinA, uint8_t newPinB, int8_t newStepsPerDetent) {
    pinA = newPinA;
    pinB = newPinB;
    stepsPerDetent = newStepsPerDetent;
  }

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
    accumulator += transitionTable[index];
    if (accumulator >= stepsPerDetent) {
      const int8_t clicks = accumulator / stepsPerDetent;
      accumulator %= stepsPerDetent;
      return -clicks;
    } else if (accumulator <= -stepsPerDetent) {
      const int8_t clicks = accumulator / stepsPerDetent;
      accumulator %= stepsPerDetent;
      return -clicks;
    }
    return 0;
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

Encoder speedEncoder;
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
uint8_t lastBearingAState = HIGH;
int8_t bearingEdgeDirectionSum = 0;
constexpr size_t kCncRxBufferSize = 96;
char cncRxBuffer[kCncRxBufferSize] = {};
size_t cncRxLength = 0;

RecordedGCodeLine recordedGCode[kRecordedGCodeCapacity] = {};
size_t recordedGCodeCount = 0;
bool replayModeActive = false;
size_t replayIndex = 0;
bool printReplayActive = false;
size_t printReplayIndex = 0;
bool printReplayWaitingForOk = false;

float toRadians(float deg) { return deg * DEG_TO_RAD; }

int16_t clampBearing(int16_t value) {
  // Clamps to 0-360 and warns that the gimble lock is in action by lighting the LED
  // The gimble lock prevents the data cable to the puck from being twisted too much when the bearing is near the wraparound point. 
  // The LED will light to warn the user that the gimble is at the limit of rotation
  if(value < 0)
  {
    value = 0;
    digitalWrite(kGimbleLock, LOW);
  }
  else if(value > 360)
  {
    value = 360;
    digitalWrite(kGimbleLock, LOW);
  }
  else
  {
    digitalWrite(kGimbleLock, HIGH);
  }
  // Drive the orthogonal lock LED when near the cardinal directions to help the user align to them, which is a common use case 
  digitalWrite(kOrthogonal, HIGH);
  if((value > 85) && (value < 95))digitalWrite(kOrthogonal, LOW);
  else if((value > 175) && (value < 185))digitalWrite(kOrthogonal, LOW);
  else if((value > 265) && (value < 275))digitalWrite(kOrthogonal, LOW);
  else if((value > 355) || (value < 5))digitalWrite(kOrthogonal, LOW);
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

void streamLine(const char* line, bool shouldRecord = true, bool includeUsbDebug = true) {
//  Serial2.println(line);
  publishMQTT(GCodeTopic, (char *)line);
#if GCODE_USB_DEBUG
  if (includeUsbDebug) {
    Serial.printf("Speed: %d mm/s, Bearing: %d deg -> ", speedMmPerSec, bearingDeg);
  }
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

void applyVehicleSelection(uint8_t vehicle) {
  currentVehicle = vehicle;
  speedMmPerSec = 0;
  poseDirty = false;

  streamLines(kVehicleDeselectGCode[currentVehicle], 3);
  bearingDeg = clampBearing(static_cast<int16_t>(vehiclePoses[currentVehicle].heading));
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

  int8_t bearingDelta = 0;
  const uint8_t bearingAState = static_cast<uint8_t>(digitalRead(kBearingEncoderA));
  if (bearingAState != lastBearingAState) {
    const uint8_t bearingBState = static_cast<uint8_t>(digitalRead(kBearingEncoderB));
    // On each A edge, B phase gives direction; accumulate until one logical step is reached.
    int8_t edgeDirection = (bearingAState != bearingBState) ? 1 : -1;
    if (kBearingDirectionInverted) {
      edgeDirection = -edgeDirection;
    }

    bearingEdgeDirectionSum += edgeDirection;
    if (abs(bearingEdgeDirectionSum) >= kBearingEncoderTransitionsPerDegree) {
      bearingDelta = (bearingEdgeDirectionSum > 0) ? 1 : -1;
      bearingEdgeDirectionSum = 0;
    }
    lastBearingAState = bearingAState;
  }

  if (bearingDelta != 0) {
    if (replayModeActive) {
      resumeNormalStreamingFromReplay();
    }
    const int16_t newBearing = clampBearing(static_cast<int16_t>(bearingDeg) + bearingDelta);
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

void replayRecordedLine(size_t index, bool includeUsbDebug = true) {
  if (index >= recordedGCodeCount) {
    return;
  }

  replayIndex = index;
  restoreRecordedState(recordedGCode[index].state);
  speedMmPerSec = 0;
  poseDirty = false;
  streamLine(recordedGCode[index].line, false, includeUsbDebug);
}

void processReplayButtons() {
  if (!replayModeActive || printReplayActive || speedMmPerSec != 0 || recordedGCodeCount == 0 ) {
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
  }
}

void processPrintReplay() {
  if (!printReplayActive || recordedGCodeCount == 0) {
    return;
  }

  if (printReplayIndex >= recordedGCodeCount) {
    printReplayActive = false;
    return;
  }

  replayRecordedLine(printReplayIndex++, false);
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
  for (uint8_t i = 0; i < kVehicleCount; ++i) {
    vehiclePoses[i] = kInitialVehiclePoses[i];
  }

  clearRecordedGCode();
  streamLines(kInitGCode, sizeof(kInitGCode) / sizeof(kInitGCode[0]));
  pinMode(kGimbleLock, OUTPUT);
  digitalWrite(kGimbleLock, HIGH);
  pinMode(kOrthogonal, OUTPUT);
  digitalWrite(kOrthogonal, HIGH);
  speedEncoder.configure(kSpeedEncoderA, kSpeedEncoderB, kSpeedEncoderTransitionsPerStep);
  speedEncoder.begin();
  pinMode(kBearingEncoderA, INPUT_PULLUP);
  pinMode(kBearingEncoderB, INPUT_PULLUP);
  lastBearingAState = static_cast<uint8_t>(digitalRead(kBearingEncoderA));
  pinMode(kStopButtonInput, INPUT_PULLUP);
  forwardButton.begin();
  backwardButton.begin();
  printButton.begin();

  for (uint8_t pin : kVehicleInputs) {
    pinMode(pin, INPUT_PULLUP);
  }

  setupSPIFFS();
  node.loadConfig();
  setupWiFi();
  setupMQTTComms();

  pendingVehicle = readVehicleSelectionRaw();
  pendingVehicleSinceMs = millis();
  lastPoseUpdateMs = millis();
  lastGCodeSentMs = lastPoseUpdateMs;
  lastStopDecelMs = lastPoseUpdateMs;
}

void loop() {
  //processCncSerialInput();
  processVehicleSelection();
  processEncoders();
  processStopButton();
  processReplayButtons();
  processPrintReplay();
  processPoseOutput();

}
