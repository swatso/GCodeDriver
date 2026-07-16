#include <Arduino.h>

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
constexpr uint16_t kMaxGCodeRateMs = 1000;  // Pose settle window after encoder activity
constexpr int8_t kSpeedDeadbandMmPerSec = 3;  // speeds within [-deadband, +deadband] are treated as stopped
constexpr float kHeadingOnlyFeedrateDegPerSec = 50.0F;
constexpr float kNudgeStepMm = 1.0F;
constexpr float kNudgeMaxStepMm = 20.0F;     // Hard limit on nudge step size to prevent runaway if encoder is turned too fast
constexpr uint32_t kNudgeRampWindowMs = 250;   // Time window for ramping up nudge step size based on encoder speed
constexpr int16_t kNudgeMaxMultiplier = 20;   // Maximum multiplier for nudge step size based on encoder speed
constexpr uint32_t kSpeedNudgeDebounceMs = 50;  // Ignore back-to-back speed encoder edges that are too close together
constexpr float kPoseXMin = 0.0F;
constexpr float kPoseXMax = 265.0F;
constexpr float kPoseYMin = 0.0F;
constexpr float kPoseYMax = 225.0F;
constexpr uint32_t kVehicleEncSettleMs = 5000;

// GPIO mappings.
constexpr uint8_t kSpeedEncoderA = 13;          // Bit 00
constexpr uint8_t kSpeedEncoderB = 12;          // Bit 01
constexpr uint8_t kBearingEncoderA = 4;         // Bit 02
constexpr uint8_t kBearingEncoderB = 14;        // Bit 03
constexpr uint8_t kVehicleEnc1 = 16;            // Bit 04 
constexpr uint8_t kVehicleEnc2 = 27;            // Bit 05
constexpr uint8_t kVehicleEnc4 = 17;            // Bit 06
constexpr uint8_t kVehicleEnc8 = 26;            // Bit 07
constexpr uint8_t kNudge = 25;                  // Bit 08
constexpr uint8_t kStopButtonInput = 32;        // Bit 0B
constexpr uint8_t kSetButtonInput = 19;      // Bit 0A   (using the Replay button on the panel atm)
constexpr uint8_t kBackButtonInput = 18;        // Bit 09
constexpr uint8_t kForwardButtonInput = 21;     // Bit 0C
constexpr uint8_t kGimbleLock = 22;             // Bit 0D
constexpr uint8_t kOrthogonal = 23;             // Bit 0E

// GPIO 33   Bit 0F
constexpr uint16_t kStopDecelIntervalMs = 100;
constexpr int8_t kStopDecelStepMmPerSec = 30;
constexpr int8_t kSpeedEncoderTransitionsPerStep = 2;
constexpr int8_t kBearingEncoderTransitionsPerDegree = kSpeedEncoderTransitionsPerStep;
constexpr bool kBearingDirectionInverted = false;
constexpr uint32_t kUsbSerialBaudRate = 115200;
constexpr uint16_t kStopSetDebounceMs = 40;
constexpr size_t kWaypointBufferSize = 64;

struct PoseSnapshot {
  float x;
  float y;
  float bearing;
  uint32_t timestampMs;
};

struct DebouncedActiveLowButton {
  uint8_t pin = 0;
  bool stablePressed = false;
  bool lastRawPressed = false;
  uint32_t lastRawChangeMs = 0;

  void begin(uint8_t inputPin) {
    pin = inputPin;
    const bool pressed = (digitalRead(pin) == LOW);
    stablePressed = pressed;
    lastRawPressed = pressed;
    lastRawChangeMs = millis();
  }

  bool consumePressedEdge(uint32_t nowMs) {
    const bool rawPressed = (digitalRead(pin) == LOW);
    if (rawPressed != lastRawPressed) {
      lastRawPressed = rawPressed;
      lastRawChangeMs = nowMs;
    }

    if ((nowMs - lastRawChangeMs) < kStopSetDebounceMs) {
      return false;
    }

    if (stablePressed != rawPressed) {
      stablePressed = rawPressed;
      return stablePressed;
    }
    return false;
  }
};

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

Encoder speedEncoder;

int8_t speedMmPerSec = 0;
int16_t bearingDeg = 0;

bool poseDirty = false;
uint32_t lastGCodeSentMs = 0;
uint32_t lastPoseChangeMs = 0;
uint32_t lastStopDecelMs = 0;
uint8_t lastBearingAState = HIGH;
int8_t bearingEdgeDirectionSum = 0;
int16_t pendingBearingDeltaDeg = 0;
float pendingNudgeDeltaX = 0.0F;
float pendingNudgeDeltaY = 0.0F;
int currentEncValue = -1;
uint8_t pendingEncValue = 0;
uint32_t pendingEncSinceMs = 0;
bool vehicleEncSawChangeSinceBoot = false;
uint32_t lastNudgeXEventMs = 0;
uint32_t lastNudgeYEventMs = 0;
PoseSnapshot waypointBuffer[kWaypointBufferSize] = {};
size_t waypointWriteIndex = 0;
size_t waypointCount = 0;
size_t waypointReadOffsetFromNewest = 0;
DebouncedActiveLowButton stopSetButton;
DebouncedActiveLowButton backButton;
DebouncedActiveLowButton forwardButton;

float toRadians(float deg) { return deg * DEG_TO_RAD; }

bool isNudgeActive() { return digitalRead(kNudge) == LOW; }

bool isSpeedEffectivelyStationary() {
  return speedMmPerSec <= kSpeedDeadbandMmPerSec &&
         speedMmPerSec >= -kSpeedDeadbandMmPerSec;
}

uint8_t readVehicleEncoderRawValue() {
  uint8_t value = 0;
  if (digitalRead(kVehicleEnc1) == LOW) {
    value |= 0x01;
  }
  if (digitalRead(kVehicleEnc2) == LOW) {
    value |= 0x02;
  }
  if (digitalRead(kVehicleEnc4) == LOW) {
    value |= 0x04;
  }
  if (digitalRead(kVehicleEnc8) == LOW) {
    value |= 0x08;
  }
  return value;
}

void processVehicleEncoderSwitch() {
  const uint32_t nowMs = millis();
  const uint8_t rawValue = readVehicleEncoderRawValue();

  if (rawValue != pendingEncValue) {
    pendingEncValue = rawValue;
    pendingEncSinceMs = nowMs;
    vehicleEncSawChangeSinceBoot = true;
  }
  if (vehicleEncSawChangeSinceBoot && pendingEncValue != currentEncValue &&
      (nowMs - pendingEncSinceMs) >= kVehicleEncSettleMs) {
    currentEncValue = pendingEncValue;
    Serial.printf("currentEncValue: %d\n", currentEncValue);
    publishCurrentEncValue(currentEncValue);
  }
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

int16_t clampBearing(int16_t value) {
  if (value < 0) {
    return 0;
  }
  if (value > 360) {
    return 360;
  }
  return value;
}

bool isWithinFiveDegrees(float bearingDegValue, int16_t targetDeg) {
  return fabsf(bearingDegValue - static_cast<float>(targetDeg)) <= 5.0F;
}

void processBearingLeds() {
  if (!receivedPose.valid) {
    digitalWrite(kGimbleLock, HIGH);
    digitalWrite(kOrthogonal, HIGH);
    return;
  }

  const float bearing = receivedPose.bearing;
  const bool gimbleLockOn = (bearing == 0.0F || bearing == 360.0F);
  const bool orthogonalOn = isWithinFiveDegrees(bearing, 0) ||
                            isWithinFiveDegrees(bearing, 90) ||
                            isWithinFiveDegrees(bearing, 180) ||
                            isWithinFiveDegrees(bearing, 270) ||
                            isWithinFiveDegrees(bearing, 360);

  // Active-low outputs: LOW turns LED on.
  digitalWrite(kGimbleLock, gimbleLockOn ? LOW : HIGH);
  digitalWrite(kOrthogonal, orthogonalOn ? LOW : HIGH);
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

float computeNudgeDeltaMm(int8_t encoderDelta, uint32_t& lastEventMs, uint32_t nowMs) {
  if (encoderDelta == 0) {
    return 0.0F;
  }

  // Faster consecutive detents increase nudge size; slow turns stay at 1 mm.
  const uint32_t elapsedMs =
      (lastEventMs == 0) ? kNudgeRampWindowMs : (nowMs - lastEventMs);
  lastEventMs = nowMs;

  const uint32_t clampedElapsedMs = min(elapsedMs, kNudgeRampWindowMs);
  const int16_t multiplier =
      1 + static_cast<int16_t>(((kNudgeRampWindowMs - clampedElapsedMs) *
                                static_cast<uint32_t>(kNudgeMaxMultiplier - 1)) /
                               kNudgeRampWindowMs);

  float requested = kNudgeStepMm * static_cast<float>(abs(encoderDelta) * multiplier);
  requested = min(requested, kNudgeMaxStepMm);
  return (encoderDelta < 0) ? -requested : requested;
}

void streamLine(const char* line, bool includeUsbDebug = true) {
  publishMQTT(GCodeTopic, (char *)line);
#if GCODE_USB_DEBUG
  if (includeUsbDebug) {
    Serial.printf("Speed: %d mm/s, Bearing: %d deg -> ", speedMmPerSec, bearingDeg);
  }
#endif
  Serial.println(line);
}

void streamCurrentPoseGCode() {
  const uint32_t nowMs = millis();
  const bool nudgeActive = isNudgeActive();
  const float dtSec = static_cast<float>(nowMs - lastGCodeSentMs) / 1000.0F;
  bool limitHit = false;

  // Treat speed as zero when within the deadband, consistent with processPoseOutput().
  const bool effectivelyStationary = isSpeedEffectivelyStationary();
  const float effectiveSpeedMmPerSec = effectivelyStationary ? 0.0F : static_cast<float>(speedMmPerSec);

  float deltaX = 0.0F;
  float deltaY = 0.0F;
  float deltaZ = 0.0F;
  float feedrateMmPerMin = 0.0F;

  if (nudgeActive) {
    deltaX = pendingNudgeDeltaX;
    deltaY = pendingNudgeDeltaY;
    feedrateMmPerMin = kHeadingOnlyFeedrateDegPerSec * 60.0F;
  } else {
    const float distanceMm = effectiveSpeedMmPerSec * dtSec;
    // Use the authoritative bearing from the received pose when available;
    // fall back to locally tracked bearing until first update arrives.
    const float headingRad = toRadians(receivedPose.valid ? receivedPose.bearing
                                                           : static_cast<float>(bearingDeg));
    deltaX = -distanceMm * sinf(headingRad);
    deltaY = distanceMm * cosf(headingRad);
    deltaZ = static_cast<float>(pendingBearingDeltaDeg);

    if (receivedPose.valid) {
      const float unclampedX = receivedPose.x + deltaX;
      const float unclampedY = receivedPose.y + deltaY;
      const float clampedX = clampFloat(unclampedX, kPoseXMin, kPoseXMax);
      const float clampedY = clampFloat(unclampedY, kPoseYMin, kPoseYMax);

      limitHit = (clampedX != unclampedX) || (clampedY != unclampedY);
      deltaX = clampedX - receivedPose.x;
      deltaY = clampedY - receivedPose.y;

      // Keep local pose tracking aligned with the clamped output pose.
      receivedPose.x = clampedX;
      receivedPose.y = clampedY;
    }

    feedrateMmPerMin = fabsf(effectiveSpeedMmPerSec) * 60.0F;
    if (effectivelyStationary && pendingBearingDeltaDeg != 0) {
      feedrateMmPerMin = kHeadingOnlyFeedrateDegPerSec * 60.0F;
    }
  }

  if (limitHit && speedMmPerSec != 0) {
    speedMmPerSec = 0;
    poseDirty = true;
    lastPoseChangeMs = nowMs;
  }

  char gcodeLine[64];
  snprintf(gcodeLine, sizeof(gcodeLine), "G1 X%.3f Y%.3f Z%.3f F%.1f", deltaX, deltaY,
           deltaZ, feedrateMmPerMin);
  streamLine(gcodeLine);
  lastGCodeSentMs = nowMs;
  pendingBearingDeltaDeg = 0;
  pendingNudgeDeltaX = 0.0F;
  pendingNudgeDeltaY = 0.0F;
}

bool isStopButtonPressed() { return digitalRead(kStopButtonInput) == LOW; }

size_t getWaypointBufferIndexFromNewestOffset(size_t offsetFromNewest) {
  // waypointWriteIndex always points to the next write slot.
  return (waypointWriteIndex + kWaypointBufferSize - 1U - offsetFromNewest) % kWaypointBufferSize;
}

bool getWaypointByOffsetFromNewest(size_t offsetFromNewest, PoseSnapshot& outPose) {
  if (waypointCount == 0 || offsetFromNewest >= waypointCount) {
    return false;
  }
  const size_t index = getWaypointBufferIndexFromNewestOffset(offsetFromNewest);
  outPose = waypointBuffer[index];
  return true;
}

void publishWaypointSnapshot(const PoseSnapshot& pose, const char* sourceLabel,
                             bool publishToGCodeTopic = false) {
  const bool published = publishToGCodeTopic
                             ? publishWaypointAndGCodePose(pose.x, pose.y, pose.bearing)
                             : publishWaypointPose(pose.x, pose.y, pose.bearing);
  Serial.printf("%s waypoint: G1 X%.3f Y%.3f Z%.3f (%s) readOffset=%u count=%u\n",
                sourceLabel,
                pose.x,
                pose.y,
                pose.bearing,
                published ? "published" : "publish failed",
                static_cast<unsigned>(waypointReadOffsetFromNewest),
                static_cast<unsigned>(waypointCount));
}

void resetWaypointWritePointerAfterReadOffset() {
  if (waypointCount == 0 || waypointReadOffsetFromNewest == 0) {
    return;
  }

  // If we're reading older entries, branch from that point by discarding newer history.
  if (waypointReadOffsetFromNewest >= waypointCount) {
    waypointReadOffsetFromNewest = waypointCount - 1U;
  }
  waypointWriteIndex =
      (waypointWriteIndex + kWaypointBufferSize - waypointReadOffsetFromNewest) %
      kWaypointBufferSize;
  waypointCount -= waypointReadOffsetFromNewest;
  waypointReadOffsetFromNewest = 0;
}

void storeWaypointPose(float x, float y, float bearing, uint32_t timestampMs) {
  waypointBuffer[waypointWriteIndex] = PoseSnapshot{x, y, bearing, timestampMs};
  waypointWriteIndex = (waypointWriteIndex + 1U) % kWaypointBufferSize;
  if (waypointCount < kWaypointBufferSize) {
    waypointCount++;
  }
}

void processStopSetButton() {
  const uint32_t nowMs = millis();
  if (!stopSetButton.consumePressedEdge(nowMs)) {
    return;
  }

  if (!receivedPose.valid) {
    Serial.println("Stop-set pressed but no valid pose available");
    return;
  }

  storeWaypointPose(receivedPose.x, receivedPose.y, receivedPose.bearing, nowMs);
  waypointReadOffsetFromNewest = 0;
  PoseSnapshot newest = {};
  if (getWaypointByOffsetFromNewest(waypointReadOffsetFromNewest, newest)) {
    publishWaypointSnapshot(newest, "Set");
  }
}

void processBackButton() {
  const uint32_t nowMs = millis();
  if (!backButton.consumePressedEdge(nowMs)) {
    return;
  }

  if (waypointCount == 0) {
    Serial.println("Back pressed but waypoint buffer is empty");
    return;
  }

  if ((waypointReadOffsetFromNewest + 1U) < waypointCount) {
    waypointReadOffsetFromNewest++;
  }

  PoseSnapshot pose = {};
  if (getWaypointByOffsetFromNewest(waypointReadOffsetFromNewest, pose)) {
    publishWaypointSnapshot(pose, "Back", true);
  }
}

void processForwardButton() {
  const uint32_t nowMs = millis();
  if (!forwardButton.consumePressedEdge(nowMs)) {
    return;
  }

  if (waypointCount == 0) {
    Serial.println("Forward pressed but waypoint buffer is empty");
    return;
  }

  if (waypointReadOffsetFromNewest > 0) {
    waypointReadOffsetFromNewest--;
  }

  PoseSnapshot pose = {};
  if (getWaypointByOffsetFromNewest(waypointReadOffsetFromNewest, pose)) {
    publishWaypointSnapshot(pose, "Forward", true);
  }
}

void processEncoders() {
  const uint32_t nowMs = millis();
  const bool nudgeActive = isNudgeActive();
  const int8_t speedDelta = speedEncoder.readDelta();
  const bool speedNudgeDebounced =
      nudgeActive && speedDelta != 0 && receivedPose.valid && lastNudgeXEventMs != 0 &&
      (nowMs - lastNudgeXEventMs) < kSpeedNudgeDebounceMs;

  if (!nudgeActive) {
    lastNudgeXEventMs = 0;
    lastNudgeYEventMs = 0;
  }

  if (nudgeActive && speedDelta != 0 && receivedPose.valid && !speedNudgeDebounced) {
    const float xNudgeDeltaMm = computeNudgeDeltaMm(speedDelta, lastNudgeXEventMs, nowMs);
    const float nextX = clampFloat(receivedPose.x - xNudgeDeltaMm, kPoseXMin, kPoseXMax);
    const float appliedDeltaX = nextX - receivedPose.x;
    if (appliedDeltaX != 0.0F) {
      receivedPose.x = nextX;
      pendingNudgeDeltaX += appliedDeltaX;
      poseDirty = true;
        lastPoseChangeMs = nowMs;
    }
  } else if (!nudgeActive && speedDelta != 0 && !isStopButtonPressed()) {
    int8_t newSpeed = clampSpeed(static_cast<int32_t>(speedMmPerSec) + speedDelta);
    // Snap to zero when moving towards zero and landing within the deadband.
    const bool movingTowardsZero = (abs(newSpeed) < abs(speedMmPerSec));
    const bool withinDeadband =
        (newSpeed <= kSpeedDeadbandMmPerSec && newSpeed >= -kSpeedDeadbandMmPerSec);
    if (movingTowardsZero && withinDeadband) {
      newSpeed = 0;
    }
    if (newSpeed != speedMmPerSec) {
      speedMmPerSec = newSpeed;
      poseDirty = true;
      lastPoseChangeMs = nowMs;
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
    if (nudgeActive && receivedPose.valid) {
      const float yNudgeDeltaMm = computeNudgeDeltaMm(bearingDelta, lastNudgeYEventMs, nowMs);
      const float nextY = clampFloat(receivedPose.y + yNudgeDeltaMm, kPoseYMin, kPoseYMax);
      const float appliedDeltaY = nextY - receivedPose.y;
      if (appliedDeltaY != 0.0F) {
        receivedPose.y = nextY;
        pendingNudgeDeltaY += appliedDeltaY;
        poseDirty = true;
        lastPoseChangeMs = nowMs;
      }
    } else if (receivedPose.valid) {
      // Apply encoder delta to the authoritative received bearing.
      const float newBearing = static_cast<float>(
          clampBearing(static_cast<int16_t>(receivedPose.bearing) + bearingDelta));
      if (newBearing != receivedPose.bearing) {
        receivedPose.bearing = newBearing;
        pendingBearingDeltaDeg += bearingDelta;
        poseDirty = true;
        lastPoseChangeMs = nowMs;
      }
    } else {
      // No received pose yet; update the local fallback bearing.
      const int16_t newBearing = clampBearing(static_cast<int16_t>(bearingDeg) + bearingDelta);
      if (newBearing != bearingDeg) {
        bearingDeg = newBearing;
        pendingBearingDeltaDeg += bearingDelta;
        poseDirty = true;
        lastPoseChangeMs = nowMs;
      }
    }
  }
}

void processStopButton() {
  if (isNudgeActive()) {
    return;
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
    lastPoseChangeMs = nowMs;
  }
  lastStopDecelMs = nowMs;
}

void processPoseOutput() {
  const uint32_t nowMs = millis();
  const bool periodicDriveUpdateDue =
      !isNudgeActive() && !isSpeedEffectivelyStationary() &&
      (nowMs - lastGCodeSentMs) >= kMaxGCodeRateMs;

  if (!poseDirty && !periodicDriveUpdateDue) {
    return;
  }

  if (poseDirty && !periodicDriveUpdateDue && (nowMs - lastPoseChangeMs) < kMaxGCodeRateMs) {
    return;
  }

  streamCurrentPoseGCode();
  poseDirty = false;
}

}  // namespace

void setup() {
  Serial.begin(kUsbSerialBaudRate);
  speedEncoder.configure(kSpeedEncoderA, kSpeedEncoderB, kSpeedEncoderTransitionsPerStep);
  speedEncoder.begin();
  pinMode(kBearingEncoderA, INPUT_PULLUP);
  pinMode(kBearingEncoderB, INPUT_PULLUP);
  lastBearingAState = static_cast<uint8_t>(digitalRead(kBearingEncoderA));
  pinMode(kVehicleEnc1, INPUT_PULLUP);
  pinMode(kVehicleEnc2, INPUT_PULLUP);
  pinMode(kVehicleEnc4, INPUT_PULLUP);
  pinMode(kVehicleEnc8, INPUT_PULLUP);
  pinMode(kNudge, INPUT_PULLUP);
  pinMode(kStopButtonInput, INPUT_PULLUP);
  pinMode(kSetButtonInput, INPUT_PULLUP);
  pinMode(kBackButtonInput, INPUT_PULLUP);
  pinMode(kForwardButtonInput, INPUT_PULLUP);
  pinMode(kGimbleLock, OUTPUT);
  pinMode(kOrthogonal, OUTPUT);
  digitalWrite(kGimbleLock, HIGH);
  digitalWrite(kOrthogonal, HIGH);

  setupSPIFFS();
  node.loadConfig();
  setupWiFi();
  setupMQTTComms();

  lastGCodeSentMs = millis();
  lastPoseChangeMs = lastGCodeSentMs;
  lastStopDecelMs = lastGCodeSentMs;
  currentEncValue = -1;
  pendingEncValue = readVehicleEncoderRawValue();
  pendingEncSinceMs = millis();
  vehicleEncSawChangeSinceBoot = false;
  stopSetButton.begin(kSetButtonInput);
  backButton.begin(kBackButtonInput);
  forwardButton.begin(kForwardButtonInput);
}

void loop() {
  //processCncSerialInput();
  processVehicleEncoderSwitch();
  processEncoders();
  processStopButton();
  processStopSetButton();
  processBackButton();
  processForwardButton();
  processPoseOutput();
  processBearingLeds();

}
