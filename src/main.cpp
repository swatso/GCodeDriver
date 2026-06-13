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
constexpr uint16_t kMaxGCodeRateMs = 1000;  // 1 Hz
constexpr int8_t kSpeedDeadbandMmPerSec = 3;  // speeds within [-deadband, +deadband] are treated as stopped
constexpr float kHeadingOnlyFeedrateDegPerSec = 50.0F;
constexpr float kNudgeStepMm = 1.0F;
constexpr float kPoseXMin = 0.0F;
constexpr float kPoseXMax = 265.0F;
constexpr float kPoseYMin = 0.0F;
constexpr float kPoseYMax = 225.0F;

// GPIO mappings.
constexpr uint8_t kSpeedEncoderA = 13;          // Bit 00
constexpr uint8_t kSpeedEncoderB = 12;          // Bit 01
constexpr uint8_t kBearingEncoderA = 4;         // Bit 02
constexpr uint8_t kBearingEncoderB = 14;        // Bit 03


constexpr uint8_t kNudge = 25;                  // Bit 08
constexpr uint8_t kStopButtonInput = 32;        // Bit 0B

constexpr uint8_t kGimbleLock = 22;              // Bit 0D
constexpr uint8_t kOrthogonal = 23;             // Bit 0E

// GPIO 33   Bit 0F
constexpr uint16_t kStopDecelIntervalMs = 100;
constexpr int8_t kStopDecelStepMmPerSec = 30;
constexpr int8_t kSpeedEncoderTransitionsPerStep = 2;
constexpr int8_t kBearingEncoderTransitionsPerDegree = kSpeedEncoderTransitionsPerStep;
constexpr bool kBearingDirectionInverted = false;
constexpr uint32_t kUsbSerialBaudRate = 115200;

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
uint32_t lastStopDecelMs = 0;
uint8_t lastBearingAState = HIGH;
int8_t bearingEdgeDirectionSum = 0;
int16_t pendingBearingDeltaDeg = 0;
float pendingNudgeDeltaX = 0.0F;
float pendingNudgeDeltaY = 0.0F;

float toRadians(float deg) { return deg * DEG_TO_RAD; }

bool isNudgeActive() { return digitalRead(kNudge) == LOW; }

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

  // Treat speed as zero when within the deadband, consistent with processPoseOutput().
  const bool effectivelyStationary =
      (speedMmPerSec <= kSpeedDeadbandMmPerSec && speedMmPerSec >= -kSpeedDeadbandMmPerSec);
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

    feedrateMmPerMin = fabsf(effectiveSpeedMmPerSec) * 60.0F;
    if (effectivelyStationary && pendingBearingDeltaDeg != 0) {
      feedrateMmPerMin = kHeadingOnlyFeedrateDegPerSec * 60.0F;
    }
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

void processEncoders() {
  const bool nudgeActive = isNudgeActive();
  const int8_t speedDelta = speedEncoder.readDelta();
  if (nudgeActive && speedDelta != 0 && receivedPose.valid) {
    const float nextX = clampFloat(receivedPose.x - (kNudgeStepMm * static_cast<float>(speedDelta)),
                                   kPoseXMin, kPoseXMax);
    const float appliedDeltaX = nextX - receivedPose.x;
    if (appliedDeltaX != 0.0F) {
      receivedPose.x = nextX;
      pendingNudgeDeltaX += appliedDeltaX;
      poseDirty = true;
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
      const float nextY = clampFloat(receivedPose.y + (kNudgeStepMm * static_cast<float>(bearingDelta)),
                                     kPoseYMin, kPoseYMax);
      const float appliedDeltaY = nextY - receivedPose.y;
      if (appliedDeltaY != 0.0F) {
        receivedPose.y = nextY;
        pendingNudgeDeltaY += appliedDeltaY;
        poseDirty = true;
      }
    } else if (receivedPose.valid) {
      // Apply encoder delta to the authoritative received bearing.
      const float newBearing = static_cast<float>(
          clampBearing(static_cast<int16_t>(receivedPose.bearing) + bearingDelta));
      if (newBearing != receivedPose.bearing) {
        receivedPose.bearing = newBearing;
        pendingBearingDeltaDeg += bearingDelta;
        poseDirty = true;
      }
    } else {
      // No received pose yet; update the local fallback bearing.
      const int16_t newBearing = clampBearing(static_cast<int16_t>(bearingDeg) + bearingDelta);
      if (newBearing != bearingDeg) {
        bearingDeg = newBearing;
        pendingBearingDeltaDeg += bearingDelta;
        poseDirty = true;
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
  }
  lastStopDecelMs = nowMs;
}

void processPoseOutput() {
  const bool nudgeActive = isNudgeActive();
  const bool isMoving =
      (speedMmPerSec > kSpeedDeadbandMmPerSec || speedMmPerSec < -kSpeedDeadbandMmPerSec);
  const bool hasNudgeDelta = (pendingNudgeDeltaX != 0.0F || pendingNudgeDeltaY != 0.0F);

  if (!poseDirty && !isMoving && !hasNudgeDelta) {
    return;
  }

  const uint32_t nowMs = millis();
  if (!nudgeActive && (nowMs - lastGCodeSentMs) < kMaxGCodeRateMs) {
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
  pinMode(kNudge, INPUT_PULLUP);
  pinMode(kStopButtonInput, INPUT_PULLUP);
  pinMode(kGimbleLock, OUTPUT);
  pinMode(kOrthogonal, OUTPUT);
  digitalWrite(kGimbleLock, HIGH);
  digitalWrite(kOrthogonal, HIGH);

  setupSPIFFS();
  node.loadConfig();
  setupWiFi();
  setupMQTTComms();

  lastGCodeSentMs = millis();
  lastStopDecelMs = lastGCodeSentMs;
}

void loop() {
  //processCncSerialInput();
  processEncoders();
  processStopButton();
  processPoseOutput();
  processBearingLeds();

}
