#include <Arduino.h>

#include "vehicle_config.h"

#ifndef GCODE_USB_DEBUG
#define GCODE_USB_DEBUG 1
#endif

namespace {
constexpr int8_t kSpeedMin = -50;
constexpr int8_t kSpeedMax = 50;
constexpr uint16_t kVehicleSettleMs = 3000;
constexpr uint16_t kMaxGCodeRateMs = 1000;  // 1 Hz
constexpr int8_t kSpeedDeadbandMmPerSec = 2;  // speeds within [-deadband, +deadband] are treated as stopped
constexpr float kHeadingOnlyFeedrateDegPerSec = 30.0F;

// GPIO mappings.
constexpr uint8_t kSpeedEncoderA = 13;
constexpr uint8_t kSpeedEncoderB = 12;
constexpr uint8_t kBearingEncoderA = 4;
constexpr uint8_t kBearingEncoderB = 14;
constexpr uint8_t kCncSerialTx = 22;
constexpr uint8_t kCncSerialRx = 23;
constexpr uint8_t kVehicleInputs[kVehicleInputCount] = {16, 27, 17, 26, 25};
constexpr uint32_t kUsbSerialBaudRate = 115200;

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

Encoder speedEncoder{kSpeedEncoderA, kSpeedEncoderB, 0};
Encoder bearingEncoder{kBearingEncoderA, kBearingEncoderB, 0};

Pose vehiclePoses[kVehicleCount];
uint8_t currentVehicle = 0;
uint8_t pendingVehicle = 0;
uint32_t pendingVehicleSinceMs = 0;

int8_t speedMmPerSec = 0;
int16_t bearingDeg = 0;

bool poseDirty = false;
uint32_t lastPoseUpdateMs = 0;
uint32_t lastGCodeSentMs = 0;

#if GCODE_USB_DEBUG
constexpr size_t kCncRxBufferSize = 96;
char cncRxBuffer[kCncRxBufferSize] = {};
size_t cncRxLength = 0;
#endif

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

void streamLine(const char* line) {
  Serial2.println(line);
#if GCODE_USB_DEBUG
  Serial.printf("Speed: %d mm/s, Bearing: %d deg -> ", speedMmPerSec, bearingDeg);
  Serial.println(line);
#endif
}

void streamLines(const char* const* lines, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    streamLine(lines[i]);
  }
}

void streamCurrentPoseGCode() {
  if (currentVehicle == 0) {
    return;
  }

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
  pose.x += distanceMm * cosf(headingRad);
  if(pose.x < 0) {
    pose.x = 0;
  } else if(pose.x > kBedSizeX) {
    pose.x = kBedSizeX;
  }
  pose.y += distanceMm * sinf(headingRad);
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

#if GCODE_USB_DEBUG
void processCncSerialInput() {
  while (Serial2.available() > 0) {
    const char ch = static_cast<char>(Serial2.read());

    if (ch == '\r' || ch == '\n') {
      if (cncRxLength > 0) {
        Serial.print("Marlin:");
        Serial.println(cncRxBuffer);
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
#else
void processCncSerialInput() {}
#endif

void applyVehicleSelection(uint8_t vehicle) {
  currentVehicle = vehicle;
  speedMmPerSec = 0;
  poseDirty = false;

  if (currentVehicle == 0) {
    return;
  }

  bearingDeg = normalizeBearing(static_cast<int16_t>(vehiclePoses[currentVehicle].heading));
  lastPoseUpdateMs = millis();
  lastGCodeSentMs = lastPoseUpdateMs - kMaxGCodeRateMs;

  streamLines(kVehicleSelectGCode[currentVehicle - 1], 2);
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

void processEncoders() {
  const int8_t speedDelta = speedEncoder.readDelta();
  if (speedDelta != 0) {
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

  const int8_t bearingDelta = bearingEncoder.readDelta();
  if (bearingDelta != 0) {
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

void processPoseOutput() {
  if (currentVehicle == 0) {
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
#if GCODE_USB_DEBUG
  Serial.begin(kUsbSerialBaudRate);
#endif
  Serial2.begin(250000, SERIAL_8N1, kCncSerialRx, kCncSerialTx);

  for (uint8_t i = 0; i < kVehicleCount; ++i) {
    vehiclePoses[i] = kInitialVehiclePoses[i];
  }

  streamLines(kInitGCode, sizeof(kInitGCode) / sizeof(kInitGCode[0]));

  speedEncoder.begin();
  bearingEncoder.begin();

  for (uint8_t pin : kVehicleInputs) {
    pinMode(pin, INPUT_PULLUP);
  }

  pendingVehicle = readVehicleSelectionRaw();
  pendingVehicleSinceMs = millis();
  lastPoseUpdateMs = millis();
  lastGCodeSentMs = lastPoseUpdateMs;
}

void loop() {
  processCncSerialInput();
  processVehicleSelection();
  processEncoders();
  processPoseOutput();
}
