#include <Arduino.h>

#include "vehicle_config.h"

namespace {
constexpr int8_t kSpeedMin = -50;
constexpr int8_t kSpeedMax = 50;
constexpr uint16_t kVehicleSettleMs = 3000;
constexpr uint16_t kMaxGCodeRateMs = 500;  // 2 Hz
constexpr int8_t kSpeedDeadbandMmPerSec = 0;  // speeds within [-deadband, +deadband] are treated as stopped

// GPIO mappings.
constexpr uint8_t kSpeedEncoderA = 13;
constexpr uint8_t kSpeedEncoderB = 12;
constexpr uint8_t kBearingEncoderA = 4;
constexpr uint8_t kBearingEncoderB = 14;
constexpr uint8_t kCncSerialTx = 22;
constexpr uint8_t kCncSerialRx = 23;
constexpr uint8_t kVehicleInputs[kVehicleInputCount] = {16, 27, 17, 26, 25};

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

float toRadians(float deg) { return deg * DEG_TO_RAD; }

int16_t normalizeBearing(int16_t value) {
  while (value < 0) {
    value += 360;
  }
  while (value > 360) {
    value -= 360;
  }
  return value;
}

int8_t clampSpeed(int32_t value) {
  if (value < kSpeedMin) {
    return kSpeedMin;
  }
  if (value > kSpeedMax) {
    return kSpeedMax;
  }
  return static_cast<int8_t>(value);
}

void streamLines(const char* const* lines, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    Serial2.println(lines[i]);
  }
}

void streamCurrentPoseGCode() {
  if (currentVehicle == 0) {
    return;
  }

  const uint32_t nowMs = millis();
  const float dtSec = static_cast<float>(nowMs - lastPoseUpdateMs) / 1000.0F;
  Pose& pose = vehiclePoses[currentVehicle];

  const float distanceMm = static_cast<float>(speedMmPerSec) * dtSec;
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
  pose.heading = static_cast<float>(bearingDeg);

  Serial2.print("G1 X");
  Serial2.print(pose.x, 3);
  Serial2.print(" Y");
  Serial2.print(pose.y, 3);
  Serial2.print(" Z");
  Serial2.println(pose.heading, 3);

  lastPoseUpdateMs = nowMs;
  lastGCodeSentMs = nowMs;
}

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
    const int8_t newSpeed = clampSpeed(static_cast<int32_t>(speedMmPerSec) + speedDelta);
    if (newSpeed != speedMmPerSec) {
      speedMmPerSec = newSpeed;
      poseDirty = true;
    }
  }

  const int8_t bearingDelta = bearingEncoder.readDelta();
  if (bearingDelta != 0) {
    const int16_t newBearing = normalizeBearing(static_cast<int16_t>(bearingDeg) + bearingDelta);
    if (newBearing != bearingDeg) {
      bearingDeg = newBearing;
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
  Serial2.begin(115200, SERIAL_8N1, kCncSerialRx, kCncSerialTx);

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
  processVehicleSelection();
  processEncoders();
  processPoseOutput();
}
