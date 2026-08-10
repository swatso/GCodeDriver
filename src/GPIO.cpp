#include "GPIO.h"
#include "MQTTComms.h"
#include "GCode.h"

namespace {

// GPIO mappings from docs/GPIO Mapping.ods.
constexpr uint8_t kBearingEncoderA = 13;
constexpr uint8_t kBearingEncoderB = 12;
constexpr uint8_t kStepSize1Pin = 4;
constexpr uint8_t kStepSize5Pin = 14;
constexpr uint8_t kStepSize10Pin = 16;
constexpr uint8_t kStepSize50Pin = 27;
constexpr uint8_t kSpeed1Pin = 17;
constexpr uint8_t kSpeed2Pin = 26;
constexpr uint8_t kSpeed3Pin = 25;
constexpr uint8_t kSpeed4Pin = 18;
constexpr uint8_t kSpeed5Pin = 19;
constexpr uint8_t kSpeed6Pin = 32;
constexpr uint8_t kDirectionPin = 21;

constexpr int8_t kBearingEncoderTransitionsPerDegree = 2;
constexpr bool kBearingDirectionInverted = false;
constexpr unsigned long kBearingEncoderAggregationWindowMs = 1000;

// Compile-time speed mapping values (mm/s); edit as needed.
constexpr int kSpeedValue1 = 100;
constexpr int kSpeedValue2 = 150;
constexpr int kSpeedValue3 = 200;
constexpr int kSpeedValue4 = 350;
constexpr int kSpeedValue5 = 500;
constexpr int kSpeedValue6 = 1000;
constexpr unsigned long kSpeedSettlingTimeMs = 2000;

uint8_t lastBearingAState = HIGH;
int8_t bearingEdgeDirectionSum = 0;
int pendingBearingDelta = 0;
unsigned long bearingEncoderWindowStartMs = 0;
int lastPublishedSpeed = INT32_MIN;
int pendingSpeed = INT32_MIN;
unsigned long pendingSpeedSinceMs = 0;

}  // namespace

namespace gpio {

void begin() {
  pinMode(kBearingEncoderA, INPUT_PULLUP);
  pinMode(kBearingEncoderB, INPUT_PULLUP);

  pinMode(kStepSize1Pin, INPUT_PULLUP);
  pinMode(kStepSize5Pin, INPUT_PULLUP);
  pinMode(kStepSize10Pin, INPUT_PULLUP);
  pinMode(kStepSize50Pin, INPUT_PULLUP);

  pinMode(kSpeed1Pin, INPUT_PULLUP);
  pinMode(kSpeed2Pin, INPUT_PULLUP);
  pinMode(kSpeed3Pin, INPUT_PULLUP);
  pinMode(kSpeed4Pin, INPUT_PULLUP);
  pinMode(kSpeed5Pin, INPUT_PULLUP);
  pinMode(kSpeed6Pin, INPUT_PULLUP);

  pinMode(kDirectionPin, INPUT_PULLUP);

  lastBearingAState = static_cast<uint8_t>(digitalRead(kBearingEncoderA));
  bearingEdgeDirectionSum = 0;
  pendingBearingDelta = 0;
  bearingEncoderWindowStartMs = millis();

  pendingSpeed = -1;
  pendingSpeedSinceMs = millis();
}

int8_t bearingEncoderDelta() {
  int8_t bearingDelta = 0;
  const uint8_t bearingAState = static_cast<uint8_t>(digitalRead(kBearingEncoderA));
  if (bearingAState != lastBearingAState) {
    const uint8_t bearingBState = static_cast<uint8_t>(digitalRead(kBearingEncoderB));

    int8_t edgeDirection = (bearingAState != bearingBState) ? 1 : -1;
    if (kBearingDirectionInverted) {
      edgeDirection = -edgeDirection;
    }

    bearingEdgeDirectionSum += edgeDirection;
    if (abs(bearingEdgeDirectionSum) >= kBearingEncoderTransitionsPerDegree) {
      bearingDelta = (bearingEdgeDirectionSum > 0) ? 1 : -1;
      bearingEdgeDirectionSum = 0;
      Serial.printf("[GPIO] bearingEncoderDelta=%d\n", bearingDelta);
    }
    lastBearingAState = bearingAState;
  }

  return bearingDelta;
}

int stepSize() {
  int value = 1;
  if (digitalRead(kStepSize1Pin) == LOW) {
    value = 1;
  } else if (digitalRead(kStepSize5Pin) == LOW) {
    value = 5;
  } else if (digitalRead(kStepSize10Pin) == LOW) {
    value = 10;
  } else if (digitalRead(kStepSize50Pin) == LOW) {
    value = 50;
  }

  static int lastPrinted = -1;
  if (value != lastPrinted) {
    Serial.printf("[GPIO] stepSize=%d\n", value);
    lastPrinted = value;
  }
  return value;
}

int speed() {
  int value = kSpeedValue1;
  if (digitalRead(kSpeed1Pin) == LOW) {
    value = kSpeedValue1;
  } else if (digitalRead(kSpeed2Pin) == LOW) {
    value = kSpeedValue2;
  } else if (digitalRead(kSpeed3Pin) == LOW) {
    value = kSpeedValue3;
  } else if (digitalRead(kSpeed4Pin) == LOW) {
    value = kSpeedValue4;
  } else if (digitalRead(kSpeed5Pin) == LOW) {
    value = kSpeedValue5;
  } else if (digitalRead(kSpeed6Pin) == LOW) {
    value = kSpeedValue6;
  }

  static int lastPrinted = INT32_MIN;
  if (value != lastPrinted) {
    Serial.printf("[GPIO] speed=%d\n", value);
    lastPrinted = value;
  }
  return value;
}

int direction() {
  const bool active = (digitalRead(kDirectionPin) == LOW);
  if(active) {
    Serial.println("[GPIO] direction=forward");
    return 1;
  } else {
    Serial.println("[GPIO] direction=reverse");
    return -1;
  }
}

void speedHelper() {
  // called from the main loop to check for speed changes and publish them via MQTT

  const int currentSpeed = speed();
  if(pendingSpeed == -1) {
    pendingSpeed = currentSpeed;
    lastPublishedSpeed = currentSpeed;
  }
  if (currentSpeed != pendingSpeed) {
    pendingSpeed = currentSpeed;
    pendingSpeedSinceMs = millis();
  }

  if (pendingSpeed != lastPublishedSpeed &&
      (millis() - pendingSpeedSinceMs >= kSpeedSettlingTimeMs)) {
    lastPublishedSpeed = pendingSpeed;
    publishCurrentSpeedValue(lastPublishedSpeed);
  }
}

void bearingEncoderHelper() {
  // Accumulate bearing encoder changes over a one-second window and publish once per window.
  const unsigned long now = millis();
  if (bearingEncoderWindowStartMs == 0) {
    bearingEncoderWindowStartMs = now;
  }

  pendingBearingDelta += bearingEncoderDelta();

  if (now - bearingEncoderWindowStartMs >= kBearingEncoderAggregationWindowMs) {
    if (pendingBearingDelta != 0) {
      Serial.printf("[GPIO] applying accumulated bearing delta=%d\n", pendingBearingDelta);
      GCode::updateCurrentPoseGCode(0, 0, pendingBearingDelta);
    }
    pendingBearingDelta = 0;
    bearingEncoderWindowStartMs = now;
  }
}
}  // namespace gpio
