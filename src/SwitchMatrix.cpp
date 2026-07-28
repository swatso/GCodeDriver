#include "SwitchMatrix.h"

#include <Wire.h>
#include "GPIO.h"
#include "GCode.h"
#include "MQTTComms.h"
#include "PoseTracking.h"

namespace {
constexpr uint8_t kTca8418Address = 0x34;

constexpr uint8_t kRegCfg = 0x01;
constexpr uint8_t kRegIntStat = 0x02;
constexpr uint8_t kRegKeyLockEventCount = 0x03;
constexpr uint8_t kRegKeyEventA = 0x04;
constexpr uint8_t kRegKpGpio1 = 0x1D;
constexpr uint8_t kRegKpGpio2 = 0x1E;
constexpr uint8_t kRegKpGpio3 = 0x1F;

constexpr uint8_t kCfgAi = 0x80;
constexpr uint8_t kCfgKeyEventInterruptEnable = 0x01;

bool gSwitchMatrixReady = false;
float toRadians(int deg) { return deg * DEG_TO_RAD; }

}  // namespace

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kTca8418Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t* outValue) {
  if (outValue == nullptr) {
    return false;
  }

  Wire.beginTransmission(kTca8418Address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(kTca8418Address), 1) != 1) {
    return false;
  }

  *outValue = Wire.read();
  return true;
}

bool probeDevice() {
  Wire.beginTransmission(kTca8418Address);
  return Wire.endTransmission() == 0;
}

void serviceKeyXPlusYPlus() { 
  Serial.println("[TCA8418] X+Y+ pressed"); 
  int deltaX = gpio::stepSize();
  int deltaY = gpio::stepSize();
  GCode::updateCurrentPoseGCode(deltaX, deltaY, 0);
}

void serviceKeyYPlus() { 
  Serial.println("[TCA8418] Y+ pressed"); 
  int deltaY = gpio::stepSize();
  GCode::updateCurrentPoseGCode(0, deltaY, 0);
}
void serviceKeyXMinusYPlus() { 
  Serial.println("[TCA8418] X-Y+ pressed"); 
  int deltaX = -gpio::stepSize();
  int deltaY = gpio::stepSize();
  GCode::updateCurrentPoseGCode(deltaX, deltaY, 0);
}
void serviceKeyXPlus() { 
  Serial.println("[TCA8418] X+ pressed"); 
  int deltaX = gpio::stepSize();
  GCode::updateCurrentPoseGCode(deltaX, 0, 0);
}
void serviceKeyHome() { 
  Serial.println("[TCA8418] Home pressed"); 
  GCode::setHome();
}
void serviceKeyXMinus() { 
  Serial.println("[TCA8418] X- pressed"); 
  int deltaX = -gpio::stepSize();
  GCode::updateCurrentPoseGCode(deltaX, 0, 0);
}
void serviceKeyXPlusYMinus() { 
  Serial.println("[TCA8418] X+Y- pressed"); 
  int deltaX = gpio::stepSize();
  int deltaY = -gpio::stepSize();
  GCode::updateCurrentPoseGCode(deltaX, deltaY, 0);
}
void serviceKeyYMinus() { 
  Serial.println("[TCA8418] Y- pressed"); 
  int deltaY = -gpio::stepSize();
  GCode::updateCurrentPoseGCode(0, deltaY, 0);
}
void serviceKeyXMinusYMinus() { 
  Serial.println("[TCA8418] X-Y- pressed"); 
  int deltaX = -gpio::stepSize();
  int deltaY = -gpio::stepSize();
  GCode::updateCurrentPoseGCode(deltaX, deltaY, 0);
}

void serviceKeyLock() { 
  Serial.println("[TCA8418] Lock pressed"); 
  GCode::setLock();
}
void serviceKeySet() {
  Serial.println("[TCA8418] Set pressed");
  if (!receivedPose.valid) {
    Serial.println("[PoseTracking] ignoring set - no valid pose");
    return;
  }

  PoseTracking::Pose pose = {
      static_cast<int>(receivedPose.x),
      static_cast<int>(receivedPose.y),
      static_cast<int>(receivedPose.bearing)};
  PoseTracking::add(pose);
  Serial.printf("[PoseTracking] added pose X=%d Y=%d B=%d\n", pose.x, pose.y, pose.bearing);
}
void serviceKeyPlay() { Serial.println("[TCA8418] Play pressed"); }
void serviceKeyUnlock() { 
  Serial.println("[TCA8418] UnLock pressed"); 
  GCode::setUnlock();
}
void serviceKeyClear() {
  Serial.println("[TCA8418] Clear pressed");
  PoseTracking::clear();
}
void serviceKeyFwd() {
  Serial.println("[TCA8418] Fwd pressed");
  PoseTracking::Pose pose;
  if (!PoseTracking::previous(&pose)) {
    return;
  }

  const int deltaX = pose.x - static_cast<int>(receivedPose.x);
  const int deltaY = pose.y - static_cast<int>(receivedPose.y);
  const int deltaB = pose.bearing - static_cast<int>(receivedPose.bearing);
  GCode::updateCurrentPoseGCode(deltaX, deltaY, deltaB);
}

void serviceKeyMove() { 
  Serial.println("[TCA8418] Move pressed"); 
  float deltaX = (gpio::direction() * gpio::stepSize() * sin(toRadians(receivedPose.bearing)));
  float deltaY = (-1 * gpio::direction() * gpio::stepSize() * cos(toRadians(receivedPose.bearing)));
Serial.printf("[TCA8418] Move deltaX=%f deltaY=%f\n", deltaX, deltaY);
  GCode::updateCurrentPoseGCode((int)deltaX, (int)deltaY, 0);

}

void serviceKeyBack() {
  Serial.println("[TCA8418] Back pressed");
  PoseTracking::Pose pose;
  if (!PoseTracking::next(&pose)) {
    return;
  }

  const int deltaX = pose.x - static_cast<int>(receivedPose.x);
  const int deltaY = pose.y - static_cast<int>(receivedPose.y);
  const int deltaB = pose.bearing - static_cast<int>(receivedPose.bearing);
  GCode::updateCurrentPoseGCode(deltaX, deltaY, deltaB);
}

bool dispatchRecognizedKey(uint8_t row, uint8_t col) {
  if (row == 0 && col == 0) {
    serviceKeyXPlusYPlus();
    return true;
  }
  if (row == 0 && col == 1) {
    serviceKeyYPlus();
    return true;
  }
  if (row == 0 && col == 2) {
    serviceKeyXMinusYPlus();
    return true;
  }
  if (row == 1 && col == 0) {
    serviceKeyXPlus();
    return true;
  }
  if (row == 1 && col == 1) {
    serviceKeyHome();
    return true;
  }
  if (row == 1 && col == 2) {
    serviceKeyXMinus();
    return true;
  }
  if (row == 2 && col == 0) {
    serviceKeyXPlusYMinus();
    return true;
  }
  if (row == 2 && col == 1) {
    serviceKeyYMinus();
    return true;
  }
  if (row == 2 && col == 2) {
    serviceKeyXMinusYMinus();
    return true;
  }
  if (row == 3 && col == 3) {
    serviceKeyLock();
    return true;
  }
  if (row == 3 && col == 4) {
    serviceKeySet();
    return true;
  }
  if (row == 3 && col == 5) {
    serviceKeyPlay();
    return true;
  }
  if (row == 4 && col == 3) {
    serviceKeyUnlock();
    return true;
  }
  if (row == 4 && col == 4) {
    serviceKeyClear();
    return true;
  }
  if (row == 4 && col == 5) {
    serviceKeyFwd();
    return true;
  }
  if (row == 5 && col == 3) {
    serviceKeyMove();
    return true;
  }
  if (row == 5 && col == 5) {
    serviceKeyBack();
    return true;
  }

  return false;
}

void handleKeyEvent(uint8_t eventCode) {
  if (eventCode == 0) {
    return;
  }

  const bool keyReleased = (eventCode & 0x80U) != 0;
  if (keyReleased) {
    return;
  }

  const uint8_t keyIndex = static_cast<uint8_t>(eventCode & 0x7FU);
  if (keyIndex == 0 || keyIndex > 80) {
    Serial.printf("[TCA8418] Ignoring out-of-range key event: 0x%02X\n", eventCode);
    return;
  }

  const uint8_t row = static_cast<uint8_t>((keyIndex - 1U) / 10U);
  const uint8_t col = static_cast<uint8_t>((keyIndex - 1U) % 10U);
  if (!dispatchRecognizedKey(row, col)) {
    Serial.printf("[TCA8418] Unmapped key press row=%u col=%u code=0x%02X\n",
                  static_cast<unsigned>(row),
                  static_cast<unsigned>(col),
                  static_cast<unsigned>(eventCode));
  }
}

bool setupSwitchMatrixScanner() {
  if (!probeDevice()) {
    Serial.printf("[TCA8418] Device not detected at 0x%02X\n",
                  static_cast<unsigned>(kTca8418Address));
    gSwitchMatrixReady = false;
    return false;
  }

  // Configure R0..R5 and C0..C5 as keypad matrix lines.
  const bool cfgOk = writeRegister(kRegCfg, static_cast<uint8_t>(kCfgAi | kCfgKeyEventInterruptEnable));
  const bool kp1Ok = writeRegister(kRegKpGpio1, 0x3F);
  const bool kp2Ok = writeRegister(kRegKpGpio2, 0x3F);
  const bool kp3Ok = writeRegister(kRegKpGpio3, 0x00);
  const bool clearOk = writeRegister(kRegIntStat, 0xFF);

  gSwitchMatrixReady = cfgOk && kp1Ok && kp2Ok && kp3Ok && clearOk;
  Serial.printf("[TCA8418] setup %s (cfg=%d kp1=%d kp2=%d kp3=%d clr=%d)\n",
                gSwitchMatrixReady ? "ok" : "failed",
                static_cast<int>(cfgOk),
                static_cast<int>(kp1Ok),
                static_cast<int>(kp2Ok),
                static_cast<int>(kp3Ok),
                static_cast<int>(clearOk));

  return gSwitchMatrixReady;
}

void serviceSwitchMatrixScanner() {
  if (!gSwitchMatrixReady) {
    return;
  }

  uint8_t keyInfo = 0;
  if (!readRegister(kRegKeyLockEventCount, &keyInfo)) {
    return;
  }

  uint8_t eventCount = static_cast<uint8_t>(keyInfo & 0x0FU);
  while (eventCount > 0) {
    uint8_t eventCode = 0;
    if (!readRegister(kRegKeyEventA, &eventCode)) {
      break;
    }

    handleKeyEvent(eventCode);
    --eventCount;
  }

  // Clear any pending interrupt flags after draining events.
  uint8_t intStatus = 0;
  if (readRegister(kRegIntStat, &intStatus) && intStatus != 0) {
    writeRegister(kRegIntStat, intStatus);
  }
}