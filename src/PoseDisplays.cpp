#include "PoseDisplays.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include <Wire.h>

namespace {
constexpr uint8_t kDisplayAddressX = 0x70;
constexpr uint8_t kDisplayAddressY = 0x71;
constexpr uint8_t kDisplayAddressZ = 0x72;

constexpr uint8_t kCmdOscillatorOn = 0x21;
constexpr uint8_t kCmdDisplayOnNoBlink = 0x81;
constexpr uint8_t kCmdBrightnessBase = 0xE0;

// Standard 7-segment encoding for HT16K33 backpacks.
constexpr uint16_t kDigitSegments[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
};
constexpr uint16_t kMinusSegments = 0x40;
constexpr uint16_t kBlankSegments = 0x00;

bool gDisplayReady = false;

bool sendCommand(uint8_t address, uint8_t command) {
  Wire.beginTransmission(address);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

bool setDisplayBrightness(uint8_t address, PoseDisplayBrightness brightness) {
  const uint8_t level = static_cast<uint8_t>(brightness) & 0x0F;
  return sendCommand(address, static_cast<uint8_t>(kCmdBrightnessBase | level));
}

uint16_t encodeChar(char c) {
  if (c >= '0' && c <= '9') {
    return kDigitSegments[c - '0'];
  }
  if (c == '-') {
    return kMinusSegments;
  }
  return kBlankSegments;
}

void writeFourChars(uint8_t address, const char chars[4]) {
  uint16_t displayBuffer[8] = {0};

  // 4-digit HT16K33 boards map digit 3 to index 4, with index 2 used by colon.
  displayBuffer[0] = encodeChar(chars[0]);
  displayBuffer[1] = encodeChar(chars[1]);
  displayBuffer[3] = encodeChar(chars[2]);
  displayBuffer[4] = encodeChar(chars[3]);

  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(0x00));
  for (uint8_t i = 0; i < 8; ++i) {
    Wire.write(static_cast<uint8_t>(displayBuffer[i] & 0xFF));
    Wire.write(static_cast<uint8_t>((displayBuffer[i] >> 8) & 0xFF));
  }
  Wire.endTransmission();
}

void writeNumber(uint8_t address, float value) {
  const long roundedValue = lroundf(value);
  char text[5] = {' ', ' ', ' ', ' ' , '\0'};

  // Keep output inside what a 4-digit display can show.
  if (roundedValue >= -999 && roundedValue <= 9999) {
    snprintf(text, sizeof(text), "%4ld", roundedValue);
  } else {
    memcpy(text, "----", 4);
  }

  writeFourChars(address, text);
}

bool initSingleDisplay(uint8_t address) {
  const bool oscillatorOk = sendCommand(address, kCmdOscillatorOn);
  const bool displayOnOk = sendCommand(address, kCmdDisplayOnNoBlink);
  const bool brightnessOk = setDisplayBrightness(address, PoseDisplayBrightness::Medium);
  //writeFourChars(address, "    ");
  writeFourChars(address, "1234");
  return oscillatorOk && displayOnOk && brightnessOk;
}

}  // namespace

bool setupPoseDisplays(uint8_t sdaPin, uint8_t sclPin) {
  Wire.begin(sdaPin, sclPin);

  const bool xOk = initSingleDisplay(kDisplayAddressX);
  const bool yOk = initSingleDisplay(kDisplayAddressY);
  const bool zOk = initSingleDisplay(kDisplayAddressZ);
  gDisplayReady = xOk && yOk && zOk;

  if (!gDisplayReady) {
    Serial.printf("[I2C] Display init status X:%s Y:%s Z:%s\n",
                  xOk ? "ok" : "missing",
                  yOk ? "ok" : "missing",
                  zOk ? "ok" : "missing");
  }

  return gDisplayReady;
}

void updatePoseDisplays(float x, float y, float z, PoseDisplayBrightness brightness) {
//  if (!gDisplayReady) {
//    return;
//  }

  setDisplayBrightness(kDisplayAddressX, brightness);
  setDisplayBrightness(kDisplayAddressY, brightness);
  setDisplayBrightness(kDisplayAddressZ, brightness);
Serial.printf("[I2C] Updating displays: X=%.3f Y=%.3f Z=%.3f\n", x, y, z);

  writeNumber(kDisplayAddressX, x);
  writeNumber(kDisplayAddressY, y);
  writeNumber(kDisplayAddressZ, z);
}

bool updatePoseDisplaysFromGCode(const char* gcodeLine,
                                 PoseDisplayBrightness brightness) {
  if (gcodeLine == nullptr) {
    return false;
  }

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float feedrate = 0.0F;

  if (sscanf(gcodeLine, "G1 X%f Y%f Z%f F%f", &x, &y, &z, &feedrate) == 4 ||
      sscanf(gcodeLine, "G1 X%f Y%f Z%f", &x, &y, &z) == 3) {
    updatePoseDisplays(x, y, z, brightness);
    return true;
  }

  return false;
}
