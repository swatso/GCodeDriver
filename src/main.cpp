#include <Arduino.h>

#include "vehicle_config.h"
#include "FileSystem.h"
#include "GCode.h"
#include "GPIO.h"
#include "MQTTComms.h"
#include "NodeServices.h"
#include "SwitchMatrix.h"
#include "WiFiManager.h"

#ifndef GCODE_USB_DEBUG
#define GCODE_USB_DEBUG 1
#endif

#ifndef MARLIN_USB_DEBUG
#define MARLIN_USB_DEBUG 1
#endif

namespace {

constexpr uint32_t kUsbSerialBaudRate = 115200;
constexpr uint32_t kWiFiReconnectIntervalMs = 15000;

}  // namespace

void setup() {
  Serial.begin(kUsbSerialBaudRate);
  gpio::begin();
  setupSPIFFS();
  node.loadConfig();
  setupWiFi();
  setupMQTTComms();
  setupSwitchMatrixScanner();
}

void loop() {
  serviceWiFi();

  static uint32_t lastWiFiServiceMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastWiFiServiceMs >= kWiFiReconnectIntervalMs) {
    lastWiFiServiceMs = nowMs;
    checkWiFiConnection();
  }

  serviceSwitchMatrixScanner();
  gpio::bearingEncoderHelper();
  gpio::speedHelper();
}
