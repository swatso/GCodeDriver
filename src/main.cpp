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
  serviceSwitchMatrixScanner();
  gpio::bearingEncoderHelper();
  gpio::speedHelper();
}
