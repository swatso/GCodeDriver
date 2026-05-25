#pragma once

#include <Arduino.h>

struct Pose {
  float x;
  float y;
  float heading;
  int forward;
};

constexpr uint8_t kVehicleInputCount = 5;
constexpr uint8_t kVehicleCount = 6;  // 0 = none, 1..5 = selectable vehicles

const Pose kInitialVehiclePoses[kVehicleCount] = {
    {0.0F, 0.0F, 0.0F, -1},   // 0 = no vehicle selected
    {0.0F, 0.0F, 0.0F, -1},   // 1
    {0.0F, 0.0F, 315.0F, -1},  // 2
    {0.0F, 0.0F, 135.0F, 1}, // 3
    {-100.0F, 0.0F, 270.0F, -1}, // 4
    {0.0F, -100.0F, 45.0F, -1},  // 5
};

const char* const kInitGCode[] = {
    "G21",      // mm units
    "G90",      // absolute positioning
    "G28",      // home all axes
    "G1 X0 Y0 Z0"
};

const char* const kVehicleDeselectGCode[kVehicleInputCount][3] = {
    {"M106 S0", "G4 S2", "G1 E0 F1800"},
    {"M106 S0", "G4 S2", "G1 E0 F1800"},
    {"M106 S0", "G4 S2", "G1 E0 F1800"},
    {"M106 S0", "G4 S2", "G1 E0 F1800"},
    {"M106 S0", "G4 S2", "G1 E0 F1800"}
};

const char* const kVehicleSelectGCode[kVehicleInputCount][2] = {
    {"M106 S0", "G4 S2"},
    {"M106 S255", "G4 S2"},
    {"M106 S255", "G4 S2"},
    {"M106 S255", "G4 S2"},
    {"M106 S255", "G4 S2"},
};
