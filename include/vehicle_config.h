#pragma once

#include <Arduino.h>

struct Pose {
  float x;
  float y;
  float heading;
};

constexpr uint8_t kVehicleInputCount = 5;
constexpr uint8_t kVehicleCount = 6;  // 0 = none, 1..5 = selectable vehicles

const Pose kInitialVehiclePoses[kVehicleCount] = {
    {0.0F, 0.0F, 0.0F},   // 0 = no vehicle selected
    {0.0F, 0.0F, 0.0F},   // 1
    {100.0F, 0.0F, 90.0F},  // 2
    {0.0F, 100.0F, 180.0F}, // 3
    {-100.0F, 0.0F, 270.0F}, // 4
    {0.0F, -100.0F, 45.0F},  // 5
};

const char* const kInitGCode[] = {
    "G21",      // mm units
    "G90",      // absolute positioning
    "G92 X0 Y0 Z0"};

const char* const kVehicleSelectGCode[kVehicleInputCount][2] = {
    {"M117 Vehicle 1", "G4 P50"},
    {"M117 Vehicle 2", "G4 P50"},
    {"M117 Vehicle 3", "G4 P50"},
    {"M117 Vehicle 4", "G4 P50"},
    {"M117 Vehicle 5", "G4 P50"},
};
