#ifndef POSE_DISPLAYS_H
#define POSE_DISPLAYS_H

#include <Arduino.h>

enum class PoseDisplayBrightness : uint8_t {
  Medium = 8,
  High = 15,
};

bool setupPoseDisplays(uint8_t sdaPin = 23, uint8_t sclPin = 22);
void updatePoseDisplays(float x, float y, float z, PoseDisplayBrightness brightness);
bool updatePoseDisplaysFromGCode(const char* gcodeLine,
                                 PoseDisplayBrightness brightness);

#endif  // POSE_DISPLAYS_H
