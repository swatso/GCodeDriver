#pragma once

#include <Arduino.h>

namespace PoseTracking {

struct Pose {
  int x;
  int y;
  int bearing;
};

bool add(const Pose& pose);
bool previous(Pose* outPose);
bool next(Pose* outPose);
void clear();

}  // namespace PoseTracking