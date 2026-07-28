#pragma once

#include <Arduino.h>

namespace GCode {

void updateCurrentPoseGCode(int deltaX, int deltaY, int deltaZ);
void setHome();
void setLock(); 
void setUnlock();

}  // namespace GCode
