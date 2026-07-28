
#include "GCode.h"
#include "MQTTComms.h"


namespace GCode {

constexpr int kPoseXMin = 0;
constexpr int kPoseXMax = 265;
constexpr int kPoseYMin = 0;
constexpr int kPoseYMax = 225;
constexpr int kPoseZMin = 0;
constexpr int kPoseZMax = 360;
const char kHomePayload[] = "Home";
const char kLockPayload[] = "Lock";
const char kUnlockPayload[] = "Unlock";
float toRadians(int deg) { return deg * DEG_TO_RAD; }

int clampInt(int value, int minimum, int maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}



void updateCurrentPoseGCode(int deltaX, int deltaY, int deltaZ) {
  if (receivedPose.valid) {
    const int unclampedX = receivedPose.x + deltaX;
    const int unclampedY = receivedPose.y + deltaY;
    const int unclampedZ = receivedPose.bearing + deltaZ;
    const int clampedX = static_cast<int>(clampInt(static_cast<int>(unclampedX), kPoseXMin, kPoseXMax));
    const int clampedY = static_cast<int>(clampInt(static_cast<int>(unclampedY), kPoseYMin, kPoseYMax));
    const int clampedZ = static_cast<int>(clampInt(static_cast<int>(unclampedZ), kPoseZMin, kPoseZMax));

    if (clampedX != receivedPose.x || clampedY != receivedPose.y || clampedZ != static_cast<int>(receivedPose.bearing)) {
      receivedPose.x = clampedX;
      receivedPose.y = clampedY;
      receivedPose.bearing = clampedZ;
      char gcodeLine[64];
      snprintf(gcodeLine, sizeof(gcodeLine), "G1 X%i Y%i Z%i", deltaX, deltaY,deltaZ);
      publishMQTT(GCodeTopic, gcodeLine);
    }
  }
}

void setHome() {
  receivedPose.x = 0;
  receivedPose.y = 0;
  receivedPose.bearing = 0;
  receivedPose.valid = true;
  publishMQTT(HomeTopic, kHomePayload);
}

void setLock() {
  publishMQTT(LockTopic, kLockPayload);
}

void setUnlock() {
  publishMQTT(UnlockTopic, kUnlockPayload);
}


}  // namespace GCode