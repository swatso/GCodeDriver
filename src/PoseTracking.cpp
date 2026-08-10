#include "PoseTracking.h"
#include "MQTTComms.h"

namespace {

constexpr size_t kPoseBufferSize = 32;

PoseTracking::Pose poseBuffer[kPoseBufferSize];
size_t writeIndex = 0;
size_t outputIndex = 0;
size_t storedCount = 0;

size_t oldestIndex() {
  return (writeIndex + kPoseBufferSize - storedCount) % kPoseBufferSize;
}

size_t newestIndex() {
  return (writeIndex + kPoseBufferSize - 1U) % kPoseBufferSize;
}

void logState(const char* action) {
  Serial.printf("[PoseTracking] %s write=%u output=%u stored=%u newest=%u\n",
                action,
                static_cast<unsigned>(writeIndex),
                static_cast<unsigned>(outputIndex),
                static_cast<unsigned>(storedCount),
                static_cast<unsigned>(storedCount == 0 ? 0U : newestIndex()));

  if (storedCount > 0) {
    const PoseTracking::Pose& currentPose = poseBuffer[outputIndex];
    Serial.printf("[PoseTracking] current X=%d Y=%d B=%d\n",
                  currentPose.x,
                  currentPose.y,
                  currentPose.bearing);
  }
}

bool moveOutputByOffset(int offset, PoseTracking::Pose* outPose) {
  if (outPose == nullptr || storedCount == 0) {
    return false;
  }

  const size_t oldest = oldestIndex();
  const size_t currentOffset = (outputIndex + kPoseBufferSize - oldest) % kPoseBufferSize;
  const int nextOffset = static_cast<int>(currentOffset) + offset;
  if (nextOffset < 0 || nextOffset >= static_cast<int>(storedCount)) {
    *outPose = poseBuffer[outputIndex];
    return false;
  }

  outputIndex = static_cast<size_t>((oldest + static_cast<size_t>(nextOffset)) % kPoseBufferSize);
  *outPose = poseBuffer[outputIndex];
  return true;
}

}  // namespace

namespace PoseTracking {

bool add(const Pose& pose) {
  poseBuffer[writeIndex] = pose;
  publishWaypointPose(pose.x, pose.y, pose.bearing);
  outputIndex = writeIndex;
  writeIndex = (writeIndex + 1U) % kPoseBufferSize;
  if (storedCount < kPoseBufferSize) {
    ++storedCount;
  }
  logState("add");
  return true;
}

bool previous(Pose* outPose) {
  const bool moved = moveOutputByOffset(-1, outPose);
  logState(moved ? "previous" : "previous-blocked");
  return moved;
}

bool next(Pose* outPose) {
  const bool moved = moveOutputByOffset(1, outPose);
  logState(moved ? "next" : "next-blocked");
  return moved;
}

void clear() {
  writeIndex = 0;
  outputIndex = 0;
  storedCount = 0;
  logState("clear");
}

}  // namespace PoseTracking