#ifndef MQTTCOMMS_H
#define MQTTCOMMS_H

#define MAX_TOPIC_TO_PUBLISH 15
#define BACKGROUNDPUBLISHRATE 4000

#include <Arduino.h>

struct ReceivedPose {
  float x;
  float y;
  float bearing;  // absolute heading in degrees, sourced from Z axis of received G1
  uint32_t sequence;  // increments on each successfully parsed inbound pose update
  bool valid;
};

extern ReceivedPose receivedPose;
extern char GCodeTopic[30];
extern char CurrentEncTopic[30];
extern char WayPointTopic[30];
extern char HomeTopic[30];
extern char LockTopic[30];
extern char UnlockTopic[30];

void setupMQTTComms();
void initTopics(char* currentNodeID);
boolean connectMQTTClient();
boolean publishMQTT(const char* topic, const char* message);
void serviceConnection();
void MQTTcallback(char* topic, byte* payload, unsigned int length);
boolean checkMQTTState();
unsigned long getMQTTUptime();
boolean publishCurrentSpeedValue(int value);
boolean publishWaypointPose(int x, int y, int bearing);

#endif // MQTTCOMMS_H