#ifndef MQTTCOMMS_H
#define MQTTCOMMS_H

#define MAX_TOPIC_TO_PUBLISH 15
#define BACKGROUNDPUBLISHRATE 4000

#include <Arduino.h>

void setupMQTTComms();
void initTopics(char* currentNodeID);
boolean connectMQTTClient();
boolean  subscribeTopics();
boolean MQTTPublishNext();
boolean publishBit(byte bitNo);
boolean publishMQTT(char* topic, char* message);
void serviceConnection();
void sensorReceiverTask(void *pvParameters);
void messageReceiverTask(void *pvParameters);
void MQTTcallback(char* topic, byte* payload, unsigned int length);
boolean checkMQTTState();
unsigned long getMQTTUptime();

#endif // MQTTCOMMS_H