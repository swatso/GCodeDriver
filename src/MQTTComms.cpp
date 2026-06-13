// This module handles MQTT data transfers via wifi
// Intended for use with JMRI using turnout and sensor topics
// subscribes to turnout topics 00 through 0F, maintaining bits in the controlbytes C0 and C1
// and drives sensor topics 00 through 0F based on the state of status bytes S0 and S1
// Use this alongside the WIFI_Manager (asynchronous web server)

// Call initMQTT() at startup
//
//  Needs the following:-
//    nodeIDString          -string containing the Wifi Node ID which will be used in the topics
//    brokerIP              -string containing the IP address of the MQTT Broker

// Provides the following functions:-
//  
//    checkMQTTState()      -true = client connected
//    getMQTTUptime()       -connection uptime in minutes

// everything then runs on a ticker
//

#include <string>
#include <cstring>
#include <Ticker.h>
#include <PubSubClient.h>         // MQTT library
#include <WiFi.h>
#include <ESPAsyncWebServer.h>                    		
#include <AsyncTCP.h>                             		
#include "MQTTComms.h"
#include "NodeServices.h"
#include "WiFiManager.h"

Ticker runMQTT;

long MQTTConnectionTime;

const char* GCodeDriverReporterTopic = "track/reporter/2600";
char GCodeTopic[30];

const char* const kPoseReporterTopic = "track/reporter/2800";
ReceivedPose receivedPose = {};

WiFiClient espClient;
PubSubClient client(espClient);
TaskHandle_t MQTTSensorService;
TaskHandle_t MQTTMessageService;

void setupMQTTComms() 
{
  Serial.println("(setupMQTTComms)");
  initTopics(node.getNodeIDstring());
  client.setBufferSize(512);
  client.setServer(node.brokerIP, 1883);
  client.setCallback(MQTTcallback);
//  xTaskCreatePinnedToCore(messageReceiverTask, "Message Task", 2048, NULL, 1, &MQTTMessageService, 1);
  connectMQTTClient();
  runMQTT.attach_ms(500, serviceConnection);
}

boolean connectMQTTClient() 
{
  // Attempt to reconnect, return true if successful, otherwise false
  if (!client.connected()) 
  {
    Serial.println("(connectMQTTClient) Not connected");
    // Build clientID based on the NodeID
    String clientId = node.getNodeIDstring();
    clientId += "-";
    clientId += String(micros() & 0xff, 16); // to randomise. sort of
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) 
    {
      Serial.print("Connected to MQTT Broker at: ");
      Serial.println(node.brokerIP);
      client.subscribe(kPoseReporterTopic);
      MQTTConnectionTime = millis();
      return(true);     // Connected
    }
    return(false);    // Not yet connected
  } 
  return(true);         // already connected
}



boolean publishMQTT(char* topic, char* message)
{
   if (client.connected())
   {
      client.publish(topic , message);
      return(true);
   }
   return(false);
}


void serviceConnection()
{
  // Service the MQTT client
  if (client.connected()) 
  {
    client.loop();
  }
  else 
  {
    Serial.println("(serviceConnection) MQTT not connected - attempting reconnect");
    connectMQTTClient();
  }
}

/*
// Message Receiver task: waits for data from the message queue and publishes it
void messageReceiverTask(void *pvParameters) 
{
  MQTTMessagePayload receivedMessage;
  while (true) {
    // Wait indefinitely for data
    if (xQueueReceive(MQTTMessageQueue, &receivedMessage, portMAX_DELAY) == pdPASS) 
    {
      Serial.printf("[Message] Received value: %s\n", receivedMessage.message);
      publishMQTT(receivedMessage.topic, receivedMessage.message);
      vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
  }
}
*/

void MQTTcallback(char* topic, byte* payload, unsigned int length)
{
  if (strcmp(topic, kPoseReporterTopic) != 0) {
    return;
  }

  // Null-terminate the payload for sscanf
  char buf[64];
  if (length >= sizeof(buf)) {
    return;
  }
  memcpy(buf, payload, length);
  buf[length] = '\0';

  float x, y, bearing, f;
  if (sscanf(buf, "G1 X%f Y%f Z%f F%f", &x, &y, &bearing, &f) == 4) {
    receivedPose.x = x;
    receivedPose.y = y;
    receivedPose.bearing = bearing;
    receivedPose.valid = true;
    Serial.printf("[MQTT] Received pose: X=%.3f, Y=%.3f, Bearing=%.3f, F=%.3f\n", x, y, bearing, f);
  }
}

boolean checkMQTTState()
{
  // returns the MQTT client connection state
  return(client.connected());
}

unsigned long getMQTTUptime()
{
  if(checkMQTTState() == true)return((millis()-MQTTConnectionTime)/60000);
  else return(0);
}

void initTopics(char* currentNodeID)
{
  // Build the basic subscription strings by inserting the node ID
  int i;
  for(i=0; i<30 && GCodeDriverReporterTopic[i] != 0; i++)GCodeTopic[i] = GCodeDriverReporterTopic[i];
  GCodeTopic[15]=currentNodeID[0];
  GCodeTopic[16]=currentNodeID[1];  

}
