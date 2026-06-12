#include "NodeServices.h"
#include "FileSystem.h"

const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* nodeIDPath = "/nodeID.txt";
const char* brokerIPPath = "/brokerIP.txt";

MQTTnode node;

MQTTnode::MQTTnode()
{
  ID = 0;
//  hostName ="node00";
}

void MQTTnode::setNodeID(uint8_t nodeID)
{
  // ID is the master source for nodeID. Each time it is 
  // updated the following secondary stores are modified to suit
  // hostName    - takes the form 'nodenn'   where nn = nodeID in ASCII Hex
  // nodeIDString - stores the current nodeID in 2 digit ASCII Hex for for convenience
  ID = nodeID;
  Serial.print("setNodeID:");
  Serial.println(ID);
  hostName[0]='n';
  hostName[1]='o';
  hostName[2]='d';
  hostName[3]='e';
  hostName[6]='\0';
  toHex(ID);
  Serial.print("hostname:");
  Serial.println(hostName);
}

uint8_t MQTTnode::getNodeID()
{
  return(ID);
}


char* MQTTnode::getNodeIDstring()
{
  return(&nodeIDstring[0]);
}

void MQTTnode::loadConfig()
{
  // Loads up the node configuration data from SPIFFS
  // nodeID is stored as a two digit ASCII Hex form in the file
  // but master value in class is stored in ID (uint8_T)
  int i;
  const char* nodeIDstring = readFileC(SPIFFS, nodeIDPath, defaultNodeID);
  Serial.print("(loadConfig) nodeIDString:");
  Serial.println(nodeIDstring);
  char* endptr;
  int ID = strtol(nodeIDstring, &endptr, 16);                     // convert string from Hex to decimal
  Serial.print(ID);
  node.setNodeID(ID);

  // MQTT Broker IP Address
  /*********************************/
 // deleteFile(SPIFFS, brokerIPPath);
/***********************************/


  const char* brokerIPstr = readFileC(SPIFFS, brokerIPPath, defaultBrokerIP);
  for(i=0; (brokerIPstr[i] != '\0')&& (i<17); i++)node.brokerIP[i] = brokerIPstr[i];
  node.brokerIP[i]='\0';
  Serial.print("node.brokerIP:");
  Serial.println(node.brokerIP);
  
  //deleteFile(SPIFFS,ssidPath);
  // WiFi connection credentials
  //node.ssid = readFileC(SPIFFS, ssidPath,"");
  const char* ssidstr = readFileC(SPIFFS, ssidPath, defaultSSID);
  for(i=0; (ssidstr[i] != '\0')&& (i<17); i++)
  {
    node.ssid[i] = ssidstr[i];
  }
  node.ssid[i]='\0';
  Serial.print("node.ssid: ");
  Serial.println(node.ssid);

  //node.ssid = readFileC(SPIFFS, passPath,"");
  const char* passstr = readFileC(SPIFFS, passPath, defaultPassword);
  for(i=0; (passstr[i] != '\0')&& (i<17); i++)node.pass[i] = passstr[i];
  node.pass[i]='\0';
  Serial.print("node.pass: ");
  Serial.println(node.pass);

}

void MQTTnode::toHex(uint8_t val)
{
  // Convert given val into 2 digit Hex (ASCII) string
//    char* valStr = "00";
    char ch;
    ch = val/16 + 0x30;
    if(ch > 57)ch+=7;
    hostName[4]= ch;
    nodeIDstring[0] = ch;
    ch = val % 16 + 0x30;
    if(ch > 57)ch+=7;
    hostName[5] = ch;
    nodeIDstring[1] = ch;
    nodeIDstring[2] = '\0';
}