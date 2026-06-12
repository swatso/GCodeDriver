#ifndef NODESERVICES_H_
#define NODESERVICES_H_


//============ Includes ====================
#include <cstdint>


#define defaultBrokerIP "192.168.0.2"
#define defaultNodeID "00"
#define defaultHostName "node00"
#define defaultSSID "defaultSSID"
#define defaultPassword "defaultPassword"
  

extern const char* ssidPath;
extern const char* passPath;
extern const char* nodeIDPath;
extern const char* brokerIPPath;


class MQTTnode 
{
private:
  uint8_t ID;
  char nodeIDstring[3];
  void toHex(uint8_t val);

public:
  char hostName[7];
  char brokerIP[16];
  char ssid[20];
  char pass[20];

  // Constructor
  MQTTnode();

  // Set the Node ID
  void loadConfig();
  void setNodeID(uint8_t ID);
  char* getNodeIDstring();
  uint8_t getNodeID();
};

extern MQTTnode node;

#endif // NODESERVICES_H_