#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <cstring>
#include <Arduino.h>

unsigned long WiFiGetCommsUptime();                             
String WiFiGetIPAddress();                                      
String WiFiGetSSID();                                           
int WiFiGetRSSI(); 
void setupWiFi(); 
bool initWiFi();                                            
void checkWiFiConnection();
void serviceWiFi();

extern char GCodeTopic[];

#endif // WIFIMANAGER_H
