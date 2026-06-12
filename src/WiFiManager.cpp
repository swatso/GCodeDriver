/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-wi-fi-manager-asyncwebserver/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include "WiFiManager.h"
#include "NodeServices.h"
#include "FileSystem.h"
#include "MQTTComms.h"

#include <Arduino.h>                              		
#include <WiFi.h>                                 		
#include <ESPAsyncWebServer.h>                    		
#include <AsyncTCP.h>                             		
#include <ESPmDNS.h>                              		
#include <ArduinoJson.h>                          		
#include <AsyncJson.h>                            		
#include <WiFiUdp.h>                              		
#include <ArduinoOTA.h>                           		

// Search for parameter in HTTP POST request when in access point mode


const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "hostName";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String brokerIP;
//String nodeIDstring;
int sceneNo;
//int nodeID;

//Variables to save values from HTML form
String ssid;
String pass;
//String hostName;

boolean haveConnected;
long wifiConnectionTime;

// Timer variables
unsigned long previousMillis = 0;
const long interval = 15000;  // interval to wait for Wi-Fi connection (milliseconds)

// Initialize WiFi
bool initWiFi() 
{
  haveConnected = false;
  WiFi.setHostname(node.hostName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(node.ssid, node.pass);
  Serial.println(" Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) 
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) 
    {
      Serial.println("Failed to connect.");
      return false;
    }
    yield();
  }
  String localHostIP = WiFiGetIPAddress();
  Serial.println(localHostIP);
  Serial.println("(initOTA)...");
  ArduinoOTA.setHostname(node.hostName);
  ArduinoOTA
    .onStart([]() 
    {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
      {
        type = "filesystem";
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        SPIFFS.end();
      }
      Serial.println("Start updating " + type);
    })
    
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  if(!MDNS.begin(node.hostName)) 
  {
     Serial.println("Error starting mDNS");
  }
  else
  {
    MDNS.addService("_http", "_tcp", 80);
    Serial.print("MDNS started for host:");
    Serial.println(node.hostName);
  }
  haveConnected = true;
  return true;
}

void  checkWiFiConnection()
{
  // Checks the Wifi connection and re-connects if it is down
  // Call it from the main loop every 15 seconds or so...
  if(haveConnected == true)
  {
    // Have connected to WiFi at some point
    if(WiFi.status() != WL_CONNECTED)
    {
      // Wifi connection has dropped out, try and restart it
      wifiConnectionTime = 0;
      Serial.println("(checkWiFiConnection) WiFi dropped out");
      WiFi.disconnect();
      WiFi.reconnect();
      if(WiFi.status() == WL_CONNECTED)wifiConnectionTime = millis();
    }
    else 
    {
//      Serial.println("(checkWiFiConnection) WiFi ok");
    }
  }
  else 
  {
    Serial.print("(checkWiFiConnection) WiFi never connected");
  }
}

void setupWiFi() 
{
  if(initWiFi()) 
  {
//    Serial.println("initialising web server");
    server.serveStatic("/", SPIFFS, "/");
    
    // Route for /home web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("Serve node.html");
      request->send(SPIFFS, "/node.html", "text/html", false);
    });

    // Route for /node web page
    server.on("/page/node", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("Serve node.html");
      request->send(SPIFFS, "/node.html", "text/html", false);
    });

    // Route for node configuration web page
    server.on("/page/nodeconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("Serve nodeConfig.html");
      request->send(SPIFFS, "/nodeConfig.html", "text/html", false);
    });

    // Route for /favicon
    server.on("/favicon", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("Serve favicon.png");
      request->send(SPIFFS, "/Favicon.png", "image/png", false);
    });

    //======== Server REST WebAPI Endpoints ===============================//
    //
    // GET endpoints
    //
    
    server.on("/api/nodeid/value", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (request->url() == "/api/nodeid/value") 
      {
        Serial.println("Received /api/nodeid/value GET request");
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        JsonDocument doc;
        doc["value"] = node.getNodeIDstring();
        serializeJson(doc,*response);  
        request->send(response);
      }
    });

    server.on("/api/brokerip/value", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (request->url() == "/api/brokerip/value") 
      {
        //Serial.println("Received /api/brokerip/value GET request");
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        JsonDocument doc;
        doc["value"] = node.brokerIP;
        serializeJson(doc,*response);  
        request->send(response);
      }
    });

    server.on("/api/hostname/value", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (request->url() == "/api/hostname/value") 
      {
//        Serial.println("Received /api/hostname/value GET request");
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        JsonDocument doc;
        doc["value"] =node.hostName;
        serializeJson(doc,*response);  
        request->send(response);
      }
    });

    server.on("/api/node/status", HTTP_GET, [](AsyncWebServerRequest *request) 
    {

      if (request->url() == "/api/node/status") 
      {
//        Serial.println("Received /api/node/status GET request");
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        JsonDocument doc;
        doc["WIFIuptime"] = WiFiGetCommsUptime();
        doc["ipAddress"] = WiFiGetIPAddress();
        doc["rssi"] = WiFiGetRSSI();
        doc["mqttState"] = checkMQTTState();
        doc["mqttUptime"] = getMQTTUptime();
        serializeJson(doc,*response);  
        request->send(response);
      }
    });


    // Handle POST requests
    // ********************
    server.onRequestBody([](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) 
    {
      Serial.println("Received api POST request");
      Serial.println(request->url());

      if (request->url() == "/api/node/restart") 
      {
        Serial.println("Restart Requested");
        delay(1000);
        ESP.restart();
      }

      if (request->url() == "/api/nodeid/value") 
      {
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, (const char*)data);
          if(error)
          {
              Serial.println("Deserialisationerror");
          }
          else
          {
            const char* nodeID = doc["value"].as<const char*>();
//            Serial.print("MQTT NodeID:");
//            Serial.println(nodeID);    
            node.setNodeID(strtol(nodeID,NULL,16));
            writeFile(SPIFFS, nodeIDPath, node.getNodeIDstring());
//            Serial.print("MQTT NodeID:");
//            Serial.println(node.getNodeIDstring());
          }
      }

      if (request->url() == "/api/brokerip/value") 
      {
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, (const char*)data);
          if(error)
          {
              Serial.println("Deserialisationerror");
          }
          else
          {
            //String bIP = doc["value"];
            const char* buff;
            buff = doc["value"].as<const char*>();
            writeFile(SPIFFS, brokerIPPath, buff); 
            strcpy(node.brokerIP,buff);
            //Serial.print("MQTT brokerIP:");
            //Serial.println(node.brokerIP);
          }
      }

      if (request->url() == "/api/hostname/value") 
      {
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, (const char*)data);
          if(error)
          {
              Serial.println("Deserialisationerror");
          }
          else
          {
            const char* buff;
            buff = doc["value"].as<const char*>();
//            writeFile(SPIFFS, hostNamePath, buff); 
            strcpy(node.hostName,buff);
            //Serial.print("Hostname:");
            //Serial.println(node.hostName);
          }
      }

      //Serial.println("200");
      request->send(200,"application/json","OK");
    });      
    server.begin();
  }
  else 
  {
    // Connect to Wi-Fi network with SSID and password
    Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP); 

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost())
        {
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) 
          {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to host (.local): ");
      delay(3000);
      ESP.restart();
    });
    server.begin();
  }
}


unsigned long WiFiGetCommsUptime()
{
  if(wifiConnectionTime != 0)return((millis()-wifiConnectionTime)/60000);
  else return(0);
}

String WiFiGetIPAddress()
{
  return(WiFi.localIP().toString());
}

String WiFiGetSSID()
{
  return(WiFi.SSID());
}

int WiFiGetRSSI()
{
  return(WiFi.RSSI());
}
