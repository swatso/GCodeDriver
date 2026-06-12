
// Collection of functions which allow access to the SPIFFS file system
#include "FileSystem.h"
#include <SPIFFS.h>                               	

#define HASH 35

// File paths to save input values permanently
const char* nodeConfigPath = "/config#.txt";
SemaphoreHandle_t fsLock;

// Initialize SPIFFS
void setupSPIFFS() 
{
  if (!SPIFFS.begin(true)) 
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
  fsLock = xSemaphoreCreateMutex();
}

// Read a File from SPIFFS, if the file does not exist then a new one is created to store the default string (deft)
// Simply reads a single string from SPIFFS file
char* readFileC(fs::FS &fs, const char *path, const char *deft)
{
//  Serial.printf("(readFileC): %s\r\n", path);
  vTaskDelay(20);
  xSemaphoreTake(fsLock,portMAX_DELAY);
  static char fileContent[20];
  uint8_t i;
  for(i=0; i<20; i++)fileContent[i]='\0';     // clear the static buffer
  File file = fs.open(path);
  if(!file || file.isDirectory())
  {
//    Serial.print("- failed to open file for reading, creating file and restoring default:");
//    Serial.println(deft);
    for(i=0; deft[i]>0x1F;i++)fileContent[i]=deft[i];
    fileContent[i]='\0';
//Serial.println(fileContent);
    xSemaphoreGive(fsLock);
    writeFile(SPIFFS, path, fileContent);
    return (&fileContent[0]);
  }
  // file is available, read it
  i=0;
  while(file.available())
  {
    fileContent[i]=file.read();
//        Serial.print("fileContent[i]:");
//      Serial.println(Serial.printf("%d \n",fileContent[i]));
    if(fileContent[i]>0x1F)i++;
    else
    {
      
      fileContent[i] = '\0';
//      Serial.print("i:");
//      Serial.println(i);
      while(file.available())file.read();
    }
  }
      int e = i;
//    Serial.println("ReadFileC");
//    for(i=0; i<=e; i++)Serial.printf("%d \n",fileContent[i]);
//    Serial.println("done");
  file.close();
  xSemaphoreGive(fsLock);
  return(&fileContent[0]);
}

// Read a File from SPIFFS, if the file does not exist then a new one is created to store the default string (deft)
// Simply reads a single string from SPIFFS file
String readFile(fs::FS &fs, const char *path, const char *deft)
{
//  Serial.printf("(readFile): %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory())
  {
//    Serial.print("- failed to open file for reading, creating file and restoring default:");
//    Serial.println(deft);
    file.close();
    xSemaphoreGive(fsLock);
    writeFile(SPIFFS, path, deft);
    return (deft);
  }
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    //Serial.print(fileContent);
    break;     
  }
  file.close();
  return fileContent;
}

// Write file to SPIFFS
// Creates a file and writes the string into  it
void writeFile(fs::FS &fs, const char * path, const char * message)
{
//  Serial.printf("Writing char* to file: %s\r\n", path);
//  Serial.println(message);

  xSemaphoreTake(fsLock,portMAX_DELAY);
  File file = fs.open(path, FILE_WRITE);
  if(!file){
//    Serial.println("- failed to open file for writing");
    xSemaphoreGive(fsLock);
    return;
  }
  if(file.print(message))
  {
    //Serial.println("- file written");
  } 
  else 
  {
    //Serial.println("- write failed");
  }
  file.close();
  xSemaphoreGive(fsLock);
}

// Write single integer to SPIFFS
void writeFile(fs::FS &fs, const char * path, int value)
{
  //Serial.printf("Writing integer to file: %s\r\n", path);

  xSemaphoreTake(fsLock,portMAX_DELAY);
  File file = fs.open(path, FILE_WRITE);
  if(!file)
  {
    //Serial.println("- failed to open file for writing");
    xSemaphoreGive(fsLock);
    return;
  }
  if(file.print(value))
  {
    //Serial.println("- file written");
  } 
  else 
  {
    //Serial.println("- write failed");
  }
  file.close();
  xSemaphoreGive(fsLock);
}


void  deleteFile(fs::FS &fs, const char * path)
{
  xSemaphoreTake(fsLock,portMAX_DELAY);
  fs.remove(path);
  xSemaphoreGive(fsLock);
}

char* hashPath(const char* path, uint8_t bit)
{
  // replaces the 'hash' in the given path with the single digit 
  // hexadecimal equivalent of the supplied bit
  static char newPath[30];
  uint8_t i,j;
  for(i=0, j=0; path[i] != 0; i++)
  {
    if(path[i] != HASH)newPath[j++] = path[i];
    else
    {
      char buffer[2];
      sprintf(buffer, "%01X", bit);
      newPath[j++]=buffer[0];
    }
  }
  newPath[j]=0;
  return(&newPath[0]);
}