#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RobertLogger.h>

//Initialize a robertlogger object named board
RobertLogger board;
void setup() {
  Serial.begin(9600);
  Serial.println("Startup");
  pinMode(13, OUTPUT);
  if(!board.initialize()){
    Serial.println("Error");
    while(true){
    }
  }
  //Sets up the data file on the SD card
  board.initializeFiles();
}

void loop() {
  
  IMUFusedData currentData = board.readFusedData();
  IMURawData currentRawData = board.readRawData();
  board.LogData(currentData, currentRawData);
  
//  Serial.print("Roll:");Serial.print(currentData.roll);Serial.print(" degrees, ");
//  Serial.print("Pitch:");Serial.print(currentData.pitch);Serial.print(" degrees, ");
//  Serial.print("Yaw:");Serial.print(currentData.yaw);Serial.print(" degrees, ");
//  Serial.print("Altitude:");Serial.print(currentData.altitude);Serial.println(" meters");

  digitalWrite(13, !digitalRead(13));

}
