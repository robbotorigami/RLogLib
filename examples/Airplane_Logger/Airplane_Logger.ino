#include <Wire.h>
#include <SPI.h>
#include <RLog.h>

//Initialize a rlog object named board
RLog board(RLOG_EULER);

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
  IMUFusedData currentData;
  IMURawData currentRawData;

  unsigned long old = millis();
  board.readFusedData(&currentData);
  board.readRawData(&currentRawData);
  board.LogData(&currentData, &currentRawData);
  
  board.handleEvents();
  digitalWrite(13, !digitalRead(13));

}
