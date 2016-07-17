#include "Arduino.h"

#include <Wire.h>

#include "BME280.h"

BME280::BME280(){
	
}

void BME280::initialize(){
	Wire.begin();
	dig_T1 = (readByte(0x89)<<8) | readByte(0x88);
	dig_T2 = (readByte(0x8B)<<8) | readByte(0x8A);
	dig_T3 = (readByte(0x8D)<<8) | readByte(0x8C);
	
	dig_P1 = (readByte(0x8F)<<8) | readByte(0x8E);
	dig_P2 = (readByte(0x91)<<8) | readByte(0x90);
	dig_P3 = (readByte(0x93)<<8) | readByte(0x92);
	dig_P4 = (readByte(0x95)<<8) | readByte(0x94);
	dig_P5 = (readByte(0x97)<<8) | readByte(0x96);
	dig_P6 = (readByte(0x99)<<8) | readByte(0x98);
	dig_P7 = (readByte(0x9B)<<8) | readByte(0x9A);
	dig_P8 = (readByte(0x9D)<<8) | readByte(0x9C);
	dig_P9 = (readByte(0x9F)<<8) | readByte(0x9E);
	
	dig_H1 = readByte(0xA1);
	dig_H2 = (readByte(0xE2)<<8) | readByte(0xE1);
	dig_H3 = readByte(0xE3);
	dig_H4 = (readByte(0xE4)<<4) | (readByte(0xE5) & 0x0F);
	dig_H5 = (readByte(0xE6)<<4)| (readByte(0xE5)>>4);
	dig_H6 = readByte(0xE7);
	
	writeByte(0xF4, 0x00);
	//Configure
	//110 - 10ms standby time
	//010 - 4x filter
	//00 SPI, dont care
	writeByte(0xF5, 0xC8);
	
	//Configure
	//00000xxx
	//011 4x oversampling humidity
	writeByte(0xF2, 0x03);
	
	//Configure
	//xxxyyyzz
	//011 - 4x oversampling for temp
	//011 - 4x oversampling for pres
	//11 - Normal mode
	
	writeByte(0xF4, 0x6F);
	
	delay(3000); //Give time for start up
	
	takeBaselineReading();
}

void BME280::writeByte(byte reg, byte value){
	Wire.beginTransmission(BME_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.write((uint8_t) value);
	Wire.endTransmission();
}

byte BME280::readByte(byte reg){
	Wire.beginTransmission(BME_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BME_ADDRESS, (uint8_t)1);
	return Wire.read();
}

void BME280::readMultiple(byte reg, byte* buffer, uint8_t numBytes){
	Wire.beginTransmission(BME_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BME_ADDRESS, (uint8_t)numBytes);
	
	for(int i=0; i<numBytes; i++){
		buffer[i] = Wire.read();
	}
}

void BME280::takeBaselineReading(){
	getTemperature();
	baselinePressure = getPressure();
}

double BME280::getTemperature(){
	uint8_t rawData[3];
	readMultiple(0xFA, (uint8_t*)rawData, 3);
	//Return temperature in degrees C
	int32_t adc_T = ((uint32_t)rawData[0] <<12) |((uint32_t)rawData[1]<<4) | (((uint32_t)rawData[2]>>4)&0x0F);
	
	int32_t var1, var2;
	
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11; 
 	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * 
 	((int32_t)dig_T3)) >> 14; 
 	t_fine = var1 + var2; 
 	double output = (t_fine * 5 + 128) >> 8; 
 
 
 	output = output / 100; 
	 
 	return output; 

}

double BME280::getPressure(){
	//Returns pressure in Pa
	//Must call getTemperature before calling
	uint8_t rawData[3];
	readMultiple(0xF7, (uint8_t*)rawData, 3);
	
	int32_t adc_P = ((uint32_t)rawData[0] <<12) |((uint32_t)rawData[1]<<4) | (((uint32_t)rawData[2]>>4)&0x0F);
	
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1*var1*(int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<34);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2) <<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if(var1 == 0)
	{
		return 0; //don't want to divide by 0
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1+ var2) >> 8)+(((int64_t)dig_P7)<<4);
	
	return (double)p/256.0;
}

double BME280::getAltitude(double P){
	return ((double)45846.1)*(1.0-pow(P/baselinePressure, 0.190263));
}