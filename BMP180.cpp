#include "Arduino.h"

#include <Wire.h>

#include "BMP180.h"

BMP180::BMP180(){
	
}

void BMP180::initialize(){
	Wire.begin();
	double c3,c4,b1;
	
	AC1 = (readByte(0xAA)<<8) | readByte(0xAB);
	AC2 = (readByte(0xAC)<<8) | readByte(0xAD);
	AC3 = (readByte(0xAE)<<8) | readByte(0xAF);
	AC4 = (readByte(0xB0)<<8) | readByte(0xB1);
	AC5 = (readByte(0xB2)<<8) | readByte(0xB3);
	AC6 = (readByte(0xB4)<<8) | readByte(0xB5);
	VB1 = (readByte(0xB6)<<8) | readByte(0xB7);
	VB2 = (readByte(0xB8)<<8) | readByte(0xB9);
	MB = (readByte(0xBA)<<8) | readByte(0xBB);
	MC = (readByte(0xBC)<<8) | readByte(0xBD);
	MD = (readByte(0xBE)<<8) | readByte(0xBF);
	
	c3 = 160.0 * pow(2,-15) * AC3;
	c4 = pow(10,-3) * pow(2,-15) * AC4;
	b1 = pow(160,2) * pow(2,-30) * VB1;
	c5 = (pow(2,-15) / 160) * AC5;
	c6 = AC6;
	mc = (pow(2,11) / pow(160,2)) * MC;
	md = MD / 160.0;
	x0 = AC1;
	x1 = 160.0 * pow(2,-13) * AC2;
	x2 = pow(160,2) * pow(2,-25) * VB2;
	y0 = c4 * pow(2,15);
	y1 = c4 * c3;
	y2 = c4 * b1;
	p0 = (3791.0 - 8.0) / 1600.0;
	p1 = 1.0 - 7357.0 * pow(2,-20);
	p2 = 3038.0 * 100.0 * pow(2,-36);
	
	takeBaselineReading();
	
}

void BMP180::writeByte(byte reg, byte value){
	Wire.beginTransmission(BMP_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.write((uint8_t) value);
	Wire.endTransmission();
}

byte BMP180::readByte(byte reg){
	Wire.beginTransmission(BMP_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BMP_ADDRESS, (uint8_t)1);
	return Wire.read();
}

void BMP180::takeBaselineReading(){
	takeTemperatureReading();
	delay(5);
	double Temperature = getTemperature();
	takePressureReading();
	delay(8);
	baselinePressure = getPressure(Temperature);
}

void BMP180::takePressureReading(){
	writeByte(0xF4, 0x74);
}

void BMP180::takeTemperatureReading(){
	writeByte(0xF4, 0x2E);
}

double BMP180::getTemperature(){
	double tu, a;
	
	byte data1 = readByte(0xF6);
	byte data2 = readByte(0xF7);
	
	tu = (data1 * 256.0) + data2;
	a = c5 * (tu - c6);
	double T = a + (mc / (a + md));
	
	return T;
}

double BMP180::getPressure(double T){
	
	double pu,s,x,y,z;
	
	byte data1 = readByte(0xF6);
	byte data2 = readByte(0xF7);
	byte data3 = readByte(0xF8);
	
	pu = (data1 * 256.0) + data2 + (data3/256.0);
	s = T - 25.0;
	x = (x2 * pow(s,2)) + (x1 * s) + x0;
	y = (y2 * pow(s,2)) + (y1 * s) + y0;
	z = (pu - x) / y;
	double P = (p2 * pow(z,2)) + (p1 * z) + p0;
	return P;
}

double BMP180::getAltitude(double P){
	return(44330.0*(1-pow(P/baselinePressure,1/5.255)));
}