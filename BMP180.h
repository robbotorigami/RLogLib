#ifndef BMP180_h
#define BMP180_h

#include "Arduino.h"

class BMP180{
	public:
		BMP180();
		
		void initialize();
		
		void takeTemperatureReading();
		
		void takePressureReading();
		
		double getTemperature();		
		double getPressure(double);
		double getAltitude(double);
		
		void takeBaselineReading();
		
	private:
		static const byte BMP_ADDRESS = 0x77;
		void writeByte(byte, byte);
		byte readByte(byte);
		int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
		uint16_t AC4,AC5,AC6; 
		double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
		double baselinePressure;
};



#endif