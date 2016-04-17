#ifndef BME280_h
#define BME280_h

#include "Arduino.h"

class BME280{
	public:
		BME280();
		
		void initialize();
		
		void takeTemperatureReading();
		
		void takePressureReading();
		
		double getTemperature();		
		double getPressure();
		double getAltitude(double);
		
		void takeBaselineReading();
		
	private:
		static const byte BME_ADDRESS = 0x76;
		void writeByte(byte, byte);
		byte readByte(byte);
		double baselinePressure;
		uint16_t dig_T1, dig_P1;
		int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
		uint8_t dig_H1, dig_H3;
		int8_t dig_H6;
		int32_t t_fine;
};



#endif