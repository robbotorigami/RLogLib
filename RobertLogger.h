#ifndef RobertLogger_h
#define RobertLogger_h

#include "Arduino.h"

#include "BNO055.h"
#include "BMP180.h"

#include <SPI.h>
#include <SD.h>
#include <String.h>

typedef struct{
	float roll;
	float pitch;
	float yaw;
	double altitude;
} IMUFusedData;

typedef struct{
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	double pressure;
} IMURawData;

typedef struct{
	IMUFusedData fused;
	IMURawData raw;
} IMUData;

class RobertLogger{
	public:
		RobertLogger();
		bool initialize();
		IMUFusedData readFusedData();
		IMURawData readRawData();
		IMUData readAllData();
		void initializeFiles();
		void baselinePressure();
		void LogData(IMUFusedData, IMURawData);
		void LogData(IMUFusedData);
		
	private:
		BNO055 bno;
		BMP180 bmp;
		char dataName[8];
		static void floatToString(char*, float, int);
		
};









#endif