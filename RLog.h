#ifndef RLog_h
#define RLog_h

#include "Arduino.h"

#include "BNO055.h"
#include "BME280.h"

#include <SPI.h>
#include <SD.h>
#include <String.h>

typedef struct{
	float roll;
	float pitch;
	float yaw;
	double altitude;
	double temperature;
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

class RLog{
	public:
		RLog();
		bool initialize();
		void readFusedData(IMUFusedData*);
		void readRawData(IMURawData*);
		void readAllData(IMUData*);
		void initializeFiles();
		void baselinePressure();
		void LogData(IMUFusedData*, IMURawData*);
		void LogData(IMUFusedData*);
		
	private:
		BNO055 bno;
		BME280 bme;
		char dataName[8];
		//File dataFile;
		static int floatToString(char*, float, int);
		
};









#endif