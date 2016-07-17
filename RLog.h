#ifndef RLog_h
#define RLog_h

//Set the i2c speed to be 400kHz
#define TWI_FREQ 400000L

#include "Arduino.h"

#include "BNO055.h"
#include "BME280.h"

#include <SPI.h>
#include <SdFat.h>
#include <String.h>

#define UP 1
#define DOWN 0

#define MAX_ALTITUDE_EVENTS 1

typedef struct{
	float datax; //Euler: Roll,  Quaternion: x
	float datay; //Euler: Pitch, Quaternion: y
	float dataz; //Euler: Yaw,   Quaternion: z
	float dataw; //Euler: None,  Quaternion: w
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

typedef enum{
	RLOG_QUATERNION,
	RLOG_EULER
}RLogMode;

typedef struct{
	void (*function)(void);
	float altitude;
	uint8_t direction;
} altitudeEvent;

class RLog{
	public:
		RLog(RLogMode);
		
		
		bool initialize();
		
		void readFusedData(IMUFusedData*);
		void readRawData(IMURawData*);
		void readAllData(IMUData*);
		void initializeFiles();
		void baselinePressure();
		void handleEvents();
		void addAltitudeEvent(void (*)(void), float, uint8_t);
		void LogData(IMUFusedData*, IMURawData*);
		void LogData(IMUFusedData*);
		
	private:
		BNO055 bno;
		BME280 bme;
		RLogMode mode;
		SdFat sd;
		
		//Events
		altitudeEvent altitudeEvents[MAX_ALTITUDE_EVENTS];
		uint8_t numAltEvents;
		
		// Log file.
		SdFile dataFile;
		char dataName[8];
		static int floatToString(char*, float, int);
		
};









#endif