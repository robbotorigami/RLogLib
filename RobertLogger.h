#ifndef RobertLogger_h
#define RobertLogger_h

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "BNO055.h"



typedef struct{
	float roll;
	float pitch;
	float yaw;
	float altitude;
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
	float pressure;
} IMURawData;

class RobertLogger{
	public:
		RobertLogger();
		bool initialize();
		IMUFusedData readFusedData();
		IMURawData readRawData();
		
	private:
		BNO055 bno;
		
}









#endif