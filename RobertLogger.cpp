
#include "Arduino.h"

#include "RobertLogger.h"
RobertLogger::RobertLogger(){
	
}

void RobertLogger::initialize(){
	bno = new BNO055();
}

IMUFusedData RobertLogger::readFusedData(){
	IMUFusedData currentData;
	bno.ReadRPY(currentData.roll, currentData.pitch, currentData.yaw);
	
	//TODO: Add in the code for BMP180 read
	
	return currentData;
}


IMURawData RobertLogger::readRawData(){
	IMURawData currentData;
	bno.ReadAccelRaw(currentData.accel_x, currentData.accel_y, currentData.accel_z);
	bno.ReadGyroRaw(currentData.gyro_x, currentData.gyro_y, currentData.gyro_z);
	bno.ReadMagRaw(currentData.mag_x, currentData.mag_y, currentData.mag_z);
	
	//TODO: Add in the code for BMP180 read
	return currentData;
}