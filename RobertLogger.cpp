
#include "Arduino.h"

#include "RobertLogger.h"
RobertLogger::RobertLogger(){
	bno = BNO055();
	bmp = BMP180();
}

bool RobertLogger::initialize(){
	bno.initialize();
	bmp.initialize();
	if(!SD.begin(10)){
		return false;
	}
	
	return true;
}

void RobertLogger::initializeFiles(){
	int i = 0;
	sprintf(dataName, "data%i.csv", i);
	while(SD.exists(dataName)){
		i++;
		sprintf(dataName, "data%i.csv", i);
	}
	File dataFile = SD.open(dataName, FILE_WRITE);
	dataFile.print("Millis,Roll,Pitch,Yaw,Altitude,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Pressure\n");
	dataFile.close();
	
	
}

void RobertLogger::LogData(IMUFusedData fusedData, IMURawData rawData){
	
	File dataFile = SD.open(dataName, FILE_WRITE);
	unsigned long mill = millis();
	/*
	char roll[10] = "";
	char pitch[10] = "";
	char yaw[10] = "";
	char altitude[10] = "";
	char ax[10] = "";
	char ay[10] = "";
	char az[10] = "";
	char gx[10] = "";
	char gy[10] = "";
	char gz[10] = "";
	char mx[10] = "";
	char my[10] = "";
	char mz[10] = "";
	
	floatToString(roll, fusedData.roll, 2);
	floatToString(pitch, fusedData.pitch, 2);
	floatToString(yaw, fusedData.yaw, 2);
	floatToString(altitude, (float)fusedData.altitude, 2);
	
	floatToString(ax, rawData.accel_x, 2);
	floatToString(ay, rawData.accel_y, 2);
	floatToString(az, rawData.accel_z, 2);
	
	floatToString(gx, rawData.gyro_x, 2);
	floatToString(gy, rawData.gyro_y, 2);
	floatToString(gz, rawData.gyro_z, 2);
	
	floatToString(mx, rawData.mag_x, 2);
	floatToString(my, rawData.mag_y, 2);
	floatToString(mz, rawData.mag_z, 2);	
	
	char buffer[100];
	sprintf(buffer, "%i,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", mill, roll, pitch, yaw, altitude, ax, ay, az, gx, gy, gz, mx, my, mz);
	dataFile.print(buffer);*/
	
	dataFile.print(mill);
	dataFile.print(",");
	dataFile.print(fusedData.roll);
	dataFile.print(",");
	dataFile.print(fusedData.pitch);
	dataFile.print(",");
	dataFile.print(fusedData.yaw);
	dataFile.print(",");
	dataFile.print(fusedData.altitude);
	dataFile.print(",");
	
	
	dataFile.print(rawData.accel_x);
	dataFile.print(",");
	dataFile.print(rawData.accel_y);
	dataFile.print(",");
	dataFile.print(rawData.accel_z);
	dataFile.print(",");
	
	dataFile.print(rawData.gyro_x);
	dataFile.print(",");
	dataFile.print(rawData.gyro_y);
	dataFile.print(",");
	dataFile.print(rawData.gyro_z);
	dataFile.print(",");
	
	dataFile.print(rawData.mag_x);
	dataFile.print(",");
	dataFile.print(rawData.mag_y);
	dataFile.print(",");
	dataFile.print(rawData.mag_z);
	dataFile.print(",");
	
	dataFile.print(rawData.pressure);
	dataFile.print("\n");
	dataFile.close();
	
	
}

void RobertLogger::LogData(IMUFusedData fusedData){
	
	File dataFile = SD.open(dataName, FILE_WRITE);
	unsigned long mill = millis();
	
	dataFile.print(mill);
	dataFile.print(",");
	dataFile.print(fusedData.roll);
	dataFile.print(",");
	dataFile.print(fusedData.pitch);
	dataFile.print(",");
	dataFile.print(fusedData.yaw);
	dataFile.print(",");
	dataFile.print(fusedData.altitude);
	dataFile.print(",");
	dataFile.print("\n");
	dataFile.close();
	
	
}

IMUFusedData RobertLogger::readFusedData(){
	
	/*
	IMUFusedData currentData;
	bno.ReadRPY(&currentData.roll, &currentData.pitch, &currentData.yaw);
	
	bmp.takeTempuratureReading();
	delay(5);
	double Temperature = bmp.getTemperature();
	bmp.takePressureReading();
	delay(8);
	double Pressure = bmp.getPressure(Temperature);
	currentData.altitude = bmp.getAltitude(Pressure);
	return currentData;*/
	
	
	
	IMUFusedData currentData;
	
	bmp.takeTemperatureReading();
	delay(5);
	double Temperature = bmp.getTemperature();
	unsigned long old = millis();
	bmp.takePressureReading();
	bno.ReadRPY(&currentData.roll, &currentData.pitch, &currentData.yaw);
	while(millis()-old < 8);
	double Pressure = bmp.getPressure(Temperature);
	currentData.altitude = bmp.getAltitude(Pressure);
	return currentData;
}

IMUData RobertLogger::readAllData(){
	unsigned long old;
	IMUData currentData;
	
	old = millis();
	bmp.takeTemperatureReading();
	bno.ReadRPY(&currentData.fused.roll, &currentData.fused.pitch, &currentData.fused.yaw);
	while(millis()-old < 5);
	double Temperature = bmp.getTemperature();
	
	old = millis();
	bmp.takePressureReading();
	bno.ReadAccelRaw(&currentData.raw.accel_x, &currentData.raw.accel_y, &currentData.raw.accel_z);
	bno.ReadGyroRaw(&currentData.raw.gyro_x, &currentData.raw.gyro_y, &currentData.raw.gyro_z);
	bno.ReadMagRaw(&currentData.raw.mag_x, &currentData.raw.mag_y, &currentData.raw.mag_z);
	
	while(millis()-old < 8);
	currentData.raw.pressure = bmp.getPressure(Temperature);
	currentData.fused.altitude = bmp.getAltitude(currentData.raw.pressure);
	return currentData;
}


IMURawData RobertLogger::readRawData(){
	IMURawData currentData;
	
	
	
	unsigned long old = millis();
	bmp.takeTemperatureReading();
	bno.ReadAccelRaw(&currentData.accel_x, &currentData.accel_y, &currentData.accel_z);
	while(millis()-old < 5);
	double Temperature = bmp.getTemperature();
	
	old = millis();
	bmp.takePressureReading();
	bno.ReadGyroRaw(&currentData.gyro_x, &currentData.gyro_y, &currentData.gyro_z);
	bno.ReadMagRaw(&currentData.mag_x, &currentData.mag_y, &currentData.mag_z);
	while(millis()-old < 8);
	currentData.pressure = bmp.getPressure(Temperature);
	
	return currentData;
}

//Note, this function is only useful for small numbers
void RobertLogger::floatToString(char* buffer, float f, int decimals){
	int iPart = (int) f;
	int fPart = abs((int)((f-(float)iPart)*pow(10, decimals)));
	sprintf(buffer, "%d.%d", iPart, fPart);
	
} 

void RobertLogger::baselinePressure(){
	bmp.takeBaselineReading();
}