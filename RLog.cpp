
#include "Arduino.h"

#include "RLog.h"
RLog::RLog(){
	bno = BNO055();
	bme = BME280();
}

bool RLog::initialize(){
	bno.initialize();
	bme.initialize();
	if(!SD.begin(10)){
		return false;
	}
	
	return true;
}

void RLog::initializeFiles(){
	int i = 0;
	sprintf(dataName, "data%i.csv", i);
	while(SD.exists(dataName)){
		i++;
		sprintf(dataName, "data%i.csv", i);
	}
	File dataFile = SD.open(dataName, FILE_WRITE);
	dataFile.print("Millis,Roll,Pitch,Yaw,Altitude,Temperature,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Pressure\n");
	dataFile.close();
}

void RLog::LogData(IMUFusedData* fusedData, IMURawData* rawData){
	File dataFile = SD.open(dataName, FILE_WRITE);
	unsigned long mill = millis();
	
	/*
	char bufferString[200];
	char convBuffer[20];
	char* buffer = bufferString;
	
	buffer += sprintf(buffer, "%d", mill);
	
	buffer += floatToString(buffer, fusedData->roll, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->pitch, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->yaw, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->altitude, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->temperature, 3);
	*(buffer++) = ',';
	
	buffer += floatToString(buffer, rawData->accel_x, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, rawData->accel_y, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, rawData->accel_z, 3);
	*(buffer++) = ',';
	
	buffer += floatToString(buffer, rawData->gyro_x, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, rawData->gyro_y, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, rawData->gyro_z, 3);
	*(buffer++) = ',';
	
	buffer += floatToString(buffer, rawData->mag_x, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, rawData->mag_y, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, rawData->mag_z, 3);
	*(buffer++) = ',';
	
	buffer += floatToString(buffer, rawData->pressure, 3);
	sprintf(buffer, "\n");
	
	dataFile.print(buffer);
	dataFile.close();
	*/
	
	
	dataFile.print(mill);
	dataFile.print(",");
	dataFile.print(fusedData->roll);
	dataFile.print(",");
	dataFile.print(fusedData->pitch);
	dataFile.print(",");
	dataFile.print(fusedData->yaw);
	dataFile.print(",");
	dataFile.print(fusedData->altitude);
	dataFile.print(",");
	dataFile.print(fusedData->temperature);
	dataFile.print(",");
	
	
	dataFile.print(rawData->accel_x);
	dataFile.print(",");
	dataFile.print(rawData->accel_y);
	dataFile.print(",");
	dataFile.print(rawData->accel_z);
	dataFile.print(",");
	
	dataFile.print(rawData->gyro_x);
	dataFile.print(",");
	dataFile.print(rawData->gyro_y);
	dataFile.print(",");
	dataFile.print(rawData->gyro_z);
	dataFile.print(",");
	
	dataFile.print(rawData->mag_x);
	dataFile.print(",");
	dataFile.print(rawData->mag_y);
	dataFile.print(",");
	dataFile.print(rawData->mag_z);
	dataFile.print(",");
	
	dataFile.print(rawData->pressure);
	dataFile.print("\n");
	dataFile.close();
	
}

void RLog::LogData(IMUFusedData *fusedData){
	
	File dataFile = SD.open(dataName, FILE_WRITE);
	unsigned long mill = millis();
	
	dataFile.print(mill);
	dataFile.print(",");
	dataFile.print(fusedData->roll);
	dataFile.print(",");
	dataFile.print(fusedData->pitch);
	dataFile.print(",");
	dataFile.print(fusedData->yaw);
	dataFile.print(",");
	dataFile.print(fusedData->altitude);
	dataFile.print(",");
	dataFile.print(fusedData->temperature);
	dataFile.print("\n");
	dataFile.close();
	
	
}

void RLog::readFusedData(IMUFusedData * currentData){
	
	bno.ReadRPY(&currentData->roll, &currentData->pitch, &currentData->yaw);
	double Temperature = bme.getTemperature();
	double Pressure = bme.getPressure();
	currentData->temperature = Temperature;
	currentData->altitude = bme.getAltitude(Pressure);
}


void RLog::readRawData(IMURawData* currentData){
	
	bno.ReadAccelRaw(&currentData->accel_x, &currentData->accel_y, &currentData->accel_z);	
	bno.ReadGyroRaw(&currentData->gyro_x, &currentData->gyro_y, &currentData->gyro_z);
	bno.ReadMagRaw(&currentData->mag_x, &currentData->mag_y, &currentData->mag_z);
	
	double Temperature = bme.getTemperature();
	currentData->pressure = bme.getPressure();
	
}

//Note, this function is only useful for small numbers
int RLog::floatToString(char* buffer, float f, int decimals){
	int iPart = (int) f;
	int fPart = abs((int)((f-(float)iPart)*pow(10, decimals)));
	return sprintf(buffer, "%d.%d", iPart, fPart);
} 

void RLog::baselinePressure(){
	bme.takeBaselineReading();
}