

#include "Arduino.h"

#include "RLog.h"
RLog::RLog(RLogMode modein):
	mode(modein)
{
	bno = BNO055();
	bme = BME280();
	numAltEvents = 0;
}

bool RLog::initialize(){
	//TODO check for correct initialization of BNO and BME
	bno.initialize();
	bme.initialize();
	
	pinMode(10, OUTPUT);
	digitalWrite(10, HIGH);
	
	if(!sd.begin(10, SPI_FULL_SPEED)){
		return false;
	}
	
	return true;
	
}

void RLog::initializeFiles(){
	int i = 0;
	sprintf(dataName, "data%i.csv", i);
	while(sd.exists(dataName)){
		i++;
		sprintf(dataName, "data%i.csv", i);
	}
	dataFile.open(dataName, O_CREAT | O_WRITE | O_EXCL);
	switch(mode){
		case RLOG_EULER:
			dataFile.print("Millis,Roll,Pitch,Yaw,Unused,Altitude,Temperature,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Pressure\n");
			break;
		case RLOG_QUATERNION:
			dataFile.print("Millis,X,Y,Z,W,Altitude,Temperature,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Pressure\n");
			break;
	}
	//dataFile.close();
}


void RLog::handleEvents(){
	static double prevAlt = 0;
	double currAlt = bme.getAltitude(bme.getPressure());
	for(int i = 0; i < numAltEvents; i++){
		if(altitudeEvents[i].direction == UP){
			if(currAlt > altitudeEvents[i].altitude && prevAlt < altitudeEvents[i].altitude)
				(*altitudeEvents[i].function)();
		}else{
			if(currAlt < altitudeEvents[i].altitude && prevAlt > altitudeEvents[i].altitude)
				(*altitudeEvents[i].function)();
		}
	}
	prevAlt = currAlt;
	
}
void RLog::addAltitudeEvent(void (*function)(void), float altitude, uint8_t direction){
	altitudeEvents[numAltEvents].function = function;
	altitudeEvents[numAltEvents].altitude = altitude;
	altitudeEvents[numAltEvents].direction = direction;
	numAltEvents++;
}

void RLog::LogData(IMUFusedData* fusedData, IMURawData* rawData){
	char bufferString[200];
	char* buffer = bufferString;
	
	buffer += sprintf(buffer, "%lu,", millis());
	
	buffer += floatToString(buffer, fusedData->datax, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->datay, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->dataz, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->dataw, 3);
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
	buffer += sprintf(buffer, "\n");
	
	dataFile.print(bufferString);
	dataFile.sync();	
}

void RLog::LogData(IMUFusedData *fusedData){
	char bufferString[200];
	char* buffer = bufferString;
	
	buffer += sprintf(buffer, "%lu,", millis());
	
	buffer += floatToString(buffer, fusedData->datax, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->datay, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->dataz, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->dataw, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->altitude, 3);
	*(buffer++) = ',';
	buffer += floatToString(buffer, fusedData->temperature, 3);
	*(buffer++) = ',';
	buffer += sprintf(buffer, "\n");
	
	dataFile.print(bufferString);
	dataFile.flush();
	
	
}

void RLog::readFusedData(IMUFusedData* currentData){
	switch(mode){
		case RLOG_EULER:
			bno.ReadRPY(&currentData->datax, &currentData->datay, &currentData->dataz);
			break;
		case RLOG_QUATERNION:
			bno.ReadQuaternion(&currentData->datax, &currentData->datay, &currentData->dataz, &currentData->dataw);
			break;
	}
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