
#include "Arduino.h"

#include <Wire.h>

#include "BNO055.h"

BNO055::BNO055(){
}

void BNO055::initialize(){
	Wire.begin();
	setPage(0);
	setUnits(true, true, true, true, true);
	writeByte(BNO_OPR_MODE, 0b00001100);
	writeByte(BNO_INT_EN, 0x00);
}

void BNO055::setPage(int page){
	//If it is an invalid page do nothing
	if(page != 0 && page != 1){
		return;
	}
	
	writeByte(BNO_PAGE_REG, page);	
}

void BNO055::ReadRPY(float* roll, float* pitch, float* yaw){
	// Use quaternion data, because of a flaw in the way BNO055 calculates euler angles
	
	float w, x, y, z;
	
	ReadQuaternion(&w, &x, &y, &z);
	
	double q2sqr = y * y;
    double t0 = -2.0 * (q2sqr + z * z) + 1.0;
    double t1 = +2.0 * (x * y + w * z);
    double t2 = -2.0 * (x * z - w * y);
    double t3 = +2.0 * (y * z + w * x);
    double t4 = -2.0 * (x * x + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    *pitch = asin(t2);
    *roll = atan2(t3, t4);
    *yaw = atan2(t1, t0);
	
	
	*yaw = *yaw * 57.295780;
	*pitch = *pitch * 57.295780;
	*roll = *roll * 57.295780;
	
	// byte roll_LSB = readByte(BNO_ROLL_LSB);
	// byte roll_MSB = readByte(BNO_ROLL_MSB);
	
	// byte pitch_LSB = readByte(BNO_PITCH_LSB);
	// byte pitch_MSB = readByte(BNO_PITCH_MSB);
	
	// byte yaw_LSB = readByte(BNO_YAW_LSB);
	// byte yaw_MSB = readByte(BNO_YAW_MSB);
	
	// int16_t r = roll_LSB | (roll_MSB<<8);
	// int16_t p = pitch_LSB | (pitch_MSB<<8);
	// int16_t y = yaw_LSB | (yaw_MSB<<8);
	
	// if(settings.Degrees){
		// *roll = (float)r/16;
		// *pitch = (float)p/16;
		// *yaw = (float)y/16;
	// }else{
		// *roll = (float)r/900;
		// *pitch = (float)p/900;
		// *yaw = (float)y/900;
	// }
	
}

void BNO055::ReadQuaternion(float* dataw, float* datax, float* datay, float* dataz){
	
	uint8_t rawData[8];
	//Read all data at once for optimization purposes
	readMultiple(BNO_QUATW_LSB, (uint8_t*)rawData, 8);
	int16_t w = rawData[0] | (rawData[1] << 8);
	int16_t x = rawData[2] | (rawData[3] << 8);
	int16_t y = rawData[4] | (rawData[5] << 8);
	int16_t z = rawData[6] | (rawData[7] << 8);
	
	*dataw = (float)w / 16384.0;
	*datax = (float)x / 16384.0;
	*datay = (float)y / 16384.0;
	*dataz = (float)z / 16384.0;
}

void BNO055::ReadAccelRaw(float* accel_x, float* accel_y, float* accel_z){
	uint8_t rawData[6];
	//Read all data at once for optimization purposes
	readMultiple(BNO_ACCEL_RAW_X_LSB, (uint8_t*)rawData, 6);
	
	byte x_LSB = rawData[0];
	byte x_MSB = rawData[1];
	
	byte y_LSB = rawData[2];
	byte y_MSB = rawData[3];
	
	byte z_LSB = rawData[4];
	byte z_MSB = rawData[5];
	
	int16_t x = x_LSB | (x_MSB<<8);
	int16_t y = y_LSB | (y_MSB<<8);
	int16_t z = z_LSB | (z_MSB<<8);
	
	if(settings.MPS){
		*accel_x = (float)x/100;
		*accel_y = (float)y/100;
		*accel_z = (float)z/100;
	}else{
		*accel_x = (float)x;
		*accel_y = (float)y;
		*accel_z = (float)z;
	}
}

void BNO055::ReadGyroRaw(float* gyro_x, float* gyro_y, float* gyro_z){
	uint8_t rawData[6];
	//Read all data at once for optimization purposes
	readMultiple(BNO_GYRO_RAW_X_LSB, (uint8_t*)rawData, 6);
	
	byte x_LSB = rawData[0];
	byte x_MSB = rawData[1];
	
	byte y_LSB = rawData[2];
	byte y_MSB = rawData[3];
	
	byte z_LSB = rawData[4];
	byte z_MSB = rawData[5];
	
	int16_t x = x_LSB | (x_MSB<<8);
	int16_t y = y_LSB | (y_MSB<<8);
	int16_t z = z_LSB | (z_MSB<<8);
	
	if(settings.MPS){
		*gyro_x = (float)x/16;
		*gyro_y = (float)y/16;
		*gyro_z = (float)z/16;
	}else{
		*gyro_x = (float)x/900;
		*gyro_y = (float)y/900;
		*gyro_z = (float)z/900;
	}
}

void BNO055::ReadMagRaw(float* mag_x, float* mag_y, float* mag_z){
	uint8_t rawData[6];
	//Read all data at once for optimization purposes
	readMultiple(BNO_MAG_RAW_X_LSB, (uint8_t*)rawData, 6);
	
	byte x_LSB = rawData[0];
	byte x_MSB = rawData[1];
	
	byte y_LSB = rawData[2];
	byte y_MSB = rawData[3];
	
	byte z_LSB = rawData[4];
	byte z_MSB = rawData[5];
	
	int16_t x = x_LSB | (x_MSB<<8);
	int16_t y = y_LSB | (y_MSB<<8);
	int16_t z = z_LSB | (z_MSB<<8);
	
	*mag_x = (float)x/16;
	*mag_y = (float)y/16;
	*mag_z = (float)z/16;
}

void BNO055::writeByte(BNO_Register reg, byte value){
	Wire.beginTransmission(BNO_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.write((uint8_t) value);
	Wire.endTransmission();
}

byte BNO055::readByte(BNO_Register reg){
	Wire.beginTransmission(BNO_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BNO_ADDRESS, (uint8_t)1);
	return Wire.read();
}

void BNO055::readMultiple(BNO_Register reg, byte* buffer, int numBytes){
	Wire.beginTransmission(BNO_ADDRESS);
	Wire.write((uint8_t) reg);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BNO_ADDRESS, numBytes);
	
	for(int i=0; i<numBytes; i++){
		buffer[i] = Wire.read();
	}
}

void BNO055::setUnits(bool Windows, bool Celsius, bool Degrees, bool DPS, bool MPS){
	//Set the appropriate units and record in settings
	settings.Windows = Windows;
	settings.Celsius = Celsius;
	settings.Degrees = Degrees;
	settings.DPS = DPS;
	settings.MPS = MPS;
	
	byte values = 0b00000000;
	if(!Windows){
		values &= 0b10000000;
	}
	if(!Celsius){
		values &= 0b00010000;
	}
	if(!Degrees){
		values &= 0b00000100;
	}
	if(!DPS){
		values &= 0b00000010;
	}
	if(!MPS){
		values &= 0b00000001;
	}
	
	writeByte(BNO_UNIT_SET, values);	
}