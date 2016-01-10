#ifndef BNO055_h
#define BNO055_h

#include "Arduino.h"

class BNO055{
	public:
		BNO055();
		
		typedef enum {
			
			//Registers for accelerometer data
			BNO_ACCEL_RAW_X_LSB            		            = 0X08,
			BNO_ACCEL_RAW_X_MSB             		        = 0X09,
			BNO_ACCEL_RAW_Y_LSB               		        = 0X0A,
			BNO_ACCEL_RAW_Y_MSB                             = 0X0B,
			BNO_ACCEL_RAW_Z_LSB                             = 0X0C,
			BNO_ACCEL_RAW_Z_MSB                             = 0X0D,
			
			//Registers for gyroscope data
			BNO_MAG_RAW_X_LSB                               = 0x0E,
			BNO_MAG_RAW_X_MSB                               = 0x0F,
			BNO_MAG_RAW_Y_LSB                               = 0x10,
			BNO_MAG_RAW_Y_MSB                               = 0x11,
			BNO_MAG_RAW_Z_LSB                               = 0x12,
			BNO_MAG_RAW_Z_MSB                               = 0x13,
			
			//Registers for gyroscope data
			BNO_GYRO_RAW_X_LSB                              = 0x14,
			BNO_GYRO_RAW_X_MSB                              = 0x15,
			BNO_GYRO_RAW_Y_LSB                              = 0x16,
			BNO_GYRO_RAW_Y_MSB                              = 0x17,
			BNO_GYRO_RAW_Z_LSB                              = 0x18,
			BNO_GYRO_RAW_Z_MSB                              = 0x19,
			
			//Registers for RPY data
			BNO_ROLL_LSB                                    = 0x1C,
			BNO_ROLL_MSB                                    = 0x1D,
			BNO_PITCH_LSB                                   = 0x1E,
			BNO_PITCH_MSB                                   = 0x1F,
			BNO_YAW_LSB                                     = 0x1A,
			BNO_YAW_MSB                                     = 0x1B,
			
			BNO_PAGE_REG                                    = 0x07,
			
			BNO_UNIT_SET                                    = 0x3B,
			
			BNO_OPR_MODE                                    = 0x3D,
			
		} BNO_Register;
		
		void ReadRPY(float*, float*, float*);
		void ReadAccelRaw(float*, float*, float*);
		void ReadGyroRaw(float*, float*, float*);
		void ReadMagRaw(float*, float*, float*);
		void initialize();
		
		
	private:
		//Address of the BNO in i2c communications
		static const byte BNO_ADDRESS = 0x28;
		void setPage(int);
		void writeByte(BNO_Register, byte);
		byte readByte(BNO_Register);
		void setUnits(bool, bool, bool, bool, bool);
		struct{
			bool Windows;
			bool Celsius;
			bool Degrees;
			bool DPS;
			bool MPS;
		} settings;
};



#endif