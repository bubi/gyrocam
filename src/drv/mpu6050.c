/*
 * mpu6050.c
 *
 *  Created on: Jan 24, 2013
 *      Author: bumi
 */

#include "mpu6050.h"
#include "target_config.h"
#include "driver_config.h"

#include "i2c.h"

extern volatile uint8_t 	I2CMasterBuffer[BUFSIZE], I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t 	I2CReadLength, I2CWriteLength;


uint8_t MPU6050_init() {

	uint8_t state;

	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_PWR_MGMT_1;
	I2CMasterBuffer[2] 	= 0b10000000; // reset device;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_PWR_MGMT_1;
	I2CMasterBuffer[2] 	= 0b00000011; // wakeup and set clock to z-axis gyro;

	state = I2CEngine();
	if(state != I2C_OK) return 1;


	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_CONFIG;
	I2CMasterBuffer[2] 	= 0b00000000; // gyro range defaults to 250°/s but for future use

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	I2CWriteLength 		= 3;
	I2CReadLength		= 0;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_CONFIG;
	I2CMasterBuffer[2] 	= 0b00000001; // 4G range

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	return 0;
}

uint8_t MPU6050_whoami(){

	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 1;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_WHO_AM_I;
	I2CMasterBuffer[2] 	= MPU6050_ADRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	if(I2CSlaveBuffer[0] != 0x34) return 1;

	return 0;
}

uint8_t MPU6050_getValue(sMPU_Value* sValue){

	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 14;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_XOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	sValue = I2CSlaveBuffer;

	return 0;
}

float MPU6050_getGyroRoll_rad(){

	uint16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_GYRO_XOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	tmp = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1] ;

	return tmp * 123; /* TBD */
}

float MPU6050_getAccel_y(){

	uint16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_YOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	tmp = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1] ;

	return tmp * 123; /* TBD */
}

float MPU6050_getAccel_z(){

	uint16_t tmp;
	uint8_t state;

	I2CWriteLength 		= 2;
	I2CReadLength		= 2;
	I2CMasterBuffer[0] 	= MPU6050_ADRESS;
	I2CMasterBuffer[1] 	= MPU6050_RA_ACCEL_ZOUT_H;
	I2CMasterBuffer[2] 	= MPU6050_ADRESS | RD_BIT;

	state = I2CEngine();
	if(state != I2C_OK) return 1;

	tmp = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1] ;

	return tmp * 123; /* TBD */
}
