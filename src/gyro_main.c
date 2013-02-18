

#include "driver_config.h"
#include "target_config.h"

#include "gpio.h"
#include "i2c.h"
#include "type.h"
#include "clkconfig.h"

#include "servo.h"
#include "mpu6050.h"
#include "kalman.h"

#include "math.h"

uint8_t gSysTick_10 = 0;


void SysTick_Handler(void){
	gSysTick_10++;
}


int main (void){


	float acc_x, acc_z, gyro_x;
	float acc_angle,kal_angle;

	/* Init Systick to 1ms */
	SysTick_Config( SystemCoreClock / 1000);

	/* Initialize GPIO (sets up clock) */
	GPIOInit();
	SERVO_init();


	if(I2CInit(I2CMASTER) == FALSE){
	  while(1);	/* fatal error */
	}

	if(MPU6050_whoami()){
		return 0;
	}

	if(MPU6050_init()){
		return 0;
	}

	MPU6050_setZero();

	kalman_init();

	while(1){

		/* 100Hz loop */
		if(gSysTick_10 >= 9){
			gSysTick_10 = 0;

			/* get sensor values */
			gyro_x 	= 	MPU6050_getGyroRoll_degree();
			acc_x 	= -(MPU6050_getAccel_x());
			acc_z 	= 	MPU6050_getAccel_z();

			/* acc angle */
			//acc_angle = atan2(acc_x, -acc_z) * 180/3.14159 ; // calculate accel angle


			kal_angle = kalman_update(90,gyro_x, 0.0093);

			SERVO_set_slew((-kal_angle) - MECH_OFFSET);
		}
	}
}
