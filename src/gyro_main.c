

#include "driver_config.h"
#include "target_config.h"

#include <math.h>
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "uart.h"
#include "type.h"
#include "clkconfig.h"

#include "servo.h"
#include "mpu6050.h"
#include "kalman.h"



uint8_t gSysTick = 0;
uint8_t gSysTick_10 = 0;
uint16_t gSysTick_1000 = 0;
uint8_t led_toggle = 0;

float dump[4][100];

void SysTick_Handler(void){
	gSysTick++;
	gSysTick_10++;
	gSysTick_1000++;
}

float get_Time_s(void){
	float ticks = gSysTick;
	gSysTick = 0;
	return ticks / 1000; /* return in seconds */
}

int main (void){


	float acc_x, acc_z, gyro_x;
	float drift = 0;
	float acc_angle, gyro_angle,gyro_angle_last, kal_angle,kal_angle_filtered, kal_angle_buf, kal_angle_last, drift_filtered, true_angle;
	float true_angle_round, true_angle_quater, integral_tmp, tmp;
	float servo_angle = 0;

	uint8_t string[80];

	uint8_t i,n, lowpass_cnt;

	for(i=0;i<80;i++)string[i]=0;
	/* Init Systick to 1ms */
	SysTick_Config( SystemCoreClock / 1000);

	/* Initialize GPIO (sets up clock) */
	GPIOInit();
	/* Set LED port pin to output */
	GPIOSetDir( LED_PORT, LED_BIT, 1 );
	SERVO_init();

	/*
	LPC_IOCON->PIO0_1 &= ~0x07;
	LPC_IOCON->PIO0_1 |= 0x01;
	CLKOUT_Setup(CLKOUTCLK_SRC_MAIN_CLK);
	*/

	UARTInit(UART_BAUD); // 19200;

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


	GPIOSetValue( LED_ON );

	kalman_init();
	lowpass_cnt = 0;
	while(1){
		GPIOSetValue( LED_ON );
		/* 100Hz loop */
		if(gSysTick_10 >= 9){
			gSysTick_10 = 0;
			GPIOSetValue( LED_OFF );

			/* get sensor values */
			gyro_x 	= MPU6050_getGyroRoll_degree();
			acc_x 	= -(MPU6050_getAccel_x());
			acc_z 	= MPU6050_getAccel_z();

			/* acc angle */
			acc_angle = atan2(acc_x, -acc_z) * 180/3.14159 ; // calculate accel angle
			/* kalman angle */
			kal_angle_last = kal_angle;
			kal_angle = kalman_update(acc_angle,gyro_x, 0.0096);
			/* gyro angle*/
			gyro_angle_last = gyro_angle;
			gyro_angle += (gyro_x) * 0.0096;

			/* drift compensation */
			/* lowpass for kalman output */
			kal_angle_buf += kal_angle;
			lowpass_cnt++;
			if(lowpass_cnt == 20){
				kal_angle_last = kal_angle_filtered;
				kal_angle_filtered = kal_angle_buf / 20;
				kal_angle_buf = 0;
				lowpass_cnt = 0;
			}

			drift = gyro_angle - kal_angle_filtered;

			if((fabsf(gyro_angle_last - gyro_angle) < 2)){
				/* check if kalman is stable enough*/
				if(fabsf(kal_angle_last - kal_angle_filtered) < 0.5){
					/* trust him and update drift modifier*/
					drift_filtered = drift;
					/* if its realy stable, we correct the acutal gyro pos */
					if(fabsf(kal_angle_last - kal_angle_filtered) < 0.1){
						gyro_angle = kal_angle_filtered;
					}
				}
				/* use delta - filtered drift */
				true_angle = gyro_angle - drift_filtered;
			}

			if(fabsf(servo_angle - true_angle) >= MIN_ANGLE){
				/* set servo to new angle */
					servo_angle = -(true_angle);
				}
				//servo_angle = -true_angle_quater;
				/* implement function here (4.7 mechanical offset)*/
			SERVO_set_slew(servo_angle - MECH_OFFSET);



#ifdef DEBUG_OUTPUT
			/* 10 Hz loop */
			if(gSysTick_1000 >= (DEBUG_TIME_MS - 1)){
#ifndef DUMP
				sprintf(string,"%f;%f;%f;	Servo:%f; True:%f; \n\r",acc_x,acc_z,gyro_x,		 servo_angle, true_angle_quater);
				UARTSend ((uint8_t *) string,80);
				gSysTick_1000 = 0;
#endif
#ifdef DUMP
				if(n<100){
					dump[0][n] = acc_angle;
					dump[1][n] = gyro_angle;
					dump[2][n] = true_angle;
					dump[3][n] = servo_angle;
					n++;
				}else{
					for(n = 0;n<100;n++){
						sprintf(string, "%f;%f;%f;%f \n\r",dump[0][n],dump[1][n],dump[2][n],dump[3][n]);
						UARTSend ((uint8_t *) string,80);
					}
					n = 0;
				}
#endif
				gSysTick_1000 = 0;
			}
#endif
		}
	}
}
