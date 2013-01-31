

#include "driver_config.h"
#include "target_config.h"

#include <math.h>
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "uart.h"
#include "type.h"
#include "clkconfig.h"

#include "mpu6050.h"
#include "kalman.h"
#include "ars.h"


uint8_t gSysTick = 0;
uint8_t gSysTick_100 = 0;
uint8_t led_toggle = 0;

void SysTick_Handler(void){
	gSysTick++;
	gSysTick_100++;
}

float get_Time_s(void){
	float ticks = gSysTick;
	gSysTick = 0;
	return ticks / 1000; /* return in seconds */
}

int main (void){

	struct Gyro1DKalman filter_roll;
	float dt;

	float acc_x_buff, acc_z_buff,acc_x, acc_z, gyro_x, drift;
	float acc_angle, gyro_angle, kal_angle, drift_buf, true_angle;

	uint8_t string[80];

	uint8_t i, drift_cnt;

	for(i=0;i<80;i++)string[i]=0;
	/* Init Systick to 1ms */
	SysTick_Config( SystemCoreClock / 1000);

	/* Initialize GPIO (sets up clock) */
	GPIOInit();
	/* Set LED port pin to output */
	GPIOSetDir( LED_PORT, LED_BIT, 1 );


	/*
	LPC_IOCON->PIO0_1 &= ~0x07;
	LPC_IOCON->PIO0_1 |= 0x01;
	CLKOUT_Setup(CLKOUTCLK_SRC_MAIN_CLK);
	*/

	UARTInit(UART_BAUD); // 115200;

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

	/*kalman form http://tom.pycke.be/ */
	//init_Gyro1DKalman(&filter_roll,0.0001, 0.003,0.1);
	kalman_init();
	drift_cnt = 0;
	while(1){
		dt = get_Time_s();

		gyro_x = MPU6050_getGyroRoll_degree();
		//ars_predict(&filter_roll, gyro_x, dt);  // Kalman predict
		acc_x = MPU6050_getAccel_x();
		acc_z = MPU6050_getAccel_z();
		acc_angle = atan2(acc_x, -acc_z) * 180/3.14159 ; // calculate accel angle
	   // kal_angle = ars_update(&filter_roll, acc_angle);        // Kalman update + result (angle)
		kal_angle = kalman_update(acc_angle,gyro_x, dt);
		drift_buf += (kal_angle-gyro_angle);
		drift_cnt++;
		if(drift_cnt == 20){
			drift = drift_buf / 20;
			drift_buf = 0;
			drift_cnt = 0;
		}
		gyro_angle += (gyro_x) * dt;
		true_angle = gyro_angle + drift * 0.7;
	   // sprintf(string,"$1;1;;%f;%f;%f;\r\n",acc_angle, gyro_angle, kal_angle);
		sprintf(string,"%f;  %f;  %f;  %f; %f; \n\r",acc_angle,gyro_angle,kal_angle,true_angle, drift);
		//itoa(acc_y,char_acc_y,10);
	    UARTSend ((uint8_t *) string,80);
	    gSysTick_100 = 0;

	}
}
