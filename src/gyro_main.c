

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



uint8_t gSysTick = 0;
uint8_t gSysTick_20 = 0;
uint16_t gSysTick_1000 = 0;
uint8_t led_toggle = 0;

void SysTick_Handler(void){
	gSysTick++;
	gSysTick_20++;
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
	float acc_angle, gyro_angle,gyro_angle_last, kal_angle, kal_angle_last, drift_buf, drift_last, true_angle;
	float true_angle_round, true_angle_quater, integral_tmp, tmp;
	float servo_angle = 0;

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
	drift_cnt = 0;
	while(1){
		GPIOSetValue( LED_ON );
		/* 50Hz loop */
		if(gSysTick_20 >= 19){
			gSysTick_20 = 0;
			GPIOSetValue( LED_OFF );

			/* get sensor values */

			gyro_x 	= MPU6050_getGyroRoll_degree();
			acc_x 	= MPU6050_getAccel_x();
			acc_z 	= MPU6050_getAccel_z();

			/* acc angle */
			acc_angle = atan2(acc_x, -acc_z) * 180/3.14159 ; // calculate accel angle
			/* kalman angle */
			kal_angle_last = kal_angle;
			kal_angle = kalman_update(acc_angle,gyro_x, 0.02);
			/* gyro angle*/

			gyro_angle += (gyro_x) * 0.02;

			/* drift compensation */
			/* lowpass for kalman output */
			drift_buf += (gyro_angle - kal_angle);
			drift_cnt++;
			if(drift_cnt == 10){
				drift = drift_buf / 10;
				drift_buf = 0;
				drift_cnt = 0;
			}

			/* true angle calculated from gyro + drift calculateD from filtered kalman output */
			if((fabsf(gyro_angle_last - gyro_angle) > 1)){
				/* angle change seems to be greater then 1° so use moste actual data */
				true_angle = gyro_angle - drift;
				drift_last = drift;
				gyro_angle_last = gyro_angle;
			}else{
				/* angle change seems small, so trust gyro and hold angle in place */
				/* check if kalman is stable */
				if(fabsf(kal_angle_last - kal_angle) < 1){
					/* trust him and update drift modifier*/
					drift_last = drift;
				}
				/* use delta - filtered drift */
				true_angle = gyro_angle - drift_last;
			}
			/* cut of to XX.xx
			 * zB: 10.0345785°
			 * *100 = 1003.45785
			 * ceil() = 1004
			 * :100 = 10.04
			 */
			true_angle_round = ceilf((true_angle * 100))/100;
			/* 10.04 ->
			 * tmp = 0.04
			 */
			tmp = modf(true_angle_round, &integral_tmp);
			/* calculate quater */
			/* 10.0 (10.25, 10.50, 10.75)
			 * 10.04 = uint32t 10*/
			tmp = (int32_t) (tmp/0.25);
			true_angle_quater =((int32_t) true_angle_round) + 0.25 * tmp;

			if(fabsf(servo_angle - true_angle_quater) >= dMIN_ANGLE){
				/* set servo to new angle */
				servo_angle = true_angle_quater;
				/* implement function here */
			}



#ifdef DEBUG_OUTPUT
			/* 10 Hz loop */
			if(gSysTick_1000 >= (DEBUG_TIME_MS - 1)){
				sprintf(string,"%f;%f;%f;	Servo:%f; True:%f; \n\r",acc_angle,kal_angle,gyro_angle,		 servo_angle, true_angle);
				UARTSend ((uint8_t *) string,80);
				gSysTick_1000 = 0;
			}
#endif
		}
	}
}
