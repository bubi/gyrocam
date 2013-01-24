

#include "driver_config.h"
#include "target_config.h"

#include <math.h>
#include <string.h>

#include "gpio.h"
#include "i2c.h"
#include "uart.h"
#include "type.h"

#include "mpu6050.h"
#include "ars.h"


uint8_t gSysTick;

void SysTick_Handler(void){
	gSysTick++;
}

float get_Time_s(void){
	float ticks = gSysTick;
	gSysTick = 0;
	return ticks / 1000; /* return in seconds */
}

int main (void){

	struct Gyro1DKalman filter_roll;
	float acc_tmp;
	float dt;
	float roll_angle;
	uint8_t angle[10];
	uint8_t crlf[] = { 0x0D, 0x0A }; // \n\r

	/* Init Systick to 1ms */
	SysTick_Config(0xFFFF); /* TBD */
	/* Initialize GPIO (sets up clock) */
	GPIOInit();
	/* Set LED port pin to output */
	GPIOSetDir( LED_PORT, LED_BIT, 1 );

	UARTInit(UART_BAUD); // 115200;

	if(I2CInit(I2CMASTER) == FALSE){
	  while(1);	/* fatal error */
	}


	if(MPU6050_init()){
		/*error*/
	}

	if(MPU6050_whoami()){
		/*error*/
		return 1;
	}

	/*kalman form http://tom.pycke.be/ */
	init_Gyro1DKalman(&filter_roll, 0.0001, 0.0003, 0.69);

	while(1){

		dt = get_Time_s();
		ars_predict(&filter_roll, MPU6050_getGyroRoll_rad() , dt);  // Kalman predict
		acc_tmp = -(atan2(-MPU6050_getAccel_z(), MPU6050_getAccel_y())-(3.14159/2.0)); /* calculate accel angle */
	    roll_angle = ars_update(&filter_roll, acc_tmp);        // Kalman update + result (angle)
	    memcpy(angle, &roll_angle, sizeof(roll_angle));		   // convert to char array
	    UARTSend( (uint8_t *) angle, 10);					   // print
	    UARTSend( (uint8_t *) crlf, 2);						   // \n \r
	}
}
