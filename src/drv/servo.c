/*
 * servo.c
 *
 *  Created on: Feb 7, 2013
 *      Author: bumi
 */

#include "driver_config.h"
#include "target_config.h"

#include "servo.h"


float gLastAngle = 0;
void SERVO_init(){
	/* enable timer 1
	 * periode SystemCoreClock / 1000 -> 1ms * 20 -> 20ms -> 50Hz
	 * pin PIO1_2 AD3 MAT 1
	 */
	/* enable clk for timer 1 */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);
	/* reset on MR3 */
	LPC_TMR32B1->MCR = (1<<10);
	/* external match on EM1 */
	LPC_TMR32B1->EMR = (1<<6) | (1<<1);
	/* enable MAT1 for PIO1_2 */
	LPC_IOCON->R_PIO1_2 &= ~0x07;
	LPC_IOCON->R_PIO1_2 |= 0x03;
	/* enable PWM control for MAT1 and MAT3 */
	LPC_TMR32B1->PWMC = (1<<3) | (1<<1);
	/* set frequency (MAT3) */
	LPC_TMR32B1->MR3 = SERVO_PERIODE;
	/* 1.5ms for servo */
	LPC_TMR32B1->MR1 = SERVO_ZERO;
	/* enable timer */
	LPC_TMR32B1->TCR = 1;
}
void SERVO_set(float angle){
	uint32_t match;
	float tmp;

	tmp = (angle + SERVO_MAX_ANGLE/2) * (1400/SERVO_MAX_ANGLE);
	tmp = tmp * TIMER_1US;
	match = (uint32_t) tmp + SERVO_MAX_R;
	LPC_TMR32B1->MR1 =  match;
}

void SERVO_set_slew(float angle){
	uint32_t match;
	float tmp;

	angle = gLastAngle + (angle - gLastAngle) / SLEW_RATE;
	gLastAngle = angle;

	tmp = (angle + SERVO_MAX_ANGLE/2) * (1400/SERVO_MAX_ANGLE);
	tmp = tmp * TIMER_1US;
	match = (uint32_t) tmp + SERVO_MAX_R;
	LPC_TMR32B1->MR1 =  match;
}

