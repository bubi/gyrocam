/*
 * servo.h
 *
 *  Created on: Feb 7, 2013
 *      Author: bumi
 */

#ifndef SERVO_H_
#define SERVO_H_

#define TIMER_1MS 			(SystemCoreClock / 1000)
#define TIMER_1US	 		(SystemCoreClock / 1000000)

void SERVO_init(void);
void SERVO_set(float angle);
void SERVO_set_slew(float angle);


#endif /* SERVO_H_ */
