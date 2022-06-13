/*
 * motor_const.h
 *
 *  Created on: Apr 9, 2022
 *      Author: jacek
 */

#ifndef MOTOR_CONST_H_
#define MOTOR_CONST_H_

//minimum timer frequency in Hz
#define MIN_FREQ 1600
#define MAX_FREQ 28000
//steps per resolution at 1/16 step driver mode
#define SPR 6400

#define ALPHA (2*3.14159/SPR)
#define CLK_FRQ 84000000

#define MAX_VEL (CLK_FRQ*ALPHA/3)
#define MIN_VEL (CLK_FRQ*ALPHA*10/525)

#define STOP 0
#define ACCEL 1
#define DECEL 2
#define RUN 3

typedef struct motorInfo
{
//unsigned min_interval ; // == 1/max_speed
	unsigned long max_speed;
	unsigned long  max_speed_ARR;
	int dir;
	volatile unsigned long auto_reload;// length of current pulse in timer ticks

	TIM_HandleTypeDef *timer;
	GPIO_TypeDef *GPIOX;
	uint16_t GPIO_Label;
	unsigned long peak_velocity;
	unsigned long accel_stop;
	unsigned long decel_start;
	unsigned long acceleration;
	unsigned long deceleration;
	volatile unsigned long rest;
	volatile char state;
	volatile unsigned long steps;
	unsigned long total_steps;
	volatile long step_position;



	unsigned long T1,T2,T3;
	unsigned long movement_start;
	double acc_time;
	double dec_time;

	double time_to_complete;
	double time_before_correction;

	volatile char movement_done;
	volatile char display_flag;

}motorInfo;



#endif /* MOTOR_CONST_H_ */
