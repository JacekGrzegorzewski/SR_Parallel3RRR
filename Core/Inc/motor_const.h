/*
 * motor_const.h
 *
 *  Created on: Apr 9, 2022
 *      Author: jacek
 */

#ifndef MOTOR_CONST_H_
#define MOTOR_CONST_H_

//timer frequency in Hz
#define MIN_FREQ 1600

//steps per resolution ant 1/16 step driver mode
#define SPR 6400

#define ALPHA (2*3.14159/SPR)
#define CLK_FRQ 84000000

#define MAX_VEL CLK_FRQ*ALPHA/30
#define MIN_VEL CLK_FRQ*ALPHA/525

#define STOP 0
#define ACCEL 1
#define DECEL 2
#define RUN 3

typedef struct motorInfo
{
//unsigned min_interval ; // == 1/max_speed
	unsigned long max_speed;
	unsigned long  max_speed_ARR;
	char dir;
	unsigned long auto_reload;// length of current pulse in timer ticks

	unsigned long acc_lim;
	unsigned long peak_velocity;
	unsigned long decel_start;

	unsigned long acceleration;
	unsigned long deceleration;

	unsigned long starting_frequency;

	unsigned long steps;
	unsigned long total_steps;
	volatile long step_position;

}motorInfo;



#endif /* MOTOR_CONST_H_ */