/*
 * arena.c
 *
 *  Created on: 4 Apr 2019
 *      Author: astridhochart
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <arena.h>
#include "pid.h"
#include "leds.h"
#include <chprintf.h>
#include "motors_advanced.h"
#include "audio/microphone.h"

#include <audio_processing.h>
#include <fft.h>
#include "communications.h"
#include <arm_math.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"

#define ANGLE_MAX 360
#define NUMBER_OF_MEASURE 20
#define ANGLE_RESOLUTION ANGLE_MAX / NUMBER_OF_MEASURE
#define PI                  3.1415926536f
#define RAD PI/180
#define WHEEL_DISTANCE      5.35f    //cm TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

#define HIGH_SPEED	10
#define STD_SPEED	8
#define LOW_SPEED	5
#define VERY_LOW_SPEED	3
#define RAYON_EPUCK 365 //MM
#define RAYON_ARENA 2570 //MM
#define ERROR_RESOLUTION 50 //MM

#define FREQUENCE1 10000 //Hz
#define FREQUENCE2 10200 //Hz
#define FREQUENCE3 10400 //Hz

#define DIST1 10
#define DIST2 20
#define DIST3 30

BUFFER_NAME_t name = 0;
static  uint16_t pos_waste; //on peut peut être enlever
static  uint16_t frequence;

void gohome(void);
void precise_alignment_wall(void);

void init_arena(void)
{
	//inits the sensor
	VL53L0X_start();
	motors_advanced_init();
	chThdSleepMilliseconds(200);
	proximity_start();
	pid_pause(1);
	pid_regulator_start();
}

void gotoarenacenter(void)
{
	 gotoedge();
	 motors_advanced_turnright(135, HIGH_SPEED);
	 goforward(PID_PAUSE, 30, HIGH_SPEED);

}
uint8_t wasteinsight(int16_t angle){
	uint16_t dist = VL53L0X_get_dist_mm(); //première mesure = 0 ?
	chThdSleepMilliseconds(110);
	if ( abs(dist - function_distance_arena(angle*ANGLE_RESOLUTION*RAD)) < ERROR_RESOLUTION)
	{
			return 1;
	}

	return 0;
}
void searchwaste(void){

	static uint8_t state = 0;
	int16_t angle = 0;
	while(1)
	{
	switch(state){
	        	case 0:
	        				while(angle < NUMBER_OF_MEASURE)
						{
							uint16_t test = VL53L0X_get_dist_mm();
							chThdSleepMilliseconds(110);
							if (wasteinsight(angle) == 1)
							{
								set_body_led(1);
								pos_waste = angle;
								chThdSleepMilliseconds(110);
								state = 1;
								break;
							}
							else
							{
								motors_advanced_turnright(ANGLE_RESOLUTION, STD_SPEED);
								chThdSleepMilliseconds(110);
								angle += 1;
								if(angle ==  NUMBER_OF_MEASURE - 1)
									angle = 0;
							}
						}
	        				break; // c'est possible qu'il reste coincé dans cette boucle
	        	case 1: set_body_led(0);
	        			pickupwaste();
	        			state = 2;
	        			break;

	        	case 2:
	        			gohome();
	        			state = 3;
	        			break;

	        	case 3:
	        			goback(frequence);
	        			motors_advanced_turnleft(90, STD_SPEED);
	        			throwwaste();
	        			state = 4;
	        			break;

	        	case 4: gotoarenacenter();
	        			state = 0;
	        			break;
	        }
	}
}


int16_t findwall(void){
	uint8_t max_norm_index = -1;
	uint16_t max_norm = 1000;
	uint16_t tmp = VL53L0X_get_dist_mm();
	//measure all the distances from 0° to 360°
	for(uint16_t i = 0; i < NUMBER_OF_MEASURE; i++)
	{
		set_body_led(0);
		chThdSleepMilliseconds(110);
		tmp = VL53L0X_get_dist_mm();
		if(tmp > 1000)
		{
			motors_advanced_turnleft(ANGLE_RESOLUTION, HIGH_SPEED);
			continue;
		}
		if(tmp < max_norm){
			chThdSleepMilliseconds(110);
			tmp = VL53L0X_get_dist_mm();
			if(tmp < max_norm){
				set_body_led(1);
				max_norm = tmp;
				max_norm_index = i;
				chThdSleepMilliseconds(100);
				set_body_led(0);
			}
		}
		motors_advanced_turnleft(ANGLE_RESOLUTION, HIGH_SPEED);
	}
	if(max_norm_index == -1)
		return 0;

	return max_norm_index*ANGLE_RESOLUTION -1;

}


void gotowall(void){
	int16_t angle_min = 0;
	angle_min = findwall();
	set_front_led(1);
	aligntothewall(angle_min);
	set_body_led(0);
	chThdSleepMilliseconds(150);
	goforward(PID_PAUSE, 0, HIGH_SPEED);
	precise_alignment_wall();
}

void precise_alignment_wall(void)
{
	set_body_led(1);
	int left = 0; int right = 0;
	motors_advanced_set_speed(1,1);
	while(left < 1000 || right < 1000)
	{
		left = get_prox(7);
		right = get_prox(0);
		chThdSleepMilliseconds(5);
	}
	motors_advanced_stop();
	set_body_led(0);
	chThdSleepMilliseconds(1000);
	set_body_led(1);

	if(left>right)//turnleft
		motors_advanced_set_speed(0.5,-0.5);
	else
		motors_advanced_set_speed(-0.5,0.5);

	while(abs(left-right) > 40)
	{
		left = get_prox(7);
		right = get_prox(0);
		chThdSleepMilliseconds(1);
	}
	motors_advanced_stop();
	set_body_led(1);
	chThdSleepMilliseconds(1000);
	set_body_led(0);
}

void aligntothewall(int16_t angle_min)
{
	if(angle_min <= 180)
		motors_advanced_turnleft(angle_min, STD_SPEED);
	else
		motors_advanced_turnright(360-angle_min, STD_SPEED);
}

void goforward(uint8_t pid_or_not, float distance, uint8_t speed)
{
	if(pid_or_not == PID_PAUSE)
	{
		pid_pause(PID_PAUSE);
		if(distance == 0)
		{
			motors_advanced_set_speed(5, 5);
			while(VL53L0X_get_dist_mm() > TOO_CLOSE_OF_THE_WALL)
				chThdSleepMilliseconds(110);
			motors_advanced_stop();
			set_front_led(0);
		}
		else
		{
			motors_advanced_set_position(distance, distance, speed, speed);
		}
	}
	else
	{
		pid_pause(PID_PLAY);
	}
}

void gotoedge(void)
{
	gotowall();
	walltoright();
	goforward(PID_PLAY, 0, LOW_SPEED);
	while(1)
	{
		if(get_prox(0) > 1000 || get_prox(7) > 1000)
		{
			motors_advanced_stop();
			pid_pause(PID_PAUSE);
			precise_alignment_wall();
			motors_advanced_turnright(90, LOW_SPEED);
			precise_alignment_wall();
			break;
		}
		chThdSleepMilliseconds(10);
	}
}

void walltoright(void){

	motors_advanced_turnleft(90, HIGH_SPEED);

}

void pickupwaste(void){
	goforward(PID_PAUSE, 0, LOW_SPEED);
	goback(5);
	shoveldown();
	goforward(PID_PAUSE, 1, LOW_SPEED);
	shovelup();
}
void goback(uint16_t frequence){

	if(frequence == FREQUENCE1)
	{
		goforward(PID_PAUSE, DIST1, -LOW_SPEED);
	}
	if(frequence == FREQUENCE2)
	{
		goforward(PID_PAUSE, DIST2, -LOW_SPEED);
	}
	if(frequence == FREQUENCE3)
	{
		goforward(PID_PAUSE, DIST3, -LOW_SPEED);
	}

}
void shoveldown(void){


}
void shovelup(void){


}
void gohome(void){
	gotoedge();
	while(1)
	{
	if(get_frequence() == FREQUENCE1 || FREQUENCE2 || FREQUENCE3)
	{
		if(get_frequence() == FREQUENCE1)
			frequence = FREQUENCE1;
		if(get_frequence() == FREQUENCE2)
			frequence = FREQUENCE2;
		if(get_frequence() == FREQUENCE3)
			frequence = FREQUENCE3;
	}
	else go_to_another_edge();
	}
}
void throwwaste(void){


}
uint16_t function_distance_arena(uint16_t angle_robot){
	uint16_t dist_robot = 0;
	dist_robot = RAYON_ARENA/cos(angle_robot) - RAYON_EPUCK ;
	return dist_robot;

}
void go_to_another_edge(void){
	goforward(PID_PLAY, 0, STD_SPEED);
		while(1)
		{
			if(get_prox(0) > 1000 || get_prox(7) > 1000)
			{
				motors_advanced_stop();
				pid_pause(PID_PAUSE);
				precise_alignment_wall();
				motors_advanced_turnright(90, LOW_SPEED);
				precise_alignment_wall();
				break;
			}
			chThdSleepMilliseconds(10);
		}
}
