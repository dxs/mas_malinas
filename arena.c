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

#include "audio_processing.h"
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
static  uint16_t frequence;
static float look_up_table[19] =
			 {395.354887218045,
			  367.559398496240,
			  342.850891935721,
			  321.229367536488,
			  302.69482529854,
			  287.247265221878,
			  274.886687306502,
			  265.613091552410,
			  259.426477959605,
			  256.326846528085,
			  256.314197257851,
			  259.388530148902,
			  265.549845201238,
			  274.798142414861,
			  287.133421789768,
			  302.555683325962,
			  321.064927023441,
			  342.661152882205,
			  367.344360902255};

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
	 goforward(PID_PAUSE, 30, HIGH_SPEED,TOO_CLOSE_OF_THE_WALL);

}
uint16_t wasteinsight(int16_t angle){
	uint16_t dist = VL53L0X_get_dist_mm();
	uint16_t dist2 = 0;
	while(1)
	{
		chThdSleepMilliseconds(110);
		dist2 = VL53L0X_get_dist_mm();

		if(abs(dist-dist2)<ERROR_RESOLUTION)
			break;
		dist = dist2;
	}
	// chThdSleepMilliseconds(110);
	// if ( abs(dist - function_distance_arena(angle*ANGLE_RESOLUTION*RAD)) < ERROR_RESOLUTION)
	// {
	// 	dist = VL53L0X_get_dist_mm();
	// 	return 1;
	// }
	return dist;
}
void searchwaste(void){

	static uint8_t state = 0;
	static int angle =0;
	static uint16_t angle_waste;
	int16_t p[19] = {0};
	p[0] = 1;
	while(1)
	{
		switch(state){
			case 0:
			 while(state==0){
				while(1)
				{
					chThdSleepMilliseconds(p[0]);
					chThdSleepMilliseconds(120);
					p[angle] = wasteinsight(angle);

					motors_advanced_turnright(5, LOW_SPEED);
					angle++;

					if(angle ==  18)
					{
						angle = 0;
						break;
					}
				}
				for(int i=0; i<18; i++)
				{
					if(abs(p[i]-look_up_table[i]) > ERROR_RESOLUTION && abs(p[i+1]-look_up_table[i+1]) > ERROR_RESOLUTION
							&& abs(p[i+2]-look_up_table[i+2]) > ERROR_RESOLUTION)
					{
						set_front_led(1);
						angle_waste = i+2;
						state=1;
						break;
					}
				}
				chThdSleepMilliseconds(120);
				if(state == 0)
				{
				motors_advanced_turnright(5, LOW_SPEED);
				//offset de 5 degrés pour compenser le fait que le robot tourne pas forcément de 5 degrés à chaque fois
				}
			 }

			case 1:
				pickupwaste(angle_waste);
				state = 2;
				break;

			case 2:
				gohome();
				state = 3;
				break;

			case 3:
				goback(frequence,LOW_SPEED);
				motors_advanced_turnleft(90, STD_SPEED);
				throwwaste();
				state = 4;
				break;

			case 4:
				gotoarenacenter();
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
	goforward(PID_PAUSE, 0, HIGH_SPEED,TOO_CLOSE_OF_THE_WALL);
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

void goforward(uint8_t pid_or_not, float distance, uint8_t speed, uint8_t stop_dist)
{
	if(pid_or_not == PID_PAUSE)
	{
		pid_pause(PID_PAUSE);
		if(distance == 0)
		{
			motors_advanced_set_speed(speed, speed);
			while(VL53L0X_get_dist_mm() > stop_dist)
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
	goforward(PID_PLAY, 0, LOW_SPEED,TOO_CLOSE_OF_THE_WALL);
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

void pickupwaste(uint16_t _angle){
	motors_advanced_turnleft(90 - _angle*5, LOW_SPEED);
	chThdSleepMilliseconds(2000);
	goforward(PID_PAUSE, 0, LOW_SPEED,100);
	//goforward(PID_PAUSE, -10, LOW_SPEED,TOO_CLOSE_OF_THE_WASTE);
	//shoveldown();
	//goforward(PID_PAUSE, 1, LOW_SPEED);
	//shovelup();
}
void goback(uint16_t frequence, uint8_t speed){

	if(frequence == FREQUENCE1)
	{
		goforward(PID_PAUSE, DIST1, -speed,TOO_CLOSE_OF_THE_WALL);
	}
	if(frequence == FREQUENCE2)
	{
		goforward(PID_PAUSE, DIST2, -speed,TOO_CLOSE_OF_THE_WALL);
	}
	if(frequence == FREQUENCE3)
	{
		goforward(PID_PAUSE, DIST3, -speed,TOO_CLOSE_OF_THE_WALL);
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
	goforward(PID_PLAY, 0, STD_SPEED,TOO_CLOSE_OF_THE_WALL);
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
