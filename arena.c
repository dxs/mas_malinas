/*
 * arena.c
 *
 *  Created on: 4 Apr 2019
 *      Author: Astrid Hochart & Sven Borden
 *      Version 0.8
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <arena.h>
#include <arm_math.h>
#include "audio_processing.h"
#include "ch.h"
#include "communications.h"
#include "fft.h"
#include "hal.h"
#include "leds.h"
#include "memory_protection.h"
#include "motors_advanced.h"
#include "pid.h"
#include "spi_comm.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"
#include <usbcfg.h>
#include <chprintf.h>

#define ANGLE_MAX 360
#define NUMBER_OF_MEASURE 20
#define ANGLE_RESOLUTION ANGLE_MAX / NUMBER_OF_MEASURE
#define PI                  3.1415926536f
#define RAD PI/180
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

#define VERY_HIGH_SPEED 13
#define HIGH_SPEED		10
#define STD_SPEED		8
#define LOW_SPEED		5
#define VERY_LOW_SPEED	3
#define ERROR_RESOLUTION 80 //MM

#define FREQ_DECHET_1 19	//296Hz
#define FREQ_DECHET_2 20 //312Hz
#define FREQ_DECHET_3 21 //327Hz
#define FREQUENCE_RESOLUTION 2 //Hz

#define DIST_DECHET_1 5
#define DIST_DECHET_2 10
#define DIST_DECHET_3 15

#define STATE_SEARCH 	0
#define STATE_PICKUP 	1
#define STATE_HOME		2
#define STATE_RECYCLE	3
#define STATE_RESET		4


static  uint8_t frequence = 0;

static float look_up_table[19] =
			 {395.354887218045,
			  367.559398496240,
			  342.850891935721,
			  321.229367536488,
			  302.694825298540,
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

void goforward(uint8_t pid, float distance, int8_t speed, uint8_t stop_dist);
void gotoedge(void);
void gotowall(void);
int16_t findwall(void);
void align_to_wall(int16_t angle_min);
void precise_alignment_wall(void);
uint16_t get_distance(void);
void wall_to_right(void);
uint16_t waste_in_sight(void);
void pickup_waste(uint16_t angle);
void goback(uint16_t frequence, int8_t speed);
void shoveldown(void);
void shovelup(void);
void throwwaste(void);
void gohome(void);
void go_to_another_edge(void);


/// <summary>
/// Initialize sensors
/// </summary>
void init_arena(void)
{
	set_rgb_led(0, 255, 255, 0);
	set_rgb_led(1, 255, 0, 255);
	set_rgb_led(2, 255, 0, 255);
	set_rgb_led(3, 255, 255, 0);

	VL53L0X_start();
	motors_advanced_init();
	chThdSleepMilliseconds(200);
	proximity_start();
	pid_pause(1);
	pid_regulator_start();
	set_mic_state(MIC_PAUSE);
}

/// <summary>
/// Controls position of the robot for straight lines
/// </summary>
/// <param name="pid">Control the uses of the PID</param>
/// <param name="distance">Limit the distance to a value in cm, if zero limit is desactivated</param>
/// <param name="speed">Control the speed of the motors in cm/s</param>
/// <param name="stop_fist">Safety distance to avoid collision to walls or objects</param>
/// <returns>Void</returns>
void goforward(uint8_t pid, float distance, int8_t speed, uint8_t stop_dist)
{
	if(pid == PID_PAUSE)
	{
		pid_pause(PID_PAUSE);
		if(distance == 0)
		{
			while(1)
			{
				if(stop_dist > get_distance()) // Goback if too close
					if(stop_dist > get_distance())
						motors_advanced_set_speed(-speed, -speed);
				else
					break;
			}
			motors_advanced_set_speed(speed, speed);
			while(get_distance() > stop_dist)
				chThdSleepMilliseconds(10);
			motors_advanced_stop();
			set_front_led(0);
		}
		else
			motors_advanced_set_position(distance, distance, speed, speed);
	}
	else
	{
		pid_pause(PID_PLAY);
		while(get_distance() > stop_dist)
			chThdSleepMilliseconds(10);
		pid_pause(PID_PAUSE);
	}
}


/// <summary>
/// Takes the robot to arena center
/// </summary>
/// <returns>Void</returns>
void gotoarenacenter(void)
{
	 gotoedge();
	 motors_advanced_turnright(134, HIGH_SPEED);
	 goforward(PID_PAUSE, 30, VERY_HIGH_SPEED, TOO_CLOSE_OF_THE_WALL);
}

/// <summary>
/// Takes the robot to an edge
/// </summary>
/// <returns>Void</returns>
void gotoedge(void)
{
	gotowall();
	wall_to_right();
	goforward(PID_PLAY, 0, STD_SPEED,TOO_CLOSE_OF_THE_WALL);
	motors_advanced_set_speed(1,1);
	while(1)
	{
		if(get_prox(0) > 1000 || get_prox(7) > 1000)
		{
			motors_advanced_stop();
			precise_alignment_wall();
			motors_advanced_turnright(90, LOW_SPEED);
			precise_alignment_wall();
			break;
		}
		chThdSleepMilliseconds(10);
	}
}

/// <summary>
/// Takes the robot to the nearest wall
/// </summary>
/// <returns>Void</returns>
void gotowall(void)
{
	int16_t angle_min = 0;
	angle_min = findwall();
	set_front_led(1);
	align_to_wall(angle_min);
	set_body_led(0);
	chThdSleepMilliseconds(150);
	goforward(PID_PAUSE, 0, HIGH_SPEED,TOO_CLOSE_OF_THE_WALL);
	precise_alignment_wall();
}

/// <summary>
/// Find the direction of the nearest wall
/// </summary>
/// <returns>int16_t angle in degree to the nearest wall relatives to started position</returns>
int16_t findwall(void)
{
	uint8_t max_norm_index = -1;
	uint16_t max_norm = 1000;
	uint16_t tmp = VL53L0X_get_dist_mm();
	//measure all the distances from 0 to 360
	for(uint16_t i = 0; i < NUMBER_OF_MEASURE; i++)
	{
		set_body_led(0);
		tmp = get_distance();
		if(tmp > 1000)
		{
			motors_advanced_turnleft(ANGLE_RESOLUTION, HIGH_SPEED);
			continue;
		}
		if(tmp < max_norm){
			tmp = get_distance();
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

/// <summary>
/// First direction to align faster to a given angle to the wall
/// </summary>
/// <param name="angle_min">angle to move</param>
/// <returns>Void</returns>
void align_to_wall(int16_t angle_min)
{
	if(angle_min <= 180)
		motors_advanced_turnleft(angle_min, STD_SPEED);
	else
		motors_advanced_turnright(360-angle_min, STD_SPEED);
}


/// <summary>
/// Use the two IR sensors in front of the robot to align perpendicular to the wall at a static distance.
/// </summary>
/// <returns>Void</returns>
void precise_alignment_wall(void)
{
	set_body_led(1);
	int left = 2000; int right = 2000;
	motors_advanced_set_speed(-2,-2);
	while(left > 1000 || right > 1000)
	{
		left = get_prox(7);
		right = get_prox(0);
		chThdSleepMilliseconds(5);
	}
	left = 0; right = 0;
	motors_advanced_set_speed(2,2);
	while(left < 1000 || right < 1000)
	{
		left = get_prox(7);
		right = get_prox(0);
		chThdSleepMilliseconds(5);
	}
	motors_advanced_stop();
	chThdSleepMilliseconds(100);

	if(left>right)//turnleft
		motors_advanced_set_speed(0.5,-0.5);
	else
		motors_advanced_set_speed(-0.5,0.5);

	uint8_t align = 0;
	while(align < 2)
	{
		left = get_prox(7);
		right = get_prox(0);
		if(left>right)//turnleft
			motors_advanced_set_speed(0.5,-0.5);
		else
			motors_advanced_set_speed(-0.5,0.5);
		if(abs(left-right) < 40)
			align++;
		else
			align = 0;
		chThdSleepMilliseconds(1);
	}
	motors_advanced_stop();
	chThdSleepMilliseconds(100);
	set_body_led(0);
}

/// <summary>
/// Utilitary function to read distance from TOF sensor
/// </summary>
/// <returns>distance in mm</returns>
uint16_t get_distance(void)
{
	int d = 500;
	uint8_t counter = 0;
	while(d > 400)
	{
		counter++;
		chThdSleepMilliseconds(60);
		d = VL53L0X_get_dist_mm();
		if(counter == 6)
			d = 399;
	}
	return d;
}

/// <summary>
/// keep wall to the right
/// </summary>
/// <returns>Void</returns>
void wall_to_right(void){
	motors_advanced_turnleft(90, HIGH_SPEED);
}

/// <summary>
/// Main loop which control the process of cleaning waste
/// It uses 4 states
/// </summary>
/// <returns>Void</returns>
void search_waste(void){

	static uint8_t state = 0;
	static int angle =0;
	static uint16_t angle_waste;
	int16_t p[19] = {0};
	p[0] = 1;
	while(1)
	{
		switch(state){
			case STATE_SEARCH:
			 while(state==STATE_SEARCH){
				while(1)
				{
					chThdSleepMilliseconds(50);
					p[angle] = waste_in_sight();

					motors_advanced_turnright(5, LOW_SPEED);
					angle++;

					if(angle ==  19)
					{
						angle = 0;
						break;
					}
				}
				for(int i=0; i<19; i++)
				{
					if(abs(p[i]-look_up_table[i]) > ERROR_RESOLUTION && abs(p[i+1]-look_up_table[i+1]) > ERROR_RESOLUTION
							&& abs(p[i+2]-look_up_table[i+2]) > ERROR_RESOLUTION)
					{
						set_front_led(1);
						angle_waste = i+2;
						state=STATE_PICKUP;
						break;
					}
				}
				chThdSleepMilliseconds(120);
				if(state == STATE_SEARCH)
				{
				motors_advanced_turnright(5, LOW_SPEED);
				//offset de 5 degres pour compenser le fait que le robot tourne pas forcement de 5 degres a chaque fois
				}
			 }
			case STATE_PICKUP:
				pickup_waste(angle_waste);
				state = STATE_HOME;
				break;

			case STATE_HOME:
				gohome();
				state = STATE_RECYCLE;
				break;

			case STATE_RECYCLE:
				goback(frequence,HIGH_SPEED);
				motors_advanced_turnleft(90, STD_SPEED);
				throwwaste();
				state = STATE_RESET;
				break;

			case STATE_RESET:
				motors_advanced_turnleft(90, VERY_HIGH_SPEED);
				goforward(PID_PLAY, 0, STD_SPEED, TOO_CLOSE_OF_THE_WALL);
				motors_advanced_turnright(135, HIGH_SPEED);
				goforward(PID_PAUSE, 30, HIGH_SPEED,TOO_CLOSE_OF_THE_WALL);
				state = STATE_SEARCH;
				break;
		}
	}
}

uint16_t waste_in_sight(void)
{
	uint16_t dist = get_distance();
	uint16_t dist2 = 0;
	while(1)
	{
		dist2 = get_distance();

		if(abs(dist-dist2)<10)
			break;
		dist = dist2;
	}
	return dist;
}

void pickup_waste(uint16_t _angle){
	motors_advanced_turnleft(90 - _angle*5, LOW_SPEED);
	chThdSleepMilliseconds(2000);
	goforward(PID_PAUSE, 0, LOW_SPEED,100);
	chThdSleepMilliseconds(1000);
	//shoveldown();
	//goforward(PID_PAUSE, 1, LOW_SPEED);
	//shovelup();
}
void goback(uint16_t frequence, int8_t speed){

	if(abs(frequence-FREQ_DECHET_1) < FREQUENCE_RESOLUTION)
	{
		goforward(PID_PAUSE, DIST_DECHET_1, -speed,TOO_CLOSE_OF_THE_WALL);
	}
	if(abs(frequence-FREQ_DECHET_2) < FREQUENCE_RESOLUTION)
	{
		goforward(PID_PAUSE, DIST_DECHET_2, -speed,TOO_CLOSE_OF_THE_WALL);
	}
	if(abs(frequence-FREQ_DECHET_3) < FREQUENCE_RESOLUTION)
	{
		goforward(PID_PAUSE, DIST_DECHET_3, -speed,TOO_CLOSE_OF_THE_WALL);
	}

}
void shoveldown(void)
{
}

void shovelup(void)
{
}


void throwwaste(void)
{
}

void gohome(void)
{
	gotoedge();
	set_mic_state(MIC_PLAY);
	start_listening();
	uint8_t found_camera = 0;
	while(found_camera == 0)
	{
		wait_send_to_computer();
		int16_t listen_freq = get_frequence();

		switch(listen_freq)
		{
			case FREQ_DECHET_1:
			case FREQ_DECHET_2:
			case FREQ_DECHET_3:
				frequence = listen_freq;
				set_front_led(1);
				found_camera = 1;

				break;
			default:
				motors_advanced_turnleft(180, VERY_HIGH_SPEED);
				go_to_another_edge();
		}
	}

}

void go_to_another_edge(void){
	chThdSleepMilliseconds(200);
	goforward(PID_PLAY, 0, STD_SPEED,40);
	while(1)
	{
		motors_advanced_set_speed(1,1);
		if(get_prox(0) > 1000 || get_prox(7) > 1000)
		{
			motors_advanced_stop();
			precise_alignment_wall();
			motors_advanced_turnright(90, HIGH_SPEED);
			precise_alignment_wall();
			break;
		}
		chThdSleepMilliseconds(10);
	}
}
