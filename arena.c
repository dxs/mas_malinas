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
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>
#define ANGLE_MAX 360
#define ANGLE_resolution 18
#define number_of_measure 20

static VL53L0X_Dev_t* device;

void init_arena(void){
	//inits the sensor
	VL53L0X_init(device);

}

void gotoarenacenter(void){
	gotoedge();
	//turnleft(135);
	//goforward(false, 250);

}
void searchwaste(void){


}
void findwall(void){

	uint16_t distance[number_of_measure];
	int16_t max_norm_index = -1;
	int16_t max_norm = 0;
	int16_t i = 0;
	//measure all the distances from 0° to 360°
	for(i = 0; i < number_of_measure; i++)
	{
		VL53L0X_startMeasure(device, VL53L0X_DEVICEMODE_SINGLE_RANGING);
		VL53L0X_getLastMeasure(device);
		//distance[i] = device->Data->LastRangeMeasure->RangeMilliMeter;
		turnleft(ANGLE_resolution);
	}
	for(i = 0; i < number_of_measure; i++)
	{
			//search for the highest distance
				if(distance[i] > max_norm){
					max_norm = distance[i];
					max_norm_index = i;
				}
	}
}


void gotowall(void){


}
int16_t aligntothewall(int16_t angle_min){


}
void goforward(bool pid_or_not, uint16_t distance){


}
void gotoedge(void){
	gotowall();
	walltoright();
	//goforward(true, 0);

}

void walltoright(void){


}
void turnleft(int16_t angle){


}


