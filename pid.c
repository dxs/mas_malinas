#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors_advanced.h>
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include <pid.h>

static uint8_t pid_sleep = 0;


//simple PI regulator implementation
float pid_regulator_V1(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < 0.1){
		return -0.1;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}


	speed = (KP * error) - 0.5;

	return speed;
	if(speed > 0)
		speed = speed/2;
	else
		speed *= 2;




    return speed;
}
float get_distance_cm_sensor(int sensor_number){
	float distance_cm = 0;
	int tmp = get_prox(sensor_number);
	distance_cm = -0.000423*tmp+2;
	return distance_cm;
}

//simple PI regulator implementation
int16_t pid_regulator_V2(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	if(fabs(error) < 50){

	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = 0.01 * error - 1.0;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPid, 256);
static THD_FUNCTION(Pid, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    float speed = 0;

    while(1){
			if(pid_sleep == PID_PAUSE)
					chThdSleepUntilWindowed(time, time + MS2ST(4000));
			else {
				time = chVTGetSystemTime();

				//computes the speed to give to the motors
				//distance_cm is modified by the image processing thread
				speed = pid_regulator_V1(get_distance_cm_sensor(2), GOAL_DISTANCE);
				float frontdist = get_distance_cm_sensor(1);
				if(frontdist < 1.3)
					motors_advanced_turnleft(5,5);
				//applies the speed from the PI regulator and the correction for the rotation
				motors_advanced_set_speed(10 - ROTATION_COEFF*speed, 10 + ROTATION_COEFF *speed);
				chThdSleepUntilWindowed(time, time + MS2ST(10));
			}
    }
}

void pid_regulator_start(void){
	chThdCreateStatic(waPid, sizeof(waPid), NORMALPRIO, Pid, NULL);
}
void pid_pause(uint8_t _sleep){
	pid_sleep = _sleep;
}
