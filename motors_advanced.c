#include <motors.h>
#include <motors_advanced.h>
#define WHEEL_PERIMETER	13
#define MAX_SPEED_STEP	1000

void motors_advanced_reset_position(void);

static int target_position_right = 0;
static int target_position_left = 0;

void motors_advanced_init(void)
{
	motors_init();
	motors_advanced_set_speed(0,0);
	motors_advanced_reset_position();
}

void motors_advanced_reset_position()
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

/*vitesses en cm/s*/
void motors_advanced_set_speed(float speed_r, float speed_l)
{
	if(speed_r > WHEEL_PERIMETER)
		speed_r = WHEEL_PERIMETER;
	if(speed_l > WHEEL_PERIMETER)
		speed_l = WHEEL_PERIMETER;
	left_motor_set_speed(speed_l/WHEEL_PERIMETER*MAX_SPEED_STEP);
	right_motor_set_speed(speed_r/WHEEL_PERIMETER*MAX_SPEED_STEP);
}

void motors_advanced_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	motors_advanced_reset_position();
	target_position_right = position_r * MAX_SPEED_STEP / WHEEL_PERIMETER;
	target_position_left = position_l * MAX_SPEED_STEP / WHEEL_PERIMETER;
	motors_advanced_set_speed(speed_r, speed_l);
	while(motors_advanced_position_reached() == POSITION_NOT_REACHED){
		continue;
	}
	motors_advanced_stop();

}
void motors_advanced_stop(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

uint8_t motors_advanced_position_reached(void)
{
	if(abs(left_motor_get_pos()) >= abs(target_position_left) && abs(right_motor_get_pos()) >= abs(target_position_right))
		return 1;
	else
		return POSITION_NOT_REACHED;
}

void motors_advanced_turnleft(int16_t angle, uint16_t speed)
{
	float corrected_angle = (float)angle/ANGLE_MAX*PERIMETER_EPUCK;
	motors_advanced_set_position(corrected_angle, corrected_angle, speed, -speed);
}

void motors_advanced_turnright(int16_t angle, uint16_t speed)
{
	float corrected_angle = (float)angle/ANGLE_MAX*PERIMETER_EPUCK;
	motors_advanced_set_position(corrected_angle, corrected_angle, -speed, speed);
}
