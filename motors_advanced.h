#ifndef MOTOR_ADVANCED_H
#define MOTOR_ADVANCED_H


#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

#define ANGLE_MAX 360
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

//Initialize motors
void motors_advanced_init(void);

//Set a speed in cm/s to each motor independantly
void motors_advanced_set_speed(float speed_r, float speed_l);

//Reach a position at a given speed
void motors_advanced_set_position(float position_r, float position_l, float speed_r, float speed_l);

//Stop the motors
void motors_advanced_stop(void);

//Control whether the position is reached or not
uint8_t motors_advanced_position_reached(void);

//Turn to a certain angle at a certain speed to the left
void motors_advanced_turnleft(int16_t angle, uint16_t speed);

//Turn to a certain angle at a certain speed to the right
void motors_advanced_turnright(int16_t angle, uint16_t speed);

#endif /* MOTOR_ADVANCED_H */
