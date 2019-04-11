#ifndef MOTOR_ADVANCED_H
#define MOTOR_ADVANCED_H


#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

void motors_advanced_init(void);
void motors_advanced_set_speed(float speed_r, float speed_l);
void motors_advanced_set_position(float position_r, float position_l, float speed_r, float speed_l);
void motors_advanced_stop(void);
uint8_t motors_advanced_position_reached(void);

#endif /* MOTOR_ADVANCED_H */
