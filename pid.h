#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			0.8f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.0001f	//[cm] because of the noise of the camera
#define KP						1.01f
#define KI 						1.01f	//must not be zero
#define MAX_SUM_ERROR 			0.3
#define TOO_CLOSE_OF_THE_WALL 	50
#define PID_PAUSE				1
#define PID_PLAY				0


//start the PI regulator thread
void pid_regulator_start(void);
void pid_pause(uint8_t _sleep);

#endif /* PI_REGULATOR_H */
