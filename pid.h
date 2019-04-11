#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			2.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define TOO_CLOSE_OF_THE_WALL 50


//start the PI regulator thread
void pid_regulator_start(void);

#endif /* PI_REGULATOR_H */
