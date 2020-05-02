#include "ch.h"
#include "hal.h"

#include <motors.h>
#include <chprintf.h>
#include <movements.h>

//-------------------------------------------------------defines-------------------------------------------------------

#define NSTEP_ONE_TURN      	1000 // number of steps for 1 turn of the motor
#define MOTOR_MIN_SPEED_STEP   	230  // [step/s]
#define MOTOR_MIN_SPEED_CM   	2.99f // [cm/s]

#define WHEEL_PERIMETER     	13 // [cm]
#define WHEEL_DISTANCE     	 	5.35f    //cm
#define PI                 	 	3.1415926536f
#define EPUCK_R_PERIMETER		WHEEL_DISTANCE*PI // Perimeter of one E-Puck2 rotation


#define TURN_COEF				PI/180

//------------------------------------------------------macros-------------------------------------------------------------

#define CM2STEPS(x)		NSTEP_ONE_TURN*(x)/WHEEL_PERIMETER
#define ANGLE2STEPS(x)	CM2STEPS(EPUCK_R_PERIMETER*(x)/360)

//--------------------------------------------------static variables-------------------------------------------------------

static bool ongoing_mvt = false;

static int32_t r_stepCnt = 0;
static int32_t l_stepCnt = 0;

//----------------------------------------------------semaphores-----------------------------------------------------------

static BSEMAPHORE_DECL(end_of_movement_sem, TRUE);

//----------------------------------------------------functions------------------------------------------------------------

void mvt_stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);

	r_stepCnt = 0;
	l_stepCnt = 0;

	ongoing_mvt = false;
	chBSemSignal(&end_of_movement_sem);
}

void mvt_set_speed(float speed_r, float speed_l)
{
	//	speed[cm/s]/Perimeter[cm] = [turn/s] => [turn/s]*nb_step_one_turn[step/turn] = [step/s]
	float speed_r_converted = CM2STEPS(speed_r);
	float speed_l_converted = CM2STEPS(speed_l);

	//	Limitation to minimum [step/s] corresponding to minimum speed (3 cm/s) before motors start missing steps (bad luck)
	if(abs(speed_r_converted) < MOTOR_MIN_SPEED_STEP){
		speed_r_converted = 0;
		r_stepCnt = 0; //Avoid waiting on a counter that doesn't change
	}
	if(abs(speed_l_converted) < MOTOR_MIN_SPEED_STEP){
		speed_l_converted = 0;
		l_stepCnt = 0; //Avoid waiting on a counter that doesn't change
	}

	right_motor_set_speed(speed_r_converted);
	left_motor_set_speed(speed_l_converted);
}

void mvt_calibrate(void){
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void mvt_set_position(int32_t steps_r, int32_t steps_l, float speed_r, float speed_l){

	speed_r = abs(speed_r);
	speed_l = abs(speed_l);

	//Sets the motor counters to the right amount of steps
	//If the speed is positive, the counter will go up by one every step so the value is set negative.
	r_stepCnt = -steps_r;
	if(steps_r < 0) speed_r = -speed_r;

	l_stepCnt = -steps_l;
	if(steps_l < 0) speed_l = -speed_l;

	mvt_set_speed(speed_r, speed_l);
}

bool mvt_move(float dist, float speed){

	if(ongoing_mvt) return false;
	ongoing_mvt = true;

	float steps = CM2STEPS(dist);

	mvt_set_position((int) steps, (int) steps, speed, speed);

	return true;
}

bool mvt_rotate(int16_t angle, float speed){

	if(ongoing_mvt) return false;
	ongoing_mvt = true;

	//Has to be a float to avoid calculation errors
	float steps = ANGLE2STEPS(angle);

	mvt_set_position((int) steps,(int) -steps,speed,speed);

	return true;
}

int16_t mvt_get_angle(void){

	int32_t diff = right_motor_get_pos() - left_motor_get_pos();

	float angle = (diff/7.2);
	while(angle>180)angle-=360;
	while(angle<-180)angle+=360;

	return (int16_t) angle;
}

void mvt_wait_end_of_movement(void){
	if(ongoing_mvt) chBSemWait(&end_of_movement_sem);
}

/**
* @thread   Thread for reading motor positions and doing various motor-related actions
*/
static THD_WORKING_AREA(waMotorRegulation, 256);
static THD_FUNCTION(MotorRegulation, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //Memories of the positions during the previous sampling
    int32_t r_oldPos = 0;
    int32_t l_oldPos = 0;

    while(1){
    	time = chVTGetSystemTime();

    	int32_t rPos = right_motor_get_pos();
    	int32_t lPos = left_motor_get_pos();

    	if(ongoing_mvt){
    		bool right_done = false;
    		bool left_done = false;

    		r_stepCnt += (rPos - r_oldPos);
    		l_stepCnt += (lPos - l_oldPos);

			if(r_stepCnt == 0){
				right_motor_set_speed(0);
				right_done = true;
			}
			if(l_stepCnt == 0){
				left_motor_set_speed(0);
				left_done = true;
			}
			if(right_done && left_done){
				ongoing_mvt = false;
				chBSemSignal(&end_of_movement_sem);
			}
    	}

		r_oldPos = rPos;
		l_oldPos = lPos;

    	//The positions check is made every 500us. (The motors max speed is 0.55 step/500us)
        chThdSleepUntilWindowed(time, time + US2ST(500));
    }
}

void mvt_init(void){
	motors_init();
	chThdCreateStatic(waMotorRegulation, sizeof(waMotorRegulation), NORMALPRIO+1, MotorRegulation, NULL);
}

/*
bool mvt_turn(float radius, float angle, float speed)
{

	if(radius == 0) return mvt_rotate(angle, speed);
	else if(radius < 0) return true;
	if(ongoing_mvt) return false;

	//Distances and speeds so the center of the robot follows the arc at the given speed
	float distExt = TURN_COEF*(radius + WHEEL_DISTANCE/2)*angle;
	float distInt = TURN_COEF*(radius - WHEEL_DISTANCE/2)*angle;
	float speedExt = speed/radius*(radius + WHEEL_DISTANCE/2);
	float speedInt = speed/radius*(radius - WHEEL_DISTANCE/2);

	//Special condition to avoid missing motor steps
	if(speedInt < MOTOR_MIN_SPEED_CM || speedExt < MOTOR_MIN_SPEED_CM) return true;

	//Turning left or right
	if(angle > 0) return mvt_set_position(distExt, distInt, speedExt, speedInt);
	else return mvt_set_position(-distInt, -distExt, speedInt, speedExt);
}
*/
