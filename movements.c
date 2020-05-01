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

#define CM2STEPS_COEF			NSTEP_ONE_TURN/WHEEL_PERIMETER
#define TURN_COEF				PI/180

//--------------------------------------------------static variables-------------------------------------------------------

static bool ongoing_mvt = false;

//----------------------------------------------------semaphores-----------------------------------------------------------

static BSEMAPHORE_DECL(end_of_movement_sem, TRUE);

//----------------------------------------------------functions------------------------------------------------------------

void mvt_stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);

	ongoing_mvt = false;
	chBSemSignal(&end_of_movement_sem);
}

void mvt_set_speed(float speed_r, float speed_l)
{
	//	speed[cm/s]/Perimeter[cm] = [turn/s] => [turn/s]*nb_step_one_turn[step/turn] = [step/s]
	float speed_r_converted = speed_r/WHEEL_PERIMETER*NSTEP_ONE_TURN;
	float speed_l_converted = speed_l/WHEEL_PERIMETER*NSTEP_ONE_TURN;

	//	Limitation to minimum [step/s] corresponding to minimum speed (3 cm/s) before motors start missing steps (bad luck)
	if(speed_r_converted < MOTOR_MIN_SPEED_STEP && speed_r_converted > -MOTOR_MIN_SPEED_STEP) speed_r_converted = 0;
	if(speed_l_converted < MOTOR_MIN_SPEED_STEP && speed_l_converted > -MOTOR_MIN_SPEED_STEP) speed_l_converted = 0;

	right_motor_set_speed(speed_r_converted);
	left_motor_set_speed(speed_l_converted);
}

bool mvt_set_position(float position_r, float position_l, float speed_r, float speed_l){

	if(ongoing_mvt) return false;

	ongoing_mvt = true;

	if(speed_r < 0) speed_r = -speed_r;
	if(speed_r < MOTOR_MIN_SPEED_CM) position_r = 0;
	if(speed_l < 0) speed_l = -speed_l;
	if(speed_l < MOTOR_MIN_SPEED_CM) position_l = 0;

	//Prepares the motor positions to countdown the right amount of steps
	//If the speed is positive, the counter will go up by one every step so the value is set negative.
	if(position_r < 0){
		right_motor_set_pos(-position_r*CM2STEPS_COEF);
		speed_r = -speed_r;
	} else right_motor_set_pos(-position_r*CM2STEPS_COEF);

	if(position_l < 0){
		left_motor_set_pos(-position_l*CM2STEPS_COEF);
		speed_l = -speed_l;
	} else left_motor_set_pos(-position_l*CM2STEPS_COEF);

	mvt_set_speed(speed_r, speed_l);

	return true;
}

bool mvt_move(float dist, float speed){

	return mvt_set_position(dist, dist, speed, speed);

}

bool mvt_rotate(float angle, float speed){

	if(ongoing_mvt) return false;

	float dist = EPUCK_R_PERIMETER*angle/360;

	return mvt_set_position(dist,-dist,speed,speed);
}

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

    while(1){
    	time = chVTGetSystemTime();

    	int32_t rPos = right_motor_get_pos();
    	int32_t lPos = left_motor_get_pos();

    	if(ongoing_mvt){
    		bool right_done = false;
    		bool left_done = false;
    		// The stopping condition is a range of multiple values because
    		// the sampling is done at low frequency (it may miss some steps)
			if(rPos == 0){
				right_motor_set_speed(0);
				right_done = true;
			}
			if(lPos == 0){
				left_motor_set_speed(0);
				left_done = true;
			}
			if(right_done && left_done){
				ongoing_mvt = false;
				chBSemSignal(&end_of_movement_sem);
			}
    	}
    	//The positions check is made every ms. (The motors max speed is 1.1 step/ms)
        chThdSleepUntilWindowed(time, time + US2ST(500));
    }
}

void mvt_init(void){
	motors_init();
	chThdCreateStatic(waMotorRegulation, sizeof(waMotorRegulation), NORMALPRIO+1, MotorRegulation, NULL);
}
