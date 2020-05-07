#include "ch.h"
#include "hal.h"

#include <motors.h>
#include <movements.h>

//-------------------------------------------------------defines-------------------------------------------------------

#define NSTEP_ONE_TURN      	1000 // number of steps for 1 turn of the motor
#define MOTOR_MIN_SPEED_STEP   	150  // [step/s]

#define WHEEL_PERIMETER     	13    // [cm]
#define WHEEL_DISTANCE     	 	5.35f //cm
#define PI                 	 	3.1415926536f
#define EPUCK_R_PERIMETER		WHEEL_DISTANCE*PI // Perimeter of one E-Puck2 rotation

#define FULL_TURN				360
#define MAX_ANGLE				180
#define DIFF_STEPS_PER_DEGREE	7.1827f

//------------------------------------------------------macros-------------------------------------------------------------

#define CM2STEPS(x)		NSTEP_ONE_TURN*(x)/WHEEL_PERIMETER
#define MM2STEPS(x)		CM2STEPS(x)/10
#define ANGLE2STEPS(x)	CM2STEPS(EPUCK_R_PERIMETER*(x)/360)

//--------------------------------------------------static variables-------------------------------------------------------

static bool ongoing_mvt = false;

static int32_t r_stepCnt = 0;
static int32_t l_stepCnt = 0;

//----------------------------------------------------functions------------------------------------------------------------

void mvt_stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);

	r_stepCnt = 0;
	l_stepCnt = 0;

	ongoing_mvt = false;
}

void mvt_calibrate(void){
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void mvt_set_speed(int8_t speed_r, int8_t speed_l)
{
	//	speed[cm/s]/Perimeter[cm] = [turn/s] => [turn/s]*nb_step_one_turn[step/turn] = [step/s]
	int16_t speed_r_converted = CM2STEPS(speed_r);
	int16_t speed_l_converted = CM2STEPS(speed_l);

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

void mvt_set_position(int32_t steps_r, int32_t steps_l, int8_t speed_r, int8_t speed_l){

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

bool mvt_move(int16_t dist, int8_t speed){

	if(ongoing_mvt) return false;
	ongoing_mvt = true;

	int16_t steps = MM2STEPS(dist);

	mvt_set_position(steps, steps, speed, speed);

	return true;
}

bool mvt_rotate(int16_t angle, int8_t speed){

	if(ongoing_mvt) return false;
	ongoing_mvt = true;

	int16_t steps = ANGLE2STEPS(angle);

	mvt_set_position(steps, -steps, speed, speed);

	return true;
}

int16_t mvt_get_angle(void){

	int32_t diff = right_motor_get_pos() - left_motor_get_pos();

	int16_t angle = (diff/DIFF_STEPS_PER_DEGREE);
	while(angle>MAX_ANGLE)angle-=FULL_TURN;
	while(angle<-MAX_ANGLE)angle+=FULL_TURN;

	return (int16_t) angle;
}

void mvt_wait_end_of_movement(void){
	while(ongoing_mvt) chThdSleepMilliseconds(50);
}

/**
* @thread   Thread for reading motor positions and doing various movement-related actions
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
			if(right_done && left_done) ongoing_mvt = false;
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
