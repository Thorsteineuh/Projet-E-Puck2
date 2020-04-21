#include <motor.h>
#include <motors.h>

//-------------------------------------------------------defines-------------------------------------------------------

#define NSTEP_ONE_TURN      	1000 // number of step for 1 turn of the motor
#define MOTOR_MIN_SPEED_LIMIT   230 // [step/s]

#define WHEEL_PERIMETER     13 // [cm]
#define WHEEL_DISTANCE      5.35f    //cm
#define PI                  3.1415926536f

//--------------------------------------------------static variables-------------------------------------------------------


//--------------------------------------------private functions declarations----------------------------------------------


//----------------------------------------------------functions------------------------------------------------------------



//	Stops the motors
void motor_stop(void)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

//	Sets the speed of the motors.
//	The parameters are in cm/s for the speed.
void motors_set_speed(float speed_r, float speed_l)
{
	//	speed[cm/s]/Perimeter[cm] = [turn/s] => [turn/s]*nb_step_one_turn[step/turn] = [step/s]
	float speed_r_converted = speed_r/WHEEL_PERIMETER*NSTEP_ONE_TURN;
	float speed_l_converted = speed_l/WHEEL_PERIMETER*NSTEP_ONE_TURN;

	//	Limitation to minimum [step/s] corresponding to minimum speed (3 cm/s) before motors start missing steps (bad luck)
	if(speed_r_converted < MOTOR_MIN_SPEED_LIMIT && speed_r_converted > -MOTOR_MIN_SPEED_LIMIT) speed_r_converted = 0;
	if(speed_l_converted < MOTOR_MIN_SPEED_LIMIT && speed_l_converted > -MOTOR_MIN_SPEED_LIMIT) speed_l_converted = 0;

	right_motor_set_speed(speed_r_converted);
	left_motor_set_speed(speed_l_converted);
}
/*
//	Sets the position to reach for each motor.
//	The parameters are in cm for the positions and in cm/s for the speeds.
//	Returns false if the precedent request is not fulfilled yet.
bool motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	if(!ready) return false;

	// dist[cm]/perimeter[cm] = nb_turn[turn] => nb_turn[turn]*nb_step_one_turn[step/turn] = nb_step[step]
	leftcnt = position_l/WHEEL_PERIMETER*NSTEP_ONE_TURN;
	rightcnt = position_r/WHEEL_PERIMETER*NSTEP_ONE_TURN;

	motor_set_speed(speed_r,speed_l);

	ready = false;
	return true;
}

bool motor_turn(float radius, int angle, float speed)
{
	if(!ready) return false;

	if(radius > WHEEL_DISTANCE/2){

		float valExt = PI/180*(radius + WHEEL_DISTANCE/2)*angle/WHEEL_PERIMETER*NSTEP_ONE_TURN;
		float valInt = PI/180*(radius - WHEEL_DISTANCE/2)*angle/WHEEL_PERIMETER*NSTEP_ONE_TURN;
		float speedExt = speed/radius*(radius + WHEEL_DISTANCE/2);
		float speedInt = speed/radius*(radius - WHEEL_DISTANCE/2);

		leftcnt = valExt;
		rightcnt = valInt;

		motor_set_speed(speedInt, speedExt);

	} else if(radius < -WHEEL_DISTANCE/2){
		radius = -radius;
		float valExt = PI/180*(radius + WHEEL_DISTANCE/2)*angle/WHEEL_PERIMETER*NSTEP_ONE_TURN;
		float valInt = PI/180*(radius - WHEEL_DISTANCE/2)*angle/WHEEL_PERIMETER*NSTEP_ONE_TURN;
		float speedExt = speed/radius*(radius + WHEEL_DISTANCE/2);
		float speedInt = speed/radius*(radius - WHEEL_DISTANCE/2);

		leftcnt = valInt;
		rightcnt = valExt;

		motor_set_speed(speedExt,speedInt);
	} else {
		motor_set_position(10,10,5,5);
	}

	ready = false;
	return true;
}

//	Interrupt of the timer of the right motor.
//	Performs a step of the motor and stops it if it reaches the position given in motor_set_position().
void MOTOR_RIGHT_IRQHandler(void)
{
	static int counter = 0;

	right_motor_update(step_table[counter]);

	if(rightcnt > 0){ 			//	Motor must make one step forward
		rightcnt --;
		counter ++;
		if(counter == 4) counter = 0;
	}else if(rightcnt < 0){		//	Motor must make one step backward
		rightcnt ++;
		counter --;
		if(counter == -1) counter = 3;
	}else if(rightcnt == 0){	//	Motor is at requested position, no movement
		MOTOR_RIGHT_TIMER->ARR = 0;
		if(leftcnt == 0) ready = true;
	}

	// Clear interrupt flag
	MOTOR_RIGHT_TIMER->SR &= ~TIM_SR_UIF;
	MOTOR_RIGHT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

//	Interrupt of the timer of the left motor.
//	Performs a step of the motor and stops it if it reaches the position given in motor_set_position().
void MOTOR_LEFT_IRQHandler(void)
{
	static int counter = 0;

	left_motor_update(step_table[counter]);

	if(leftcnt > 0){ 			//	Motor must make one step forward
		leftcnt --;
		counter ++;
		if(counter == 4) counter = 0;
	}else if(leftcnt < 0){		//	Motor must make one step backward
		leftcnt ++;
		counter --;
		if(counter == -1) counter = 3;
	}else if(leftcnt == 0){		//	Motor is at requested position, no movement
		MOTOR_LEFT_TIMER->ARR = 0;
		if(rightcnt == 0) ready = true;
	}

	// Clear interrupt flag
    MOTOR_LEFT_TIMER->SR &= ~TIM_SR_UIF;
    MOTOR_LEFT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}
*/

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l){

	right_motor_set_pos(position_r*NSTEP_ONE_TURN/WHEEL_PERIMETER);
	left_motor_set_pos(position_l*NSTEP_ONE_TURN/WHEEL_PERIMETER);

	motors_set_speed(speed_r, speed_l);
}

static THD_WORKING_AREA(waMotorRegulation, 256);
static THD_FUNCTION(MotorRegulation, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
    	time = chVTGetSystemTime();

    	int32_t rPos = right_motor_get_pos();
    	int32_t lPos = left_motor_get_pos();

    	//if(rPos < 2 && rPos > -2) right_motor_set_speed(0);
    	//if(lPos < 2 && lPos > -2) left_motor_set_speed(0);

        chThdSleepUntilWindowed(time, time + MS2ST(1));
    }
}

void motor_init(void){
	motors_init();
	chThdCreateStatic(waMotorRegulation, sizeof(waMotorRegulation), NORMALPRIO+2, MotorRegulation, NULL);
}
