/*
 * conveyor_bot.c
 *
 *  Created on: 25 avr. 2020
 *      Author: Hadrien
 */
#include "ch.h"
#include "hal.h"


#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include <math.h>
#include <arm_math.h>

#include <conveyor_bot.h>
#include <movements.h>
#include <world_analysis.h>

//-----------------------------------------------------defines-------------------------------------------------------------

//#define CUSTOM_ANGLE	1.114f	//Angle to make 4 motor steps
#define COMPLETE_TURN	360	  	//Number of custom angles to make a full turn
#define MAX_ANGLE		162	  	//Half a turn in custom angles

#define ACQUISITION_MVT_SPEED	3	//Speed at which to rotate
//#define ACQUISITION_ANGLE_STEP	10*CUSTOM_ANGLE 	//Angle in degrees to rotate between analysis

#define DEG2RAD M_PI/161.58	//Custom angles to radians

//--------------------------------------------------static variables-------------------------------------------------------

//------------------------------------------private functions declarations-------------------------------------------------

void update_coor(position_t * tableau, uint16_t dist, int16_t angle);

//----------------------------------------------------functions------------------------------------------------------------

void update_coor(position_t * tableau, uint16_t dist, int16_t angle) {
	for (int8_t i = 0; i < NB_GAMEOBJECT; i++) {
		if (angle == 0) {
			uint16_t D = tableau[i].dist;
			float32_t theta = tableau[i].angle * DEG2RAD;
			uint16_t Dp = sqrt(dist*dist + D*D - 2*dist*D*arm_cos_f32(theta));
			tableau[i].dist = Dp;
			if (theta * dist >= 0) tableau[i].angle += acos((D*D + Dp*Dp - dist*dist)/(2*D*Dp)) / DEG2RAD;
			else tableau[i].angle -= acos((D*D + Dp*Dp - dist*dist)/(2*D*Dp)) / DEG2RAD;
		} else if (dist == 0) tableau[i].angle += angle;
	}
}

/**
* @thread   Thread for general behavior of the conveyor bot
*/
static THD_WORKING_AREA(waConveyorBot, 256);
static THD_FUNCTION(ConveyorBot, arg) {

    chRegSetThreadName(__FUNCTION__);
    position_t *obj_pos = (position_t *)arg;

    obj_pos[BLUE_TGT].angle = 87.6;
    obj_pos[MEDIUM_OBJ].dist = 25;
    systime_t time;

    gameState_t state = ACQUISITION;
    int16_t offset = 0;
    bool found_object = false;

    mvt_calibrate();

    while(1){
    	time = chVTGetSystemTime();

    	switch(state){
    	case ACQUISITION :
    		//The robot rotates while the analysis is done
    		if(!found_object) mvt_rotate(COMPLETE_TURN, ACQUISITION_MVT_SPEED);
    		wa_wait_analysis_done();

    		found_object = wa_getObject(&offset);
    		if(found_object){
    			mvt_stop();
    			if(abs(offset) > 50){
    				mvt_rotate(offset/abs(offset), ACQUISITION_MVT_SPEED);
    				mvt_wait_end_of_movement();
    				//chprintf((BaseSequentialStream *)&SD3, "Centered ! \r");
    			}
    		}

    		break;
    	case TAKE_OBJECT_1 :
    	case TAKE_OBJECT_2 :
    	case TAKE_OBJECT_3 :
    		//palTogglePad(GPIOD, GPIOD_LED_FRONT);
    		//chThdSleepMilliseconds(500);
    		break;
    	case MOVE_TO_TARGET_1 :
    	case MOVE_TO_TARGET_2 :
    	case MOVE_TO_TARGET_3 :
    		break;
    	case END_OF_TASK :
    		break;
    	default : chThdSleepUntilWindowed(time, time + MS2ST(2000));
    	}


    	//chprintf((BaseSequentialStream *)&SDU1, "Dist = %d mm\r", mm);


    }
}

void conveyor_bot_init(void){

	static position_t objects_pos[NB_GAMEOBJECT];
	objects_pos[LARGE_OBJ].dist = 118;

	mvt_init();
	world_analysis_start();

	chThdCreateStatic(waConveyorBot, sizeof(waConveyorBot), NORMALPRIO, ConveyorBot, objects_pos);
}
