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
#define DEG2RAD M_PI/180

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

    obj_pos[BLUE_TARGET].angle = 87.6;
    obj_pos[MEDIUM_OBJ].dist = 25;
    systime_t time;

    while(1){
    	time = chVTGetSystemTime();

    	uint16_t mm = get_distance_mm();

    	chprintf((BaseSequentialStream *)&SDU1, "Dist = %d mm\r", mm);

        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}

void conveyor_bot_init(void){

	static position_t objects_pos[NB_GAMEOBJECT];
	objects_pos[LARGE_OBJ].dist = 118;

	mvt_init();
	world_analysis_start();

	chThdCreateStatic(waConveyorBot, sizeof(waConveyorBot), NORMALPRIO, ConveyorBot, objects_pos);
}
