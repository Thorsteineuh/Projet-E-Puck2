/*
 * conveyor_bot.c
 *
 *  Created on: 25 avr. 2020
 *      Author: Hadrien
 */
#include "ch.h"
#include "hal.h"


#include <chprintf.h>
#include <main.h>

#include <conveyor_bot.h>
#include <movements.h>
#include <process_image.h>

//-------------------------------------------------------defines-------------------------------------------------------

//--------------------------------------------------static variables-------------------------------------------------------

//----------------------------------------------------functions------------------------------------------------------------


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

    	chprintf((BaseSequentialStream *)&SD3, "Blue angle = %f Medium dist = %f \r", obj_pos[BLUE_TARGET].angle, obj_pos[LARGE_OBJ].dist);

        chThdSleepUntilWindowed(time, time + MS2ST(1));
    }
}

void conveyor_bot_init(void){

	static position_t objects_pos[NB_GAMEOBJECT];
	objects_pos[LARGE_OBJ].dist = 118;

	mvt_init();

	chThdCreateStatic(waConveyorBot, sizeof(waConveyorBot), NORMALPRIO, ConveyorBot, objects_pos);
}
