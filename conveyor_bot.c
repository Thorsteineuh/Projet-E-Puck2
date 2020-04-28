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

#include <conveyor_bot.h>
#include <movements.h>
#include <world_analysis.h>

//-----------------------------------------------------defines-------------------------------------------------------------

//--------------------------------------------------static variables-------------------------------------------------------

//------------------------------------------private functions declarations-------------------------------------------------

//----------------------------------------------------functions------------------------------------------------------------


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

    while(1){
    	time = chVTGetSystemTime();

    	/*uint16_t mm = 0;

    	for(int i = 0; i < 10; i++){
    		mm += get_distance_mm();
    		chThdSleepMilliseconds(100);
    	}
    	mm/=10;
    	chprintf((BaseSequentialStream *)&SDU1, "Dist = %d mm\r", mm);
*/
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
