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
#define COMPLETE_TURN	360
#define MAX_ANGLE		180

#define ACQUISITION_MVT_SPEED	3	//Speed at which to rotate
#define CENTER_MAX_ERROR 10
//#define ACQUISITION_ANGLE_STEP	10*CUSTOM_ANGLE 	//Angle in degrees to rotate between analysis

#define E_PUCK_RADIUS 40

#define DEG2RAD M_PI/180	//Degrees to radians coef

//--------------------------------------------------static variables-------------------------------------------------------

//------------------------------------------private functions declarations-------------------------------------------------

void update_coor(position_t * tableau, uint16_t dist, int16_t angle);

//----------------------------------------------------functions------------------------------------------------------------

void update_coor(position_t * tableau, uint16_t dist, int16_t angle) {
	for (int8_t i = 0; i < NB_GAMEOBJECT; i++) {
		if (angle == 0) {
			uint16_t D = tableau[i].dist;
			float32_t theta = tableau[i].angle * DEG2RAD;
			float Dp = sqrt(dist*dist + D*D - 2*dist*D*arm_cos_f32(theta));
			tableau[i].dist = Dp;
			float value = (D*D + Dp*Dp - dist*dist)/(2*D*Dp);
			float valeur = acos(value) / DEG2RAD;
			if (theta * dist >= 0) tableau[i].angle += (int16_t) valeur;
			else tableau[i].angle -= (int16_t)valeur;
		} else if (dist == 0) tableau[i].angle += angle;
	}
	for(uint8_t i = 0; i < NB_GAMEOBJECT; i++){
		chprintf((BaseSequentialStream *)&SD3, "Obj %d dist %d angle %d \r",i,tableau[i].dist,tableau[i].angle);
	}chprintf((BaseSequentialStream *)&SD3, "\n \n \n \r");
}

/**
* @thread   Thread for general behavior of the conveyor bot
*/
static THD_WORKING_AREA(waConveyorBot, 512);
static THD_FUNCTION(ConveyorBot, arg) {

    chRegSetThreadName(__FUNCTION__);
    position_t *obj_pos = (position_t *)arg;

    for(uint8_t i = 0; i < NB_GAMEOBJECT; i++){
    	obj_pos[i].angle = 0;
    	obj_pos[i].dist = 0;
    }

    systime_t time;
    gameState_t state = ACQUISITION;

    gameObject_t object;
    gameObject_t target;
    object = SMALL_OBJ;
    target = RED_TGT;

    bool found_object = false;
    bool turn_complete = false;
    bool all_obj = false;
    int16_t offset = 0;
    int16_t current_angle = 0;
    int8_t stability_cnt = 0;

    uint8_t radius[3];
    radius[0]=15;
    radius[1]=20;
    radius[2]=25;

    mvt_calibrate();

    while(1){
    	time = chVTGetSystemTime();

    	switch(state){
    	case ACQUISITION :{
    		//The robot looks around to find the gameObjects
			current_angle = mvt_get_angle();
			turn_complete = current_angle < 0 && current_angle > -5;
			if(turn_complete){
				mvt_stop();
				all_obj = true;
				for(uint8_t i = 0; i < NB_GAMEOBJECT; i++){
				    	if(obj_pos[i].dist <= 15) all_obj = false;
				}
				if(all_obj){
					state++;
					update_coor(obj_pos,0,-mvt_get_angle());
					object = SMALL_OBJ;
					target = RED_TGT;
				}
				else mvt_set_speed(ACQUISITION_MVT_SPEED, -ACQUISITION_MVT_SPEED);
			}else if(!found_object) mvt_set_speed(ACQUISITION_MVT_SPEED, -ACQUISITION_MVT_SPEED);

			wa_wait_analysis_done();

			found_object = wa_getObject(&offset);
			if(found_object){
				mvt_stop();
				if(abs(offset) > CENTER_MAX_ERROR){
					mvt_rotate(-offset/abs(offset), ACQUISITION_MVT_SPEED);
					mvt_wait_end_of_movement();
					stability_cnt = 0;
				}else {
					chThdSleepMilliseconds(100); //Wait to be certain the next measure is stable
					if(stability_cnt ==10){
						//If the robot did not move for the past second analyzes the object
						wa_wait_analysis_done();
						wa_store_object(obj_pos,current_angle);
						mvt_rotate(20,ACQUISITION_MVT_SPEED);
						mvt_wait_end_of_movement();
						found_object = false;
					}else stability_cnt++;
				}
			}

			break;
    	}
    	case TAKE_OBJECT_1 :
    	case TAKE_OBJECT_2 :
    	case TAKE_OBJECT_3 :{
    		mvt_rotate(obj_pos[object].angle,3);
    		update_coor(obj_pos,0,-obj_pos[object].angle);
    		mvt_wait_end_of_movement();

    		mvt_move((obj_pos[object].dist-E_PUCK_RADIUS-radius[object-3])/10,3);
    		update_coor(obj_pos,obj_pos[object].dist-E_PUCK_RADIUS-radius[object-3],0);
			mvt_wait_end_of_movement();
    		state++;
    		break;
    	}
    	case MOVE_TO_TARGET_1 :
    	case MOVE_TO_TARGET_2 :
    	case MOVE_TO_TARGET_3 :{
    		mvt_rotate(obj_pos[target].angle,3);
			update_coor(obj_pos,0,-obj_pos[target].angle);
			mvt_wait_end_of_movement();

			mvt_move((obj_pos[target].dist-E_PUCK_RADIUS-radius[object-3])/10,3);
			update_coor(obj_pos,obj_pos[target].dist-E_PUCK_RADIUS-radius[object-3],0);
			mvt_wait_end_of_movement();

			mvt_move(-6,3);
			update_coor(obj_pos,-60,0);
			mvt_wait_end_of_movement();

			object++;
			target++;
			state++;
    		break;
    	}
    	case END_OF_TASK :
    		palTogglePad(GPIOB, GPIOB_LED_BODY);
    		chThdSleepMilliseconds(500);
    		break;
    	default : chThdSleepUntilWindowed(time, time + MS2ST(2000));
    	}


    	//chprintf((BaseSequentialStream *)&SDU1, "Dist = %d mm\r", mm);


    }
}

void conveyor_bot_init(void){

	static position_t objects_pos[NB_GAMEOBJECT];

	mvt_init();
	world_analysis_start();

	chThdCreateStatic(waConveyorBot, sizeof(waConveyorBot), NORMALPRIO, ConveyorBot, objects_pos);
}
