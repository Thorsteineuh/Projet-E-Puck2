/*
 * conveyor_bot.c
 *
 *  Created on: 25 avr. 2020
 *      Author: Hadrien
 */
#include "ch.h"
#include "hal.h"

#include <main.h>
#include <math.h>

#include <conveyor_bot.h>
#include <movements.h>
#include <world_analysis.h>

//-----------------------------------------------------defines-------------------------------------------------------------

typedef enum{
	ACQUISITION,		//The robot looks at his surroundings to locate each gameObject
	TAKE_OBJECT_1,		//The robot moves to the object closest to the targets and grabs it
	MOVE_TO_TARGET_1,	//The robot goes to deliver the object to its objective
	TAKE_OBJECT_2,		//Repeats with second object
	MOVE_TO_TARGET_2,	//...
	TAKE_OBJECT_3,		//Repeats with third object
	MOVE_TO_TARGET_3,	//...
	END_OF_TASK			//Nothing left to do. A dance maybe ?
} gameState_t;

#define SMALL_RADIUS	15
#define MEDIUM_RADIUS	20
#define LARGE_RADIUS	25
#define E_PUCK_RADIUS 	40
#define BACK_UP_DIST	60

#define COMPLETE_TURN_ERROR 5
#define COMPLETE_TURN	360
#define MAX_ANGLE		180
#define SKIP_OBJ_ANGLE	20

#define ROTATION_SPEED	3
#define MOVE_SPEED		6

#define SMALL_DRAG		1.0014f
#define MEDIUM_DRAG		1.0141f
#define LARGE_DRAG		1.0141f

#define CENTER_MAX_ERROR 	10

#define STABILITY_THRESHOLD	10

//------------------------------------------------------macros-------------------------------------------------------------

#define DEG2RAD(x) M_PI*(x)/180
#define RAD2DEG(x) 180*(x)/M_PI

//--------------------------------------------------static variables-------------------------------------------------------

//------------------------------------------private functions declarations-------------------------------------------------

void center_on_target(void);

void update_coordinates(position_t * tableau, int16_t dist, int16_t angle);

//----------------------------------------------------functions------------------------------------------------------------

void center_on_target(void){

	mvt_stop();

	bool centered = false;
	int16_t offset;
	int8_t stability_cnt = 0;

	while(!centered){

		wa_wait_analysis_done();
		offset = wa_getOffset();

		if(abs(offset) > CENTER_MAX_ERROR){
			mvt_rotate(-offset/abs(offset), ROTATION_SPEED);
			mvt_wait_end_of_movement();
			stability_cnt = 0;
		}else {
			chThdSleepMilliseconds(100); //Wait to be certain the next measure is stable
			if(stability_cnt == STABILITY_THRESHOLD){
				//If the robot did not move for the past second : It is centered
				centered = true;
			}else stability_cnt++;
		}
	}
}

void update_coordinates(position_t * tableau, int16_t dist, int16_t angle) {

	for (int8_t i = 0; i < NB_GAMEOBJECT; i++) {
		if (angle == 0) {

			//Change is purely linear, calculation via cosine theorem
			uint16_t oldDist = tableau[i].dist;
			int16_t newAngle;
			float newDist;

			if(dist >= 0){
				float cosOldA = cosf(DEG2RAD(tableau[i].angle));
				newDist = sqrt(oldDist*oldDist + dist*dist - 2*dist*oldDist*cosOldA);
				float newAngleComp = RAD2DEG(acosf((dist-oldDist*cosOldA)/newDist));
				newAngle = (int16_t) (180-newAngleComp);
			}else {
				float cosCompOldA = cosf(DEG2RAD(180-abs(tableau[i].angle)));
				newDist = sqrt(oldDist*oldDist + dist*dist + 2*dist*oldDist*cosCompOldA);
				newAngle = RAD2DEG(acosf((-dist-oldDist*cosCompOldA)/newDist));
			}

			if(tableau[i].angle < 0) newAngle = -abs(newAngle);
			else newAngle = abs(newAngle);
			tableau[i].angle = newAngle;
			tableau[i].dist = (uint16_t) newDist;

		} else if(dist == 0){
			//Change is a pure rotation
			if(tableau[i].dist != 0){
				tableau[i].angle -= angle;
				while(tableau[i].angle > MAX_ANGLE) tableau[i].angle -= COMPLETE_TURN;
				while(tableau[i].angle < -MAX_ANGLE) tableau[i].angle += COMPLETE_TURN;
			}else tableau[i].angle = 0;
		}
	}
}

/**
* @thread   Thread for general behavior of the conveyor bot
*/
static THD_WORKING_AREA(waConveyorBot, 512);
static THD_FUNCTION(ConveyorBot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


	static position_t obj_pos[NB_GAMEOBJECT];
    for(uint8_t i = 0; i < NB_GAMEOBJECT; i++){
    	obj_pos[i].angle = 0;
    	obj_pos[i].dist = 0;
    }

    gameState_t state = ACQUISITION;

    //Initial couple of targets
    gameObject_t object = SMALL_OBJ;
    gameObject_t target = RED_TGT;

    //Coefficients for the drag correction when rotating
    float drag[3];
    drag[0] = SMALL_DRAG;
    drag[1] = MEDIUM_DRAG;
    drag[2] = LARGE_DRAG;

    uint8_t radius[3];
    radius[0] = SMALL_RADIUS;
    radius[1] = MEDIUM_RADIUS;
    radius[2] = LARGE_RADIUS;

    //Sets origin of rotation
    mvt_calibrate();
	wa_camera_enable(true);

    while(1){
    	switch(state){
    	case ACQUISITION :{ //The robot looks around to find the gameObjects

    		int16_t current_angle = mvt_get_angle();
			bool turn_complete = current_angle < 0 && current_angle > -COMPLETE_TURN_ERROR;

			if(turn_complete){
				mvt_stop();
				bool all_obj_stored = true;
				//Ensure all objects are stored before continuing
				for(uint8_t i = 0; i < NB_GAMEOBJECT-1; i++)
					if(obj_pos[i].dist <= E_PUCK_RADIUS) all_obj_stored = false;
				if(all_obj_stored){
					state++;
					//Sets all coordinates to be relative to the robot's position
					update_coordinates(obj_pos, 0, mvt_get_angle());
				}
				else mvt_set_speed(ROTATION_SPEED, -ROTATION_SPEED);
			}else mvt_set_speed(ROTATION_SPEED, -ROTATION_SPEED);

			wa_wait_analysis_done();

			if(wa_getObject()){
				center_on_target();
				wa_store_object(obj_pos, mvt_get_angle());
				mvt_rotate(SKIP_OBJ_ANGLE, ROTATION_SPEED);
				mvt_wait_end_of_movement();
			}
			break;
    	}
    	case TAKE_OBJECT_1 :
    	case TAKE_OBJECT_2 :
    	case TAKE_OBJECT_3 :{
    		mvt_rotate(obj_pos[object].angle, ROTATION_SPEED);
    		update_coordinates(obj_pos, 0, obj_pos[object].angle);
    		mvt_wait_end_of_movement();

    		wa_camera_enable(true);
    		center_on_target();
    		wa_camera_enable(false); //Disable camera while moving

    		int16_t dist = obj_pos[object].dist - E_PUCK_RADIUS - radius[object-3];
    		mvt_move(dist, MOVE_SPEED);
    		update_coordinates(obj_pos, dist, 0);
			mvt_wait_end_of_movement();
    		state++;
    		break;
    	}
    	case MOVE_TO_TARGET_1 :
    	case MOVE_TO_TARGET_2 :
    	case MOVE_TO_TARGET_3 :{

    		mvt_rotate(obj_pos[target].angle * drag[object-3], ROTATION_SPEED);
    		update_coordinates(obj_pos, 0, obj_pos[target].angle);
			mvt_wait_end_of_movement();

			int16_t dist = obj_pos[target].dist - E_PUCK_RADIUS - radius[object-3];
			mvt_move(dist, MOVE_SPEED);
			update_coordinates(obj_pos, dist, 0);
			mvt_wait_end_of_movement();

			mvt_move(-BACK_UP_DIST, MOVE_SPEED);
			update_coordinates(obj_pos, -BACK_UP_DIST, 0);
			mvt_wait_end_of_movement();

			object++;
			target++;
			state++;
    		break;
    	}
    	case END_OF_TASK :
    		mvt_rotate(obj_pos[ORIGIN].angle, ROTATION_SPEED);
    		mvt_wait_end_of_movement();
    		mvt_move(obj_pos[ORIGIN].dist, MOVE_SPEED);
    		mvt_wait_end_of_movement();
    		mvt_set_speed(ROTATION_SPEED, -ROTATION_SPEED);
    		state++;
    		break;
    	default :
    		palTogglePad(GPIOB, GPIOB_LED_BODY);
    		chThdSleepMilliseconds(500);
    	}
    }
}

void conveyor_bot_init(void){

	mvt_init();
	world_analysis_start();

	chThdCreateStatic(waConveyorBot, sizeof(waConveyorBot), NORMALPRIO, ConveyorBot, NULL);
}
