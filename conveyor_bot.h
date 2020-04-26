/*
 * conveyor_bot.h
 *
 *  Created on: 25 avr. 2020
 *      Author: Hadrien
 */

#ifndef CONVEYOR_BOT_H_
#define CONVEYOR_BOT_H_

/** Struct containing a position in space in regards to the robot */
typedef struct {
    /** Trigonometric angle between robot orientation and point */
    float angle;
    /** Distance between robot center and point */
    float dist;
} position_t;

typedef enum{
	RED_TARGET,			//Target where the small obj must be deposited
	GREEN_TARGET,		//Target where the medium obj must be deposited
	BLUE_TARGET,		//Target where the large obj must be deposited
	SMALL_OBJ,			//Black tube 3 cm in diameter
	MEDIUM_OBJ,			//Black tube 4 cm in diameter
	LARGE_OBJ,			//Black tube 5 cm in diameter
	NB_GAMEOBJECT		//Number of objects in the playing area
} gameObject_t;

void conveyor_bot_init(void);

#endif /* CONVEYOR_BOT_H_ */
