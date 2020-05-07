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
    int16_t angle;
    /** Distance between robot center and point */
    uint16_t dist;
} position_t;

typedef enum{
	RED_TGT,			//Target where the small obj must be deposited
	GREEN_TGT,			//Target where the medium obj must be deposited
	BLUE_TGT,			//Target where the large obj must be deposited
	SMALL_OBJ,			//Black tube 3 cm in diameter
	MEDIUM_OBJ,			//Black tube 4 cm in diameter
	LARGE_OBJ,			//Black tube 5 cm in diameter
	ORIGIN,			//Equivalent to null
	NB_GAMEOBJECT		//Number of objects in the playing area
} gameObject_t;

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

/*
 * @brief	Initializes all the necessary threads
 */
void conveyor_bot_init(void);

#endif /* CONVEYOR_BOT_H_ */
