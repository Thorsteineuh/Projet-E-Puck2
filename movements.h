#ifndef MOTOR_H
#define MOTOR_H


#include <stdlib.h>

/**
* @brief   Stops the robot movements immediately
*/
void mvt_stop(void);

/**
* @brief   Sets the motors to the desired speed
*
* @param speed_r, speed_l	speed of right and left motor
*/
void mvt_set_speed(float speed_r, float speed_l);

/**
* @brief 	Rotates the motors for the desired distance at the desired speed
* 			The parameters are in [cm] and [cm/s]
* 			Positions sign give the direction. Speeds are treated as positive
*
* @param position_r, position_l		Distance in [cm] the right/left motor has to rotate
*
* @return	True if the movements are started,
* 			false if the motors have previous instructions ongoing.
*/
bool mvt_set_position(float position_r, float position_l, float speed_r, float speed_l);

/**
* @brief 	Moves the robot in a straight line
*
* @param dist		Distance to move in [cm]
* @param speed		Speed of the movement
*
* @return	True if the movements are started,
* 			false if the motors have previous instructions ongoing.
*/
bool mvt_move(float dist, float speed);

/**
* @brief   Rotates the robot on its center
*
* @param angle		angle of rotation in degrees
* @param speed		speed of the motors in [cm/s]
*
* @return	True if the movements are started,
* 			false if the motors have previous instructions ongoing.
*/
bool mvt_rotate(float angle, float speed);

/**
* @brief   Moves the robot along an arc
*
* @param radius		radius of the arc in [cm]
* @param angle		angle of rotation in degrees
* @param speed		speed of the motors in [cm/s]
*
* @return	True if the movements are started,
* 			false if the motors have previous instructions ongoing.
*/
bool mvt_turn(float radius, float angle, float speed);

/**
* @brief   Waits until there is no ongoing movement
*/
void mvt_wait_end_of_movement(void);

/**
* @brief   Initialization of the robot movements module
*/
void mvt_init(void);


#endif /* MOTOR_H */
