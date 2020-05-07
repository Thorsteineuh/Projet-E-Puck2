#ifndef MOVEMENTS_H
#define MOVEMENTS_H

/**
* @brief   Stops the robot movements immediately
*/
void mvt_stop(void);

/**
* @brief   Sets the rotation origin to zero
*/
void mvt_calibrate(void);

/**
* @brief   Sets the motors to the desired speed
* 		   Speeds in [cm/s]
*
* @param speed_r, speed_l	speed of right and left motor
*/
void mvt_set_speed(int8_t speed_r, int8_t speed_l);

/**
* @brief 	Rotates the motors for the desired distance at the desired speed
* 			The parameters are in steps and [cm/s]
* 			Positions sign give the direction. Speeds are treated as positive
*
* @param position_r, position_l		Number of steps the right/left motor has to rotate
* @param speed_r, speed_l			Speeds of the motors in [cm/s]
*/
void mvt_set_position(int32_t position_r, int32_t position_l, int8_t speed_r, int8_t speed_l);

/**
* @brief 	Moves the robot in a straight line
*
* @param dist		Distance to move in [mm]
* @param speed		Speed of the movement [cm/s]
*
* @return	True if the movements are started,
* 			false if the motors have previous instructions ongoing.
*/
bool mvt_move(int16_t dist, int8_t speed);

/**
* @brief   Rotates the robot on its center
*
* @param angle		angle of rotation in degrees
* @param speed		speed of the motors in [cm/s]
*
* @return	True if the movements are started,
* 			false if the motors have previous instructions ongoing.
*/
bool mvt_rotate(int16_t angle, int8_t speed);

/**
* @brief   Returns the angle the robot makes with its calibrated position
*/
int16_t mvt_get_angle(void);

/**
* @brief   Waits until there is no ongoing movement
*/
void mvt_wait_end_of_movement(void);

/**
* @brief   Initialization of the robot movements module
*/
void mvt_init(void);


#endif /* MOVEMENTS_H */
