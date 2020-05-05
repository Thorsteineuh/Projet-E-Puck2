#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <conveyor_bot.h>

/**
* @brief 	Stops/starts the execution of the camera thread
*/
void wa_camera_enable(bool enable);

/**
* @brief 	Wait until the analysis data if refreshed
*/
void wa_wait_analysis_done(void);

/*
* @return	true if there is an object in the field of view, false otherwise
*/
bool wa_getObject(void);

/*
* @return	offset in pixel between object and center
*/
int16_t wa_getOffset(void);

/**
* @brief 	Stores the position of the object facing the camera
*
* @param obj_pos		Table of positions
* @param angle			Current angle of the robot
*/
void wa_store_object(position_t *obj_pos, int16_t angle);

/**
* @brief 	Manages the moving average with a cyclic storage
*
* @param values			Table of previous values
* @param new_value		new value
* @param i				index needed for the remainder calculation
*/
uint8_t get_mean(uint8_t * values, uint8_t new_value, uint16_t i);

/**
* @brief 	Initialization of the analysis threads
*/
void world_analysis_start(void);

#endif /* PROCESS_IMAGE_H */
