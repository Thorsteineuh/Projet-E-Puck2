#ifndef WORLD_ANALYSIS_H
#define WORLD_ANALYSIS_H

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
* @brief 	Initialization of the analysis threads
*/
void world_analysis_start(void);

#endif /* WORLD_ANALYSIS_H */
