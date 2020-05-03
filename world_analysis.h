#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <conveyor_bot.h>

bool wa_getObject(int16_t *offset);

void wa_store_object(position_t *obj_pos, int16_t angle);

void wa_wait_analysis_done(void);

void wa_camera_enable(bool enable);

void world_analysis_start(void);

uint16_t get_mean(uint16_t * values, uint16_t new_value, uint16_t i);

#endif /* PROCESS_IMAGE_H */
