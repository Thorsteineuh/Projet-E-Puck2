#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <conveyor_bot.h>

void wa_camera_enable(bool enable);

void wa_wait_analysis_done(void);

bool wa_getObject(void);

int16_t wa_getOffset(void);

void wa_store_object(position_t *obj_pos, int16_t angle);

void world_analysis_start(void);

#endif /* PROCESS_IMAGE_H */
