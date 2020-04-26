#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <conveyor_bot.h>

uint16_t get_distance_mm(void);
float get_distance_cm(void);
void world_analysis_start(void);
uint16_t image_analysis(uint8_t* canal, uint16_t size);

#endif /* PROCESS_IMAGE_H */
