#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void process_image_start(void);
uint16_t image_analysis(uint8_t* canal, uint16_t size);

#endif /* PROCESS_IMAGE_H */
