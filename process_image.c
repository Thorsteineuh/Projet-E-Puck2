#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
    	//systime_t time = chVTGetSystemTime();
    	//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//chThdSleepMilliseconds(12);
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

		//time = chVTGetSystemTime() - time;
		//chprintf((BaseSequentialStream *)&SDU1, "time = %d \n", time);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	bool send = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(int i = 1; i < IMAGE_BUFFER_SIZE*2; i += 2){
			image[(i-1)/2] = *(img_buff_ptr + i) & 31;
		}

		if(send){
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
			send = false;
		}
		else send = true;

		image_analysis(image, IMAGE_BUFFER_SIZE);

    }
}

void image_analysis(uint8_t* data, uint16_t size){

	uint16_t mean = 0;
	uint8_t min = 32;
	uint8_t max = 0;


	for(int i = 0; i < size; i++){
		if(data[i] < min) min = data[i];
		if(data[i] > max) max = data[i];
	}
	mean = (min + max)/2;

	uint16_t step = 0;
	uint16_t width = 0;

	while ((step<size/2)&&(data[size/2+step]<mean)) step++;
	width += step;
	step = 0;
	while ((step<size/2-1)&&(data[size/2-step-1]<mean)) step++;
	width += step;


	distance_cm = 2*640/width;
	//chprintf((BaseSequentialStream *)&SDU1, "distance = %f \n", distance_cm);
}

float get_distance_cm(void){

	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
