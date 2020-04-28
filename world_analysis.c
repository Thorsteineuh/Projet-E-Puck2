#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <world_analysis.h>
#include <sensors/VL53L0X/VL53L0X.h>


#define HEIGHT_THRES 20		//a def
#define MM_THRES 4
#define ANGLE_CORR 0.9063	// = cos(25)

static float distance_cm = 0;

//semaphores
//static BSEMAPHORE_DECL(image_request_sem, TRUE);
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
    	//Takes images only if asked to do so
    	//chBSemWait(&image_request_sem);
    	//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);			//a augmenter peut-etre
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	static uint8_t red[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t green[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t blue[IMAGE_BUFFER_SIZE] = {0};
	uint8_t *img_buff_ptr;
	bool send = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(int i = 0; i < IMAGE_BUFFER_SIZE*2; i += 2){
			blue[i/2]	= *(img_buff_ptr + i + 1) & 0b11111;	//isolate the blue channel
			green[i/2]	= *(img_buff_ptr + i + 1) >> 5;			//isolate the green channel (3 LSB)
			green[i/2] += (*(img_buff_ptr + i) & 0b111) << 5;	//isolate the green channel (3 MSB)
			red[i/2]	= *(img_buff_ptr + i) >> 3;				//isolate the red channel
		}

		if(send){
			SendUint8ToComputer(green, IMAGE_BUFFER_SIZE);
			send = false;
		}
		else send = true;

		uint16_t r_width = image_analysis(red, IMAGE_BUFFER_SIZE);
		uint16_t g_width = image_analysis(green, IMAGE_BUFFER_SIZE);
		uint16_t b_width = image_analysis(blue, IMAGE_BUFFER_SIZE);

		//chprintf((BaseSequentialStream *)&SDU1, "largeur red = %d ", r_width);
		//chprintf((BaseSequentialStream *)&SDU1, "largeur green = %d ", g_width);
		//chprintf((BaseSequentialStream *)&SDU1, "largeur blue = %d \r", b_width);

		if (g_width == 0 && r_width>0 && b_width>0) chprintf((BaseSequentialStream *)&SDU1, "c'est une cible verte \r");
		if (r_width == 0 && g_width>0 && b_width>0) chprintf((BaseSequentialStream *)&SDU1, "c'est une cible rouge \r");
		if (b_width == 0 && r_width>0 && g_width>0) chprintf((BaseSequentialStream *)&SDU1, "c'est une cible bleue \r");

		if (g_width>0 && r_width>0 && b_width>0) {
			uint16_t dist_mm = VL53L0X_get_dist_mm() * ANGLE_CORR;
			uint8_t width_mm = g_width * dist_mm * 0.414 * 2 / IMAGE_BUFFER_SIZE;		//ici 0.414 = tan(22.5)
			//chprintf((BaseSequentialStream *)&SDU1, "largeur = %d mm \r", width_mm);

			//if (abs(width_mm-30)<MM_THRES) chprintf((BaseSequentialStream *)&SDU1, "c'est un petit cylindre \r");
			//if (abs(width_mm-40)<MM_THRES) chprintf((BaseSequentialStream *)&SDU1, "c'est un moyen cylindre \r");
			//if (abs(width_mm-50)<MM_THRES) chprintf((BaseSequentialStream *)&SDU1, "c'est un grand cylindre \r");
		}
    }
}

uint16_t image_analysis(uint8_t* canal, uint16_t size){

	uint8_t min = -1;			//value of the min
	uint16_t test_i = 2*size;

	for(uint16_t i = 0; i < size; i++){			//finding the smallest value
		if(canal[i] < min) {
			test_i = i;			//index of the min
			min = canal[i];
		}
	}

	uint16_t width = 0;
	uint8_t test_val = min + HEIGHT_THRES;
	min = test_i;			// /!\ index of the min
	test_i = 2*size;

	for (uint16_t i = min; i < size; i++) {
		if (canal[i]>test_val) {
			test_val = canal[i] + HEIGHT_THRES;
			test_i = i;			//index of the right edge of object
		}
	}

	width = test_i;
	test_val = canal[min] + HEIGHT_THRES;

	for (uint16_t i = min - 1; i > 0; i--) {
		if (canal[i]>test_val) {
			test_val = canal[i] + HEIGHT_THRES;
			test_i = i;			//index of the left edge of object
		}
	}

	width -= test_i;
	if (width > size) width = 0;
	//uint8_t width_mm = width * dist_mm * 1 * 2 / IMAGE_BUFFER_SIZE;		//ici 1 = tan(45)
	//chprintf((BaseSequentialStream *)&SDU1, "distance = %f \n", distance_cm);

	return 0;
}

uint16_t get_distance_mm(void){

	return VL53L0X_get_dist_mm();
}

float get_distance_cm(void){

	return distance_cm;
}

void world_analysis_start(void){

	VL53L0X_start();
    dcmi_start();
	po8030_start();

	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
