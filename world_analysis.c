#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <world_analysis.h>
#include <sensors/VL53L0X/VL53L0X.h>

//-----------------------------------------------------defines-------------------------------------------------------------

#define HEIGHT_THRES 20		//a def
#define MM_THRES 4
#define ANGLE_CORR 0.9063	// = cos(25)

//------------------------------------------------------macros-------------------------------------------------------------

// Functions determined via experimentation
#define TOF_TRUEDIST_SMALL(x) 1e-5*(x)*(x)*(x)-0.0061*(x)*(x)+1.5318*(x)-4.673
#define TOF_TRUEDIST_MEDIUM(x) 9e-6*(x)*(x)*(x)-0.0054*(x)*(x)+1.5506*(x)-6.8456
#define TOF_TRUEDIST_LARGE(x) 5e-6*(x)*(x)*(x)-0.0032*(x)*(x)+1.2356*(x)+11.739

// Functions determined via experimentation
#define CAMERA_TRUEDIST_SMALL(x) 21900/(x)
#define CAMERA_TRUEDIST_MEDIUM(x) 28800/(x)
#define CAMERA_TRUEDIST_LARGE(x) 35700/(x)

//--------------------------------------------------static variables-------------------------------------------------------

static float distance_cm = 0;

//semaphores
//static BSEMAPHORE_DECL(image_request_sem, TRUE);
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//------------------------------------------private functions declarations-------------------------------------------------

/**
* @brief 	Corrects the distance returned by the ToF sensor

* @param tof_mm		Raw distance given by the time of flight sensor in [mm]
* @param object		The object supposed to be in front of the robot
*
* @return	calculated distance in [mm]
*/
uint16_t get_ToF_trueDist_mm(uint16_t tof_mm, gameObject_t object);

/**
* @brief 	Converts the width seen by the camera into a distance
*
* @param object_width		Width seen by the camera in [pixels]
* @param object				The object supposed to be in front of the robot
*
* @return	calculated distance in [mm]
*/
uint16_t get_camera_trueDist_mm(uint16_t object_width, gameObject_t object);

/**
* @brief 	Compares distances given by the camera and the ToF
* 			to discriminate the object facing the robot
*
* @param tof_mm			Raw distance given by the time of flight sensor in [mm]
* @param cam_width		Width seen by the camera in [pixels]
* @param dist_return	Address to return the final distance in [mm]
*
* @return	Object found in front of the robot
*/
gameObject_t get_facing_object_and_distance(uint16_t tof_mm, uint16_t cam_width, uint16_t *dist_return);

//----------------------------------------------------functions------------------------------------------------------------

uint16_t get_ToF_trueDist_mm(uint16_t tof_mm, gameObject_t object){

	if(object <= BLUE_TGT) return 0;

	if(object == SMALL_OBJ){
		float dist = TOF_TRUEDIST_SMALL(tof_mm);
		return (uint16_t) dist;
	} else if(object == MEDIUM_OBJ){
		float dist = TOF_TRUEDIST_MEDIUM(tof_mm);
		return (uint16_t) dist;
	} else {
		float dist = TOF_TRUEDIST_LARGE(tof_mm);
		return (uint16_t) dist;
	}
}

uint16_t get_camera_trueDist_mm(uint16_t object_width, gameObject_t object){

	if(object <= BLUE_TGT) return 0;

	if(object == SMALL_OBJ){
		float dist = CAMERA_TRUEDIST_SMALL(object_width);
		return (uint16_t) dist;
	} else if(object == MEDIUM_OBJ){
		float dist = CAMERA_TRUEDIST_MEDIUM(object_width);
		return (uint16_t) dist;
	} else {
		float dist = CAMERA_TRUEDIST_LARGE(object_width);
		return (uint16_t) dist;
	}
}

gameObject_t get_facing_object_and_distance(uint16_t tof_mm, uint16_t cam_width, uint16_t *dist_return){

	uint16_t tof_corr;
	uint16_t cam_corr;
	gameObject_t obj_return = SMALL_OBJ;

	//Hypothesis of the small object
	tof_corr = get_ToF_trueDist_mm(tof_mm, SMALL_OBJ);
	cam_corr = get_camera_trueDist_mm(cam_width, SMALL_OBJ);
	uint16_t error_small = abs(tof_corr - cam_corr);
	*dist_return = cam_corr;

	//Hypothesis of the medium object
	tof_corr = get_ToF_trueDist_mm(tof_mm, MEDIUM_OBJ);
	cam_corr = get_camera_trueDist_mm(cam_width, MEDIUM_OBJ);
	uint16_t error_medium = abs(tof_corr - cam_corr);
	if(error_medium < error_small){
		*dist_return = cam_corr;
		obj_return = MEDIUM_OBJ;
	}

	//Hypothesis of the large object
	tof_corr = get_ToF_trueDist_mm(tof_mm, LARGE_OBJ);
	cam_corr = get_camera_trueDist_mm(cam_width, LARGE_OBJ);
	uint16_t error_large = abs(tof_corr - cam_corr);
	if(error_large < error_medium && error_large < error_small){
		*dist_return = cam_corr;
		obj_return = LARGE_OBJ;
	}

	return obj_return;
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 240 + 241 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 240, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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
		chprintf((BaseSequentialStream *)&SDU1, "largeur green = %d \r", g_width);
		//chprintf((BaseSequentialStream *)&SDU1, "largeur blue = %d \r", b_width);

		/*if (g_width == 0 && r_width>0 && b_width>0) chprintf((BaseSequentialStream *)&SDU1, "c'est une cible verte \r");
		if (r_width == 0 && g_width>0 && b_width>0) chprintf((BaseSequentialStream *)&SDU1, "c'est une cible rouge \r");
		if (b_width == 0 && r_width>0 && g_width>0) chprintf((BaseSequentialStream *)&SDU1, "c'est une cible bleue \r");
*/
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
