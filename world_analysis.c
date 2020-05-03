#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <math.h>
#include <camera/po8030.h>

#include <world_analysis.h>
#include <sensors/VL53L0X/VL53L0X.h>

//-----------------------------------------------------defines-------------------------------------------------------------

#define VAL_THRES 20
#define WIDTH_MAX_ERROR 15

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

static bool camera_enabled = true;
static bool found_object = false;
static int16_t center_offset = 0;
static gameObject_t facing_object;
static uint16_t facing_dist;
//static bool ongoing_analysis = false;

//----------------------------------------------------semaphores-----------------------------------------------------------

static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(analyze_request_sem, TRUE);
static BSEMAPHORE_DECL(analysis_done_sem, TRUE);

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
/*
void wa_analyze_image(void){
	chBSemSignal(&analyze_request_sem);
}*/

void wa_wait_analysis_done(void){
	/*if(ongoing_analysis)*/ chBSemWait(&analysis_done_sem);
}

void wa_camera_enable(bool enable){
	camera_enabled = enable;
}

uint16_t image_analysis(uint8_t* canal, uint16_t size){

	uint8_t mean = 0;
	uint8_t min = 32;
	uint8_t max = 0;


	for(uint16_t i = 0; i < size; i++){			//finding the smallest and biggest values
		if(canal[i] < min) min = canal[i];
		if(canal[i] > max) max = canal[i];
	}

	if (max-min < VAL_THRES) return 0;		//s'il n'y a pas de creux ignorer - thres a definir

	mean = (min + max)/4;				//magic num

	uint16_t start = -1;
	uint16_t moyen;
	uint16_t temp1;
	uint16_t temp2 = canal[size/2];
	uint16_t temp3 = canal[size/2+1];
	uint16_t temp4 = canal[size/2+2];


	for(uint16_t i = size/2 + 3; i < size; i++){
		temp1 = temp2;
		temp2 = temp3;
		temp3 = temp4;
		temp4 = canal[i];
		moyen = (temp1 + temp2 + temp3 + temp4)/4;

		if (moyen<mean) {
			start = i - 3;
			break;
		}
	}

	temp2 = canal[size/2-1];
	temp3 = canal[size/2-2];
	temp4 = canal[size/2-3];

	for(uint16_t i = size/2 - 4; i > 0; i--){
		temp1 = temp2;
		temp2 = temp3;
		temp3 = temp4;
		temp4 = canal[i];
		moyen = (temp1 + temp2 + temp3 + temp4)/4;

		if ((moyen<mean)&&(start-size/2>size/2-i+4)) {
			start = i + 4;
			break;
		}
	}

	uint16_t step = 3;
	uint16_t width = 0;
	temp2 = canal[start];
	temp3 = canal[start+1];
	temp4 = canal[start+2];

	while (start+step<size) {
		temp1 = temp2;
		temp2 = temp3;
		temp3 = temp4;
		temp4 = canal[start+step];
		moyen = (temp1 + temp2 + temp3 + temp4)/4;

		if (moyen<mean) step++;
		else break;
	}
	if (start+step==size){
		found_object = false;
		return 0;
	}
	width += step - 3;

	step = 4;
	temp2 = canal[start-1];
	temp3 = canal[start-2];
	temp4 = canal[start-3];

	while (start-step>0) {
		temp1 = temp2;
		temp2 = temp3;
		temp3 = temp4;
		temp4 = canal[start-step];
		moyen = (temp1 + temp2 + temp3 + temp4)/4;
		if (moyen<mean) step++;
		else break;
	}
	if (start-step==0){
		found_object = false;
		return 0;
	}
	width += step;

	center_offset = (start-step+width/2)-size/2;
	found_object = (width <= 10)? false : true;

	return width;
}

bool wa_getObject(int16_t *offset){
	*offset = center_offset;
	return found_object;
}

uint16_t get_mean(uint16_t * values, uint16_t new_value, uint16_t i) {
	values[i%4] = new_value/4;
	return values[0] + values[1] + values[2] + values[3];
}

void wa_store_object(position_t *obj_pos, int16_t angle){
	obj_pos[facing_object].angle = angle;
	obj_pos[facing_object].dist = facing_dist;
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
    	if(camera_enabled){
			//starts a capture
			dcmi_capture_start();
			//waits for the capture to be done
			wait_image_ready();
			//signals an image has been captured
			chBSemSignal(&image_ready_sem);
    	} else chThdSleepMilliseconds(1000);
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	static uint8_t red[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t green[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t blue[IMAGE_BUFFER_SIZE] = {0};

	uint8_t *img_buff_ptr;

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

		uint16_t r_width = image_analysis(red, IMAGE_BUFFER_SIZE);
		uint16_t g_width = image_analysis(green, IMAGE_BUFFER_SIZE);
		uint16_t b_width = image_analysis(blue, IMAGE_BUFFER_SIZE);

		bool rg_same = abs(r_width-g_width) < WIDTH_MAX_ERROR;
		bool gb_same = abs(g_width-b_width) < WIDTH_MAX_ERROR;
		bool br_same = abs(b_width-r_width) < WIDTH_MAX_ERROR;
		if(rg_same && gb_same && br_same) get_facing_object_and_distance(VL53L0X_get_dist_mm(), g_width, &facing_dist);

		(void)r_width;
		(void)b_width;
		//chprintf((BaseSequentialStream *)&SD3, "Width = %d mm \r", g_width);
		SendUint8ToComputer(blue,IMAGE_BUFFER_SIZE);

		//ongoing_analysis = false;
		chBSemSignal(&analysis_done_sem);
    }
}

void world_analysis_start(void){

	VL53L0X_start();
    dcmi_start();
	po8030_start();

	camera_enabled = true;

	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
