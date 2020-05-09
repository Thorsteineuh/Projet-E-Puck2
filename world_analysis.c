#include "ch.h"
#include "hal.h"

#include <main.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include "i2c_bus.h"

#include <world_analysis.h>

//-----------------------------------------------------defines-------------------------------------------------------------

typedef enum{ //Camera channel colors
	RED,
	GREEN,
	BLUE
} color_t;

#define WIDTH_MAX_ERROR 40
#define E_PUCK_RADIUS	35 		//Distance between center of rotation and camera [mm]
#define TARGETS_DIST	270 	//Distance between tip of the claw and wall when in center [mm]
#define MOV_AVG_SIZE 4

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

static thread_t *tof_Thd;

static bool camera_enabled = true;

static bool found_object = false;
static int16_t center_offset = 0;

static gameObject_t facing_object;
static uint16_t facing_dist;

static bool VL53L0X_configured = false;
static uint16_t dist_mm = 0;

//----------------------------------------------------semaphores-----------------------------------------------------------

static BSEMAPHORE_DECL(image_ready_sem, TRUE);
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

/**
* @brief 	Searches the image given by the camera for gameObjects
*
* @param channel		Array of values from one color channel
* @param size			Size of the array
* @param color			The color of the given channel
*
* @return	width of the dent in the values if any
*/
uint16_t image_analysis(uint8_t* channel, uint16_t size, color_t color);

/**
* @brief 	Manages the moving average with a cyclic buffer
*
* @param values			Table of previous values
* @param new_value		new value
* @param i				index needed for the remainder calculation
*
* @return	current mean
*/
uint8_t get_mean(uint8_t * values, uint8_t new_value, uint16_t i);

//----------------------------------------------------functions------------------------------------------------------------

uint16_t get_ToF_trueDist_mm(uint16_t tof_mm, gameObject_t object){

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

uint16_t image_analysis(uint8_t* channel, uint16_t size, color_t color){

	uint8_t threshold = 0;
	uint8_t min = -1;
	uint8_t max = 0;


	for(uint16_t i = 0; i < size; i++){			//finding the smallest and biggest values
		if(channel[i] < min) min = channel[i];
		if(channel[i] > max) max = channel[i];
	}

	if(color == RED)  threshold = max/2;
	if(color == GREEN)threshold = max/4;		//different threshold for each channel
	if(color == BLUE) threshold = max/2;

	uint16_t start = -1;
	//array for the moving average on four values
	uint8_t temp[MOV_AVG_SIZE] = {channel[size/2], channel[size+1/2], channel[size+2/2], 0};

	//check if there is a dip in intensity to the right of the center
	for(uint16_t i = size/2 + MOV_AVG_SIZE - 1; i < size; i++){
		if (get_mean(temp, channel[i], i)<threshold) {
			start = i - MOV_AVG_SIZE + 1;
			break;
		}
	}

	//check if there is a dip in intensity to the left of the center

	//fills the array with the first values for the moving average
	temp[1] = channel[size/2-1];
	temp[2] = channel[size/2-2];
	temp[3] = channel[size/2-3];
	//if this dip is nearer than the right one, choose this one
	for(uint16_t i = size/2 - MOV_AVG_SIZE; i > 0; i--){
		if ((get_mean(temp, channel[i], i)<threshold)&&(start-size/2 > size/2-i + MOV_AVG_SIZE)) {
			start = i + MOV_AVG_SIZE;
			break;
		}
	}

	//store the number of steps to the right to reach an edge
	uint16_t step = MOV_AVG_SIZE - 1;
	uint16_t width = 0;
	temp[0] = channel[start];
	temp[1] = channel[start+1];
	temp[2] = channel[start+2];

	while (start+step<size) {
		if (get_mean(temp, channel[start+step], step)<threshold) step++;
		else break;
	}
	if (start+step==size) return 0;
	width += step - MOV_AVG_SIZE + 1;

	//store the number of steps to the left to reach an edge (left+right)=width
	step = MOV_AVG_SIZE;
	temp[1] = channel[start-1];
	temp[2] = channel[start-2];
	temp[3] = channel[start-3];

	while (start-step>0) {
		if (get_mean(temp, channel[start-step], step)<threshold) step++;
		else break;
	}
	if (start-step==0) return 0;
	width += step - MOV_AVG_SIZE;

	//if a large enough dip was found, there is an object
	if(width > WIDTH_MAX_ERROR){
		found_object = true;
		center_offset = (start-step+width/2)-size/2;
	}

	return width;
}

uint8_t get_mean(uint8_t * values, uint8_t new_value, uint16_t i) {
	values[i % MOV_AVG_SIZE] = new_value;
	return (values[0] + values[1] + values[2] + values[3])/MOV_AVG_SIZE;
}


void wa_camera_enable(bool enable){
	camera_enabled = enable;
	//Takes the semaphore to avoid use of old data
	if(!enable) chBSemWait(&analysis_done_sem);
}

void wa_wait_analysis_done(void){
	chBSemWait(&analysis_done_sem);
}

bool wa_getObject(void){
	return found_object;
}

int16_t wa_getOffset(void){
	return center_offset;
}

void wa_store_object(position_t *obj_pos, int16_t angle){
	//If the detected object is the origin, there was a problem in the detection
	if(facing_object == ORIGIN) return;
	obj_pos[facing_object].angle = angle;
	obj_pos[facing_object].dist = facing_dist+E_PUCK_RADIUS;
}

/*
 * Custom thread to control the mode and update frequency of the ToF sensor
*/
static THD_WORKING_AREA(waVL53L0XThd, 512);
static THD_FUNCTION(VL53L0XThd, arg) {

	chRegSetThreadName("VL53L0x Thd");
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	(void)arg;
	static VL53L0X_Dev_t device;

	device.I2cDevAddr = VL53L0X_ADDR;

	status = VL53L0X_init(&device);

	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configAccuracy(&device, VL53L0X_DEFAULT_MODE);
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configured = true;
	}

    /* Reader thread loop.*/
    while (chThdShouldTerminateX() == false) {
    	if(VL53L0X_configured){
    		VL53L0X_getLastMeasure(&device);
   			dist_mm = device.Data.LastRangeMeasure.RangeMilliMeter;
    	}
		chThdSleepMilliseconds(50);
    }
}

/*
 * Thread to take 2 lines of visual data in the middle of the camera field of view
 */

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 240 + 241 (Middle of the field of view)
	po8030_advanced_config(FORMAT_RGB565, 0, 240, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

	po8030_set_awb(0); //Disable auto white balance

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

/*
 * Thread that analyzes the image and gets the right objects
 */

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
			green[i/2] += (*(img_buff_ptr + i) & 0b111) << 3;	//isolate the green channel (3 MSB)
			red[i/2]	= *(img_buff_ptr + i) >> 3;				//isolate the red channel
		}

		//Default values if no object is found
		facing_object = ORIGIN;
		found_object = false;
		center_offset = 0;

		uint16_t b_width = image_analysis(blue, IMAGE_BUFFER_SIZE, BLUE);
		uint16_t g_width = image_analysis(green, IMAGE_BUFFER_SIZE, GREEN);
		uint16_t r_width = image_analysis(red, IMAGE_BUFFER_SIZE, RED);
		//Red is last because it is the most precise

		bool r_hole = r_width > WIDTH_MAX_ERROR;
		bool g_hole = g_width > WIDTH_MAX_ERROR;
		bool b_hole = b_width > WIDTH_MAX_ERROR;

		if(r_hole && g_hole && b_hole){
			//Black object
			found_object = true;
			facing_object = get_facing_object_and_distance(dist_mm, g_width, &facing_dist);
		}else if(!r_hole && g_hole && b_hole){
			//Red object
			facing_object = RED_TGT;
			found_object = true;
			facing_dist = TARGETS_DIST;
		}else if(r_hole && !g_hole && b_hole){
			//Green object
			facing_object = GREEN_TGT;
			found_object = true;
			facing_dist = TARGETS_DIST;
		}else if(r_hole && !g_hole && !b_hole){
			//Cyan object
			facing_object = BLUE_TGT;
			found_object = true;
			facing_dist = TARGETS_DIST;
		}else {
			//Not recognized color
			found_object = false;
			facing_object = ORIGIN;
		}

		chBSemSignal(&analysis_done_sem);
    }
}


void world_analysis_start(void){

	i2c_start();
    dcmi_start();
	po8030_start();

	camera_enabled = true;

	tof_Thd = chThdCreateStatic(waVL53L0XThd, sizeof(waVL53L0XThd), NORMALPRIO + 10, VL53L0XThd, NULL);
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void wa_stop_tof(void) {
    chThdTerminate(tof_Thd);
    chThdWait(tof_Thd);
    tof_Thd = NULL;
}
