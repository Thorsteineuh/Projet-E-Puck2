#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <arm_math.h>

#include <main.h>

#include <usbcfg.h>
#include <chprintf.h>
#include <selector.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <camera/po8030.h>

//--------------------------------- Include of homemade files -----------------------------------

#include <conveyor_bot.h>
#include <movements.h>
#include <process_image.h>

//----------------------------------------- Functions ------------------------------------------

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
	// Basics initialization
    halInit();
    chSysInit();
    mpu_init();

    // Activate usb communication module
    serial_start();
    usb_start();
    // Activate ToF module
    //VL53L0X_start();
    // Activate motors module
    conveyor_bot_init();
    //mvt_init();
    // Activate piano module (IR and buzzer)
    //piano_init();

    //starts the camera
    //dcmi_start();
	//po8030_start();
	//process_image_start();

    //motor_set_position(10,10,5,5);

    /* Infinite loop. */
	int state = 1;
    while(1){
    	if(get_selector() == 0){
			switch(state){
			case -1:chThdSleepMilliseconds(1000);state++;break;
			case 0: if(mvt_move(10,10))state++;break;
			case 1: if(mvt_turn(5,180,10))state=0;break;
			default: break;
			}
    	} else {
    		state = -1;
    		mvt_stop();
    	}
    	//motor_set_position(20,20,speed,speed);
    	//float val = VL53L0X_get_dist_mm()*0.9;
        //chprintf((BaseSequentialStream *)&SD3, "Distance = %f \r", val);
    	//chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
