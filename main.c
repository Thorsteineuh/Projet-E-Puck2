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

//--------------------------------- Include of homemade files -----------------------------------

#include <motor.h>
#include <piano.h>

//----------------------------------------- Functions ------------------------------------------

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
    VL53L0X_start();
    // Activate motors module
    motor_init();
    // Activate piano module (IR and buzzer)
    piano_init();

    //motor_set_position(10,10,5,5);

    /* Infinite loop. */
    while(1){
    	int speed = get_selector();
    	motors_set_speed(speed,speed);
        //chprintf((BaseSequentialStream *)&SD3, "Ambient = %d Prox = %d Calibrated Prox = %d \r", amb,prox,calProx);
    	chThdSleep(MS2ST(100));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
