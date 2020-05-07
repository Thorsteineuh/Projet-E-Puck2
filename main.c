#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <main.h>

//--------------------------------- Include of homemade files -----------------------------------

#include <conveyor_bot.h>

//----------------------------------------- Functions ------------------------------------------

int main(void)
{
	// Basics initialization
    halInit();
    chSysInit();
    mpu_init();

    // Activate the robot
    conveyor_bot_init();

    /* Infinite loop. */
    while(1){
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
