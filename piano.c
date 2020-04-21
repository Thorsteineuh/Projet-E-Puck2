/*
 * piano.c
 *
 *  Created on: 19 avr. 2020
 *      Author: Hadrien
 */

#include <piano.h>
#include <main.h>

#include <sensors/proximity.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <selector.h>

#define PROX_TOPIC_NAME "/proximity"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static BSEMAPHORE_DECL(PlaySound_sem, TRUE);
static int sound_frequency;

int range_to_frequency(int range);

static THD_WORKING_AREA(waPlaySound, 256);
static THD_FUNCTION(PlaySound, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	while(1){
		chBSemWait(&PlaySound_sem);
		playNote(sound_frequency,250);
	}
}

static THD_WORKING_AREA(waPiano, 256);
static THD_FUNCTION(Piano, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t * prox_topic = NULL;
    proximity_msg_t prox_data;

    //Wait until the topic is advertised and found
    while(prox_topic == NULL) prox_topic = messagebus_find_topic(&bus, "/proximity");

    calibrate_ir();

	int mean = 0;
	int count = 0;

    while(1){
    	messagebus_topic_wait(prox_topic, &prox_data, sizeof(prox_data));

    	int selec = get_selector();

    	if(selec != 0){
    		int range = prox_data.delta[2];
			mean += range;
			count++;
			if(count == 15){
				count = 0;
				mean = mean/15;
				mean++;
				sound_frequency = range_to_frequency(mean);
				chBSemSignal(&PlaySound_sem);
				mean = 0;
			}

    	}
    }
}

int range_to_frequency(int range){

	int freq = 0;

	if(range < 25) freq = 0;
	else if(range < 50) freq = NOTE_C2;
	else if(range < 150) freq = NOTE_G2;
	else freq = NOTE_B2;

	return freq;
}

void piano_init(){

    dac_start();

    messagebus_init(&bus,&bus_lock,&bus_condvar);
    proximity_start();

	chThdCreateStatic(waPiano, sizeof(waPiano), NORMALPRIO, Piano, NULL);
	chThdCreateStatic(waPlaySound, sizeof(waPlaySound), NORMALPRIO, PlaySound, NULL);

}
