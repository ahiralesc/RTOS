/*
 * pool.c
 *
 *  Created on: Oct 17, 2021
 *      Author: ahiralesc
 */
#include "pool.h"

void initiate_pool(void) {
	poolSemaphoreHandle = osSemaphoreNew(1, 1, &poolSemaphore_attributes);
}


void set_rate(int led, int rate)
{
	switch(led){
		case 1 : 	pool.green_rate = rate;
					break;
		case 2 : 	pool.red_rate = rate;
					break;
	}
}

int get_rate(int led) {

	switch(led) {
	case 1: return pool.green_rate;
	case 2: return pool.red_rate;
	}
	return -1;
}

