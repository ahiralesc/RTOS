/*
 * pool.c
 *
 *  Created on: Oct 17, 2021
 *      Author: ahiralesc
 */

#include "data_pool.h"
#include "cmsis_os.h"

/*
 * Two external variables are defined in the main program and used
 * in this container, namely the data pool semaphore handdle (dataPoolSempHandle)
 * and the data poll (dpool).
 *
 * Note, declaration and initialization mnust be done at the main container (main.c)
 * Otherwise, if such variables are initialized here they will be considered a new
 * instantiated variable.
 */
extern osSemaphoreId dataPoolSempHandle;
//extern LFREQUENCY dpool;
LFREQUENCY dpool = {100, 100};


void set_frequency(enum LCOLOR color, int frequency)
{
	/* Critical section */
	osSemaphoreWait(dataPoolSempHandle, osWaitForever);

	/* Although variables in the share dpool are independent
	 * a race condition may occur. Thus, they must be managed
	 * within a critical section */
	switch(color){
		case green :
					dpool.green_frequency = frequency;
					break;
		case red :
					dpool.red_frequency = frequency;
					break;
	}

	osSemaphoreRelease(dataPoolSempHandle);
}



int get_frequency(enum LCOLOR color)
{
	/* Critical section */
	osSemaphoreWait(dataPoolSempHandle, osWaitForever);

	/* A value of -1 indicates an error has occurred */
	int frequency = -1;

	/* Although variables in the share dpool are independent
	 * a race condition may occur. Thus, they must be managed
	 * within a critical section */
	switch(color) {
		case green:
			frequency = dpool.green_frequency;
				break;
		case red:
			frequency = dpool.red_frequency;
				break;
	}

	osSemaphoreRelease(dataPoolSempHandle);

	/* A context switch still may occur before the rate is returned */
	return frequency;
}

