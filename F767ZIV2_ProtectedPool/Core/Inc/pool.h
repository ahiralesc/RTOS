/*
 * pool.h
 *
 *  Created on: Oct 17, 2021
 *      Author: ahiralesc
 */

#ifndef INC_POOL_H_
#define INC_POOL_H_

#include "main.h"
#include "cmsis_os.h"

typedef struct {
	int green_rate;
	int red_rate;
} LED_RATE;


osSemaphoreId_t poolSemaphoreHandle;
const osSemaphoreAttr_t poolSemaphore_attributes = {
  .name = "poolSemaphore"
};

LED_RATE pool = { 10, 5 };

#endif /* INC_POOL_H_ */
