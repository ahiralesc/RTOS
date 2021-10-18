/*
 * pool.h
 *
 *  Created on: Oct 17, 2021
 *      Author: ahiralesc
 */

#ifndef INC_DATA_POOL_H_
#define INC_DATA_POOL_H_


/* The LED colors described in the problem model */
enum LCOLOR { red = 0, green = 1 };

/* The shared data pool structure */
typedef struct {
	int green_frequency;
	int red_frequency;
} LFREQUENCY;

/* The data poll APIs */
void set_frequency(enum LCOLOR, int);
int  get_frequency(enum LCOLOR);


#endif /* INC_DATA_POOL_H_ */
