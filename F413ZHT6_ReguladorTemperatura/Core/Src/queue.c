#include "stm32f4xx_hal.h"

#define MAX 4


typedef struct Queue {
	int queue[MAX];
	int front;
	int rear;
} Q;

void Insert(int front, int rear, uint16_t value, uint16_t queue[MAX] ){
	if((front == 0 && rear == MAX-1) || (front == rear + 1)){
				for(int i = 0; i<MAX; i++){
					queue[i] = -1;
				}
				front = 0;
				rear = 0;
			}
			if(front == -1){
				front = 0;
				rear = 0;
			} else {
				if (rear == MAX-1){
					rear = 0;
				} else { rear = rear + 1;}
			}

			queue[rear] = value;
}




