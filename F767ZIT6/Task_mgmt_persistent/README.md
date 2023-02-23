### Task management: Persistent task

The example illustrates how to create a single persistent task. Periodic work is done by interchangeably toggling (ON/OFF) a led and executing a fix length delay. This gives the illusion that the task is periodic, but it is persistent as it never yields control of the processor unless an interrupt occurs. 


```C
void persistentTaskHook(void const * argument)
{
	/**
	 *  Alternatives to vTaskDelay include: HAL_Delay 
	 *  and osDelay.
	 **/
	for(;;)
	{
		/* 1. The green LED is toggled ON (GPIO_PIN_SET) */
		HAL_GPIO_WritePin(IGreen_GPIO_Port, IGreen_Pin, GPIO_PIN_SET);

		/* 2. A delay of 2 seconds is executed */
		vTaskDelay(pdMS_TO_TICKS(2000));

		/* 3. The green LED is toggled OFF */
		HAL_GPIO_WritePin(IGreen_GPIO_Port, IGreen_Pin, GPIO_PIN_RESET);

		/* 4. A delay if 2 seconds in performed */
		HAL_Delay(2000);
	}
}
```