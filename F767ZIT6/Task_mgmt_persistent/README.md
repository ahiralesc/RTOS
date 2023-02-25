### Task management: Persistent task

The example illustrates how to create a single persistent task. Periodic work is done by interchangeably toggling (ON/OFF) a led and executing a delay for a given time length. The delay gives the impression that the task is periodic. Note:
- ```vTaskDelay```, places the task in **block state** for a given time length clock ticks).
- ```HAL_Delay```, emulates the delay via a **[busy wait](https://en.wikipedia.org/wiki/Busy_waiting)**.

Thus, if in the following code you were to use HAL_Delay in lines 2 and 3. The resulting task is persistent as it would not yield control of the processor unless a higher priority task or interrupt preempts control. Which in this example, only the later case may occur. On the other hand, if you were to use vTaskDelay in line 2 and 3. Then, the task becomes periodic as the task yields control each time it calls vTaskDelay. However, there are no warranties that the delay length is constant. See vTaskDelay documentation for further details. 


```C
void persistentTaskHook(void const * argument)
{
	/**
	 *  Alternatives to vTaskDelay include: HAL_Delay 
	 *  and osDelay.
	 **/
	for(;;)
	{
		/* 1. Toggled the green LED ON */
		HAL_GPIO_WritePin(IGreen_GPIO_Port, IGreen_Pin, GPIO_PIN_SET);

		/* 2. Delay the task execution for 2 seconds */
		vTaskDelay(pdMS_TO_TICKS(2000));

		/* 3. Toggled the green LED OFF */
		HAL_GPIO_WritePin(IGreen_GPIO_Port, IGreen_Pin, GPIO_PIN_RESET);

		/* 4. Delay the task execution for 2 seconds */
		HAL_Delay(2000);
	}
}
```

See [Task mgmt persistent](Task_mgmt_persistent.pdf) for proyect setup. See [README_ESP](README_ESP.md) for spanish translation.  