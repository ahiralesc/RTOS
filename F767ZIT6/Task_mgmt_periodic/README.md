### Task management: Periodic task

The example illustrates how to create a single periodic task. Periodic work is done by toggling (ON/OFF) a led and executing a finite length delay twice. A delay gives the impression that the task is periodic. ```vTaskDelayUntil``` registers the cycle instance at wich the task must start execution.


```C
void periodicTaskHook(void const * argument)
{
	/* 1. The cycle time period (or tick count) the task will remain in block state */
	TickType_t tickCount;

	/* 2. The time the task will remain in block state is 2 seconds */
	TickType_t frequency = pdMS_TO_TICKS(2000);

	/* 3. Gets the system tick count */
	tickCount = xTaskGetTickCount();

	for(;;)
	{
			/* 4. Emulate some work by toggling ON the board blue LED */
			HAL_GPIO_WritePin(IBlue_GPIO_Port, IBlue_Pin, GPIO_PIN_SET);

			/*
			 * 5. Delay the task execution for frequency (2s) length. This will transition the task
			 * from running to block state.
			 */
			vTaskDelayUntil( &tickCount, frequency );

			/* 6. Emulate some work by toggling OFF the board blue LED */
			HAL_GPIO_WritePin(IBlue_GPIO_Port, IBlue_Pin, GPIO_PIN_RESET);

			/* 7. Delay the task execution for 2s */
			vTaskDelayUntil( &tickCount, frequency );
		}
}
```

See [Task mgmt periodic](Task_mgmt_periodic.pdf) for proyect setup. See [README_ESP](README_ESP.md) for spanish translation.  