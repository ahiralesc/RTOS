### Gestión de tareas: tarea periodica + regulación de la frecuencia de trabajo

El ejemplo ilustra como crear una tarea periodica. Tal realiza prende/apaga un LED a una frecuencia de 20Hz y cede control por 2 segundos. 

 

```C
void toggleBlueHook(void *argument)
{
	/* 1.
	 * 		TimeStamp and DelayTimeMsec, keep track of the time instance at which
	 * 		the task must unblock and proceed execution.
	 * 		last and interval, keep track of the task remaining execution length.
	 *
	 * */

	TickType_t TimeStamp, last;
	TickType_t DelayTimeMsec = pdMS_TO_TICKS(2000);
	TickType_t interval = 0;

	/* 2. gets the current tick count */
	TimeStamp = xTaskGetTickCount();

	for(;;){

		HAL_GPIO_WritePin(IRed_GPIO_Port, IRed_Pin, GPIO_PIN_SET);

		/* 3. gets the current tick count */
		last = xTaskGetTickCount();
		do {
			HAL_GPIO_TogglePin(IBlue_GPIO_Port, IBlue_Pin);

			/* 4. emulates a frequency of 20 Hz */
			osDelay(50);

			/* 5. estimates the remaining task length relative to a given time length (1000)*/
			interval = xTaskGetTickCount() - last;
		}while(interval < 1000);

		HAL_GPIO_WritePin(IRed_GPIO_Port, IRed_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IBlue_GPIO_Port, IBlue_Pin, GPIO_PIN_RESET);

		/* 6. yield control for 2 seconds */
		TimeStamp += DelayTimeMsec;
		osDelayUntil(TimeStamp);
	}
}
```

Vea  administración de tareas periodica [regulación de la frecuencia de trabajo](Task_mgmt_frequency_regulation.pdf) para parámetros de configuración del proyecto. Vea [README_ESP](README.md) documentación en ingles.  