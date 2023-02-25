### Gestión de tareas: tarea persistente

El ejemplo ilustra cómo crear una sola tarea persistente. La tare realiza trabajo alternando un LED (ENCENDIDO/APAGADO) y ejecutando un retraso durante un período de tiempo determinado. El retraso da la impresión de que la tarea es periódica. Note:

- ```vTaskDelay```, coloca la tarea en **estado de bloqueo** durante un período de tiempo determinado.
- ```HAL_Delay```, emula el retraso a través de una [espera ocupada](https://en.wikipedia.org/wiki/Busy_waiting).

Por lo tanto, si en el siguiente código usara HAL_Delay en las líneas 2 y 3, la tarea resultante será persistente, ya que no cede el control del procesador a menos que una tarea de mayor prioridad o una interrupción tome el control. En este ejemplo, solo puede ocurrir el segundo caso. Por otro lado, si usara vTaskDelay en las líneas 2 y 3. Entonces, la tarea se vuelve periódica ya que la tarea cede el control cada vez que llama a vTaskDelay. Sin embargo, no hay garantías de que la duración del retraso sea constante. Consulte la documentación de vTaskDelay para obtener más detalles.
 

```C
void persistentTaskHook(void const * argument)
{
	/**
	 *  Alternativas para vTaskDelay incluyen: HAL_Delay 
	 *  y osDelay.
	 **/
	for(;;)
	{
		/* 1. Intercala el LED verde a ON */
		HAL_GPIO_WritePin(IGreen_GPIO_Port, IGreen_Pin, GPIO_PIN_SET);

		/* 2. Demora/retrasa la execución por 2 segundos */
		vTaskDelay(pdMS_TO_TICKS(2000));

		/* 3. Intercala el LED verde a OFF */
		HAL_GPIO_WritePin(IGreen_GPIO_Port, IGreen_Pin, GPIO_PIN_RESET);

		/* 4. Demora/retrasa la execución por 2 segundos */
		HAL_Delay(2000);
	}
}
```

Vea [Task mgmt persistent](Task_mgmt_persistent.pdf) para parámetros de configuración del proyecto. Vea [README_ESP](README.md) documentación en ingles.  