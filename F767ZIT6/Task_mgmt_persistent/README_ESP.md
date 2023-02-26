### Gestión de tareas: tarea persistente

El siguiente ejemplo ilustra cómo crear una sola tarea persistente. Tal alterna un LED (ENCENDIDO/APAGADO) y aplica dos retrasos de longitud finita. Un retraso, también referenciado como demora, crea la ilusión de que la tarea es periódica. Existen al menos dos estrategias para crear una demora:

- **[Espera ocupada](https://en.wikipedia.org/wiki/Busy_waiting)**,  itera un lazo un número finito de veces. Sin embargo, no realiza trabajo util y desperdicia tiempo valioso de procesamiento. ```HAL_Delay``` cae dentro de esta categoria.
- **Cede voluntario**, registra en el núcleo del sistema operativo la instancia de tiempo en que la tarea **desea** ser reactivada y cede control colocando la tarea en estado bloqueado. En una máquina con interrupciones y tareas con mayor prioridad ```vTaskDelay``` no ofrece garantías de que la tarea sea reactivada en el tiempo deseado. Consulte la documentación de vTaskDelay para obtener más detalles.

Por lo tanto, si en el siguiente código se usara HAL_Delay en las líneas 2 y 3, la tarea resultante será persistente, ya que no cedera el control del procesador a menos de que exista alguna tarea o interrupción de mayor prioridad. Por otro lado, si usara vTaskDelay en las líneas 2 y 3. Entonces, la tarea se vuelve periódica ya que la tarea cede el control cada vez que llama a vTaskDelay.
 

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