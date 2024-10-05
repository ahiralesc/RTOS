### Gestión de tareas: tarea periodica

El siguiente ejemplo ilustra cómo crear una tarea periodica. Tal alterna un LED (ENCENDIDO/APAGADO) y aplica dos retrasos de longitud finita. Un retraso, también referenciado como demora, crea la ilusión de que la tarea es periódica. ```vTaskDelayUntil``` registra en el núcleo del sistema operativo el ciclo en que la tarea **debe** iniciar ejecución.
 

```C
void periodicTaskHook(void const * argument)
{
	/* 1. Número de ciclos que la tarea permanecerá en estado bloqueado  */
	TickType_t tickCount;

	/* 2. Longitud del periodo que la tarea permanece bloqueada (2s) */
	TickType_t frequency = pdMS_TO_TICKS(2000);

	/* 3. Obtiene el número de ciclos actual */
	tickCount = xTaskGetTickCount();

	for(;;)
	{
			/* 4. Emula trabajo prendiendo el LED interno azul */
			HAL_GPIO_WritePin(IBlue_GPIO_Port, IBlue_Pin, GPIO_PIN_SET);

			/*
			 * 5. Demora la ejecución de la tarea por el periodo solicitado. Esto hará que la tarea
			 * transicione a estado bloqueado. 
			 */
			vTaskDelayUntil( &tickCount, frequency );

			/* 6. Emula trabajo apagando el LED interno azul */
			HAL_GPIO_WritePin(IBlue_GPIO_Port, IBlue_Pin, GPIO_PIN_RESET);

			/* 7. Demora nuevamente la ejecución de la tarea por 2s */
			vTaskDelayUntil( &tickCount, frequency );
		}
}
```

Vea [Task mgmt periodic](Task_mgmt_periodic.pdf) para parámetros de configuración del proyecto. Vea [README_ESP](README.md) documentación en ingles.  