### Sincronización: bilateral

El ejemplo sincroniza la ejecución de dos tareas utilizando dos semáforos binarios. Un semáforo es utilizado por una tarea para señalizar a la otra tarea que puede proceder con su ejecución. Al iniciar la aplicación ambos semáforos son adquiridos para bloquear ambas tareas previo su ejecución. 

```C
osSemaphoreAcquire(signalBlueSemHandle, portMAX_DELAY);
osSemaphoreAcquire(signalRedSemHandle, portMAX_DELAY);
```

Suponga la tarea roja inicia ejecución. Al intentar adquirir el semáforo la tarea quedará bloqueada, dado que el semáforo no está disponible. Esto resultará en un cede de control y la selección de la tarea azul.

```C
void toggleRedHook(void *argument)
{
	for(;;) {
		/* 1r. Either block or acquire the semaphore to enter the critical region */
		osSemaphoreAcquire(signalRedSemHandle, portMAX_DELAY);

		/* 2r. Use the critical section */
		toggle(IRed_GPIO_Port, IRed_Pin, (uint32_t) 100, 5000);

		/* 3r. The red task signals the blue task, thus un-blocking it */
		osSemaphoreRelease(signalBlueSemHandle);
	}
}
```

Cuando la tarea azul inicia ejecución. Tal libera el semáforo de la tarea roja y se bloquea en el semáforo azul. Esto causa nuevamente un cede de control y la selección de la tarea roja. La tarea roja queda desbloqueada, realiza trabajo, desbloquea a la tarea azul y nuevamente cede control. Este proceso continua de manera indefinida. 

```C
void toggleBlueHook(void *argument)
{
	for(;;) {
		/* 1b. Blue task signals the red task, thus un-bloking it */
		osSemaphoreRelease(signalRedSemHandle);

		/* 2b. Either block or acquire the semaphore to enter the critical region */
		osSemaphoreAcquire(signalBlueSemHandle, portMAX_DELAY);

		/* 3b. Use the critical region */
		toggle(IBlue_GPIO_Port, IBlue_Pin, (uint32_t) 100, 5000);
	}
}
```

Vea sincronización [bilateral](Sync_bilateral.pdf) para parámetros de configuración del proyecto. Vea [README](README.md) documentación en ingles.  