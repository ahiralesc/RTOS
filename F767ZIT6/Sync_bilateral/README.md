### Synchronization: bilateral 

The example synchronizes the execution of two tasks using two binary semaphores. A semaphore is used by a task to signal to the other that it can proceed with its execution. When the application starts, both semaphores are acquired in order to block both tasks prior to their execution. 

```C
osSemaphoreAcquire(signalBlueSemHandle, portMAX_DELAY);
osSemaphoreAcquire(signalRedSemHandle, portMAX_DELAY);
```

Suppose the red task starts execution. It will immediatly block since the semaphore will not be available. This will result in a relinquishment of control and the selection of the blue task.

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

When the blue task starts execution, it releases the red task semaphore and yields control since it blocks on its semaphore. At a later time instance, the red task unblocks, performs its work, and unlocks the blue task semaphore. This process continues in lock-step sequence resulting in a sincronized execution of both tasks.


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

See synchronization [bilateral](Sync_bilateral.pdf) for proyect setup. See [README_ESP](README_ESP.md) for spanish translation.  