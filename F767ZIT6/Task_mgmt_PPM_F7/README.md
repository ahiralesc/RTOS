# 

The aim of the task management projects is to highlight the impact of poor task management practices on operating system performance. These projects serve as illustrative examples, demonstrating how improper task management can lead to the degradation of system performance. By studying these mini projects, learners can gain insights into the importance of effective task management in maintaining optimal system efficiency and responsiveness. Mini-projects include:

|Sequence|Project| Description|
|:--:|:------|:------|
|I.1|[Persistent task](https://github.com/ahiralesc/RTOS/tree/main/F767ZIT6/Task_mgmt_persistent)|shows how to create a persistent task in RTOS|
|I.2|[Periodic task](https://github.com/ahiralesc/RTOS/tree/main/F767ZIT6/Task_mgmt_periodic)|shows how to create a periodic task in RTOS|
|I.3|[Task frequency  regulation](https://github.com/ahiralesc/RTOS/tree/main/F767ZIT6/Task_mgmt_frequency_regulation)|shows how to regulate the task frequency of a periodic task| 

## II. Multitasking and synchronization

In a multiprogramming operating system, only one task executes at a time, while all other tasks are forced to wait for the processor to become available. In contrast, a multiprocessing operating system can handle the execution of one or more tasks simultaneously. When the execution context is preemptive, tasks may be interleaved, but this can lead to unexpected behavior in certain situations. This is particularly evident when tasks share, compete for resources, or require coordination to achieve specific goals. The subsequent examples serve to illustrate these problems and provide effective solutions for addressing them.

|Sequence|Project| Description|
|:--:|:------|:------|
|II.1|[Non-reentrant protection](https://github.com/ahiralesc/RTOS/tree/main/F767ZIT6/Sync_contention)|Shows how a non-reentrant function does not provide protection of a critical region|