### UART Command Handler

#### Introduction

The solution is based on the **mediator pattern**. The mediator (coordinator) manages communication between various components (e.g., the producer and consumer) to reduce direct dependencies. It ensures that the producer and consumer do not interact with each other directly. This approach demonstrates how to manage serial input commands via UART, effectively separating UART communication from processing logic, thereby making the system more modular and promoting loose coupling.

The responsibilities of the components are as follows:
- **Producer task** (HAL_UART_RxCpltCallback): Reads incoming control messages from UART and enqueues them into a control message queue. Control messages are fixed-length, JSON-formatted strings. These messages are sent to the producer task via serial communication from the computer but can also be received through wireless communication. Since the producer task is an Interrup Service Routine (ISR) it must be quick and predictable to ensure real-time system responsiveness. 
- **Coordinator task** (coordinatorHandler): Dequeues control messages from the control message queue, validates their format, and forwards them to the appropriate consumer queue.
- **Consumer task** (solenoidControllerHandler): Dequeues control messages from its queue (solenoidQueueHandle), extracts and processes the message. The solution can support multiple consumer tasks, though only one consumer task is illustrated in this example. 

#### Use case

The application of this pattern is demonstrated through the construction of an irrigation system. Only one use case is discussed: As a user, I want to specify the location, frequency, and duration of irrigation to control the soil moisture for each plant in a selected area. An LED emulates a solenoid that serves as a control valve for water flow. Three LEDs are used to represent three locations in the garden. The control message follows this format:

```Python
{
	"id" :        XX, # The LED (solenoid) ID (two-digit integer).
	"frequency" : XX, # The irrigation frequency (two-digit integer).
	"duration"  : XX  # The irrigation duration in minutes (two-digit integer).
}
```

An example control message is {"id":01, "frequency":30, "duration":01}. The UART requires a buffer size of 39 bytes: 37 bytes for the JSON string, plus 1 byte for the "\x0" character, which is typically used to terminate the string. Since LEDs are used instead of solenoids, the toggling (flickering) frequency must range between 30 and 60 Hz. Beyond 60 Hz, the flickering typically appears as continuous, stable light to the human eye. In a drip irrigation system, water is supplied at much lower rates, usually measured in drops per second or liters per hour.

### Processing workflow

![Trace 1](img/Monitor.png "Fig 1. Monitor workflow")
Fig. 1 A collaborative diagram of the monitor pattern. 



The message queue (msgQueueHandle) stores incoming integer values by value. This approach helps avoid race conditions between the producer and consumer tasks. However, one drawback is that the tasks are not synchronized, meaning the consumer task doesn't immediately know when a new message has arrived.

Since the consumer task blocks on the read operation and runs persistently (continuously executing), this lack of synchronization doesn't cause functional issues. However, it comes at the cost of increased power consumption.