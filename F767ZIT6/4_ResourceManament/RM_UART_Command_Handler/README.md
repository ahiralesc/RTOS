### UART Command Handler

This example demonstrates how to manage serial input commands via UART, illustrating how to separate UART communication from processing logic to make the system more modular. 

Tasks:
- Producer Task: This task reads incoming data from the UART (USART/Serial) and enqueues it into a queue. It is implemented in the HAL_UART_RxCpltCallback function, which handles the interrupt triggered by UART-PC communication.
- Consumer Task: This task reads data from the queue and processes it. In this example, the consumer task is ledConsumerHandler, which enables or disables LEDs based on the message type.

Asynchronous communication occurs both between the computer and the MCU, as well as between the producer and consumer tasks. The control message sent from the PC has the following format:

```C
typedef struct{
	uint8_t id;         // The led id: 1 is red, 2 blue, and 3 green.
	uint8_t frequency;  // The frequency rate.
} LED_Ctrl_msg;
```

Humans can perceive the toggling (flickering) of an LED in the frequency range of approximately 30 to 60 Hz. Beyond 60 Hz, the flickering typically appears as continuous, stable light to the human eye.

It is important to note that parsing the control message and validating the frequency rate should not be performed in the interrupt service routine (HAL_UART_RxCpltCallback), as interrupt service routines (ISRs) must execute quickly and predictably. ISRs should be minimal and deterministic to ensure real-time system responsiveness.

The message queue (msgQueueHandle) stores incoming integer values by value. This approach helps avoid race conditions between the producer and consumer tasks. However, one drawback is that the tasks are not synchronized, meaning the consumer task doesn't immediately know when a new message has arrived.

Since the consumer task blocks on the read operation and runs persistently (continuously executing), this lack of synchronization doesn't cause functional issues. However, it comes at the cost of increased power consumption.