### UART Command Handler

This example demonstrates how to manage serial input commands via UART, illustrating how to separate UART communication from processing logic to make the system more modular. 

Tasks:
- Producer Task: This task reads incoming data from the UART (USART/Serial) and enqueues it into a queue. It is implemented in the HAL_UART_RxCpltCallback function, which handles the interrupt triggered by UART-PC communication.
- Consumer Task: This task reads data from the queue and processes it. In this example, the consumer task is ledConsumerHandler, which enables or disables LEDs based on the message type.

Asynchronous communication occurs both between the computer and the MCU, as well as between the producer and consumer tasks. The control message sent from the PC is an integer value, where:
- Integer 1 value indicates the red LED.
- Integer 2 value indicates the blue LED.
- Integer 3 value indicates the green LED.

The message queue (msgQueueHandle) stores incoming integer values by value. This approach has the advantage of avoiding race conditions between the producer and consumer tasks. However, the drawback is that the tasks are not synchronized, meaning the consumer task does not directly know when a new message has arrived. Since the consumer task runs persistently, this is not an issue—it continuously executes and the read operation on the queue is blocking.