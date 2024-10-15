### Telaire 6703 CO2 Module

**Project description**

The objective of the project is to monitor CO2 via the Telaire 6703 (T6703) series $CO_2$ module. The unit is factory calibrated to measure $CO_2$ levels up to 5000 ppm. It uses Non Dispersive Infrared (NDIR) diffusion sampling and applies a logic self calibrated algorithm during sampling. See T6703 datasheet

The module warm-up time is of 2m. It updates the signal every 5s. The solution omits the 2m warm-up interval, you can just eliminate the first two minutes of data. A poll request is sent every 5s via USART. Each request is followed by a 0.5s delay which is intended to give time to T6703 to respond the request via an interrupt, also produced by USART. The response is written to a buffer which is then transmitted to another USART port. The output can be rendered in a MobaXterm terminal. 

**Components**
- STM32 Microcontroller. A super-loop implementation is used. 
- One Telaire 6703 series $CO_2$ module.

**Task behavior**
*Super-loop (SL)*:
- SL enables the timer interrupt. 
- SL sequest data from T6713 by sending a cmd message. 
- SL delays 0.5s.
- Finally, it prepares a message with the CO2 measure and fowards it to USART. 

*TRxCpltCallback*:
- Receives the CO2 msg from T6713 via USART.
- Validates the msg.
- And allocates the received msg to a global variable. 

This solution uses a **timer (TIM6 in F7)** to create a delay for precise timing. See
- [Specification](https://github.com/ahiralesc/RTOS/blob/main/F767ZIT6/3_GPIO/GPIO_T6703/GPIO_T6703.pdf) for CubeMX settup. 
- And ```src/main.c``` for solution.


**Software solution**

1. A custom delay is built based on the timing characteristics of timer 6.

```C
void delay(uint16_t length) {
	/* TIM6 timer settings in STM32CubeMX are
	 * - HCLK: 16 MHz.
	 * - APB1: 16 MHz.
	 * - TIM6 Pre-scalar: 16-1 thus APB1 is scaled to 1 MHz
	 * - TIM6 Counter period: 65535-1. Thus, the maximum counter period is 65.535 ms.
	 *
	 * The maximum signal length of the DHT11 protocol in all phases (initialization, response,
	 * and data transmission) is 18 ms. Thus, a timer length of 65.535 ms is more than enough.
	 * The counter is increasing.
	 */
	__HAL_TIM_SET_COUNTER(&htim6, 0);				// Set the time to 0
	while((__HAL_TIM_GET_COUNTER(&htim6))<length);  // Increment the timer until the desired length.
}
```

2. Create a command message for the T6713 sensor

```C
static const uint8_t cmd[8] = {
		  0x15,	// slave address
		  0x04,	// read input register function
		  0x13,	// register address 5003 (MSB)
		  0x8B,	// register address 5003 (LSB)
		  0x00, // number of registers (MSB)
		  0x01,	// number of registers (LSB)
		  0x46,	// CRC (LSB)
		  0x70	// CRC (MSB)
};


static uint16_t cmd_length =
  		  (uint16_t)(sizeof(cmd)/sizeof(uint8_t));
```

3. Create two buffers, one for UART/USART communication between the board and PC and the second for communication with the T6713 sensor. 

```C
uint8_t rcv_buff[8];   // Used to receive the CO2 reading from USART from T6713.
uint8_t snd_buff[12];  // Used to send the CO2 reading to PC via USART.
uint16_t co2 = -1;     // This field is updated when the T6713 produces an interrupt.
```

4. On interrupt, ```HAL_UART_RxCpltCallback``` is called and the CO2 message is received from T6713. The CO2 message is validated and allocated to co2 variable. 

```C
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {

		/* Receive data from T6713 */
		HAL_UART_Receive_IT(&huart2, rcv_buff, 8);

		/* Validate data packet is not corrupt */
		if( rcv_buff[0] == 0x15 && rcv_buff[1] == 0x04 && rcv_buff[2] == 0x02){
			uint16_t new_co2 = (uint16_t)(rcv_buff[3] << 8 | rcv_buff[4]);
			co2 = new_co2;
		}
	}
}
```

5. Prior sending a poll request to T6713 the interrupt service routine corresponding to UART must be enabled. This two lines go before the main loop.

```C
 HAL_UART_Receive_IT(&huart2, rcv_buff, 8);
 HAL_TIM_Base_Start_IT(&htim6);
```

6. The following code sends the poll request to T6713, waits for the interrupt to occur, and prepares a message of the read value to the PC.

```C
 while (1)
  {
	  /* Request data from T6713 */
	  HAL_UART_Transmit(&huart2, cmd, cmd_length, 1000);

	  /* Delay so that HAL_UART_RxCpltCallback is called */
	  HAL_Delay(500);

	  /* Prepare a message packet */
	  sprintf(snd_buff,"CO2:%04d\r\n", co2);

	  /* Transmit the message via UART3 */
	  HAL_UART_Transmit(&huart3, snd_buff, strlen(snd_buff), 100);

	  HAL_Delay(5000);	// The module updates the signal every 5s
  }
```  