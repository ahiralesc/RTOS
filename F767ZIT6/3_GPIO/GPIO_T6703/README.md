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
- [specification](https://github.com/ahiralesc/RTOS/blob/main/F767ZIT6/3_GPIO/GPIO_T6703/GPIO_T6703.pdf) for CubeMX settup. 
- And ```src/main.c``` for solution.


![Trace 1](img/trace1.png "Fig 1. Task timing constraints")
Fig. 1 Illustrates ToggleGreenTask and ToggleRedTask periods are not satisfied. Can you spot where are the errors? 


