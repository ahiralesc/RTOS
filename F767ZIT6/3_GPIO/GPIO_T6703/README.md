### Telaire 6703 CO2 Module

**Project description**

The objective of the project is to monitor CO2 via the Telaire 6703 (T6703) series $CO_2$ module. The unit is factory calibrated to measure $CO_2$ levels up to 5000 ppm. It uses Non Dispersive Infrared (NDIR) diffusion sampling and applies a logic self calibrated algorithm during sampling. See T6703 datasheet

The module warm-up time is of 2m. It updates the signal every 5s. The solution omits the 2m warm-up interval, you can just eliminate the first two minutes of data. A poll request is sent every 5s via USART. Each request is followed by a 0.5s delay which is intended to give time to T6703 to respond the request via an interrupt, also produced by USART. The response is written to a buffer which is then transmitted to another USART port. The output can be rendered in a MobaXterm terminal. 

  

**Components**
- STM32 Microcontroller. Set up using an RTOS (e.g., FreeRTOS).
- Four different color LEDs connected to GPIO pins of the STM32. 
- Two periodic Tasks. Each task controls the blinking of two LEDs.

**Task behavior**
*ToggleGreenTask* must execute periodically every 6s:
- At startup, it must toggle ON the blue LED. 
- During a period of 4s, it must toggle the green LED on/off at a frequency of 20Hz. 
- At completion, it must toggle OFF the blue LED. 

*ToggleRedTask* must execute periodically every 2s:
- At startup, it must toggle ON the yellow LED.
- During a period of 2s, it must toggle the red LED on/off at a frequency of 20Hz.
- At completion, it must toggle OFF the yellow LED. 

This solution uses **timers** to create keep tasks in execution before yielding control. See
- [specification](https://github.com/ahiralesc/RTOS/blob/main/F767ZIT6/2_Synchronization/Task_mgmt_PPM_Delay_F7/Task_mgmt_PPM_For_F7.pdf) for CubeMX settup. 
- And ```src/main.c``` for solution.


![Trace 1](img/trace1.png "Fig 1. Task timing constraints")
Fig. 1 Illustrates ToggleGreenTask and ToggleRedTask periods are not satisfied. Can you spot where are the errors? 


