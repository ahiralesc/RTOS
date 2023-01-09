# Real Time Operating Systems (RTOS)

This repository integrates theoretical and technical knowledge on how to build multitasking and real-time applications for STM32 and EPSarchitectures. The examples incrementally explore concurrency problems such as task creation, critical sections, inter-task communication, queueing, and many topics of interest. The examples were coded on the following boards:
- STM32 [F767ZI](https://www.st.com/en/microcontrollers-microprocessors/stm32f767zi.html)
- STM32 [F413ZHT6](https://www.st.com/en/microcontrollers-microprocessors/stm32f413zh.html)
 
[STM32Cube](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube.html) and [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) toolchain is used to produce FreeRTOS code. Serial communication is done via UART/USART. We use [Hercules](https://www.hw-group.com/software/hercules-setup-utility) or [MobaXterm](https://mobaxterm.mobatek.net/) terminals to debug and illustrate asynchronous communication. 

Communication between the PC and STM32 modules is done via [Segger J-link](https://www.segger.com/downloads/jlink/) and [ST-Link/V2](https://www.st.com/en/development-tools/stsw-link009.html).

Projects are labeled as follows: ```board type``` + ```CMSIS version``` + ```small description of the problem being addressed```. i.e.: ```F767ZIV1_Task_creation```. Many of the examples are for educational purposes. These are based on Jim Cooling book on Real-Time Operating Systems and Richar Barry book technical manual on mastering the FreeRTOS Real Time Kernel. Some of the projects require additional hardware. 
