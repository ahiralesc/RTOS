# Real Time Operating Systems (RTOS)

[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fahcrtos%2Fhit-counter&count_bg=%23041CEF&title_bg=%23000003&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)


This repository integrates theoretical insights and technical expertise to facilitate the development of multitasking, real-time embedded systems on ARM-based platforms. It targets [ST](https://www.st.com/content/st_com/en.html) STM32 patforms such as [F767ZI](https://www.st.com/en/microcontrollers-microprocessors/stm32f767zi.html) and [F413ZHT6](https://www.st.com/en/microcontrollers-microprocessors/stm32f413zh.html) leveraging [STM32Cube](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube.html) and [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) to generate code templates.

Communication between the workstation and the ST development node, we utilize Segger [J-link](https://www.segger.com/downloads/jlink/) and [ST-Link/V2](https://www.st.com/en/development-tools/stsw-link009.html) interfaces. Further, serial communication via UART/USART is done using [Hercules](https://www.hw-group.com/software/hercules-setup-utility) and/or [MobaXterm](https://mobaxterm.mobatek.net/) terminals. 


In preemptive execution contexts, addressing challenges like concurrency, shared resource management, interrupt handling, task synchronization, and inter-task communication is essential. This repository serves as a comprehensive resource, providing examples that cover many of these complex scenarios. Multitasking applications are based on [FreeRTOS](https://www.freertos.org/).  


A [challenge-based learning](https://www.challengebasedlearning.org/) approach is applied to develop competencies. Topics include creating various types of tasks, accessing critical regions, synchronizing processes, and instrumenting input devices, among others.


Many of the solutions presented here were developed by Computer Science and Cybernetic Electronics students as part of our Operating Systems course. I appreciate their efforts, and where applicable, students’ contributions are expressly acknowledged. Great job, everyone!. 

Documentation is gradually being updated. Enjoy exploring the project! Please don’t forget to give us a star and cite this repository if you use the code in your projects. Thank you! 