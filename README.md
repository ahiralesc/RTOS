# Real Time Operating Systems (RTOS)

This repository integrates theoretical and technical knowledge on how to build multitasking and real-time applications for STM32 architectures. The examples incrementally explore concurrency problems such as task creation, critical sections, inter-task communication, and many topics of interest. The examples were coded on the [STM32F767ZI](https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f7-series/stm32f7x7/stm32f767zi.html) board. However, they should work on other STM32 base architectures. 

The development chain that I used was [STM32Cube](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube.html) and [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html). For UART serial communication, I have used both the [Hercules](https://www.hw-group.com/software/hercules-setup-utility) or [MobaXterm](https://mobaxterm.mobatek.net/) terminals. Projects were flash with the [Segger J-link](https://www.segger.com/downloads/jlink/) debugger probe. An alternative probe that is often recommended  is [ST-Link/V2](https://www.st.com/en/development-tools/stsw-link009.html). Projects use the [FreeRTOS](https://www.freertos.org/index.html) operating system.


Projects are labeled as follows: ```board type``` + ```CMSIS version``` + ```small description of the problem being addressed```. i.e.: ```F767ZIV1_Task_creation```. Many of the examples are for educational purposes. These are based on Jim Cooling book on Real-Time Operating Systems and Richar Barry book technical manual on mastering the FreeRTOS Real Time Kernel. Some of the projects require additional hardware in order to observe the task behavior. Thus, you might need to build the following circuit.

<img src="img/F767ZI_LED.svg" title="GPIO pinout config for the F767ZI Board" width="400" height="400" />

Pins are connected as described in Table I. I listed the pin layouts for different board architectures. If you have a different board layout, you must proper connect your board to the circuit as specified in the board schematics.  


<table class="tg">
  <caption>Table I. Microcontroller pinout configuration</caption>
  <tr>
    <th class="tg-c3ow">Board</th>
    <th class="tg-c3ow">STM32 Pin</th>
    <th class="tg-c3ow">Board Pin</th>
    <th class="tg-c3ow">Label</th>
    <th class="tg-c3ow">Class</th>
  </tr>
  <tr>
    <td class="tg-fymr"><b>F767ZI</b></td>
    <td class="tg-c3ow">PA3</td>
    <td class="tg-c3ow">A0</td>
    <td class="tg-c3ow">EGreen<br></td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PC0</td>
    <td class="tg-c3ow">A1</td>
    <td class="tg-c3ow">EBlue</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PC3</td>
    <td class="tg-c3ow">A2</td>
    <td class="tg-c3ow">ERed</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PF3</td>
    <td class="tg-c3ow">A3</td>
    <td class="tg-c3ow">EYellow</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
 <tr>
    <td class="tg-fymr"><b>F042K6</b></td>
    <td class="tg-c3ow">PB3</td>
    <td class="tg-c3ow">D13</td>
    <td class="tg-c3ow">Green<br></td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PA12</td>
    <td class="tg-c3ow">D2</td>
    <td class="tg-c3ow">Blue</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PB7</td>
    <td class="tg-c3ow">D4</td>
    <td class="tg-c3ow">Red</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PF0</td>
    <td class="tg-c3ow">D7</td>
    <td class="tg-c3ow">Yellow</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
 <tr>
    <td class="tg-fymr"><b>L432KC</b></td>
    <td class="tg-c3ow">PB3</td>
    <td class="tg-c3ow">D13</td>
    <td class="tg-c3ow">Green<br></td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PA12</td>
    <td class="tg-c3ow">D2</td>
    <td class="tg-c3ow">Blue</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PB7</td>
    <td class="tg-c3ow">D4</td>
    <td class="tg-c3ow">Red</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
  <tr>
    <td class="tg-c3ow"></td>
    <td class="tg-c3ow">PC14</td>
    <td class="tg-c3ow">D7</td>
    <td class="tg-c3ow">Yellow</td>
    <td class="tg-c3ow">GPIO_Output<br></td>
  </tr>
</table>

Following [Graham et. al. notation](https://en.wikipedia.org/wiki/Notation_for_theoretic_scheduling_problems) for theoretic scheduling problems. Most of the projects are classified as 1|r_j, w_j, pmnt with 1 processing unit, r_j tasks released times are known, w_j tasks priorities are given, and pmnt preemtions are allowed. In each project folder comes with a PDF file that defines the hardware and operating system setup. The following table describes the problem model that each project addressed. 

|Project label|Problem model|
|:---|:---|
|F767ZIV1_Persistent_task|A single persistent toggles an LED at 0.5Hz. Premptions are allowed.|
|F767ZIV1_Periodic_task|A single periodic task toggles an LED at 0.5Hz. The task is periodically executed each 2s. Premptions are allowed.|
|F767ZIV1_PPM|Two periodic tasks execute. The firt executes for 4s and toggles an LED at 20Hz, the second task executes for 1s toggles an LED at 20Hz. Equal task priorities are considered. Priority preemptive is applied|
|F767ZIV1_Contention|Two periodic tasks execute. They have different release times and durations. Contention, when accesing a shared function, is illustrated by enabling a blue LED. Preemptions are allowed.|
|F767ZIV1_Disabling_interrupts|Contention problems observed in F767ZIV1_Contention are solved by disabling interrupts|
|F767ZIV1_Disabling_scheduler|Contention problems observed in F767ZIV1_Contention are solved by disabling the scheduler|

Contributers:

## Authors
- Gustavo Salazar - T025605 - Github: https://github.com/gustav0924}
- Rodolfo Macías - T029902 
