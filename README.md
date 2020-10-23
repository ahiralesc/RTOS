# Real Time Operating Systems (RTOS)

This repository seeks to integrate theoretical and technical knowledge regarding regarding how to build multitasking and real-time applications for reduced capacity architectures. Specifically, ARM based. Examples incrementally explore concurrency problems, inter-task communication, and other topics of interest. The code was developed for the [STM32F767ZI](https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f7-series/stm32f7x7/stm32f767zi.html) and the [STM32F042K6T6](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-f042k6.html) boards.

The code was developed using the STM32Cude and STM32CubeIDE. Projects are named in accordance to the following naming convention: ```number``` + ```board type``` + ```CMSIS version``` + ```small description of the problem being addressed```. i.e.: ```E1F767ZIV1_Task_creation```. Project documentation and source code was produces with the [STM32Cube](https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32cube.html) and [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) toolchain. For UART serial communication I use the [Hercules](https://www.hw-group.com/software/hercules-setup-utility) utility. The STM32CubeIDE was equiped with both [Segger J-link](https://www.segger.com/downloads/jlink/) and the [ST-Link/V2](https://www.st.com/en/development-tools/stsw-link009.html) debug probes. All projects run the [FreeRTOS](https://www.freertos.org/index.html) operating system.

Many of the examples are for educational purposes. These are based on Jim Cooling book on Real-Time Operating Systems.Some of the projects require additional hardware in order to observe the task behavior. Thus, you might need to build the following circuit.

<img src="img/F767ZI_LED.svg" title="GPIO pinout config for the F767ZI Board" width="400" height="400" />

Pins are connected as described in Table I. I listed the pin layouts  the projects use for three different boards. If you have a different board layout, you must revise its specification and procure the circuit is properly connected to your board. 

<style type="text/css">
.tg  {border-collapse:collapse;border-spacing:0;border-color:#999;}
.tg td{font-family:Arial, sans-serif;font-size:14px;padding:10px 5px;border-style:solid;border-width:0px;overflow:hidden;word-break:normal;border-color:#999;color:#444;background-color:#F7FDFA;}
.tg th{font-family:Arial, sans-serif;font-size:14px;font-weight:normal;padding:10px 5px;border-style:solid;border-width:0px;overflow:hidden;word-break:normal;border-color:#999;color:#fff;background-color:#26ADE4;}
.tg .tg-c3ow{border-color:inherit;text-align:center;vertical-align:top}
.tg .tg-fymr{font-weight:bold;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-0pky{border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-7btt{font-weight:bold;border-color:inherit;text-align:center;vertical-align:top}
</style>
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

