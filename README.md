# Distance Measuring with a Light Sensor

<img align="middle" src="/img/PROJECT_LOGO.jpg" alt="project logo" title="logo">

This project intends to implement a light sensor to measure large distances that have as one of their end points an unreachable place for human standards, like depth of a sea, the top of a skyscraper measured from the ground or a cliffhanger. The measurements will be processed using an STM32 microprocessor and sent to the cloud via aan ADAFRUIT Wifi 802.11b communication module. 

The development chain will be a STM32Cube and a STM32CubeIDE. For UART serial communication, the Hercules or MobaXterm terminals will be used. The STM32 will be flashed with the Segger J-link debugger probe. The STM will use FreeRTOS to manage the sensor and to interpret the data collected.

**Collaborators:**
* Ivan Sanchez
* Alex Machado
* Daniel Garcia
