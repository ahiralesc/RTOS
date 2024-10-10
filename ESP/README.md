# ESP32 and ESP8266

For those looking to connect an STM32 with an ESP communication module, this tutorial will guide you through the process.

### Introduction

[Espressif's](https://www.espressif.com/) ESP series features RISC MCU architectures that support dual-band Wi-Fi (2.4 to 5 GHz), Bluetooth, and IEEE 802.15.4 (including Zigbee 3.0 and Thread 1.3) connectivity. These architectures are 32-bit single-core processors. As of this writing, the latest ESP32-C5 module has a clock speed of 240 MHz. The modules include on-chip SRAM, support for PSRAM, and up to 320 KB of ROM. The number of GPIO ports varies depending on the specific module model. In this tutorial, I will outline the general procedure for setting up the [ESP32](http://docs.micropython.org/en/latest/esp32/quickref.html) and [ESP8266](http://docs.micropython.org/en/latest/esp8266/quickref.html) modules.

### Installing the firmware

Before coding, the ESP module must be flashed. Flashing erases any existing data in the main memory. After that, the appropriate firmware must be loaded into memory. In this tutorial, I will use MicroPython as the firmware. Please visit the [MicroPython](https://micropython.org/download/#esp8266) website to download the firmware suitable for your module. For this example, I am preparing the ESP32 module, so I downloaded the firmware file named ```esp32-20190720-v1.11-167-g331c224e0.bin```.

To install the firm ware do:
1. **Install the esptool**: ```bash pip install esptool```. For more information about the ESP8266, see [Getting Started with MicroPython](https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html#deploying-the-firmware).
 
2. **Erase the MCU**: ```esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash```

3. **Flash the firmware**: ```bash esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 esp32-20190720-v1.11-167-g331c224e0.bin```. 

Note. The ESP module can operate in two modes: bootloader mode and boot mode. For early ESP modules, you may need to **hold down** the `BOOT` button and then press the `EN` button to enter bootloader mode before flashing the firmware. 
 
### Connecting to the ESP module

Once done. You can connect to the module. There are several ways to do this, via:
1. [Picocom](https://linux.die.net/man/8/picocom), a minimalistic dumb-terminal emulator.
2. [MicroPython Kernel](https://github.com/goatchurchprime/jupyter_micropython_kernel/). For this option, see the [MicroPython on ESP Using Jupyter Notebook](https://towardsdatascience.com/micropython-on-esp-using-jupyter-6f366ff5ed9) tutorial.
3. [Mu Editor][https://codewith.mu/]. A simple python editor for coding with micropython.  

### Connecting via micropyho kernel 

First lest connect the notebook by doing 

```Python
%serialconnect to --port=/dev/ttyUSB0 --baud=115200
```


Have fun, yours truly: 
--Adan Hirales Carbajal