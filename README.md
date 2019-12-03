# Crystalfontz CFA10052 Custom Firmware Example  

**NOTE: this example source-code & information is for CFA10052 hardware version 1.1 and onwards.**  

## Introduction
The Crystalfontz CFA10052 is a versatile intelligent LCD module.
Both the Crystalfontz CFA735 and CFA835 products are based on the CFA10052 hardware module.
The CFA10052 (and so the CFA735 and CFA835) can be reprogrammed to run your own custom firmware. 

CFA10052 hardware features:
  * [STMicroelectronics STM32F401 microcontroller](https://www.st.com/en/microcontrollers-microprocessors/stm32f401rc.html)
  * ARM 32-bit Cortex™-M4 CPU @ 84 MHz
  * 256K Flash, 64K RAM
  * A 244 x 68 pixel backlit LCD display
  * [Sitronix ST7529](https://www.crystalfontz.com/controllers/Sitronix/ST7529/) 32 grayscale graphic LCD controller
  * Buck-boost switching supply allows wide supply voltage range
  * Separate switching LED brightness controllers for keypad and LCD backlights
  * 6 button backlit keypad
  * 4x bi-color (red/green) LEDs
  * USB2 interface
  * microSD card slot
  * Up to 20 general purpose IO pins (GPIO's)
  * Multiple serial/SPI/I2C/CAN interfaces (depending on GPIO use).
  
## Example Firmware
This firmware, when compiled and programmed to a Crystalfontz CFA10052 module, will:
  * Displays on the LCD an alternating grid, with current backlights, LCD contrast and keypad status information.
  * Gives control of the backlights, and LCD contrast using the keypad.
  * Changes the color of the four LEDs from red to green in sequence.
  * Enables the USART serial port on Header-1 pins 1 & 2 (115200 baud), and echoes any received data.
  * Enables the USB virtual serial port, and echoes any received data back to the host.
  
**---- Important Note ----**    
The Crystalfontz CFA735 and CFA835 products are a CFA10052 hardware module programmed with a bootloader and CFA735/CFA835 firmware.  
The bootloader and CFA735/CFA835 firmware is not open-source, and cannot be copied off of the CFA10052 by the user, nor can it be programmed onto the CFA10052 by the user.
Once a CFA735 or CFA835 is programmed with custom firmware, the module will loose all CFA735/CFA835 functionality.
For a CFA10052 module to be re-programmed with CFA735 or CFA835 firmware it will need to be physically returned to Crystalfontz.
Please [email us](mailto:support@crystalfontz.com) if you need more information on this topic.

## Software & Hardware Requirements  
  * A [Crystalfontz CFA10052 (hardware v1.1 or later) Module (CFA735 / CFA835)](https://www.crystalfontz.com/product/cfa835tfk)
  * A PC (Windows/Linux/OSX) with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) installed
  * A [STMicroelectronics ST-LINK (V2 or V3) programming interface](https://www.st.com/en/development-tools/st-link-v2.html)
  * A CFA10052 programming cable (details below)
  * If you are using Windows 7/8/8.1/10 (or equivalent Server versions) and would like to test the USB virtual serial port, you'll need to [download drivers from here](https://www.st.com/en/development-tools/stsw-stm32102.html]). The USB serial port will work without requiring additional drivers on Windows 10+, Linux, OS-X.
  
A simple CFA10052 to ST-LINK programming cable is required. This cable may be obtained from Crystalfontz ([email us](mailto:support@crystalfontz.com)), or you may make one yourself using the connection table below:
  
ST-LINK Pin | CFA10052 H1 Pin | Description
--- | --- | ----
1 | 16 | +5V
7 | 11 | SWD-IO
9 | 12 | SWD-CLK
15 | 14 | RESET
20 | 15 | GND

*Note: If you're having trouble with ST-LINK to CFA10052 communications, try adding 10K pull-up resistors to the SWD-IO and SWD-CLK lines.*

## STM32CubeIDE Software
The firmware in this example has been partly created by, and written in STMicroelectronics STM32CubeIDE.  
**It is recommended that you use the STM32CubeIDE to load and use this example firmware project.**  
The STM32CubeIDE is a free to download & use integrated development environment based on Eclipse that
has been modified/extended by STMicroelectronics to include STM32 specific tools
(graphical device configuration tool, GCC ARM compiler, etc).  
You can read more about STM32CubeIDE, and download it on the 
[STM32CubeIDE Webpage](https://www.st.com/en/development-tools/stm32cubeide.html).  

To maintain the correct operation of the STM device configuration tool, you must only edit
the device configuration tool created source-code between the matching "USER CODE BEGIN xxx" and "USER CODE END xxx" comment blocks.  

## Compiling & Loading Firmware Onto The CFA10052  
If the CFA10052 module is loaded with the Crystalfontz supplied CFA735 or CFA835 firmware you'll need
to follow a few extra steps to erase the CFA735/CFA835 firmware before you may install this example
firmware, or your own custom firmware.  
*See the "Important Note" in the "Example Firmware" section above before removing the CFA735/CFA835 firmware.*

To erase the CFA735/CFA835 firmware:
  * Disconnect (or remove power supply) from the CFA10052 module.
  * Connect the BOOT0 test-point (a small pad on the back of the CFA10052 module, near the H1 connector) to 3.3V or 5V.
  * Power on the CFA10052 (or connect it to USB power). The display should be blank.
  * Use the STM32CubeIDE to compile and load this example firmware onto the CFA10052.
  * Once the CFA735/CFA835 firmware has been erased/overwritten, connection of the BOOT0 pin to 3.3V/5V is no longer needed.

*Note: If you are purchasing new modules and wish to remove the CFA735/CFA835 firmware erasure steps above, please ([email us](mailto:support@crystalfontz.com)) to discuss options.*

Firmware can be loaded via any of the normal STM32 bootloader methods. Using a STMLink v2/v3 is
preferred as this will also allow live debugging. You'll need to make a JTAG/SWD debugging
cable to connect the STLink to the H1 header (see the table above).  
If using serial connection, USART1 may be used (RX=H1-Pin1 and TX=H1-Pin2).  
Form more detailed information about the STM32 bootloader, [see the PDF here](https://www.st.com/resource/en/application_note/cd00167594.pdf).

## Licences
Crystalfontz supplied source-code is provided using The Unlicense.
A license with no conditions whatsoever which dedicates works to the public domain. Unlicensed works, modifications, and larger works may be distributed under different terms and without source code.  
See the UNLICENCE file, or (https://unlicense.org/) for details.

STM32CubeIDE created source-code and STMicroelectronics libraries are Copyright (c) 2019 STMicroelectronics.
All rights reserved. The software component is licensed by ST under BSD 3-Clause license, the "License";
You may not use these files except in compliance with the License. You may obtain a copy of the License at (http://opensource.org/licenses/BSD-3-Clause).
