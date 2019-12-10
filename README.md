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
  
This example firmware, when compiled and programmed to a Crystalfontz CFA10052 module will:
  * Display on the LCD an alternating grid, with current backlights, LCD contrast and keypad status information.
  * Provides control of the backlights, and LCD contrast using the keypad.
  * Cycles the color of the four LEDs from red to green in sequence.
  * Enables the USART serial port on Header-1 pins 1 & 2 (115200 baud), and echoes any received data.
  * Enables the USB virtual serial port, and echoes any received data back to the host.
  
## Software & Hardware Requirements  
  * A [Crystalfontz CFA10052 (hardware v1.1 or later) Module (CFA735 / CFA835)](https://www.crystalfontz.com/product/cfa835tfk)
  * A PC (Windows/Linux/OSX) with [STM32 ST-LINK Utility](https://www.st.com/en/development-tools/stsw-link004.html) and [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) installed.
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

*Note: If you're having trouble with ST-LINK to CFA10052 communications, add 10K pull-up resistors to the SWD-IO and SWD-CLK lines.*

## Removing the CFA735/CFA835 Firmware
The CFA10052 is supplied by Crystalfontz with either the CFA735 or CFA835 firmware and a bootloader pre-installed. The flash
memory is also read & write protected. The protection must be removed, and flash erased before custom firmware can be loaded.  

**---- Important Note ----**    
**The Crystalfontz supplied CFA735/CFA835 firmware and bootloader is not open-source, and cannot be copied off of the CFA10052 by the user,
nor can it be programmed onto the CFA10052 by the user. Once a CFA735 or CFA835 is programmed with custom firmware, the module will loose all CFA735/CFA835 functionality.
To regain CFA735/CFA835 functionality, the CFA10052 module will need to be physically returned to Crystalfontz for reprogramming.**  
Please [email us](mailto:support@crystalfontz.com) if you need more information on this topic.  

To remove the CFA735/CFA835 firmware:
  * Disconnect the USB cable (or power supply) from the CFA10052 module.
  * Connect the CFA10052 to the ST-LINK using the programming cable (see above), and the ST-LINK to the host PC.
  * Hold the up & down keys on the CFA10052 while plugging the USB cable into the CFA10052 (or power supply).  
   The CFA10052 should now show the Crystalfontz Bootloader screen.
  * Run the [STM32 ST-LINK Utility](https://www.st.com/en/development-tools/stsw-link004.html).
  * In the "Target" menu, open the "Option Bytes" window.
  * In the "Read Out Protection" box, select "Level 0". Click Apply.
  * The Crystalfontz firmware has now been removed and any custom firmware may now be programmed.  
  
Alternative method (if you cannot enter the Crystalfontz Bootloader by holding keys):  
  * Disconnect the USB cable (or power supply) from the CFA10052 module.
  * Connect the BOOT0 test-point (a small pad on the back of the CFA10052 module, near the H1 connector) to 3.3V or 5V.
  * Connect the CFA10052 to the ST-LINK using the programming cable (see above), and the ST-LINK to the host PC.
  * Power on the CFA10052 (or connect it to USB power). The display should be blank.
  * Run the [STM32 ST-LINK Utility](https://www.st.com/en/development-tools/stsw-link004.html).
  * In the "Target" menu, open the "Option Bytes" window.
  * In the "Read Out Protection" box, select "Level 0". Click Apply.
  * The Crystalfontz firmware has now been removed and any custom firmware may now be programmed.  
   Connection of the BOOT0 pin to 3.3V/5V is no longer needed.
   
*Note: If you are purchasing new CFA10052 modules for custom firmware use, please ([contact us](mailto:support@crystalfontz.com)) to discuss firmware removal/programming options.*  

## STM32CubeIDE Software
The firmware in this example has been partly created by, and written in STMicroelectronics STM32CubeIDE.  
The STM32CubeIDE is a free to download & use integrated development environment based on Eclipse that
has been modified/extended by STMicroelectronics to include STM32 specific tools
(graphical device configuration tool, compiler & toolset, etc).  
You can read more about STM32CubeIDE, and download it on the 
[STM32CubeIDE Webpage](https://www.st.com/en/development-tools/stm32cubeide.html).    

To maintain the correct operation of the STM device configuration tool, you must only edit
the device configuration tool created source-code between the matching "USER CODE BEGIN xxx" and
"USER CODE END xxx" comment blocks.  

## Compiling & Loading Firmware Onto The CFA10052  
Once the Crystalfontz supplied CFA735/CFA835 firmware is removed (see section above), you may compile and load
this example custom firmware (or firmware of your own) onto the CFA10052.

To compile the firmware:
  * Open STM32CubeIDE.
  * In the File menu, choose Import, then "Import Existing Projects Into Workspace".
  * In the root directory box, select the directory of this example firmware. Click the Finish button.
  * In the Project Explorer, select the cfa10052_example project, then open the Src, and "main.c" file.
  * In the Project menu, select "Build Project".
  
To program and run the firmware on the CFA10052:
  * Disconnect the USB cable (or power supply) from the CFA10052 module.
  * Connect the CFA10052 to the ST-LINK using the programming cable (see above), and the ST-LINK to the host PC.
  * Connect the USB cable (or power supply) to the CFA10052.
  * Make sure the firmware project has been built (see steps above), and "Binaries" appears under "cfa10052_example" in the Project Explorer.
    If "Binaries" isn't visible, right-click the "cfa10052_example" project and select Refresh.
  * Select the Run menu, then "Debug Configurations".
  * In the debug target types selection box on the left, Right-Click "STM32 Cortex-M Application", and select "New Configuration".
  * A configuration window will be shown. The default settings are OK. Click the Apply then the Close button.
  * In the Run menu, select "Debug As", then "STM32 Cortex Application".  
    STM32CubeIDE should now connect to the ST-LINK, and upload and run the firmware on the CFA10052.
  
The above steps are only required on loading the project for the first time in STM32CubeIDE. After firmware source-code changes
have been made, only re-building the project (Ctrl-B shortcut) and programming the CFA10052 (F11 key shortcut) are needed.

Firmware can also be loaded via any of the normal STM32 bootloader methods (debugging is only availiable using the SWD interface
and a ST-LINK). For example, if using serial connection, USART1 may be used (RX=H1-Pin1 and TX=H1-Pin2).  
Form more detailed information about the STM32 bootloader and interfaces, [see the PDF here](https://www.st.com/resource/en/application_note/cd00167594.pdf).

## Licences
Crystalfontz supplied source-code is provided using The Unlicense.
A license with no conditions whatsoever which dedicates works to the public domain. Unlicensed works, modifications, and larger works may be distributed under different terms and without source code.  
See the UNLICENCE file, or (https://unlicense.org/) for details.

STM32CubeIDE created source-code and STMicroelectronics libraries are Copyright (c) 2019 STMicroelectronics.
All rights reserved. The software component is licensed by ST under BSD 3-Clause license, the "License";
You may not use these files except in compliance with the License. You may obtain a copy of the License at (http://opensource.org/licenses/BSD-3-Clause).
