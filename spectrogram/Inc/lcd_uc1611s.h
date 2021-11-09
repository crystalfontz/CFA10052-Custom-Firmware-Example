/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 *
 * Crystalfontz CFA10052 (hardware v1.1 onwards) example/base firmware.
 *
 * For more information about this CFA10052 example custom firmware package,
 * please see the README.md file.
 *
 ******************************************************************************
 *
 * Crystalfontz supplied source-code is provided using The Unlicense.
 * A license with no conditions whatsoever which dedicates works to the public
 * domain. Unlicensed works, modifications, and larger works may be distributed
 * under different terms and without source code.
 * See the UNLICENCE file, or https://unlicense.org/ for details.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#pragma once
#include "main.h"

#ifdef LCD_UC1611S

/////////////////////////////////////////////////////////

#define LCDCore_Init			UC1611S_Init
#define LCDCore_WriteContrast	UC1611S_WriteContrast
#define LCDCore_BufferToLCD		UC1611S_BufferToLCD
#define LCDCore_BusToWrite		UC1611S_BusToWrite
#define LCDCore_BusToRead		UC1611S_BusToRead

/////////////////////////////////////////////////////////

//contrast / lcd options
#define LCD_WIDTH				244
#define LCD_HEIGHT				68

#define LCD_CONTRAST_DEFAULT	80

/////////////////////////////////////////////////////////

//global lcd framebuffer
extern uint8_t LCD_FrameBuffer[LCD_WIDTH * LCD_HEIGHT];

// core functions
void UC1611S_Init(void);
void UC1611S_WriteContrast(uint16_t contrast);
void UC1611S_BufferToLCD(uint8_t * bufferMem);

void UC1611S_BusToWrite(void);
void UC1611S_BusToRead(void);

#endif
