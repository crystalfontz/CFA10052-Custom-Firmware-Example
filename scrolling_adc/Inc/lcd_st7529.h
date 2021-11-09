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
#include "lcd_st7529_defs.h"

//def's
#define LCD_WIDTH				244
#define LCD_HEIGHT				68

#define LCD_CONTRAST_MIN		130
#define LCD_CONTRAST_MAX		350
#define LCD_OSC_FREQ_ADJ		7
#define LCD_BOOSTER_FREQ		0
#define LCD_BIAS_RATIO			1
#define LCD_CONTRAST_INIT		176

/////////////////////////////////////////////////////////

//public vars

//global lcd framebuffer
extern uint8_t LCD_FrameBuffer[LCD_WIDTH * LCD_HEIGHT];

//public functions
void ST7529_Init(void);
void ST7529_BusToWrite(void);
void ST7529_BusToRead(void);
void ST7529_WriteContrast(uint16_t Contrast);
void ST7529_BufferToLCD(uint8_t *Buffer);
void ST7529_ExtSet2(uint8_t LCDOscFreq, uint8_t LCDBoosterFreq, uint8_t LCDBiasRatio);
