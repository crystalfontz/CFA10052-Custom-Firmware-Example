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

//def's
#define FONT_WIDTH				6
#define FONT_HEIGHT				8
#define FONT_BASE				16
#define FONT_CHARNUM			240
#define BLOCK_CURSOR            0
#define UNDERLINE_CURSOR        1

//public functions
void Font_DrawCharacter(uint8_t *Buffer, uint8_t XPos, uint8_t YPos, uint8_t Character);
void Font_WriteString(uint8_t *Buffer, uint8_t XPos, uint8_t YPos, const char *Text);
