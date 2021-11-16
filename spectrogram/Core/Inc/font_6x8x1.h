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
#ifndef __6X8X1_F0NT_H_
#define __6X8X1_F0NT_H_

#include "main.h"

#define FONT_WIDTH				6
#define FONT_HEIGHT				8
#define FONT_BASE				16
#define FONT_CHARNUM			240
#define BLOCK_CURSOR            0
#define UNDERLINE_CURSOR        1

// 8 bits across of which the right-most 6 are used each byte is the row, from left to right
extern const unsigned char Font_6X8_Cursors[2][FONT_HEIGHT];
extern const unsigned char Font_6X8[FONT_CHARNUM][FONT_HEIGHT];

#endif
