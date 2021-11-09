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

//defs
#define LED_1_RED		0
#define LED_1_GREEN		1
#define LED_2_RED		2
#define LED_2_GREEN		3
#define LED_3_RED		4
#define LED_3_GREEN		5
#define LED_4_RED		6
#define LED_4_GREEN		7

//public vars

//public functions
void LEDs_Init(void);
void LEDs_Set(uint8_t LED, uint8_t Value);
void LEDs_LCDBacklightSet(uint8_t Value);
void LEDs_KeypadBacklightSet(uint8_t Value);
