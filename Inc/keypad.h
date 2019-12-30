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

//key defs
#define KEY_NUM					6
#define KEY_UP					0
#define KEY_UP_PIN				LCD_D0_KEY_UP_GPIO_Port,LCD_D0_KEY_UP_Pin
#define KEY_ENTER				1
#define KEY_ENTER_PIN			LCD_D4_KEY_OK_GPIO_Port,LCD_D4_KEY_OK_Pin
#define KEY_CANCEL				2
#define KEY_CANCEL_PIN			LCD_D5_KEY_CANCEL_GPIO_Port,LCD_D5_KEY_CANCEL_Pin
#define KEY_LEFT				3
#define KEY_LEFT_PIN			LCD_D2_KEY_LEFT_GPIO_Port,LCD_D2_KEY_LEFT_Pin
#define KEY_RIGHT				4
#define KEY_RIGHT_PIN			LCD_D3_KEY_RIGHT_GPIO_Port,LCD_D3_KEY_RIGHT_Pin
#define KEY_DOWN				5
#define KEY_DOWN_PIN			LCD_D1_KEY_DOWN_GPIO_Port,LCD_D1_KEY_DOWN_Pin

#define BUTTON_COUNT			6
#define DEBOUNCE_DELAY			25
#define BUTTON_HOLD_DELAY		1000

#define BUTTON_PRESSED			(1<<0)
#define BUTTON_RELEASED			(1<<1)
#define BUTTON_HELD				(1<<2)
#define BUTTON_HELD_RELEASED	(1<<3)

//global public vars
extern uint8_t Keypad_ButtonState[BUTTON_COUNT];

//public functions
void Keypad_Init(void);
void Keypad_CheckButtons(void);
