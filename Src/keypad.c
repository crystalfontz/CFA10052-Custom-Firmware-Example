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
#include "keypad.h"

//public var holds state
uint8_t Keypad_ButtonState[BUTTON_COUNT];

/////////////////////////////////////////////////////////////////////////////////////////////////

void Keypad_Init(void)
{
	uint8_t i;
	for (i = 0; i < BUTTON_COUNT; i++)
		Keypad_ButtonState[i] = 0;
}

static void Keypad_PinsInit(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = LCD_D0_KEY_UP_Pin|LCD_D1_KEY_DOWN_Pin|LCD_D2_KEY_LEFT_Pin|LCD_D3_KEY_RIGHT_Pin
						  |LCD_D4_KEY_OK_Pin|LCD_D5_KEY_CANCEL_Pin|LCD_D6_Pin|LCD_D7_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Keypad_CheckButtons(void)
{
	uint8_t Read = 0;
	uint8_t i;

	//set pin modes for read
	Keypad_PinsInit();

	//read all buttons
	Read |= (LL_GPIO_IsInputPinSet(KEY_UP_PIN) == GPIO_PIN_RESET) ? (1 << KEY_UP) : 0;
	Read |= (LL_GPIO_IsInputPinSet(KEY_DOWN_PIN) == GPIO_PIN_RESET) ? (1 << KEY_DOWN) : 0;
	Read |= (LL_GPIO_IsInputPinSet(KEY_LEFT_PIN) == GPIO_PIN_RESET) ? (1 << KEY_LEFT) : 0;
	Read |= (LL_GPIO_IsInputPinSet(KEY_RIGHT_PIN) == GPIO_PIN_RESET) ? (1 << KEY_RIGHT) : 0;
	Read |= (LL_GPIO_IsInputPinSet(KEY_CANCEL_PIN) == GPIO_PIN_RESET) ? (1 << KEY_CANCEL) : 0;
	Read |= (LL_GPIO_IsInputPinSet(KEY_ENTER_PIN) == GPIO_PIN_RESET) ? (1 << KEY_ENTER) : 0;

	//debounce
	static uint32_t DebounceTimer = 0;
	static uint8_t LastRead = 0;
	if (!DebounceTimer && (Read != LastRead))
		DebounceTimer = HAL_GetTick();
	if (DebounceTimer != 0)
	{
		uint32_t Diff = HAL_GetTick() - DebounceTimer;
		if (Diff > DEBOUNCE_DELAY)
		{
			LastRead = Read;
			DebounceTimer = 0;
		}
	}

	//check states
	static uint32_t ButtonTimer[BUTTON_COUNT] = {0,};
	for (i = 0; i < BUTTON_COUNT; i++)
	{
		//clear released flags
		Keypad_ButtonState[i] &= ~BUTTON_RELEASED;
		Keypad_ButtonState[i] &= ~BUTTON_HELD_RELEASED;

		//check state
		uint8_t State = (LastRead >> i) & 1;
		if (State == 1)
		{
			//button pressed
			if (Keypad_ButtonState[i] & BUTTON_HELD)
				//button is still held, nothing to do
				continue;
			//button flag is not held
			if (ButtonTimer[i] != 0)
			{
				//timer has already been started
				if ((HAL_GetTick() - ButtonTimer[i]) > BUTTON_HOLD_DELAY)
				{
					//has elapsed, flag as held
					Keypad_ButtonState[i] &= ~BUTTON_PRESSED;
					Keypad_ButtonState[i] |= BUTTON_HELD;
					ButtonTimer[i] = 0;
					//nothing else to do
					continue;
				}
			}
			else
				//start the hold timer
				ButtonTimer[i] = HAL_GetTick();
			//is pressed
			Keypad_ButtonState[i] |= BUTTON_PRESSED;
		}
		else
		{
			//button not pressed
			ButtonTimer[i] = 0;
			if (Keypad_ButtonState[i] & BUTTON_HELD)
			{
				//not held anymore
				Keypad_ButtonState[i] &= ~BUTTON_HELD;
				Keypad_ButtonState[i] &= ~BUTTON_PRESSED;
				Keypad_ButtonState[i] |= BUTTON_HELD_RELEASED;
				continue;
			}
			if (Keypad_ButtonState[i] & BUTTON_PRESSED)
			{
				//not pressed
				Keypad_ButtonState[i] &= ~BUTTON_HELD;
				Keypad_ButtonState[i] &= ~BUTTON_PRESSED;
				Keypad_ButtonState[i] |= BUTTON_RELEASED;
			}
		}
	}
}
