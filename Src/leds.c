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
#include "leds.h"

void LEDs_Init(void)
{
	//setup PWM timer channels
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N);	//lcd & keypad backlight
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);		//led0r, led0g
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);		//led1r, led1g
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 |
		LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH4);								//led2r, led2g, led3r, led3g

	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableAllOutputs(TIM2);
	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_EnableAllOutputs(TIM4);


	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableCounter(TIM4);
}

void LEDs_LCDBacklightSet(uint8_t Value)
{
	LL_TIM_OC_SetCompareCH1(TIM1, Value);
}

void LEDs_KeypadBacklightSet(uint8_t Value)
{
	LL_TIM_OC_SetCompareCH2(TIM1, Value);
}

void LEDs_Set(uint8_t LED, uint8_t Value)
{
	switch (LED)
	{
		case LED_1_RED:
			LL_TIM_OC_SetCompareCH1(TIM2, Value);
			break;
		case LED_1_GREEN:
			LL_TIM_OC_SetCompareCH2(TIM2, Value);
			break;
		case LED_2_RED:
			LL_TIM_OC_SetCompareCH1(TIM3, Value);
			break;
		case LED_2_GREEN:
			LL_TIM_OC_SetCompareCH2(TIM3, Value);
			break;
		case LED_3_RED:
			LL_TIM_OC_SetCompareCH1(TIM4, Value);
			break;
		case LED_3_GREEN:
			LL_TIM_OC_SetCompareCH2(TIM4, Value);
			break;
		case LED_4_RED:
			LL_TIM_OC_SetCompareCH3(TIM4, Value);
			break;
		case LED_4_GREEN:
			LL_TIM_OC_SetCompareCH4(TIM4, Value);
			break;
		default: break;
	}
}
