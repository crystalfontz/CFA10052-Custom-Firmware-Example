/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
 * STM32Cube created source-code and STMicroelectronics libraries are Copyright(c)
 * 2019 STMicroelectronics. All rights reserved.
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License.
 * You may obtain a copy of the License at: opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "keypad.h"
#include "leds.h"
#include "lcd_core.h"
#include "tprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//LCD Controller/Panel type must be set here!
// CFA10052 hardware version v1.0 to v1.5 = LCDCORE_ST7529
// CFA10052 hardware version v1.6 to v1.9 = LCDCORE_UC1611S
// CFA10052 hardware version v2.0+ = LCDCORE_UC1611S_INV

//#define LCD_TYPE		LCDCORE_ST7529
//#define LCD_TYPE		LCDCORE_UC1611S
//#define LCD_TYPE		LCDCORE_UC1611S_INV

#ifndef LCD_TYPE
# error LCD_TYPE NOT SET!!!
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t LCD_FrameBuffer[LCD_WIDTH * LCD_HEIGHT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void User_USART1_IRQHandler(void)
{
	//incoming data on USART1 (CFA10052 H1, pins 1 & 2)
	//called by USART1_IRQHandler() in stm32f4xx_it.c
	//echo data back to USART1
	if (LL_USART_IsActiveFlag_RXNE(USART1))
	{
		uint8_t Data = LL_USART_ReceiveData8(USART1);
		LL_USART_TransmitData8(USART1, Data);
		LL_USART_ClearFlag_RXNE(USART1);
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	//CFA10052 specific init's
	LL_SYSTICK_EnableIT();
	LEDs_Init();
	Keypad_Init();
	LCDCore_Init(LCD_TYPE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	char TempS[40];
	uint8_t LEDRotate = 0;
	uint8_t Contrast = 128;
	uint8_t Backlight = 40;
	uint8_t Keypad = 30;
	uint8_t c = 0;
	uint8_t Pressed[BUTTON_COUNT] =	{ 0, };
	while (1)
	{
		//keypad check
		Keypad_CheckButtons();

		//display / leds update (every ~500mS)
		if (c++ > 100)
		{
			//led rotate
			LEDs_Set(LED_1_RED, (LEDRotate == 0) ? 100 : 0);
			LEDs_Set(LED_1_GREEN, (LEDRotate == 0) ? 0 : 100);
			LEDs_Set(LED_2_RED, (LEDRotate == 1) ? 100 : 0);
			LEDs_Set(LED_2_GREEN, (LEDRotate == 1) ? 0 : 100);
			LEDs_Set(LED_3_RED, (LEDRotate == 2) ? 100 : 0);
			LEDs_Set(LED_3_GREEN, (LEDRotate == 2) ? 0 : 100);
			LEDs_Set(LED_4_RED, (LEDRotate == 3) ? 100 : 0);
			LEDs_Set(LED_4_GREEN, (LEDRotate == 3) ? 0 : 100);
			LEDRotate++;
			if (LEDRotate > 3)
				LEDRotate = 0;

			//handle button holding
			if (Keypad_ButtonState[KEY_UP] & BUTTON_HELD)
				Backlight += (Backlight > 97) ? 0 : 4;
			if (Keypad_ButtonState[KEY_DOWN] & BUTTON_HELD)
				Backlight -= (Backlight < 4) ? 0 : 4;

			if (Keypad_ButtonState[KEY_RIGHT] & BUTTON_HELD)
				Contrast += (Contrast > 0xFE) ? 0 : 2;
			if (Keypad_ButtonState[KEY_LEFT] & BUTTON_HELD)
				Contrast -= (Contrast < 2) ? 0 : 2;

			if (Keypad_ButtonState[KEY_ENTER] & BUTTON_HELD)
				Keypad += (Keypad > 97) ? 0 : 4;
			if (Keypad_ButtonState[KEY_CANCEL] & BUTTON_HELD)
				Keypad -= (Keypad < 4) ? 0 : 4;

			//update lcd
			uint16_t x, y;
			//blank the frame buffer
			memset(LCD_FrameBuffer, 0x00, LCD_WIDTH * LCD_HEIGHT);
			//draw alternating check pattern
			if (LEDRotate >> 1)
				for (x = 0; x < LCD_WIDTH / 2; x++)
					for (y = 0; y < LCD_HEIGHT; y++)
						LCD_FrameBuffer[(x * 2) + (y & 1) + (y * LCD_WIDTH)] = 0xFF;
			else
				for (x = 0; x < LCD_WIDTH / 2; x++)
					for (y = 0; y < LCD_HEIGHT; y++)
						LCD_FrameBuffer[(x * 2) + (1 - (y & 1)) + (y * LCD_WIDTH)] = 0xFF;
			//draw a box
			for (x = 0; x < LCD_WIDTH; x++)
			{
				LCD_FrameBuffer[x] = 0xFF;
				LCD_FrameBuffer[x + ((LCD_HEIGHT - 1) * LCD_WIDTH)] = 0xFF;
			}
			for (y = 0; y < LCD_HEIGHT; y++)
			{
				LCD_FrameBuffer[(LCD_WIDTH * y)] = 0xFF;
				LCD_FrameBuffer[(LCD_WIDTH - 1) + (LCD_WIDTH * y)] = 0xFF;
			}
			//blank out a text box
			for (x = 12; x < 105; x++)
				for (y = 8; y < 54; y++)
					LCD_FrameBuffer[x + (y * LCD_WIDTH)] = 0;
			//show info text
			tsprintf(TempS, "Backlight: %03d", Backlight);
			Font_WriteString(LCD_FrameBuffer, 10, 10, TempS);
			tsprintf(TempS, "Keypad BL: %03d", Keypad);
			Font_WriteString(LCD_FrameBuffer, 10, 22, TempS);
			tsprintf(TempS, "Contrast : %03d", Contrast);
			Font_WriteString(LCD_FrameBuffer, 10, 34, TempS);
			//show button status
			tsprintf(TempS, "Keypad Check: U%c D%c L%c R%c E%c X%c", (Pressed[KEY_UP] ? '\xBB' : '.'), (Pressed[KEY_DOWN] ? '\xBB' : '.'),
				(Pressed[KEY_LEFT] ? '\xBB' : '.'), (Pressed[KEY_RIGHT] ? '\xBB' : '.'), (Pressed[KEY_ENTER] ? '\xBB' : '.'),
				(Pressed[KEY_CANCEL] ? '\xBB' : '.'));
			Font_WriteString(LCD_FrameBuffer, 10, 34 + 12, TempS);
			//write framebuffer to lcd controller ic
			LCDCore_BufferToLCD(LCD_FrameBuffer);

			//reset loop counter
			c = 0;
		}

		//handle button presses
		if (Keypad_ButtonState[KEY_UP] & BUTTON_RELEASED)
		{
			Backlight += (Backlight > 99) ? 0 : 1;
			Pressed[KEY_UP] = 1;
		}
		if (Keypad_ButtonState[KEY_DOWN] & BUTTON_RELEASED)
		{
			Backlight -= (Backlight == 0) ? 0 : 1;
			Pressed[KEY_DOWN] = 1;
		}

		if (Keypad_ButtonState[KEY_RIGHT] & BUTTON_RELEASED)
		{
			Contrast += (Contrast > 0xFF) ? 0 : 1;
			Pressed[KEY_RIGHT] = 1;
		}
		if (Keypad_ButtonState[KEY_LEFT] & BUTTON_RELEASED)
		{
			Contrast -= (Contrast == 0) ? 0 : 1;
			Pressed[KEY_LEFT] = 1;
		}

		if (Keypad_ButtonState[KEY_ENTER] & BUTTON_RELEASED)
		{
			Keypad += (Keypad > 99) ? 0 : 1;
			Pressed[KEY_ENTER] = 1;
		}
		if (Keypad_ButtonState[KEY_CANCEL] & BUTTON_RELEASED)
		{
			Keypad -= (Keypad == 0) ? 0 : 1;
			Pressed[KEY_CANCEL] = 1;
		}

		//update contrst
		LCDCore_SetContrast(Contrast);
		//update lcd backlight brightness
		LEDs_LCDBacklightSet(Backlight);
		//update keypad backlight brightness
		LEDs_KeypadBacklightSet(Keypad);
		//loop delay
		LL_mDelay(5);            //5mS
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_4);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
	LL_Init1msTick(84000000);
	LL_SetSystemCoreClock(84000000);
	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
