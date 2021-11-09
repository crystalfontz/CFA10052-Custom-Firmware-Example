/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * Crystalfontz CFA10052 (hardware v1.1 onwards) example/base firmware.
 *
 * This example displays a moving histogram graph of voltages acquired from
 * one of the two available ADC inputs on the CFA10052.
 *
 * The ADC input is Header 1 - Pin 5.
 * Due to the resistive divider on the CFA10052 ADC input pin, maximum input
 * voltage is 6.6V.
 *
 * The ADC is sampled at 21Khz, and averaged out over the length of one sample
 * period (one horizontal pixel on the LCD).
 *
 * Up/Down keys change the length of the sample period.
 * Left/Right keys move the voltage measurement cursor.
 * The X key starts/stops sampling & graph scrolling.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "keypad.h"
#include "leds.h"
#include "lcd_core.h"
#include "tprintf.h"
#include "math.h"
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
#define LCD_TYPE		LCDCORE_UC1611S_INV
#ifndef LCD_TYPE
# error LCD_TYPE NOT SET!!!
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SAMPLE_TIME			(250) /*mS*/
#define SAMPLE_TIME_MIN		(10)
#define SAMPLE_TIME_MAX		(60000)
#define SAMPLE_RECORDS		(LCD_WIDTH)
#define KEY_REPEAT_TIMER	(100) /*mS*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t LCD_FrameBuffer[LCD_WIDTH * LCD_HEIGHT];
volatile uint64_t ADCDataSum = 0;
volatile uint32_t ADCDataCount = 0;
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//ADC conversion complete callback (from ADC_IRQHandler() in stm32f4xx_it.c)
	//Runs at PCLK2/8/480 = 84000000/8/480 = 21875 KHz
	//ADC values are summed to create an average in the main loop
	ADCDataSum += HAL_ADC_GetValue(&hadc1);
	ADCDataCount++;
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
	HAL_Init();

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
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	//CFA10052 specific init's
	HAL_SuspendTick();
	LL_Init1msTick(8000000);
	HAL_ResumeTick();
	LL_SYSTICK_EnableIT();

	LEDs_Init();
	Keypad_Init();
	LCDCore_Init(LCD_TYPE);
	LEDs_LCDBacklightSet(50);
	LEDs_KeypadBacklightSet(50);

	//Start the ADC
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_IT(&hadc1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//averaged ADC sample storage
	//let's make use of that hardware float processor
	double		ADCSamples[SAMPLE_RECORDS];
	uint32_t	ADCSamplePos;

	//local vars
	uint32_t	i, j;
	char		TempS[40];
	uint32_t	SampleLength, SampleLengthPrev;
	uint32_t	SampleTimer, KeyTimer;
	uint32_t	XCursorPos;
	double		XCursorValue;
	uint8_t		DoLCDUpdate;


	//init local vars
	for (i = 0; i < SAMPLE_RECORDS; i++)
		ADCSamples[i] = -1.0f;
	ADCSamplePos = 0;
	DoLCDUpdate = 1;
	SampleLength = SAMPLE_TIME;
	XCursorPos = LCD_WIDTH / 2;
	XCursorValue = -1.0f;

	//init timers
	TIMER_SET_NEXT(SampleTimer, SampleLength)
	TIMER_SET_NEXT(KeyTimer, KEY_REPEAT_TIMER)

	//main loop
	while (1)
	{
		//keypad check
		Keypad_CheckButtons();

		//handle button presses / holds
		if (Keypad_ButtonState[KEY_RIGHT] & BUTTON_RELEASED)
		{
			//right key pressed, move cursor right
			XCursorPos += (XCursorPos >= LCD_WIDTH-1) ? 0 : 1;
			DoLCDUpdate = 1;
		}
		if (Keypad_ButtonState[KEY_LEFT] & BUTTON_RELEASED)
		{
			//left key pressed, move cursor left
			XCursorPos -= (XCursorPos == 0) ? 0 : 1;
			DoLCDUpdate = 1;
		}
		if (Keypad_ButtonState[KEY_UP] & BUTTON_RELEASED)
		{
			//up key pressed, increase sample period
			SampleLength += (SampleLength > SAMPLE_TIME_MAX-10) ? 0 : 10;
			DoLCDUpdate = 1;
		}
		if (Keypad_ButtonState[KEY_DOWN] & BUTTON_RELEASED)
		{
			//down key pressed, decrease sample period
			SampleLength -= (SampleLength < SAMPLE_TIME_MIN+10) ? 0 : 10;
			DoLCDUpdate = 1;
		}
		if (Keypad_ButtonState[KEY_CANCEL] & BUTTON_RELEASED)
		{
			//X key pressed, toggle pause
			if (SampleLength == 0)
				SampleLength = SampleLengthPrev;
			else
			{
				SampleLengthPrev = SampleLength;
				SampleLength = 0;
			}
			DoLCDUpdate = 1;
		}

		//key hold repeat timer
		IF_TIMER_EXPIRED(KeyTimer)
		{
			//reset timer
			TIMER_SET_NEXT(KeyTimer, KEY_REPEAT_TIMER)

			//handle key holds
			if (Keypad_ButtonState[KEY_RIGHT] & BUTTON_HELD)
			{
				//right key held, move cursor right
				XCursorPos += (XCursorPos >= LCD_WIDTH-1) ? 0 : 1;
				DoLCDUpdate = 1;
			}
			if (Keypad_ButtonState[KEY_LEFT] & BUTTON_HELD)
			{
				//left key held, move cursor left
				XCursorPos -= (XCursorPos == 0) ? 0 : 1;
				DoLCDUpdate = 1;
			}
			if (Keypad_ButtonState[KEY_UP] & BUTTON_HELD)
			{
				//up key held, increase sample period
				SampleLength += (SampleLength > SAMPLE_TIME_MAX-10) ? 0 : 10;
				DoLCDUpdate = 1;
			}
			if (Keypad_ButtonState[KEY_DOWN] & BUTTON_HELD)
			{
				//down key held, decrease sample period
				SampleLength -= (SampleLength < SAMPLE_TIME_MIN+10) ? 0 : 10;
				DoLCDUpdate = 1;
			}
		}

		//take averaged ADC value if not paused, and sample timer expired
		if (SampleLength != 0)
		{
			IF_TIMER_EXPIRED(SampleTimer)
			{
				//set next sample timer expiry
				TIMER_SET_NEXT(SampleTimer, SampleLength)

				//stop ADC interrupts
				HAL_NVIC_DisableIRQ(ADC_IRQn);
				//calculate ADC average value over period
				double ADCAvg = ADCDataSum / (double)ADCDataCount;
				//reset ADC sum vars
				ADCDataCount = 0;
				ADCDataSum = 0;
				//re-enable ADC interrupts
				HAL_NVIC_EnableIRQ(ADC_IRQn);
				//store the averaged ADC value
				ADCSamples[ADCSamplePos % SAMPLE_RECORDS] = ADCAvg;
				ADCSamplePos++;
				//do lcd update
				DoLCDUpdate = 1;
			}
		}

		//do LCD update if needed
		if (DoLCDUpdate == 1)
		{
			//LCD update needed

			//blank the lcd frame buffer
			memset(LCD_FrameBuffer, 0x00, LCD_WIDTH * LCD_HEIGHT);

			//display moving graph
			//LCD is 68 pixels high, ADC has max value of 4096 (=3.3V*2 in)
			for (i = 0; i < LCD_WIDTH; i++)
			{
				//draw right to left
				uint32_t XPos = (LCD_WIDTH-1-i);
				//get ADC value
				double val = -1.0f;
				if (i < ADCSamplePos-1)
					val = ADCSamples[(ADCSamplePos-1-i) % SAMPLE_RECORDS];
				//check if we are a X cursor point, if so keep the value
				if (XPos == XCursorPos)
					XCursorValue = val;
				//if ADC value is valid, draw it
				if (val > -0.0f)
				{
					//calc Y pixel position
					uint8_t YPos = round((val / 4096.0f) * LCD_HEIGHT);
					//flip it the right way up
					YPos = (LCD_HEIGHT-1) - YPos;
					//check limits
					if (YPos > LCD_HEIGHT - 1)
						YPos = LCD_HEIGHT - 1;
					//draw faded bar to the point
					for (j = YPos; j < LCD_HEIGHT; j++)
						LCD_FrameBuffer[XPos + (j * LCD_WIDTH)] = 0x70;
					//draw solid point, right to left
					LCD_FrameBuffer[XPos + (YPos * LCD_WIDTH)] = 0xff;
				}
			}

			//draw x cursor (dotted vertical line)
			for (i = 0; i < LCD_HEIGHT; i++)
				LCD_FrameBuffer[XCursorPos + (i*LCD_WIDTH)] = (i % 2 ? 0xff : 0x00);

			//draw x cursor value text if valid in volts on ADC pin
			if (XCursorValue > -0.0f)
			{
				//ADC-Volts = (ADCValue / MaxADCCount) * ADCMaxV * ResistorDivisor
				double Volts = (XCursorValue / 4096.0f) * 3.3f * 2.0f;
				tsprintf(TempS, "CVal:%fV", Volts);
			}
			else
				tsprintf(TempS, "CVal:N/A");
			Font_WriteString(LCD_FrameBuffer, 2, LCD_HEIGHT-30, TempS);

			//x cursor position text
			double XCursorPosS = (LCD_WIDTH - 1 - XCursorPos) * SampleLength / 1000.0f;
			tsprintf(TempS, "CPos:%fS", XCursorPosS);
			Font_WriteString(LCD_FrameBuffer, 2, LCD_HEIGHT-20, TempS);

			//sample time text
			if (SampleLength == 0)
				//paused
				tsprintf(TempS, "STime:PAUSE");
			else
				tsprintf(TempS, "STime:%fS", (double)SampleLength/1000.0f);
			Font_WriteString(LCD_FrameBuffer, 2, LCD_HEIGHT-10, TempS);

			//send framebuffer to the LCD controller
			LCDCore_BufferToLCD(LCD_FrameBuffer);

			//done
			DoLCDUpdate = 0;
		}

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
	LL_SetSystemCoreClock(84000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
	{
		Error_Handler();
	}
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
