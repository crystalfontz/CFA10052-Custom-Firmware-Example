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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "keypad.h"
#include "leds.h"
#include "font_6x8x1.h"
#include "lcd_st7529.h"
#include "tprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
ADC_HandleTypeDef hadc1;

SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */

volatile uint64_t ADCDataSum = 0;
volatile uint32_t ADCDataCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void User_USART1_IRQHandler(void)
{
	//incoming data on USART1 (CFA10052 H1, pins 1 & 2)
	//called by USART1_IRQHandler() in stm32f4xx_it.c
	//dont do anything other than clear the flag
	LL_USART_ClearFlag_RXNE(USART1);
}

void USB_IncomingData(uint8_t *Buf, uint32_t *Len)
{
	//incoming data from USB serial (ACM CDC)
	//called from CDC_Receive_FS() in usbd_cdc_if.c
	//dont do anything
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
	//For some reason HAL doesnt always get this right
	//after SystemClock_Config(). Do it again.
	HAL_InitTick(TICK_INT_PRIORITY);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
	MX_USART2_UART_Init();
	MX_SDIO_SD_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	//CFA10052 specific init's
	LEDs_Init();
	Keypad_Init();
	ST7529_Init();
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
			ST7529_BufferToLCD(LCD_FrameBuffer);

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

	if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
		Error_Handler();
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_4);
	LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLQ_DIV_7);
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
	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;
	if (HAL_SD_Init(&hsd) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct =
	{ 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct =
	{ 0 };
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct =
	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	TIM_InitStruct.Prescaler = 9;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_DOWN;
	TIM_InitStruct.Autoreload = 100;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 0;
	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM1);
	LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = 0;
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**TIM1 GPIO Configuration
	 PB13   ------> TIM1_CH1N
	 PB14   ------> TIM1_CH2N
	 */
	GPIO_InitStruct.Pin = LCD_BL_PWM_Pin | KEY_BL_PWM_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct =
	{ 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct =
	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	TIM_InitStruct.Prescaler = 420;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 100;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM2);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**TIM2 GPIO Configuration
	 PA0-WKUP   ------> TIM2_CH1
	 PA1   ------> TIM2_CH2
	 */
	GPIO_InitStruct.Pin = LED0R_Pin | LED0G_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct =
	{ 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct =
	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	TIM_InitStruct.Prescaler = 420;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 100;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM3, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM3);
	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM3);
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**TIM3 GPIO Configuration
	 PA6   ------> TIM3_CH1
	 PA7   ------> TIM3_CH2
	 */
	GPIO_InitStruct.Pin = LED1R_Pin | LED1G_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct =
	{ 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct =
	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	TIM_InitStruct.Prescaler = 420;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 100;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM4, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM4);
	LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM4);
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**TIM4 GPIO Configuration
	 PB6   ------> TIM4_CH1
	 PB7   ------> TIM4_CH2
	 PB8   ------> TIM4_CH3
	 PB9   ------> TIM4_CH4
	 */
	GPIO_InitStruct.Pin = LED2R_Pin | LED2G_Pin | LED3R_Pin | LED3G_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct =
	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**USART1 GPIO Configuration
	 PA9   ------> USART1_TX
	 PA10   ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = H1_2_TX_Pin | H1_1_RX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 interrupt Init */
	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct =
	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**USART2 GPIO Configuration
	 PA2   ------> USART2_TX
	 PA3   ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = FBSCAB_TX_Pin | FBSCAB_RX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART2);
	LL_USART_Enable(USART2);
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	/**/
	LL_GPIO_ResetOutputPin(GPIOC,
		LCD_D0_KEY_UP_Pin | LCD_D1_KEY_DOWN_Pin | LCD_D2_KEY_LEFT_Pin | LCD_D3_KEY_RIGHT_Pin | LCD_D4_KEY_OK_Pin | LCD_D5_KEY_CANCEL_Pin | LCD_D6_Pin
			| LCD_D7_Pin);

	/**/
	LL_GPIO_ResetOutputPin(GPIOB, LCD_A0_BOOT1_Pin | LCD_CS_Pin | LCD_WR_Pin);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LCD_RD_Pin | LCD_RST_Pin);

	/**/
	GPIO_InitStruct.Pin = H1_13_GPIO4_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(H1_13_GPIO4_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LCD_D0_KEY_UP_Pin | LCD_D1_KEY_DOWN_Pin | LCD_D2_KEY_LEFT_Pin | LCD_D3_KEY_RIGHT_Pin | LCD_D4_KEY_OK_Pin | LCD_D5_KEY_CANCEL_Pin
		| LCD_D6_Pin | LCD_D7_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = H1_9_GPIO2_Pin | H1_10_GPIO3_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = H1_6_GPIO6_Pin | H1_4_GPIO10_Pin | H1_3_GPIO9_Pin | H1_7_GPIO11_Pin | H1_8_GPIO12_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LCD_A0_BOOT1_Pin | LCD_CS_Pin | LCD_WR_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LCD_RD_Pin | LCD_RST_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
