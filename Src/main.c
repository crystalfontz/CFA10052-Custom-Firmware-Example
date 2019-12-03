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
#include "fatfs.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */

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

void USB_IncomingData(uint8_t* Buf, uint32_t *Len)
{
	//incoming data from USB serial (ACM CDC)
	//called from CDC_Receive_FS() in usbd_cdc_if.c
	//echo data back out to USB

	//this is super basic, and for example only.
	//It will fail if incoming data is not full transmitted
	//before new data is received. A proper FIFO is needed.
	memcpy(UserTxBufferFS, Buf, *Len);
	CDC_Transmit_FS(UserTxBufferFS, *Len);

	//function that calls us takes care of setting up the RX buffer
	//for the next transaction, so nothing to do here
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
	MX_USB_DEVICE_Init();
	MX_USART2_UART_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	//CFA10052 specific init's
	LEDs_Init();
	Keypad_Init();
	ST7529_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	char TempS[40];
	uint8_t LEDRotate = 0;
	uint8_t Contrast = LCD_CONTRAST_INIT;
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
			tsprintf(TempS, "Keypad Check: \003%c \004%c \002%c \001%c \006%c \007%c", (Pressed[KEY_UP] ? '\xBB' : '.'), (Pressed[KEY_DOWN] ? '\xBB' : '.'),
				(Pressed[KEY_LEFT] ? '\xBB' : '.'), (Pressed[KEY_RIGHT] ? '\xBB' : '.'), (Pressed[KEY_ENTER] ? '\xBB' : '.'),
				(Pressed[KEY_CANCEL] ? '\xBB' : '.'));
			Font_WriteString(LCD_FrameBuffer, 10, 34 + 12, TempS);
			//write framebuffer to lcd controller ic
			ST7529_BufferToLCD(LCD_FrameBuffer);

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
		ST7529_WriteContrast(Contrast);
		//update lcd backlight brightness
		LEDs_LCDBacklightSet(Backlight);
		//update keypad backlight brightness
		LEDs_KeypadBacklightSet(Keypad);

		//loop delay
		HAL_Delay(5);            //5mS

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
	GPIO_InitStruct.Pin = H1_5_GPIO5_Pin | H1_6_GPIO6_Pin | H1_4_GPIO10_Pin | H1_3_GPIO9_Pin | H1_7_GPIO11_Pin | H1_8_GPIO12_Pin;
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
