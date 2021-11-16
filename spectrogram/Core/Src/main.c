/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * Crystalfontz CFA10052 (hardware v1.1 onwards) Example firmware.
 *
 * This example displays a 0 to ~22khz spectrogram on the LCD, as sampled by
 * the first of the two ADC input pins.
 * See the README for example AC signal input circuitry.
 *
 * The ADC input is Header 1 - Pin 5.
 * Due to the resistive divider on the CFA10052 ADC input pin, maximum input
 * voltage is 6.6V.
 *
 * The ADC is sampled at 43,750Hz.
 *
 * The Tick key toggles between manual and auto amplitude range
 * Up/Down keys change the amplitude when in manual amplitude mode
 * Left/Right keys move the frequency measurement cursor
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
// CFA10052 hardware version v1.9+ = LCDCORE_UC1611S_INV
//#define LCD_TYPE		LCDCORE_ST7529
//#define LCD_TYPE		LCDCORE_UC1611S
//#define LCD_TYPE		LCDCORE_UC1611S_INV

#ifndef LCD_TYPE
# error LCD_TYPE NOT SET!!!
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_SAMP_FREQ	(84000000/4/480)
#define ADC_NYQUIST		(ADC_SAMP_FREQ/2)

#define FFT_N			(1024)
#define FFT_2N			(FFT_N*2)
#define FFT_N2			(FFT_N/2)

#define KEY_REPEAT_TIMER	(100) /*mS*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t LCD_FrameBuffer[LCD_WIDTH * LCD_HEIGHT];
volatile uint32_t ADCSamples[FFT_N];
volatile uint32_t ADCSampleCount = 0;
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
	//Runs at PCLK2/4/480 = 84000000/4/480 = 43750 Hz
	if (ADCSampleCount < FFT_N - 1)
	{
		ADCSamples[ADCSampleCount] = HAL_ADC_GetValue(&hadc1);
		ADCSampleCount++;
	}
}

//speedy log10 approximation
const float32_t C[4] =
{ 1.23149591368684f, -4.11852516267426f, 6.02197014179219f, -3.13396450166353f };
const float32_t LOG10_2 = log10(2.0f);
float32_t log10_approx(float32_t x)
{
	//reference: https://community.arm.com/tools/f/discussions/4292/cmsis-dsp-new-functionality-proposal
	float32_t f, l;
	int e;
	f = frexpf(fabsf(x), &e);
	l = LOG10_2 * (C[0] * f * f * f + C[1] * f * f + C[2] * f + C[3] + e);
	return l;
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
	HAL_ResumeTick();
	HAL_Delay(1);

	LEDs_Init();
	Keypad_Init();
	LCDCore_Init(LCD_TYPE);
	LEDs_LCDBacklightSet(50);
	LEDs_KeypadBacklightSet(50);

	//Start the ADC
	ADCSampleCount = 0;
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//local vars
	uint32_t i, j;
	char TempS[40];
	float32_t max;
	uint32_t index;
	uint8_t Mode;
	float32_t AmplAuto, AmplManual, AmplMult;
	float32_t FFT0;
	uint32_t FreqPeak;
	uint32_t XCursorPos;
	float32_t LCDData[LCD_WIDTH];
	float32_t FFTData[FFT_2N];
	uint32_t KeyTimer;

	//init FFT vars
	arm_cfft_radix4_instance_f32 FFTInst;
	arm_cfft_radix4_init_f32(&FFTInst, FFT_N, 0, 1);

	//init local vars
	Mode = 0;            //auto/manual ampl
	AmplManual = 0.5;
	AmplAuto = 0.5;
	XCursorPos = LCD_WIDTH / 2;

	//pre-calc window function (hanning)
	float32_t FFTWindow[FFT_N];
	for (i = 0; i < FFT_N; i++)
		FFTWindow[i] = (0.5 * (1 - arm_cos_f32(2 * PI * i / (FFT_N - 1))));

	//main loop
	TIMER_SET_NEXT(KeyTimer, KEY_REPEAT_TIMER);
	while (1)
	{
		//keypad check
		Keypad_CheckButtons();

		//handle button presses / holds
		if (Keypad_ButtonState[KEY_UP] & BUTTON_RELEASED)
			//up key pressed, increase manual amplitude value
			AmplManual += (AmplManual > 5.0) ? 0 : 0.05;
		if (Keypad_ButtonState[KEY_DOWN] & BUTTON_RELEASED)
			//down key pressed, decrease manual amplitude value
			AmplManual -= (AmplManual < 0.1) ? 0 : 0.05;
		if (Keypad_ButtonState[KEY_ENTER] & BUTTON_RELEASED)
			//tick key pressed, toggle auto/manual amplitude
			Mode = !Mode;
		if (Keypad_ButtonState[KEY_RIGHT] & BUTTON_RELEASED)
			//right key pressed, move cursor right
			XCursorPos += (XCursorPos >= LCD_WIDTH - 1) ? 0 : 1;
		if (Keypad_ButtonState[KEY_LEFT] & BUTTON_RELEASED)
			//left key pressed, move cursor left
			XCursorPos -= (XCursorPos < 1) ? 0 : 1;

		//key hold repeat timer
		IF_TIMER_EXPIRED(KeyTimer)
		{
			//reset timer
			TIMER_SET_NEXT(KeyTimer, KEY_REPEAT_TIMER)

			//handle key holds
			if (Keypad_ButtonState[KEY_UP] & BUTTON_HELD)
				//up key pressed, increase manual amplitude value
				AmplManual += (AmplManual > 10.0) ? 0 : 0.05f;
			if (Keypad_ButtonState[KEY_DOWN] & BUTTON_HELD)
				//down key pressed, decrease manual amplitude value
				AmplManual -= (AmplManual < 0.1f) ? 0 : 0.05f;
			if (Keypad_ButtonState[KEY_RIGHT] & BUTTON_HELD)
				//right key pressed, move cursor right
				XCursorPos += (XCursorPos >= LCD_WIDTH - 1) ? 0 : 1;
			if (Keypad_ButtonState[KEY_LEFT] & BUTTON_HELD)
				//left key pressed, move cursor left
				XCursorPos -= (XCursorPos < 1) ? 0 : 1;
		}

		//wait for the adc sample buffer to be filled
		//todo: change ADC conversion process to use DMA
		while (ADCSampleCount < FFT_N - 1)
			__NOP();

		//apply the window to the adc data, and convert for fft
		for (i = 0; i < FFT_N; i++)
		{
			//apply window, keep real part
			FFTData[i * 2] = (((float32_t) ADCSamples[i] - 2048.0) / 2048.0) * FFTWindow[i];
			//imaginary part
			FFTData[(i * 2) + 1] = 0;
		}
		//restart the ADC by zeroing the counter
		ADCSampleCount = 0;

		//process the data through the CFFT/CIFFT module
		arm_cfft_radix4_f32(&FFTInst, FFTData);
		//calc magnitude at each bin
		//input and output data use same buffer to save ram
		arm_cmplx_mag_f32(FFTData, FFTData, FFT_N);

		//keep max value, zero out index 0,1 (false values)
		FFT0 = FFTData[0];
		FFTData[0] = 0;
		FFTData[1] = 0;

		//find freq peak position for displaying as text later
		arm_max_f32(FFTData, FFT_N2, &max, &index);
		FreqPeak = (index * ADC_NYQUIST) / FFT_N2;

		//resize FFTData FFT_N2 length array down to LCD_WIDTH length for display
		//todo: this could be better, use max of values, not average
		for (i = 0; i < LCD_WIDTH; i++)
		{
			float32_t fIndex = (i * FFT_N2) / (float32_t)LCD_WIDTH;
			uint32_t uIndex = floor(fIndex);
			float32_t diffIndex = fIndex - uIndex;
			LCDData[i] = ((1.0 - diffIndex) * FFTData[uIndex]) + (diffIndex * FFTData[uIndex + 1]);
		}

		//scale data by max value of FFT0 & convert output to dB
		for (i = 0; i < LCD_WIDTH; i++)
			LCDData[i] = 20.0 * log10_approx((LCDData[i] / FFT0) + 1.0);

		//find max value in output
		arm_max_f32(LCDData, LCD_WIDTH, &max, &index);
		//auto display amplitude control
		if (max > AmplAuto)
			//new max value is higher than current amplitude, change to match
			AmplAuto = max;
		else
			//new max is lower than current amplitude, slowly lower
			AmplAuto *= 0.995;

		//clear the lcd frame buffer
		memset(LCD_FrameBuffer, 0x00, LCD_WIDTH * LCD_HEIGHT);

		//draw the graph
		AmplMult = (!Mode) ? AmplAuto : AmplManual;
		for (i = 0; i < LCD_WIDTH; i++)
		{
			//convert float data into Y pixel position using max amplitude value/setting
			uint32_t YPos = round((LCDData[i] * LCD_HEIGHT) / AmplMult);
			//check range
			if (YPos > LCD_HEIGHT - 1)
				YPos = LCD_HEIGHT - 1;
			//invert
			YPos = (LCD_HEIGHT - 1) - YPos;
			//draw a bar vertical bar
			for (j = YPos; j < LCD_HEIGHT; j++)
				LCD_FrameBuffer[i + (j * LCD_WIDTH)] = 0xFF;
		}

		//draw x cursor (dotted vertical line)
		for (i = 0; i < LCD_HEIGHT; i++)
			LCD_FrameBuffer[XCursorPos + (i * LCD_WIDTH)] = (i % 2 ? 0xff : 0x00);

		//freq peak text
		tsprintf(TempS, "PEAK:%u Hz", FreqPeak);
		Font_WriteString(LCD_FrameBuffer, 160, 5, TempS);
		//cursor frequency
		uint32_t FreqCursor = (XCursorPos * ADC_NYQUIST) / LCD_WIDTH;
		tsprintf(TempS, "CFRQ:%u Hz", FreqCursor);
		Font_WriteString(LCD_FrameBuffer, 160, 15, TempS);
		//max amplitude text
		if (Mode)
			tsprintf(TempS, "AMPL:M %f", AmplMult);
		else
			tsprintf(TempS, "AMPL:A %f", AmplMult);
		Font_WriteString(LCD_FrameBuffer, 160, 25, TempS);

		//send framebuffer to the LCD controller
		LCDCore_BufferToLCD(LCD_FrameBuffer);
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
