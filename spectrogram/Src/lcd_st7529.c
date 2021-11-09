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
#include "main.h"

#ifdef LCD_ST7529

#include "lcd_st7529.h"

//testing slower IO times
//#define SLOW_IO
#ifdef SLOW_IO
# warning LCD SLOW_IO ENABLED!!!
#endif

// Defines for the parallel interface to the ST7529 LCD Controller
#define ST7529_DATA_GPIO					LCD_D0_KEY_UP_GPIO_Port
#define ST7529_DATA_PINS					(LCD_D0_KEY_UP_Pin | LCD_D1_KEY_DOWN_Pin | LCD_D2_KEY_LEFT_Pin | \
		LCD_D3_KEY_RIGHT_Pin | LCD_D4_KEY_OK_Pin | LCD_D5_KEY_CANCEL_Pin | LCD_D6_Pin | LCD_D7_Pin)

#define ST7529_DATA_SET_LSB					0
#define ST7529_DATA_SET_FIELD_LEN			8
#define ST7529_DATA_SET_MASK				(((1 << ST7529_DATA_SET_FIELD_LEN) - 1) << ST7529_DATA_SET_LSB)
#define ST7529_DATA_SET_GET(ST7529WORD)		(((ST7529WORD) & ST7529_DATA_SET_MASK) >> ST7529_DATA_SET_LSB)
#define ST7529_DATA_SET_PUT(ST7529WORD)		(((ST7529WORD) << ST7529_DATA_SET_LSB) & ST7529_DATA_SET_MASK)

#define ST7529_DATA_RESET_LSB				16
#define ST7529_DATA_RESET_FIELD_LEN			8
#define ST7529_DATA_RESET_MASK				(((1 << ST7529_DATA_RESET_FIELD_LEN) - 1) << ST7529_DATA_RESET_LSB)
#define ST7529_DATA_RESET_GET(ST7529WORD)	(((ST7529WORD) & ST7529_DATA_RESET_MASK) >> ST7529_DATA_RESET_LSB)
#define ST7529_DATA_RESET_PUT(ST7529WORD)	(((ST7529WORD) << ST7529_DATA_RESET_LSB) & ST7529_DATA_RESET_MASK)

#define ST7529_A0_PORT          			LCD_A0_BOOT1_GPIO_Port
#define ST7529_A0							LCD_A0_BOOT1_Pin

#define ST7529_nWR_PORT         			LCD_WR_GPIO_Port
#define ST7529_nWR              			LCD_WR_Pin

#define ST7529_nRD_PORT         			LCD_RD_GPIO_Port
#define ST7529_nRD              			LCD_RD_Pin

#define ST7529_nCS_PORT         			LCD_CS_GPIO_Port
#define ST7529_nCS              			LCD_CS_Pin

#define ST7529_nRST_PORT        			LCD_RST_GPIO_Port
#define ST7529_nRST             			LCD_RST_Pin

//fast functions
#define ST7529_RD_LOW()		LL_GPIO_ResetOutputPin(ST7529_nRD_PORT, ST7529_nRD);
#define ST7529_RD_HIGH()	LL_GPIO_SetOutputPin(ST7529_nRD_PORT, ST7529_nRD);

#define ST7529_WR_LOW()		LL_GPIO_ResetOutputPin(LCD_WR_GPIO_Port, ST7529_nWR);
#define ST7529_WR_HIGH()	LL_GPIO_SetOutputPin(LCD_WR_GPIO_Port, ST7529_nWR);

#define ST7529_CS_LOW()		LL_GPIO_ResetOutputPin(ST7529_nCS_PORT, ST7529_nCS);
#define ST7529_CS_HIGH()	LL_GPIO_SetOutputPin(ST7529_nCS_PORT, ST7529_nCS);

#define ST7529_A0_LOW()		LL_GPIO_ResetOutputPin(ST7529_A0_PORT, ST7529_A0);
#define ST7529_A0_HIGH()	LL_GPIO_SetOutputPin(ST7529_A0_PORT, ST7529_A0);

#define ST7529_RST_LOW()	LL_GPIO_ResetOutputPin(ST7529_nRST_PORT, ST7529_nRST);
#define ST7529_RST_HIGH()	LL_GPIO_SetOutputPin(ST7529_nRST_PORT, ST7529_nRST);

//private functions
static void ST7529_WriteCmd(uint8_t Cmd);
static void ST7529_WriteDat(uint8_t Data);

//set STM32 GPIO pins to write data
void ST7529_BusToWrite(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = ST7529_DATA_PINS;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ST7529_DATA_GPIO, &GPIO_InitStruct);

}

//set STM32 GPIO pins to read data
void ST7529_BusToRead(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = ST7529_DATA_PINS;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ST7529_DATA_GPIO, &GPIO_InitStruct);
}

//write a command uint8_t to the ST7529
static void ST7529_WriteCmd(uint8_t Cmd)
{
	//A0 low for command
	ST7529_A0_LOW();
	//short delay for address setup time (tAW8 > 20nS)
	//2019-10-10 - measured A0 to CS = 132nS
	__NOP();
#ifdef SLOW_IO
	__NOP(); __NOP();
#endif
	//set data lines
	LL_GPIO_SetOutputPin(ST7529_DATA_GPIO, (uint32_t)(Cmd) | (uint32_t)(ST7529_DATA_RESET_MASK));
	//chip-select & write pin low to initiate data write
	ST7529_CS_LOW();
	ST7529_WR_LOW();
	//delay before deselecting (tDS8 > 150nS)
	//2019-10-10 - measured = 216nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#ifdef SLOW_IO
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#endif
	//chip-select and write pin high
	ST7529_WR_HIGH();
	ST7529_CS_HIGH();
	//short delay for address hold time (tAH8 > 20nS)
	//2019-10-10 - measured = 46nS
	__NOP();
#ifdef SLOW_IO
	__NOP(); __NOP();
#endif
	//A0 back to high
	ST7529_A0_HIGH();
	//short delay for system cycle time (tCYC8 > 200nS)
	//2019-10-10 - typically measured = ~400nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#ifdef SLOW_IO
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#endif
}

//write a data uint8_t to the ST7529
static void ST7529_WriteDat(uint8_t Data)
{
	//A0 is already high
	//set data lines
	LL_GPIO_SetOutputPin(ST7529_DATA_GPIO, (uint32_t)(Data) | (uint32_t)(ST7529_DATA_RESET_MASK));
	//chip-select & write pin low to initiate data write
	ST7529_CS_LOW();
	ST7529_WR_LOW();
	//delay before deselecting (tDS8 > 150nS)
	//2019-10-10 - measured = 216nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#ifdef SLOW_IO
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#endif
	//chip-select and write pin high
	ST7529_WR_HIGH();
	ST7529_CS_HIGH();
	//short delay for system cycle time (tCYC8 > 200nS)
	//2019-10-10 - typically measured = ~400nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#ifdef SLOW_IO
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#endif
}

//initialize the ST7529 LCD controller
void ST7529_Init(void)
{
	//blank the framebuffer
	memset(LCD_FrameBuffer, 0x00, LCD_WIDTH * LCD_HEIGHT);

	//other pin inits already done in main()
	ST7529_BusToWrite();

	// Deselect the ST7529: Chip Select high
	ST7529_RD_HIGH();
	ST7529_WR_HIGH();
	ST7529_CS_HIGH();
	ST7529_A0_LOW();

	// Reset the LCD
	ST7529_RST_LOW();
	HAL_Delay(15);
	ST7529_RST_HIGH();
	HAL_Delay(15);

	ST7529_WriteCmd(LCD_NOP);
	ST7529_WriteCmd(LCD_NOP);

	ST7529_WriteCmd(0x30);        //Ext = 0
	ST7529_WriteCmd(0x94);        //Sleep Out
	ST7529_WriteCmd(0xD1);        //OSC On

	ST7529_WriteCmd(0x20);        //Power Control Set
	ST7529_WriteDat(0x08);        //Booster Must Be On First
	HAL_Delay(5);

	ST7529_WriteCmd(0x20);        //Power Control Set
	ST7529_WriteDat(0x0B);        //Booster, Regulator, FollowerON
	HAL_Delay(5);

	ST7529_WriteContrast(LCD_CONTRAST_INIT);

	ST7529_WriteCmd(0xCA);        //Display Control
	ST7529_WriteDat(0x00);        //CL=X1
	ST7529_WriteDat(0x10);        //Duty=68
	ST7529_WriteDat(0x00);        //FR Inverse-Set Value
	ST7529_WriteCmd(0xA7);        //Normal Display

	ST7529_WriteCmd(0xBB);        //COM Scan Direction
	ST7529_WriteDat(0x00);        //0-79,80-159

	ST7529_WriteCmd(0xBC);        //Data Scan Direction
	ST7529_WriteDat(0x00);        //Normal
	ST7529_WriteDat(0x00);
	ST7529_WriteDat(0x02);

	ST7529_WriteCmd(0x75);        //Line Address Set
	ST7529_WriteDat(0x00);        //Start Line=0
	ST7529_WriteDat(0x43);        //End Line =67

	ST7529_WriteCmd(0x15);        //Column Address Set
	ST7529_WriteDat(0x00);        //Start Column=0
	ST7529_WriteDat(0x51);        //End Column =81

	ST7529_WriteCmd(0x31);        //Ext = 1
	ST7529_WriteCmd(0x32);        //Analog Circuit Set
	ST7529_WriteDat(0x00);        //OSC Frequency =000 (Default)
	ST7529_WriteDat(0x01);        //Booster Efficiency=01(Default)
	ST7529_WriteDat(0x05);        //Bias=1/9

	// Software initialize and EXT=0 Commands to follow
	ST7529_WriteCmd(LCD_SOFTWARE_INITIAL);

	//Gray table
	ST7529_WriteCmd(LCD_SOFTWARE_INITIAL);
	ST7529_WriteCmd(LCD_FRAME_1_GRAY_PWM_SET);
	ST7529_WriteDat(0);
	ST7529_WriteDat(2);
	ST7529_WriteDat(4);
	ST7529_WriteDat(6);
	ST7529_WriteDat(8);
	ST7529_WriteDat(10);
	ST7529_WriteDat(12);
	ST7529_WriteDat(14);
	ST7529_WriteDat(16);
	ST7529_WriteDat(18);
	ST7529_WriteDat(20);
	ST7529_WriteDat(22);
	ST7529_WriteDat(24);
	ST7529_WriteDat(26);
	ST7529_WriteDat(28);
	ST7529_WriteDat(30);
	ST7529_WriteCmd(LCD_FRAME_2_GRAY_PWM_SET);
	ST7529_WriteDat(0);
	ST7529_WriteDat(2);
	ST7529_WriteDat(4);
	ST7529_WriteDat(6);
	ST7529_WriteDat(8);
	ST7529_WriteDat(10);
	ST7529_WriteDat(12);
	ST7529_WriteDat(14);
	ST7529_WriteDat(16);
	ST7529_WriteDat(18);
	ST7529_WriteDat(20);
	ST7529_WriteDat(22);
	ST7529_WriteDat(24);
	ST7529_WriteDat(26);
	ST7529_WriteDat(28);
	ST7529_WriteDat(30);

	//done
	ST7529_WriteCmd(LCD_EXT_SET_0);
	ST7529_WriteCmd(LCD_DISPLAY_ON);
}

//initialize extra LCD panel settings (on flash settings load, after first LCD init)
void ST7529_ExtSet2(uint8_t LCDOscFreq, uint8_t LCDBoosterFreq, uint8_t LCDBiasRatio)
{
	ST7529_BusToWrite();
	ST7529_WriteCmd(LCD_EXT_SET_1);
	ST7529_WriteCmd(LCD_ANALOG_CIRCUIT_SET);
	ST7529_WriteDat(LCDOscFreq);
	ST7529_WriteDat(LCDBoosterFreq);
	ST7529_WriteDat(LCDBiasRatio);
	ST7529_WriteCmd(LCD_EXT_SET_0);
	ST7529_WriteCmd(LCD_DISPLAY_ON);
}

//set the LCD contrast
void ST7529_WriteContrast(uint16_t Contrast)
{
	ST7529_WriteCmd(LCD_CONTRAST_CONTROL);
	ST7529_WriteDat(Contrast & 0b00111111);
	ST7529_WriteDat(Contrast >> 6);
}

//write specified frame buffer memory to the LCD controller
void ST7529_BufferToLCD(uint8_t *Buffer)
{
	uint8_t Row, Col;
	//init GPIOs
	ST7529_BusToWrite();
	//set pixel coordinates (0x0 = top-left) for data write
	ST7529_WriteCmd(LCD_LINE_ADDR_SET);
	ST7529_WriteDat(LINE_ADDR_SET_PB1_START_LINE_PUT(0));
	ST7529_WriteDat(LINE_ADDR_SET_PB2_END_LINE_PUT(160));
	ST7529_WriteCmd(LCD_COL_ADDR_SET);
	ST7529_WriteDat(COL_ADDR_SET_PB1_START_COL_PUT(0));
	ST7529_WriteDat(COL_ADDR_SET_PB2_END_COL_PUT(255));
	ST7529_WriteCmd(LCD_MEM_WRITE);

	//write the frame buffer data to the LCD controller
	for (Col = 0; Col < LCD_HEIGHT; Col++)
	{
		for (Row = 0; Row < LCD_WIDTH; Row++)
			//write a byte of data
			ST7529_WriteDat(*Buffer++);
		//two unused pixels at the end of the row
		ST7529_WriteDat(0);
		ST7529_WriteDat(0);
	}
}

#endif
