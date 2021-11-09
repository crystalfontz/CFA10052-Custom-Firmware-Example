#include "main.h"
#include "uc1611s_core.h"

#define LCD_WIDTH		(244)
#define LCD_HEIGHT 		(68)

// Defines for the parallel interface to the UC1611S LCD Controller
#define UC1611S_DATA_GPIO					LCD_D0_KEY_UP_GPIO_Port
#define UC1611S_DATA_PINS					(LCD_D0_KEY_UP_Pin | LCD_D1_KEY_DOWN_Pin | LCD_D2_KEY_LEFT_Pin | \
		LCD_D3_KEY_RIGHT_Pin | LCD_D4_KEY_OK_Pin | LCD_D5_KEY_CANCEL_Pin | LCD_D6_Pin | LCD_D7_Pin)

#define UC1611S_DATA_SET_LSB				0
#define UC1611S_DATA_SET_FIELD_LEN			8
#define UC1611S_DATA_SET_MASK				(((1 << UC1611S_DATA_SET_FIELD_LEN) - 1) << UC1611S_DATA_SET_LSB)
#define UC1611S_DATA_SET_GET(UC1611Suint16_t)		(((UC1611Suint16_t) & UC1611S_DATA_SET_MASK) >> UC1611S_DATA_SET_LSB)
#define UC1611S_DATA_SET_PUT(UC1611Suint16_t)		(((UC1611Suint16_t) << UC1611S_DATA_SET_LSB) & UC1611S_DATA_SET_MASK)

#define UC1611S_DATA_RESET_LSB				16
#define UC1611S_DATA_RESET_FIELD_LEN		8
#define UC1611S_DATA_RESET_MASK				(((1 << UC1611S_DATA_RESET_FIELD_LEN) - 1) << UC1611S_DATA_RESET_LSB)
#define UC1611S_DATA_RESET_GET(UC1611Suint16_t)	(((UC1611Suint16_t) & UC1611S_DATA_RESET_MASK) >> UC1611S_DATA_RESET_LSB)
#define UC1611S_DATA_RESET_PUT(UC1611Suint16_t)	(((UC1611Suint16_t) << UC1611S_DATA_RESET_LSB) & UC1611S_DATA_RESET_MASK)

#define UC1611S_ROW_WIDTH					0xFF /* LCD is 244 pixels wide, but row mem is 255 */

#define UC1611S_A0_PORT          			LCD_A0_BOOT1_GPIO_Port
#define UC1611S_A0							LCD_A0_BOOT1_Pin

#define UC1611S_nWR_PORT         			LCD_WR_GPIO_Port
#define UC1611S_nWR              			LCD_WR_Pin

#define UC1611S_nRD_PORT         			LCD_RD_GPIO_Port
#define UC1611S_nRD              			LCD_RD_Pin

#define UC1611S_nCS_PORT         			LCD_CS_GPIO_Port
#define UC1611S_nCS              			LCD_CS_Pin

#define UC1611S_nRST_PORT        			LCD_RST_GPIO_Port
#define UC1611S_nRST             			LCD_RST_Pin

//fast functions
#define UC1611S_RD_LOW()	LL_GPIO_ResetOutputPin(UC1611S_nRD_PORT, UC1611S_nRD);
#define UC1611S_RD_HIGH()	LL_GPIO_SetOutputPin(UC1611S_nRD_PORT, UC1611S_nRD);

#define UC1611S_WR_LOW()	LL_GPIO_ResetOutputPin(LCD_WR_GPIO_Port, UC1611S_nWR);
#define UC1611S_WR_HIGH()	LL_GPIO_SetOutputPin(LCD_WR_GPIO_Port, UC1611S_nWR);

#define UC1611S_CS_LOW()	LL_GPIO_ResetOutputPin(UC1611S_nCS_PORT, UC1611S_nCS);
#define UC1611S_CS_HIGH()	LL_GPIO_SetOutputPin(UC1611S_nCS_PORT, UC1611S_nCS);

#define UC1611S_A0_LOW()	LL_GPIO_ResetOutputPin(UC1611S_A0_PORT, UC1611S_A0);
#define UC1611S_A0_HIGH()	LL_GPIO_SetOutputPin(UC1611S_A0_PORT, UC1611S_A0);

#define UC1611S_RST_LOW()	LL_GPIO_ResetOutputPin(UC1611S_nRST_PORT, UC1611S_nRST);
#define UC1611S_RST_HIGH()	LL_GPIO_SetOutputPin(UC1611S_nRST_PORT, UC1611S_nRST);

//private functions
static void UC1611S_WriteCmd(uint8_t Cmd);
static void UC1611S_WriteDat(uint8_t Data);
void UC1611S_Clear(void);
//static void UC1611S_WriteLineAddr(uint8_t StartLine, uint8_t EndLine);
//static void UC1611S_WriteColAddr(uint8_t StartCol, uint8_t EndCol);

//public functions
void UC1611S_BusToWrite(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = UC1611S_DATA_PINS;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(UC1611S_DATA_GPIO, &GPIO_InitStruct);

}

void UC1611S_BusToRead(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = UC1611S_DATA_PINS;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(UC1611S_DATA_GPIO, &GPIO_InitStruct);
}

static void UC1611S_WriteCmd(uint8_t Cmd)
{
	// Write a command uint8_t

	//set data lines
	LL_GPIO_SetOutputPin(UC1611S_DATA_GPIO, (uint32_t)(Cmd) | (uint32_t)(UC1611S_DATA_RESET_MASK));

	//A0 low for command
	UC1611S_A0_LOW();
	//CS select
	UC1611S_CS_LOW();
	//write pin low to initiate data write
	UC1611S_WR_LOW();

	//delay before deselecting (tPWW80 > 65nS)
	//tested 2020-03-02 gcc -Os/-O3 = 84nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	//write pin high to load command
	UC1611S_WR_HIGH();


	//write pin high to load data
	UC1611S_WR_HIGH();

	//delay for WR high (tDH80 > 65nS)
	//tested 2020-03-02 gcc -Os/-O3 = 84nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	//return CS
	UC1611S_CS_HIGH();
	UC1611S_A0_HIGH();
}

static uint8_t UC1611S_ReadDat(void)
{
	//read a data uint8_t
	uint8_t data;

	//A0 is already high
	//CS select
	UC1611S_CS_LOW();
	//read pin low to initiate data read
	UC1611S_RD_LOW();

	//delay before deselecting (tPWW80 > 65nS)
	//tested 2020-03-02 gcc -Os/-O3 = 84nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	//write pin high to load data
	UC1611S_RD_HIGH();

	//read the data lines
	data = LL_GPIO_ReadInputPort(UC1611S_DATA_GPIO) & 0xFF;

	//delay for WR high (tDH80 > 65nS)
	//tested 2020-03-02 gcc -Os/-O3 = 84nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	//return CS
	UC1611S_CS_HIGH();

	return data;
}

static void UC1611S_WriteDat(uint8_t Data)
{
	//Write a data uint8_t

	//set data lines
	LL_GPIO_SetOutputPin(UC1611S_DATA_GPIO, (uint32_t)(Data) | (uint32_t)(UC1611S_DATA_RESET_MASK));

	//A0 is already high
	//CS select
	UC1611S_CS_LOW();
	//write pin low to initiate data write
	UC1611S_WR_LOW();

	//delay before deselecting (tPWW80 > 65nS)
	//tested 2020-03-02 gcc -Os/-O3 = 84nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	//write pin high to load data
	UC1611S_WR_HIGH();

	//delay for WR high (tDH80 > 65nS)
	//tested 2020-03-02 gcc -Os/-O3 = 84nS
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	//return CS
	UC1611S_CS_HIGH();
}

////////////////////////////////////////////////////////////

/*
 * From Linux UC1611s FB Driver
 *
 * LCD voltage is a combination of ratio, gain, pot and temp
 *
 * V_LCD = V_BIAS * ratio
 * V_LCD = (C_V0 + C_PM × pot) * (1 + (T - 25) * temp)
 * C_V0 and C_PM depend on ratio and gain
 * T is ambient temperature
 *
 * contrast = (gain & 0x03) << 6 | (pot & 0x3F)
 */

/*
//default GAIN=3 POT=16
#define CONT_GAIN	(3)
#define CONT_POT	(16)
#define CONT_CALC	((CONT_GAIN & 0x03) << 6 | (CONT_POT & 0x3F))
*/

// Config the STM32 to talk to the UC1611S and init the UC1611S
void UC1611S_Init(uint8_t Mirrored)
{
	//other pin inits already done in main()
	UC1611S_BusToWrite();

	// Deselect the UC1611S
	UC1611S_RD_HIGH();
	UC1611S_WR_HIGH();
	UC1611S_CS_HIGH();
	UC1611S_A0_LOW();

	// Reset the LCD
	UC1611S_RST_LOW();
	LL_mDelay(10);
	UC1611S_RST_HIGH();
	LL_mDelay(200);

	// Controller Init
//	UC1611S_WriteCmd(0xe2);            //SYS RST
//	HAL_Delay(200);

	UC1611S_WriteCmd(0x25);			//TMP Compensation
	UC1611S_WriteCmd(0x2b);			//set panel loading
	UC1611S_WriteCmd(0x2f);			//set pump control

	UC1611S_WriteCmd(0x81);
	//set VOL CV0+CPM*PM=9.091+0.02822*pm
	UC1611S_WriteCmd(82);

	UC1611S_WriteCmd(0x87);			//set partial display ENABLE
	UC1611S_WriteCmd(0b10001001);	//set RAM address control
	UC1611S_WriteCmd(0xa3);			//set line rate
	UC1611S_WriteCmd(0xa4);			//set SEG OUT ON
	UC1611S_WriteCmd(0xa6);			//set inverse  display

	UC1611S_WriteCmd(0xc0);			//set mapping control
	if (Mirrored == 0)
		//pre hw2v0
		UC1611S_WriteCmd(0b0110);		//LC3=0, MY=1, MX=1, MSF=0
	else
		//mirrored post hw2v0
		UC1611S_WriteCmd(0b0100);		//LC3=0, MY=1, MX=0, MSF=0

	UC1611S_WriteCmd(0xc8);			//SET N-line Inversion
	UC1611S_WriteCmd(0x08);

	UC1611S_WriteCmd(0xd1);			//set display pattern
	UC1611S_WriteCmd(0xe9);

	UC1611S_WriteCmd(0xf1);			//set com END
	UC1611S_WriteCmd(0x9f);

	UC1611S_WriteCmd(0xf2);			//set dst
	UC1611S_WriteCmd(92);

	UC1611S_WriteCmd(0xf3);			//set DEN
	UC1611S_WriteCmd(159);

	UC1611S_WriteCmd(0xA8 | 0b111);	//set display Enable in 16 shade mode

	UC1611S_Clear();
}

////////////////////////////////////////////////////////////////////////

/*
	// Set column address (not used by driver)
	write_reg(par, xs & 0x0F);
	write_reg(par, 0x10 | (xs >> 4));

	// Set page address (divide ys by 2)
	write_reg(par, 0x60 | ((ys >> 1) & 0x0F));
	write_reg(par, 0x70 | (ys >> 5));
*/

/*
	x = ((u8x8_tile_t *)arg_ptr)->x_pos;
	x *= 8;
	x += u8x8->x_offset;

	u8x8_cad_SendCmd(u8x8, 0x000 | ((x&15)));
	u8x8_cad_SendCmd(u8x8, 0x010 | (x>>4));

	y = ((u8x8_tile_t *)arg_ptr)->y_pos;
	u8x8_cad_SendCmd(u8x8, 0x060 | (y&15));
	u8x8_cad_SendCmd(u8x8, 0x070 | (y>>4));
*/

/*
inline static void UC1611S_WriteLineAddr(uint8_t StartLine, uint8_t EndLine)
{
	// Set the line address on the UC1611S
	UC1611S_WriteCmd(LCD_LINE_ADDR_SET);
	UC1611S_WriteDat(LINE_ADDR_SET_PB1_START_LINE_PUT(StartLine + LCD_ROW_OFFSET));
	UC1611S_WriteDat(LINE_ADDR_SET_PB2_END_LINE_PUT(EndLine + LCD_ROW_OFFSET));
}

inline static void UC1611S_WriteColAddr(uint8_t StartCol, uint8_t EndCol)
{
	// Set the column address on the UC1611S
	UC1611S_WriteCmd(LCD_COL_ADDR_SET);
	UC1611S_WriteDat(COL_ADDR_SET_PB1_START_COL_PUT(StartCol + LCD_COL_OFFSET));
	UC1611S_WriteDat(COL_ADDR_SET_PB2_END_COL_PUT(EndCol + LCD_COL_OFFSET));
}
*/

void UC1611S_WriteContrast(uint16_t contrast)
{
	//set contrast
	UC1611S_WriteCmd(0x81);
	UC1611S_WriteCmd(contrast);
}

void UC1611S_Clear(void)
{
	//set data address
	UC1611S_WriteCmd(0x000); //x to 0
	UC1611S_WriteCmd(0x010);
	UC1611S_WriteCmd(0x060); //y to 0
	UC1611S_WriteCmd(0x070);

	for (uint32_t i = 0; i < UC1611S_ROW_WIDTH * LCD_HEIGHT / 2; i++)
		UC1611S_WriteDat(0x00);
}

void UC1611S_BufferToLCD(uint8_t *BufferMem)
{
	uint8_t Row, Col;

	UC1611S_BusToWrite();

	//set data address
	UC1611S_WriteCmd(0x000); //x to 0
	UC1611S_WriteCmd(0x010);
	UC1611S_WriteCmd(0x060); //y to 0
	UC1611S_WriteCmd(0x070);

	//send the screen buffer
	for (Col = 0; Col < LCD_HEIGHT/2; Col++)
	{
		//two rows (4 bits per pixel)
		for (Row = 0; Row < LCD_WIDTH; Row++)
		{
			uint8_t out = (*(BufferMem+LCD_WIDTH) & 0xF0) | (*BufferMem >> 4);
			UC1611S_WriteDat(out);
			BufferMem++;
		}
		BufferMem += LCD_WIDTH;
		//last 12 pixels on row (not shown on LCD)
		for (Row = 0; Row < 12; Row++)
			UC1611S_WriteDat(0x00);
	}
}

void UC1611S_LCDToBuffer(uint8_t *BufferMem)
{
	uint8_t Row, Col;
	uint8_t Data;

	//set data address
	UC1611S_BusToWrite();
	UC1611S_WriteCmd(0x000); //x to 0
	UC1611S_WriteCmd(0x010);
	UC1611S_WriteCmd(0x060); //y to 0
	UC1611S_WriteCmd(0x070);

	//read up the data
	UC1611S_BusToRead();
	//first read is a dummy uint8_t
	UC1611S_ReadDat();

	//read the screen buffer
	for (Col = 0; Col < LCD_HEIGHT/2; Col++)
	{
		//two rows (4 bits per pixel)
		for (Row = 0; Row < LCD_WIDTH; Row++)
		{
			Data = UC1611S_ReadDat();
			*BufferMem = (Data & 0x0F) << 4;
			*(BufferMem + LCD_WIDTH) = (Data & 0xF0);
			BufferMem++;
		}
		BufferMem += LCD_WIDTH; //skip the next row
		//last 12 pixels on row (not shown on LCD)
		for (Row = 0; Row < 12; Row++)
			UC1611S_ReadDat();
	}
}
