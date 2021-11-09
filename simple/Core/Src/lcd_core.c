#include "main.h"
#include "lcd_core.h"
#include "st7529_core.h"
#include "uc1611s_core.h"
#include "font_6x8x1.h"

static uint8_t LCDCore_Controller = LCDCORE_NONE;

void LCDCore_Init(uint8_t LCD_Type)
{
	//set LCD controller type by hardware version
	LCDCore_Controller = LCD_Type;

	//LCD init
	switch (LCDCore_Controller)
	{
		case LCDCORE_NONE: return; break;
		case LCDCORE_ST7529:
			ST7529_Init();
			break;
		case LCDCORE_UC1611S:
			UC1611S_Init(0);
			break;
		case LCDCORE_UC1611S_INV:
			UC1611S_Init(1); //mirrored
			break;
	}
}

void LCDCore_SetContrast(uint8_t NewValue)
{
	int Value;
	int ContrastMin = 0;
	int ContrastMax = 0xFF;

	//the CFA735 has a approx 43 value shift from the CFA835
	//(CFA735/CFA835 data sheet "about right" values)
	//so adjust by 43 here
#ifdef CFA735
	if (NewValue < 255 - 43)
		NewValue += 43;
#endif

	//get contrast min/max/default values
	switch (LCDCore_Controller)
	{
		case LCDCORE_ST7529:
			ContrastMin = ST7529_CONTRAST_MIN;
			ContrastMax = ST7529_CONTRAST_MAX;
			break;
		case LCDCORE_UC1611S:
		case LCDCORE_UC1611S_INV:
			ContrastMin = UC1611S_CONTRAST_MIN;
			ContrastMax = UC1611S_CONTRAST_MAX;
			break;
		case LCDCORE_NONE: return; break;
	}

	//calc new controller value
	Value = (((ContrastMax - ContrastMin) * (int)NewValue) / 255) + ContrastMin;
	if (Value < 0) Value = 0;
	if (Value > 0xFF) Value = 0xFF;

	//set it
	switch (LCDCore_Controller)
	{
		case LCDCORE_NONE: return; break;
		case LCDCORE_ST7529:
			LCDCore_BusToWrite();
			ST7529_WriteContrast((uint8_t)Value);
			break;
		case LCDCORE_UC1611S:
		case LCDCORE_UC1611S_INV:
			LCDCore_BusToWrite();
			UC1611S_WriteContrast((uint8_t)Value);
			break;
	}
}

void LCDCore_BufferToLCD(uint8_t *BufferMem)
{
	switch (LCDCore_Controller)
	{
		case LCDCORE_NONE: return; break;
		case LCDCORE_ST7529:
			ST7529_BufferToLCD(BufferMem);
			break;
		case LCDCORE_UC1611S:
		case LCDCORE_UC1611S_INV:
			UC1611S_BufferToLCD(BufferMem);
			break;
	}
}

void LCDCore_LCDToBuffer(uint8_t *BufferMem)
{
	switch (LCDCore_Controller)
	{
		case LCDCORE_NONE: return; break;
		case LCDCORE_ST7529:
			ST7529_LCDToBuffer(BufferMem);
			break;
		case LCDCORE_UC1611S:
		case LCDCORE_UC1611S_INV:
			UC1611S_LCDToBuffer(BufferMem);
			break;
	}
}

void LCDCore_BusToWrite(void)
{
	switch (LCDCore_Controller)
	{
		case LCDCORE_NONE: return; break;
		case LCDCORE_ST7529:
			ST7529_BusToWrite();
			break;
		case LCDCORE_UC1611S:
		case LCDCORE_UC1611S_INV:
			UC1611S_BusToWrite();
			break;
	}
}

void LCDCore_BusToRead(void)
{
	switch (LCDCore_Controller)
	{
		case LCDCORE_NONE: return; break;
		case LCDCORE_ST7529:
			ST7529_BusToRead();
			break;
		case LCDCORE_UC1611S:
		case LCDCORE_UC1611S_INV:
			UC1611S_BusToRead();
			break;
	}
}

//////////////////////////////////////////////////////////////////////////////

//render a character to the specified frame buffer memory
void Font_DrawCharacter(uint8_t *Buffer, uint8_t XPos, uint8_t YPos, uint8_t Character)
{
	uint8_t		*BufferPos;
	uint8_t		State;
	uint8_t		x, y;
	uint8_t		*CChar;
	uint8_t		ExtraShift;

	//starting frame buffer memory location
	BufferPos = Buffer + XPos + (YPos * LCD_WIDTH);
	ExtraShift = 0;
	//check character type
	if (Character > FONT_BASE-1)
	{
		//is standard text
		Character -= FONT_BASE;
		if (Character > FONT_CHARNUM)
			Character = ' ' + FONT_BASE;
		CChar = (uint8_t*)Font_6X8[Character];
	}
	else
	{
		//out of range
		Character = ' ' + FONT_BASE;
		CChar = (uint8_t*)Font_6X8[Character];
	}

	//render the character
	for (y = 0; y < FONT_HEIGHT; y++)
	{
		for (x = 0; x < FONT_WIDTH; x++)
		{
			//get the bit state
			State = ((CChar[y] >> (FONT_WIDTH - x - ExtraShift)) & 1) ? 0xFF : 0x00;
			//set the pixel in the frame buffer
			*BufferPos = State;
			//next pixel
			BufferPos++;
		}
		//move to the next frame buffer pixel row
		BufferPos += LCD_WIDTH - FONT_WIDTH;
	}
}

//render a null-terminated string to the specified frame buffer memory
void Font_WriteString(uint8_t *Buffer, uint8_t XPos, uint8_t YPos, const char *Text)
{
	uint8_t	i;
	//one character at a time, up to 40 chars long
	for (i = 0; i < 40; i++)
	{
		if (Text[i] == 0)
			//stop if string is null-terminated
			break;
		//advance x pixel position
		XPos += FONT_WIDTH;
		//render the character
		Font_DrawCharacter(Buffer, XPos, YPos, Text[i]);
	}
}
