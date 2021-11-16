#ifndef __LCD_CORE_H__
#define __LCD_CORE_H__

#include "main.h"

#define LCDCORE_NONE			0
#define LCDCORE_ST7529			1
#define LCDCORE_UC1611S			2
#define LCDCORE_UC1611S_INV		3

#define LCD_WIDTH		(244)
#define LCD_HEIGHT 		(68)

void LCDCore_Init(uint8_t LCD_Type);
void LCDCore_SetContrast(uint8_t NewValue);
void LCDCore_BufferToLCD(uint8_t *BufferMem);
void LCDCore_LCDToBuffer(uint8_t *BufferMem);
void LCDCore_BusToWrite(void);
void LCDCore_BusToRead(void);

void Font_DrawCharacter(uint8_t *Buffer, uint8_t XPos, uint8_t YPos, uint8_t Character);
void Font_WriteString(uint8_t *Buffer, uint8_t XPos, uint8_t YPos, const char *Text);

#endif
