#ifndef __ST7529_CORE_H__
#define __ST7529_CORE_H__

#include "main.h"
#include "st7529_defs.h"

/////////////////////////////////////////////////////////

//contrast / lcd options
//make sure these are the same between CFA735/CFA835 and bootloader firmwares!
#define ST7529_CONTRAST_MIN		120
#define ST7529_CONTRAST_MAX		233

/////////////////////////////////////////////////////////

// core functions
void ST7529_Init(void);
void ST7529_WriteContrast(uint16_t contrast);
void ST7529_BufferToLCD(uint8_t * bufferMem);
void ST7529_LCDToBuffer(uint8_t *BufferMem);

void ST7529_BusToWrite(void);
void ST7529_BusToRead(void);

#endif
