#ifndef UC1611S_CORE_H_
#define UC1611S_CORE_H_

#include "main.h"

/////////////////////////////////////////////////////////

//contrast / lcd options
//make sure these are the same between CFA735/CFA835 and bootloader firmwares!
#define UC1611S_CONTRAST_MIN		-12
#define UC1611S_CONTRAST_MAX		175

/////////////////////////////////////////////////////////

// core functions
void UC1611S_Init(uint8_t Mirrored);
void UC1611S_WriteContrast(uint16_t contrast);
void UC1611S_BufferToLCD(uint8_t *BufferMem);
void UC1611S_LCDToBuffer(uint8_t *BufferMem);

void UC1611S_BusToWrite(void);
void UC1611S_BusToRead(void);

#endif
