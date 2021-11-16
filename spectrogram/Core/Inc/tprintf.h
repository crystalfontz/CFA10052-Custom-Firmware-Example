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
#ifndef CORE_LIB_TPRINTF_H_TXT_
#define CORE_LIB_TPRINTF_H_TXT_

#include <stdarg.h>

void tsprintf(char* s,char *fmt, ...);
void tvsprintf(char *s, char *fmt, va_list args);

void uli2a(unsigned long int num, unsigned int base, int uc, char * bf);
void ui2a(unsigned int num, unsigned int base, int uc, char * bf);
void ftoa(double f, char *buf, int precision);

#endif /* CORE_LIB_TPRINTF_H_TXT_ */
