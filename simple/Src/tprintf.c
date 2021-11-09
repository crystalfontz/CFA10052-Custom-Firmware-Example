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

#include "tprintf.h"

void uli2a(unsigned long int num, unsigned int base, int uc, char * bf)
{
	int n = 0;
	unsigned int d = 1;
	while (num / d >= base)
		d *= base;
	while (d != 0)
	{
		int dgt = num / d;
		num %= d;
		d /= base;
		if (n || dgt > 0 || d == 0)
		{
			*bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
			++n;
		}
	}
	*bf = 0;
}

static void li2a(long num, char * bf)
{
	if (num < 0)
	{
		num = -num;
		*bf++ = '-';
	}
	uli2a(num, 10, 0, bf);
}

void ui2a(unsigned int num, unsigned int base, int uc, char * bf)
{
	int n = 0;
	unsigned int d = 1;
	while (num / d >= base)
		d *= base;
	while (d != 0)
	{
		int dgt = num / d;
		num %= d;
		d /= base;
		if (n || dgt > 0 || d == 0)
		{
			*bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
			++n;
		}
	}
	*bf = 0;
}

static void i2a(int num, char * bf)
{
	if (num < 0)
	{
		num = -num;
		*bf++ = '-';
	}
	ui2a(num, 10, 0, bf);
}

static int a2d(char ch)
{
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	else
		if (ch >= 'a' && ch <= 'f')
			return ch - 'a' + 10;
		else
			if (ch >= 'A' && ch <= 'F')
				return ch - 'A' + 10;
			else
				return -1;
}

static char a2i(char ch, char** src, int base, int* nump)
{
	char* p = *src;
	int num = 0;
	int digit;
	while ((digit = a2d(ch)) >= 0)
	{
		if (digit > base)
			break;
		num = num * base + digit;
		ch = *p++;
	}
	*src = p;
	*nump = num;
	return ch;
}

#define putcp(p,c) *(*((char**)(p)))++ = (c)

static void putchw(void* putp, int n, char z, char* bf)
{
	char fc = z ? '0' : ' ';
	char ch;
	char* p = bf;
	while (*p++ && n > 0)
		n--;
	while (n-- > 0)
		putcp(putp, fc);
	while ((ch = *bf++))
		putcp(putp, ch);
}

static void tformat(void* putp, char *fmt, va_list va)
{
	char bf[12];
	char ch;

	while ((ch = *(fmt++)))
	{
		if (ch != '%')
			putcp(putp, ch);
		else
		{
			char lz = 0;
			char lng = 0;
			int w = 0;
			ch = *(fmt++);
			if (ch == '0')
			{
				ch = *(fmt++);
				lz = 1;
			}
			if (ch >= '0' && ch <= '9')
			{
				ch = a2i(ch, &fmt, 10, &w);
			}
			if (ch == 'l')
			{
				ch = *(fmt++);
				lng = 1;
			}
			switch (ch)
			{
				case 0:
					goto abort;
				case 'u':
				{
					if (lng)
						uli2a(va_arg(va, unsigned long int), 10, 0, bf);
					else
						ui2a(va_arg(va, unsigned int), 10, 0, bf);
					putchw(putp, w, lz, bf);
					break;
				}
				case 'd':
				{
					if (lng)
						li2a(va_arg(va, unsigned long int), bf);
					else
						i2a(va_arg(va, int), bf);
					putchw(putp, w, lz, bf);
					break;
				}
				case 'x':
				case 'X':
					if (lng)
						uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
					else
						ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
					putchw(putp, w, lz, bf);
					break;
				case 'c':
					putcp(putp, (char)(va_arg(va, int)));
					break;
				case 's':
					putchw(putp, w, 0, va_arg(va, char*));
					break;
				case '%':
					putcp(putp, ch);
					break;
				default:
					break;
			}
		}
	}
	abort: ;

}

inline void tsprintf(char *s, char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);
	tformat(&s, fmt, va);
	putcp(&s, 0);
	va_end(va);
}

inline void tvsprintf(char *s, char *fmt, va_list args)
{
	tformat(&s, fmt, args);
	putcp(&s, 0);
}
