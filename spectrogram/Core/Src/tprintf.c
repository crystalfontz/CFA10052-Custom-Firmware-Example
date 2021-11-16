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
#include <stdarg.h>
#include "tprintf.h"

#define TPRINTF_FLOAT

#ifdef TPRINTF_FLOAT
# pragma message "TPRINTF_FLOAT defined"
#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

void ftoa(double f, char *buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;
}
#endif

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
#ifdef TPRINTF_FLOAT
				case 'f':
					ftoa(va_arg(va, double), bf, 2);
					putchw(putp, w, lz, bf);
					break;
#endif
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
