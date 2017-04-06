#include <math.h>

extern const short window[512];
extern const short sin_tbl[512];
extern const float window_f[512];
extern const float sin_tbl_f[512];

#define PI 3.14159265358979323846

unsigned long dw_sqrt(unsigned long x)
{
	unsigned long tmp;
	unsigned long v_bit = 15;
	unsigned long n = 0;
	unsigned long b = 0x8000ul;

	if (x < 1)
	{
		return x;
	}

	do {
		tmp = ((n << 1) + b) << (v_bit--);
		if (x >= tmp)
		{
			n += b;
			x -= tmp;
		}
	} while (b >>= 1);

	return n;
}

void dft(short *buf, short *out)
{
	long r, i;
	long tmp;
	int k, n;

	for (k = 0; k < 512; k++)
	{
		r = 0, i = 0;
		for (n = 0; n < 512; n++)
		{
			tmp = buf[n] * window[n];
			tmp >>= 15;

			r += (tmp * sin_tbl[((n*k) + 128) & 0x1ff]) >> 15;
			i -= (tmp * sin_tbl[((n*k) + 0) & 0x1ff]) >> 15;
		}

		r >>= 7;
		i >>= 7;

		tmp = r * r;
		tmp += i * i;
		tmp *= 100;

		tmp = dw_sqrt(tmp);

		out[k] = (short)tmp;
	}
}

void dft_f(short *buf, short *out)
{
	float r, i;
	float tmp;
	int k, n;

	for (k = 0; k < 256; k++)
	{
		r = 0, i = 0;
		for (n = 0; n < 512; n++)
		{
			tmp = buf[n] * window_f[n];


			r += (tmp * sin_tbl_f[((n*k) + 128) & 0x1ff]);
			i -= (tmp * sin_tbl_f[((n*k) + 0) & 0x1ff]);
		}

		r /= 256;
		i /= 256;

		tmp = r * r;
		tmp += i * i;
		tmp *= 400;

		tmp = (float)sqrt(tmp);

		out[k] = (short)tmp;
	}
}
