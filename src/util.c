#include <inttypes.h>
#include "util.h"

uint8_t fp_scale(int value, int scale)
{
	return (int32_t)value * (int32_t)scale / 256;
}

int16_t fp_scale16(int value, int scale)
{
	return (int32_t)value * (int32_t)scale / 256;
}

int32_t fp_scale32(int32_t value, int32_t scale)
{
	return (int32_t)value * (int32_t)scale / 65536;
}