#include <inttypes.h>

#include <stdio.h>
#include <avr/pgmspace.h>

#include "util.h"

// (lambda scale, pidegrees: [int(round(asin(1.0*x/scale)/pi*pidegrees)) for x in xrange(0,scale+1)])(256,1024)
static const int16_t asintab[] = {0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29, 31, 32, 33, 34, 36, 37, 38, 40, 41, 42, 43, 45, 46, 47, 49, 50, 51, 52, 54, 55, 56, 58, 59, 60, 61, 63, 64, 65, 67, 68, 69, 71, 72, 73, 74, 76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96, 97, 98, 100, 101, 102, 104, 105, 106, 108, 109, 110, 112, 113, 114, 116, 117, 118, 120, 121, 123, 124, 125, 127, 128, 129, 131, 132, 134, 135, 136, 138, 139, 141, 142, 143, 145, 146, 148, 149, 150, 152, 153, 155, 156, 158, 159, 160, 162, 163, 165, 166, 168, 169, 171, 172, 174, 175, 177, 178, 180, 181, 183, 184, 186, 187, 189, 190, 192, 193, 195, 196, 198, 199, 201, 202, 204, 206, 207, 209, 210, 212, 214, 215, 217, 218, 220, 222, 223, 225, 227, 228, 230, 232, 233, 235, 237, 238, 240, 242, 244, 245, 247, 249, 251, 252, 254, 256, 258, 260, 261, 263, 265, 267, 269, 271, 273, 275, 276, 278, 280, 282, 284, 286, 288, 290, 292, 294, 296, 298, 301, 303, 305, 307, 309, 311, 314, 316, 318, 320, 323, 325, 327, 330, 332, 335, 337, 340, 342, 345, 347, 350, 353, 355, 358, 361, 364, 367, 370, 373, 376, 379, 382, 386, 389, 393, 396, 400, 404, 408, 412, 416, 421, 425, 430, 436, 441, 447, 454, 462, 471, 483, 512};

static uint16_t h1_median;
static uint16_t h2_medianplus;
static uint16_t h2_medianminus;
static uint16_t h1_min;
//static uint16_t h1_scale;
static uint32_t h1_scale;

void encoder_setHallsCalibration(uint16_t _h1_min, uint16_t h1_max, uint16_t _h2_medianplus, uint16_t _h2_medianminus)
{
	h1_min = _h1_min;
	h1_median = h1_min + (h1_max - h1_min) / 2;

	h2_medianplus = _h2_medianplus;
	h2_medianminus = _h2_medianminus;

	h1_scale = 512L*65536L / (h1_max - h1_min - 4); 
	printf_P(PSTR(" cal: h1_scale=%ld\n"), h1_scale);
}

int16_t encoder_getAngle(int16_t h1, int16_t h2)
{
	int16_t result = 0;
	int16_t scaled;

	h1 = h1 - h1_median;

	if (h1 >= 0) {
		scaled = fp_scale32(h1, h1_scale);
		if (scaled > 256) {
			scaled = 256;
		}
		result = asintab[scaled];
	} else {
		scaled = fp_scale32(-h1, h1_scale);
		if (scaled > 256) {
			scaled = 256;
		}
		result = -asintab[scaled];
	}
	
	printf_P(PSTR("  h1=%d h2=%d res=%d "), h1, h2, result);

	if (h1 > 0 && h2 <= h2_medianplus) {
		return result;
	} else if (h1 > 0 && h2 > h2_medianplus) {
		return 1024 - result;
	} else if (h1 < 0 && h2 > h2_medianminus) {
		return 1024 - result;
	} else {
		return 2048 + result;
	}
	return result;
}