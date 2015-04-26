#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"

static uint8_t state;
static int16_t position;
static uint16_t errors;

void qencoder_Init()
{
	// Quadrature inputs are INT0/INT1 pins: PD2/PD3
	DDRD &= ~(_BV(2) | _BV(3));
	// No pullups
	PORTD &= ~(_BV(2) | _BV(3));

	// interrupts INT0 and INT1 on any change
	//MCUCR = (MCUCR & 0xf0) | _BV(ISC10) | _BV(ISC00);
	MCUCR |= _BV(ISC10) | _BV(ISC00);
	// enable external interrupts 0 and 1
	GICR |= _BV(INT1) | _BV(INT0);

	state = (PIND >> 2) & 3;
}


//    ____           ____
//  _|	  |_________|    |___
//       ____      ____
//  ____|    |____|    |___
// 0  1  1  0  0   0  1  1  0  lsb
// 0  0  1  1  0   1  1  0  0  msb
// 0  1  3  2  0   2  3  1  0  int
// ---- CW ----+---- CCW ----

static uint8_t dirtab[4][4] = {
//new 0   1   2   3    // old
	{ 0, -1, +1,  0},  // 0
	{+1,  0,  0, -1},  // 1
	{-1,  0,  0, +1},  // 2
	{ 0, +1, -1,  0}}; // 3

static inline void qencoder_Poll()
{
	uint8_t input = (PIND >> 2) & 3;

	if (state == input) {
		return;
	}

	int8_t dir = dirtab[state][input];
	if (dir == 0) {
		errors++;
	}
	state = input;
	position += dir;
}

int16_t qencoder_GetPosition()
{	
	int16_t result;
	atomic(result = position);
	return result;
}

uint16_t qencoder_GetErrorCount()
{
	return errors;
}

ISR(INT0_vect)
{
	qencoder_Poll();
}

ISR(INT1_vect,ISR_ALIASOF(INT0_vect));
