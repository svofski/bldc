#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "util.h"

volatile int16_t chan6;
volatile int16_t chan7;

volatile int16_t hall_transition = 0;

int16_t get_hallTransitionCount()
{
	int16_t retval;
	atomic(retval = hall_transition);

	return retval;
}

int16_t pop_hallTransitionCount()
{	
	int16_t retval;
	atomic(retval = hall_transition);
	return retval;
}

int16_t drop_hallTransitionCount()
{
	int16_t retval;
	atomic(
		retval = hall_transition;
		hall_transition = 0;
	);
	return retval;
}

void adc_init()
{
	// Vref = VCC
	ADMUX = _BV(REFS0);
}

static volatile uint8_t chan6_state;
static volatile uint8_t dummy = 0;

#define HYST 20

ISR(ADC_vect)
{
	dummy = !dummy;
	if (dummy) {
		uint8_t chan = ADMUX & 7;
		if (chan == 6) {
			chan6 = (chan6 + ADC) >> 1;
			if ((!chan6_state && chan6 >= 512 + HYST) || (chan6_state && chan6 < 512 - HYST)) {
				chan6_state = !chan6_state;
				hall_transition++;
			}

			led_set(chan6 > 512);
		} else {
			chan7 = (chan7 + ADC) >> 1;
		}
		// toggle between channel 6 and 7
		ADMUX ^= 1;
	}
	ADCSRA |= _BV(ADSC);
}

void adc_start()
{
	ADCSRA |= _BV(ADEN);
	// start with channel 6
	ADMUX = (ADMUX & 0370) | 6;
	// start conversion
	// ADC Enable, Free Running Mode, Interrupt Enable, Prescaler = 128
	//ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | 7; //Prescaler 7 = 128

	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | 5; // prescaler 5 = 32
}

void adc_stop()
{
	ADCSRA &= ~(_BV(ADEN) | _BV(ADIE));
}

void adc_getValues(int16_t *a, int16_t *b)
{
	atomic(
		*a = chan6;
		*b = chan7;
	);
}
