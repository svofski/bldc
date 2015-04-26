#pragma once

#include <avr/interrupt.h>

#define BV2(a,b) (_BV(a)|_BV(b))
#define BV3(a,b,c) (_BV(a)|_BV(b)|_BV(c))
#define BV4(a,b,c,d) (_BV(a)|_BV(b)|_BV(c)|_BV(d))
#define BV5(a,b,c,d,e) (_BV(a)|_BV(b)|_BV(c)|_BV(d)|_BV(e))
#define BV6(a,b,c,d,e,f) (_BV(a)|_BV(b)|_BV(c)|_BV(d)|_BV(e)|_BV(f))
#define BV7(a,b,c,d,e,f,g) (_BV(a)|_BV(b)|_BV(c)|_BV(d)|_BV(e)|_BV(f)|_BV(g))

// Half-bridge ENABLE bits (PORTC)
#define ENA1 0
#define ENA2 1
#define ENA3 2
#define ENA123_MASK (~BV3(ENA1,ENA2,ENA3))
#define ENA123_BITVAL BV3(ENA1,ENA2,ENA3)

// PWM bits (PORTB)
#define PWM1 1
#define PWM2 2
#define PWM3 3
#define PWM123_BITVAL BV3(PWM1,PWM2,PWM3)

#ifdef TEST
#define atomic(x) { x; }
#else
#define atomic(x) { cli(); x; sei(); }
#endif

void led_init();
void led_set(uint8_t on);

uint8_t fp_scale(int value, int scale);
int16_t fp_scale16(int value, int scale);
int32_t fp_scale32(int32_t value, int32_t scale);
