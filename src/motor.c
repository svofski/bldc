#include <inttypes.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#ifdef TEST
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#endif

#include "util.h"
#include "motor.h"
#if 0
#define FULL_CIRCLE (96*256)
// full circle, 2π = 96 steps
const int8_t sintab[] = {0, 8, 17, 25, 33, 41, 49, 56, 63, 71, 77, 84, 90, 95, 101, 106, 110, 114, 117, 120, 123, 125, 126, 127, 127, 127, 126, 125, 123, 120, 117, 114, 110, 106, 101, 95, 90, 84, 77, 71, 63, 56, 49, 41, 33, 25, 17, 8, 0, -8, -17, -25, -33, -41, -49, -56, -63, -71, -77, -84, -90, -95, -101, -106, -110, -114, -117, -120, -123, -125, -126, -127, -127, -127, -126, -125, -123, -120, -117, -114, -110, -106, -101, -95, -90, -84, -77, -71, -64, -56, -49, -41, -33, -25, -17, -8};

#define PHASE1 (0)
#define PHASE2 (32 << 8)
#define PHASE3 (64 << 8)

#else
// (lambda s: [int(round(127*sin(2*pi*x/s))) for x in xrange(s)])(192)
static const int8_t sintab[] = {0, 4, 8, 12, 17, 21, 25, 29, 33, 37, 41, 45, 49, 52, 56, 60, 63, 67, 71, 74, 77, 81, 84, 87, 90, 93, 95, 98, 101, 103, 106, 108, 110, 112, 114, 116, 117, 119, 120, 122, 123, 124, 125, 125, 126, 126, 127, 127, 127, 127, 127, 126, 126, 125, 125, 124, 123, 122, 120, 119, 117, 116, 114, 112, 110, 108, 106, 103, 101, 98, 95, 93, 90, 87, 84, 81, 77, 74, 71, 67, 63, 60, 56, 52, 49, 45, 41, 37, 33, 29, 25, 21, 17, 12, 8, 4, 0, -4, -8, -12, -17, -21, -25, -29, -33, -37, -41, -45, -49, -52, -56, -60, -63, -67, -71, -74, -77, -81, -84, -87, -90, -93, -95, -98, -101, -103, -106, -108, -110, -112, -114, -116, -117, -119, -120, -122, -123, -124, -125, -125, -126, -126, -127, -127, -127, -127, -127, -126, -126, -125, -125, -124, -123, -122, -120, -119, -117, -116, -114, -112, -110, -108, -106, -103, -101, -98, -95, -93, -90, -87, -84, -81, -77, -74, -71, -67, -64, -60, -56, -52, -49, -45, -41, -37, -33, -29, -25, -21, -17, -12, -8, -4};

#define PHASE1 (0)
#define PHASE2 (64 << 8)
#define PHASE3 (128L << 8)
#endif

static int32_t phase_sinA;
static int32_t phase_sinB;
static int32_t phase_sinC;

static uint32_t phase_acc;
static int16_t phase_inc;
static uint8_t pwm_val;
static int16_t power = 120;

static int32_t mechanical_angle, mechanical_angle_continuous;
static uint8_t electrical_turns = 0;

static uint8_t enabled = 0;

#define SVPWM_RES 128

// for sv indexes of v0 and v1
static uint8_t v0_index, v1_index;
// duty cycle between v0 and v1
static uint8_t v0v1duty;
static uint8_t svpwm_count;

#define PWM_CKDIV8

void motor_init() 
{
#ifdef TEST
	return;
#endif
	// Timer/Counter1 mode 1:
	//	WGM13:0 = 0001	phase and frequency correct, TOP = 0xFF
	//  CS12:0 = 001	F = F_CPU (18.432 MHz)
	TCCR1A = BV3(COM1A1, COM1B1, WGM10);
#ifdef PWM_CKDIV8
	TCCR1B = _BV(CS11);  // Clk/8 nice whiny PWM
#else
	TCCR1B = _BV(CS10);
#endif
	OCR1A = 0;
	OCR1B = 0;

	// Timer/Counter2 mode 1
	//	WGM21:0 = 01 	phase correct PWM, TOP = 0xFF
	// CS20 no prescaling
	// COM21:0 = 10 - clear OC2 when upcounting, set when downcounting
	//		     11 - set OC2 when upcounting, clear when downcounting
	//TCCR2 = BV3(WGM20, COM21, CS20);
#ifdef PWM_CKDIV8
	TCCR2 = BV3(WGM20, COM21, CS21); // Clk/8 nice whiny PWM
#else
	TCCR2 = BV3(WGM20, COM21, CS20);
#endif

	OCR2 = 0;

	// Make Enable lines outputs, pull down
	PORTC &= ENA123_MASK;
	DDRC |= ENA123_BITVAL;

	// Make PWM lines outputs
	DDRB |= PWM123_BITVAL;

	phase_acc = 0;
	phase_inc = 0;
	pwm_val = 128;
	phase_sinA = PHASE1;
	phase_sinB = PHASE2;
	phase_sinC = PHASE3;
}

void motor_setPWM(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3)
{
#ifndef TEST
	OCR1A = pwm1;
	OCR1B = pwm2;
	OCR2 = pwm3;
#endif
}

void motor_getPWM(uint8_t *pwm1, uint8_t *pwm2, uint8_t *pwm3)
{
	*pwm1 = OCR1A;
	*pwm2 = OCR1B;
	*pwm3 = OCR2;
}

uint8_t motor_getEnable()
{
	uint8_t enas = PORTC & ENA123_BITVAL;
	return ((enas & _BV(ENA1)) >> ENA1) | (((enas & _BV(ENA2)) >> ENA2) << 1) | (((enas & _BV(ENA3)) >> ENA3) << 2);
}

void motor_setEnable(uint8_t enable_bits)
{
#ifndef TEST
	uint8_t ena1 = (enable_bits & 1) << ENA1;
	uint8_t ena2 = ((enable_bits >> 1) & 1) << ENA2;
	uint8_t ena3 = ((enable_bits >> 2) & 1) << ENA3;
	uint8_t ena = ena1 | ena2 | ena3;

	// clear off bits
	uint8_t portc = PORTC & (ENA123_MASK | ena);
	// set on bits
	portc |= ena;
	PORTC = portc;
#endif
}

#if 0

// trapezoidal phases
const int8_t phase_a[] = {  1,  1,  0, -1, -1,  0};
const int8_t phase_b[] = { -1,  0,  1,  1,  0, -1};
const int8_t phase_c[] = {  0, -1, -1,  0,  1,  1};

void motor_tick_trapezoidal() {
	if (!enabled) {
		motor_setEnable(0);
		motor_setPWM(0, 0, 0);
		return;
	}

	phase_acc += phase_inc;
	uint8_t phase_int = phase_acc >> 8;
	if (phase_int > 5) {
		phase_acc -= 6 << 8;
		phase_int = 0;
	}
	uint8_t pwmA = 128 + phase_a[phase_int] * power;
	uint8_t pwmB = 128 + phase_b[phase_int] * power;
	uint8_t pwmC = 128 + phase_c[phase_int] * power;

	uint8_t enable = (phase_a[phase_int] != 0) ? 1 : 0;
	enable |= (phase_b[phase_int] != 0) ? 2 : 0;
	enable |= (phase_c[phase_int] != 0) ? 4 : 0;

	motor_setEnable(enable);
	motor_setPWM(pwmA, pwmB, pwmC);
}

#define SVPWM_HALFSWING 127
// software pwm between two adjacent vectors
void motor_setvector()
{
	if (!enabled) {
		motor_setEnable(0);
		motor_setPWM(0, 0, 0);
		return;
	}

	uint8_t phase_int;

	if (svpwm_count < v0v1duty) {
		phase_int = v0_index;
	} else {
		phase_int = v1_index;
	}
	uint8_t pwmA = SVPWM_HALFSWING + phase_a[phase_int] * SVPWM_HALFSWING;
	uint8_t pwmB = SVPWM_HALFSWING + phase_b[phase_int] * SVPWM_HALFSWING;
	uint8_t pwmC = SVPWM_HALFSWING + phase_c[phase_int] * SVPWM_HALFSWING;

	uint8_t enable = (phase_a[phase_int] != 0) ? 1 : 0;
	enable |= (phase_b[phase_int] != 0) ? 2 : 0;
	enable |= (phase_c[phase_int] != 0) ? 4 : 0;

	motor_setEnable(enable);
	motor_setPWM(pwmA, pwmB, pwmC);

	if (++svpwm_count == SVPWM_RES) {
		svpwm_count = 0;
	}
}

// try to approximate flux vector by switching between adjacent vectors
void motor_tick_sv()
{
	phase_acc += phase_inc;
	if (phase_acc >= FULL_CIRCLE) {
		phase_acc -= FULL_CIRCLE;
		electrical_turns++;
	} else if (phase_acc < 0) {
		phase_sinA += FULL_CIRCLE;
		electrical_turns--;
	}

	// in which of the 6 phases (sectors) we're in
	v0_index = phase_acc / (FULL_CIRCLE/6);
	v1_index = v0_index + 1;
	if (v1_index > 5) {
		v1_index = 0;
	}
	// angle within sector
	int16_t phi = phase_acc % (FULL_CIRCLE/6);
#ifdef TEST	
	//printf("phase_acc=%d sector=%d phi=%d ", phase_acc, v0_index, phi);
#endif
	// map 60º range to 90º range phi = 3/2 * ϕ
	phi += phi >> 1;
#ifdef TEST
	//printf("phi90=%d ", phi);
#endif
	// v0 = cos(phi) = sin(π/2 - ϕ)
	// v1 = sin(phi)
	int8_t v0 = sintab[(PI_HALF - phi) >> 8];
	int8_t v1 = sintab[phi >> 8];
	
	uint16_t scale = 256L * 128 / (v0 + v1);
	v0v1duty = fp_scale(v0, scale);
	// for 8 v0v1duty = v0v1duty >> 4;
	//v0v1duty = v0v1duty >> 1;
	v0v1duty = v0v1duty;


#ifdef TEST
	double base0_x = cos(v0_index * M_PI/3);
	double base0_y = sin(v0_index * M_PI/3);
	double base1_x = cos(v1_index * M_PI/3);
	double base1_y = sin(v1_index * M_PI/3);

	double x0 = base0_x * v0;
	double y0 = base0_y * v0;	
	double x1 = base1_x * v1;
	double y1 = base1_y * v1;
	double mid_x = (x0 + x1) / 2;
	double mid_y = (y0 + y1) / 2;
	printf("0 0 %f %f\n", mid_x, mid_y);
#endif
}

#endif 


uint8_t adjust_scale(uint8_t pwm, uint16_t speed)
{
	// scale = 64  when < 266
	// scale = 256 when > 650
	if (speed < 266) {
		return pwm >> 2;
	} else if (speed < 650) {
		return fp_scale(pwm, 256 - ((650 - speed) >> 1));
	} else {
		return pwm;
	}
}

uint8_t adjust_scale_fullon(uint8_t pwm, uint16_t speed)
{
	return pwm;
}

uint8_t adjust_scale_power(uint8_t pwm, uint16_t speed)
{
	switch (power >> 5) {
	case 0:
		return (pwm >> 2) + (pwm >> 3); // pwm / 4
	case 1:
		return (pwm >> 1); // 1/2
	case 2:
		return (pwm >> 1) + (pwm >> 2); // 3/4
	case 3:
		return pwm;
	}
	return 0;
//	return fp_scale(pwm, power);
}

uint8_t adjust_step(uint8_t pwm, uint16_t speed)
{
	if (speed < 300) {
		return pwm >> 2;
	} else if (speed < 650) {
		return pwm >> 1;
	} 
	return pwm;
}

void motor_tick_sin() 
{
	if (!enabled) {
		motor_setEnable(0);
		motor_setPWM(0, 0, 0);
		return;
	}

	phase_sinA += phase_inc;
	if (phase_sinA >= FULL_CIRCLE) {
		phase_sinA -= FULL_CIRCLE;
		electrical_turns++;
	} else if (phase_sinA < 0) {
		phase_sinA += FULL_CIRCLE;
		electrical_turns--;
	}

	phase_sinB += phase_inc;
	if (phase_sinB >= FULL_CIRCLE) {
		phase_sinB -= FULL_CIRCLE;
	} else if (phase_sinB < 0) {
		phase_sinB += FULL_CIRCLE;
	}

	phase_sinC += phase_inc;
	if (phase_sinC >= FULL_CIRCLE) {
		phase_sinC -= FULL_CIRCLE;
	} else if (phase_sinC < 0) {
		phase_sinC += FULL_CIRCLE;
	}

	uint8_t pwmA, pwmB, pwmC;
	// pwmA = adjust_step(sintab[phase_sinA >> 8] + 128, abs(phase_inc));
	// pwmB = adjust_step(sintab[phase_sinB >> 8] + 128, abs(phase_inc));
	// pwmC = adjust_step(sintab[phase_sinC >> 8] + 128, abs(phase_inc));
	// pwmA = adjust_scale_fullon(sintab[phase_sinA >> 8] + 128, abs(phase_inc));
	// pwmB = adjust_scale_fullon(sintab[phase_sinB >> 8] + 128, abs(phase_inc));
	// pwmC = adjust_scale_fullon(sintab[phase_sinC >> 8] + 128, abs(phase_inc));
	pwmA = adjust_scale_power(sintab[phase_sinA >> 8] + 128, abs(phase_inc));
	pwmB = adjust_scale_power(sintab[phase_sinB >> 8] + 128, abs(phase_inc));
	pwmC = adjust_scale_power(sintab[phase_sinC >> 8] + 128, abs(phase_inc));

	motor_setEnable(7);
	motor_setPWM(pwmA, pwmB, pwmC);

	if (phase_inc > 0) {
		mechanical_angle += phase_inc;
		mechanical_angle_continuous += phase_inc;
		if (mechanical_angle >= FULL_MECHANICAL_CIRCLE) {
			mechanical_angle -= FULL_MECHANICAL_CIRCLE;
		}
	} else if (phase_inc < 0) {
		mechanical_angle += phase_inc;
		mechanical_angle_continuous += phase_inc;
		if (mechanical_angle < 0) {
			mechanical_angle += FULL_MECHANICAL_CIRCLE;
		}
	}
}

void motor_TestPrint()
{
	printf_P(PSTR("power=%d speed=%d pwmA=%d\n"), power, phase_inc, adjust_scale_power(sintab[phase_sinA >> 8] + 128, abs(phase_inc)));
}

void motor_SetSpeed(int16_t speed) 
{
	phase_inc = speed;
}

int16_t motor_GetPower() 
{
	return power;
}

void motor_SetPower(int16_t _power)
{
	power = _power;
}

int32_t motor_getMechanicalAngle()
{
	int32_t result;
	atomic(result = mechanical_angle_continuous);
	return result;
}

int16_t motor_getMechanicalAngleDegrees()
{
	return 360L * motor_getMechanicalAngle() / FULL_MECHANICAL_CIRCLE;
}

void motor_powerOn(uint8_t enable)
{
	enabled = enable;
}

uint8_t motor_getPowerOn()
{
	return enabled;
}

