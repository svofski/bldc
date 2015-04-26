#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usrat.h"
#include "util.h"
#include "motor.h"
#include "adc.h"
#include "qencoder.h"

volatile uint16_t seconds = 0;
volatile uint16_t tenthseconds = 0;
volatile uint16_t timer0_count;
volatile uint32_t timer0_longcount = 0;
volatile uint16_t timer0_tenthscount = 0;
volatile int16_t rps;
volatile int32_t measured_speed; 	// this many 1/9000s of a second per hall transition (1/2 of revolution) 

#define USE_ENCODER

void led_init() 
{
    DDRC |= _BV(3);
}

void led_set(uint8_t on) 
{
   	PORTC = (PORTC & ~_BV(3)) | (on ? _BV(3) : 0);
}

//volatile int skip = 0;

void timer0_init()
{
    TIMSK |= _BV(TOIE0);   	// enable Timer0 overflow interrupt
    TCCR0 = _BV(CS01);      // F_CPU/8 = 2304000 Hz
    TCNT0 = 0;				// /256 = 9000 Hz
}

int16_t ServoGoal;
int16_t ServoSpeed;
int16_t ServoPower;
int16_t ServoSpeedP;

void servo_Init()
{
	ServoGoal = 0;
	ServoSpeed = 0;
	ServoPower = 127;
	//ServoSpeedP = 40;
    ServoSpeedP = 80;
    //ServoSpeedP = 128;
}

#define SERVO_MAX_SPEED 500
// #define ACCEL_THRES1404
// void servo_Tick()
// {
// #ifdef USE_ENCODER
// 	int16_t angle = qencoder_GetPosition();
// #else
//     int16_t angle = motor_getMechanicalAngleDegrees();
// #endif
// 	int16_t error = ServoGoal - angle;

// 	int16_t error_amp = fp_scale16(error, ServoSpeedP);
// 	int16_t speed_diff = error_amp - ServoSpeed;
// 	if (speed_diff > 0) {
//         if (speed_diff > ACCEL_THRESH) {
//             ServoPower = 127;
//             ServoSpeed += ACCEL_THRESH;
//         } else {
//             ServoPower = 31;
//             ServoSpeed = error_amp;
//         }
//         if (ServoSpeed > SERVO_MAX_SPEED) ServoSpeed = SERVO_MAX_SPEED;
// 	} else if (speed_diff < 0) {
//         if (speed_diff < ACCEL_THRESH) {
//             ServoPower = 127;
//             ServoSpeed -= ACCEL_THRESH;
//         } else {
//             ServoPower = 31;
//             ServoSpeed = error_amp;
//         }
//         if (ServoSpeed < -SERVO_MAX_SPEED) ServoSpeed = -SERVO_MAX_SPEED;
// 	}
//     if (ServoSpeed == SERVO_MAX_SPEED || ServoSpeed == -SERVO_MAX_SPEED) putchar('!');
// //	printf_P(PSTR("S: error=%d error_amp=%d speed=%d power=%d "), error, error_amp, ServoSpeed, ServoPower);

// 	motor_SetSpeed(ServoSpeed);
// 	motor_SetPower(ServoPower);
// }

int16_t last_error = 0;
int32_t integral_error = 0;
int16_t pid_Kp = 80;
int16_t pid_Ki = 120;
int16_t pid_Kd = 80;

const int32_t MAX_E = 65536*10;

void servo_PID()
{
#ifdef USE_ENCODER
    int16_t angle = qencoder_GetPosition();
#else
    int16_t angle = motor_getMechanicalAngleDegrees();
#endif
    int16_t error = ServoGoal - angle;
    
    integral_error = (int32_t)integral_error + (int32_t)error;
    if (integral_error > MAX_E) integral_error = MAX_E;
    if (integral_error < -MAX_E) integral_error = -MAX_E;
    int16_t diff_error = error - last_error;

    int32_t control = error * pid_Kp + (integral_error >> 8) * pid_Ki + diff_error * pid_Kd;
    ServoSpeed = control >> 8;
    if (ServoSpeed > SERVO_MAX_SPEED) {
        ServoSpeed = SERVO_MAX_SPEED;
    } else if (ServoSpeed < -SERVO_MAX_SPEED) {
        ServoSpeed = -SERVO_MAX_SPEED;
    }
    motor_SetSpeed(ServoSpeed);
    uint8_t power = abs(ServoSpeed);
    if (power < 5) {
        power = 31;
    } else if (power < 10) {
        power = 63;
    } else if (power < 15) {
        power = 95;
    } else {
        power = 127;
    }

    motor_SetPower(power);
}

void servo_SetGoal(int16_t goal) 
{
	ServoGoal = goal;
}

ISR(TIMER0_OVF_vect)
{
	TCNT0 = 0;
	++timer0_longcount;
	int16_t transitions = drop_hallTransitionCount();
	if (transitions > 0) {
		measured_speed = timer0_longcount / transitions;
		timer0_longcount = 0;
	}
	if (++timer0_count == 9000) {
		timer0_count = 0;
		seconds++;
		rps = drop_hallTransitionCount();
	}
	if (++timer0_tenthscount == 10) {
		timer0_tenthscount = 0;
		tenthseconds++;
	}
	motor_tick_sin();
    //motor_tick_sv();
}

void test_PrintPid()
{
    printf_P(PSTR("Kp=%d Ki=%d Kd=%d\n"), pid_Kp, pid_Ki, pid_Kd);
}

void main() {
	uint8_t pwm1, pwm2, pwm3;

	int16_t speed = 8;
	int16_t goal = 0;
    uint8_t loop = 0;
	uint16_t lastsecond = 0;
	int16_t lastangle = 0;

    usart_init(F_CPU/16/115200-1);
    printf_P(PSTR("\033[2J\033[HQUI NE RISQUE RIEN N'A RIEN %ld %s %02x\n"), F_CPU, BUILDNUM, MCUCR);

    led_init();
    motor_init();

    sei();

	timer0_init();
	// adc_init();
	// adc_start();
	qencoder_Init();
	servo_Init();

    for(int i = 0;; i++) {    	
    	if (uart_available()) {
	    	motor_getPWM(&pwm1, &pwm2, &pwm3);
	    	int c;
    		switch (c = getchar()) {
    			case '1':	
    			case '2':
    			case '3':
    			case '4':
    			case '5':
    			case '6':
    			case '7':
    			case '8':
    			case '9':
    			case '0':
                    #ifdef USE_ENCODER
    				servo_SetGoal(FULL_REV_COUNT * (c - '0') / 10);
                    #else                    
                    servo_SetGoal(360 * (c - '0') / 10);
                    #endif
    				break;
    			case 'g':
    				motor_SetSpeed(speed = 0);
    				motor_powerOn(!motor_getPowerOn());
    				goal = 0;
    				break;
    			case 'z':
    				goal--;
    				break;
    			case 'x':
    				goal++;
    				break;
    			case 'Z':
    				goal -= 10;
    				break;
    			case 'X':    				
    				goal += 10;
    				break;
    			case 'r':
    				goal = -goal;
    				break;
                case 'l':
                    loop = !loop;
                    break;
                case ' ':
                    motor_TestPrint();
                    printf_P(PSTR("ServoGoal=%d ServoSpeed=%d encoder=%d integral_error=%ld\n"), 
                            ServoGoal, ServoSpeed, qencoder_GetPosition(),
                            integral_error);
                    break;
                case 'P':
                    pid_Kp++;
                    test_PrintPid();
                    break;
                case 'p':
                    pid_Kp--;
                    test_PrintPid();
                    break;
                case 'I':
                    pid_Ki++;
                    test_PrintPid();
                    break;
                case 'i':
                    pid_Ki--;
                    test_PrintPid();
                    break;
                case 'D':
                    pid_Kd++;
                    test_PrintPid();
                    break;
                case 'd':
                    pid_Kd--;
                    test_PrintPid();
                    break;
                case '+':
                    motor_SetPower(motor_GetPower() + 4);
                    motor_TestPrint();
                    break;
                case '-':
                    motor_SetPower(motor_GetPower() - 4);
                    motor_TestPrint();
                    break;
    			// case '9':
    			// 	goal = 1700;    				
    			// 	if (speed < 0) {
    			// 		goal = -goal;
    			// 	}
    			//	break;
    		}
    		//printf_P(PSTR("enable=%x pwm=[%d, %d, %d] seconds=%d t0=%d\n"), motor_getEnable(), pwm1, pwm2, pwm3, seconds, timer0_count);
    	} else {
            //motor_setvector();
            // if (speed < goal) {
            //   motor_SetSpeed(speed += (abs(goal - speed) > 100) ? 10 : 1);
            // } else if (speed > goal) {
            //   motor_SetSpeed(speed -= (abs(speed - goal) > 100) ? 10 : 1);
            // }                
            // Servo Loop
            if (tenthseconds > lastsecond) {
                lastsecond = tenthseconds;
                servo_PID();
                if (loop) {
                    servo_SetGoal(FULL_REV_COUNT * goal / 360);
                    if (++goal == 360) goal= 0;
                }
            }
    	}
    }
}
