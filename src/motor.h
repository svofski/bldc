#pragma once

#define FULL_CIRCLE (256*192L)
#define PI_HALF (FULL_CIRCLE/4)
#define ELECTROMECH_RATIO 6
#define FULL_MECHANICAL_CIRCLE (FULL_CIRCLE*ELECTROMECH_RATIO) 

void motor_init();
void motor_setPWM(uint8_t pwm1, uint8_t pwm2, uint8_t pwm3);
void motor_getPWM(uint8_t *pwm1, uint8_t *pwm2, uint8_t *pwm3);
uint8_t motor_getEnable();
void motor_setEnable(uint8_t enable_bits);
void motor_tick_trapezoidal();
void motor_tick_sin();
void motor_tick_sv(void);
void motor_setvector();
void motor_SetSpeed(int16_t speed);
int16_t motor_GetPower();
void motor_SetPower(int16_t power);
int32_t motor_getMechanicalAngle();
int16_t motor_getMechanicalAngleDegrees();
void motor_powerOn(uint8_t on);
uint8_t motor_getPowerOn(void);
void motor_TestPrint();