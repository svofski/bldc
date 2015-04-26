#include <stdint.h>
#include <stdio.h>
#include "motor.h"

int main()
{
	motor_setEnable(7);
	motor_SetSpeed(192);
	motor_powerOn(1);
	for(int i = 0; i < FULL_CIRCLE / 192; i++) {
		motor_tick_sv();
	}

	return 0;
}