#pragma once

void encoder_setHallsCalibration(uint16_t h1_min, uint16_t h1_max, uint16_t h2_min, uint16_t h2_max);
int16_t encoder_getAngle(uint16_t h1, uint16_t h2);