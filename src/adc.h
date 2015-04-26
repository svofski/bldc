#pragma once

void adc_init();
void adc_start();
void adc_stop();
void adc_getValues(int16_t *a, int16_t *b);
int16_t get_hallTransitionCount();
int16_t pop_hallTransitionCount();
int16_t drop_hallTransitionCount();
