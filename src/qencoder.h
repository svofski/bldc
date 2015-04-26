#pragma once

#include <inttypes.h>

#define FULL_REV_COUNT 1650L

void qencoder_Init(void);
//void qencoder_Poll(void);
int16_t qencoder_GetPosition(void);
uint16_t qencoder_GetErrorCount();