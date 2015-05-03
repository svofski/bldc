#pragma once

#define DEBUG_USARTN 3

void USART_Config(void);
int xputchar(int c);
int xavail(void);
int xgetchar(void);
void xflush(void);
