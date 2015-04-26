#pragma once

void USART_Config(void);
void USART1_IRQHandler(void);
int xputchar(int c);
int xavail(void);
int xgetchar(void);
void xflush(void);