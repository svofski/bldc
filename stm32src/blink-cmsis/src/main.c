#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "adns-9800.h"
#include "usrat.h"
#include "xprintf.h"
#include "qenc.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure;

void delay(volatile unsigned long delay)
{
    while(delay) delay--;
}

void RCC_Configuration(void)
{
    // Periph clock = HCLK / 2
    RCC_PCLK2Config(RCC_HCLK_Div2); 

    // Enable GPIO clock for GPIOA and GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
}


void GPIO_Configuration(void)
{
    // Configure pin as output push-pull (led)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure pin as output push-pull (led)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// void NVIC_Configuration(void)
// {
//     NVIC_InitTypeDef NVIC_InitStructure;

//     /* 1 bit for pre-emption priority, 3 bits for subpriority */
//     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

//     // /* Configure and enable SPI_MASTER interrupt -------------------------------*/
//     // NVIC_InitStructure.NVIC_IRQChannel = SPI_MASTER_IRQn;
//     // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//     // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//     // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     // NVIC_Init(&NVIC_InitStructure);
// }

__IO uint32_t SysTickDowncounter = 0;

void SysTick_Handler(void)
{
    if (SysTickDowncounter > 0)
         SysTickDowncounter--;
}

void Delay_us100(__IO uint32_t ms) {
    SysTickDowncounter = ms;
    while (SysTickDowncounter);
}


// PA12 and PB12

int main(void)
{
    SysTick_Config(SystemCoreClock / 10000);
    NVIC_SetPriority(SysTick_IRQn, 14);

    RCC_Configuration();
    GPIO_Configuration();

    USART_Config();

    xputchar('@');

    xprintf("\rON COUCHE ENSEMBLE CE SOIR @%x\n", __get_MSP());

    // Configure SPI for ADNS-9800
    ADNS9800_Config();
    ADNSInitialize();

    // Quadrature encoder
    QEnc_Init();    

    GPIOB->ODR ^= GPIO_Pin_12;
    int a = 0;
    int lastqenc = 0;
    while(1)
    {

        if (xavail()) {
            char c;
            switch(c = xgetchar()) {
            case 'f':
                if (ADNSFrameCapture() == 0) {
                    xprintf("DUMP\n");
                    ADNSFrameDump();
                }
                break;
            case 'H':
                // Trap to 4
                *((uint8_t*)(uint8_t)0x20002000) = 0x5a;
                break;
            }
        }

        ADNSDMAPoll();
        if (ADNSCheckMotion()) {
            xprintf("%d %d SQUAL=%d FPS=%d\n", ADNSGetX(), ADNSGetY(), ADNSGetSQUAL(), ADNSGetFramerate());
            xflush();
        }
        // if (ADNSDMAPoll()) {
        //     xprintf("%d %d\n", ADNSGetX(), ADNSGetY());
        //     xflush();
        // }

        a = (a + 1) % 10;
        // GPIOA->ODR ^= GPIO_Pin_12;
        // GPIOB->ODR ^= GPIO_Pin_12;
        Delay_us100(40);
        //Delay_us100(500);
        //xprintf("%04x %c %c\n", GPIOB->IDR, ((GPIOB->IDR & 16) != 0) ? '*' : '_', ((GPIOB->IDR & 8) != 0) ? '*' : '_');
        int qenc = QEnc_GetPosition();
        if (qenc != lastqenc) {
            xprintf("Q=%d\n", qenc);    
            lastqenc = qenc;
        }
        
    }
}
