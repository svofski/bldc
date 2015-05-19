#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"

#include "adns-9800.h"
#include "usrat.h"
#include "xprintf.h"
#include "qenc.h"
#include "motori.h"
#include "util.h"
#include <stdio.h>
#include <stdlib.h>

GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

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

    // Configure pin as output push-pull (led)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
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
__IO uint32_t SysTickCounter = 0;

void SysTick_Handler(void)
{
    SysTickCounter++;
    // if (SysTickDowncounter > 0)
    //      SysTickDowncounter--;
}

// warning: ~ 110 hours uptime
void Delay_us100(__IO uint32_t ms) {    
    uint32_t end = SysTickCounter + ms;
    while (SysTickCounter < end);
}


#define MAX_SPEED 2000
#define MAX_POWER 255
#define MIN_POWER 150

// Leds are PA12 and PB12

static volatile uint32_t tach_tick;
static volatile int32_t tach_qenc;
static volatile uint32_t tach_value;

void updateTach(int qenc)
{
    if (SysTickCounter - tach_tick >= 1000) { // 100mS
        tach_tick = SysTickCounter;
        int qdiff = qenc - tach_qenc;
        tach_qenc = qenc;
        // qdiff == counts per 100mS, qdiff * 10 = counts per second
        tach_value = qdiff * 10 * 100 / 720;
        xprintf("Tach=%d.%02d rps\033[K\r", tach_value / 100, tach_value % 100);
    }    
}

int QEncToElectricDegrees(int enc) {
    if (enc >= 720) 
        enc %= 720;
    else if (enc < 0)
        enc = (enc % 720) + 720;

    // 0..719 = 360 mechanical degrees
    // Motor has 7 electrical turns, 51.4 mech degrees per electric turn
    // mech_angle     = (enc / 2) 
    //                   0..359
    // elec_angle     = (mech_angle * 7) % 360  = (enc * 7 / 2) % 360
    return (enc * 7 / 2) % 360;
}

extern volatile int pid_Kp, pid_Ki, pid_Kd, pid_Kb;

unsigned int lfsr113_Bits (void)
{
   static unsigned int z1 = 12345, z2 = 12345, z3 = 12345, z4 = 12345;
   unsigned int b;
   b  = ((z1 << 6) ^ z1) >> 13;
   z1 = ((z1 & 4294967294U) << 18) ^ b;
   b  = ((z2 << 2) ^ z2) >> 27; 
   z2 = ((z2 & 4294967288U) << 2) ^ b;
   b  = ((z3 << 13) ^ z3) >> 21;
   z3 = ((z3 & 4294967280U) << 7) ^ b;
   b  = ((z4 << 3) ^ z4) >> 12;
   z4 = ((z4 & 4294967168U) << 13) ^ b;
   return (z1 ^ z2 ^ z3 ^ z4);
}

int min3(int a, int b, int c)
{
    if (a < b) 
        return min(a, c);
    else
        return min(b, c);
}

int shortestpath(int from, int to)
{
    int p1 = abs(to - from);
    int p2 = abs(to + 720 - from);
    int p3 = abs(to - 720 - from);
    int m = min3(p1, p2, p3);

    if (m == p2) 
        return to + 720;
    else if (m == p3)
        return to - 720;
    else
        return to;
}

int servo_set = 0;
int servo_timestart;

void BenchRunTo(int setpos) 
{
    xprintf("START\nT=0 Q=%d\n", QEnc_GetPosition());
    servo_set = setpos;
    servo_timestart = SysTickCounter;
    Motori_SetPosition(servo_set);
}

int main(void)
{
    int motorSpeed = 0;

    SysTick_Config(SystemCoreClock / 10000);
    NVIC_SetPriority(SysTick_IRQn, 14);

    RCC_Configuration();
    GPIO_Configuration();

    USART_Config();

    xputchar('@');

    xprintf("\rON COUCHE ENSEMBLE CE SOIR @%x\n", __get_MSP());

#ifdef WITH_ADNS9800
    // Configure SPI for ADNS-9800
    ADNS9800_Config();
    ADNSInitialize();
#endif
    // Quadrature encoder
    QEnc_Init();    

    // Motor
    Motori_Init();
    Motori_SetSpeed(motorSpeed);


    GPIOB->ODR ^= GPIO_Pin_12;
    int lastqenc = 0;
    while(1)
    {
        int qenc = QEnc_GetPosition();

        if (xavail()) {
            char c = xgetchar();
            switch(c) {
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            // case '6':
            // case '7':
            // case '8':
            // case '9':
                BenchRunTo(720 / 6 * (int)(c - '0'));
                break;
            case '6':
            case '7':
            case '8':
            case '9':
                BenchRunTo(720 * (int)(c - '6') / 96);
                break;
            case '!':
                BenchRunTo(servo_set = 262 - 0);
                break;
            case '@':
                BenchRunTo(servo_set = -158 /*+ 57*/);
                break;
            case '\\':
                {
                    uint32_t start = SysTickCounter;

                    for (int i = 0; i < 96 && !xavail(); i++) {
                        char c = lfsr113_Bits() % 96;
                        qenc = QEnc_GetPosition();
                        int pos = 720 * c / 96;
                        xprintf("target pos=%d ", pos);
                        pos = shortestpath(qenc, pos);

                        // if (pos - qenc < -360) {
                        //     // ccw
                        //     pos += 720;
                        // } else if (pos - qenc > 360) {
                        //     pos -= 720;
                        // } 
                        xprintf("goto pos=%d\n", pos);
                        Motori_SetPosition(pos);
                        while(QEnc_GetPosition() != pos);
                        extern volatile int velocity_counter;
                        while(velocity_counter < 200);
                        GPIOC->BSRR |= GPIO_Pin_0;
                        Delay_us100(5);
                        GPIOC->BRR ^= GPIO_Pin_0;
                    }

                    int temps = (SysTickCounter - start) / 10000;
                    xprintf("Elapsed time=%d cps=%d.%d\n", temps, (960/temps)/10, (960/temps)%10);
                    Motori_Enable(0);
                }
                break;
            case '|':
                {
                    uint32_t start = SysTickCounter;

                    for (int i = 0; i < 96 && !xavail(); i++) {
                        char c = i;
                        int pos = 720 * c / 96;
                        qenc = QEnc_GetPosition();
                        pos = shortestpath(qenc, pos);
                        xprintf("goto pos=%d\n", pos);
                        Motori_SetPosition(pos);
                        while(QEnc_GetPosition() != pos);
                        extern volatile int velocity_counter;
                        while(velocity_counter < 200);
                        GPIOC->BSRR |= GPIO_Pin_0;
                        Delay_us100(5);
                        GPIOC->BRR ^= GPIO_Pin_0;
                    }

                    int temps = (SysTickCounter - start) / 10000;
                    xprintf("Elapsed time=%d cps=%d.%d\n", temps, (960/temps)/10, (960/temps)%10);
                    Motori_Enable(0);
                }
                break;
            case 'c':
                xprintf("\nCalibrate: setting zero reference: ");
                servo_set = 0;
                motorSpeed = 0;
                Motori_Reset();
                Motori_SetPower(MAX_POWER);
                Motori_Enable(1);
                Delay_us100(2500);
                QEnc_Reset();
                Motori_SetPower(MIN_POWER);
                xprintf("Done\n");
                break;
            case 'a':
                pid_Kp -= 10;
                xprintf("\nP=%d\n", pid_Kp);
                break;
            case 's':
                pid_Kp += 10;
                xprintf("\nP=%d\n", pid_Kp);
                break;
            case 'd':
                pid_Ki -= 10;
                xprintf("\nI=%d\n", pid_Ki);
                break;
            case 'f':
                pid_Ki += 10;
                xprintf("\nI=%d\n", pid_Ki);
                break;
            case 'D':
                pid_Ki -= 1;
                xprintf("\nI=%d\n", pid_Ki);
                break;
            case 'F':
                pid_Ki += 1;
                xprintf("\nI=%d\n", pid_Ki);
                break;
            case 'g':
                pid_Kd -= 1;
                xprintf("\nD=%d\n", pid_Kd);
                break;
            case 'h':
                pid_Kd += 1;
                xprintf("\nD=%d\n", pid_Kd);
                break;
            case 'j':
                pid_Kb -= 1;
                xprintf("\nB=%d\n", pid_Kb);
                break;
            case 'k':
                pid_Kb += 1;
                xprintf("\nB=%d\n", pid_Kb);
                break;
            case 'e':
                Motori_Enable(1);
                break;
            case '`':
            case 033:
                Motori_Enable(0);
                break;
            case '+':
                Motori_SetPower(Motori_GetPower() + 10);
                break;
            case '-':
                Motori_SetPower(Motori_GetPower() - 10);
                break;
            case 'z':
                Motori_SetSpeed(Motori_GetSpeed() - 1);
                break;
            case 'x':
                Motori_SetSpeed(Motori_GetSpeed() + 1);
                break;
            case 'Z':
                Motori_SetSpeed(Motori_GetSpeed() - 10);
                break;
            case 'X':
                Motori_SetSpeed(Motori_GetSpeed() + 10);
                break;
            // case 'F':
            //     if (ADNSFrameCapture() == 0) {
            //         xprintf("DUMP\n");
            //         ADNSFrameDump();
            //     }
            //     break;
            case 'H':
                // Trap to 4
                //*((uint8_t*)(uint8_t)0x20002000) = 0x5a;
                break;
            }

            xprintf("\nEnabled=%d Speed=%d Power=%d\n", Motori_GetEnabled(), Motori_GetSpeed(), Motori_GetPower());
        }

#ifdef WITH_ADNS9800
        ADNSDMAPoll();
        if (ADNSCheckMotion()) {
            xprintf("%d %d SQUAL=%d FPS=%d\n", ADNSGetX(), ADNSGetY(), ADNSGetSQUAL(), ADNSGetFramerate());
            xflush();
        }
#endif
        //Delay_us100(10);
        if (qenc != lastqenc) {
            extern int servostate, electric_error, electric_enc, diff_error, start_error, integral_error, servo_velocity, servo_error, phaseA;
            extern volatile int velocity_counter;

            xprintf("\rT=%d Q=%d vel=%d S=%d\n", SysTickCounter - servo_timestart, 
                    qenc, servo_velocity, velocity_counter);

            // updateTach(qenc);
            //int curangle = 360 * (qenc % 103) / 103;
            // __disable_irq();
            //int gotSpeed = Motori_GetSpeed();
            // __enable_irq();
            // xprintf("\rT=%d Q=%d S=%d EA=%d serr=%d servo_error=%d state=%d degrees=%d | eerr=%d phaseA=%d start_err=%d vel=%d\n", SysTickCounter - servo_timestart, 
            //         qenc, gotSpeed, Motori_GetElectricalAngle(), servo_set - qenc, servo_error, servostate,
            //         curangle, electric_error, phaseA, start_error, servo_velocity);
            // xprintf("\rQ=%d S=%d EA=%d servo_err=%d state=%d degrees=%d | eerr=%d int_err=%d diff_err=%d start_err=%d", qenc, gotSpeed, Motori_GetElectricalAngle(), servo_set - qenc, servostate,
            //             curangle, electric_error, integral_error, diff_error, start_error);
            // xprintf(" Tach=%d.%02d rps Time=%d\033[K\n", tach_value/100, tach_value%100, SysTickCounter - servo_timestart);
            lastqenc = qenc;
        }

    }
}
