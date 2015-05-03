#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "qenc.h"
// Quadrature encoder on pins PB3, PB4

extern GPIO_InitTypeDef GPIO_InitStructure;

void QEnc_Init(void)
{
	// Enable alternate port functions clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// // Enable EXTI on PB3, PB4
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PB;  // EXTI3 = PB
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PB;  // EXTI4 = PB

	// Enable interrupts
	EXTI->IMR |= EXTI_IMR_MR3 | EXTI_IMR_MR4;    // Unmask interrupts on lines 3 and 4
	EXTI->RTSR |= EXTI_RTSR_TR3 | EXTI_RTSR_TR4; // Rising trigger selection register: enable lines 3 and 4
	EXTI->FTSR |= EXTI_FTSR_TR3 | EXTI_FTSR_TR4; // Falling trigger selection register: enable lines 3 and 4

	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
}

//    ____           ____
//  _|	  |_________|    |___
//       ____      ____
//  ____|    |____|    |___
// 0  1  1  0  0   0  1  1  0  lsb
// 0  0  1  1  0   1  1  0  0  msb
// 0  1  3  2  0   2  3  1  0  int
// ---- CW ----+---- CCW ----

static uint8_t dirtab[4][4] = {
//new 0   1   2   3    // old
	{ 0, -1, +1,  0},  // 0
	{+1,  0,  0, -1},  // 1
	{-1,  0,  0, +1},  // 2
	{ 0, +1, -1,  0}}; // 3

static int state = 0;
static int position = 0;
static int errors = 0;

static inline void qenc_tick()
{
	uint8_t input = (GPIOB->IDR >> 3) & 3;

	if (state == input) {
		return;
	}

	int8_t dir = dirtab[state][input];
	if (dir == 0) {
		errors++;
	}
	state = input;
	position += dir;
}

int QEnc_GetPosition(void)
{
	return position;
}

void EXTI3_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR3;
    qenc_tick();
    GPIOA->ODR ^= GPIO_Pin_12;
}

void EXTI4_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR4;
    qenc_tick();
    GPIOB->ODR ^= GPIO_Pin_12;
}