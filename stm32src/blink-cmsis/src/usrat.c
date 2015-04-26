#include <inttypes.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usrat.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define BUFFER_SIZE   32

volatile uint8_t TxBuffer[BUFFER_SIZE];
volatile uint8_t RxBuffer[BUFFER_SIZE];

volatile uint8_t tx_head, tx_tail;
volatile uint8_t rx_head, rx_tail;

static void USART_NVIC_Configuration(void);

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// USART1 Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Configure the GPIO ports( USART1 Transmit and Receive Lines)
	// Configure the USART1_Tx as Alternate function Push-Pull
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure the USART1_Rx as input floating 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 115200 8 N 1 -crtscts
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	USART_ClearITPendingBit(USART1, USART_IT_TXE);

    // Enable RX interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_NVIC_Configuration();

	tx_head = rx_head = 0;
	tx_tail = rx_tail = 0;

    // Enable the USART1
    USART_Cmd(USART1, ENABLE);	
}

static void USART_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static inline uint8_t tx_count()
{
	return (tx_head >= tx_tail) ? tx_head - tx_tail : tx_head + BUFFER_SIZE - tx_tail;
}

static inline uint8_t rx_count()
{
	return (rx_head >= rx_tail) ? rx_head - rx_tail : rx_head + BUFFER_SIZE - rx_tail;
}

void xflush() {
	while(tx_count() != 0);
}

int xavail() {
	return rx_count();
}

int xgetchar()
{
	if (xavail()) {
		if (++rx_tail == BUFFER_SIZE) {
			rx_tail = 0;
		}
		return RxBuffer[rx_tail];
	} else {
		return -1;
	}
}

void emit()
{
	do {
		if (tx_head != tx_tail) {
			// if there's a send in progress, exit to let it finish 
			if ((USART1->SR & USART_SR_TXE) == 0) {  // USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET
				break;
			}
			if (++tx_tail == BUFFER_SIZE) {
				tx_tail = 0;
			}
			USART1->DR = TxBuffer[tx_tail]; 	// USART_SendData(USART1, TxBuffer[tx_tail]);			
			USART1->CR1 |= USART_CR1_TXEIE; 	// USART_ITConfig(USART1, USART_IT_TXE, ENABLE);			
		} else {			
			USART1->CR1 &= ~USART_CR1_TXEIE; 	// USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	} while(0);
}

int xputchar(int ch)
{	
	if (ch == '\n') {
		xputchar('\r');
	}

	// wait until buffer has room
	uint8_t count;
	while ((count = tx_count()) == BUFFER_SIZE - 1);

	__disable_irq();
	{
		// put character in buffer
		if (++tx_head == BUFFER_SIZE) {
			tx_head = 0;
		}
		TxBuffer[tx_head] = (uint8_t) ch;

		// if the buffer was empty, initiate transfer
		if (count == 0) {
		 	emit();
		}
	}
	__enable_irq();
	return ch;
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		if (++rx_head == BUFFER_SIZE) {
			rx_head = 0;
		}
	 	RxBuffer[rx_head] = USART_ReceiveData(USART1);
	 }

	// if transmit register empty
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {   
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
		emit();
	}
}
