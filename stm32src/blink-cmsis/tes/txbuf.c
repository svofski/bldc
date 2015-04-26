#include <inttypes.h>
#include <stdio.h>

#define BUFFER_SIZE   4

volatile uint8_t TxBuffer[BUFFER_SIZE];
volatile uint8_t RxBuffer[BUFFER_SIZE];

volatile uint8_t tx_head, tx_tail;
volatile uint8_t rx_head, rx_tail;
volatile int USART1_IRQ_COUNT = 0;

int irq_enable = 0;
int cycle = 0;

int uart_reg = -1;
int uart_irq = 0;
int uart_busy = 0;

void idle(void);

static inline uint8_t tx_count()
{
	return (tx_head >= tx_tail) ? tx_head - tx_tail : tx_head + BUFFER_SIZE - tx_tail;
}


void emit()
{
	if (tx_head != tx_tail) {		
		if (++tx_tail == BUFFER_SIZE) {
			tx_tail = 0;
		}
		uart_reg = TxBuffer[tx_tail];
		irq_enable = 1;
	} else {
		irq_enable = 0;
	}
}

int xputchar(int ch)
{	
	if (ch == '\n') {
		xputchar('\r');
	}
	// wait until buffer has room
	uint8_t count;
	while ((count = tx_count()) == BUFFER_SIZE - 1) {
		putchar('i');
		idle();
	}

	// put character in buffer
	if (++tx_head == BUFFER_SIZE) {
		tx_head = 0;
	}
	TxBuffer[tx_head] = (uint8_t) ch;

	// if the buffer was empty, initiate transfer
	if (count == 0) {
		emit();
	}
	return ch;
}

void USART1_IRQHandler(void)
{
	// if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
	// 	/* Read one byte from the receive data register */
	// 	RxBuffer[RxCounter++] = (USART_ReceiveData(USART1) & 0x7F);

	// 	if(RxCounter == NbrOfDataToRead) {
	// 	  /* Disable the USART1 Receive interrupt */
	// 	  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	// 	}
	// }
	putchar('!');
	USART1_IRQ_COUNT++;
	uart_irq = 0;
	// if transmit register empty
	emit();
}

void uart() {
	if (uart_reg != -1) {
		if (!uart_busy) {
			uart_busy = 15;
		}
	}
	if (uart_busy > 0) {
		if (--uart_busy == 0) {
			putchar(uart_reg);
			uart_irq = 1;
		}
	}

}

void idle(void) {
	uart();
}

main() {
	for (; cycle < 100; cycle++) {
		xputchar('0' + (cycle % 10));
		idle();
	}
}
