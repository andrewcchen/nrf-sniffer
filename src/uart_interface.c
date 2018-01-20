#include <stdlib.h>

#include "nrf51/nrf51.h"

#include "config.h"

#include "uart_interface.h"

void uart_setup(void) {
	NRF_GPIO->OUTSET = 1 << UART_TX_PIN;
	NRF_GPIO->PIN_CNF[UART_TX_PIN] =
		GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos |
		GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;

	NRF_GPIO->PIN_CNF[UART_RX_PIN] =
		GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos |
		GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;

	NRF_UART0->PSELTXD = UART_TX_PIN;
	NRF_UART0->PSELRXD = UART_RX_PIN;
	NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud1M;
	NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;
	NRF_UART0->TASKS_STARTTX = 1;
}

void uart_write(const uint8_t *buf, size_t count) {
	for (size_t i = 0; i < count; i++) {
		NRF_UART0->TXD = buf[i];
		while (NRF_UART0->EVENTS_TXDRDY == 0);
		NRF_UART0->EVENTS_TXDRDY = 0;
	}
}
