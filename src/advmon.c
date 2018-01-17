/* Copyright (c) 2018, Andrew Chen <andrew.chuanye.chen@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>

#include "nrf51/nrf51.h"

#define UART_TX_PIN 0
#define UART_RX_PIN 1

static void clock_setup(void) {
	NRF_CLOCK->XTALFREQ = CLOCK_XTALFREQ_XTALFREQ_16MHz;

	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;

	// wait until 32Mhz crystal has started
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

	// wait until 32.768khz crystal has started
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}

static inline void uart_putchar(uint8_t c) {
	while (NRF_UART0->EVENTS_TXDRDY == 0);
	NRF_UART0->EVENTS_TXDRDY = 0;
	NRF_UART0->TXD = c;
}

static void uart_setup(void) {
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
	NRF_UART0->TXD = 0;
	uart_putchar(0);
	uart_putchar(0);
	uart_putchar(0);
	uart_putchar(0);
	uart_putchar(0);
}

static void timer_setup(void) {
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER0->PRESCALER = 4; // div 16 -> 1Mhz
	NRF_TIMER0->TASKS_START = 1;
}

static inline uint32_t timer_get(void) {
	NRF_TIMER0->TASKS_CAPTURE[1] = 1;
	return NRF_TIMER0->CC[1];
}

static void radio_setup(void) {
	NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit;

	NRF_RADIO->PCNF0 =
		8 << RADIO_PCNF0_LFLEN_Pos |
		1 << RADIO_PCNF0_S0LEN_Pos |
		0 << RADIO_PCNF0_S1LEN_Pos;

	NRF_RADIO->PCNF1 =
		254 << RADIO_PCNF1_MAXLEN_Pos |
		3 << RADIO_PCNF1_BALEN_Pos |
		RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos |
		RADIO_PCNF1_WHITEEN_Msk;

	NRF_RADIO->CRCCNF =
		RADIO_CRCCNF_LEN_Three |
		RADIO_CRCCNF_SKIPADDR_Msk;
	NRF_RADIO->CRCPOLY = 0x0000065B;

	if ((NRF_FICR->OVERRIDEEN & FICR_OVERRIDEEN_BLE_1MBIT_Msk) == 0) {
		NRF_RADIO->OVERRIDE0 = NRF_FICR->BLE_1MBIT[0];
		NRF_RADIO->OVERRIDE1 = NRF_FICR->BLE_1MBIT[1];
		NRF_RADIO->OVERRIDE2 = NRF_FICR->BLE_1MBIT[2];
		NRF_RADIO->OVERRIDE3 = NRF_FICR->BLE_1MBIT[3];
		NRF_RADIO->OVERRIDE4 = NRF_FICR->BLE_1MBIT[4] | RADIO_OVERRIDE4_ENABLE_Msk;
	}

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
	NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
	NVIC_SetPriority(RADIO_IRQn, 0 << 5);
	NVIC_EnableIRQ(RADIO_IRQn);

	NRF_RADIO->PREFIX0 = 0x8E;
	NRF_RADIO->BASE0 = 0x89BED600;
	NRF_RADIO->RXADDRESSES = 1 << 0;

	NRF_RADIO->FREQUENCY = 2;
	NRF_RADIO->DATAWHITEIV = 37;
	NRF_RADIO->CRCINIT = 0x555555;
}

#define BUF_COUNT 32
static volatile size_t tail;
static volatile uint32_t buf[BUF_COUNT][65];

int main(void) {
	clock_setup();
	uart_setup();
	timer_setup();
	radio_setup();
	
	NRF_RADIO->PACKETPTR = (uint32_t)(&buf[tail][1]);
	NRF_RADIO->TASKS_RXEN = 1;

	size_t head = 0;

	while (1) {
		if (head != tail) {
			uint8_t *ptr = (uint8_t *)buf[head];
			size_t len = ptr[5] + 6;
			for (size_t i = 0; i < len; i++) {
				uart_putchar(ptr[i]);
			}
			head = (head + 1) % BUF_COUNT;
		}
	}
}

void RADIO_IRQHandler(void);

void RADIO_IRQHandler(void) {
	NRF_RADIO->EVENTS_END = 0;

	if (NRF_RADIO->CRCSTATUS != 0) {
		int oldtail = tail;
		int newtail = (oldtail + 1) % BUF_COUNT;

		NRF_RADIO->PACKETPTR = (uint32_t)(&buf[newtail][1]);
		NRF_RADIO->TASKS_START = 1;

		buf[oldtail][0] = timer_get();
		tail = newtail;
	} else {
		NRF_RADIO->TASKS_START = 1;
	}
}
