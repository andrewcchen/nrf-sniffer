#include <stdint.h>

#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

// Definitions for UART interface to PC

enum ble_packet_direction {
	SLAVE_TO_MASTER = 0,
	MASTER_TO_SLAVE = 1,
};

struct uart_output_packet {
	uint32_t timestamp;
	struct { // little endian is assumed
		unsigned int crc_match:1;
		unsigned int direction:1;
		unsigned int _reserved:6;
	} __attribute__((packed)) flags;
	uint8_t channel_id;
	uint16_t _reserved;

	uint8_t pdu[];
} __attribute__((packed));


void uart_setup(void);
void uart_write(const uint8_t *buf, size_t count);

#endif // UART_INTERFACE_H
