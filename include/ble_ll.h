#include <stdint.h>
#include <stdbool.h>

// Definitions for BLE link layer packets

enum ble_pdu_type {
	ADV_IND = 0,
	ADV_DIRECT_IND = 1,
	ADV_NONCONN_IND = 2,
	SCAN_REQ = 3,
	SCAN_RSP = 4,
	CONNECT_IND = 5,
	ADV_SCAN_IND = 6,
};

struct ble_advertising_pdu {
	unsigned int pdu_type:4;
	unsigned int _reserved:1;
	unsigned int chsel:1;
	unsigned int txadd:1;
	unsigned int rxadd:1;
	uint8_t length;
	uint8_t payload[];
};

struct ble_data_pdu {
	unsigned int llid:2;
	unsigned int nesn:1;
	unsigned int sn:1;
	unsigned int md:1;
	unsigned int _reserved:3;
	uint8_t length;
	uint8_t payload[];
};

typedef uint16_t ble_dev_addr[3];

static inline bool ble_address_equal(ble_dev_addr a, ble_dev_addr b) {
	return (a[0] == b[0])
	    && (a[1] == b[1])
	    && (a[2] == b[2]);
}

struct ble_adv_ind {
	ble_dev_addr adv_addr;
	uint8_t adv_data[];
};

struct ble_scan_req {
	ble_dev_addr scan_addr;
	ble_dev_addr adv_addr;
};

struct ble_scan_rsp {
	ble_dev_addr adv_addr;
	uint8_t scan_rsp_data[];
};

struct ble_connect_req {
	ble_dev_addr init_addr;
	ble_dev_addr adv_addr;

	uint32_t access_addr;
	unsigned int crc_init:24;
	uint8_t win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint8_t channel_map[5];
	unsigned int hop:5;
	unsigned int sca:3;
} __attribute__((packed));
