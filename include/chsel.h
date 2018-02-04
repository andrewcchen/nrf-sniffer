#include <stdint.h>

struct ble_chsel1 {
	uint8_t last_unmapped_chn;

	uint8_t hop_inc;
	uint8_t num_used_chns;
	uint8_t chn_used_bitmap[5];
	uint8_t used_chn_list[37];
};

void ble_chsel1_init(struct ble_chsel1* info, uint8_t hop, uint8_t map[5]);
int ble_chsel1_next_chn(struct ble_chsel1 *info);

int ble_chsel1_map(struct ble_chsel1 *info, int unmapped_chn);
