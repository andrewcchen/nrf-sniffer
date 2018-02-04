#include <stdbool.h>

#include "chsel.h"


/**
 * Construct a channel map used by channel selection algorithm 1 from the
 * channel map given in CONNECT_REQ.
 */
void ble_chsel1_init(struct ble_chsel1* info, uint8_t hop, uint8_t map[5]) {
	info->last_unmapped_chn = 0;
	info->hop_inc = hop;

	uint8_t used_cnt = 0;
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 8; j++) {
			if (map[i] & (1 << j)) {
				info->used_chn_list[used_cnt] = i*8+j;
				used_cnt++;
			}
		}
		info->chn_used_bitmap[i] = map[i];
	}
	info->num_used_chns = used_cnt;
}

static inline bool chn_used(struct ble_chsel1 *info, int chn) {
	return info->chn_used_bitmap[chn/8] & (1 << (chn%8));
}

int ble_chsel1_map(struct ble_chsel1 *info, int unmapped_chn) {
	if (chn_used(info, unmapped_chn)) {
		return unmapped_chn;
	} else {
		return info->used_chn_list[unmapped_chn % info->num_used_chns];
	}
}

int ble_chsel1_next_chn(struct ble_chsel1 *info) {
	int unmapped_chn = (info->last_unmapped_chn + info->hop_inc) % 37;
	info->last_unmapped_chn = unmapped_chn;
	return ble_chsel1_map(info, unmapped_chn);
}
