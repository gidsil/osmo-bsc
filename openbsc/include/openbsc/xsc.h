#pragma once

#include <stdint.h>

struct msgb;
struct gsm_network;

typedef int (*mncc_recv_cb_t)(struct gsm_network *, struct msgb *);

#define MAX_A5_KEY_LEN	(128/8)

struct gsm_encr {
	uint8_t alg_id;
	uint8_t key_len;
	uint8_t key[MAX_A5_KEY_LEN];
};

struct gsm_network *gsm_network_init(void *ctx,
				     uint16_t country_code,
				     uint16_t network_code,
				     mncc_recv_cb_t mncc_recv);
