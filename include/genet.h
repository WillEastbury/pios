#pragma once
#include "types.h"

/*
 * GENET v5 Ethernet MAC driver for BCM2712 (Pi 5).
 * Minimal: single TX/RX queue, polling, basic PHY init.
 */

#define ETH_FRAME_MAX   1518
#define ETH_ALEN        6

bool genet_init(void);
bool genet_send(const u8 *frame, u32 len);
bool genet_recv(u8 *frame, u32 *len);
void genet_get_mac(u8 *mac);
bool genet_link_up(void);

/* Safe feature-gated hooks for checksum/TSO offload plumbing. */
void genet_set_tx_checksum_offload(bool enable);
void genet_set_rx_checksum_offload(bool enable);
void genet_set_tso(bool enable);
bool genet_tx_checksum_offload_enabled(void);
bool genet_rx_checksum_offload_enabled(void);
bool genet_tso_enabled(void);
