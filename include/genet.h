#pragma once
#include "types.h"

/*
 * GENET v5 Ethernet MAC driver for BCM2711 (Pi 4).
 * Pi 5 wired Ethernet is Cadence MACB on RP1, not this block.
 * Minimal: single TX/RX queue, GIC RX interrupt, basic PHY init.
 * Protocol work is FIFO-only (ADR-033); the IRQ top half only acks.
 */

#define ETH_FRAME_MAX   1518
#define ETH_ALEN        6

bool genet_init(void);
bool genet_send(const u8 *frame, u32 len);
bool genet_send_parts(const void *head, u32 head_len, const void *tail, u32 tail_len);
bool genet_recv(u8 *frame, u32 *len, bool *checksum_trusted);
void genet_get_mac(u8 *mac);
bool genet_link_up(void);
u32  genet_link_mbps(void);
bool genet_link_full_duplex(void);

/* Safe feature-gated hooks for checksum/TSO offload plumbing. */
void genet_set_tx_checksum_offload(bool enable);
void genet_set_rx_checksum_offload(bool enable);
void genet_set_tso(bool enable);
bool genet_tx_checksum_offload_enabled(void);
bool genet_rx_checksum_offload_enabled(void);
bool genet_tso_enabled(void);

/* IRQ top-half helpers. Mask, ack INTRL2, post AIRQ; unmask after drain. */
void genet_irq_mask_rx(void);
u32  genet_irq_ack(void);
void genet_irq_unmask_rx(void);
void genet_irq_enable(void);
