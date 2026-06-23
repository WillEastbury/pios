/*
 * macb.h - Cadence GEM/MACB Ethernet driver for Pi 5 (RP1)
 *
 * The Pi 5 Ethernet is a Cadence GEM on the RP1 southbridge at
 * 0x1F00100000, NOT the Broadcom GENET used on Pi 4.
 *
 * This provides the same interface as genet.h so the net stack
 * can use it without changes.
 */

#pragma once
#include "types.h"

#define MACB_BASE           0x1F00100000UL

/* Same API as genet.h — drop-in replacement */
bool macb_init(void);
bool macb_send(const u8 *frame, u32 len);
bool macb_recv(u8 *frame, u32 *len, bool *checksum_trusted);
void macb_get_mac(u8 *mac);
bool macb_link_up(void);
u32  macb_link_mbps(void);
bool macb_link_full_duplex(void);
void macb_dump_full_state(const char *tag);

struct macb_irq_snapshot {
    u32 imr;
    u32 rsr;
    u32 tsr;
    u32 ncr;
    u32 isr_clear;
} PACKED;

void macb_irq_snapshot(struct macb_irq_snapshot *out, bool read_clear_isr);
void macb_irq_enable_rx(void);
u32  macb_irq_ack_rx(void);

/* Read-only health snapshot for live wedge diagnosis (poll over HTTP while
 * ramping load). rx_owned = RX descriptors the MAC has filled but the CPU has
 * not yet drained (== ring backlog; 32 means the ring is full → RX overrun). */
struct macb_diag {
    u32 rx_idx;
    u32 tx_idx;
    u32 rx_owned;   /* RX descriptors with OWN set (MAC-filled backlog) */
    u32 rx_recv;    /* lifetime frames received */
    u32 tx_send;    /* lifetime frames sent */
    u32 nsr;
    u32 rsr;
    u32 tsr;
    u32 ncr;
    u32 rbqp;
    u32 tbqp;
    u32 imr;            /* interrupt mask */
    u32 eth_cfg_stat;   /* RP1 ETH_CFG STAT: AXI bus error flags */
    u32 rx_recover;     /* lifetime RX-overrun recoveries performed */
    u32 ring_size;      /* RX ring depth (NUM_RX) */
    u32 tx_drop;        /* lifetime TX frames dropped (ring full) */
    u32 tx_recover;     /* lifetime TX ring recoveries performed */
} PACKED;
void macb_diag(struct macb_diag *out);

/* Detect a latched RX overrun/BNA stall and, if present, reset+restart the RX
 * ring so the polling driver recovers instead of staying wedged. Cheap to call
 * every poll; returns true only when it actually performed a recovery. */
bool macb_rx_recover(void);

/* Try to recover a stalled MAC: clear latched status bits, halt+restart TX,
 * dump ETH_CFG_STAT for AXI bus errors. Returns true if it did anything. */
bool macb_kick_stall(void);

void macb_set_tx_checksum_offload(bool enable);
void macb_set_rx_checksum_offload(bool enable);
void macb_set_tso(bool enable);
bool macb_tx_checksum_offload_enabled(void);
bool macb_rx_checksum_offload_enabled(void);
bool macb_tso_enabled(void);
void macb_offload_regs(u32 *ncfgr_out, u32 *dmacfg_out);
