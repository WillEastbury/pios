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
bool macb_recv(u8 *frame, u32 *len);
void macb_get_mac(u8 *mac);
bool macb_link_up(void);
