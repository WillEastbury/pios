#pragma once
#include "types.h"

/* wifi_nic.h - WiFi NIC backend for CYW43455
 *
 * Thin adapter implementing the nic.h nic_ops surface by delegating to the
 * CYW43455 FullMAC driver (cyw43.h). Only compiled/linked on platforms with
 * PIOS_HAS_WIFI_SDIO2 (see platform.h) -- callers must guard with the same
 * flag.
 */

bool wifi_nic_init(void);
void wifi_nic_adopt_loaded(void);
bool wifi_nic_send(const u8 *frame, u32 len);
bool wifi_nic_recv(u8 *frame, u32 *len);
void wifi_nic_get_mac(u8 *mac);
bool wifi_nic_link_up(void);
void wifi_nic_poll(void);
void wifi_nic_counters(u64 *rx_bytes, u64 *tx_bytes);
