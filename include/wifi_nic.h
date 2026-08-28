#pragma once
#include "types.h"

/* wifi_nic.h - WiFi NIC backend for CYW43455
 *
 * Thin adapter implementing the nic.h nic_ops surface by delegating to the
 * CYW43455 FullMAC driver (cyw43.h). Only activated on platforms with a
 * supported WiFi SDIO host (see platform.h).
 */

bool wifi_nic_init(void);
typedef void (*wifi_nic_progress_fn)(u32 stage);
void wifi_nic_set_progress_hook(wifi_nic_progress_fn hook);
void wifi_nic_adopt_loaded(void);
bool wifi_nic_send(const u8 *frame, u32 len);
bool wifi_nic_recv(u8 *frame, u32 *len);
void wifi_nic_get_mac(u8 *mac);
bool wifi_nic_link_up(void);
void wifi_nic_poll(void);
void wifi_nic_counters(u64 *rx_bytes, u64 *tx_bytes);
