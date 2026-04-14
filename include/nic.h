#pragma once
#include "types.h"

#define ETH_FRAME_MAX   1518
#define ETH_ALEN        6

bool nic_init(void);
bool nic_init_wifi(void);
bool nic_is_wifi(void);
bool nic_send(const u8 *frame, u32 len);
bool nic_send_parts(const void *head, u32 head_len, const void *tail, u32 tail_len);
bool nic_recv(u8 *frame, u32 *len, bool *checksum_trusted);
void nic_get_mac(u8 *mac);
bool nic_link_up(void);

void nic_set_tx_checksum_offload(bool enable);
void nic_set_rx_checksum_offload(bool enable);
void nic_set_tso(bool enable);
bool nic_tx_checksum_offload_enabled(void);
bool nic_rx_checksum_offload_enabled(void);
bool nic_tso_enabled(void);
