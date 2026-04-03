/*
 * nic.c - Active NIC backend for Raspberry Pi 5
 *
 * Pi 5 networking is provided by RP1 Cadence MACB/GEM, not BCM GENET.
 * Keep a small stable NIC API for the rest of the stack while the old
 * GENET-specific code is retired.
 */

#include "nic.h"
#include "macb.h"
#include "simd.h"

static bool tx_checksum_offload;
static bool rx_checksum_offload;
static bool tso_enabled;

bool nic_init(void)
{
    tx_checksum_offload = false;
    rx_checksum_offload = false;
    tso_enabled = false;
    return macb_init();
}

bool nic_send(const u8 *frame, u32 len)
{
    return macb_send(frame, len);
}

bool nic_send_parts(const void *head, u32 head_len, const void *tail, u32 tail_len)
{
    static u8 tx_frame[2048] ALIGNED(64);
    u32 total = head_len + tail_len;
    if (total > sizeof(tx_frame))
        return false;
    if (head_len)
        simd_memcpy(tx_frame, head, head_len);
    if (tail_len)
        simd_memcpy(tx_frame + head_len, tail, tail_len);
    return macb_send(tx_frame, total);
}

bool nic_recv(u8 *frame, u32 *len, bool *checksum_trusted)
{
    if (checksum_trusted)
        *checksum_trusted = false;
    return macb_recv(frame, len);
}

void nic_get_mac(u8 *mac)
{
    macb_get_mac(mac);
}

bool nic_link_up(void)
{
    return macb_link_up();
}

void nic_set_tx_checksum_offload(bool enable)
{
    tx_checksum_offload = enable;
}

void nic_set_rx_checksum_offload(bool enable)
{
    rx_checksum_offload = enable;
}

void nic_set_tso(bool enable)
{
    tso_enabled = enable && tx_checksum_offload;
}

bool nic_tx_checksum_offload_enabled(void)
{
    return tx_checksum_offload;
}

bool nic_rx_checksum_offload_enabled(void)
{
    return rx_checksum_offload;
}

bool nic_tso_enabled(void)
{
    return tso_enabled;
}
