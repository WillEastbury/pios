/*
 * nic.c - Active NIC backend for Raspberry Pi 5
 *
 * Pi 5 networking is provided by RP1 Cadence MACB/GEM (Ethernet) or
 * CYW43455 WiFi via SDIO (FullMAC). nic_init() tries Ethernet first,
 * falling back to WiFi if Ethernet is unavailable or the caller
 * explicitly selects WiFi via nic_select_wifi().
 */

#include "nic.h"
#include "macb.h"
#include "cyw43.h"
#include "simd.h"

/* WiFi NIC backend (wifi_nic.c) */
extern bool wifi_nic_init(void);
extern bool wifi_nic_send(const u8 *frame, u32 len);
extern bool wifi_nic_recv(u8 *frame, u32 *len);
extern void wifi_nic_get_mac(u8 *mac);
extern bool wifi_nic_link_up(void);
extern void wifi_nic_poll(void);

static bool tx_checksum_offload;
static bool rx_checksum_offload;
static bool tso_enabled;
static bool using_wifi;

bool nic_init(void)
{
    tx_checksum_offload = false;
    rx_checksum_offload = false;
    tso_enabled = false;
    using_wifi = false;
    return macb_init();
}

bool nic_init_wifi(void)
{
    tx_checksum_offload = false;
    rx_checksum_offload = false;
    tso_enabled = false;
    using_wifi = true;
    return wifi_nic_init();
}

bool nic_is_wifi(void)
{
    return using_wifi;
}

bool nic_send(const u8 *frame, u32 len)
{
    if (using_wifi)
        return wifi_nic_send(frame, len);
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
    if (using_wifi)
        return wifi_nic_send(tx_frame, total);
    return macb_send(tx_frame, total);
}

bool nic_recv(u8 *frame, u32 *len, bool *checksum_trusted)
{
    if (checksum_trusted)
        *checksum_trusted = false;
    if (using_wifi)
        return wifi_nic_recv(frame, len);
    return macb_recv(frame, len);
}

void nic_get_mac(u8 *mac)
{
    if (using_wifi) {
        wifi_nic_get_mac(mac);
        return;
    }
    macb_get_mac(mac);
}

bool nic_link_up(void)
{
    if (using_wifi)
        return wifi_nic_link_up();
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
