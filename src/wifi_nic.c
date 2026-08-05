/*
 * wifi_nic.c - WiFi NIC backend for CYW43455
 *
 * Implements the nic.h interface by delegating to the CYW43455
 * FullMAC driver. Ethernet frames pass through SDPCM data channel.
 *
 * Used by nic.c when WiFi is the active NIC backend.
 */

#include "cyw43.h"
#include "nic.h"
#include "wifi_nic.h"
#include "uart.h"

static bool wifi_nic_initialized;
static u64 wifi_rx_bytes;
static u64 wifi_tx_bytes;

bool wifi_nic_init(void)
{
    wifi_nic_initialized = false;
    wifi_rx_bytes = 0U;
    wifi_tx_bytes = 0U;

    uart_puts("[wnic] init CYW43455...\n");

    /* Pre-load firmware blobs from SD BEFORE SDIO2 init disturbs EMMC2 */
    uart_puts("[wnic] preload blobs...\n");
    if (!cyw43_preload_blobs()) {
        uart_puts("[wnic] preload fail\n");
        return false;
    }

    if (!cyw43_init()) {
        uart_puts("[wnic] cyw init fail\n");
        return false;
    }

    if (!cyw43_load_firmware()) {
        uart_puts("[wnic] fw load fail\n");
        return false;
    }

    wifi_nic_initialized = true;
    uart_puts("[wnic] ready\n");
    return true;
}

void wifi_nic_adopt_loaded(void)
{
    wifi_nic_initialized = true;
}

bool wifi_nic_send(const u8 *frame, u32 len)
{
    if (!wifi_nic_initialized)
        return false;
    bool ok = cyw43_send_frame(frame, len);
    if (ok)
        wifi_tx_bytes += len;
    return ok;
}

bool wifi_nic_recv(u8 *frame, u32 *len)
{
    if (!wifi_nic_initialized)
        return false;
    bool ok = cyw43_recv_frame(frame, len);
    if (ok)
        wifi_rx_bytes += *len;
    return ok;
}

void wifi_nic_get_mac(u8 *mac)
{
    cyw43_get_mac(mac);
}

bool wifi_nic_link_up(void)
{
    return cyw43_is_connected();
}

void wifi_nic_poll(void)
{
    if (wifi_nic_initialized)
        cyw43_poll();
}

void wifi_nic_counters(u64 *rx_bytes, u64 *tx_bytes)
{
    if (rx_bytes)
        *rx_bytes = wifi_rx_bytes;
    if (tx_bytes)
        *tx_bytes = wifi_tx_bytes;
}
