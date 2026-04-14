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
#include "uart.h"

static bool wifi_nic_initialized;

bool wifi_nic_init(void)
{
    wifi_nic_initialized = false;

    uart_puts("[wnic] init CYW43455...\n");

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

bool wifi_nic_send(const u8 *frame, u32 len)
{
    if (!wifi_nic_initialized)
        return false;
    return cyw43_send_frame(frame, len);
}

bool wifi_nic_recv(u8 *frame, u32 *len)
{
    if (!wifi_nic_initialized)
        return false;
    return cyw43_recv_frame(frame, len);
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
