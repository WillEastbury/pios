#include <stdio.h>
#include "types.h"
#include "nic.h"
#include "wifi_nic.h"
#include "cyw43.h"

static u32 failures, calls;
static bool have_frame = true;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #c); failures++; \
} } while (0)

void uart_puts(const char *s) { (void)s; }
bool cyw43_preload_blobs(void) { return true; }
bool cyw43_init(void) { return true; }
bool cyw43_load_firmware(void) { return true; }
bool cyw43_send_frame(const u8 *p, u32 n) { (void)p; (void)n; return true; }
void cyw43_get_mac(u8 *mac) { memset(mac, 0, 6); }
bool cyw43_is_connected(void) { return true; }
void cyw43_poll(void) {}
bool cyw43_recv_frame(u8 *frame, u32 *len)
{
    calls++;
    CHECK(*len == ETH_FRAME_MAX);
    if (!have_frame)
        return false;
    memset(frame, 0x42, 60);
    *len = 60;
    return true;
}

int main(void)
{
    u8 frame[ETH_FRAME_MAX + 1];
    memset(frame, 0, sizeof(frame));
    frame[ETH_FRAME_MAX] = 0xAD;
    u32 len = 0;
    CHECK(!wifi_nic_recv(frame, &len) && calls == 0);
    CHECK(wifi_nic_init());
    CHECK(wifi_nic_recv(frame, &len) && len == 60);
    CHECK(frame[0] == 0x42 && frame[59] == 0x42);
    CHECK(frame[ETH_FRAME_MAX] == 0xAD);
    have_frame = false;
    CHECK(!wifi_nic_recv(frame, &len) && len == 0);
    u64 rx = 0, tx = 0;
    wifi_nic_counters(&rx, &tx);
    CHECK(rx == 60 && tx == 0);
    printf("WiFi NIC capacity: %u failures\n", failures);
    return failures ? 1 : 0;
}
