#include <stdio.h>
#include "types.h"
#include "nic.h"
#include "macb.h"
#include "virtio_net.h"
#include "fb.h"
#include "pioscap.h"
#include "net_dispatch.h"

static u32 remaining, received, captured, failures;
static bool malformed;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #c); failures++; \
} } while (0)

bool virtio_net_probe(void) { return true; }
bool virtio_net_init(void) { return true; }
bool virtio_net_link_up(void) { return true; }
void virtio_net_get_mac(u8 *mac) { memset(mac, 0, 6); }
bool virtio_net_send(const u8 *frame, u32 len)
{ (void)frame; (void)len; return true; }
bool virtio_net_recv(u8 *frame, u32 *len)
{
    if (!remaining)
        return false;
    remaining--;
    received++;
    memset(frame, 0, 64);
    frame[12] = 0x88; frame[13] = 0xB5;
    *len = malformed ? 0 : 64;
    return true;
}

bool macb_init(void) { return false; }
bool macb_link_up(void) { return false; }
u32 macb_link_mbps(void) { return 0; }
bool macb_link_full_duplex(void) { return false; }
void macb_get_mac(u8 *mac) { memset(mac, 0, 6); }
bool macb_send(const u8 *frame, u32 len)
{ (void)frame; (void)len; return false; }
bool macb_recv(u8 *frame, u32 *len, bool *trusted)
{ (void)frame; (void)len; (void)trusted; return false; }
void macb_set_tx_checksum_offload(bool v) { (void)v; }
void macb_set_rx_checksum_offload(bool v) { (void)v; }
void macb_set_tso(bool v) { (void)v; }
bool macb_tx_checksum_offload_enabled(void) { return false; }
bool macb_rx_checksum_offload_enabled(void) { return false; }
bool macb_tso_enabled(void) { return false; }
void macb_offload_regs(u32 *a, u32 *b) { *a = *b = 0; }
u64 timer_ticks(void) { return 0; }
void fb_putc(char c) { (void)c; }
void fb_puts(const char *s) { (void)s; }
void fb_set_color(u32 a, u32 b) { (void)a; (void)b; }
void pioscap_init(void) {}
void pioscap_rx(const u8 *frame, u32 len)
{ (void)frame; CHECK(len == 64); captured++; }
void pioscap_tx(const u8 *frame, u32 len) { (void)frame; (void)len; }
bool net_dispatch_enabled(void) { return false; }
bool net_dispatch_submit_egress(nic_iface_t iface, const u8 *frame, u32 len)
{ (void)iface; (void)frame; (void)len; return false; }

int main(void)
{
    CHECK(nic_init());
    u8 frame[ETH_FRAME_MAX];
    u32 len;
    bool trusted;
    nic_filter_set_default(false, true);
    remaining = 33;
    for (u32 i = 0; i < 32; i++) {
        CHECK(nic_recv_on(NIC_IFACE_WIRED, frame, &len, &trusted));
        CHECK(len == 0 && received == i + 1);
    }
    CHECK(captured == 0 && remaining == 1);
    nic_filter_set_default(true, true);
    CHECK(nic_recv(frame, &len, &trusted) && len == 64);
    CHECK(captured == 1 && remaining == 0);
    CHECK(!nic_recv(frame, &len, &trusted));
    malformed = true;
    remaining = 1;
    CHECK(nic_recv(frame, &len, &trusted) && len == 0);
    CHECK(!nic_recv(frame, &len, &trusted));
    CHECK(captured == 1);
    printf("NIC receive: %u failures\n", failures);
    return failures ? 1 : 0;
}
