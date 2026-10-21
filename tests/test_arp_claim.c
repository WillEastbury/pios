#include <assert.h>
#include <stdio.h>
#include "arp.h"
#include "dhcp.h"
#include "net.h"

static u64 clock_ms;
static bool tx_ok = true, configure_ok = true;
static u32 arp_sends, dhcp_sends, applied;
static u8 arp_wire[60], dhcp_wire[600];
static u32 probed[256];
static nic_iface_t wire_iface;
static udp_recv_cb dhcp_receive;
static const u8 local_mac[6] = {2, 1, 2, 3, 4, 5};
static const u8 peer_mac[6] = {2, 6, 7, 8, 9, 10};
static const u32 pool = 0xc0a84d00U;

u64 timer_monotonic_ms(void) { return clock_ms; }
u64 timer_ticks(void) { return clock_ms; }
void timer_delay_ms(u64 ms) { (void)ms; assert(!"claim must not wait"); }
void uart_puts(const char *s) { (void)s; }
void uart_putc(char c) { (void)c; }
void uart_hex(u64 n) { (void)n; }
void nic_record_rate_limited(void) {}
bool nic_iface_active(nic_iface_t iface)
{ return iface == NIC_IFACE_WIRED || iface == NIC_IFACE_WIFI; }
nic_iface_t nic_default_iface(void) { return NIC_IFACE_WIRED; }
void nic_get_mac_for(nic_iface_t iface, u8 *mac)
{ (void)iface; memcpy(mac, local_mac, 6); }
u32 hw_crc32c(const void *p, u32 n)
{
    const u8 *bytes = p;
    u32 hash = 0;
    for (u32 i = 0; i < n; i++) hash = hash * 31U + bytes[i];
    return hash;
}
u16 simd_checksum(const void *p, u32 n) { (void)p; (void)n; return 0; }
void net_dispatch_yield(void) { assert(!"asynchronous path must not yield"); }
bool net_udp_subscribe(udp_recv_cb cb) { dhcp_receive = cb; return true; }
bool net_add_interface(nic_iface_t iface, u32 ip, u32 gw, u32 mask,
                       const u8 *mac)
{
    (void)mac;
    assert(iface == NIC_IFACE_WIRED && ip == pool + 15U);
    assert(gw == pool + 1U && mask == 0xffffff00U);
    if (configure_ok) applied++;
    return configure_ok;
}

static u32 read32(const u8 *p)
{ return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3]; }
static void write32(u8 *p, u32 ip)
{ p[0] = ip >> 24; p[1] = ip >> 16; p[2] = ip >> 8; p[3] = ip; }

bool nic_send_on(nic_iface_t iface, const u8 *frame, u32 len)
{
    wire_iface = iface;
    if (frame[12] == 8 && frame[13] == 6) {
        assert(len == sizeof(arp_wire));
        memcpy(arp_wire, frame, len);
        arp_sends++;
        u32 target = read32(frame + 38);
        if (read32(frame + 28) == 0U && target >= pool && target < pool + 256U)
            probed[target - pool]++;
    } else {
        assert(len <= sizeof(dhcp_wire));
        memcpy(dhcp_wire, frame, len);
        dhcp_sends++;
    }
    return tx_ok;
}

static void incoming(nic_iface_t iface, u32 sender, u32 target, u16 opcode,
                     const u8 *mac, bool valid)
{
    u8 frame[60] = {0};
    memset(frame, 0xff, 6);
    memcpy(frame + 6, mac, 6);
    frame[12] = 8; frame[13] = 6;
    frame[15] = 1; frame[16] = 8;
    frame[18] = 6; frame[19] = 4;
    frame[20] = opcode >> 8; frame[21] = opcode;
    memcpy(frame + 22, mac, 6);
    write32(frame + 28, sender);
    write32(frame + 38, target);
    if (!valid) frame[18] = 5;
    arp_input_iface(iface, frame, sizeof(frame));
}

static enum arp_claim_result complete(struct arp_claim_handle handle)
{
    enum arp_claim_result result = ARP_CLAIM_PENDING;
    for (u32 i = 0; i < 1201U && result == ARP_CLAIM_PENDING; i++) {
        u32 before = arp_sends;
        result = arp_claim_poll(handle);
        assert(arp_sends - before <= 8U);
        clock_ms += 100U;
    }
    assert(result != ARP_CLAIM_PENDING);
    return result;
}

static void test_claims(void)
{
    struct arp_claim_handle h, wifi, old;
    arp_init_iface(NIC_IFACE_WIRED, 0, 0, local_mac);
    arp_init_iface(NIC_IFACE_WIFI, 0, 0, local_mac);
    assert(!arp_claim_begin(NIC_IFACE_ANY, pool, pool, &h));
    assert(!arp_claim_begin(NIC_IFACE_WIRED, 0, 0xffffffffU, &h));
    assert(!arp_claim_begin(NIC_IFACE_WIRED, pool, pool + 256U, &h));
    assert(arp_claim_begin(NIC_IFACE_WIRED, pool + 15U, pool + 15U, &h));
    assert(!arp_claim_begin(NIC_IFACE_WIRED, pool + 16U, pool + 16U, &old));
    assert(arp_claim_poll(h) == ARP_CLAIM_PENDING);
    assert(read32(arp_wire + 28) == 0U && read32(arp_wire + 38) == pool + 15U);
    assert(memcmp(arp_wire + 32, "\0\0\0\0\0\0", 6) == 0);
    assert(wire_iface == NIC_IFACE_WIRED);
    assert(!arp_claim_accepts(NIC_IFACE_WIFI, arp_wire, sizeof(arp_wire)));
    assert(arp_claim_accepts(NIC_IFACE_WIRED, arp_wire, sizeof(arp_wire)));
    assert(!arp_claim_accepts(NIC_IFACE_WIRED, arp_wire, 41));
    incoming(NIC_IFACE_WIRED, 0, pool + 15U, 1, local_mac, true);
    incoming(NIC_IFACE_WIRED, pool + 15U, pool + 15U, 1, peer_mac, false);
    incoming(NIC_IFACE_WIFI, pool + 15U, 0, 2, peer_mac, true);
    assert(!arp_claim_conflicted(h, pool + 15U));
    assert(complete(h) == ARP_CLAIM_AVAILABLE);
    assert(probed[15] == 3U);
    /* A late announcement still invalidates an available session. */
    incoming(NIC_IFACE_WIRED, pool + 15U, pool + 15U, 1, peer_mac, true);
    assert(arp_claim_poll(h) == ARP_CLAIM_CONFLICT);
    arp_claim_cancel(h);
    old = h;
    assert(arp_claim_poll(old) == ARP_CLAIM_ERROR);
    assert(arp_claim_begin(NIC_IFACE_WIRED, pool, pool + 255U, &h));
    assert(arp_claim_begin(NIC_IFACE_WIFI, pool + 15U, pool + 15U, &wifi));
    arp_claim_cancel(old);
    incoming(NIC_IFACE_WIRED, 0, pool + 3U, 1, peer_mac, true);
    incoming(NIC_IFACE_WIRED, pool + 5U, 0, 2, peer_mac, true);
    assert(arp_claim_conflicted(h, pool + 3U));
    assert(arp_claim_conflicted(h, pool + 5U));
    assert(!arp_claim_conflicted(wifi, pool + 15U));
    memset(probed, 0, sizeof(probed));
    assert(complete(h) == ARP_CLAIM_CONFLICT);
    for (u32 i = 0; i < 256U; i++) assert(probed[i] == 3U);
    struct arp_snapshot_entry entries[2];
    arp_set_interface(NIC_IFACE_WIRED);
    assert(arp_snapshot(entries, 2) == 0U);
    arp_claim_cancel(h);
    arp_claim_cancel(wifi);
    assert(arp_claim_begin(NIC_IFACE_WIRED, pool + 15U, pool + 15U, &h));
    tx_ok = false;
    assert(arp_claim_poll(h) == ARP_CLAIM_ERROR);
    tx_ok = true;
    arp_claim_cancel(h);
    assert(arp_claim_begin(NIC_IFACE_WIRED, pool + 15U, pool + 15U, &h));
    clock_ms += 120000U;
    assert(arp_claim_poll(h) == ARP_CLAIM_ERROR);
    arp_claim_cancel(h);
    /* A bound host must defend its address against addressless probes. */
    arp_init_iface(NIC_IFACE_WIRED, pool + 15U, 0xffffff00U, local_mac);
    u32 before = arp_sends;
    incoming(NIC_IFACE_WIRED, 0, pool + 15U, 1, peer_mac, true);
    assert(arp_sends == before + 1U && arp_wire[21] == 2);
    assert(read32(arp_wire + 38) == 0U);
    assert(arp_snapshot(entries, 2) == 0U);
}

static void reply(u8 type, nic_iface_t iface)
{
    u8 msg[300] = {0};
    msg[0] = 2; msg[1] = 1; msg[2] = 6;
    memcpy(msg + 4, dhcp_wire + 42 + 4, 4);
    write32(msg + 16, pool + 15U);
    write32(msg + 236, 0x63825363U);
    u32 n = 240;
    msg[n++] = 53; msg[n++] = 1; msg[n++] = type;
    msg[n++] = 54; msg[n++] = 4; write32(msg + n, pool + 1U); n += 4;
    msg[n++] = 1; msg[n++] = 4; write32(msg + n, 0xffffff00U); n += 4;
    msg[n++] = 3; msg[n++] = 4; write32(msg + n, pool + 1U); n += 4;
    msg[n++] = 255;
    dhcp_receive(iface, 0xffffffffU, pool + 1U, 67, 68, msg, (u16)n);
}

static void offer_and_ack(void)
{
    arp_init_iface(NIC_IFACE_WIRED, 0, 0, local_mac);
    assert(dhcp_begin());
    dhcp_poll();
    assert(dhcp_get_state() == DHCP_SELECTING);
    reply(2, NIC_IFACE_WIFI);
    assert(dhcp_get_state() == DHCP_SELECTING);
    reply(2, NIC_IFACE_WIRED);
    assert(dhcp_get_state() == DHCP_REQUESTING);
    reply(5, NIC_IFACE_WIRED);
    assert(dhcp_get_state() == DHCP_PROBING && dhcp_get_lease() == NULL);
}

static void test_dhcp(void)
{
    offer_and_ack();
    for (u32 i = 0; i < 40; i++) { dhcp_poll(); clock_ms += 100U; }
    assert(dhcp_get_state() == DHCP_BOUND && applied == 1U);
    assert(dhcp_get_lease()->ip == pool + 15U);
    offer_and_ack();
    incoming(NIC_IFACE_WIRED, 0, pool + 15U, 1, peer_mac, true);
    for (u32 i = 0; i < 40; i++) { dhcp_poll(); clock_ms += 100U; }
    assert(dhcp_get_state() == DHCP_INIT && dhcp_get_lease() == NULL);
    assert(applied == 1U);
    assert(dhcp_wire[42 + 240 + 2] == 4U); /* Actual DHCPDECLINE. */
    u32 before = dhcp_sends;
    dhcp_poll();
    assert(dhcp_sends == before);
    clock_ms += 10000U;
    dhcp_poll();
    assert(dhcp_get_state() == DHCP_SELECTING);
    offer_and_ack();
    tx_ok = false;
    dhcp_poll();
    tx_ok = true;
    assert(dhcp_get_state() == DHCP_INIT && applied == 1U);
    offer_and_ack();
    configure_ok = false;
    for (u32 i = 0; i < 40; i++) { dhcp_poll(); clock_ms += 100U; }
    assert(dhcp_get_state() == DHCP_INIT && applied == 1U);
}

int main(void)
{
    test_claims();
    test_dhcp();
    puts("ARP claim / DHCP: PASS");
    return 0;
}
