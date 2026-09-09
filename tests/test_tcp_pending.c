#include <stdio.h>
#include "types.h"
#include "tcp.h"
#include "net.h"
#include "highmem.h"
#include "dtrace.h"

static u32 failures, sent_seq, sent_ack, send_calls;
static bool reject_next_send;
static const u8 mac[6] = {2, 0, 0, 0, 0, 1};
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #c); failures++; \
} } while (0)

static u32 be32(const u8 *p)
{ return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }
static void put32(u8 *p, u32 v)
{ p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

u64 timer_monotonic_ms(void) { return 1000; }
void highmem_status(struct highmem_status *out) { memset(out, 0, sizeof(*out)); }
void *highmem_alloc(u64 size, u64 align) { (void)size; (void)align; return NULL; }
bool net_interface_configured(nic_iface_t iface) { return iface == NIC_IFACE_WIRED; }
u32 net_get_our_ip_for(nic_iface_t iface) { (void)iface; return 0xC0A800C9; }
const u8 *net_resolve_mac_on(nic_iface_t iface, u32 ip)
{ (void)iface; (void)ip; return mac; }
void net_get_mac_for(nic_iface_t iface, u8 *out)
{ (void)iface; memcpy(out, mac, sizeof(mac)); }
nic_iface_t nic_default_iface(void) { return NIC_IFACE_WIRED; }
bool nic_tx_checksum_offload_enabled_for(nic_iface_t iface) { (void)iface; return false; }
bool nic_send_on(nic_iface_t iface, const u8 *frame, u32 len)
{
    (void)iface;
    CHECK(len >= 54);
    sent_seq = be32(frame + 38);
    sent_ack = be32(frame + 42);
    send_calls++;
    if (reject_next_send) {
        reject_next_send = false;
        return false;
    }
    return true;
}
bool nic_send_parts_on(nic_iface_t iface, const void *head, u32 head_len,
                       const void *tail, u32 tail_len)
{
    (void)tail; (void)tail_len;
    return nic_send_on(iface, head, head_len);
}
void uart_puts(const char *s) { (void)s; }
volatile u32 g_dtrace_active_mask;
void dtrace_emit(u32 c, u16 e, u64 a, u64 b, u64 d, u64 f)
{ (void)c; (void)e; (void)a; (void)b; (void)d; (void)f; }
u16 simd_checksum(const void *data, u32 len)
{ (void)data; (void)len; return 0; } /* IP checksums are outside this TCP state test. */
u32 hw_crc32c(const void *data, u32 len)
{
    const u8 *p = data;
    u32 crc = 0;
    for (u32 i = 0; i < len; i++) {
        crc ^= p[i];
        for (u32 bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0x82F63B78U);
    }
    return crc;
}

static void input(u32 seq, u32 ack, u8 flags, const u8 *data, u32 len)
{
    u8 segment[20 + TCP_MSS + 1] = {0};
    CHECK(len <= TCP_MSS + 1);
    segment[0] = 0xB0; segment[1] = 0x01;
    segment[3] = 80;
    put32(segment + 4, seq);
    put32(segment + 8, ack);
    segment[12] = 0x50;
    segment[13] = flags;
    segment[14] = 0x20;
    if (len)
        memcpy(segment + 20, data, len);
    tcp_input(NULL, 0, 0xC0A80001, 0xC0A800C9,
              segment, 20 + len, true, NIC_IFACE_WIRED);
}

int main(void)
{
    const u8 data[] = {0, 1, 0xFF, 'G', 'E', 'T', '\r', '\n'};
    for (u32 mode = 0; mode < 3; mode++) {
        tcp_init();
        tcp_conn_t listener = tcp_listen(80);
        CHECK(listener >= 0);
        input(100, 0, 0x02, NULL, 0);
        u32 cookie = sent_seq;
        CHECK(sent_ack == 101);
        if (mode != 1)
            input(101, cookie + 1, 0x10, NULL, 0);
        input(101, cookie + 1, 0x18, data, sizeof(data));
        CHECK(sent_ack == 101 + sizeof(data));
        if (mode == 2)
            input(101, cookie + 1, 0x18, data, sizeof(data));
        tcp_conn_t conn = tcp_accept(listener);
        CHECK(conn >= 0 && tcp_readable(conn) == sizeof(data));
        u8 received[sizeof(data)] = {0};
        CHECK(tcp_read(conn, received, sizeof(received)) == sizeof(data));
        CHECK(memcmp(received, data, sizeof(data)) == 0);
        CHECK(tcp_readable(conn) == 0 && tcp_accept(listener) < 0);
    }
    tcp_init();
    tcp_conn_t listener = tcp_listen(80);
    input(100, 0, 0x02, NULL, 0);
    u32 cookie = sent_seq;
    input(101, cookie + 1, 0x10, NULL, 0);
    u8 oversized[TCP_MSS + 1] = {0};
    input(101, cookie + 1, 0x18, oversized, sizeof(oversized));
    CHECK(sent_ack == 101);
    tcp_conn_t conn = tcp_accept(listener);
    CHECK(conn >= 0 && tcp_readable(conn) == 0);

    tcp_init();
    listener = tcp_listen(80);
    input(200, 0, 0x02, NULL, 0);
    cookie = sent_seq;
    input(201, cookie + 1, 0x10, NULL, 0);
    conn = tcp_accept(listener);
    CHECK(conn >= 0);
    const u8 outbound[] = {'q', 'u', 'e', 'u', 'e'};
    reject_next_send = true;
    u32 before_calls = send_calls;
    CHECK(tcp_write(conn, outbound, sizeof(outbound)) == sizeof(outbound));
    CHECK(send_calls == before_calls + 1U);
    u32 rejected_seq = sent_seq;
    /* A service revisit after queue credit returns must retry the same
     * sequence without a peer ACK or retransmission timeout. */
    u8 ignored = 0U;
    CHECK(tcp_write(conn, &ignored, 0U) == 0U);
    CHECK(send_calls == before_calls + 2U);
    CHECK(sent_seq == rejected_seq);
    printf("TCP pending: %u failures\n", failures);
    return failures ? 1 : 0;
}
