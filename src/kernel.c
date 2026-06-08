/*
 * kernel.c - PIOS main entry point
 *
 * Boot flow (all on core 0), colour-coded on HDMI:
 *   BLACK   - Firmware handed off to kernel (pre-framebuffer)
 *   GREEN   - VideoCore framebuffer + HDMI online
 *              EL2, exceptions, GIC, MMU, timer, DMA init
 *   PINK    - PCIe root complex + RP1 southbridge connected
 *   RED     - USB (xHCI) + UART / TTY online
 *   GREY    - Filesystem (SD + WALFS) online
 *   BLUE    - NIC / MAC (Cadence MACB/GEM Ethernet) online
 *   YELLOW  - Multicore — secondary cores started
 *   PURPLE  - PIOS operational (pink text on purple = terminal)
 *
 *   If boot hangs, the last visible colour identifies the subsystem.
 */

#include "types.h"
#include "uart.h"
#include "fb.h"
#include "fifo.h"
#include "sd.h"
#include "nic.h"
#include "macb.h"
#include "net.h"
#include "arp.h"
#include "tcp.h"
#include "dhcp.h"
#include "dns.h"
#include "core.h"
#include "core_env.h"
#include "simd.h"
#include "mmu.h"
#include "gic.h"
#include "exception.h"
#include "timer.h"
#include "dma.h"
#include "gpu.h"
#include "tensor.h"
#include "walfs.h"
#include "bcache.h"
#include "principal.h"
#include "proc.h"
#include "pix.h"
#include "pcie.h"
#include "rp1.h"
#include "rp1_gpio.h"
#include "rp1_clk.h"
#include "rp1_uart.h"
#include "usb.h"
#include "xhci.h"
#include "usb_storage.h"
#include "usb_kbd.h"
#include "ipc_queue.h"
#include "build_version.h"
#include "ipc_stream.h"
#include "ipc_proc.h"
#include "pipe.h"
#include "setup.h"
#include "ksem.h"
#include "workq.h"
#include "ksvc.h"
#include "picowal_db.h"
#include "el2.h"
#include "crypto.h"
#include "watchdog.h"
#include "fat32.h"
#include "pios_addr.h"
#include "picoscript.h"
#include "keystore.h"
#include "tls.h"
#include "brotli.h"
#include "x509.h"
#include "acme.h"
#include "abi.h"
#include "mmio.h"

/* ---- libc replacements (linked globally for compiler-generated calls) ---- */

void *memset(void *dst, int c, usize n) {
    u8 *p = (u8 *)dst;
    while (n--) *p++ = (u8)c;
    return dst;
}

void *memcpy(void *dst, const void *src, usize n) {
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    while (n--) *d++ = *s++;
    return dst;
}

int memcmp(const void *a, const void *b, usize n) {
    const u8 *pa = (const u8 *)a;
    const u8 *pb = (const u8 *)b;
    while (n--) {
        if (*pa != *pb) return *pa - *pb;
        pa++; pb++;
    }
    return 0;
}

u32 pios_strlen(const char *s) {
    u32 n = 0;
    while (*s++) n++;
    return n;
}

/* ---- Version + network configuration (static - no ARP/DHCP) ---- */

#define MY_IP       IP4(192, 168, 0, 200)
#define MY_GW       IP4(192, 168, 0, 1)
#define MY_MASK     IP4(255, 255, 0, 0)

/* Gateway MAC - MUST be configured (no ARP to discover it) */
static const u8 MY_GW_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* Static neighbor: the dev host PC. Pre-populates the ARP/neighbor table
 * so the Pi can reply to and originate frames to this peer even when
 * inbound ARP processing is suspect. */
#define HOST_PC_IP  IP4(192, 168, 218, 9)
static const u8 HOST_PC_MAC[6] = { 0x04, 0xBF, 0x1B, 0xE1, 0xD7, 0x78 };

/* ---- Echo servers ---- */
#define ECHO_UDP_PORT  7
#define ECHO_TCP_PORT  7
#define HTTP_TCP_PORT  80
#define HTTPS_TLS_TCP_PORT 443
#define ADMIN_STATUS_TCP_PORT  8080
#define ADMIN_REBOOT_TCP_PORT  8081
#define ADMIN_UPDATE_TCP_PORT  8082
#define DEBUG_TCP_PORT 2323
#define DEBUG_TCP_LINE_MAX 256
#define HTTP_AUTH_ENABLED 0
#define HTTP_SIMPLE_MODE 0
#define HTTP_DIAG_VERBOSE 0
#define HTTP_ADMIN_EXPERIMENTAL 0
#define HTTP_TRACE_RING_VERBOSE 0
#define HTTP_TRACE_UART_VERBOSE 0
#define HOTPATCH_SLOT_BYTES (PIOS_STAGE2_END_OFFSET + 1U)
#define HOTPATCH_SLOT_MAGIC PIOS_RESERVED_HEADER_MAGIC
#define HTTP_LOG_RING_SIZE 64
#define ADMIN_HTTP_REQ_MAX 8192
#define ADMIN_HTTP_RESP_MAX 4096
#define ADMIN_SERVICE_WATCHDOG_MS 180000ULL
#define ADMIN_CLIENT_STALL_MS 10000ULL

#define HTTP_EVT_LISTEN       1U
#define HTTP_EVT_ACCEPT       2U
#define HTTP_EVT_RX           3U
#define HTTP_EVT_COMPLETE     4U
#define HTTP_EVT_BUILD_ENTER  5U
#define HTTP_EVT_BUILD_EXIT   6U
#define HTTP_EVT_TX           7U
#define HTTP_EVT_CLOSE        8U
#define HTTP_EVT_ABORT        9U
#define HTTP_EVT_RESET        10U
#define HTTP_EVT_BAD_STATE    11U
#define HTTP_EVT_STATUS_ENTER 12U
#define HTTP_EVT_STATUS_EXIT  13U
#define HTTP_EVT_ADMIN_BUILD  14U

#define HTTP_ROUTE_UNKNOWN    0U
#define HTTP_ROUTE_ROOT       1U
#define HTTP_ROUTE_STATUS     2U
#define HTTP_ROUTE_REBOOT     3U
#define HTTP_ROUTE_LOGS       4U
#define HTTP_ROUTE_UPDATE     5U
#define HTTP_ROUTE_HOTPATCH   6U
#define HTTP_ROUTE_PLACEHOLDER 7U
#define HTTP_ROUTE_NOT_FOUND  8U
#define HTTP_ROUTE_NETSTAT    9U
#define HTTP_ROUTE_ACME       10U

#define HTTP_ERR_NONE         0U
#define HTTP_ERR_RESP_TIMEOUT 1U
#define HTTP_ERR_REQ_TIMEOUT  2U
#define HTTP_ERR_BAD_STATE    3U
#define HTTP_ERR_HEADER_BIG   4U

static tcp_conn_t echo_listen_conn = -1;
static tcp_conn_t echo_client_conn = -1;
static tcp_conn_t http_listen_conn = -1;
static tcp_conn_t http_client_conn = -1;
static tcp_conn_t https_tls_listen_conn = -1;
static tcp_conn_t https_tls_tcp_conn = -1;
static tls_conn_t https_tls_conn = -1;
static u8 https_tls_req_buf[512];
static u32 https_tls_req_len;
static bool https_tls_accepted;
static bool https_tls_response_sent;
static u64 https_tls_last_activity_ms;
static u8 http_req_buf[1024];
static u32 http_req_len;
static bool http_auth_checked;
static bool http_auth_ok;
static bool http_reboot_pending;
static i32 ksvc_net_id = -1;
static i32 ksvc_tcp_id = -1;
static i32 ksvc_debug_id = -1;
static i32 ksvc_ui_id = -1;
static i32 ksvc_dashboard_id = -1;
static i32 ksvc_timer_id = -1;
static char http_resp_buf[16000];
static u32 http_resp_len;
static u32 http_resp_off;
static u32 http_last_state;
static u32 http_last_readable;
static u32 http_last_writable;
static u32 http_last_write;
static u32 http_complete_tick;
static bool http_prefix_dumped;
static bool http_req_prefix_dumped;
static u64 http_last_activity_ms;
static char http_last_req_prefix[25];
static char http_last_resp_prefix[25];
static struct {
    u64 accepts;
    u64 reads;
    u64 built;
    u64 write_calls;
    u64 write_bytes;
    u64 write_zero;
    u64 closes;
    u64 aborts;
    u64 unauthorized;
    u64 not_found;
    u32 event;
    u32 route;
    u32 error;
    u32 conn;
    u32 build_len;
    u32 body_off;
    u32 content_len;
    u32 request_done;
} http_diag;

struct admin_http_service {
    const char *name;
    u16 port;
    tcp_conn_t listen_conn;
    tcp_conn_t client_conn;
    u8 req[ADMIN_HTTP_REQ_MAX];
    u32 req_len;
    char resp[ADMIN_HTTP_RESP_MAX];
    u32 resp_len;
    u32 resp_off;
    u64 last_activity_ms;
    u64 last_ok_ms;
    u32 completions;
};

static struct admin_http_service admin_status_svc = {
    .name = "status", .port = ADMIN_STATUS_TCP_PORT,
    .listen_conn = -1, .client_conn = -1
};
static struct admin_http_service admin_reboot_svc = {
    .name = "reboot", .port = ADMIN_REBOOT_TCP_PORT,
    .listen_conn = -1, .client_conn = -1
};
static struct admin_http_service admin_update_svc = {
    .name = "update", .port = ADMIN_UPDATE_TCP_PORT,
    .listen_conn = -1, .client_conn = -1
};

struct http_log_entry {
    u32 seq;
    u32 tick_ms;
    const char *event;
    u32 a;
    u32 b;
};
static struct http_log_entry http_log_ring[HTTP_LOG_RING_SIZE];
static u32 http_log_seq;
struct ota_update_state {
    bool active;
    u32 target_slot;
    u32 target_slot_offset;
    u32 total;
    u32 received;
    u32 chunks;
    u32 commits;
    u32 errors;
    const char *last_error;
};
static struct ota_update_state ota_update;
static tcp_conn_t debug_listen_conn = -1;
static tcp_conn_t debug_client_conn = -1;
static bool debug_tcp_unlocked;
static char debug_tcp_line[DEBUG_TCP_LINE_MAX];
static u32 debug_tcp_len;
static u32 debug_tcp_iac_skip;
static volatile u64 core0_eth_irq_count;
static volatile u32 core0_eth_irq_last_mip;
static volatile u32 core0_eth_irq_last_macb_isr;
static volatile bool core0_eth_irq_oneshot;
static volatile bool core0_eth_irq_deferred_quench;
static volatile u32 core0_eth_irq_quench_passes;
static u32 core0_eth_source_diag[24];
static volatile u32 core0_eth_source_diag_seq;

static bool ui_streq(const char *a, const char *b);
static const char *ui_proc_state_str(u32 s);
static const char *ui_priority_str(u32 p);
static u32 http_header_body_offset(const u8 *buf, u32 len);
static bool http_content_length(const u8 *req, u32 len, u32 *out);
static void ui_console_write(const char *s);
static void ui_console_exec(char *line);
static void debug_tcp_poll(void);
static void net_services_listen(void);
static void admin_services_listen(void);
static void admin_services_poll(void);
static void http_log_event(const char *event, u32 a, u32 b);
static u32 http_build_kernel_update_response(char *out, u32 max, const u8 *req, u32 req_len);
static void pios_bootctrl_mark_success(void);
static void core0_sched_snapshot(u64 *wake, u64 *wfi_count, u64 *idle_ticks,
                                 u64 *total_ticks, u32 *busy_permille,
                                 u32 *last_flags);
static void core0_eth_irq_handler(void);
static bool core0_eth_irq_drain_and_quench(bool host_route);
static const char *tcp_state_name(u32 state);
static const char *tcp_owner_label(u16 port);
static bool http_request_complete(const u8 *req, u32 len);

static void debug_tcp_send_len(const char *s, u32 len)
{
    if (debug_client_conn < 0 || !debug_tcp_unlocked)
        return;
    if (tcp_state(debug_client_conn) != TCP_ESTABLISHED)
        return;
    (void)tcp_write(debug_client_conn, s, len);
}

static void debug_tcp_send(const char *s)
{
    debug_tcp_send_len(s, pios_strlen(s));
}

static bool debug_tcp_unlock_line(const char *line)
{
    return ui_streq(line, "unlock pios");
}

static void debug_tcp_close(void)
{
    if (debug_client_conn >= 0)
        tcp_close(debug_client_conn);
    debug_client_conn = -1;
    debug_tcp_unlocked = false;
    debug_tcp_len = 0;
    debug_tcp_iac_skip = 0;
}

static void debug_tcp_handle_line(void)
{
    debug_tcp_line[debug_tcp_len] = 0;
    debug_tcp_len = 0;

    if (!debug_tcp_unlocked) {
        if (debug_tcp_unlock_line(debug_tcp_line)) {
            debug_tcp_unlocked = true;
            debug_tcp_send("\r\nOK: debug console unlocked\r\npios> ");
        } else {
            static const char msg[] = "\r\nERR: type 'unlock pios' first\r\ndebug> ";
            (void)tcp_write(debug_client_conn, msg, sizeof(msg) - 1);
        }
        return;
    }

    if (debug_tcp_line[0])
        ui_console_exec(debug_tcp_line);
    debug_tcp_send("\r\npios> ");
}

static void debug_tcp_feed(u8 c)
{
    if (debug_tcp_iac_skip) {
        debug_tcp_iac_skip--;
        return;
    }
    if (c == 0xFF) {
        debug_tcp_iac_skip = 2; /* ignore simple TELNET IAC command triplets */
        return;
    }
    if (c == '\r')
        return;
    if (c == '\n') {
        debug_tcp_handle_line();
        return;
    }
    if (c == 8 || c == 127) {
        if (debug_tcp_len > 0) {
            debug_tcp_len--;
            static const char bs[] = "\b \b";
            (void)tcp_write(debug_client_conn, bs, sizeof(bs) - 1);
        }
        return;
    }
    if (c < 0x20 || c > 0x7E)
        return;
    if (debug_tcp_len + 1 >= sizeof(debug_tcp_line)) {
        static const char msg[] = "\r\nERR: line too long\r\ndebug> ";
        (void)tcp_write(debug_client_conn, msg, sizeof(msg) - 1);
        debug_tcp_len = 0;
        return;
    }
    debug_tcp_line[debug_tcp_len++] = (char)c;
    (void)tcp_write(debug_client_conn, &c, 1);
}

static void debug_tcp_poll(void)
{
    if (debug_client_conn < 0 && debug_listen_conn >= 0) {
        debug_client_conn = tcp_accept(debug_listen_conn);
        if (debug_client_conn >= 0) {
            debug_tcp_unlocked = false;
            debug_tcp_len = 0;
            debug_tcp_iac_skip = 0;
            static const char banner[] =
                "\r\nPIOS TCP debug console\r\n"
                "WARNING: commands run on the live kernel.\r\n"
                "Type: unlock pios\r\n"
                "debug> ";
            (void)tcp_write(debug_client_conn, banner, sizeof(banner) - 1);
        }
    }

    if (debug_client_conn < 0)
        return;

    u32 st = tcp_state(debug_client_conn);
    if (st == TCP_ESTABLISHED) {
        static u8 buf[128];
        u32 n = tcp_read(debug_client_conn, buf, sizeof(buf));
        for (u32 i = 0; i < n; i++)
            debug_tcp_feed(buf[i]);
    } else if (st == TCP_CLOSED || st >= TCP_CLOSING) {
        debug_tcp_close();
    }
}

static void net_services_listen(void)
{
    echo_client_conn = -1;
    http_client_conn = -1;
    https_tls_tcp_conn = -1;
    https_tls_conn = -1;
    https_tls_req_len = 0;
    https_tls_accepted = false;
    https_tls_response_sent = false;
    debug_client_conn = -1;
    debug_tcp_unlocked = false;
    debug_tcp_len = 0;
    debug_tcp_iac_skip = 0;

    echo_listen_conn = -1;
    http_listen_conn = tcp_listen(HTTP_TCP_PORT);
    https_tls_listen_conn = tcp_listen(HTTPS_TLS_TCP_PORT);
    debug_listen_conn = tcp_listen(DEBUG_TCP_PORT);
    admin_services_listen();
    http_log_event("net-listen", HTTP_TCP_PORT, ADMIN_STATUS_TCP_PORT);
}

static void http_log_event(const char *event, u32 a, u32 b)
{
    u32 slot = http_log_seq % HTTP_LOG_RING_SIZE;
    http_log_ring[slot].seq = http_log_seq++;
    http_log_ring[slot].tick_ms = (u32)timer_monotonic_ms();
    http_log_ring[slot].event = event;
    http_log_ring[slot].a = a;
    http_log_ring[slot].b = b;
}

static void http_trace(u32 event, u32 route, u32 a, u32 b)
{
    http_diag.event = event;
    http_diag.route = route;
    if (event != HTTP_EVT_ABORT && event != HTTP_EVT_BAD_STATE)
        http_diag.error = HTTP_ERR_NONE;
#if HTTP_TRACE_RING_VERBOSE
    http_log_event("http-trace", (event << 24) | (route << 16) | (a & 0xFFFFU), b);
#else
    if (event == HTTP_EVT_ABORT || event == HTTP_EVT_BAD_STATE)
        http_log_event("http-error", (event << 24) | (route << 16) | (a & 0xFFFFU), b);
#endif
#if HTTP_TRACE_UART_VERBOSE
    uart_puts("[http-trace] ev=");
    uart_hex(event);
    uart_puts(" route=");
    uart_hex(route);
    uart_puts(" a=");
    uart_hex(a);
    uart_puts(" b=");
    uart_hex(b);
    uart_puts("\n");
#else
    (void)a;
    (void)b;
#endif
}

static void http_append(char *out, u32 *len, u32 max, const char *s)
{
    if (!out || !len || max == 0)
        return;
    while (*s && *len < max - 1)
        out[(*len)++] = *s++;
    out[*len] = 0;
}

static void http_append_u64(char *out, u32 *len, u32 max, u64 v)
{
    char tmp[21];
    u32 n = 0;
    if (v == 0) {
        http_append(out, len, max, "0");
        return;
    }
    while (v && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (n && *len < max - 1)
        out[(*len)++] = tmp[--n];
    out[*len] = 0;
}

static void http_append_ip4(char *out, u32 *len, u32 max, u32 ip)
{
    http_append_u64(out, len, max, (ip >> 24) & 0xFF);
    http_append(out, len, max, ".");
    http_append_u64(out, len, max, (ip >> 16) & 0xFF);
    http_append(out, len, max, ".");
    http_append_u64(out, len, max, (ip >> 8) & 0xFF);
    http_append(out, len, max, ".");
    http_append_u64(out, len, max, ip & 0xFF);
}

static void http_append_fw_ip_spec(char *out, u32 *len, u32 max,
                                   u32 flags, bool src, const nic_filter_rule_t *r)
{
    u32 exact = src ? NIC_FILTER_IP_FROM : NIC_FILTER_IP_TO;
    u32 range = src ? NIC_FILTER_IP_FROM_RANGE : NIC_FILTER_IP_TO_RANGE;
    u32 ip = src ? r->ip_from : r->ip_to;
    u32 mask = src ? r->ip_from_mask : r->ip_to_mask;
    u32 end = src ? r->ip_from_end : r->ip_to_end;
    if (flags & range) {
        http_append_ip4(out, len, max, ip);
        http_append(out, len, max, "-");
        http_append_ip4(out, len, max, end);
    } else if (flags & exact) {
        http_append_ip4(out, len, max, ip);
        if (mask != 0) {
            http_append(out, len, max, "/");
            http_append_ip4(out, len, max, mask);
        }
    } else {
        http_append(out, len, max, "any");
    }
}

static void http_append_firewall_list(char *out, u32 *len, u32 max)
{
    u32 n = nic_filter_count();
    http_append(out, len, max, "Firewall rules=");
    http_append_u64(out, len, max, n);
    http_append(out, len, max, " (first match wins; inbound default deny, outbound default allow)\n");
    for (u32 i = 0; i < n; i++) {
        nic_filter_rule_t r;
        if (!nic_filter_get(i, &r))
            continue;
        http_append_u64(out, len, max, i);
        http_append(out, len, max, ": ");
        http_append(out, len, max, r.action == NIC_FILTER_ALLOW ? "allow " : "deny ");
        if (r.direction == NIC_FILTER_DIR_IN) http_append(out, len, max, "in ");
        else if (r.direction == NIC_FILTER_DIR_OUT) http_append(out, len, max, "out ");
        else http_append(out, len, max, "both ");
        if ((r.flags & NIC_FILTER_ETHERTYPE) && r.ethertype == ETH_P_ARP) {
            http_append(out, len, max, "arp ");
        } else if ((r.flags & NIC_FILTER_IP_PROTO) && r.ip_proto == IP_PROTO_TCP) {
            http_append(out, len, max, "tcp ");
        } else if ((r.flags & NIC_FILTER_IP_PROTO) && r.ip_proto == IP_PROTO_UDP) {
            http_append(out, len, max, "udp ");
        } else if ((r.flags & NIC_FILTER_IP_PROTO) && r.ip_proto == IP_PROTO_ICMP) {
            http_append(out, len, max, "icmp ");
        } else if ((r.flags & NIC_FILTER_ETHERTYPE) && r.ethertype == ETH_P_IP) {
            http_append(out, len, max, "ip ");
        } else {
            http_append(out, len, max, "any ");
        }
        http_append(out, len, max, "src=");
        http_append_fw_ip_spec(out, len, max, r.flags, true, &r);
        http_append(out, len, max, " dst=");
        http_append_fw_ip_spec(out, len, max, r.flags, false, &r);
        if (r.flags & NIC_FILTER_TCP_PORT_TO) {
            http_append(out, len, max, " tcp_dport=");
            http_append_u64(out, len, max, r.tcp_port_to);
        }
        if (r.flags & NIC_FILTER_TCP_PORT_TO_RANGE) {
            http_append(out, len, max, " tcp_dport=");
            http_append_u64(out, len, max, r.tcp_port_to);
            http_append(out, len, max, "-");
            http_append_u64(out, len, max, r.tcp_port_to_end);
        }
        if (r.flags & NIC_FILTER_TCP_PORT_FROM) {
            http_append(out, len, max, " tcp_sport=");
            http_append_u64(out, len, max, r.tcp_port_from);
        }
        if (r.flags & NIC_FILTER_TCP_PORT_FROM_RANGE) {
            http_append(out, len, max, " tcp_sport=");
            http_append_u64(out, len, max, r.tcp_port_from);
            http_append(out, len, max, "-");
            http_append_u64(out, len, max, r.tcp_port_from_end);
        }
        if (r.flags & NIC_FILTER_UDP_PORT_TO) {
            http_append(out, len, max, " udp_dport=");
            http_append_u64(out, len, max, r.udp_port_to);
        }
        if (r.flags & NIC_FILTER_UDP_PORT_TO_RANGE) {
            http_append(out, len, max, " udp_dport=");
            http_append_u64(out, len, max, r.udp_port_to);
            http_append(out, len, max, "-");
            http_append_u64(out, len, max, r.udp_port_to_end);
        }
        if (r.flags & NIC_FILTER_UDP_PORT_FROM) {
            http_append(out, len, max, " udp_sport=");
            http_append_u64(out, len, max, r.udp_port_from);
        }
        if (r.flags & NIC_FILTER_UDP_PORT_FROM_RANGE) {
            http_append(out, len, max, " udp_sport=");
            http_append_u64(out, len, max, r.udp_port_from);
            http_append(out, len, max, "-");
            http_append_u64(out, len, max, r.udp_port_from_end);
        }
        http_append(out, len, max, "\n");
    }
}

static void http_append_route_table(char *out, u32 *len, u32 max)
{
    struct net_route_entry routes[NET_ROUTE_MAX];
    u32 n = net_route_snapshot(routes, NET_ROUTE_MAX);
    http_append(out, len, max, "ROUTE DST MASK GW FLAGS PFX\n");
    for (u32 i = 0; i < n; i++) {
        http_append_u64(out, len, max, i);
        http_append(out, len, max, " ");
        http_append_ip4(out, len, max, routes[i].dst);
        http_append(out, len, max, " ");
        http_append_ip4(out, len, max, routes[i].mask);
        http_append(out, len, max, " ");
        http_append_ip4(out, len, max, routes[i].gateway);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, routes[i].flags);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, routes[i].prefix_len);
        http_append(out, len, max, "\n");
    }
}

static const char *net_egress_source_name(u8 s)
{
    switch (s) {
    case NET_EGRESS_MAC_BCAST:     return "broadcast";
    case NET_EGRESS_MAC_GW_STATIC: return "gateway-static";
    case NET_EGRESS_MAC_NEIGHBOR:  return "neighbor-static";
    case NET_EGRESS_MAC_ARP:       return "arp";
    case NET_EGRESS_MAC_NO_ROUTE:  return "no-route";
    case NET_EGRESS_MAC_NO_MAC:    return "no-mac";
    default:                       return "none";
    }
}

static void http_append_mac6(char *out, u32 *len, u32 max, const u8 *mac)
{
    static const char hx[] = "0123456789ABCDEF";
    for (u32 i = 0; i < 6; i++) {
        if (i) http_append(out, len, max, ":");
        char b[3] = { hx[mac[i] >> 4], hx[mac[i] & 0xF], 0 };
        http_append(out, len, max, b);
    }
}

static void http_append_net_egress_trace(char *out, u32 *len, u32 max)
{
    struct net_egress_snapshot e;
    net_egress_snapshot(&e);
    http_append(out, len, max, "egress resolves=");
    http_append_u64(out, len, max, e.resolve_calls);
    http_append(out, len, max, " no_route=");
    http_append_u64(out, len, max, e.no_route);
    http_append(out, len, max, " no_mac=");
    http_append_u64(out, len, max, e.no_mac);
    http_append(out, len, max, " udp_attempts=");
    http_append_u64(out, len, max, e.udp_attempts);
    http_append(out, len, max, " udp_ok=");
    http_append_u64(out, len, max, e.udp_ok);
    http_append(out, len, max, " udp_fail=");
    http_append_u64(out, len, max, e.udp_fail);
    http_append(out, len, max, "\nlast dst=");
    http_append_ip4(out, len, max, e.last_dst_ip);
    http_append(out, len, max, " next_hop=");
    http_append_ip4(out, len, max, e.last_next_hop);
    http_append(out, len, max, " route=");
    http_append_ip4(out, len, max, e.last_route_dst);
    http_append(out, len, max, "/");
    http_append_u64(out, len, max, e.last_route_prefix);
    http_append(out, len, max, " gw=");
    http_append_ip4(out, len, max, e.last_route_gateway);
    http_append(out, len, max, " flags=");
    http_append_u64(out, len, max, e.last_route_flags);
    http_append(out, len, max, " mac_source=");
    http_append(out, len, max, net_egress_source_name(e.last_mac_source));
    http_append(out, len, max, " mac=");
    http_append_mac6(out, len, max, e.last_mac);
    http_append(out, len, max, "\nlast_udp sport=");
    http_append_u64(out, len, max, e.last_udp_src_port);
    http_append(out, len, max, " dport=");
    http_append_u64(out, len, max, e.last_udp_dst_port);
    http_append(out, len, max, " len=");
    http_append_u64(out, len, max, e.last_udp_len);
    http_append(out, len, max, " dst=");
    http_append_ip4(out, len, max, e.last_udp_dst_ip);
    http_append(out, len, max, " next_hop=");
    http_append_ip4(out, len, max, e.last_udp_next_hop);
    http_append(out, len, max, " mac_source=");
    http_append(out, len, max, net_egress_source_name(e.last_udp_mac_source));
    http_append(out, len, max, " mac=");
    http_append_mac6(out, len, max, e.last_udp_mac);
    http_append(out, len, max, " ok=");
    http_append(out, len, max, e.last_udp_ok ? "yes" : "no");
    http_append(out, len, max, "\n");
}

static void http_append_arp_table(char *out, u32 *len, u32 max)
{
    struct arp_snapshot_entry e[ARP_TABLE_SIZE];
    u32 n = arp_snapshot(e, ARP_TABLE_SIZE);
    http_append(out, len, max, "ARP IP MAC STATE RETRY CONS AGE_MS\n");
    for (u32 i = 0; i < n; i++) {
        http_append_ip4(out, len, max, e[i].ip);
        http_append(out, len, max, " ");
        for (u32 j = 0; j < 6; j++) {
            static const char hx[] = "0123456789ABCDEF";
            if (j) http_append(out, len, max, ":");
            char b[3] = { hx[e[i].mac[j] >> 4], hx[e[i].mac[j] & 0xF], 0 };
            http_append(out, len, max, b);
        }
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, e[i].state);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, e[i].retries);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, e[i].consistency);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, e[i].age_ms);
        http_append(out, len, max, "\n");
    }
}

static bool http_char_ieq(char a, char b)
{
    if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
    if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
    return a == b;
}

static bool http_match_ci(const u8 *p, const char *s)
{
    while (*s) {
        if (!http_char_ieq((char)*p++, *s++))
            return false;
    }
    return true;
}

static i32 http_b64_val(u8 c)
{
    if (c >= 'A' && c <= 'Z') return (i32)(c - 'A');
    if (c >= 'a' && c <= 'z') return (i32)(c - 'a' + 26);
    if (c >= '0' && c <= '9') return (i32)(c - '0' + 52);
    if (c == '+') return 62;
    if (c == '/') return 63;
    if (c == '=') return -2;
    return -1;
}

static bool http_b64_decode(const u8 *in, u32 in_len, u8 *out, u32 out_max, u32 *out_len)
{
    u32 o = 0;
    u32 i = 0;
    while (i < in_len) {
        i32 v[4];
        for (u32 j = 0; j < 4; j++) {
            if (i >= in_len) return false;
            v[j] = http_b64_val(in[i++]);
            if (v[j] == -1) return false;
        }
        if (v[0] < 0 || v[1] < 0) return false;
        u32 n = ((u32)v[0] << 18) | ((u32)v[1] << 12);
        if (v[2] >= 0) n |= (u32)v[2] << 6;
        if (v[3] >= 0) n |= (u32)v[3];
        if (o >= out_max) return false;
        out[o++] = (u8)(n >> 16);
        if (v[2] != -2) {
            if (o >= out_max) return false;
            out[o++] = (u8)(n >> 8);
        }
        if (v[3] != -2) {
            if (o >= out_max) return false;
            out[o++] = (u8)n;
        }
        if (v[2] == -2 || v[3] == -2)
            break;
    }
    if (out_len) *out_len = o;
    return true;
}

static bool http_admin_authorized(const u8 *req, u32 len)
{
    static const char auth_hdr[] = "authorization:";
    static const char basic[] = "basic ";

    for (u32 i = 0; i + sizeof(auth_hdr) - 1 < len; i++) {
        if (i != 0 && req[i - 1] != '\n')
            continue;
        if (!http_match_ci(&req[i], auth_hdr))
            continue;
        i += (u32)sizeof(auth_hdr) - 1;
        while (i < len && (req[i] == ' ' || req[i] == '\t')) i++;
        if (i + sizeof(basic) - 1 >= len || !http_match_ci(&req[i], basic))
            return false;
        i += (u32)sizeof(basic) - 1;

        u32 start = i;
        while (i < len && req[i] != '\r' && req[i] != '\n' && req[i] != ' ' && req[i] != '\t')
            i++;

        u8 decoded[96];
        u32 decoded_len = 0;
        if (!http_b64_decode(&req[start], i - start, decoded, sizeof(decoded) - 1, &decoded_len))
            return false;
        decoded[decoded_len] = 0;

        char user[32];
        char pass[64];
        u32 u = 0, p = 0;
        u32 j = 0;
        while (j < decoded_len && decoded[j] != ':' && u < sizeof(user) - 1)
            user[u++] = (char)decoded[j++];
        if (j >= decoded_len || decoded[j] != ':')
            return false;
        j++;
        while (j < decoded_len && p < sizeof(pass) - 1)
            pass[p++] = (char)decoded[j++];
        user[u] = 0;
        pass[p] = 0;

        u32 principal_id = PRINCIPAL_ROOT;
        if (!principal_auth(user, pass, &principal_id))
            return false;
        return principal_has_cap(principal_id, PRINCIPAL_ADMIN);
    }
    return false;
}

static bool http_try_early_auth(const u8 *req, u32 len, bool *complete, bool *authorized)
{
    static const char auth_hdr[] = "authorization:";
    if (complete) *complete = false;
    if (authorized) *authorized = false;

    for (u32 i = 0; i + sizeof(auth_hdr) - 1 < len; i++) {
        if (i != 0 && req[i - 1] != '\n')
            continue;
        if (!http_match_ci(&req[i], auth_hdr))
            continue;

        u32 line_end = i;
        while (line_end < len && req[line_end] != '\n')
            line_end++;
        if (line_end >= len)
            return false;

        if (complete) *complete = true;
        if (authorized) *authorized = http_admin_authorized(&req[i], line_end - i);
        return true;
    }
    return false;
}

static u32 http_build_unauthorized(char *out, u32 max)
{
    u32 len = 0;
    http_append(out, &len, max,
        "HTTP/1.0 401 Unauthorized\r\n"
        "WWW-Authenticate: Basic realm=\"PIOS Admin\"\r\n"
        "Content-Type: text/plain\r\n"
        "Connection: close\r\n\r\n"
        "PIOS admin authentication required.\n");
    return len;
}

static bool http_request_path_start(const u8 *req, u32 len, u32 *out)
{
    if (!req || len < 3 || !out)
        return false;
    u32 i = 0;
    while (i < len && req[i] != ' ' && req[i] != '\r' && req[i] != '\n')
        i++;
    if (i == 0 || i >= len || req[i] != ' ')
        return false;
    i++;
    if (i >= len || req[i] != '/')
        return false;
    *out = i;
    return true;
}

static bool http_request_is_root_get(const u8 *req, u32 len)
{
    if (!req || len < 6)
        return false;
    if (req[0] != 'G' || req[1] != 'E' || req[2] != 'T' || req[3] != ' ')
        return false;
    if (req[4] != '/')
        return false;
    return req[5] == ' ' || req[5] == '?' || req[5] == '\r' || req[5] == '\n';
}

static bool http_request_path_is(const u8 *req, u32 len, const char *path)
{
    if (!req || !path || len < 6)
        return false;
    u32 i = 0;
    if (!http_request_path_start(req, len, &i))
        return false;
    while (*path) {
        if (i >= len || req[i++] != (u8)*path++)
            return false;
    }
    return i < len && (req[i] == ' ' || req[i] == '?' || req[i] == '\r' || req[i] == '\n');
}

static bool http_request_path_prefix_token(const u8 *req, u32 len, const char *prefix,
                                           char *out, u32 out_max)
{
    u32 i = 0;
    u32 o = 0;
    if (!out || out_max == 0) return false;
    out[0] = 0;
    if (!http_request_path_start(req, len, &i))
        return false;
    while (*prefix) {
        if (i >= len || req[i++] != (u8)*prefix++)
            return false;
    }
    while (i < len && req[i] != ' ' && req[i] != '?' && req[i] != '\r' && req[i] != '\n') {
        if (o + 1 >= out_max)
            return false;
        out[o++] = (char)req[i++];
    }
    out[o] = 0;
    return o > 0;
}

static u32 http_route_id(const u8 *req, u32 len)
{
    char token[ACME_TOKEN_MAX];
    if (http_request_path_prefix_token(req, len, "/.well-known/acme-challenge/", token, sizeof(token)))
        return HTTP_ROUTE_ACME;
    if (http_request_path_is(req, len, "/api/status"))
        return HTTP_ROUTE_STATUS;
    if (http_request_path_is(req, len, "/api/netstat"))
        return HTTP_ROUTE_NETSTAT;
    if (http_request_path_is(req, len, "/api/admin/reboot"))
        return HTTP_ROUTE_REBOOT;
    if (http_request_path_is(req, len, "/api/admin/log-stream") ||
        http_request_path_is(req, len, "/api/logs") ||
        http_request_path_is(req, len, "/logs"))
        return HTTP_ROUTE_LOGS;
    if (http_request_path_is(req, len, "/api/admin/kernel-update"))
        return HTTP_ROUTE_UPDATE;
    if (http_request_path_is(req, len, "/api/admin/hotpatch-kernel"))
        return HTTP_ROUTE_HOTPATCH;
    if (http_request_path_is(req, len, "/api/terminal") ||
        http_request_path_is(req, len, "/api/process") ||
        http_request_path_is(req, len, "/api/user") ||
        http_request_path_is(req, len, "/api/walfs"))
        return HTTP_ROUTE_PLACEHOLDER;
    if (http_request_is_root_get(req, len))
        return HTTP_ROUTE_ROOT;
    return HTTP_ROUTE_NOT_FOUND;
}

static i32 http_hex_value(u8 c)
{
    if (c >= '0' && c <= '9') return (i32)(c - '0');
    if (c >= 'a' && c <= 'f') return (i32)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (i32)(c - 'A' + 10);
    return -1;
}

static bool http_query_value(const u8 *req, u32 len, const char *path,
                             const char *key, char *out, u32 out_max)
{
    if (!out || out_max == 0 || !key)
        return false;
    out[0] = 0;
    if (!http_request_path_is(req, len, path))
        return false;

    u32 i = 0;
    if (!http_request_path_start(req, len, &i))
        return false;
    while (i < len && req[i] != '?' && req[i] != ' ' && req[i] != '\r' && req[i] != '\n')
        i++;
    if (i >= len || req[i] != '?')
        return false;
    i++;

    while (i < len && req[i] != ' ' && req[i] != '\r' && req[i] != '\n') {
        u32 k = 0;
        u32 start = i;
        bool key_bad = false;
        while (i < len && req[i] != '=' && req[i] != '&' && req[i] != ' ' &&
               req[i] != '\r' && req[i] != '\n') {
            if (!key_bad && key[k] && req[i] == (u8)key[k])
                k++;
            else
                key_bad = true;
            i++;
        }
        bool match = !key_bad && key[k] == 0;
        if (i < len && req[i] == '=') {
            i++;
            u32 o = 0;
            while (i < len && req[i] != '&' && req[i] != ' ' &&
                   req[i] != '\r' && req[i] != '\n') {
                u8 c = req[i++];
                if (c == '+') c = ' ';
                else if (c == '%' && i + 1 < len) {
                    i32 hi = http_hex_value(req[i]);
                    i32 lo = http_hex_value(req[i + 1]);
                    if (hi >= 0 && lo >= 0) {
                        c = (u8)((hi << 4) | lo);
                        i += 2;
                    }
                }
                if (match && o + 1 < out_max)
                    out[o++] = (char)c;
            }
            if (match) {
                out[o] = 0;
                return true;
            }
        } else {
            i = start;
            while (i < len && req[i] != '&' && req[i] != ' ' &&
                   req[i] != '\r' && req[i] != '\n')
                i++;
        }
        if (i < len && req[i] == '&')
            i++;
    }
    return false;
}

static bool http_parse_u32(const char *s, u32 *out)
{
    if (!s || !*s || !out) return false;
    u32 base = 10;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        if (!*s) return false;
    }
    u32 v = 0;
    while (*s) {
        u32 d;
        char c = *s++;
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return false;
        if (d >= base) return false;
        v = v * base + d;
    }
    *out = v;
    return true;
}

static bool http_parse_u64(const char *s, u64 *out)
{
    if (!s || !*s || !out) return false;
    u32 base = 10;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        if (!*s) return false;
    }
    u64 v = 0;
    while (*s) {
        u32 d;
        char c = *s++;
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return false;
        if (d >= base) return false;
        v = v * (u64)base + (u64)d;
    }
    *out = v;
    return true;
}

static bool http_mem_width(const char *s, u32 *out)
{
    u32 w = 0;
    if (!http_parse_u32(s, &w))
        return false;
    if (w != 1 && w != 2 && w != 4 && w != 8)
        return false;
    *out = w;
    return true;
}

static u64 http_mem_read(u64 addr, u32 width)
{
    if (width == 1) return *(volatile u8 *)(usize)addr;
    if (width == 2) return *(volatile u16 *)(usize)addr;
    if (width == 8) return *(volatile u64 *)(usize)addr;
    return *(volatile u32 *)(usize)addr;
}

static void http_mem_write(u64 addr, u64 value, u32 width)
{
    if (width == 1) *(volatile u8 *)(usize)addr = (u8)value;
    else if (width == 2) *(volatile u16 *)(usize)addr = (u16)value;
    else if (width == 8) *(volatile u64 *)(usize)addr = value;
    else *(volatile u32 *)(usize)addr = (u32)value;
    dsb();
    isb();
}

static void http_append_json_string(char *out, u32 *len, u32 max, const char *s)
{
    http_append(out, len, max, "\"");
    if (s) {
        while (*s && *len < max - 1) {
            u8 c = (u8)*s++;
            if (c == '"' || c == '\\') {
                if (*len < max - 1) out[(*len)++] = '\\';
                if (*len < max - 1) out[(*len)++] = (char)c;
            } else if (c == '\n') {
                http_append(out, len, max, "\\n");
            } else if (c == '\r') {
                http_append(out, len, max, "\\r");
            } else if (c == '\t') {
                http_append(out, len, max, "\\t");
            } else if (c >= 32 && c < 127) {
                out[(*len)++] = (char)c;
            }
        }
        out[*len] = 0;
    }
    http_append(out, len, max, "\"");
}

static void http_append_json_bytes_text(char *out, u32 *len, u32 max, const u8 *data, u32 n)
{
    http_append(out, len, max, "\"");
    static const char hx[] = "0123456789ABCDEF";
    for (u32 i = 0; i < n && *len < max - 1; i++) {
        u8 c = data[i];
        if (c == '"' || c == '\\') {
            if (*len < max - 1) out[(*len)++] = '\\';
            if (*len < max - 1) out[(*len)++] = (char)c;
        } else if (c == '\n') {
            http_append(out, len, max, "\\n");
        } else if (c == '\r') {
            http_append(out, len, max, "\\r");
        } else if (c == '\t') {
            http_append(out, len, max, "\\t");
        } else if (c >= 32 && c < 127) {
            out[(*len)++] = (char)c;
            out[*len] = 0;
        } else {
            http_append(out, len, max, "\\u00");
            if (*len < max - 1) out[(*len)++] = hx[(c >> 4) & 0xF];
            if (*len < max - 1) out[(*len)++] = hx[c & 0xF];
            out[*len] = 0;
        }
    }
    http_append(out, len, max, "\"");
}

static void http_append_hex8(char *out, u32 *len, u32 max, u8 v)
{
    static const char hx[] = "0123456789ABCDEF";
    char s[3] = { hx[(v >> 4) & 0xF], hx[v & 0xF], 0 };
    http_append(out, len, max, s);
}

static void http_append_hex32(char *out, u32 *len, u32 max, u32 v)
{
    static const char hx[] = "0123456789ABCDEF";
    char s[9];
    for (u32 i = 0; i < 8; i++)
        s[i] = hx[(v >> ((7 - i) * 4)) & 0xF];
    s[8] = 0;
    http_append(out, len, max, s);
}

static void http_append_hex64(char *out, u32 *len, u32 max, u64 v)
{
    static const char hx[] = "0123456789ABCDEF";
    char s[17];
    for (u32 i = 0; i < 16; i++)
        s[i] = hx[(v >> ((15 - i) * 4)) & 0xF];
    s[16] = 0;
    http_append(out, len, max, s);
}

static void http_append_json_metric(char *out, u32 *len, u32 max,
                                    const char *name, u64 value, bool comma)
{
    http_append(out, len, max, "\"");
    http_append(out, len, max, name);
    http_append(out, len, max, "\":");
    http_append_u64(out, len, max, value);
    if (comma) http_append(out, len, max, ",");
}

static u32 http_core_ram_used_kib(u32 core)
{
    struct core_env *e = core_env_of(core);
    if (e->id == core && e->ram_base == (u8 *)(usize)core_ram_bases[core] &&
        e->ram_end == e->ram_base + CORE_PRIV_SIZE &&
        e->heap_ptr >= e->ram_base && e->heap_ptr <= e->ram_end)
        return (u32)((usize)(e->heap_ptr - e->ram_base) >> 10);
    return 0;
}

static u32 http_build_status_json(char *out, u32 max, const u8 *req, u32 req_len)
{
    static bool boot_success_marked_by_health;
    (void)req;
    (void)req_len;
    u32 len = 0;
    http_trace(HTTP_EVT_STATUS_ENTER, HTTP_ROUTE_STATUS, req_len, max);

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");

    http_append(out, &len, max, "{\"ok\":true,\"version\":\"");
    http_append(out, &len, max, PIOS_VERSION);
    http_append(out, &len, max, "\",\"build\":\"");
    http_append(out, &len, max, PIOS_BUILD_LABEL);
    http_append(out, &len, max, "\",");
    http_append_json_metric(out, &len, max, "uptime", timer_monotonic_ms() / 1000ULL, true);
    http_append(out, &len, max, "\"ip\":\"");
    http_append_ip4(out, &len, max, net_get_our_ip());
    http_append(out, &len, max, "\",");
    http_append(out, &len, max, "\"mode\":\"minimal-json-safe\",");
    http_append(out, &len, max, "\"diag\":{");
    http_append_json_metric(out, &len, max, "event", http_diag.event, true);
    http_append_json_metric(out, &len, max, "route", http_diag.route, true);
    http_append_json_metric(out, &len, max, "error", http_diag.error, true);
    http_append_json_metric(out, &len, max, "conn", http_diag.conn, true);
    http_append_json_metric(out, &len, max, "req", http_req_len, true);
    http_append_json_metric(out, &len, max, "resp", http_resp_len, true);
    http_append_json_metric(out, &len, max, "off", http_resp_off, true);
    http_append_json_metric(out, &len, max, "body", http_diag.body_off, true);
    http_append_json_metric(out, &len, max, "clen", http_diag.content_len, true);
    u64 sched_wake = 0, sched_wfi = 0, sched_idle = 0, sched_total = 0;
    u32 sched_busy = 0, sched_flags = 0;
    core0_sched_snapshot(&sched_wake, &sched_wfi, &sched_idle, &sched_total,
                         &sched_busy, &sched_flags);
    http_append_json_metric(out, &len, max, "sched_wake", sched_wake, true);
    http_append_json_metric(out, &len, max, "sched_wfi", sched_wfi, true);
    http_append_json_metric(out, &len, max, "sched_busy_permille", sched_busy, true);
    http_append_json_metric(out, &len, max, "sched_flags", sched_flags, false);
    http_append(out, &len, max, "}}");
    http_append(out, &len, max, "\n");
    http_trace(HTTP_EVT_STATUS_EXIT, HTTP_ROUTE_STATUS, len, max);
    if (!boot_success_marked_by_health) {
        pios_bootctrl_mark_success();
        watchdog_hw_pet();
        boot_success_marked_by_health = true;
    }
    return len;
}

static void http_append_json_ip4(char *out, u32 *len, u32 max, u32 ip)
{
    http_append(out, len, max, "\"");
    if (ip)
        http_append_ip4(out, len, max, ip);
    else
        http_append(out, len, max, "0.0.0.0");
    http_append(out, len, max, "\"");
}

static u32 http_build_netstat_json(char *out, u32 max)
{
    u32 len = 0;
    tcp_snapshot_entry_t snap[TCP_MAX_CONNECTIONS];
    u32 n = tcp_snapshot(snap, TCP_MAX_CONNECTIONS);
    const tcp_diag_t *td = tcp_diag();
    u64 rx_drop = 0, tx_drop = 0;
    nic_filter_stats(&rx_drop, &tx_drop);

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max, "{\"ok\":true,\"count\":");
    http_append_u64(out, &len, max, n);
    http_append(out, &len, max, ",\"diag\":{");
    http_append_json_metric(out, &len, max, "syn", td->syn_seen, true);
    http_append_json_metric(out, &len, max, "synack", td->synack_sent, true);
    http_append_json_metric(out, &len, max, "accepted", td->accepted, true);
    http_append_json_metric(out, &len, max, "noListen", td->no_listener, true);
    http_append_json_metric(out, &len, max, "badCsum", td->bad_checksum, true);
    http_append_json_metric(out, &len, max, "pendQ", td->pending_queued, true);
    http_append_json_metric(out, &len, max, "pendFull", td->pending_full, false);
    http_append(out, &len, max, "},\"fw\":{");
    http_append_json_metric(out, &len, max, "rxDrop", rx_drop, true);
    http_append_json_metric(out, &len, max, "txDrop", tx_drop, false);
    http_append(out, &len, max, "},\"cols\":[\"id\",\"st\",\"lip\",\"lp\",\"rip\",\"rp\",\"own\",\"pend\",\"rx\",\"tx\",\"ret\"],\"rows\":[");
    for (u32 i = 0; i < n; i++) {
        tcp_snapshot_entry_t *e = &snap[i];
        if (i) http_append(out, &len, max, ",");
        http_append(out, &len, max, "[");
        http_append_u64(out, &len, max, (u32)e->conn);
        http_append(out, &len, max, ",");
        http_append_json_string(out, &len, max, tcp_state_name(e->state));
        http_append(out, &len, max, ",");
        http_append_json_ip4(out, &len, max, e->local_ip);
        http_append(out, &len, max, ",");
        http_append_u64(out, &len, max, e->local_port);
        http_append(out, &len, max, ",");
        http_append_json_ip4(out, &len, max, e->remote_ip);
        http_append(out, &len, max, ",");
        http_append_u64(out, &len, max, e->remote_port);
        http_append(out, &len, max, ",");
        http_append_json_string(out, &len, max, tcp_owner_label(e->local_port));
        http_append(out, &len, max, ",");
        http_append_u64(out, &len, max, e->pending_count);
        http_append(out, &len, max, ",");
        http_append_u64(out, &len, max, e->rx_used);
        http_append(out, &len, max, ",");
        http_append_u64(out, &len, max, e->tx_used);
        http_append(out, &len, max, ",");
        http_append_u64(out, &len, max, e->retries);
        http_append(out, &len, max, "]");
    }
    http_append(out, &len, max, "]}\n");
    return len;
}

static bool http_query_cmd(const u8 *req, u32 len, char *out, u32 out_max)
{
    return http_query_value(req, len, "/api/terminal", "cmd", out, out_max) && out[0] != 0;
}

static const char *tcp_state_name(u32 state)
{
    switch (state) {
    case TCP_CLOSED: return "CLOSED";
    case TCP_LISTEN: return "LISTEN";
    case TCP_SYN_SENT: return "SYN-SENT";
    case TCP_SYN_RECEIVED: return "SYN-RECV";
    case TCP_ESTABLISHED: return "ESTAB";
    case TCP_FIN_WAIT_1: return "FIN-W1";
    case TCP_FIN_WAIT_2: return "FIN-W2";
    case TCP_CLOSE_WAIT: return "CLOSE-WAIT";
    case TCP_CLOSING: return "CLOSING";
    case TCP_LAST_ACK: return "LAST-ACK";
    case TCP_TIME_WAIT: return "TIME-WAIT";
    default: return "?";
    }
}

static const char *tcp_owner_label(u16 port)
{
    if (port == ECHO_TCP_PORT) return "kernel/echo";
    if (port == HTTP_TCP_PORT) return "kernel/http";
    if (port == HTTPS_TLS_TCP_PORT) return "kernel/tls443";
    if (port == ADMIN_STATUS_TCP_PORT) return "admin/status";
    if (port == ADMIN_REBOOT_TCP_PORT) return "admin/reboot";
    if (port == ADMIN_UPDATE_TCP_PORT) return "admin/update";
    if (port == DEBUG_TCP_PORT) return "kernel/debug";
    return "-";
}

static bool proc_ui_has_pid(const struct proc_ui_entry *snap, u32 n, u32 pid)
{
    if (!snap || pid == 0)
        return false;
    for (u32 i = 0; i < n; i++) {
        if (snap[i].pid == pid)
            return true;
    }
    return false;
}

static void http_append_pid_field(char *out, u32 *len, u32 max, u32 pid)
{
    if (pid == PROC_UI_KERNEL_PARENT_PID)
        http_append(out, len, max, "-1");
    else
        http_append_u64(out, len, max, pid);
}

static bool http_streq(const char *a, const char *b)
{
    if (!a || !b)
        return false;
    while (*a && *b) {
        if (*a != *b)
            return false;
        a++;
        b++;
    }
    return *a == 0 && *b == 0;
}

static bool http_starts_with(const char *s, const char *prefix)
{
    if (!s || !prefix) return false;
    while (*prefix) {
        if (*s++ != *prefix++) return false;
    }
    return true;
}

static u32 http_split_args(char *line, char **argv, u32 max_args)
{
    u32 argc = 0;
    char *p = line;
    while (*p && argc < max_args) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = 0;
    }
    return argc;
}

static void http_append_sanitized_bytes(char *out, u32 *len, u32 max, const u8 *data, u32 n);
static bool ui_http_fetch(bool use_tls, u32 dst_ip, u16 port,
                          const char *host, const char *path,
                          u32 timeout_ms, u8 *out, u32 out_max,
                          u32 *out_len, const char **err);
static bool ui_http_client_parse_common(u32 argc, char **argv, bool use_tls,
                                        u32 *ip, u16 *port, const char **path,
                                        u32 *timeout_ms);
static void http_append_mem_analyze(char *out, u32 *len, u32 max);
static void http_append_walfs_list_text(char *out, u32 *len, u32 max, const char *path);
static void http_append_bootctrl_status(char *out, u32 *len, u32 max);
static bool pios_bootctrl_clear_pending(void);
static bool pios_bootctrl_reset_a(void);
static bool pios_bootctrl_test_invalid_b(void);
static bool http_write_kernel_slot_header(u32 slot_offset, u32 payload_len, bool valid);
static bool irq_cntpns_test(u64 *before_out, u64 *after_out, u32 *last_intid_out,
                            u32 *d_ctlr_out, u32 *c_ctlr_out, u32 *unhandled_out);
static u32 irq_cntpns_step(u32 depth, u32 *d_ctlr_out, u32 *c_ctlr_out, u32 *pmr_out,
                           u32 *ispend_out, u32 *isenable_out, u32 *iar_out);
static void http_append_proc_image_validation(char *out, u32 *len, u32 max,
                                              const struct proc_image_validation *v);
static void http_append_proc_image_validation_json(char *out, u32 *len, u32 max,
                                                   const struct proc_image_validation *v);

static bool http_parse_db_ref(u32 argc, char **argv, u32 start, u32 *card_out, u32 *rec_out, u32 *next_arg)
{
    if (!argv || !card_out || !rec_out || !next_arg || start >= argc)
        return false;
    u16 c16 = 0;
    u32 rec = 0;
    if (pios_addr_parse_picowal(argv[start], &c16, &rec)) {
        *card_out = c16;
        *rec_out = rec;
        *next_arg = start + 1;
        return true;
    }
    u32 card = 0;
    if (start + 1 >= argc)
        return false;
    if (!http_parse_u32(argv[start], &card) || card > PICOWAL_CARD_MAX)
        return false;
    if (!http_parse_u32(argv[start + 1], &rec) || rec > PICOWAL_RECORD_MAX)
        return false;
    *card_out = card;
    *rec_out = rec;
    *next_arg = start + 2;
    return true;
}

static u32 http_build_terminal_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char cmd[128];
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");

    if (!http_query_cmd(req, req_len, cmd, sizeof(cmd)))
        http_append(out, &len, max, "usage: /api/terminal?cmd=<help|status|netstat|processes|users|ls /|firewall list|addr spec|kill pid|restart pid>\n");
    else if (http_streq(cmd, "help")) {
        http_append(out, &len, max,
            "PIOS terminal help\n"
            "Run commands exactly as shown; category names are help topics, not command prefixes.\n"
            "Examples: status | ps | netstat | ls / | firewall list | addr wal:0/3 | bootctrl status | reboot confirm\n"
            "Command help: help status | help netstat | help firewall | help reboot | help peek\n"
            "Category help on UART/TCP console: help core | help fs | help net | help svc | help dev\n");
    } else if (http_starts_with(cmd, "help ")) {
        const char *topic = cmd + 5;
        if (http_streq(topic, "status")) {
            http_append(out, &len, max, "status\n  Show system/build/network summary.\n");
        } else if (http_streq(topic, "ps") || http_streq(topic, "processes")) {
            http_append(out, &len, max, "processes\n  Show process snapshot with PPID, arena/span telemetry, and process graph roots/children.\n");
        } else if (http_streq(topic, "netstat")) {
            http_append(out, &len, max, "netstat\n  Show live TCP listeners/sessions, owners, buffers, retries, and firewall drops.\n");
        } else if (http_streq(topic, "netcfg")) {
            http_append(out, &len, max, "netcfg | netcfg routes | netcfg neighbors | netcfg trace\n  Show network summary, route table, ARP/neighbor table, and last outbound route/MAC/UDP decision.\n");
        } else if (http_streq(topic, "dns")) {
            http_append(out, &len, max, "dns resolve <hostname> | dns status | dns flush\n  Start/poll an async A-record resolver job without blocking HTTP service loops.\n");
        } else if (http_streq(topic, "http") || http_streq(topic, "https")) {
            http_append(out, &len, max, "http get <ip-or-cached-host> [path] [port] [timeout_ms]\nhttps get <ip-or-cached-host> [path] [port] [timeout_ms]\n  Console-mode HTTP client; hostnames use the DNS cache populated by dns resolve/status.\n");
        } else if (http_streq(topic, "firewall")) {
            http_append(out, &len, max, "firewall list\n  List firewall rules.\nMutations are available from UART/TCP console.\n");
        } else if (http_streq(topic, "reboot")) {
            http_append(out, &len, max, "reboot confirm\n  Reboot via PSCI SYSTEM_RESET from UART/TCP console.\nHTTP reboot: /api/admin/reboot?confirm=1 or :8081/?confirm=1\n");
        } else if (http_streq(topic, "users")) {
            http_append(out, &len, max, "users\n  Show principal/user snapshot.\n");
        } else if (http_streq(topic, "peek")) {
            http_append(out, &len, max, "peek <addr> [1|2|4|8]\n  Read memory from UART/TCP console admin/debug context.\n");
        } else if (http_streq(topic, "poke")) {
            http_append(out, &len, max, "poke <addr> <value> [1|2|4|8]\n  Write live memory from UART/TCP console admin/debug context.\n");
        } else if (http_streq(topic, "dumpmem")) {
            http_append(out, &len, max, "dumpmem <addr> [bytes]\n  Dump memory from UART/TCP console admin/debug context.\n");
        } else if (http_streq(topic, "mem")) {
            http_append(out, &len, max, "mem analyze\n  Show kernel image, raw-slot, per-core RAM, and process memory layout diagnostics.\n");
        } else if (http_streq(topic, "fs") || http_streq(topic, "ls") || http_streq(topic, "fsinspect")) {
            http_append(out, &len, max, "ls [absolute-path] | fsinspect [absolute-path] | walfs status | walfs format confirm\n  WALFS listing/status plus confirmed reserved-base format.\n");
        } else if (http_streq(topic, "bootctrl")) {
            http_append(out, &len, max, "bootctrl status | bootctrl clear-pending | bootctrl reset-a confirm | bootctrl test-invalid-b confirm\n  Show/repair/test stage0 A/B boot-control state without host raw-disk access.\n");
        } else if (http_streq(topic, "dma")) {
            http_append(out, &len, max, "dma status | dma selftest\n  Show DMA channel registers, selftest result, selected CB address mode, and retry selftest.\n");
        } else if (http_streq(topic, "addr")) {
            http_append(out, &len, max, "addr <kind:pack/card[/tail]>\n  Parse and canonicalize PIOS resource addresses. Kinds: wal,tcp,udp,stream,dev,file.\n");
        } else if (http_streq(topic, "keystore")) {
            http_append(out, &len, max, "keystore status | keystore derive <label>\n  Show sealed root status or derive a non-secret fingerprint for a label.\n");
        } else if (http_streq(topic, "tls")) {
            http_append(out, &len, max, "tls status | tls selftest | tls bridge\n  Show kernel TLS diagnostics, run record-layer selftest, or parse a PicoWeb-style plaintext HTTP bridge sample.\n");
        } else if (http_streq(topic, "brotli")) {
            http_append(out, &len, max, "brotli selftest\n  Verify the no-external-dependency Brotli stored encoder and PicoWeb micro-Brotli decoder.\n");
        } else if (http_streq(topic, "x509")) {
            http_append(out, &len, max, "x509 status | x509 generate [cn] | x509 csr [cn] | x509 p256 [cn] | x509 bind | x509 import-self | x509 selftest\n  Manage kernel-only X.509 cert/key, CSR, import, and TLS binding state.\n");
        } else if (http_streq(topic, "acme")) {
            http_append(out, &len, max, "acme status | acme prepare <domain> | acme csrhex | acme challenge <token> <keyauth> | acme clear | acme selftest\n  Prepare ACME HTTP-01 state, export CSR DER hex, and serve /.well-known/acme-challenge/<token>.\n");
        } else if (http_streq(topic, "ksvc")) {
            http_append(out, &len, max, "ksvc status\n  Show kernel service/plugin registry, core ownership, priorities, and runtime counters.\n");
        } else if (http_streq(topic, "irq")) {
            http_append(out, &len, max, "irq status | irq probe | irq selftest | irq cntpns confirm\n  Show IRQ counters, read-only GIC probes, plus opt-in watchdog-protected CNTPNS/PPI30 delivery test.\n");
        } else if (http_streq(topic, "abi")) {
            http_append(out, &len, max, "abi status | abi selftest\n  Show kernel/user ABI transition stage, ksvc foundations, and pending EL0/SVC work.\n");
        } else if (http_streq(topic, "qpu") || http_streq(topic, "tensor")) {
            http_append(out, &len, max, "qpu status | tensor selftest\n  Show V3D/QPU tensor dispatch diagnostics and verify safe NEON fallback kernels.\n");
        } else {
            http_append(out, &len, max, "ERR: unknown help topic. Try help status, help netstat, help firewall, help reboot, help dma, help tls, help brotli\n");
        }
    }
    else if (http_streq(cmd, "status")) {
        nic_packet_counters_t pc;
        nic_packet_counters(&pc);
        http_append(out, &len, max, "PIOS ");
        http_append(out, &len, max, PIOS_BUILD_LABEL);
        http_append(out, &len, max, "\nuptime=");
        http_append_u64(out, &len, max, timer_monotonic_ms() / 1000ULL);
        http_append(out, &len, max, "s\nrx=");
        http_append_u64(out, &len, max, pc.rx_total);
        http_append(out, &len, max, " tx=");
        http_append_u64(out, &len, max, pc.tx_total);
        http_append(out, &len, max, " flood=");
        http_append_u64(out, &len, max, pc.flood_blocked);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "netstat")) {
        const tcp_diag_t *td = tcp_diag();
        const arp_stats_t *ad = arp_get_stats();
        tcp_snapshot_entry_t snap[TCP_MAX_CONNECTIONS];
        u32 n = tcp_snapshot(snap, TCP_MAX_CONNECTIONS);
        u64 rx_drop = 0, tx_drop = 0;
        nic_filter_stats(&rx_drop, &tx_drop);
        http_append(out, &len, max, "CONN ST        LOCAL                  REMOTE                 OWNER          PEND RX TX RETRY\n");
        for (u32 i = 0; i < n; i++) {
            tcp_snapshot_entry_t *e = &snap[i];
            http_append_u64(out, &len, max, (u32)e->conn);
            http_append(out, &len, max, "    ");
            http_append(out, &len, max, tcp_state_name(e->state));
            http_append(out, &len, max, " ");
            http_append_ip4(out, &len, max, e->local_ip);
            http_append(out, &len, max, ":");
            http_append_u64(out, &len, max, e->local_port);
            http_append(out, &len, max, "    ");
            if (e->remote_ip) {
                http_append_ip4(out, &len, max, e->remote_ip);
                http_append(out, &len, max, ":");
                http_append_u64(out, &len, max, e->remote_port);
            } else {
                http_append(out, &len, max, "*:*");
            }
            http_append(out, &len, max, "    ");
            http_append(out, &len, max, tcp_owner_label(e->local_port));
            http_append(out, &len, max, "    ");
            http_append_u64(out, &len, max, e->pending_count);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, e->rx_used);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, e->tx_used);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, e->retries);
            http_append(out, &len, max, "\n");
        }
        http_append(out, &len, max, "syn=");
        http_append_u64(out, &len, max, td->syn_seen);
        http_append(out, &len, max, " synack=");
        http_append_u64(out, &len, max, td->synack_sent);
        http_append(out, &len, max, " accepted=");
        http_append_u64(out, &len, max, td->accepted);
        http_append(out, &len, max, " txseg=");
        http_append_u64(out, &len, max, td->tx_segments);
        http_append(out, &len, max, " tx_nomac=");
        http_append_u64(out, &len, max, td->tx_no_mac);
        http_append(out, &len, max, " tx_fail=");
        http_append_u64(out, &len, max, td->tx_send_fail);
        http_append(out, &len, max, " arp_req=");
        http_append_u64(out, &len, max, ad ? ad->requests_sent : 0);
        http_append(out, &len, max, " arp_learn=");
        http_append_u64(out, &len, max, ad ? ad->learned : 0);
        http_append(out, &len, max, " arp_spoof=");
        http_append_u64(out, &len, max, ad ? ad->drop_spoof : 0);
        http_append(out, &len, max, " fw_rx_drop=");
        http_append_u64(out, &len, max, rx_drop);
        http_append(out, &len, max, " fw_tx_drop=");
        http_append_u64(out, &len, max, tx_drop);
        http_append(out, &len, max, " act_syn=");
        http_append_u64(out, &len, max, td->active_syn_sent);
        http_append(out, &len, max, " act_synack=");
        http_append_u64(out, &len, max, td->active_synack_seen);
        http_append(out, &len, max, " act_est=");
        http_append_u64(out, &len, max, td->active_established);
        http_append(out, &len, max, " act_rst=");
        http_append_u64(out, &len, max, td->active_rst);
        http_append(out, &len, max, " act_badack=");
        http_append_u64(out, &len, max, td->active_bad_ack);
        http_append(out, &len, max, " act_timeout=");
        http_append_u64(out, &len, max, td->active_timeout);
        http_append(out, &len, max, "\nactive_last ");
        http_append_ip4(out, &len, max, td->active_last_local_ip);
        http_append(out, &len, max, ":");
        http_append_u64(out, &len, max, td->active_last_local_port);
        http_append(out, &len, max, " -> ");
        http_append_ip4(out, &len, max, td->active_last_remote_ip);
        http_append(out, &len, max, ":");
        http_append_u64(out, &len, max, td->active_last_remote_port);
        http_append(out, &len, max, " state=");
        http_append(out, &len, max, tcp_state_name(td->active_last_state));
        http_append(out, &len, max, " retries=");
        http_append_u64(out, &len, max, td->active_last_retries);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "netcfg") || http_streq(cmd, "netcfg routes")) {
        const net_stats_t *st = net_get_stats();
        http_append(out, &len, max, "net ip=");
        http_append_ip4(out, &len, max, net_get_our_ip());
        http_append(out, &len, max, " tx=");
        http_append_u64(out, &len, max, st ? st->tx_packets : 0);
        http_append(out, &len, max, " rx=");
        http_append_u64(out, &len, max, st ? st->rx_packets : 0);
        http_append(out, &len, max, " no_neighbor=");
        http_append_u64(out, &len, max, st ? st->drop_no_neighbor : 0);
        http_append(out, &len, max, "\n");
        http_append_route_table(out, &len, max);
        if (http_streq(cmd, "netcfg"))
            http_append_arp_table(out, &len, max);
    } else if (http_streq(cmd, "netcfg neighbors")) {
        http_append_arp_table(out, &len, max);
    } else if (http_streq(cmd, "netcfg trace")) {
        http_append_net_egress_trace(out, &len, max);
    } else if (http_starts_with(cmd, "dns resolve ")) {
        const char *host = cmd + 12;
        if (dns_resolve_async_start(host))
            http_append(out, &len, max, "DNS resolve started\n");
        else
            http_append(out, &len, max, "ERR: dns resolve start failed\n");
    } else if (http_streq(cmd, "dns") || http_streq(cmd, "dns status")) {
        struct dns_async_status ds;
        dns_async_status(&ds);
        http_append(out, &len, max, "dns state=");
        http_append_u64(out, &len, max, ds.state);
        http_append(out, &len, max, " server=");
        http_append_ip4(out, &len, max, ds.server_ip);
        http_append(out, &len, max, " result=");
        http_append_ip4(out, &len, max, ds.result_ip);
        http_append(out, &len, max, " attempts=");
        http_append_u64(out, &len, max, ds.attempts);
        http_append(out, &len, max, " error=");
        http_append_u64(out, &len, max, ds.last_error);
        http_append(out, &len, max, " rx=");
        http_append_u64(out, &len, max, ds.rx_total);
        http_append(out, &len, max, " server_rx=");
        http_append_u64(out, &len, max, ds.rx_server);
        http_append(out, &len, max, " ignored=");
        http_append_u64(out, &len, max, ds.rx_ignored);
        http_append(out, &len, max, " rejected=");
        http_append_u64(out, &len, max, ds.rx_rejected);
        http_append(out, &len, max, " ok_rx=");
        http_append_u64(out, &len, max, ds.rx_ok);
        http_append(out, &len, max, "\nlast_rx=");
        http_append_ip4(out, &len, max, ds.last_rx_src_ip);
        http_append(out, &len, max, ":");
        http_append_u64(out, &len, max, ds.last_rx_src_port);
        http_append(out, &len, max, " -> ");
        http_append_u64(out, &len, max, ds.last_rx_dst_port);
        http_append(out, &len, max, " len=");
        http_append_u64(out, &len, max, ds.last_rx_len);
        http_append(out, &len, max, "\nhostname=");
        http_append(out, &len, max, ds.hostname);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "dns flush")) {
        dns_cache_flush();
        http_append(out, &len, max, "DNS cache flushed\n");
    } else if (http_starts_with(cmd, "http get ") || http_starts_with(cmd, "https get ")) {
        bool use_tls = http_starts_with(cmd, "https ");
        char line[160];
        char *argv[8];
        u32 p = 0;
        while (cmd[p] && p + 1 < sizeof(line)) { line[p] = cmd[p]; p++; }
        line[p] = 0;
        u32 argc = http_split_args(line, argv, 8);
        u32 ip = 0;
        u16 port = 0;
        u32 timeout_ms = 0;
        const char *path = NULL;
        u8 body[2048];
        u32 body_len = 0;
        const char *err = NULL;
        if (!ui_http_client_parse_common(argc, argv, use_tls, &ip, &port, &path, &timeout_ms)) {
            http_append(out, &len, max, use_tls ?
                "ERR: usage https get <ip-or-cached-host> [path] [port] [timeout_ms]\n" :
                "ERR: usage http get <ip-or-cached-host> [path] [port] [timeout_ms]\n");
        } else if (!ui_http_fetch(use_tls, ip, port, argv[2], path, timeout_ms,
                                  body, sizeof(body), &body_len, &err)) {
            http_append(out, &len, max, "ERR: ");
            http_append(out, &len, max, err ? err : "request failed");
            http_append(out, &len, max, "\n");
        } else {
            http_append_sanitized_bytes(out, &len, max, body, body_len);
        }
    } else if (http_streq(cmd, "dma") || http_streq(cmd, "dma status")) {
        struct dma_diag_snapshot d;
        dma_diag_snapshot(&d);
        http_append(out, &len, max, "DMA enabled=");
        http_append(out, &len, max, d.hw_memcpy_enabled ? "yes" : "no");
        http_append(out, &len, max, " mode=");
        http_append(out, &len, max, d.direct_mode ? "direct" : "cb");
        http_append(out, &len, max, " cbaddr=");
        http_append(out, &len, max, d.cbaddr_shifted ? "shifted" : "raw");
        http_append(out, &len, max, " selftests=");
        http_append_u64(out, &len, max, d.selftest_runs);
        http_append(out, &len, max, " failures=");
        http_append_u64(out, &len, max, d.selftest_failures);
        http_append(out, &len, max, " last_error=");
        http_append_u64(out, &len, max, d.last_error);
        http_append(out, &len, max, " last_ch=");
        http_append_u64(out, &len, max, d.last_channel);
        http_append(out, &len, max, " len=");
        http_append_u64(out, &len, max, d.last_len);
        http_append(out, &len, max, " enable=");
        http_append_u64(out, &len, max, d.enable_reg);
        http_append(out, &len, max, "\nCH CS CBADDR TI SRC DST LEN DEBUG\n");
        for (u32 ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
            http_append_u64(out, &len, max, ch);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].cs);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].cbaddr);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].ti);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].src);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].dst);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].len);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, d.channel[ch].debug);
            http_append(out, &len, max, "\n");
        }
        if (d.last_error == 4U) {
            http_append(out, &len, max, "mismatch off=");
            http_append_u64(out, &len, max, d.last_mismatch_off);
            http_append(out, &len, max, " got=");
            http_append_u64(out, &len, max, d.last_got);
            http_append(out, &len, max, " expected=");
            http_append_u64(out, &len, max, d.last_expected);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "dma selftest")) {
        bool ok = dma_selftest();
        http_append(out, &len, max, ok ? "DMA selftest OK\n" : "DMA selftest FAILED\n");
    } else if (http_streq(cmd, "tls") || http_streq(cmd, "tls status")) {
        struct tls_diag_snapshot t;
        tls_diag_snapshot(&t);
        http_append(out, &len, max, "TLS kernel=enabled crypto=arm-aese+ghash-nibble bridge=picoweb-style active=");
        http_append_u64(out, &len, max, t.active);
        http_append(out, &len, max, " established=");
        http_append_u64(out, &len, max, t.established);
        http_append(out, &len, max, " hs_ok=");
        http_append_u64(out, &len, max, t.handshakes_ok);
        http_append(out, &len, max, " hs_fail=");
        http_append_u64(out, &len, max, t.handshake_failures);
        http_append(out, &len, max, " tx=");
        http_append_u64(out, &len, max, t.records_tx);
        http_append(out, &len, max, " rx=");
        http_append_u64(out, &len, max, t.records_rx);
        http_append(out, &len, max, " decrypt_fail=");
        http_append_u64(out, &len, max, t.decrypt_failures);
        http_append(out, &len, max, " bridge_ok=");
        http_append_u64(out, &len, max, t.bridge_parse_ok);
        http_append(out, &len, max, " bridge_more=");
        http_append_u64(out, &len, max, t.bridge_parse_need_more);
        http_append(out, &len, max, " bridge_err=");
        http_append_u64(out, &len, max, t.bridge_parse_error);
        http_append(out, &len, max, " selftests=");
        http_append_u64(out, &len, max, t.selftests);
        http_append(out, &len, max, " failures=");
        http_append_u64(out, &len, max, t.selftest_failures);
        http_append(out, &len, max, " last_error=");
        http_append_u64(out, &len, max, t.last_error);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tls selftest")) {
        http_append(out, &len, max, tls_selftest() ? "TLS selftest OK\n" : "TLS selftest FAILED\n");
    } else if (http_streq(cmd, "tls bridge")) {
        static const u8 sample[] = "GET / HTTP/1.1\r\nHost: pios\r\nConnection: close\r\n\r\n";
        struct tls_bridge_request br;
        i32 r = tls_bridge_parse_request(sample, (u32)(sizeof(sample) - 1), &br);
        http_append(out, &len, max, "TLS bridge result=");
        http_append_u64(out, &len, max, (u32)r);
        http_append(out, &len, max, " method=");
        http_append(out, &len, max, br.method);
        http_append(out, &len, max, " path=");
        http_append(out, &len, max, br.path);
        http_append(out, &len, max, " header_bytes=");
        http_append_u64(out, &len, max, br.header_bytes);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "brotli") || http_streq(cmd, "brotli selftest")) {
        http_append(out, &len, max, brotli_selftest() ? "Brotli selftest OK\n" : "Brotli selftest FAILED\n");
    } else if (http_streq(cmd, "x509") || http_streq(cmd, "x509 status")) {
        struct x509_status xs;
        x509_status(&xs);
        http_append(out, &len, max, "x509 initialized=");
        http_append(out, &len, max, xs.initialized ? "yes" : "no");
        http_append(out, &len, max, " key=");
        http_append(out, &len, max, xs.has_key ? "yes" : "no");
        http_append(out, &len, max, " p256_key=");
        http_append(out, &len, max, xs.has_p256_key ? "yes" : "no");
        http_append(out, &len, max, " cert=");
        http_append(out, &len, max, xs.has_cert ? "yes" : "no");
        http_append(out, &len, max, " tls_bound=");
        http_append(out, &len, max, xs.tls_bound ? "yes" : "no");
        http_append(out, &len, max, " der_ready=");
        http_append(out, &len, max, xs.der_ready ? "yes" : "no");
        http_append(out, &len, max, " der_len=");
        http_append_u64(out, &len, max, xs.der_len);
        http_append(out, &len, max, " csr_ready=");
        http_append(out, &len, max, xs.csr_ready ? "yes" : "no");
        http_append(out, &len, max, " csr_len=");
        http_append_u64(out, &len, max, xs.csr_len);
        http_append(out, &len, max, " csr_alg=");
        http_append(out, &len, max, xs.csr_alg == X509_CSR_ALG_P256 ? "p256" :
                    (xs.csr_alg == X509_CSR_ALG_ED25519 ? "ed25519" : "none"));
        http_append(out, &len, max, " generation=");
        http_append_u64(out, &len, max, xs.generation);
        http_append(out, &len, max, " key_fp=");
        http_append_u64(out, &len, max, xs.key_fingerprint);
        http_append(out, &len, max, " p256_fp=");
        http_append_u64(out, &len, max, xs.p256_key_fingerprint);
        http_append(out, &len, max, " cert_fp=");
        http_append_u64(out, &len, max, xs.cert_fingerprint);
        http_append(out, &len, max, " error=");
        http_append_u64(out, &len, max, xs.last_error);
        http_append(out, &len, max, "\nsubject=");
        http_append(out, &len, max, xs.subject);
        http_append(out, &len, max, "\nissuer=");
        http_append(out, &len, max, xs.issuer);
        http_append(out, &len, max, "\n");
    } else if (http_starts_with(cmd, "x509 generate")) {
        const char *cn = "";
        if (cmd[13] == ' ') cn = cmd + 14;
        http_append(out, &len, max, x509_generate_dev_cert(cn) ? "X509 generate OK\n" : "X509 generate FAILED\n");
    } else if (http_starts_with(cmd, "x509 csr")) {
        const char *cn = "";
        if (cmd[8] == ' ') cn = cmd + 9;
        http_append(out, &len, max, x509_generate_csr(cn) ? "X509 CSR OK\n" : "X509 CSR FAILED\n");
    } else if (http_starts_with(cmd, "x509 p256")) {
        const char *cn = "";
        if (cmd[9] == ' ') cn = cmd + 10;
        http_append(out, &len, max, x509_generate_p256_csr(cn) ? "X509 P256 CSR OK\n" : "X509 P256 CSR FAILED\n");
    } else if (http_streq(cmd, "x509 import-self")) {
        u32 cert_len = 0;
        const u8 *cert = x509_certificate_der(&cert_len);
        http_append(out, &len, max, x509_import_certificate_der(cert, cert_len) ? "X509 import-self OK\n" : "X509 import-self FAILED\n");
    } else if (http_streq(cmd, "x509 bind")) {
        http_append(out, &len, max, x509_bind_tls() ? "X509 bind OK\n" : "X509 bind FAILED\n");
    } else if (http_streq(cmd, "x509 selftest")) {
        http_append(out, &len, max, x509_selftest() ? "X509 selftest OK\n" : "X509 selftest FAILED\n");
    } else if (http_streq(cmd, "acme") || http_streq(cmd, "acme status")) {
        struct acme_status as;
        acme_status(&as);
        http_append(out, &len, max, "acme initialized=");
        http_append(out, &len, max, as.initialized ? "yes" : "no");
        http_append(out, &len, max, " account_key=");
        http_append(out, &len, max, as.account_key ? "yes" : "no");
        http_append(out, &len, max, " csr_ready=");
        http_append(out, &len, max, as.csr_ready ? "yes" : "no");
        http_append(out, &len, max, " challenge_ready=");
        http_append(out, &len, max, as.challenge_ready ? "yes" : "no");
        http_append(out, &len, max, " state=");
        http_append_u64(out, &len, max, as.state);
        http_append(out, &len, max, " account_fp=");
        http_append_u64(out, &len, max, as.account_fingerprint);
        http_append(out, &len, max, " csr_len=");
        http_append_u64(out, &len, max, as.csr_len);
        http_append(out, &len, max, " error=");
        http_append_u64(out, &len, max, as.last_error);
        http_append(out, &len, max, "\ndirectory=");
        http_append(out, &len, max, as.directory);
        http_append(out, &len, max, "\ndomain=");
        http_append(out, &len, max, as.domain);
        http_append(out, &len, max, "\ntoken=");
        http_append(out, &len, max, as.token);
        http_append(out, &len, max, "\n");
    } else if (http_starts_with(cmd, "acme prepare ")) {
        http_append(out, &len, max, acme_prepare_http01(cmd + 13) ? "ACME prepare OK\n" : "ACME prepare FAILED\n");
    } else if (http_streq(cmd, "acme csrhex")) {
        u32 csr_len = 0;
        const u8 *csr = x509_csr_der(&csr_len);
        if (!csr || csr_len == 0) {
            http_append(out, &len, max, "ERR: no CSR ready; run acme prepare <domain>\n");
        } else {
            http_append(out, &len, max, "csr_der_hex len=");
            http_append_u64(out, &len, max, csr_len);
            http_append(out, &len, max, "\n");
            for (u32 i = 0; i < csr_len; i++) {
                http_append_hex8(out, &len, max, csr[i]);
                if ((i & 31U) == 31U) http_append(out, &len, max, "\n");
            }
            http_append(out, &len, max, "\n");
        }
    } else if (http_starts_with(cmd, "acme challenge ")) {
        char token[ACME_TOKEN_MAX];
        const char *p = cmd + 15;
        u32 ti = 0;
        while (*p && *p != ' ' && ti + 1 < sizeof(token)) token[ti++] = *p++;
        token[ti] = 0;
        if (*p == ' ') p++;
        http_append(out, &len, max, acme_set_http01_challenge(token, p) ? "ACME challenge OK\n" : "ACME challenge FAILED\n");
    } else if (http_streq(cmd, "acme clear")) {
        acme_clear_http01_challenge();
        http_append(out, &len, max, "ACME clear OK\n");
    } else if (http_streq(cmd, "acme selftest")) {
        http_append(out, &len, max, acme_selftest() ? "ACME selftest OK\n" : "ACME selftest FAILED\n");
    } else if (http_streq(cmd, "peek") || http_starts_with(cmd, "peek ")) {
        char *argv[4];
        u32 argc = http_split_args(cmd, argv, 4);
        u64 addr = 0;
        u32 width = 4;
        if (argc < 2 || !http_parse_u64(argv[1], &addr) ||
            (argc >= 3 && !http_mem_width(argv[2], &width))) {
            http_append(out, &len, max, "ERR: usage peek <addr> [1|2|4|8]\n");
        } else {
            u64 v = http_mem_read(addr, width);
            http_append(out, &len, max, "0x");
            http_append_hex64(out, &len, max, addr);
            http_append(out, &len, max, " = 0x");
            if (width == 8) http_append_hex64(out, &len, max, v);
            else if (width == 4) http_append_hex32(out, &len, max, (u32)v);
            else if (width == 2) {
                http_append_hex8(out, &len, max, (u8)(v >> 8));
                http_append_hex8(out, &len, max, (u8)v);
            } else {
                http_append_hex8(out, &len, max, (u8)v);
            }
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "poke") || http_starts_with(cmd, "poke ")) {
        char *argv[5];
        u32 argc = http_split_args(cmd, argv, 5);
        u64 addr = 0, value = 0;
        u32 width = 4;
        if (argc < 3 || !http_parse_u64(argv[1], &addr) ||
            !http_parse_u64(argv[2], &value) ||
            (argc >= 4 && !http_mem_width(argv[3], &width))) {
            http_append(out, &len, max, "ERR: usage poke <addr> <value> [1|2|4|8]\n");
        } else {
            http_mem_write(addr, value, width);
            http_append(out, &len, max, "OK: 0x");
            http_append_hex64(out, &len, max, addr);
            http_append(out, &len, max, " <= 0x");
            if (width == 8) http_append_hex64(out, &len, max, value);
            else if (width == 4) http_append_hex32(out, &len, max, (u32)value);
            else if (width == 2) {
                http_append_hex8(out, &len, max, (u8)(value >> 8));
                http_append_hex8(out, &len, max, (u8)value);
            } else {
                http_append_hex8(out, &len, max, (u8)value);
            }
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "sched") || http_streq(cmd, "sched status")) {
        u64 wake = 0, wfi_count = 0, idle_ticks = 0, total_ticks = 0;
        u32 busy_permille = 0, last_flags = 0;
        core0_sched_snapshot(&wake, &wfi_count, &idle_ticks, &total_ticks,
                             &busy_permille, &last_flags);
        http_append(out, &len, max, "sched core0 wake=");
        http_append_u64(out, &len, max, wake);
        http_append(out, &len, max, " wfi=");
        http_append_u64(out, &len, max, wfi_count);
        http_append(out, &len, max, " busy_permille=");
        http_append_u64(out, &len, max, busy_permille);
        http_append(out, &len, max, " idle_ticks=");
        http_append_u64(out, &len, max, idle_ticks);
        http_append(out, &len, max, " total_ticks=");
        http_append_u64(out, &len, max, total_ticks);
        http_append(out, &len, max, " last_flags=");
        http_append_hex32(out, &len, max, last_flags);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "proc sched")) {
        struct proc_sched_core_snapshot ps[3];
        u32 pn = proc_sched_snapshot(ps, 3);
        http_append(out, &len, max, "CORE BUSY_PERMILLE IDLE WAKE IDLE_T TOTAL_T PREEMPT SOFT_EVT SOFT_BOOST\n");
        for (u32 i = 0; i < pn; i++) {
            http_append_u64(out, &len, max, ps[i].core);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].busy_permille);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].idle_count);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].wake_count);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].idle_ticks);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].total_ticks);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].preemptions);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].soft_events);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ps[i].soft_boosts);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "core status")) {
        struct core_status_entry cs[NUM_CORES];
        u32 cn = core_status_snapshot(cs, NUM_CORES);
        http_append(out, &len, max, "CORE PSCI_RET STAGE\n");
        for (u32 i = 0; i < cn; i++) {
            http_append_u64(out, &len, max, cs[i].core);
            http_append(out, &len, max, " ");
            if (cs[i].psci_ret < 0) {
                http_append(out, &len, max, "-");
                http_append_u64(out, &len, max, (u64)(-cs[i].psci_ret));
            } else {
                http_append_u64(out, &len, max, (u64)cs[i].psci_ret);
            }
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, cs[i].stage);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "rp1 irq") || http_streq(cmd, "rp1 irq clear") ||
               http_streq(cmd, "rp1 irq arm-eth") || http_streq(cmd, "rp1 irq raise-eth") ||
               http_streq(cmd, "rp1 irq pend-gic") || http_streq(cmd, "rp1 irq arm-host6") ||
               http_streq(cmd, "rp1 irq arm-host6-1") || http_streq(cmd, "rp1 irq source-diag")) {
        if (http_streq(cmd, "rp1 irq clear")) {
            core0_eth_irq_drain_and_quench(false);
            http_append(out, &len, max, "rp1 eth irq quench\n");
        } else if (http_streq(cmd, "rp1 irq source-diag")) {
            u32 old_ncr = mmio_read(MACB_BASE + 0x00);
            u32 old_imr = mmio_read(MACB_BASE + 0x30);
            for (u32 step = 0; step < 4; step++) {
                if (step == 1) {
                    mmio_write(MACB_BASE + 0x2C, 0xFFFFFFFFU);
                    mmio_write(MACB_BASE + 0x20, 0xFFFFFFFFU);
                    mmio_write(MACB_BASE + 0x24, 0xFFFFFFFFU);
                } else if (step == 2) {
                    mmio_write(MACB_BASE + 0x00, 0);
                } else if (step == 3) {
                    mmio_write(MACB_BASE + 0x00, old_ncr);
                    if ((old_imr & 2U) == 0)
                        mmio_write(MACB_BASE + 0x28, 2U);
                }
                dsb();
                core0_eth_source_diag[(step * 6U) + 0] = rp1_mip_host_status_l();
                core0_eth_source_diag[(step * 6U) + 1] = rp1_irq_status_l();
                core0_eth_source_diag[(step * 6U) + 2] = mmio_read(MACB_BASE + 0x20);
                core0_eth_source_diag[(step * 6U) + 3] = mmio_read(MACB_BASE + 0x24);
                core0_eth_source_diag[(step * 6U) + 4] = mmio_read(MACB_BASE + 0x00);
                core0_eth_source_diag[(step * 6U) + 5] = mmio_read(MACB_BASE + 0x30);
            }
            core0_eth_source_diag_seq++;
            http_append(out, &len, max, "source-diag stored\n");
        } else if (http_streq(cmd, "rp1 irq arm-eth")) {
            __asm__ volatile("msr daifset, #2" ::: "memory");
            irq_register(GIC_RP1_ETH_MSI, core0_eth_irq_handler);
            gic_set_group1(GIC_RP1_ETH_MSI);
            gic_set_priority(GIC_RP1_ETH_MSI, 0x40);
            gic_set_target(GIC_RP1_ETH_MSI, 1);
            gic_set_edge_triggered(GIC_RP1_ETH_MSI);
            gic_clear_pending(GIC_RP1_ETH_MSI);
            core0_eth_irq_oneshot = false;
            macb_irq_ack_rx();
            rp1_eth_irq_arm();
            macb_irq_enable_rx();
            dsb();
            isb();
            gic_enable_irq(GIC_RP1_ETH_MSI);
            __asm__ volatile("msr daifclr, #2" ::: "memory");
            http_append(out, &len, max, "rp1 eth irq armed\n");
        } else if (http_streq(cmd, "rp1 irq raise-eth")) {
            rp1_eth_irq_raise_test();
            http_append(out, &len, max, "rp1 eth mip raise sent\n");
        } else if (http_streq(cmd, "rp1 irq pend-gic")) {
            u64 before = core0_eth_irq_count;
            __asm__ volatile("msr daifset, #2" ::: "memory");
            irq_register(GIC_RP1_ETH_MSI, core0_eth_irq_handler);
            gic_set_group1(GIC_RP1_ETH_MSI);
            gic_set_priority(GIC_RP1_ETH_MSI, 0x40);
            gic_set_target(GIC_RP1_ETH_MSI, 1);
            gic_set_edge_triggered(GIC_RP1_ETH_MSI);
            gic_clear_pending(GIC_RP1_ETH_MSI);
            gic_enable_irq(GIC_RP1_ETH_MSI);
            dsb();
            isb();
            mmio_write(gic_runtime_gicd_base() + 0x200U + ((GIC_RP1_ETH_MSI / 32U) * 4U),
                       1U << (GIC_RP1_ETH_MSI % 32U));
            dsb();
            isb();
            __asm__ volatile("msr daifclr, #2" ::: "memory");
            for (u32 spin = 0; spin < 100000U && core0_eth_irq_count == before; spin++)
                __asm__ volatile("yield");
            http_append(out, &len, max, "rp1 eth gic pend before=");
            http_append_u64(out, &len, max, before);
            http_append(out, &len, max, " after=");
            http_append_u64(out, &len, max, core0_eth_irq_count);
            http_append(out, &len, max, "\n");
        } else if (http_streq(cmd, "rp1 irq arm-host6") || http_streq(cmd, "rp1 irq arm-host6-1")) {
            __asm__ volatile("msr daifset, #2" ::: "memory");
            core0_eth_irq_oneshot = http_streq(cmd, "rp1 irq arm-host6-1");
            irq_register(GIC_RP1_ETH_MSI, core0_eth_irq_handler);
            gic_set_group1(GIC_RP1_ETH_MSI);
            gic_set_priority(GIC_RP1_ETH_MSI, 0x40);
            gic_set_target(GIC_RP1_ETH_MSI, 1);
            gic_set_edge_triggered(GIC_RP1_ETH_MSI);
            gic_clear_pending(GIC_RP1_ETH_MSI);
            macb_irq_ack_rx();
            rp1_eth_host_arm();
            macb_irq_enable_rx();
            dsb();
            isb();
            gic_enable_irq(GIC_RP1_ETH_MSI);
            __asm__ volatile("msr daifclr, #2" ::: "memory");
            http_append(out, &len, max, core0_eth_irq_oneshot ?
                        "rp1 eth HOST6 one-shot armed\n" : "rp1 eth HOST6 armed\n");
        }
        struct rp1_irq_snapshot r;
        struct macb_irq_snapshot m;
        rp1_irq_snapshot(&r);
        macb_irq_snapshot(&m, false);
        http_append(out, &len, max, "rp1 irq eth=6 intstat_l=");
        http_append_hex32(out, &len, max, r.intstat_l);
        http_append(out, &len, max, " intstat_h=");
        http_append_hex32(out, &len, max, r.intstat_h);
        http_append(out, &len, max, " mip_status_l=");
        http_append_hex32(out, &len, max, r.mip_status_l);
        http_append(out, &len, max, " mip_mask_l=");
        http_append_hex32(out, &len, max, r.mip_mask_l);
        http_append(out, &len, max, " vpu_status_l=");
        http_append_hex32(out, &len, max, r.mip_vpu_status_l);
        http_append(out, &len, max, " vpu_mask_l=");
        http_append_hex32(out, &len, max, r.mip_vpu_mask_l);
        http_append(out, &len, max, " cfgl=");
        http_append_hex32(out, &len, max, r.mip_cfgl_host);
        http_append(out, &len, max, " eth_msix_cfg=");
        http_append_hex32(out, &len, max, r.eth_msix_cfg);
        http_append(out, &len, max, " count=");
        http_append_u64(out, &len, max, core0_eth_irq_count);
        http_append(out, &len, max, " last_mip=");
        http_append_hex32(out, &len, max, core0_eth_irq_last_mip);
        http_append(out, &len, max, " last_isr=");
        http_append_hex32(out, &len, max, core0_eth_irq_last_macb_isr);
        http_append(out, &len, max, " quench_passes=");
        http_append_u64(out, &len, max, core0_eth_irq_quench_passes);
        http_append(out, &len, max, " srcseq=");
        http_append_u64(out, &len, max, core0_eth_source_diag_seq);
        http_append(out, &len, max, "\neth_vec addr=");
        http_append_hex32(out, &len, max, r.eth_msix_addr_hi);
        http_append(out, &len, max, ":");
        http_append_hex32(out, &len, max, r.eth_msix_addr_lo);
        http_append(out, &len, max, " data=");
        http_append_hex32(out, &len, max, r.eth_msix_data);
        http_append(out, &len, max, " vctrl=");
        http_append_hex32(out, &len, max, r.eth_msix_vector_ctrl);
        http_append(out, &len, max, " pba=");
        http_append_hex32(out, &len, max, r.eth_msix_pba);
        http_append(out, &len, max, "\nmacb imr=");
        http_append_hex32(out, &len, max, m.imr);
        http_append(out, &len, max, " rsr=");
        http_append_hex32(out, &len, max, m.rsr);
        http_append(out, &len, max, " tsr=");
        http_append_hex32(out, &len, max, m.tsr);
        http_append(out, &len, max, " ncr=");
        http_append_hex32(out, &len, max, m.ncr);
        http_append(out, &len, max, " isr_clear=");
        http_append_hex32(out, &len, max, m.isr_clear);
        http_append(out, &len, max, "\n");
        if (core0_eth_source_diag_seq) {
            http_append(out, &len, max, "source h/i/rsr/isr/ncr/imr:");
            for (u32 i = 0; i < 24U; i++) {
                if ((i % 6U) == 0)
                    http_append(out, &len, max, "\n");
                else
                    http_append(out, &len, max, " ");
                http_append_hex32(out, &len, max, core0_eth_source_diag[i]);
            }
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "rp1 pci")) {
        u32 id = pcie_cfg_read(1, 0, 0, 0x00);
        u32 cmdstat = pcie_cfg_read(1, 0, 0, 0x04);
        u32 bar0 = pcie_cfg_read(1, 0, 0, 0x10);
        u32 bar1 = pcie_cfg_read(1, 0, 0, 0x14);
        u32 cap = pcie_cfg_read(1, 0, 0, 0x34) & 0xFFU;
        http_append(out, &len, max, "rp1 pci id=");
        http_append_hex32(out, &len, max, id);
        http_append(out, &len, max, " cmdstat=");
        http_append_hex32(out, &len, max, cmdstat);
        http_append(out, &len, max, " bar0=");
        http_append_hex32(out, &len, max, bar0);
        http_append(out, &len, max, " bar1=");
        http_append_hex32(out, &len, max, bar1);
        http_append(out, &len, max, " cap=");
        http_append_hex32(out, &len, max, cap);
        http_append(out, &len, max, "\nCAP OFF ID NEXT RAW0 RAW4 RAW8\n");
        for (u32 i = 0; i < 16 && cap >= 0x40 && cap < 0x100; i++) {
            u32 raw0 = pcie_cfg_read(1, 0, 0, cap);
            u32 raw4 = pcie_cfg_read(1, 0, 0, cap + 4);
            u32 raw8 = pcie_cfg_read(1, 0, 0, cap + 8);
            u32 cid = raw0 & 0xFFU;
            u32 next = (raw0 >> 8) & 0xFFU;
            http_append_hex32(out, &len, max, cap);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, cid);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, next);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, raw0);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, raw4);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, raw8);
            if (cid == 0x11U) {
                u32 msgctl = (raw0 >> 16) & 0xFFFFU;
                u32 bir = raw4 & 7U;
                u32 table = raw4 & ~7U;
                u32 pba_bir = raw8 & 7U;
                u32 pba = raw8 & ~7U;
                http_append(out, &len, max, " msix_ctl=");
                http_append_hex32(out, &len, max, msgctl);
                http_append(out, &len, max, " table_bir=");
                http_append_u64(out, &len, max, bir);
                http_append(out, &len, max, " table_off=");
                http_append_hex32(out, &len, max, table);
                http_append(out, &len, max, " pba_bir=");
                http_append_u64(out, &len, max, pba_bir);
                http_append(out, &len, max, " pba_off=");
                http_append_hex32(out, &len, max, pba);
            }
            http_append(out, &len, max, "\n");
            if (next == 0 || next == cap)
                break;
            cap = next;
        }
    } else if (http_streq(cmd, "pcie aer") || http_streq(cmd, "pcie aer clear")) {
        struct pcie_aer_snapshot a;
        bool clear = http_streq(cmd, "pcie aer clear");
        pcie_aer_snapshot(&a, clear);
        http_append(out, &len, max, "pcie aer off=");
        http_append_hex32(out, &len, max, a.aer_offset);
        http_append(out, &len, max, " uncorr=");
        http_append_hex32(out, &len, max, a.uncorr);
        http_append(out, &len, max, " corr=");
        http_append_hex32(out, &len, max, a.corr);
        http_append(out, &len, max, " hdr=");
        http_append_hex32(out, &len, max, a.hdr0);
        http_append(out, &len, max, " ");
        http_append_hex32(out, &len, max, a.hdr1);
        http_append(out, &len, max, " ");
        http_append_hex32(out, &len, max, a.hdr2);
        http_append(out, &len, max, " ");
        http_append_hex32(out, &len, max, a.hdr3);
        http_append(out, &len, max, clear ? " cleared\n" : "\n");
    } else if (http_streq(cmd, "ksvc") || http_streq(cmd, "ksvc status")) {
        struct ksvc_snapshot_entry ks[KSVC_MAX_SERVICES];
        u32 kn = ksvc_snapshot(ks, KSVC_MAX_SERVICES);
        http_append(out, &len, max, "ID CORE PRI STATE KIND CALLS ERR RST LERR PEND SENT RECV DROP LAST_T MAX_T TOTAL_T NAME\n");
        for (u32 i = 0; i < kn; i++) {
            http_append_u64(out, &len, max, ks[i].id);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].owner_core);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].priority);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, ksvc_state_name(ks[i].state));
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].kind);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].calls);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].errors);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].restarts);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].last_error);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].mailbox_pending);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].messages_sent);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].messages_recv);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].mailbox_drops);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].last_duration_ticks);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].max_duration_ticks);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ks[i].total_ticks);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, ks[i].name);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "ksvc selftest")) {
        bool ok = ksvc_mailbox_selftest(ksvc_debug_id) && ksvc_fault_policy_selftest(ksvc_debug_id);
        http_append(out, &len, max, ok ? "KSVC selftest OK\n" : "KSVC selftest FAILED\n");
    } else if (http_starts_with(cmd, "ksvc pause ")) {
        u32 id = 0;
        if (!http_parse_u32(cmd + 11, &id) || !ksvc_pause((i32)id))
            http_append(out, &len, max, "KSVC pause FAILED\n");
        else
            http_append(out, &len, max, "KSVC pause OK\n");
    } else if (http_starts_with(cmd, "ksvc resume ")) {
        u32 id = 0;
        if (!http_parse_u32(cmd + 12, &id) || !ksvc_resume((i32)id))
            http_append(out, &len, max, "KSVC resume FAILED\n");
        else
            http_append(out, &len, max, "KSVC resume OK\n");
    } else if (http_starts_with(cmd, "ksvc restart ")) {
        u32 id = 0;
        if (!http_parse_u32(cmd + 13, &id) || !ksvc_restart((i32)id))
            http_append(out, &len, max, "KSVC restart FAILED\n");
        else
            http_append(out, &len, max, "KSVC restart OK\n");
    } else if (http_starts_with(cmd, "ksvc fault ")) {
        u32 id = 0;
        if (!http_parse_u32(cmd + 11, &id) || !ksvc_mark_error_code((i32)id, 0x4B534643U))
            http_append(out, &len, max, "KSVC fault FAILED\n");
        else
            http_append(out, &len, max, "KSVC fault OK\n");
    } else if (http_streq(cmd, "irq") || http_streq(cmd, "irq status")) {
        struct irq_diag_snapshot d;
        struct irq_hw_diag_snapshot hw;
        irq_diag_snapshot(&d);
        irq_hw_diag_snapshot(&hw);
        http_append(out, &len, max, "irq total=");
        http_append_u64(out, &len, max, d.total);
        http_append(out, &len, max, " handled=");
        http_append_u64(out, &len, max, d.handled);
        http_append(out, &len, max, " unhandled=");
        http_append_u64(out, &len, max, d.unhandled);
        http_append(out, &len, max, " spurious=");
        http_append_u64(out, &len, max, d.spurious);
        http_append(out, &len, max, " timer=");
        http_append_u64(out, &len, max, d.timer);
        http_append(out, &len, max, " last_intid=");
        http_append_u64(out, &len, max, d.last_intid);
        http_append(out, &len, max, " last_core=");
        http_append_u64(out, &len, max, d.last_core);
        http_append(out, &len, max, " last_tick=");
        http_append_u64(out, &len, max, d.last_tick);
        http_append(out, &len, max, "\nper_core=");
        for (u32 i = 0; i < 4; i++) {
            if (i) http_append(out, &len, max, ",");
            http_append_u64(out, &len, max, d.per_core[i]);
        }
        http_append(out, &len, max, " current_el=");
        http_append_u64(out, &len, max, (hw.current_el >> 2) & 3U);
        http_append(out, &len, max, " daif=");
        http_append_u64(out, &len, max, hw.daif);
        http_append(out, &len, max, " vectors=");
        http_append(out, &len, max, hw.vectors_ready ? "ready" : "bad");
        http_append(out, &len, max, " gic=");
        http_append(out, &len, max, hw.gic_ready ? "ready" : "bad");
        http_append(out, &len, max, " timer=");
        http_append(out, &len, max, hw.timer_enabled ? "enabled" : "disabled");
        http_append(out, &len, max, " irq_masked=");
        http_append(out, &len, max, hw.irq_masked ? "yes" : "no");
        http_append(out, &len, max, " gicd_ctlr=");
        http_append_u64(out, &len, max, hw.gicd_ctlr);
        http_append(out, &len, max, " gicc_ctlr=");
        http_append_u64(out, &len, max, hw.gicc_ctlr);
        http_append(out, &len, max, " pmr=");
        http_append_u64(out, &len, max, hw.gicc_pmr);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "irq selftest")) {
        http_append(out, &len, max, irq_diag_selftest() ? "IRQ selftest OK\n" : "IRQ selftest FAILED\n");
    } else if (http_starts_with(cmd, "irq cntpns step ")) {
        u32 depth = 0;
        const char *arg = cmd + 16;
        while (*arg >= '0' && *arg <= '9') { depth = depth*10 + (u32)(*arg - '0'); arg++; }
        u32 d_ctlr = 0, c_ctlr = 0, pmr = 0, isp = 0, ise = 0, iar = 0;
        u32 last_step = irq_cntpns_step(depth, &d_ctlr, &c_ctlr, &pmr, &isp, &ise, &iar);
        http_append(out, &len, max, "irq cntpns step depth=");
        http_append_u64(out, &len, max, depth);
        http_append(out, &len, max, " reached=");
        http_append_u64(out, &len, max, last_step);
        http_append(out, &len, max, " d_ctlr=");
        http_append_hex32(out, &len, max, d_ctlr);
        http_append(out, &len, max, " c_ctlr=");
        http_append_hex32(out, &len, max, c_ctlr);
        http_append(out, &len, max, " pmr=");
        http_append_hex32(out, &len, max, pmr);
        http_append(out, &len, max, " ispend=");
        http_append_hex32(out, &len, max, isp);
        http_append(out, &len, max, " isenable=");
        http_append_hex32(out, &len, max, ise);
        http_append(out, &len, max, " iar=");
        http_append_hex32(out, &len, max, iar);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "irq cntpns confirm")) {
        u64 before = 0, after = 0;
        u32 last = 0, d_ctlr = 0, c_ctlr = 0, unhandled = 0;
        bool ok = irq_cntpns_test(&before, &after, &last, &d_ctlr, &c_ctlr, &unhandled);
        http_append(out, &len, max, ok ? "IRQ cntpns OK" : "IRQ cntpns FAILED");
        http_append(out, &len, max, " timer_before=");
        http_append_u64(out, &len, max, before);
        http_append(out, &len, max, " timer_after=");
        http_append_u64(out, &len, max, after);
        http_append(out, &len, max, " last_intid=");
        http_append_u64(out, &len, max, last);
        http_append(out, &len, max, " unhandled=");
        http_append_u64(out, &len, max, unhandled);
        http_append(out, &len, max, " d_ctlr=");
        http_append_hex32(out, &len, max, d_ctlr);
        http_append(out, &len, max, " c_ctlr=");
        http_append_hex32(out, &len, max, c_ctlr);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "irq trace")) {
        u32 ent=0, pre_h=0, post_h=0, pre_e=0, post_e=0, last_iar=0;
        irq_trace_dump(&ent, &pre_h, &post_h, &pre_e, &post_e, &last_iar);
        http_append(out, &len, max, "irq trace enter=");
        http_append_u64(out, &len, max, ent);
        http_append(out, &len, max, " pre_handler=");
        http_append_u64(out, &len, max, pre_h);
        http_append(out, &len, max, " post_handler=");
        http_append_u64(out, &len, max, post_h);
        http_append(out, &len, max, " pre_eoi=");
        http_append_u64(out, &len, max, pre_e);
        http_append(out, &len, max, " post_eoi=");
        http_append_u64(out, &len, max, post_e);
        http_append(out, &len, max, " last_iar=");
        http_append_hex32(out, &len, max, last_iar);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "irq trace reset")) {
        irq_trace_reset();
        http_append(out, &len, max, "irq trace reset OK\n");
    } else if (http_streq(cmd, "irq sdtrace")) {
        /* Read back the SD-resident IRQ trace sectors (LBA 15..23). Each
         * sector is 512 bytes; we render just the first 64 bytes (magic +
         * counters) as hex. Used after a watchdog-induced reboot to see
         * how far the IRQ vector got before wedging. */
        static u8 sd_sec[SD_BLOCK_SIZE] ALIGNED(64);
        for (u32 lba = 15; lba <= 23; lba++) {
            http_append(out, &len, max, "LBA ");
            http_append_u64(out, &len, max, lba);
            if (!sd_read_block(lba, sd_sec)) {
                http_append(out, &len, max, " READ_FAIL\n");
                continue;
            }
            http_append(out, &len, max, " magic=");
            for (u32 i = 0; i < 8; i++) {
                char c = (char)sd_sec[i];
                if (c >= 0x20 && c < 0x7F) {
                    char tmp[2] = { c, 0 };
                    http_append(out, &len, max, tmp);
                } else {
                    http_append(out, &len, max, ".");
                }
            }
            http_append(out, &len, max, " hex=");
            for (u32 i = 0; i < 40; i++) {
                http_append_hex32(out, &len, max, sd_sec[i]);
                if ((i & 3) == 3) http_append(out, &len, max, " ");
            }
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "irq sdtrace wipe")) {
        static u8 sd_zero[SD_BLOCK_SIZE] ALIGNED(64);
        for (u32 i = 0; i < SD_BLOCK_SIZE; i++) sd_zero[i] = 0;
        for (u32 lba = 15; lba <= 23; lba++) (void)sd_write_block(lba, sd_zero);
        http_append(out, &len, max, "irq sdtrace wipe OK\n");
    } else if (http_streq(cmd, "irq probe")) {
        struct irq_gic_probe_snapshot p;
        irq_gic_probe_snapshot(&p);
        http_append(out, &len, max, "irq probe current_driver_id=");
        http_append_u64(out, &len, max, p.current_driver_id);
        http_append(out, &len, max, " count=");
        http_append_u64(out, &len, max, p.count);
        http_append(out, &len, max, "\nID D_BASE C_BASE D_CTLR D_TYPER D_IIDR C_CTLR C_PMR PLAUSIBLE\n");
        for (u32 i = 0; i < p.count; i++) {
            const struct irq_gic_probe_entry *e = &p.entries[i];
            http_append_u64(out, &len, max, e->id);
            http_append(out, &len, max, " ");
            http_append_hex64(out, &len, max, e->gicd_base);
            http_append(out, &len, max, " ");
            http_append_hex64(out, &len, max, e->gicc_base);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, e->gicd_ctlr);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, e->gicd_typer);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, e->gicd_iidr);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, e->gicc_ctlr);
            http_append(out, &len, max, " ");
            http_append_hex32(out, &len, max, e->gicc_pmr);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, e->plausible ? "yes" : "no");
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "abi") || http_streq(cmd, "abi status")) {
        struct abi_status a;
        abi_status(&a);
        http_append(out, &len, max, "abi stage=");
        http_append(out, &len, max, abi_stage_name(a.stage));
        http_append(out, &len, max, " direct_kpi=");
        http_append(out, &len, max, a.direct_kpi ? "yes" : "no");
        http_append(out, &len, max, " ksvc_registry=");
        http_append(out, &len, max, a.ksvc_registry ? "yes" : "no");
        http_append(out, &len, max, " ksvc_mailboxes=");
        http_append(out, &len, max, a.ksvc_mailboxes ? "yes" : "no");
        http_append(out, &len, max, " ksvc_callbacks=");
        http_append(out, &len, max, a.ksvc_callbacks ? "yes" : "no");
        http_append(out, &len, max, " svc_trap=");
        http_append(out, &len, max, a.svc_trap_ready ? "ready" : "pending");
        http_append(out, &len, max, " svc_calls=");
        http_append_u64(out, &len, max, a.svc_calls);
        http_append(out, &len, max, " svc_bad=");
        http_append_u64(out, &len, max, a.svc_bad_calls);
        http_append(out, &len, max, " entry_contract=");
        http_append(out, &len, max, a.el0_entry_contract ? "ready" : "pending");
        http_append(out, &len, max, " el0=");
        http_append(out, &len, max, a.el0_ready ? "ready" : "pending");
        http_append(out, &len, max, " ttbr_split=");
        http_append(out, &len, max, a.user_ttbr_split ? "yes" : "no");
        http_append(out, &len, max, " entry_flags=");
        http_append_u64(out, &len, max, a.el0_entry_flags);
        http_append(out, &len, max, " el0_spsr=");
        http_append_u64(out, &len, max, a.el0_spsr);
        http_append(out, &len, max, " kapi_size=");
        http_append_u64(out, &len, max, a.kernel_api_version);
        http_append(out, &len, max, " pending_steps=");
        http_append_u64(out, &len, max, a.pending_steps);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "abi selftest")) {
        http_append(out, &len, max, abi_selftest() ? "ABI selftest OK\n" : "ABI selftest FAILED\n");
    } else if (http_streq(cmd, "ipc bench") || http_starts_with(cmd, "ipc bench ")) {
        u32 iters = 10000;
        if (http_starts_with(cmd, "ipc bench ")) {
            u32 parsed = 0;
            if (http_parse_u32(cmd + 10, &parsed) && parsed > 0)
                iters = parsed;
        }
        struct proc_ipc_bench_result b;
        bool ok = proc_ipc_bench(iters, &b);
        http_append(out, &len, max, ok ? "ipc bench OK" : "ipc bench ERR");
        http_append(out, &len, max, " n=");
        http_append_u64(out, &len, max, b.iterations);
        http_append(out, &len, max, " desc=");
        http_append_u64(out, &len, max, b.desc_size);
        http_append(out, &len, max, " h=");
        http_append_u64(out, &len, max, (u64)(u32)b.fifo_handle);
        http_append(out, &len, max, " errors=");
        http_append_u64(out, &len, max, b.errors);
        http_append(out, &len, max, " svc_ticks=");
        http_append_u64(out, &len, max, b.svc_ticks);
        http_append(out, &len, max, " span_ticks=");
        http_append_u64(out, &len, max, b.span_ticks);
        http_append(out, &len, max, " span_fast_ticks=");
        http_append_u64(out, &len, max, b.span_fast_ticks);
        http_append(out, &len, max, " copy64_ticks=");
        http_append_u64(out, &len, max, b.copy64_ticks);
        http_append(out, &len, max, " copy512_ticks=");
        http_append_u64(out, &len, max, b.copy512_ticks);
        http_append(out, &len, max, " memcpy2048_ticks=");
        http_append_u64(out, &len, max, b.memcpy2048_ticks);
        http_append(out, &len, max, " span2048_ticks=");
        http_append_u64(out, &len, max, b.span2048_ticks);
        http_append(out, &len, max, " cross_fifo_ticks=");
        http_append_u64(out, &len, max, b.cross_fifo_ticks);
        http_append(out, &len, max, " sev_ticks=");
        http_append_u64(out, &len, max, b.sev_ticks);
        if (b.iterations) {
            http_append(out, &len, max, " svc_per=");
            http_append_u64(out, &len, max, b.svc_ticks / b.iterations);
            http_append(out, &len, max, " span_per=");
            http_append_u64(out, &len, max, b.span_ticks / b.iterations);
            http_append(out, &len, max, " span_fast_per=");
            http_append_u64(out, &len, max, b.span_fast_ticks / b.iterations);
            http_append(out, &len, max, " copy64_per=");
            http_append_u64(out, &len, max, b.copy64_ticks / b.iterations);
            http_append(out, &len, max, " copy512_per=");
            http_append_u64(out, &len, max, b.copy512_ticks / b.iterations);
            http_append(out, &len, max, " memcpy2048_per=");
            http_append_u64(out, &len, max, b.memcpy2048_ticks / b.iterations);
            http_append(out, &len, max, " span2048_per=");
            http_append_u64(out, &len, max, b.span2048_ticks / b.iterations);
            http_append(out, &len, max, " cross_fifo_per=");
            http_append_u64(out, &len, max, b.cross_fifo_ticks / b.iterations);
            http_append(out, &len, max, " sev_per=");
            http_append_u64(out, &len, max, b.sev_ticks / b.iterations);
        }
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "qpu") || http_streq(cmd, "qpu status") ||
               http_streq(cmd, "tensor") || http_streq(cmd, "tensor status")) {
        struct tensor_status ts;
        tensor_status(&ts);
        http_append(out, &len, max, "tensor v3d_available=");
        http_append(out, &len, max, ts.v3d_available ? "yes" : "no");
        http_append(out, &len, max, " dispatch=");
        http_append(out, &len, max, ts.v3d_dispatch_supported ? "yes" : "no");
        http_append(out, &len, max, " fallback=");
        http_append(out, &len, max, ts.qpu_fallback ? "yes" : "no");
        http_append(out, &len, max, " any_kernel=");
        http_append(out, &len, max, ts.any_kernel_bound ? "yes" : "no");
        http_append(out, &len, max, " ready_mask=");
        http_append_u64(out, &len, max, ts.ready_mask);
        http_append(out, &len, max, " disabled_mask=");
        http_append_u64(out, &len, max, ts.disabled_mask);
        http_append(out, &len, max, " ident0=");
        http_append_u64(out, &len, max, ts.ident0);
        http_append(out, &len, max, " ident1=");
        http_append_u64(out, &len, max, ts.ident1);
        http_append(out, &len, max, " ident2=");
        http_append_u64(out, &len, max, ts.ident2);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tensor selftest") || http_streq(cmd, "qpu selftest")) {
        http_append(out, &len, max, tensor_selftest() ? "Tensor selftest OK\n" : "Tensor selftest FAILED\n");
    } else if (http_starts_with(cmd, "addr ")) {
        struct pios_addr a;
        char canon[160];
        if (!pios_addr_parse(cmd + 5, &a) || !pios_addr_format(&a, canon, sizeof(canon))) {
            http_append(out, &len, max, "ERR: usage addr <kind:pack/card[/tail]>\n");
        } else {
            http_append(out, &len, max, "canonical=");
            http_append(out, &len, max, canon);
            http_append(out, &len, max, "\nkind=");
            http_append(out, &len, max, pios_addr_kind_name(a.kind));
            http_append(out, &len, max, " pack=");
            http_append_u64(out, &len, max, a.pack);
            http_append(out, &len, max, " card=");
            http_append_u64(out, &len, max, a.card);
            if (a.kind == PIOS_ADDR_WAL) {
                u32 key = 0;
                if (picowal_db_pack_key((u16)a.pack, a.card, &key)) {
                    http_append(out, &len, max, " dbkey=");
                    http_append_u64(out, &len, max, key);
                }
            } else if (a.kind == PIOS_ADDR_TCP || a.kind == PIOS_ADDR_UDP) {
                http_append(out, &len, max, " port=");
                http_append_u64(out, &len, max, a.card);
            }
            if (a.tail[0]) {
                http_append(out, &len, max, " tail=");
                http_append(out, &len, max, a.tail);
            }
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "mem") || http_streq(cmd, "mem analyze")) {
        http_append_mem_analyze(out, &len, max);
    } else if (http_streq(cmd, "ls") || http_streq(cmd, "lsdir") || http_streq(cmd, "fsinspect")) {
        http_append_walfs_list_text(out, &len, max, "/");
    } else if (http_starts_with(cmd, "ls ")) {
        http_append_walfs_list_text(out, &len, max, cmd + 3);
    } else if (http_starts_with(cmd, "lsdir ")) {
        http_append_walfs_list_text(out, &len, max, cmd + 6);
    } else if (http_starts_with(cmd, "fsinspect ")) {
        http_append_walfs_list_text(out, &len, max, cmd + 10);
    } else if (http_streq(cmd, "walfs") || http_streq(cmd, "walfs status") ||
               http_streq(cmd, "fs status")) {
        struct walfs_status_snapshot ws;
        walfs_status(&ws);
        http_append(out, &len, max, "walfs mounted=");
        http_append(out, &len, max, ws.mounted ? "yes" : "no");
        http_append(out, &len, max, " root=");
        http_append(out, &len, max, ws.root_ok ? "ok" : "bad");
        http_append(out, &len, max, " super=");
        http_append(out, &len, max, ws.super_ok ? "ok" : "bad");
        http_append(out, &len, max, " legacy=");
        http_append(out, &len, max, ws.legacy_present ? "yes" : "no");
        http_append(out, &len, max, " p2_lba=");
        http_append_u64(out, &len, max, ws.partition_lba);
        http_append(out, &len, max, " walfs_lba=");
        http_append_u64(out, &len, max, ws.base_lba);
        http_append(out, &len, max, " region_blocks=");
        http_append_u64(out, &len, max, ws.region_blocks);
        http_append(out, &len, max, "\nsuper magic=");
        http_append_hex32(out, &len, max, ws.super_magic);
        http_append(out, &len, max, " version=");
        http_append_u64(out, &len, max, ws.super_version);
        http_append(out, &len, max, " records=");
        http_append_u64(out, &len, max, ws.super_records);
        http_append(out, &len, max, " head=");
        http_append_u64(out, &len, max, ws.super_head);
        http_append(out, &len, max, " tree_root=");
        http_append_u64(out, &len, max, ws.super_tree_root);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "walfs format confirm")) {
        bool ok = walfs_format_reserved();
        http_append(out, &len, max, ok ? "WALFS format OK\n" : "WALFS format FAILED\n");
        if (ok)
            http_append_walfs_list_text(out, &len, max, "/");
    } else if (http_streq(cmd, "bootctrl") || http_streq(cmd, "bootctrl status")) {
        http_append_bootctrl_status(out, &len, max);
    } else if (http_streq(cmd, "bootctrl clear-pending")) {
        http_append(out, &len, max,
                    pios_bootctrl_clear_pending() ? "bootctrl clear-pending OK\n" :
                                                    "bootctrl clear-pending FAILED\n");
        http_append_bootctrl_status(out, &len, max);
    } else if (http_streq(cmd, "bootctrl reset-a confirm")) {
        http_append(out, &len, max,
                    pios_bootctrl_reset_a() ? "bootctrl reset-a OK\n" :
                                             "bootctrl reset-a FAILED\n");
        http_append_bootctrl_status(out, &len, max);
    } else if (http_streq(cmd, "bootctrl test-invalid-b confirm")) {
        http_append(out, &len, max,
                    pios_bootctrl_test_invalid_b() ? "bootctrl test-invalid-b OK\n" :
                                                     "bootctrl test-invalid-b FAILED\n");
        http_append_bootctrl_status(out, &len, max);
    } else if (http_streq(cmd, "keystore") || http_streq(cmd, "keystore status")) {
        struct keystore_status st;
        keystore_status(&st);
        http_append(out, &len, max, "keystore initialized=");
        http_append(out, &len, max, st.initialized ? "yes" : "no");
        http_append(out, &len, max, " sealed=");
        http_append(out, &len, max, st.sealed ? "yes" : "no");
        http_append(out, &len, max, " serial=");
        http_append(out, &len, max, st.board_serial_ok ? "ok" : "fallback");
        http_append(out, &len, max, " generation=");
        http_append_u64(out, &len, max, st.generation);
        http_append(out, &len, max, " error=");
        http_append_u64(out, &len, max, st.last_error);
        http_append(out, &len, max, " lba=");
        http_append_u64(out, &len, max, st.user_records_lba);
        http_append(out, &len, max, " fingerprint=");
        http_append_u64(out, &len, max, st.fingerprint32);
        http_append(out, &len, max, "\n");
    } else if (http_starts_with(cmd, "keystore derive ")) {
        u32 fp = 0;
        if (keystore_derive_fingerprint(cmd + 16, &fp)) {
            http_append(out, &len, max, "derive fingerprint=");
            http_append_u64(out, &len, max, fp);
            http_append(out, &len, max, "\n");
        } else {
            http_append(out, &len, max, "ERR: derive failed\n");
        }
    } else if (http_starts_with(cmd, "db ")) {
        char *argv[10];
        u32 argc = http_split_args(cmd, argv, 10);
        if (argc < 2) {
            http_append(out, &len, max, "ERR: usage db key|get|put|del|list <addr>\n");
        } else if (http_streq(argv[1], "list")) {
            if (argc < 3) {
                http_append(out, &len, max, "ERR: usage db list <card|wal:pack/card>\n");
            } else {
                u32 card = 0;
                u32 ignored = 0;
                u16 c16 = 0;
                if (pios_addr_parse_picowal(argv[2], &c16, &ignored)) card = c16;
                else if (!http_parse_u32(argv[2], &card) || card > PICOWAL_CARD_MAX) card = 0xFFFFFFFFU;
                if (card == 0xFFFFFFFFU) {
                    http_append(out, &len, max, "ERR: invalid card\n");
                } else {
                    u32 ids[64];
                    u32 n = picowal_db_list((u16)card, ids, 64);
                    http_append(out, &len, max, "db card=");
                    http_append_u64(out, &len, max, card);
                    http_append(out, &len, max, " count=");
                    http_append_u64(out, &len, max, n);
                    http_append(out, &len, max, "\n");
                    for (u32 i = 0; i < n; i++) {
                        http_append(out, &len, max, "rec=");
                        http_append_u64(out, &len, max, ids[i]);
                        http_append(out, &len, max, "\n");
                    }
                }
            }
        } else {
            u32 card = 0, rec = 0, argi = 0;
            if (!http_parse_db_ref(argc, argv, 2, &card, &rec, &argi)) {
                http_append(out, &len, max, "ERR: invalid db address\n");
            } else if (http_streq(argv[1], "key")) {
                u32 key = 0;
                if (picowal_db_pack_key((u16)card, rec, &key)) {
                    http_append(out, &len, max, "key=");
                    http_append_u64(out, &len, max, key);
                    http_append(out, &len, max, " card=");
                    http_append_u64(out, &len, max, card);
                    http_append(out, &len, max, " record=");
                    http_append_u64(out, &len, max, rec);
                    http_append(out, &len, max, "\n");
                } else {
                    http_append(out, &len, max, "ERR: key pack failed\n");
                }
            } else if (http_streq(argv[1], "del")) {
                http_append(out, &len, max, picowal_db_delete((u16)card, rec) ? "OK: deleted\n" : "ERR: delete failed\n");
            } else if (http_streq(argv[1], "get")) {
                static u8 data[PICOWAL_DATA_MAX];
                i32 n = picowal_db_get((u16)card, rec, data, PICOWAL_DATA_MAX);
                if (n < 0) {
                    http_append(out, &len, max, "ERR: get failed\n");
                } else {
                    http_append(out, &len, max, "db card=");
                    http_append_u64(out, &len, max, card);
                    http_append(out, &len, max, " record=");
                    http_append_u64(out, &len, max, rec);
                    http_append(out, &len, max, " len=");
                    http_append_u64(out, &len, max, (u32)n);
                    http_append(out, &len, max, "\n");
                    for (i32 i = 0; i < n && len + 1 < max; i++) {
                        u8 c = data[i];
                        out[len++] = (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
                        out[len] = 0;
                    }
                    http_append(out, &len, max, "\n");
                }
            } else if (http_streq(argv[1], "put")) {
                if (argc <= argi) {
                    http_append(out, &len, max, "ERR: usage db put <addr> <text...>\n");
                } else {
                    static u8 data[PICOWAL_DATA_MAX];
                    u32 p = 0;
                    for (u32 i = argi; i < argc; i++) {
                        const char *s = argv[i];
                        while (*s && p < PICOWAL_DATA_MAX) data[p++] = (u8)*s++;
                        if (i + 1 < argc && p < PICOWAL_DATA_MAX) data[p++] = ' ';
                    }
                    i32 n = picowal_db_put((u16)card, rec, data, p);
                    if (n < 0) http_append(out, &len, max, "ERR: put failed\n");
                    else {
                        http_append(out, &len, max, "OK: wrote ");
                        http_append_u64(out, &len, max, (u32)n);
                        http_append(out, &len, max, " bytes\n");
                    }
                }
            } else {
                http_append(out, &len, max, "ERR: unknown db op\n");
            }
        }
    } else if (http_streq(cmd, "processes")) {
        struct proc_ui_entry snap[MAX_PROCS_PER_CORE + 1U];
        u32 n = proc_snapshot(snap, MAX_PROCS_PER_CORE + 1U);
        http_append(out, &len, max, "PID PPID CORE STATE PRI CPU MEMK ACAP AUSED AHI ABUMP ASPAN SCNT IMAGE\n");
        for (u32 i = 0; i < n; i++) {
            http_append_u64(out, &len, max, snap[i].pid);
            http_append(out, &len, max, " ");
            http_append_pid_field(out, &len, max, snap[i].parent_pid);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].affinity_core);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, ui_proc_state_str(snap[i].state));
            http_append(out, &len, max, " ");
            http_append(out, &len, max, ui_priority_str(snap[i].priority_class));
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].cpu_percent);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].mem_kib);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].arena_capacity_kib);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].arena_used_kib);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].arena_high_kib);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].arena_bump_kib);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].arena_span_kib);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, snap[i].arena_span_count);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, snap[i].image_path);
            http_append(out, &len, max, "\n");
        }
        http_append(out, &len, max, "\nGRAPH\n");
        for (u32 i = 0; i < n; i++) {
            if (snap[i].parent_pid != 0 && proc_ui_has_pid(snap, n, snap[i].parent_pid))
                continue;
            http_append_u64(out, &len, max, snap[i].pid);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, snap[i].image_path);
            http_append(out, &len, max, "\n");
            for (u32 j = 0; j < n; j++) {
                if (snap[j].parent_pid != snap[i].pid)
                    continue;
                http_append(out, &len, max, "  -> ");
                http_append_u64(out, &len, max, snap[j].pid);
                http_append(out, &len, max, " ");
                http_append(out, &len, max, snap[j].image_path);
                http_append(out, &len, max, "\n");
            }
        }
    } else if (http_starts_with(cmd, "process validate ")) {
        const char *path = cmd + 17;
        struct proc_image_validation v;
        (void)proc_validate_image_path(path, &v);
        http_append_proc_image_validation(out, &len, max, &v);
    } else if (http_streq(cmd, "users")) {
        struct principal_ui_entry users[PRINCIPAL_MAX];
        u32 n = principal_snapshot(users, PRINCIPAL_MAX);
        http_append(out, &len, max, "ID FLAGS NAME\n");
        for (u32 i = 0; i < n; i++) {
            http_append_u64(out, &len, max, users[i].id);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, users[i].flags);
            http_append(out, &len, max, " ");
            http_append(out, &len, max, users[i].name);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "firewall") || http_streq(cmd, "firewall list")) {
        http_append_firewall_list(out, &len, max);
    } else if (http_starts_with(cmd, "kill ")) {
        u32 pid = 0;
        if (http_parse_u32(cmd + 5, &pid) && proc_kill_pid(pid, 0xFFFF5002U))
            http_append(out, &len, max, "killed\n");
        else
            http_append(out, &len, max, "kill failed\n");
    } else if (http_starts_with(cmd, "restart ")) {
        http_append(out, &len, max, "restart blocked: process launch/restart is disabled on HTTP console until non-crashing harness exists\n");
    } else {
        http_append(out, &len, max, "unknown command\n");
    }
    return len;
}

static void http_append_action_result(char *out, u32 *len, u32 max,
                                      const char *action, bool ok,
                                      const char *message)
{
    http_append(out, len, max, "{\"ok\":");
    http_append(out, len, max, ok ? "true" : "false");
    http_append(out, len, max, ",\"action\":");
    http_append_json_string(out, len, max, action ? action : "");
    http_append(out, len, max, ",\"message\":");
    http_append_json_string(out, len, max, message ? message : "");
}

static u32 http_build_process_action_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char action[16];
    char pid_s[16];
    char path[96];
    char core_s[8];
    char principal_s[16];
    char prio_s[8];
    u32 pid = 0;
    i32 new_pid = -1;
    bool ok = false;
    const char *message = "bad request";
    struct proc_image_validation image_val;
    bool have_image_val = false;

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");

    if (!http_query_value(req, req_len, "/api/process", "action", action, sizeof(action))) {
        http_append_action_result(out, &len, max, "", false, "missing action");
        http_append(out, &len, max, "}\n");
        return len;
    }

    if (ui_streq(action, "validate")) {
        if (!http_query_value(req, req_len, "/api/process", "path", path, sizeof(path))) {
            message = "missing path";
        } else {
            have_image_val = true;
            ok = proc_validate_image_path(path, &image_val);
            message = proc_image_status_name(image_val.status);
        }
    } else if (ui_streq(action, "kill") || ui_streq(action, "restart")) {
        if (!http_query_value(req, req_len, "/api/process", "pid", pid_s, sizeof(pid_s)) ||
            !http_parse_u32(pid_s, &pid)) {
            message = "invalid pid";
        } else if (ui_streq(action, "kill")) {
            ok = proc_kill_pid(pid, 0xFFFF5000U);
            message = ok ? "killed" : "kill failed";
        } else {
            ok = false;
            message = "restart blocked: process launch/restart disabled until non-crashing harness exists";
        }
    } else if (ui_streq(action, "launch")) {
        (void)path;
        (void)core_s;
        (void)principal_s;
        (void)prio_s;
        ok = false;
        message = "launch blocked: prior live process launch wedged board; use local console only after root-cause harness";
    } else {
        message = "unknown action";
    }

    http_append_action_result(out, &len, max, action, ok, message);
    if (pid) {
        http_append(out, &len, max, ",\"pid\":");
        http_append_u64(out, &len, max, pid);
    }
    if (new_pid > 0) {
        http_append(out, &len, max, ",\"newPid\":");
        http_append_u64(out, &len, max, (u32)new_pid);
    }
    if (have_image_val)
        http_append_proc_image_validation_json(out, &len, max, &image_val);
    http_append(out, &len, max, "}\n");
    return len;
}

static u32 http_build_user_action_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char action[16];
    char name[32];
    char pass[48];
    char flags_s[16];
    u32 flags = PRINCIPAL_EXEC;
    bool ok = false;
    const char *message = "bad request";

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");

    if (!http_query_value(req, req_len, "/api/user", "action", action, sizeof(action))) {
        http_append_action_result(out, &len, max, "", false, "missing action");
        http_append(out, &len, max, "}\n");
        return len;
    }
    if (!http_query_value(req, req_len, "/api/user", "name", name, sizeof(name))) {
        http_append_action_result(out, &len, max, action, false, "missing name");
        http_append(out, &len, max, "}\n");
        return len;
    }

    if (ui_streq(action, "create")) {
        if (!http_query_value(req, req_len, "/api/user", "pass", pass, sizeof(pass))) {
            message = "missing password";
        } else {
            if (http_query_value(req, req_len, "/api/user", "flags", flags_s, sizeof(flags_s)))
                (void)http_parse_u32(flags_s, &flags);
            ok = principal_create(name, pass, flags);
            message = ok ? "created" : "create failed";
        }
    } else if (ui_streq(action, "passwd")) {
        if (!http_query_value(req, req_len, "/api/user", "pass", pass, sizeof(pass))) {
            message = "missing password";
        } else {
            ok = principal_set_password(name, pass);
            message = ok ? "password updated" : "password update failed";
        }
    } else if (ui_streq(action, "flags")) {
        if (!http_query_value(req, req_len, "/api/user", "flags", flags_s, sizeof(flags_s)) ||
            !http_parse_u32(flags_s, &flags)) {
            message = "invalid flags";
        } else {
            ok = principal_set_flags(name, flags);
            message = ok ? "flags updated" : "flags update failed";
        }
    } else {
        message = "unknown action";
    }

    http_append_action_result(out, &len, max, action, ok, message);
    http_append(out, &len, max, "}\n");
    return len;
}

static u32 http_build_logs_response(char *out, u32 max)
{
    u32 len = 0;
    static struct proc_log_ui_entry logs[16];
    u32 n = proc_log_snapshot(logs, 16);

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max, "{\"version\":\"");
    http_append(out, &len, max, PIOS_VERSION);
    http_append(out, &len, max, "\",\"build\":\"");
    http_append(out, &len, max, PIOS_BUILD_LABEL);
    http_append(out, &len, max, "\",\"uptime\":");
    http_append_u64(out, &len, max, timer_monotonic_ms() / 1000ULL);
    http_append(out, &len, max, ",\"http\":{");
    http_append_json_metric(out, &len, max, "accepts", http_diag.accepts, true);
    http_append_json_metric(out, &len, max, "reads", http_diag.reads, true);
    http_append_json_metric(out, &len, max, "built", http_diag.built, true);
    http_append_json_metric(out, &len, max, "writes", http_diag.write_calls, true);
    http_append_json_metric(out, &len, max, "writeBytes", http_diag.write_bytes, true);
    http_append_json_metric(out, &len, max, "closes", http_diag.closes, true);
    http_append_json_metric(out, &len, max, "aborts", http_diag.aborts, true);
    http_append_json_metric(out, &len, max, "notFound", http_diag.not_found, false);
    http_append(out, &len, max, "},\"entries\":[");
    for (u32 i = 0; i < n; i++) {
        if (i) http_append(out, &len, max, ",");
        http_append(out, &len, max, "{\"core\":"); http_append_u64(out, &len, max, logs[i].core);
        http_append(out, &len, max, ",\"seq\":"); http_append_u64(out, &len, max, logs[i].seq);
        http_append(out, &len, max, ",\"level\":"); http_append_u64(out, &len, max, logs[i].level);
        http_append(out, &len, max, ",\"msg\":"); http_append_json_string(out, &len, max, logs[i].msg);
        http_append(out, &len, max, "}");
    }
    http_append(out, &len, max, "]}\n");
    return len;
}

struct http_walfs_dir_ctx {
    char *out;
    u32 *len;
    u32 max;
    u32 count;
};

static struct http_walfs_dir_ctx http_walfs_dir;

static void http_walfs_readdir_cb(const struct walfs_dirent *entry)
{
    if (!entry || http_walfs_dir.count >= 64)
        return;
    struct walfs_inode ino;
    if (!walfs_stat(entry->child_id, &ino))
        return;
    if (http_walfs_dir.count)
        http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ",");
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, "{\"name\":");
    http_append_json_string(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, (const char *)entry->name);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ",\"id\":");
    http_append_u64(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, entry->child_id);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ",\"size\":");
    http_append_u64(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ino.size);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ",\"dir\":");
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, (ino.flags & WALFS_DIR) ? "true" : "false");
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ",\"flags\":");
    http_append_u64(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ino.flags);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, "}");
    http_walfs_dir.count++;
}

static void http_walfs_text_readdir_cb(const struct walfs_dirent *entry)
{
    if (!entry || http_walfs_dir.count >= 64)
        return;
    struct walfs_inode ino;
    if (!walfs_stat(entry->child_id, &ino))
        return;
    http_append_u64(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, entry->child_id);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, " ");
    http_append_u64(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ino.size);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, " ");
    http_append_u64(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, ino.flags);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max,
                (ino.flags & WALFS_DIR) ? " dir " : " file ");
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, (const char *)ino.name);
    http_append(http_walfs_dir.out, http_walfs_dir.len, http_walfs_dir.max, "\n");
    http_walfs_dir.count++;
}

static void http_append_walfs_list_text(char *out, u32 *len, u32 max, const char *path)
{
    if (!path || !*path)
        path = "/";
    if (path[0] != '/') {
        http_append(out, len, max, "ERR: path must be absolute\n");
        return;
    }

    u64 id = walfs_find(path);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino)) {
        if (path[0] == '/' && path[1] == 0)
            http_append(out, len, max, "ERR: WALFS root unavailable\n");
        else
            http_append(out, len, max, "ERR: path not found\n");
        return;
    }

    http_append(out, len, max, "path=");
    http_append(out, len, max, path);
    http_append(out, len, max, " id=");
    http_append_u64(out, len, max, id);
    http_append(out, len, max, " size=");
    http_append_u64(out, len, max, ino.size);
    http_append(out, len, max, " flags=");
    http_append_u64(out, len, max, ino.flags);
    http_append(out, len, max, (ino.flags & WALFS_DIR) ? " dir\n" : " file\n");
    if (!(ino.flags & WALFS_DIR))
        return;

    http_append(out, len, max, "ID SIZE FLAGS TYPE NAME\n");
    http_walfs_dir.out = out;
    http_walfs_dir.len = len;
    http_walfs_dir.max = max;
    http_walfs_dir.count = 0;
    walfs_readdir(id, http_walfs_text_readdir_cb);
    http_append(out, len, max, "entries=");
    http_append_u64(out, len, max, http_walfs_dir.count);
    http_append(out, len, max, "\n");
}

static void http_append_walfs_hex(char *out, u32 *len, u32 max, const u8 *buf, u32 n, u32 base_off)
{
    http_append(out, len, max, "\"");
    for (u32 off = 0; off < n; off += 16) {
        if (off) http_append(out, len, max, "\\n");
        http_append_hex32(out, len, max, base_off + off);
        http_append(out, len, max, ": ");
        for (u32 i = 0; i < 16; i++) {
            if (off + i < n) {
                http_append_hex8(out, len, max, buf[off + i]);
                http_append(out, len, max, " ");
            } else {
                http_append(out, len, max, "   ");
            }
        }
        http_append(out, len, max, " ");
        for (u32 i = 0; i < 16 && off + i < n; i++) {
            u8 c = buf[off + i];
            char s[2] = { (c >= 32 && c < 127) ? (char)c : '.', 0 };
            if (c == '"' || c == '\\')
                s[0] = '.';
            http_append(out, len, max, s);
        }
    }
    http_append(out, len, max, "\"");
}

static u32 http_build_walfs_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char path[256];
    char mode[12];
    char off_s[16];
    char max_s[16];
    u32 offset = 0;
    u32 want = 768;
    static u8 buf[1024];

    if (!http_query_value(req, req_len, "/api/walfs", "path", path, sizeof(path))) {
        u32 p = 0;
        http_append(path, &p, sizeof(path), "/");
    }
    if (!http_query_value(req, req_len, "/api/walfs", "mode", mode, sizeof(mode))) {
        u32 m = 0;
        http_append(mode, &m, sizeof(mode), "dir");
    }
    if (http_query_value(req, req_len, "/api/walfs", "offset", off_s, sizeof(off_s)))
        (void)http_parse_u32(off_s, &offset);
    if (http_query_value(req, req_len, "/api/walfs", "max", max_s, sizeof(max_s)))
        (void)http_parse_u32(max_s, &want);
    if (want == 0 || want > sizeof(buf))
        want = 768;

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");

    if (path[0] != '/') {
        http_append(out, &len, max, "{\"ok\":false,\"error\":\"path must be absolute\"}\n");
        return len;
    }
    u64 id = walfs_find(path);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino)) {
        http_append(out, &len, max, "{\"ok\":false,\"error\":\"path not found\",\"path\":");
        http_append_json_string(out, &len, max, path);
        http_append(out, &len, max, "}\n");
        return len;
    }

    http_append(out, &len, max, "{\"ok\":true,\"path\":");
    http_append_json_string(out, &len, max, path);
    http_append(out, &len, max, ",\"id\":"); http_append_u64(out, &len, max, id);
    http_append(out, &len, max, ",\"name\":"); http_append_json_string(out, &len, max, (const char *)ino.name);
    http_append(out, &len, max, ",\"size\":"); http_append_u64(out, &len, max, ino.size);
    http_append(out, &len, max, ",\"flags\":"); http_append_u64(out, &len, max, ino.flags);
    http_append(out, &len, max, ",\"modeBits\":"); http_append_u64(out, &len, max, ino.mode);
    http_append(out, &len, max, ",\"dir\":");
    http_append(out, &len, max, (ino.flags & WALFS_DIR) ? "true" : "false");

    if (ino.flags & WALFS_DIR) {
        http_append(out, &len, max, ",\"entries\":[");
        http_walfs_dir.out = out;
        http_walfs_dir.len = &len;
        http_walfs_dir.max = max;
        http_walfs_dir.count = 0;
        walfs_readdir(id, http_walfs_readdir_cb);
        http_append(out, &len, max, "],\"count\":");
        http_append_u64(out, &len, max, http_walfs_dir.count);
    } else {
        if (offset > ino.size)
            offset = (u32)ino.size;
        u32 remain = (u32)((ino.size - offset) > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (ino.size - offset));
        u32 n = remain < want ? remain : want;
        n = walfs_read(id, offset, buf, n);
        http_append(out, &len, max, ",\"offset\":"); http_append_u64(out, &len, max, offset);
        http_append(out, &len, max, ",\"read\":"); http_append_u64(out, &len, max, n);
        http_append(out, &len, max, ",\"view\":");
        http_append_json_string(out, &len, max, ui_streq(mode, "hex") ? "hex" : "text");
        if (ui_streq(mode, "hex")) {
            http_append(out, &len, max, ",\"data\":");
            http_append_walfs_hex(out, &len, max, buf, n, offset);
        } else {
            http_append(out, &len, max, ",\"data\":");
            http_append_json_bytes_text(out, &len, max, buf, n);
        }
    }
    http_append(out, &len, max, "}\n");
    return len;
}

extern u8 _start;
extern u8 __text_start;
extern u8 __text_end;
extern u8 __bss_start;
extern u8 __bss_end;
extern u8 __heap_start;
static const char *ui_proc_state_str(u32 s);

static bool http_confirmed(const u8 *req, u32 req_len, const char *path)
{
    char confirm[8];
    return http_query_value(req, req_len, path, "confirm", confirm, sizeof(confirm)) &&
           http_streq(confirm, "1");
}

static u32 http_query_u32_default(const u8 *req, u32 req_len, const char *path,
                                  const char *key, u32 def)
{
    char tmp[16];
    u32 v = def;
    if (http_query_value(req, req_len, path, key, tmp, sizeof(tmp)))
        (void)http_parse_u32(tmp, &v);
    return v;
}

static bool http_update_query_value(const u8 *req, u32 req_len, const char *key,
                                    char *out, u32 out_max)
{
    return http_query_value(req, req_len, "/api/admin/kernel-update", key, out, out_max) ||
           http_query_value(req, req_len, "/", key, out, out_max);
}

static bool http_update_confirmed(const u8 *req, u32 req_len)
{
    char confirm[8];
    return http_update_query_value(req, req_len, "confirm", confirm, sizeof(confirm)) &&
           http_streq(confirm, "1");
}

static u32 http_update_query_u32_default(const u8 *req, u32 req_len,
                                         const char *key, u32 def)
{
    char tmp[16];
    u32 v = def;
    if (http_update_query_value(req, req_len, key, tmp, sizeof(tmp)))
        (void)http_parse_u32(tmp, &v);
    return v;
}

static u32 http_running_kernel_image_len(void)
{
    u64 start = (u64)(usize)&_start;
    u64 end = (u64)(usize)&__bss_start;
    if (end <= start || (end - start) > 0xFFFFFFFFULL)
        return 0;
    return (u32)(end - start);
}

static void http_append_mem_analyze(char *out, u32 *len, u32 max)
{
    u64 k_start = (u64)(usize)&_start;
    u64 text_start = (u64)(usize)&__text_start;
    u64 text_end = (u64)(usize)&__text_end;
    u64 bss_start = (u64)(usize)&__bss_start;
    u64 bss_end = (u64)(usize)&__bss_end;
    u64 heap_start = (u64)(usize)&__heap_start;
    u32 image_len = http_running_kernel_image_len();
    struct proc_ui_entry snap[MAX_PROCS_PER_CORE + 1U];
    u32 n = proc_snapshot(snap, MAX_PROCS_PER_CORE + 1U);

    http_append(out, len, max, "mem kernel_start=");
    http_append_u64(out, len, max, k_start);
    http_append(out, len, max, " image_bytes=");
    http_append_u64(out, len, max, image_len);
    http_append(out, len, max, " slot_capacity=");
    http_append_u64(out, len, max, PIOS_STAGE2_ZONE_BYTES);
    http_append(out, len, max, " slot_free=");
    http_append_u64(out, len, max, image_len < PIOS_STAGE2_ZONE_BYTES ? PIOS_STAGE2_ZONE_BYTES - image_len : 0);
    http_append(out, len, max, "\ntext=");
    http_append_u64(out, len, max, text_start);
    http_append(out, len, max, "..");
    http_append_u64(out, len, max, text_end);
    http_append(out, len, max, " rodata_data_to_bss=");
    http_append_u64(out, len, max, bss_start > text_end ? bss_start - text_end : 0);
    http_append(out, len, max, " bss=");
    http_append_u64(out, len, max, bss_start);
    http_append(out, len, max, "..");
    http_append_u64(out, len, max, bss_end);
    http_append(out, len, max, " heap_start=");
    http_append_u64(out, len, max, heap_start);
    http_append(out, len, max, "\nCORE RAM_BASE SLOT0 SLOT_END\n");
    for (u32 c = 0; c < 4; c++) {
        u64 base = core_ram_bases[c];
        http_append_u64(out, len, max, c);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, base);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, base + PROC_SLOT_OFFSET);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, base + PROC_SLOT_OFFSET + ((u64)MAX_PROCS_PER_CORE * PROC_SLOT_SIZE));
        http_append(out, len, max, "\n");
    }
    http_append(out, len, max, "PROC PID CORE STATE MEMK ACAP AUSED AHI ABUMP ASPAN SCNT IMAGE\n");
    for (u32 i = 0; i < n; i++) {
        http_append_u64(out, len, max, snap[i].pid);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].affinity_core);
        http_append(out, len, max, " ");
        http_append(out, len, max, ui_proc_state_str(snap[i].state));
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].mem_kib);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].arena_capacity_kib);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].arena_used_kib);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].arena_high_kib);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].arena_bump_kib);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].arena_span_kib);
        http_append(out, len, max, " ");
        http_append_u64(out, len, max, snap[i].arena_span_count);
        http_append(out, len, max, " ");
        http_append(out, len, max, snap[i].image_path);
        http_append(out, len, max, "\n");
    }
}

static void http_append_proc_image_validation(char *out, u32 *len, u32 max,
                                              const struct proc_image_validation *v)
{
    bool ok = v && v->status == PROC_IMAGE_STATUS_OK;
    http_append(out, len, max, "process validate ");
    http_append(out, len, max, ok ? "OK" : "FAILED");
    http_append(out, len, max, " status=");
    http_append(out, len, max, proc_image_status_name(v ? v->status : PROC_IMAGE_STATUS_HEADER_INVALID));
    http_append(out, len, max, " format=");
    http_append(out, len, max, proc_image_format_name(v ? v->format : PROC_IMAGE_FORMAT_NONE));
    http_append(out, len, max, " launch=");
    http_append(out, len, max, proc_image_launch_mode_name(v ? v->launch_mode : PROC_IMAGE_LAUNCH_BLOCKED));
    if (v) {
        http_append(out, len, max, " file_bytes=");
        http_append_u64(out, len, max, v->file_bytes);
        http_append(out, len, max, " header_bytes=");
        http_append_u64(out, len, max, v->header_bytes);
        http_append(out, len, max, " entry_offset=");
        http_append_u64(out, len, max, v->entry_offset);
        http_append(out, len, max, " code=");
        http_append_u64(out, len, max, v->code_size);
        http_append(out, len, max, " data=");
        http_append_u64(out, len, max, v->data_size);
        http_append(out, len, max, " bss=");
        http_append_u64(out, len, max, v->bss_size);
        http_append(out, len, max, " load_span=");
        http_append_u64(out, len, max, v->load_span);
        if (v->format == PROC_IMAGE_FORMAT_PIX) {
            http_append(out, len, max, " type=");
            http_append_u64(out, len, max, v->image_type);
            http_append(out, len, max, " flags=");
            http_append_u64(out, len, max, v->image_flags);
            http_append(out, len, max, " relocs=");
            http_append_u64(out, len, max, v->reloc_count);
            http_append(out, len, max, " imports=");
            http_append_u64(out, len, max, v->import_count);
        }
        if (v->validator_status) {
            http_append(out, len, max, " validator=");
            http_append_u64(out, len, max, v->validator_status);
        }
    }
    http_append(out, len, max, "\n");
}

static void http_append_proc_image_validation_json(char *out, u32 *len, u32 max,
                                                   const struct proc_image_validation *v)
{
    if (!v)
        return;
    http_append(out, len, max, ",\"status\":\"");
    http_append(out, len, max, proc_image_status_name(v->status));
    http_append(out, len, max, "\",\"format\":\"");
    http_append(out, len, max, proc_image_format_name(v->format));
    http_append(out, len, max, "\",\"launchMode\":\"");
    http_append(out, len, max, proc_image_launch_mode_name(v->launch_mode));
    http_append(out, len, max, "\",\"fileBytes\":");
    http_append_u64(out, len, max, v->file_bytes);
    http_append(out, len, max, ",\"headerBytes\":");
    http_append_u64(out, len, max, v->header_bytes);
    http_append(out, len, max, ",\"entryOffset\":");
    http_append_u64(out, len, max, v->entry_offset);
    http_append(out, len, max, ",\"codeBytes\":");
    http_append_u64(out, len, max, v->code_size);
    http_append(out, len, max, ",\"dataBytes\":");
    http_append_u64(out, len, max, v->data_size);
    http_append(out, len, max, ",\"bssBytes\":");
    http_append_u64(out, len, max, v->bss_size);
    http_append(out, len, max, ",\"loadSpan\":");
    http_append_u64(out, len, max, v->load_span);
    http_append(out, len, max, ",\"imageType\":");
    http_append_u64(out, len, max, v->image_type);
    http_append(out, len, max, ",\"imageFlags\":");
    http_append_u64(out, len, max, v->image_flags);
    http_append(out, len, max, ",\"validatorStatus\":");
    http_append_u64(out, len, max, v->validator_status);
}

static void pios_write_le32(u8 *p, u32 v)
{
    p[0] = (u8)(v & 0xFF);
    p[1] = (u8)((v >> 8) & 0xFF);
    p[2] = (u8)((v >> 16) & 0xFF);
    p[3] = (u8)((v >> 24) & 0xFF);
}

static u32 pios_read_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void pios_fill_reserved_header(u8 *out, u32 magic, u32 payload_len)
{
    simd_zero(out, SD_BLOCK_SIZE);
    pios_write_le32(out + PIOS_HDR_MAGIC_OFF, magic);
    pios_write_le32(out + PIOS_HDR_STAGE2_LEN_OFF, payload_len);
    pios_write_le32(out + PIOS_HDR_LAYOUT_VERSION_OFF, PIOS_RESERVED_LAYOUT_VERSION);
    pios_write_le32(out + PIOS_HDR_RESERVED_BYTES_OFF, PIOS_RESERVED_BYTES);
    pios_write_le32(out + PIOS_HDR_STAGE2_OFFSET_OFF, PIOS_STAGE2_OFFSET);
    pios_write_le32(out + PIOS_HDR_STAGE2_BYTES_OFF, PIOS_STAGE2_ZONE_BYTES);
    pios_write_le32(out + PIOS_HDR_TCPIP_OFFSET_OFF, PIOS_TCPIP_STACK_OFFSET);
    pios_write_le32(out + PIOS_HDR_TCPIP_BYTES_OFF, PIOS_TCPIP_STACK_BYTES);
    pios_write_le32(out + PIOS_HDR_FIREWALL_OFFSET_OFF, PIOS_FIREWALL_CFG_OFFSET);
    pios_write_le32(out + PIOS_HDR_FIREWALL_BYTES_OFF, PIOS_FIREWALL_CFG_BYTES);
    pios_write_le32(out + PIOS_HDR_ADMIN_OFFSET_OFF, PIOS_ADMIN_HTTP_OFFSET);
    pios_write_le32(out + PIOS_HDR_DEBUG_OFFSET_OFF, PIOS_KERNEL_DEBUG_OFFSET);
    pios_write_le32(out + PIOS_HDR_USER_RECORDS_OFF, PIOS_USER_RECORDS_OFFSET);
    pios_write_le32(out + PIOS_HDR_HOT_LOGS_OFF, PIOS_HOT_LOGS_OFFSET);
    pios_write_le32(out + PIOS_HDR_HOT_LOGS_BYTES_OFF, PIOS_HOT_LOGS_BYTES);
    pios_write_le32(out + PIOS_HDR_CRASHDUMP_OFF, PIOS_CRASHDUMP_OFFSET);
    pios_write_le32(out + PIOS_HDR_FUTURE_OFF, PIOS_FUTURE_RESERVED_OFFSET);
    pios_write_le32(out + PIOS_HDR_WALFS_OFF, PIOS_WALFS_OFFSET);
}

static u32 pios_boot_slot_offset(u32 slot)
{
    return slot == PIOS_BOOTCTRL_SLOT_B ? PIOS_BOOT_SLOT_B_OFFSET : PIOS_BOOT_SLOT_A_OFFSET;
}

static u32 pios_bootctrl_checksum(const u8 *p)
{
    u32 sum = 0xB007C0DEU;
    for (u32 i = 0; i < PIOS_BOOTCTRL_CHECKSUM_OFF; i++)
        sum = (sum << 5) ^ (sum >> 27) ^ p[i];
    return sum;
}

static bool pios_bootctrl_valid(const u8 *p)
{
    if (pios_read_le32(p + PIOS_BOOTCTRL_MAGIC_OFF) != PIOS_BOOTCTRL_MAGIC)
        return false;
    if (pios_read_le32(p + PIOS_BOOTCTRL_VERSION_OFF) != PIOS_BOOTCTRL_VERSION)
        return false;
    return pios_read_le32(p + PIOS_BOOTCTRL_CHECKSUM_OFF) == pios_bootctrl_checksum(p);
}

static bool pios_bootctrl_read(u8 *out)
{
    if (!out)
        return false;
    u32 lba = walfs_partition_lba() + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE);
    return sd_read_block(lba, out) && pios_bootctrl_valid(out);
}

static bool pios_bootctrl_write_at(u32 root_lba, u8 *p)
{
    if (!p)
        return false;
    pios_write_le32(p + PIOS_BOOTCTRL_CHECKSUM_OFF, pios_bootctrl_checksum(p));
    u32 lba = root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE);
    return sd_write_block(lba, p);
}

static bool pios_bootctrl_write(u8 *p)
{
    return pios_bootctrl_write_at(walfs_partition_lba(), p);
}

static void pios_bootctrl_init_good_at(u32 root_lba, u32 active_slot)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    if (active_slot > PIOS_BOOTCTRL_SLOT_B)
        active_slot = PIOS_BOOTCTRL_SLOT_A;
    simd_zero(ctl, sizeof(ctl));
    pios_write_le32(ctl + PIOS_BOOTCTRL_MAGIC_OFF, PIOS_BOOTCTRL_MAGIC);
    pios_write_le32(ctl + PIOS_BOOTCTRL_VERSION_OFF, PIOS_BOOTCTRL_VERSION);
    pios_write_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, active_slot);
    pios_write_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    pios_write_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 0);
    pios_write_le32(ctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, active_slot);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, 1U << active_slot);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF, 1);
    (void)pios_bootctrl_write_at(root_lba, ctl);
}

static u32 pios_bootctrl_target_slot(void)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    if (!pios_bootctrl_read(ctl))
        return PIOS_BOOTCTRL_SLOT_B;
    u32 active = pios_read_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF);
    return active == PIOS_BOOTCTRL_SLOT_B ? PIOS_BOOTCTRL_SLOT_A : PIOS_BOOTCTRL_SLOT_B;
}

static void pios_bootctrl_mark_pending(u32 pending_slot)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    if (pending_slot > PIOS_BOOTCTRL_SLOT_B)
        pending_slot = PIOS_BOOTCTRL_SLOT_B;
    if (!pios_bootctrl_read(ctl)) {
        simd_zero(ctl, sizeof(ctl));
        pios_write_le32(ctl + PIOS_BOOTCTRL_MAGIC_OFF, PIOS_BOOTCTRL_MAGIC);
        pios_write_le32(ctl + PIOS_BOOTCTRL_VERSION_OFF, PIOS_BOOTCTRL_VERSION);
        pios_write_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, PIOS_BOOTCTRL_SLOT_A);
        pios_write_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, 1U << PIOS_BOOTCTRL_SLOT_A);
    }
    u32 gen = pios_read_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF);
    pios_write_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, pending_slot);
    pios_write_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, PIOS_BOOTCTRL_TRIES_DEFAULT);
    pios_write_le32(ctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF,
                    pios_read_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF) & ~(1U << pending_slot));
    pios_write_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF, gen + 1U);
    (void)pios_bootctrl_write(ctl);
}

static bool pios_bootctrl_clear_pending(void)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    if (!pios_bootctrl_read(ctl))
        return false;
    u32 gen = pios_read_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF);
    pios_write_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    pios_write_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 0);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF, gen + 1U);
    return pios_bootctrl_write(ctl);
}

static bool pios_bootctrl_reset_a(void)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    simd_zero(ctl, sizeof(ctl));
    pios_write_le32(ctl + PIOS_BOOTCTRL_MAGIC_OFF, PIOS_BOOTCTRL_MAGIC);
    pios_write_le32(ctl + PIOS_BOOTCTRL_VERSION_OFF, PIOS_BOOTCTRL_VERSION);
    pios_write_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, PIOS_BOOTCTRL_SLOT_A);
    pios_write_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    pios_write_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 0);
    pios_write_le32(ctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, PIOS_BOOTCTRL_SLOT_A);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, 1U << PIOS_BOOTCTRL_SLOT_A);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF, 1);
    return pios_bootctrl_write(ctl);
}

static bool pios_bootctrl_test_invalid_b(void)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    if (!http_write_kernel_slot_header(PIOS_BOOT_SLOT_B_OFFSET, SD_BLOCK_SIZE, false))
        return false;
    if (!pios_bootctrl_read(ctl)) {
        simd_zero(ctl, sizeof(ctl));
        pios_write_le32(ctl + PIOS_BOOTCTRL_MAGIC_OFF, PIOS_BOOTCTRL_MAGIC);
        pios_write_le32(ctl + PIOS_BOOTCTRL_VERSION_OFF, PIOS_BOOTCTRL_VERSION);
    }
    u32 gen = pios_read_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF);
    pios_write_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, PIOS_BOOTCTRL_SLOT_A);
    pios_write_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_B);
    pios_write_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, PIOS_BOOTCTRL_TRIES_DEFAULT);
    pios_write_le32(ctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, 1U << PIOS_BOOTCTRL_SLOT_A);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF, gen + 1U);
    return pios_bootctrl_write(ctl);
}

static void pios_bootctrl_mark_success(void)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    if (!pios_bootctrl_read(ctl))
        return;
    u32 booted = pios_read_le32(ctl + PIOS_BOOTCTRL_LAST_BOOT_OFF);
    if (booted > PIOS_BOOTCTRL_SLOT_B)
        return;
    u32 good = pios_read_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF) | (1U << booted);
    u32 gen = pios_read_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF);
    pios_write_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, booted);
    pios_write_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    pios_write_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 0);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, good);
    pios_write_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF, gen + 1U);
    (void)pios_bootctrl_write(ctl);
}

static const char *pios_bootctrl_slot_name(u32 slot)
{
    if (slot == PIOS_BOOTCTRL_SLOT_A) return "A";
    if (slot == PIOS_BOOTCTRL_SLOT_B) return "B";
    if (slot == PIOS_BOOTCTRL_SLOT_NONE) return "none";
    return "bad";
}

static void http_append_bootctrl_status(char *out, u32 *len, u32 max)
{
    static u8 ctl[SD_BLOCK_SIZE] ALIGNED(64);
    bool ok = pios_bootctrl_read(ctl);
    http_append(out, len, max, "bootctrl ok=");
    http_append(out, len, max, ok ? "yes" : "no");
    http_append(out, len, max, " lba=");
    http_append_u64(out, len, max, walfs_partition_lba() + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE));
    if (!ok) {
        http_append(out, len, max, "\n");
        return;
    }

    u32 active = pios_read_le32(ctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF);
    u32 pending = pios_read_le32(ctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF);
    u32 tries = pios_read_le32(ctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF);
    u32 last = pios_read_le32(ctl + PIOS_BOOTCTRL_LAST_BOOT_OFF);
    u32 good = pios_read_le32(ctl + PIOS_BOOTCTRL_GOOD_MASK_OFF);
    u32 gen = pios_read_le32(ctl + PIOS_BOOTCTRL_GENERATION_OFF);

    http_append(out, len, max, " active=");
    http_append(out, len, max, pios_bootctrl_slot_name(active));
    http_append(out, len, max, " pending=");
    http_append(out, len, max, pios_bootctrl_slot_name(pending));
    http_append(out, len, max, " tries=");
    http_append_u64(out, len, max, tries);
    http_append(out, len, max, " last=");
    http_append(out, len, max, pios_bootctrl_slot_name(last));
    http_append(out, len, max, " good_mask=");
    http_append_hex32(out, len, max, good);
    http_append(out, len, max, " generation=");
    http_append_u64(out, len, max, gen);
    http_append(out, len, max, "\nslot_lba A=");
    http_append_u64(out, len, max, walfs_partition_lba() + (PIOS_BOOT_SLOT_A_OFFSET / SD_BLOCK_SIZE));
    http_append(out, len, max, " B=");
    http_append_u64(out, len, max, walfs_partition_lba() + (PIOS_BOOT_SLOT_B_OFFSET / SD_BLOCK_SIZE));
    http_append(out, len, max, "\n");
}

static void ui_cmd_bootctrl(u32 argc, char **argv)
{
    static char out[512];
    u32 len = 0;
    if (argc < 2 || ui_streq(argv[1], "status")) {
        http_append_bootctrl_status(out, &len, sizeof(out));
        ui_console_write(out);
        return;
    }
    if (ui_streq(argv[1], "clear-pending")) {
        ui_console_write(pios_bootctrl_clear_pending() ? "bootctrl clear-pending OK\n" :
                                                       "bootctrl clear-pending FAILED\n");
        http_append_bootctrl_status(out, &len, sizeof(out));
        ui_console_write(out);
        return;
    }
    if (ui_streq(argv[1], "reset-a") && argc >= 3 && ui_streq(argv[2], "confirm")) {
        ui_console_write(pios_bootctrl_reset_a() ? "bootctrl reset-a OK\n" :
                                                   "bootctrl reset-a FAILED\n");
        http_append_bootctrl_status(out, &len, sizeof(out));
        ui_console_write(out);
        return;
    }
    if (ui_streq(argv[1], "test-invalid-b") && argc >= 3 && ui_streq(argv[2], "confirm")) {
        ui_console_write(pios_bootctrl_test_invalid_b() ? "bootctrl test-invalid-b OK\n" :
                                                          "bootctrl test-invalid-b FAILED\n");
        http_append_bootctrl_status(out, &len, sizeof(out));
        ui_console_write(out);
        return;
    }
    ui_console_write("ERR: usage bootctrl status | bootctrl clear-pending | bootctrl reset-a confirm | bootctrl test-invalid-b confirm\n");
}

static bool http_write_kernel_slot_range(u32 slot_offset, u32 offset, const u8 *data, u32 len,
                                         u32 *out_written)
{
    static u8 block[SD_BLOCK_SIZE] ALIGNED(64);
    if (out_written) *out_written = 0;
    if (!data && len != 0)
        return false;
    if (offset > HOTPATCH_SLOT_BYTES || len > HOTPATCH_SLOT_BYTES - offset)
        return false;

    u32 base_lba = walfs_partition_lba() + (slot_offset / SD_BLOCK_SIZE);
    u32 pos = offset;
    u32 written = 0;
    while (written < len) {
        u32 sector_off = pos % SD_BLOCK_SIZE;
        u32 n = SD_BLOCK_SIZE - sector_off;
        if (n > len - written)
            n = len - written;
        u32 lba = base_lba + (pos / SD_BLOCK_SIZE);
        if (sector_off == 0 && n == SD_BLOCK_SIZE) {
            if (!sd_write_block(lba, data + written))
                return false;
        } else {
            if (!sd_read_block(lba, block))
                simd_zero(block, sizeof(block));
            simd_memcpy(block + sector_off, data + written, n);
            if (!sd_write_block(lba, block))
                return false;
        }
        pos += n;
        written += n;
    }
    if (out_written) *out_written = written;
    return true;
}

static bool http_write_kernel_slot_header(u32 slot_offset, u32 payload_len, bool valid)
{
    static u8 header[SD_BLOCK_SIZE] ALIGNED(64);
    u32 written = 0;
    u32 magic = valid ? HOTPATCH_SLOT_MAGIC : 0U;
    pios_fill_reserved_header(header, magic, payload_len);
    return http_write_kernel_slot_range(slot_offset, 0, header, SD_BLOCK_SIZE, &written) &&
           written == SD_BLOCK_SIZE;
}

static bool http_write_kernel_payload_range(u32 slot_offset, u32 offset, const u8 *data, u32 len,
                                            u32 *out_written)
{
    if (offset > PIOS_STAGE2_ZONE_BYTES ||
        len > PIOS_STAGE2_ZONE_BYTES - offset)
        return false;
    return http_write_kernel_slot_range(slot_offset, PIOS_STAGE2_OFFSET + offset, data, len,
                                        out_written);
}

static bool http_write_kernel_slot_image(u32 slot_offset, const u8 *data, u32 len, u32 *out_written)
{
    if (out_written) *out_written = 0;
    if (!data || len == 0 || len > PIOS_STAGE2_ZONE_BYTES)
        return false;
    u32 written = 0;
    if (!http_write_kernel_slot_header(slot_offset, len, false))
        return false;
    if (!http_write_kernel_payload_range(slot_offset, 0, data, len, &written))
        return false;
    if (written != len)
        return false;
    if (!http_write_kernel_slot_header(slot_offset, len, true))
        return false;
    if (out_written) *out_written = len;
    return true;
}

static u32 http_build_reboot_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    bool confirmed = http_confirmed(req, req_len, "/api/admin/reboot");
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    if (!confirmed) {
        http_append(out, &len, max,
            "{\"ok\":false,\"error\":\"requires confirm=1\",\"endpoint\":\"/api/admin/reboot\"}\n");
        return len;
    }
    http_reboot_pending = true;
    http_log_event("reboot-queued", 0, 0);
    http_append(out, &len, max,
        "{\"ok\":true,\"message\":\"reboot queued after HTTP response close\"}\n");
    return len;
}

static u32 http_build_log_stream_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    u32 since = http_query_u32_default(req, req_len, "/api/admin/log-stream", "since", 0);
    u32 tail = http_query_u32_default(req, req_len, "/api/admin/log-stream", "tail", 0);
    if (since == 0)
        since = http_query_u32_default(req, req_len, "/logs", "since", 0);
    if (tail == 0)
        tail = http_query_u32_default(req, req_len, "/logs", "tail", 0);
    if (tail > HTTP_LOG_RING_SIZE)
        tail = HTTP_LOG_RING_SIZE;
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max, "PIOS log-stream next=");
    http_append_u64(out, &len, max, http_log_seq);
    http_append(out, &len, max, "\n");
    u32 start = http_log_seq > HTTP_LOG_RING_SIZE ? http_log_seq - HTTP_LOG_RING_SIZE : 0;
    if (tail != 0) {
        u32 tail_start = http_log_seq > tail ? http_log_seq - tail : 0;
        if (tail_start > start) start = tail_start;
    }
    if (since > start) start = since;
    for (u32 seq = start; seq < http_log_seq; seq++) {
        struct http_log_entry *e = &http_log_ring[seq % HTTP_LOG_RING_SIZE];
        if (e->seq != seq)
            continue;
        http_append_u64(out, &len, max, e->seq);
        http_append(out, &len, max, " t=");
        http_append_u64(out, &len, max, e->tick_ms);
        http_append(out, &len, max, " ");
        http_append(out, &len, max, e->event ? e->event : "?");
        http_append(out, &len, max, " a=");
        http_append_u64(out, &len, max, e->a);
        http_append(out, &len, max, " b=");
        http_append_u64(out, &len, max, e->b);
        http_append(out, &len, max, "\n");
    }
    return len;
}

static u32 http_build_hotpatch_kernel_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    bool confirmed = http_confirmed(req, req_len, "/api/admin/hotpatch-kernel");
    char target[12];
    u32 image_len = http_running_kernel_image_len();
    u32 written = 0;
    u32 capacity = HOTPATCH_SLOT_BYTES;
    bool ok = false;
    const char *error = NULL;

    if (!http_query_value(req, req_len, "/api/admin/hotpatch-kernel", "target",
                          target, sizeof(target))) {
        u32 target_len = 0;
        http_append(target, &target_len, sizeof(target), "slot");
    }

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    if (!confirmed) {
        http_append(out, &len, max,
            "{\"ok\":false,\"error\":\"requires confirm=1\",\"endpoint\":\"/api/admin/hotpatch-kernel\"}\n");
        return len;
    }

    if (ui_streq(target, "slot")) {
        capacity = HOTPATCH_SLOT_BYTES;
        if (image_len == 0 || image_len > PIOS_STAGE2_ZONE_BYTES) {
            error = "image does not fit raw bootstrap slot";
        } else {
            ok = http_write_kernel_slot_image(PIOS_BOOT_SLOT_A_OFFSET,
                                              (const u8 *)(usize)&_start,
                                              image_len, &written);
            if (!ok) error = "raw slot write failed";
        }
    } else {
        error = "unsupported target; use target=slot";
    }

    http_append(out, &len, max, "{\"ok\":");
    http_append(out, &len, max, ok ? "true" : "false");
    http_append(out, &len, max, ",\"target\":");
    http_append_json_string(out, &len, max, target);
    http_append(out, &len, max, ",\"path\":\"raw-partition2-kernel-slot\",\"imageBytes\":");
    http_append_u64(out, &len, max, image_len);
    http_append(out, &len, max, ",\"written\":");
    http_append_u64(out, &len, max, written);
    http_append(out, &len, max, ",\"capacity\":");
    http_append_u64(out, &len, max, capacity);
    http_append(out, &len, max, ",\"slotLba\":");
    http_append_u64(out, &len, max, walfs_partition_lba());
    http_append(out, &len, max, ",\"slotBytes\":");
    http_append_u64(out, &len, max, HOTPATCH_SLOT_BYTES);
    if (!ok && error) {
        http_append(out, &len, max, ",\"error\":");
        http_append_json_string(out, &len, max, error);
    }
    http_append(out, &len, max, ",\"bootstrapPlan\":\"kernel8.img is stable stage0; partition-2 block0 is the PIOS reserved-area header; second-stage payload is 0x000200..0x1FFFFF; WALFS starts at 0xA00000\"");
    http_append(out, &len, max, "}\n");
    return len;
}

static u32 http_build_kernel_update_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    bool confirmed = http_update_confirmed(req, req_len);
    u32 body_off = http_header_body_offset(req, req_len);
    u32 body_len = 0;
    u32 content_len = 0;
    bool has_content_len = false;
    char action[16];
    u32 offset = http_update_query_u32_default(req, req_len, "offset", 0);
    u32 total = http_update_query_u32_default(req, req_len, "total", 0);
    u32 written = 0;
    u32 capacity = PIOS_STAGE2_ZONE_BYTES;
    bool ok = false;
    const char *error = NULL;

    action[0] = 0;
    (void)http_update_query_value(req, req_len, "action", action, sizeof(action));
    if (body_off != 0 && body_off <= req_len)
        body_len = req_len - body_off;
    if (body_off != 0)
        has_content_len = http_content_length(req, body_off, &content_len);

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    if (!confirmed) {
        http_append(out, &len, max,
            "{\"ok\":false,\"error\":\"requires confirm=1\",\"endpoint\":\"/api/admin/kernel-update\"}\n");
        return len;
    }

    if (action[0] == 0 && body_len > 0) {
        action[0] = 'c';
        action[1] = 'h';
        action[2] = 'u';
        action[3] = 'n';
        action[4] = 'k';
        action[5] = 0;
    }

    if (action[0] == 0 || http_streq(action, "status")) {
        ok = true;
    } else if (http_streq(action, "self")) {
        u32 image_len = http_running_kernel_image_len();
        ok = image_len != 0 &&
             http_write_kernel_slot_image(PIOS_BOOT_SLOT_A_OFFSET,
                                          (const u8 *)(usize)&_start,
                                          image_len, &written);
        if (!ok) error = "self-update raw slot write failed";
        else {
            ota_update.active = false;
            ota_update.total = image_len;
            ota_update.received = image_len;
        }
    } else if (http_streq(action, "begin")) {
        if (total == 0 || total > capacity) {
            error = "invalid total; must fit raw payload slot";
        } else {
            ota_update.target_slot = pios_bootctrl_target_slot();
            ota_update.target_slot_offset = pios_boot_slot_offset(ota_update.target_slot);
        }
        if (!error && !http_write_kernel_slot_header(ota_update.target_slot_offset, total, false)) {
            error = "failed to invalidate slot header";
        } else if (!error) {
            ota_update.active = true;
            ota_update.total = total;
            ota_update.received = 0;
            ota_update.chunks = 0;
            ota_update.last_error = NULL;
            ok = true;
            http_log_event("ota-begin", total, walfs_partition_lba());
        }
    } else if (http_streq(action, "chunk") || http_streq(action, "data")) {
        if (!ota_update.active) {
            error = "no OTA update active; call action=begin first";
        } else if (total != 0 && total != ota_update.total) {
            error = "total does not match active OTA update";
        } else if (!has_content_len) {
            error = "chunk requires Content-Length";
        } else if (body_len != content_len) {
            error = "incomplete or truncated chunk body";
        } else if (body_len == 0) {
            error = "empty chunk";
        } else if (body_len > ADMIN_HTTP_REQ_MAX - 512U) {
            error = "chunk too large for request buffer";
        } else if (offset > ota_update.received) {
            error = "chunk offset is ahead of next expected byte";
        } else if (offset > ota_update.total || body_len > ota_update.total - offset) {
            error = "chunk exceeds declared total";
        } else {
            u32 skip = ota_update.received - offset;
            if (skip >= body_len) {
                written = body_len;
                ok = true;
            } else {
                u32 new_len = body_len - skip;
                ok = http_write_kernel_payload_range(ota_update.target_slot_offset,
                                                    ota_update.received,
                                                    req + body_off + skip,
                                                    new_len, &written);
                if (ok && written == new_len) {
                    ota_update.received += written;
                    written = body_len;
                } else {
                    error = "raw slot payload write failed";
                    ok = false;
                }
            }
            if (ok) {
                ota_update.chunks++;
                if ((ota_update.chunks & 0x3FU) == 1U || ota_update.received == ota_update.total)
                    http_log_event("ota-chunk", ota_update.received, ota_update.total);
            }
        }
    } else if (http_streq(action, "commit")) {
        if (!ota_update.active) {
            error = "no OTA update active";
        } else if (total != 0 && total != ota_update.total) {
            error = "total does not match active OTA update";
        } else if (ota_update.received != ota_update.total) {
            error = "OTA image incomplete";
        } else if (!http_write_kernel_slot_header(ota_update.target_slot_offset,
                                                  ota_update.received, true)) {
            error = "failed to commit slot header";
        } else {
            ok = true;
            ota_update.active = false;
            ota_update.commits++;
            pios_bootctrl_mark_pending(ota_update.target_slot);
            http_log_event("ota-commit", ota_update.received, ota_update.commits);
            char reboot[8];
            if (http_update_query_value(req, req_len, "reboot", reboot, sizeof(reboot)) &&
                http_streq(reboot, "1"))
                http_reboot_pending = true;
        }
    } else if (http_streq(action, "cancel")) {
        ota_update.active = false;
        ota_update.last_error = "cancelled";
        ok = true;
        http_log_event("ota-cancel", ota_update.received, ota_update.total);
    } else {
        error = "unknown action; use status, begin, chunk, commit, cancel, or self";
    }

    if (!ok && error) {
        ota_update.errors++;
        ota_update.last_error = error;
        http_log_event("ota-error", offset, body_len);
    }

    http_append(out, &len, max, "{\"ok\":");
    http_append(out, &len, max, ok ? "true" : "false");
    http_append(out, &len, max, ",\"action\":");
    http_append_json_string(out, &len, max, action[0] ? action : "status");
    http_append(out, &len, max, ",\"offset\":");
    http_append_u64(out, &len, max, offset);
    http_append(out, &len, max, ",\"bodyBytes\":");
    http_append_u64(out, &len, max, body_len);
    http_append(out, &len, max, ",\"written\":");
    http_append_u64(out, &len, max, written);
    http_append(out, &len, max, ",\"capacity\":");
    http_append_u64(out, &len, max, capacity);
    http_append(out, &len, max, ",\"total\":");
    http_append_u64(out, &len, max, ota_update.total);
    http_append(out, &len, max, ",\"received\":");
    http_append_u64(out, &len, max, ota_update.received);
    http_append(out, &len, max, ",\"nextOffset\":");
    http_append_u64(out, &len, max, ota_update.received);
    http_append(out, &len, max, ",\"active\":");
    http_append(out, &len, max, ota_update.active ? "true" : "false");
    http_append(out, &len, max, ",\"chunks\":");
    http_append_u64(out, &len, max, ota_update.chunks);
    http_append(out, &len, max, ",\"commits\":");
    http_append_u64(out, &len, max, ota_update.commits);
    http_append(out, &len, max, ",\"errors\":");
    http_append_u64(out, &len, max, ota_update.errors);
    http_append(out, &len, max, ",\"slotLba\":");
    http_append_u64(out, &len, max, walfs_partition_lba());
    if (error) {
        http_append(out, &len, max, ",\"error\":");
        http_append_json_string(out, &len, max, error);
    } else if (ota_update.last_error) {
        http_append(out, &len, max, ",\"lastError\":");
        http_append_json_string(out, &len, max, ota_update.last_error);
    }
    http_append(out, &len, max, "}\n");
    return len;
}

static u32 http_build_safe_placeholder_response(char *out, u32 max, const char *name)
{
    u32 len = 0;
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max, name);
    http_append(out, &len, max,
        " is visible in the tabbed console but its live data path is disabled "
        "until the snapshot code is re-enabled defensively.\n");
    return len;
}

static u32 http_build_acme_challenge_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char token[ACME_TOKEN_MAX];
    char keyauth[ACME_KEY_AUTH_MAX];
    bool ok = http_request_path_prefix_token(req, req_len, "/.well-known/acme-challenge/",
                                             token, sizeof(token)) &&
              acme_http01_key_authorization(token, keyauth, sizeof(keyauth));
    if (!ok) {
        http_append(out, &len, max,
            "HTTP/1.0 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n\r\n"
            "acme challenge not found\n");
        return len;
    }
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max, keyauth);
    http_append(out, &len, max, "\n");
    return len;
}

static u32 http_build_spa_response(char *out, u32 max)
{
    u32 len = 0;
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
        "<!doctype html><html><head><meta charset='utf-8'><title>PIOS Admin</title>"
        "<script>(()=>{const p=new URLSearchParams(window.location.search).get('clawpilotTheme');const t=p||(window.matchMedia('(prefers-color-scheme: dark)').matches?'dark':'light');document.documentElement.setAttribute('data-theme',t);})();</script>"
        "<style>:root{color-scheme:light;--cp-bg:#f7f4ef;--cp-bg-elevated:#fcfbf8;--cp-surface:#ffffff;--cp-surface-soft:#f5f5f5;--cp-border:#dedede;--cp-border-strong:#919191;--cp-text:#242424;--cp-text-muted:#5c5c5c;--cp-text-soft:#6f6f6f;--cp-accent:#b11f4b;--cp-accent-hover:#9a1a41;--cp-accent-soft:rgba(177,31,75,.08);--cp-accent-fg:#ffffff;--cp-success:#16a34a;--cp-danger:#dc2626;--cp-warning:#f59e0b;--cp-link:#0078d4;--cp-shadow:0 18px 48px rgba(0,0,0,.12);--cp-overlay:rgba(255,255,255,.8);--cp-panel:rgba(255,255,255,.86);--cp-panel-strong:rgba(255,255,255,.96);--cp-sheen:rgba(255,255,255,.55);--cp-highlight:rgba(177,31,75,.12)}html[data-theme='dark']{color-scheme:dark;--cp-bg:#3d3b3a;--cp-bg-elevated:#343231;--cp-surface:#292929;--cp-surface-soft:#2e2e2e;--cp-border:#474747;--cp-border-strong:#5f5f5f;--cp-text:#dedede;--cp-text-muted:#919191;--cp-text-soft:#b0b0b0;--cp-accent:#fd8ea1;--cp-accent-hover:#fb7b91;--cp-accent-soft:rgba(253,142,161,.14);--cp-accent-fg:#1a1a1a;--cp-success:#4ade80;--cp-danger:#f87171;--cp-warning:#fbbf24;--cp-link:#4da6ff;--cp-shadow:0 18px 48px rgba(0,0,0,.32);--cp-overlay:rgba(41,41,41,.88);--cp-panel:rgba(41,41,41,.72);--cp-panel-strong:rgba(41,41,41,.96);--cp-sheen:rgba(255,255,255,.04);--cp-highlight:rgba(253,142,161,.12)}</style>"
        "<style>body{margin:0;background:var(--cp-bg);color:var(--cp-text);font-family:'Segoe UI',Aptos,Calibri,-apple-system,BlinkMacSystemFont,sans-serif}.wrap{padding:24px}.top{display:flex;justify-content:space-between;gap:16px;align-items:flex-end}.tabs{display:flex;gap:6px;flex-wrap:wrap;margin:16px 0}.tabs button,.bt{border:1px solid var(--cp-border);background:var(--cp-surface);color:var(--cp-text);border-radius:.625rem;padding:8px 12px}.tabs button.act,.bt.primary{background:var(--cp-accent);color:var(--cp-accent-fg);border-color:var(--cp-accent)}.tabs button:hover,.bt:hover{border-color:var(--cp-border-strong)}.tools{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:0 0 12px}.tools input{width:88px}.tools label{color:var(--cp-text-muted)}input{background:var(--cp-surface);color:var(--cp-text);border:1px solid var(--cp-border);border-radius:.625rem;padding:7px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}.cd,.tile{background:var(--cp-surface);border:1px solid var(--cp-border);border-radius:16px;box-shadow:0 0 2px rgba(0,0,0,.12),0 1px 2px rgba(0,0,0,.14);padding:16px;margin:12px 0}.tile{margin:0}.muted{color:var(--cp-text-muted)}.pill{display:inline-block;border:1px solid var(--cp-border);border-radius:.625rem;padding:2px 7px;background:var(--cp-accent-soft)}a{color:var(--cp-link)}pre,code,.nt{font-family:Consolas,'Courier New',Courier,monospace}pre{white-space:pre-wrap;overflow:auto}.nt{width:100%;border-collapse:collapse;font-size:13px}.nt th,.nt td{border-bottom:1px solid var(--cp-border);padding:7px 8px;text-align:left;vertical-align:top}.nt th{color:var(--cp-text-muted);background:var(--cp-surface-soft)}.ok{color:var(--cp-success)}.warn{color:var(--cp-warning)}.bad{color:var(--cp-danger)}.term{background:var(--cp-bg-elevated);color:var(--cp-success);border:1px solid var(--cp-success);border-radius:.625rem;overflow:hidden;font-family:Consolas,'Courier New',Courier,monospace}.bar{background:var(--cp-surface-soft);padding:8px 12px;border-bottom:1px solid var(--cp-border);letter-spacing:.08em}.screen{height:420px;overflow:auto;padding:14px;white-space:pre-wrap;font-size:14px;line-height:1.35}.prompt{display:flex;gap:8px;align-items:center;border-top:1px solid var(--cp-border);background:var(--cp-surface-soft);padding:10px 12px}.prompt input{flex:1;border:0;outline:0;font-family:Consolas,'Courier New',Courier,monospace}.cursor{animation:blink 1s steps(1) infinite}@keyframes blink{50%{opacity:0}}</style></head><body><div class='wrap'><div class='top'><div><h1>PIOS Admin Console</h1><p id='sub' class='muted'>Structured tabs; manual refresh.</p></div><div class='muted'>SECOND STAGE</div></div><div class='tabs'><button class='act' data-t='overview'>Overview</button><button data-t='processes'>Processes</button><button data-t='netstat'>Netstat</button><button data-t='graphs'>Graphs</button><button data-t='system'>System</button><button data-t='users'>Users</button><button data-t='logs'>Logs</button><button data-t='walfs'>WALFS</button><button data-t='firewall'>Firewall</button><button data-t='terminal'>Terminal</button><button data-t='admin'>Admin</button></div><div id='app'></div></div>");
    http_append(out, &len, max,
        "<script>let tab='overview',samples=[],hist=['PIOS remote terminal ready. Type Help for assistance!'],auto=false,ms=3000,timer=0;const app=document.getElementById('app'),sub=document.getElementById('sub');function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}function bust(u){return u+(u.includes('?')?'&':'?')+'_='+Date.now()}async function txt(u){let r=await fetch(bust(u));return await r.text()}async function js(u){let r=await fetch(bust(u));return await r.json()}function card(t,h){return `<div class=cd><h2>${t}</h2>${h}</div>`}function tools(){return `<div class=tools><button class='bt primary' id=rf>Refresh</button><label><input id=ar type=checkbox ${auto?'checked':''}> auto-refresh</label><input id=ms type=number min=250 step=250 value='${ms}'><span class=muted>ms</span></div>`}function bind(){let r=document.getElementById('rf'),a=document.getElementById('ar'),m=document.getElementById('ms');if(r)r.onclick=draw;if(a)a.onchange=()=>{auto=a.checked;sync()};if(m)m.onchange=()=>{ms=Math.max(250,parseInt(m.value||'3000'));m.value=ms;sync()};sync()}function sync(){if(timer){clearInterval(timer);timer=0}if(auto&&tab!=='terminal')timer=setInterval(draw,ms)}function table(h,rows){let b=rows.length?rows.map(r=>'<tr>'+h.map((_,i)=>'<td>'+esc(r[i]||'')+'</td>').join('')+'</tr>').join(''):`<tr><td colspan='${h.length}' class=muted>No rows</td></tr>`;return '<table class=nt><thead><tr>'+h.map(x=>'<th>'+esc(x)+'</th>').join('')+'</tr></thead><tbody>'+b+'</tbody></table>'}function kv(o){return table(['Key','Value'],Object.keys(o).map(k=>[k,o[k]]))}function splitRows(t){return t.trim().split('\\n').filter(x=>x&&x[0]>='0'&&x[0]<='9').map(x=>x.trim().split(/\\s+/))}function spark(a){let m=Math.max(1,...a);return a.map(v=>String(v).padStart(4,' ')+ ' '+ '#'.repeat(Math.min(40,Math.round(v*40/m)))).join('\\n')}async function overview(){let d=await js('/api/status'),dg=d.diag||{};app.innerHTML=card('Overview',tools()+`<div class=grid><div class=tile><b>Build</b><p>${esc(d.build)}</p><span class=pill>${esc(d.version)}</span></div><div class=tile><b>Network</b><p>${esc(d.ip)}</p><p class=muted>${esc(d.mode)}</p></div><div class=tile><b>Uptime</b><p>${esc(d.uptime)} seconds</p></div><div class=tile><b>HTTP diagnostics</b>${kv(dg)}</div></div>`);bind()}async function system(){let r=await txt('/api/terminal?cmd=status'),rows=[];r.split(/\\s+/).forEach(p=>{let i=p.indexOf('=');if(i>0)rows.push([p.slice(0,i),p.slice(i+1)])});app.innerHTML=card('System',tools()+`<div class=grid><div class=tile><b>${esc((r.split('\\n')[0]||'PIOS'))}</b></div><div class=tile>${table(['Metric','Value'],rows)}</div></div><pre>${esc(r)}</pre>`);bind()}async function netstat(){let n=await js('/api/netstat');app.innerHTML=card('Netstat',tools()+table(n.cols,n.rows)+`<p class=muted>syn=${n.diag.syn} accepted=${n.diag.accepted} fw_rx_drop=${n.fw.rxDrop}</p>`);bind()}async function graphs(){let s=await js('/api/status'),n=await js('/api/netstat');samples.push([s.uptime,n.count,n.diag.syn,n.fw.rxDrop]);while(samples.length>48)samples.shift();app.innerHTML=card('Graphs',tools()+table(['uptime','conns','syn','fw_rx_drop'],samples)+`<div class=grid><div class=tile><h3>Connections</h3><pre>${esc(spark(samples.map(x=>x[1])))}</pre></div><div class=tile><h3>Firewall RX drops</h3><pre>${esc(spark(samples.map(x=>x[3])))}</pre></div></div>`);bind()}async function processes(){let r=await txt('/api/terminal?cmd=processes'),p=r.split('\\n\\nGRAPH\\n'),rows=splitRows(p[0]);app.innerHTML=card('Processes',tools()+table(['PID','PPID','Core','State','Pri','CPU','MemK','ArenaCap','ArenaUsed','ArenaHigh','Bump','SpanK','Span#','Image'],rows)+`<h3>Process graph</h3><pre>${esc(p[1]||'')}</pre>`);bind()}async function users(){let r=await txt('/api/terminal?cmd=users');app.innerHTML=card('Users',tools()+table(['ID','Flags','Name'],splitRows(r)));bind()}async function logs(){let a=await txt('/api/admin/log-stream?tail=24'),b=await txt('/api/logs');app.innerHTML=card('Logs',tools()+`<div class=grid><div class=tile><h3>Operator tail</h3><pre>${esc(a)}</pre></div><div class=tile><h3>Process logs</h3><pre>${esc(b)}</pre></div></div>`);bind()}async function walfs(){let d=await js('/api/walfs?path=/'),body='';if(d.entries)body=table(['ID','Name','Size','Flags'],d.entries.map(e=>[e.id,e.name,e.size,e.flags]));else body=kv(d);app.innerHTML=card('WALFS',tools()+body);bind()}async function firewall(){let r=await txt('/api/terminal?cmd=firewall%20list'),rows=[];r.split('\\n').forEach(l=>{let m=l.match(/^(\\d+):\\s+(\\S+)\\s+(\\S+)\\s+(.*)$/);if(m)rows.push([m[1],m[2],m[3],m[4]])});app.innerHTML=card('Firewall',tools()+`<p class=muted>${esc(r.split('\\n')[0]||'')}</p>`+table(['#','Action','Dir','Match'],rows)+`<h3>Mutation examples</h3><pre>firewall allow in tcp port 2323 src 192.168.218.9\\nfirewall deny in tcp port 80 src 192.168.218.0/24\\nfirewall reset</pre>`);bind()}function term(){sync();app.innerHTML=card('Terminal',`<div class=term><div class=bar>PIOS // SECOND STAGE LOADER // REMOTE CONSOLE</div><div class=screen id=screen></div><div class=prompt><span>ready&gt;</span><input id=cmd autocomplete=off spellcheck=false autofocus><span class=cursor>_</span></div></div>`);const sc=document.getElementById('screen'),cmd=document.getElementById('cmd');function paint(){sc.textContent=hist.join('\\n');sc.scrollTop=sc.scrollHeight}async function run(){let c=cmd.value.trim();cmd.value='';if(!c)return;if(c==='clear'){hist=[];paint();return}hist.push('ready> '+c);try{hist.push(await txt('/api/terminal?cmd='+encodeURIComponent(c)))}catch(e){hist.push('ERR '+e)}while(hist.length>160)hist.shift();paint()}cmd.onkeydown=e=>{if(e.key==='Enter')run()};paint();cmd.focus()}function admin(){app.innerHTML=card('Admin',tools()+`<div class=grid><div class=tile><b>Logs</b><p><a href='/api/admin/log-stream?tail=24'>Tail log stream</a></p></div><div class=tile><b>OTA</b><p><a href='/api/admin/kernel-update?confirm=1'>Kernel update status</a></p></div><div class=tile><b>Reboot</b><p><a href='/api/admin/reboot?confirm=1'>Queue hot reboot</a></p></div></div>`);bind()}async function draw(){sub.textContent='Loading '+tab+'...';try{if(tab==='overview')await overview();else if(tab==='system')await system();else if(tab==='netstat')await netstat();else if(tab==='graphs')await graphs();else if(tab==='processes')await processes();else if(tab==='users')await users();else if(tab==='logs')await logs();else if(tab==='walfs')await walfs();else if(tab==='firewall')await firewall();else if(tab==='terminal')term();else if(tab==='admin')admin();else app.innerHTML=card(tab,'unknown tab')}catch(e){app.innerHTML=card('Error',`<pre>${esc(e)}</pre>`)}sub.textContent='Tab '+tab+(auto&&tab!=='terminal'?` // auto ${ms}ms`:'')}document.querySelectorAll('[data-t]').forEach(b=>b.onclick=()=>{tab=b.dataset.t;document.querySelectorAll('[data-t]').forEach(x=>x.classList.toggle('act',x===b));draw()});draw()</script></body></html>");
    return len;
}

static u32 http_build_stats_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    u32 route = http_route_id(req, req_len);
    http_diag.route = route;
    http_trace(HTTP_EVT_BUILD_ENTER, route, req_len, max);
    if (route == HTTP_ROUTE_ACME) {
        len = http_build_acme_challenge_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (HTTP_AUTH_ENABLED && !http_admin_authorized(req, req_len)) {
        http_diag.unauthorized++;
        len = http_build_unauthorized(out, max);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 401);
        return len;
    }

    if (route == HTTP_ROUTE_STATUS) {
        len = http_build_status_json(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_NETSTAT) {
        len = http_build_netstat_json(out, max);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_REBOOT) {
        len = http_build_reboot_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_LOGS) {
        len = http_build_log_stream_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_UPDATE) {
        len = http_build_kernel_update_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_HOTPATCH) {
        len = http_build_hotpatch_kernel_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/terminal")) {
        len = http_build_terminal_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/logs")) {
        len = http_build_logs_response(out, max);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/walfs")) {
        len = http_build_walfs_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/process") &&
        http_query_value(req, req_len, "/api/process", "action", (char[2]){0}, 2)) {
        len = http_build_process_action_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/user") &&
        http_query_value(req, req_len, "/api/user", "action", (char[2]){0}, 2)) {
        len = http_build_user_action_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/process")) {
        static const u8 processes_req[] = "GET /api/terminal?cmd=processes HTTP/1.0\r\n\r\n";
        len = http_build_terminal_response(out, max,
                                           processes_req,
                                           (u32)sizeof(processes_req) - 1U);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/process")) {
        len = http_build_safe_placeholder_response(out, max, "process manager");
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/user")) {
        static const u8 users_req[] = "GET /api/terminal?cmd=users HTTP/1.0\r\n\r\n";
        len = http_build_terminal_response(out, max,
                                           users_req,
                                           (u32)sizeof(users_req) - 1U);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/logs")) {
        len = http_build_safe_placeholder_response(out, max, "log viewer");
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (http_request_path_is(req, req_len, "/api/walfs")) {
        len = http_build_safe_placeholder_response(out, max, "WALFS browser");
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
#if HTTP_ADMIN_EXPERIMENTAL
    if (http_request_path_is(req, req_len, "/api/terminal"))
        return http_build_terminal_response(out, max, req, req_len);
    if (http_request_path_is(req, req_len, "/api/process"))
        return http_build_process_action_response(out, max, req, req_len);
    if (http_request_path_is(req, req_len, "/api/user"))
        return http_build_user_action_response(out, max, req, req_len);
    if (http_request_path_is(req, req_len, "/api/logs"))
        return http_build_logs_response(out, max);
    if (http_request_path_is(req, req_len, "/api/walfs"))
        return http_build_walfs_response(out, max, req, req_len);
#endif

    if (!http_request_is_root_get(req, req_len)) {
        http_diag.not_found++;
        http_append(out, &len, max,
            "HTTP/1.0 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Connection: close\r\n\r\n"
            "PIOS admin console: unknown endpoint.\n");
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, HTTP_ROUTE_NOT_FOUND, len, 404);
        return len;
    }

    (void)len;
    len = http_build_spa_response(out, max);
    http_diag.build_len = len;
    http_trace(HTTP_EVT_BUILD_EXIT, HTTP_ROUTE_ROOT, len, 200);
    return len;
}

static bool http_headers_complete(const u8 *buf, u32 len)
{
    if (!buf || len < 4)
        return false;
    for (u32 i = 0; i + 3 < len; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n' &&
            buf[i + 2] == '\r' && buf[i + 3] == '\n')
            return true;
    }
    return false;
}

static u32 http_header_body_offset(const u8 *buf, u32 len)
{
    if (!buf || len < 4)
        return 0;
    for (u32 i = 0; i + 3 < len; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n' &&
            buf[i + 2] == '\r' && buf[i + 3] == '\n')
            return i + 4;
    }
    return 0;
}

static bool http_method_has_body(const u8 *req, u32 len)
{
    if (!req || len < 4)
        return false;
    return (req[0] == 'P' && req[1] == 'O' && req[2] == 'S' && req[3] == 'T') ||
           (req[0] == 'P' && req[1] == 'U' && req[2] == 'T');
}

static bool http_content_length(const u8 *req, u32 len, u32 *out)
{
    static const char hdr[] = "content-length:";
    if (out) *out = 0;
    for (u32 i = 0; i + sizeof(hdr) - 1 < len; i++) {
        if (i != 0 && req[i - 1] != '\n')
            continue;
        if (!http_match_ci(&req[i], hdr))
            continue;
        i += (u32)sizeof(hdr) - 1;
        while (i < len && (req[i] == ' ' || req[i] == '\t')) i++;
        u32 v = 0;
        bool any = false;
        while (i < len && req[i] >= '0' && req[i] <= '9') {
            any = true;
            v = v * 10U + (u32)(req[i] - '0');
            i++;
        }
        if (out) *out = v;
        return any;
    }
    return false;
}

static bool http_request_complete(const u8 *req, u32 len)
{
    u32 body = http_header_body_offset(req, len);
    http_diag.body_off = body;
    http_diag.content_len = 0;
    http_diag.request_done = 0;
    if (body == 0)
        return false;
    if (!http_method_has_body(req, len)) {
        http_diag.request_done = 1;
        return true;
    }
    u32 content_len = 0;
    if (!http_content_length(req, body, &content_len)) {
        http_diag.request_done = 1;
        return true;
    }
    http_diag.content_len = content_len;
    bool done = len >= body + content_len;
    http_diag.request_done = done ? 1U : 0U;
    return done;
}

static u32 http_build_header_too_large(char *out, u32 max)
{
    u32 len = 0;
    http_append(out, &len, max,
        "HTTP/1.0 431 Request Header Fields Too Large\r\n"
        "Content-Type: text/plain\r\n"
        "Connection: close\r\n\r\n"
        "PIOS admin request headers too large.\n");
    return len;
}

static void http_save_ascii_prefix(char out[25], const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    u32 n = len < 24 ? len : 24;
    for (u32 i = 0; i < n; i++) {
        u8 b = p[i];
        out[i] = (b >= 32 && b < 127) ? (char)b : '.';
    }
    out[n] = 0;
}

static void http_dump_prefix(const char *tag, const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    u32 dump = len < 24 ? len : 24;
    uart_puts("[http] ");
    uart_puts(tag);
    uart_puts(" prefix=");
    for (u32 i = 0; i < dump; i++) {
        static const char hex[] = "0123456789ABCDEF";
        u8 b = p[i];
        uart_putc(hex[b >> 4]);
        uart_putc(hex[b & 0x0F]);
    }
    uart_puts(" ascii=");
    for (u32 i = 0; i < dump; i++) {
        u8 b = p[i];
        uart_putc((b >= 32 && b < 127) ? (char)b : '.');
    }
    uart_puts("\n");
}

static void http_reset_client(bool close_conn)
{
    http_trace(HTTP_EVT_RESET, http_diag.route, close_conn ? 1U : 0U,
               http_client_conn >= 0 ? (u32)http_client_conn : 0xFFFFFFFFU);
    if (http_client_conn >= 0 && close_conn)
        tcp_close(http_client_conn);
    http_client_conn = -1;
    http_req_len = 0;
    http_auth_checked = false;
    http_auth_ok = false;
    http_prefix_dumped = false;
    http_req_prefix_dumped = false;
    http_resp_len = 0;
    http_resp_off = 0;
    http_last_write = 0;
}

static void http_abort_client(void)
{
    http_trace(HTTP_EVT_ABORT, http_diag.route, http_diag.error,
               http_client_conn >= 0 ? (u32)http_client_conn : 0xFFFFFFFFU);
    if (http_client_conn >= 0)
        tcp_abort(http_client_conn);
    tcp_purge_port(HTTP_TCP_PORT);
    http_reset_client(false);
}

static void admin_service_clear_client(struct admin_http_service *svc, bool abort_conn)
{
    if (!svc) return;
    if (svc->client_conn >= 0) {
        if (abort_conn)
            tcp_abort(svc->client_conn);
        else
            tcp_close(svc->client_conn);
    }
    svc->client_conn = -1;
    svc->req_len = 0;
    svc->resp_len = 0;
    svc->resp_off = 0;
}

static void admin_service_restart(struct admin_http_service *svc)
{
    if (!svc) return;
    admin_service_clear_client(svc, true);
    tcp_purge_port(svc->port);
    if (svc->listen_conn >= 0)
        tcp_abort(svc->listen_conn);
    svc->listen_conn = tcp_listen(svc->port);
    svc->last_activity_ms = timer_monotonic_ms();
    svc->last_ok_ms = svc->last_activity_ms;
    http_log_event("admin-restart", svc->port, svc->listen_conn >= 0 ? 1U : 0U);
}

static void admin_service_init(struct admin_http_service *svc)
{
    if (!svc) return;
    svc->listen_conn = tcp_listen(svc->port);
    svc->client_conn = -1;
    svc->req_len = 0;
    svc->resp_len = 0;
    svc->resp_off = 0;
    svc->last_activity_ms = timer_monotonic_ms();
    svc->last_ok_ms = svc->last_activity_ms;
}

static void admin_services_listen(void)
{
    admin_service_init(&admin_status_svc);
    admin_service_init(&admin_reboot_svc);
    admin_service_init(&admin_update_svc);
}

static u32 admin_build_status_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    if (http_request_path_is(req, req_len, "/api/admin/log-stream") ||
        http_request_path_is(req, req_len, "/logs"))
        return http_build_log_stream_response(out, max, req, req_len);
    if (http_request_path_is(req, req_len, "/api/status"))
        return http_build_status_json(out, max, req, req_len);

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
        "PIOS status ok\nversion=");
    http_append(out, &len, max, PIOS_VERSION);
    http_append(out, &len, max, "\nbuild=");
    http_append(out, &len, max, PIOS_BUILD_LABEL);
    http_append(out, &len, max, "\nuptime=");
    http_append_u64(out, &len, max, timer_monotonic_ms() / 1000ULL);
    http_append(out, &len, max, "\nlogs=/logs\nupdate_port=");
    http_append_u64(out, &len, max, ADMIN_UPDATE_TCP_PORT);
    http_append(out, &len, max, "\nreboot_port=");
    http_append_u64(out, &len, max, ADMIN_REBOOT_TCP_PORT);
    http_append(out, &len, max, "\n");
    return len;
}

static u32 admin_build_reboot_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    bool confirmed = http_confirmed(req, req_len, "/");
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    if (!confirmed) {
        http_append(out, &len, max, "reboot requires ?confirm=1\n");
        return len;
    }
    http_reboot_pending = true;
    http_log_event("reboot-queued", ADMIN_REBOOT_TCP_PORT, 0);
    http_append(out, &len, max, "reboot queued\n");
    return len;
}

static u32 admin_build_update_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    return http_build_kernel_update_response(out, max, req, req_len);
}

static void admin_service_build_response(struct admin_http_service *svc)
{
    u32 route = http_route_id(svc->req, svc->req_len);
    http_trace(HTTP_EVT_ADMIN_BUILD, route, svc->port, svc->req_len);
    if (svc == &admin_status_svc)
        svc->resp_len = admin_build_status_response(svc->resp, sizeof(svc->resp),
                                                    svc->req, svc->req_len);
    else if (svc == &admin_reboot_svc)
        svc->resp_len = admin_build_reboot_response(svc->resp, sizeof(svc->resp),
                                                    svc->req, svc->req_len);
    else
        svc->resp_len = admin_build_update_response(svc->resp, sizeof(svc->resp),
                                                    svc->req, svc->req_len);
    http_diag.build_len = svc->resp_len;
    svc->resp_off = 0;
}

static void admin_service_poll(struct admin_http_service *svc)
{
    if (!svc) return;
    u64 now = timer_monotonic_ms();
    if (svc->listen_conn < 0 ||
        (svc->client_conn >= 0 && now - svc->last_activity_ms > ADMIN_CLIENT_STALL_MS) ||
        (now - svc->last_ok_ms > ADMIN_SERVICE_WATCHDOG_MS && svc->completions > 0)) {
        admin_service_restart(svc);
    }

    if (svc->client_conn < 0 && svc->listen_conn >= 0) {
        svc->client_conn = tcp_accept(svc->listen_conn);
        if (svc->client_conn >= 0) {
            svc->req_len = 0;
            svc->resp_len = 0;
            svc->resp_off = 0;
            svc->last_activity_ms = now;
            http_log_event("admin-accept", svc->port, 0);
            http_trace(HTTP_EVT_ACCEPT, HTTP_ROUTE_UNKNOWN, svc->port, (u32)svc->client_conn);
        }
    }
    if (svc->client_conn < 0)
        return;

    u32 st = tcp_state(svc->client_conn);
    if (st != TCP_ESTABLISHED) {
        if (st == TCP_CLOSED || st >= TCP_CLOSING || now - svc->last_activity_ms > 1000ULL)
            admin_service_clear_client(svc, st != TCP_CLOSED);
        return;
    }

    if (svc->resp_len == 0) {
        u32 readable = tcp_readable(svc->client_conn);
        if (readable > 0 && svc->req_len < sizeof(svc->req)) {
            u32 want = sizeof(svc->req) - svc->req_len;
            if (want > readable) want = readable;
            u32 n = tcp_read(svc->client_conn, svc->req + svc->req_len, want);
            svc->req_len += n;
            svc->last_activity_ms = now;
            http_trace(HTTP_EVT_RX, http_route_id(svc->req, svc->req_len), svc->port, svc->req_len);
        }
        if (http_request_complete(svc->req, svc->req_len) ||
            svc->req_len >= sizeof(svc->req)) {
            http_trace(HTTP_EVT_COMPLETE, http_route_id(svc->req, svc->req_len), svc->port, svc->req_len);
            admin_service_build_response(svc);
            svc->last_activity_ms = now;
        }
    }

    if (svc->resp_len > 0 && svc->resp_off < svc->resp_len) {
        u32 writable = tcp_writable(svc->client_conn);
        if (writable > 0) {
            u32 remain = svc->resp_len - svc->resp_off;
            u32 chunk = remain < writable ? remain : writable;
            if (chunk > 512) chunk = 512;
            u32 n = tcp_write(svc->client_conn, svc->resp + svc->resp_off, chunk);
            if (svc->resp_off == 0)
                http_trace(HTTP_EVT_TX, http_route_id(svc->req, svc->req_len), svc->port, svc->resp_len);
            svc->resp_off += n;
            if (n > 0)
                svc->last_activity_ms = now;
        }
    }

    if (svc->resp_len > 0 && svc->resp_off >= svc->resp_len) {
        svc->completions++;
        svc->last_ok_ms = now;
        http_log_event("admin-complete", svc->port, svc->completions);
        http_trace(HTTP_EVT_CLOSE, http_route_id(svc->req, svc->req_len), svc->port, svc->resp_len);
        if (http_reboot_pending) {
            timer_delay_ms(250);
            watchdog_reboot_now(0x52454254U);
        }
        admin_service_clear_client(svc, false);
    }
}

static void admin_services_poll(void)
{
    admin_service_poll(&admin_status_svc);
    admin_service_poll(&admin_reboot_svc);
    admin_service_poll(&admin_update_svc);
}

static void http_render_diag(void)
{
#if !HTTP_DIAG_VERBOSE
    return;
#endif
    u32 row = fb_reserved_rows();
    if (row < 5) row = 5;
    row -= 4;
    fb_set_cursor(0, row);
    fb_set_color(0x00FFAA00, 0x00000000);
    fb_puts("HTTP open=");
    fb_printf("%u", http_client_conn >= 0 ? 1U : 0U);
    fb_puts(" st=");
    fb_printf("%u", http_last_state);
    fb_puts(" acc=");
    fb_printf("%u", (u32)http_diag.accepts);
    fb_puts(" tick=");
    fb_printf("%u", http_complete_tick);
    fb_puts(" close=");
    fb_printf("%u", (u32)http_diag.closes);
    fb_puts(" abort=");
    fb_printf("%u", (u32)http_diag.aborts);
    fb_puts(" ev=");
    fb_printf("%u", http_diag.event);
    fb_puts(" rt=");
    fb_printf("%u", http_diag.route);
    fb_puts(" err=");
    fb_printf("%u", http_diag.error);
    fb_puts("                              ");
    fb_set_cursor(0, row + 1);
    fb_puts("HTTP in-flight req=");
    fb_printf("%u", http_req_len);
    fb_puts(" queued=");
    fb_printf("%u", http_resp_off);
    fb_putc('/');
    fb_printf("%u", http_resp_len);
    fb_puts(" rd=");
    fb_printf("%u", http_last_readable);
    fb_puts(" wr=");
    fb_printf("%u", http_last_writable);
    fb_puts(" last=");
    fb_printf("%u", http_last_write);
    fb_puts(" calls=");
    fb_printf("%u", (u32)http_diag.write_calls);
    fb_puts(" done=");
    fb_printf("%u", http_diag.request_done);
    fb_puts("                              ");
    fb_set_cursor(0, row + 2);
    fb_puts("HTTP complete ");
    fb_putc((http_complete_tick & 1U) ? '/' : '\\');
    fb_puts(" bytes=");
    fb_printf("%u", (u32)http_diag.write_bytes);
    fb_puts(" zero=");
    fb_printf("%u", (u32)http_diag.write_zero);
    fb_puts(" body=");
    fb_printf("%u", http_diag.body_off);
    fb_puts(" clen=");
    fb_printf("%u", http_diag.content_len);
    fb_puts(" blen=");
    fb_printf("%u", http_diag.build_len);
    fb_puts("                              ");
    fb_set_cursor(0, row + 3);
    fb_puts("HTTP req='");
    fb_puts(http_last_req_prefix);
    fb_puts("' resp='");
    fb_puts(http_last_resp_prefix);
    fb_puts("'                              ");
}

/* HTTP/TLS poll — called from core0 service loop */
static void echo_tcp_poll(void) {
    /* Kernel TLS test server on port 443. This uses PIOS's kernel TLS-style
     * record wrapper, not a browser-compatible X.509 TLS endpoint yet. */
    if (https_tls_tcp_conn < 0 && https_tls_listen_conn >= 0) {
        https_tls_tcp_conn = tcp_accept(https_tls_listen_conn);
        if (https_tls_tcp_conn >= 0) {
            https_tls_conn = -1;
            https_tls_req_len = 0;
            https_tls_accepted = false;
            https_tls_response_sent = false;
            https_tls_last_activity_ms = timer_monotonic_ms();
            http_log_event("tls443-accept", HTTPS_TLS_TCP_PORT, (u32)https_tls_tcp_conn);
        }
    }
    if (https_tls_tcp_conn >= 0) {
        u32 st = tcp_state(https_tls_tcp_conn);
        if (st == TCP_CLOSED || st == TCP_CLOSE_WAIT || st >= TCP_CLOSING) {
            if (https_tls_conn >= 0) tls_close(https_tls_conn);
            else tcp_close(https_tls_tcp_conn);
            https_tls_tcp_conn = -1;
            https_tls_conn = -1;
        } else if (st == TCP_ESTABLISHED) {
            if (!https_tls_accepted) {
                if (tcp_readable(https_tls_tcp_conn) > 0) {
                    https_tls_conn = tls_accept(https_tls_tcp_conn);
                    if (https_tls_conn < 0) {
                        http_log_event("tls443-handshake-fail", HTTPS_TLS_TCP_PORT, (u32)https_tls_tcp_conn);
                        tcp_close(https_tls_tcp_conn);
                        https_tls_tcp_conn = -1;
                    } else {
                        https_tls_accepted = true;
                        https_tls_last_activity_ms = timer_monotonic_ms();
                        http_log_event("tls443-handshake-ok", HTTPS_TLS_TCP_PORT, (u32)https_tls_conn);
                    }
                } else if ((timer_monotonic_ms() - https_tls_last_activity_ms) > 5000ULL) {
                    tcp_close(https_tls_tcp_conn);
                    https_tls_tcp_conn = -1;
                }
            } else if (!https_tls_response_sent) {
                if (tcp_readable(https_tls_tcp_conn) > 0 && https_tls_req_len == 0) {
                    i32 rn = tls_read(https_tls_conn, https_tls_req_buf, sizeof(https_tls_req_buf));
                    if (rn > 0) https_tls_req_len = (u32)rn;
                }
                static const char resp[] =
                    "HTTP/1.0 200 OK\r\n"
                    "Content-Type: text/plain\r\n"
                    "Content-Length: 20\r\n"
                    "Connection: close\r\n\r\n"
                    "PIOS kernel TLS 443\n";
                if (tls_write(https_tls_conn, resp, sizeof(resp) - 1) > 0) {
                    https_tls_response_sent = true;
                    https_tls_last_activity_ms = timer_monotonic_ms();
                    http_log_event("tls443-response", HTTPS_TLS_TCP_PORT, sizeof(resp) - 1);
                    tls_close(https_tls_conn);
                    https_tls_tcp_conn = -1;
                    https_tls_conn = -1;
                }
            } else if ((timer_monotonic_ms() - https_tls_last_activity_ms) > 1000ULL) {
                tls_close(https_tls_conn);
                https_tls_tcp_conn = -1;
                https_tls_conn = -1;
            }
        }
    }

    /* HTTP server on port 80 — returns simple status page */
    if (http_client_conn < 0 && http_listen_conn >= 0) {
        http_client_conn = tcp_accept(http_listen_conn);
        if (http_client_conn >= 0) {
            http_diag.accepts++;
            http_diag.conn = (u32)http_client_conn;
            http_diag.route = HTTP_ROUTE_UNKNOWN;
            http_diag.error = HTTP_ERR_NONE;
            http_trace(HTTP_EVT_ACCEPT, HTTP_ROUTE_UNKNOWN, (u32)http_client_conn, http_diag.accepts);
            http_req_len = 0;
            http_auth_checked = false;
            http_auth_ok = false;
            http_resp_len = 0;
            http_resp_off = 0;
            http_last_write = 0;
            http_prefix_dumped = false;
            http_req_prefix_dumped = false;
            http_last_req_prefix[0] = 0;
            http_last_resp_prefix[0] = 0;
            http_last_activity_ms = timer_monotonic_ms();
            if (HTTP_DIAG_VERBOSE) uart_puts("[http] accepted\n");
        }
    }
    if (http_client_conn >= 0) {
        u32 st = tcp_state(http_client_conn);
        http_last_state = st;
        http_last_readable = tcp_readable(http_client_conn);
        http_last_writable = tcp_writable(http_client_conn);
        if (st == TCP_CLOSED || st == TCP_CLOSE_WAIT || st >= TCP_CLOSING) {
            http_diag.closes++;
            http_trace(HTTP_EVT_CLOSE, http_diag.route, http_resp_len, http_resp_off);
            http_reset_client(st != TCP_CLOSED);
        } else if (st == TCP_ESTABLISHED) {
            if (http_resp_len == 0) {
                u32 readable = tcp_readable(http_client_conn);
                if (readable > 0 && http_req_len < sizeof(http_req_buf)) {
                    u32 want = sizeof(http_req_buf) - http_req_len;
                    if (want > readable) want = readable;
                    u32 n = tcp_read(http_client_conn, http_req_buf + http_req_len, want);
                    http_req_len += n;
                    http_diag.reads++;
                    http_last_activity_ms = timer_monotonic_ms();
                    http_trace(HTTP_EVT_RX, http_diag.route, n, http_req_len);
                    if (HTTP_DIAG_VERBOSE) {
                        uart_puts("[http] request bytes=");
                        uart_hex(n);
                        uart_puts(" total=");
                        uart_hex(http_req_len);
                        uart_puts("\n");
                    }
                    if (!http_req_prefix_dumped) {
                        http_req_prefix_dumped = true;
                        http_save_ascii_prefix(http_last_req_prefix, http_req_buf, http_req_len);
                        if (HTTP_DIAG_VERBOSE) http_dump_prefix("rx", http_req_buf, http_req_len);
                    }

                    if (HTTP_AUTH_ENABLED && !http_auth_checked) {
                        bool auth_complete = false;
                        bool auth_ok = false;
                        if (http_try_early_auth(http_req_buf, http_req_len, &auth_complete, &auth_ok) &&
                            auth_complete) {
                            http_auth_checked = true;
                            http_auth_ok = auth_ok;
                            if (HTTP_DIAG_VERBOSE) uart_puts(auth_ok ? "[http] auth ok early\n" : "[http] auth failed early\n");
                            if (!auth_ok) {
                                http_diag.unauthorized++;
                                http_resp_len = http_build_unauthorized(http_resp_buf, sizeof(http_resp_buf));
                                http_diag.built++;
                            }
                        }
                    }
                }

                if (HTTP_SIMPLE_MODE && http_resp_len == 0 && http_req_len > 0) {
                    static const char simple[] =
                        "HTTP/1.0 200 OK\r\n"
                        "Content-Type: text/plain\r\n"
                        "Content-Length: 8\r\n"
                        "Connection: close\r\n\r\n"
                        "PIOS OK\n";
                    http_resp_len = 0;
                    http_append(http_resp_buf, &http_resp_len, sizeof(http_resp_buf), simple);
                    http_diag.built++;
                    if (HTTP_DIAG_VERBOSE) {
                        uart_puts("[http] simple response len=");
                        uart_hex(http_resp_len);
                        uart_puts("\n");
                    }
                } else if (http_resp_len == 0 && http_request_complete(http_req_buf, http_req_len)) {
                    http_diag.route = http_route_id(http_req_buf, http_req_len);
                    http_trace(HTTP_EVT_COMPLETE, http_diag.route, http_req_len, http_diag.body_off);
                    http_resp_len = http_build_stats_response(http_resp_buf, sizeof(http_resp_buf), http_req_buf, http_req_len);
                    http_diag.built++;
                    if (HTTP_DIAG_VERBOSE) {
                        uart_puts("[http] response len=");
                        uart_hex(http_resp_len);
                        uart_puts("\n");
                    }
                } else if (http_req_len >= sizeof(http_req_buf)) {
                    http_diag.error = HTTP_ERR_HEADER_BIG;
                    http_trace(HTTP_EVT_COMPLETE, HTTP_ROUTE_UNKNOWN, http_req_len, sizeof(http_req_buf));
                    http_resp_len = http_build_header_too_large(http_resp_buf, sizeof(http_resp_buf));
                    http_diag.built++;
                    if (HTTP_DIAG_VERBOSE) uart_puts("[http] header too large\n");
                }
            }

            if (http_resp_len > 0 && http_resp_off < http_resp_len) {
                u32 writable = tcp_writable(http_client_conn);
                u32 remain = http_resp_len - http_resp_off;
                if (writable > 0) {
                    u32 chunk = remain < writable ? remain : writable;
                    if (chunk > 512) chunk = 512;
                    if (!http_prefix_dumped) {
                        http_prefix_dumped = true;
                        http_save_ascii_prefix(http_last_resp_prefix, http_resp_buf, http_resp_len);
                        http_trace(HTTP_EVT_TX, http_diag.route, http_resp_len, writable);
                        if (HTTP_DIAG_VERBOSE) http_dump_prefix("tx", http_resp_buf, http_resp_len);
                    }
                    u32 n = tcp_write(http_client_conn, http_resp_buf + http_resp_off, chunk);
                    http_diag.write_calls++;
                    http_last_write = n;
                    if (n == 0) {
                        http_diag.write_zero++;
                    } else {
                        http_diag.write_bytes += n;
                        http_last_activity_ms = timer_monotonic_ms();
                    }
                    http_resp_off += n;
                }
            }

            if (http_resp_len > 0 && http_resp_off >= http_resp_len) {
                http_diag.closes++;
                http_complete_tick++;
                http_trace(HTTP_EVT_CLOSE, http_diag.route, http_resp_len, http_resp_off);
                if (HTTP_DIAG_VERBOSE) uart_puts("[http] close complete\n");
                if (http_reboot_pending) {
                    timer_delay_ms(250);
                    watchdog_reboot_now(0x48545450U);
                }
                http_reset_client(true);
            } else if (http_resp_len > 0 && (timer_monotonic_ms() - http_last_activity_ms) > 15000ULL) {
                http_diag.aborts++;
                http_diag.error = HTTP_ERR_RESP_TIMEOUT;
                uart_puts("[http] abort timeout st=");
                uart_hex(st);
                uart_puts(" off=");
                uart_hex(http_resp_off);
                uart_puts(" len=");
                uart_hex(http_resp_len);
                uart_puts(" writable=");
                uart_hex(http_last_writable);
                uart_puts("\n");
                http_abort_client();
            } else if (http_resp_len == 0 && (timer_monotonic_ms() - http_last_activity_ms) > 3000ULL) {
                http_diag.aborts++;
                http_diag.error = HTTP_ERR_REQ_TIMEOUT;
                http_abort_client();
            }
        } else {
            http_diag.aborts++;
            http_diag.error = HTTP_ERR_BAD_STATE;
            http_trace(HTTP_EVT_BAD_STATE, http_diag.route, st, http_last_activity_ms ? (u32)(timer_monotonic_ms() - http_last_activity_ms) : 0);
            if ((timer_monotonic_ms() - http_last_activity_ms) > 1000ULL ||
                st == 0 || st >= 8)
                http_abort_client();
        }
        http_render_diag();
    }
    admin_services_poll();
}

#define UI_MODE_NONE          0
#define UI_MODE_PROC_VIEW     1
#define UI_MODE_PROC_MANAGER  2
#define UI_MODE_CONSOLE       3
#define UI_MODE_SCHEDULER     4
#define UI_SNAPSHOT_MAX       (MAX_PROCS_PER_CORE + 1U)
#define UI_CONSOLE_LINE_MAX   256
#define UI_CONSOLE_ARGV_MAX   16
#define UI_STREAM_IN_MAX      1472
#define UI_STREAM_OUT_MAX     2048
#define UI_ENV_MAX            32
#define UI_BATCH_MAX          32
#define UI_DB_UDP_VER         1
#define UI_SHELL_TEXT_COLOR   0x00FF88CC   /* pink on purple terminal */
#define UI_SHELL_BG_COLOR    0x006600AA   /* purple terminal background */
#define UI_EDIT_MAX_LINES     128
#define UI_EDIT_LINE_MAX      120

static u32 ui_mode;
static u32 ui_selected;
static u64 ui_last_render;
static i32 ui_launch_idx;
static i32 ui_status_code;
static char ui_console_line[UI_CONSOLE_LINE_MAX];
static u32 ui_console_len;
static char ui_sched_line[UI_CONSOLE_LINE_MAX];
static u32 ui_sched_len;
static char ui_cwd[256] = "/";
static u32 ui_cfg_ip;
static u32 ui_cfg_mask;
static u32 ui_cfg_gw;
static u32 ui_cfg_dns;
static bool ui_cfg_dhcp;
static const char *ui_proc_state_str(u32 s);
static void ui_dump_sector(u32 lba);
static void ui_cmd_fsinspect(const char *path);
static void ui_cmd_netcfg(u32 argc, char **argv);
static void ui_cmd_firewall(u32 argc, char **argv);
static void ui_cmd_usb(u32 argc, char **argv);
static void ui_cmd_disk(u32 argc, char **argv);
static void ui_cmd_db(u32 argc, char **argv);
static void ui_cmd_lsdir(const char *path);
static void ui_cmd_mkdir(const char *path);
static void ui_cmd_touch(const char *path);
static void ui_cmd_copy(const char *src, const char *dst);
static void ui_cmd_cpdir(const char *src, const char *dst);
static bool ui_path_resolve(const char *in, char *out, u32 out_max);
static void ui_cmd_cat(const char *path);
static void ui_cmd_stat(const char *path);
static void ui_cmd_rm(const char *path);
static void ui_cmd_stream(u32 argc, char **argv);
static void ui_cmd_mv(const char *src, const char *dst);
static void ui_cmd_hexdump(const char *path, u32 max_bytes);
static void ui_cmd_find(const char *base, const char *needle);
static void ui_cmd_df(void);
static void ui_cmd_mount(u32 argc, char **argv);
static void ui_cmd_env(u32 argc, char **argv);
static void ui_cmd_if(u32 argc, char **argv);
static void ui_cmd_for(u32 argc, char **argv);
static void ui_cmd_foreach(u32 argc, char **argv);
static void ui_cmd_source(const char *path);
static void ui_cmd_edit(const char *path);
static void ui_cmd_capsule(u32 argc, char **argv);
static void ui_cmd_obs(u32 argc, char **argv);
static void ui_cmd_update(u32 argc, char **argv);
static void ui_cmd_watchdog(u32 argc, char **argv);
static void ui_cmd_bootctrl(u32 argc, char **argv);
static void ui_cmd_dma(u32 argc, char **argv);
static void ui_cmd_addr(u32 argc, char **argv);
static void ui_cmd_keystore(u32 argc, char **argv);
static void ui_cmd_tls(u32 argc, char **argv);
static void ui_cmd_brotli(u32 argc, char **argv);
static void ui_cmd_x509(u32 argc, char **argv);
static void ui_cmd_acme(u32 argc, char **argv);
static void ui_cmd_ksvc(u32 argc, char **argv);
static void ui_cmd_irq(u32 argc, char **argv);
static void ui_cmd_abi(u32 argc, char **argv);
static void ui_cmd_tensor(u32 argc, char **argv);
static void ui_console_exec(char *line);
static bool ui_parse_priority(const char *s, u32 *out_prio);
static const char *ui_priority_str(u32 p);
static void ui_cmd_dns(u32 argc, char **argv);
static void ui_cmd_mem(u32 argc, char **argv);
static void ui_console_u32_dec(u32 v);
static void ui_console_u64_dec(u64 v);
static void ui_console_hex_fixed(u64 v, u32 digits);
static bool ui_resolve_pis_path(const char *in, char *out, u32 out_max);
static bool ui_resolve_pix_path(const char *in, char *out, u32 out_max);
static bool ui_resolve_job_path(const char *in, char *out, u32 out_max, bool *is_script_out);
static void ui_cmd_batch(u32 argc, char **argv);
static void ui_batch_tick(void);
static void ui_cmd_svc(u32 argc, char **argv);
static void ui_service_tick(void);
static void ui_render_scheduler(void);
static void ui_scheduler_feed_char(i32 c);
static void disk_handle_request(u32 from_core);
static void ui_db_udp_cb(u32 src_ip, u16 src_port, u16 dst_port, const u8 *data, u16 len);
static bool ui_env_get(const char *key, const char **val_out);
static void ui_env_set(const char *key, const char *val, bool persistent);
static bool ui_env_save(void);
static bool ui_env_load(void);
static u32 ui_read_tty_line(char *out, u32 out_max, const char *prompt);
static bool ui_cap_manifest_validate_buf(const char *buf, u32 n, char *err, u32 err_max);

extern u8 __text_start;
extern u8 __text_end;
extern u8 __el2_integrity_start;
extern u8 __el2_integrity_end;

#define BOOT_POLICY_MAGIC   0x42504C59U /* BPLY */
#define BOOT_POLICY_VERSION 1U
#define BOOT_POLICY_CARD    0U
#define BOOT_POLICY_REC     10U
#define BOOT_ROLLBACK_REC   11U
#define BOOT_UPDATE_REC     12U
#define BOOT_UPDATE_MAGIC   0x42555044U /* BUPD */
#define BOOT_UPDATE_VERSION 1U

struct boot_policy_record {
    u32 magic;
    u32 version;
    u32 el1_hash;
    u32 el2_hash;
    u8 mac[32];
} PACKED;

struct boot_update_record {
    u32 magic;
    u32 version;
    u32 active_slot;
    u32 pending_slot;
    u32 previous_slot;
    u32 tries_left;
    u32 committed;
    u32 generation;
    u8 mac[32];
} PACKED;

static const u8 boot_policy_hmac_key[32] = {
    0x79,0xA1,0x34,0x5D,0x9C,0xE2,0x11,0x6B,0x43,0x88,0x2F,0xC0,0x7A,0xD3,0x59,0xBE,
    0x10,0x4F,0x92,0xCC,0x61,0x2A,0xE7,0x35,0xB9,0x08,0xF1,0x6D,0x54,0xAB,0x3E,0xC7
};

static u32 boot_el1_expected_hash;
static u32 boot_el2_expected_hash;
static struct boot_update_record g_boot_update_state;

#define UI_SVC_MAX 16
#define UI_SVC_TARGET_DEFAULT 1U
#define UI_SVC_TARGET_RESCUE  2U
#define UI_SVC_TARGET_ALL     3U
#define UI_SVC_RP_NEVER       0U
#define UI_SVC_RP_ONFAIL      1U
#define UI_SVC_RP_ALWAYS      2U

struct ui_service_unit {
    bool used;
    char name[32];
    char path[128];
    char depends[32];
    u32 target;
    u32 preferred_core;
    u32 principal_id;
    u32 priority_class;
    u32 restart_policy;
    u32 max_restarts;
    u32 backoff_ms;
    u32 state; /* 0 stopped, 1 running, 2 backoff, 3 failed */
    bool stop_requested;
    i32 pid;
    u32 restarts;
    u64 window_start_ms;
    u64 next_action_ms;
};
static struct ui_service_unit ui_services[UI_SVC_MAX];
static bool ui_service_running;
static u32 ui_service_target = UI_SVC_TARGET_DEFAULT;

struct ui_batch_job {
    bool used;
    u32 id;
    char path[128];
    u32 preferred_core; /* 0=auto, 1|2|3 fixed */
    u32 principal_id;   /* principal to run as */
    u32 priority_class; /* PROC_PRIO_* */
    u32 state;          /* 0 queued,1 running,2 done,3 failed,4 canceled */
    i32 pid;
    u32 attempts;
    u32 retries;
    i32 last_err;
    u64 next_due_ms;
    u32 interval_ms;    /* 0 one-shot, >0 recurring */
    bool is_script;
};
static struct ui_batch_job ui_batch_jobs[UI_BATCH_MAX];
static u32 ui_batch_next_id = 1;
static u32 ui_batch_parallel = 2;
static bool ui_batch_running;

struct ui_env_var {
    char key[32];
    char val[128];
    bool used;
    bool persistent;
};
static struct ui_env_var ui_env[UI_ENV_MAX];
static bool ui_env_loaded;
static u32 ui_script_depth;

struct ui_stream_udp_rx {
    bool waiting;
    bool ready;
    u32 src_ip;
    u16 src_port;
    u16 len;
    u8 data[1472];
};
static struct ui_stream_udp_rx ui_stream_udp;
static bool ui_act_led_ready;
static bool ui_act_led_level;
static u64 ui_act_led_next_tick;

static const char *const ui_launch_candidates[] = {
    "/bin/console.pix",
    "/bin/hexview.pix",
    "/bin/init.pix",
    "/bin/shell.pix",
    "/bin/edit.pix",
    "/bin/demo.pix",
    "/app/main.pix",
};

static void ui_act_led_init(void)
{
    /* Pi 5 ACT LED is driven from RP1 GPIO42 on most boards (active-low). */
    rp1_gpio_set_function(42, RP1_FSEL_GPIO);
    rp1_gpio_set_dir_output(42);
    ui_act_led_level = false;
    rp1_gpio_write(42, true);
    ui_act_led_ready = true;
    ui_act_led_next_tick = timer_ticks() + 250ULL;
}

static void ui_act_led_tick(void)
{
    if (!ui_act_led_ready) return;
    u64 now = timer_ticks();
    if (now < ui_act_led_next_tick) return;
    ui_act_led_next_tick = now + 250ULL;
    ui_act_led_level = !ui_act_led_level;
    /* Active-low LED: false means on. */
    rp1_gpio_write(42, ui_act_led_level ? true : false);
}

static void boot_measurements(u32 *el1_hash, u32 *el2_hash, u64 *el1_start, u32 *el1_len)
{
    u64 el1_s = (u64)(usize)&__text_start;
    u64 el1_e = (u64)(usize)&__text_end;
    u64 el2_s = (u64)(usize)&__el2_integrity_start;
    u64 el2_e = (u64)(usize)&__el2_integrity_end;
    if (el1_e <= el1_s) {
        uart_puts("[bt] EL1 text empty, skip\n");
        if (el1_hash) *el1_hash = 0;
        if (el2_hash) *el2_hash = 0;
        if (el1_start) *el1_start = 0;
        if (el1_len) *el1_len = 0;
        return;
    }
    if (el1_hash) *el1_hash = hw_crc32c((const void *)(usize)el1_s, (u32)(el1_e - el1_s));
    if (el2_s < el2_e) {
        if (el2_hash) *el2_hash = hw_crc32c((const void *)(usize)el2_s, (u32)(el2_e - el2_s));
    } else {
        uart_puts("[bt] EL2 integ empty, skip\n");
        if (el2_hash) *el2_hash = 0;
    }
    if (el1_start) *el1_start = el1_s;
    if (el1_len) *el1_len = (u32)(el1_e - el1_s);
}

/*
 * ── PIOS Boot Colour Scheme ──────────────────────────────────────────
 *
 * Each boot phase has a coloured swatch on the HDMI progress display.
 * If boot hangs, the last phase showing [..] identifies the failure.
 * Completed phases show [OK]; failed phases show [!!].
 *
 * Phase  Colour   Hex (RRGGBB)  Subsystem
 * ─────  ───────  ───────────   ──────────────────────────────────────
 *   0    Black    0x00222222    Firmware handed off to kernel (pre-fb)
 *   1    Green    0x0000AA00    VideoCore framebuffer + HDMI online
 *   2    Pink     0x00FF55AA    PCIe root complex + RP1 southbridge
 *   3    Red      0x00CC0000    USB (xHCI) + UART / TTY online
 *   4    Grey     0x00555555    Filesystem (SD + WALFS) online
 *   5    Blue     0x001144CC    NIC / MAC (Cadence MACB/GEM Ethernet) online
 *   6    Yellow   0x00CCAA00    Multicore — secondary cores started
 *   7    Purple   0x006600AA    PIOS operational and ready
 *
 * Final terminal state: pink text (0x00FF88CC) on purple background.
 * ──────────────────────────────────────────────────────────────────────
 */
#define BOOT_BLACK      0x00000000
#define BOOT_GREEN      0x0000AA00
#define BOOT_PINK       0x00FF55AA
#define BOOT_RED        0x00CC0000
#define BOOT_GREY       0x00555555
#define BOOT_BLUE       0x001144CC
#define BOOT_YELLOW     0x00CCAA00
#define BOOT_PURPLE     0x006600AA
#define BOOT_FG_WHITE   0x00FFFFFF
#define BOOT_FG_DARK    0x00000000
#define BOOT_FG_PINK    0x00FF88CC
#define BOOT_FG_DIM     0x00555555
#define BOOT_FG_LOG     0x00999999
#define BOOT_FG_OK      0x0000CC00
#define BOOT_FG_FAIL    0x00FF2200

/* Boot progress display — 8 phases with coloured swatches + scrolling log */
#define BP_COUNT     8
#define BP_LIST_ROW  3      /* first phase at text row 3 */
#define BP_STAT_COL  36     /* [OK]/[..] column */

static const u32 bp_colors[BP_COUNT] = {
    0x00222222, BOOT_GREEN, BOOT_PINK, BOOT_RED,
    BOOT_GREY, BOOT_BLUE, BOOT_YELLOW, BOOT_PURPLE,
};
static const char *bp_names[BP_COUNT] = {
    "Firmware Handoff",
    "VideoCore HDMI",
    "PCIe + RP1 Southbridge",
    "USB + UART / TTY",
    "Filesystem (SD + WALFS)",
    "NIC / MAC (MACB/GEM)",
    "Multicore",
    "PIOS Ready",
};

static u32 bp_log_y;  /* next row for boot log messages */

static void bp_uart_phase(u32 phase, const char *state)
{
    if (phase >= BP_COUNT) return;
    uart_puts("[bt] ");
    uart_puts(bp_names[phase]);
    uart_puts(" ");
    uart_puts(state);
    uart_puts("\n");
}

static void bp_uart_line(const char *prefix, const char *msg)
{
    uart_puts(prefix);
    uart_puts(msg);
    uart_puts("\n");
}

/* Draw initial progress screen: header + phase list (all dim/pending) */
static void bp_init(void) {
    fb_clear(BOOT_BLACK);
    fb_set_cursor(0, 0);
    fb_set_color(BOOT_FG_PINK, BOOT_BLACK);
    fb_puts("PIOS ");
    fb_puts(PIOS_BUILD_LABEL);
    fb_puts(" Boot Sequence\n");
    fb_set_color(0x00444444, BOOT_BLACK);
    fb_puts("=============================================\n");

    for (u32 i = 0; i < BP_COUNT; i++) {
        fb_set_cursor(0, BP_LIST_ROW + i);
        fb_set_color(BOOT_FG_WHITE, BOOT_BLACK);
        fb_putc(' ');
        /* 2-char coloured swatch (space glyphs fill with bg colour) */
        fb_set_color(bp_colors[i], bp_colors[i]);
        fb_puts("  ");
        /* Phase name — dim until active */
        fb_set_color(BOOT_FG_DIM, BOOT_BLACK);
        fb_putc(' ');
        fb_puts(bp_names[i]);
    }

    /* Separator before log area */
    fb_set_cursor(0, BP_LIST_ROW + BP_COUNT + 1);
    fb_set_color(0x00444444, BOOT_BLACK);
    fb_puts("---------------------------------------------");
    bp_log_y = BP_LIST_ROW + BP_COUNT + 2;

    uart_puts("\n[bt] PIOS ");
    uart_puts(PIOS_BUILD_LABEL);
    uart_puts(" Boot\n");
    for (u32 i = 0; i < BP_COUNT; i++)
        bp_uart_phase(i, "pending");
}

/* Mark a phase as active: brighten name, show [..] */
static void bp_active(u32 phase) {
    if (phase >= BP_COUNT) return;
    u32 row = BP_LIST_ROW + phase;
    fb_set_cursor(4, row);
    fb_set_color(BOOT_FG_WHITE, BOOT_BLACK);
    fb_puts(bp_names[phase]);
    fb_set_cursor(BP_STAT_COL, row);
    fb_set_color(BOOT_FG_WHITE, BOOT_BLACK);
    fb_puts("[..]");
    bp_uart_phase(phase, "active");
}

/* Mark a phase as done: show [OK] or [!!] */
static void bp_done(u32 phase, bool ok) {
    if (phase >= BP_COUNT) return;
    u32 row = BP_LIST_ROW + phase;
    fb_set_cursor(BP_STAT_COL, row);
    if (ok) {
        fb_set_color(BOOT_FG_OK, BOOT_BLACK);
        fb_puts("[OK]");
    } else {
        fb_set_color(BOOT_FG_FAIL, BOOT_BLACK);
        fb_puts("[!!]");
    }
    bp_uart_phase(phase, ok ? "ok" : "failed");
}

/* Update PC in register panel (col 65, row 2) — shows caller's address */
static void bp_update_pc(void) {
    u64 pc = (u64)(usize)__builtin_return_address(0);
    fb_set_cursor(65, 2);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("PC     %X            ", pc);
}

/* Spinner in top-right corner — shows kernel is alive */
static u32 bp_spin_idx;
static u32 bp_spin_color = 0x0000FF00;  /* green during boot */
static void bp_spin(void) {
    static const char spin[] = "|/-\\";
    fb_set_cursor(126, 0);
    fb_set_color(bp_spin_color, 0x00000000);
    fb_putc(spin[bp_spin_idx & 3]);
    bp_spin_idx++;
}

/* Print uptime prefix: [  0.123] */
static void bp_timestamp(void) {
    u64 ms = timer_monotonic_ms();
    u32 sec = (u32)(ms / 1000);
    u32 frac = (u32)(ms % 1000);
    fb_set_cursor(0, bp_log_y);
    fb_printf("[%3d.%03d] ", sec, frac);
}

/* Append a line to the scrolling boot log */
static void bp_log(const char *msg) {
    fb_set_color(BOOT_FG_LOG, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[diag] ", msg);
}

/* Green log — success */
static void bp_ok(const char *msg) {
    fb_set_color(BOOT_FG_OK, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[ok] ", msg);
}

/* Red log — error */
static void bp_err(const char *msg) {
    fb_set_color(BOOT_FG_FAIL, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[err] ", msg);
}

/* Yellow log — warning */
static void bp_warn(const char *msg) {
    fb_set_color(BOOT_YELLOW, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[warn] ", msg);
}

void early_boot_hdmi_mark(u32 code)
{
    static bool inited;
    if (!inited) {
        if (!fb_init(1280, 720))
            return;
        fb_clear(BOOT_BLACK);
        fb_set_color(BOOT_FG_WHITE, BOOT_BLACK);
        inited = true;
    }
    fb_puts("EARLY ");
    fb_putc((char)code);
    fb_putc('\n');
}

static void boot_policy_mac(const struct boot_policy_record *r, u8 out[32])
{
    if (!r || !out) return;
    hmac_sha256(boot_policy_hmac_key, sizeof(boot_policy_hmac_key),
                (const u8 *)r, (u32)(sizeof(*r) - sizeof(r->mac)), out);
}

static bool boot_policy_mac_ok(const struct boot_policy_record *r)
{
    if (!r) return false;
    u8 calc[32];
    boot_policy_mac(r, calc);
    u32 diff = 0;
    for (u32 i = 0; i < 32; i++)
        diff |= (u32)(calc[i] ^ r->mac[i]);
    return diff == 0;
}

static void boot_update_mac(const struct boot_update_record *r, u8 out[32])
{
    if (!r || !out) return;
    hmac_sha256(boot_policy_hmac_key, sizeof(boot_policy_hmac_key),
                (const u8 *)r, (u32)(sizeof(*r) - sizeof(r->mac)), out);
}

static bool boot_update_mac_ok(const struct boot_update_record *r)
{
    if (!r) return false;
    u8 calc[32];
    boot_update_mac(r, calc);
    u32 diff = 0;
    for (u32 i = 0; i < 32; i++)
        diff |= (u32)(calc[i] ^ r->mac[i]);
    return diff == 0;
}

static bool boot_update_store(struct boot_update_record *r)
{
    if (!r) return false;
    r->magic = BOOT_UPDATE_MAGIC;
    r->version = BOOT_UPDATE_VERSION;
    r->active_slot &= 1U;
    r->pending_slot &= 1U;
    r->previous_slot &= 1U;
    if (r->tries_left > 16U) r->tries_left = 16U;
    r->committed = r->committed ? 1U : 0U;
    boot_update_mac(r, r->mac);
    return picowal_db_put(BOOT_POLICY_CARD, BOOT_UPDATE_REC, r, sizeof(*r)) >= 0;
}

static void boot_update_load_or_seed(void)
{
    struct boot_update_record rec;
    i32 n = picowal_db_get(BOOT_POLICY_CARD, BOOT_UPDATE_REC, &rec, sizeof(rec));
    if (n < (i32)sizeof(rec) || rec.magic != BOOT_UPDATE_MAGIC || !boot_update_mac_ok(&rec)) {
        simd_zero(&rec, sizeof(rec));
        rec.active_slot = 0;
        rec.pending_slot = 0;
        rec.previous_slot = 0;
        rec.tries_left = 0;
        rec.committed = 1;
        rec.generation = 1;
        if (!boot_update_store(&rec)) {
            uart_puts("[bt] boot_update seed write FAILED — continuing\n");
        }
    }
    g_boot_update_state = rec;
}

static void boot_update_reconcile(void)
{
    struct boot_update_record rec = g_boot_update_state;
    bool changed = false;
    if (!rec.committed && rec.pending_slot == rec.active_slot) {
        if (rec.tries_left == 0) {
            rec.active_slot = rec.previous_slot;
            rec.pending_slot = rec.active_slot;
            rec.committed = 1;
            changed = true;
        } else {
            rec.tries_left--;
            changed = true;
        }
    }
    if (changed) {
        rec.generation++;
        if (!boot_update_store(&rec)) {
            uart_puts("[bt] boot_update reconcile write FAILED — continuing\n");
        }
        g_boot_update_state = rec;
    }
}

static void boot_policy_verify_or_seed(void)
{
    u32 cur_el1 = 0, cur_el2 = 0;
    boot_measurements(&cur_el1, &cur_el2, NULL, NULL);
    struct boot_policy_record rec;
    i32 n = picowal_db_get(BOOT_POLICY_CARD, BOOT_POLICY_REC, &rec, sizeof(rec));
    if (n < (i32)sizeof(rec)) {
        rec.magic = BOOT_POLICY_MAGIC;
        rec.version = BOOT_POLICY_VERSION;
        rec.el1_hash = cur_el1;
        rec.el2_hash = cur_el2;
        boot_policy_mac(&rec, rec.mac);
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_POLICY_REC, &rec, sizeof(rec)) < 0) {
            uart_puts("[bt] boot policy seed write FAILED — continuing\n");
        }
    } else {
        if (rec.magic != BOOT_POLICY_MAGIC || !boot_policy_mac_ok(&rec)) {
            /* Corrupted policy record — re-seed */
            uart_puts("[bt] policy corrupt, reseed\n");
            rec.magic = BOOT_POLICY_MAGIC;
            rec.version = BOOT_POLICY_VERSION;
            rec.el1_hash = cur_el1;
            rec.el2_hash = cur_el2;
            boot_policy_mac(&rec, rec.mac);
            if (picowal_db_put(BOOT_POLICY_CARD, BOOT_POLICY_REC, &rec, sizeof(rec)) < 0) {
                uart_puts("[bt] boot policy reseed write FAILED — continuing\n");
            }
        } else if (rec.el1_hash != cur_el1 || rec.el2_hash != cur_el2) {
            /* Kernel changed — re-seed during development */
            uart_puts("[bt] hash changed, update policy\n");
            rec.el1_hash = cur_el1;
            rec.el2_hash = cur_el2;
            rec.version++;
            boot_policy_mac(&rec, rec.mac);
            if (picowal_db_put(BOOT_POLICY_CARD, BOOT_POLICY_REC, &rec, sizeof(rec)) < 0) {
                uart_puts("[bt] boot policy update write FAILED — continuing\n");
            }
        }
    }

    u32 rollback_floor = 0;
    i32 rn = picowal_db_get(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor));
    if (rn < (i32)sizeof(rollback_floor)) {
        rollback_floor = rec.version;
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor)) < 0) {
            uart_puts("[bt] rollback floor seed write FAILED — continuing\n");
        }
    } else if (rec.version < rollback_floor) {
        /* Anti-rollback: production would PiSOD here, but in dev we warn
         * and reset the floor so the box can be recovered after walfs
         * corruption breaks the version/floor consistency. */
        uart_puts("[bt] rollback floor > rec.version (dev: resetting floor)\n");
        rollback_floor = rec.version;
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor)) < 0) {
            uart_puts("[bt] rollback floor reset write FAILED — continuing\n");
        }
    }
    if (rec.version > rollback_floor) {
        rollback_floor = rec.version;
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor)) < 0) {
            uart_puts("[bt] rollback floor update write FAILED — continuing\n");
        }
    }

    boot_el1_expected_hash = rec.el1_hash;
    boot_el2_expected_hash = rec.el2_hash;
    boot_update_load_or_seed();
    boot_update_reconcile();
}

static bool ui_streq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++;
        b++;
    }
    return (*a == 0 && *b == 0);
}

static bool ui_strneq(const char *a, const char *b, u32 n)
{
    if (!a || !b) return false;
    for (u32 i = 0; i < n; i++) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

static bool ui_parse_u32(const char *s, u32 *out)
{
    if (!s || !*s || !out) return false;
    u32 base = 10;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        if (!*s) return false;
    }
    u32 v = 0;
    while (*s) {
        u32 d;
        char c = *s++;
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return false;
        if (d >= base) return false;
        v = v * base + d;
    }
    *out = v;
    return true;
}

static bool ui_parse_u64(const char *s, u64 *out)
{
    if (!s || !*s || !out) return false;
    u32 base = 10;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        if (!*s) return false;
    }
    u64 v = 0;
    while (*s) {
        u32 d;
        char c = *s++;
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return false;
        if (d >= base) return false;
        v = v * base + d;
    }
    *out = v;
    return true;
}

static bool ui_parse_ip4(const char *s, u32 *out)
{
    if (!s || !out) return false;
    u32 oct[4] = {0,0,0,0};
    u32 idx = 0;
    u32 cur = 0;
    bool have = false;
    while (*s) {
        char c = *s++;
        if (c >= '0' && c <= '9') {
            have = true;
            cur = cur * 10U + (u32)(c - '0');
            if (cur > 255U) return false;
        } else if (c == '.') {
            if (!have || idx >= 3) return false;
            oct[idx++] = cur;
            cur = 0;
            have = false;
        } else {
            return false;
        }
    }
    if (!have || idx != 3) return false;
    oct[3] = cur;
    *out = IP4(oct[0], oct[1], oct[2], oct[3]);
    return true;
}

static bool ui_parse_cidr_prefix(const char *s, u32 *prefix_out)
{
    if (!s || !*s || !prefix_out)
        return false;
    u32 v = 0;
    while (*s) {
        if (*s < '0' || *s > '9')
            return false;
        v = v * 10U + (u32)(*s++ - '0');
        if (v > 32U)
            return false;
    }
    *prefix_out = v;
    return true;
}

static u32 ui_prefix_mask(u32 prefix)
{
    if (prefix == 0)
        return 0;
    if (prefix >= 32)
        return 0xFFFFFFFFU;
    return 0xFFFFFFFFU << (32U - prefix);
}

static bool ui_parse_ip_spec(const char *s, u32 *flags, bool src, u32 *ip,
                             u32 *mask, u32 *end)
{
    char left[32];
    char right[32];
    if (!s || !flags || !ip || !mask || !end)
        return false;
    if (ui_streq(s, "any")) {
        *ip = 0;
        *mask = 0;
        *end = 0;
        return true;
    }

    u32 i = 0;
    while (s[i] && s[i] != '/' && s[i] != '-' && i + 1 < sizeof(left)) {
        left[i] = s[i];
        i++;
    }
    left[i] = 0;
    if (!ui_parse_ip4(left, ip))
        return false;

    if (s[i] == '/') {
        u32 j = 0;
        i++;
        while (s[i] && j + 1 < sizeof(right))
            right[j++] = s[i++];
        right[j] = 0;
        u32 prefix = 0;
        if (ui_parse_cidr_prefix(right, &prefix)) {
            *mask = ui_prefix_mask(prefix);
        } else if (!ui_parse_ip4(right, mask)) {
            return false;
        }
        *end = 0;
        *flags |= src ? NIC_FILTER_IP_FROM : NIC_FILTER_IP_TO;
        return true;
    }

    if (s[i] == '-') {
        u32 j = 0;
        i++;
        while (s[i] && j + 1 < sizeof(right))
            right[j++] = s[i++];
        right[j] = 0;
        if (!ui_parse_ip4(right, end))
            return false;
        *mask = 0;
        *flags |= src ? NIC_FILTER_IP_FROM_RANGE : NIC_FILTER_IP_TO_RANGE;
        return true;
    }

    if (s[i] != 0)
        return false;
    *mask = 0;
    *end = 0;
    *flags |= src ? NIC_FILTER_IP_FROM : NIC_FILTER_IP_TO;
    return true;
}

static bool ui_parse_priority(const char *s, u32 *out_prio)
{
    if (!s || !out_prio) return false;
    if (ui_streq(s, "lazy")) { *out_prio = PROC_PRIO_LAZY; return true; }
    if (ui_streq(s, "low")) { *out_prio = PROC_PRIO_LOW; return true; }
    if (ui_streq(s, "normal")) { *out_prio = PROC_PRIO_NORMAL; return true; }
    if (ui_streq(s, "high")) { *out_prio = PROC_PRIO_HIGH; return true; }
    if (ui_streq(s, "realtime")) { *out_prio = PROC_PRIO_REALTIME; return true; }
    return false;
}

static const char *ui_priority_str(u32 p)
{
    if (p == PROC_PRIO_LAZY) return "lazy";
    if (p == PROC_PRIO_LOW) return "low";
    if (p == PROC_PRIO_HIGH) return "high";
    if (p == PROC_PRIO_REALTIME) return "realtime";
    return "normal";
}

static bool ui_parse_mac6(const char *s, u8 out[6])
{
    if (!s || !out) return false;
    for (u32 i = 0; i < 6; i++) {
        char a = *s++;
        char b = *s++;
        if (!a || !b) return false;
        u32 hi, lo;
        if (a >= '0' && a <= '9') hi = (u32)(a - '0');
        else if (a >= 'a' && a <= 'f') hi = (u32)(a - 'a' + 10);
        else if (a >= 'A' && a <= 'F') hi = (u32)(a - 'A' + 10);
        else return false;
        if (b >= '0' && b <= '9') lo = (u32)(b - '0');
        else if (b >= 'a' && b <= 'f') lo = (u32)(b - 'a' + 10);
        else if (b >= 'A' && b <= 'F') lo = (u32)(b - 'A' + 10);
        else return false;
        out[i] = (u8)((hi << 4) | lo);
        if (i < 5) {
            char sep = *s++;
            if (sep != ':') return false;
        }
    }
    return *s == 0;
}

static void ui_console_write(const char *s)
{
    uart_puts(s);
    if (ui_mode == UI_MODE_CONSOLE)
        fb_puts(s);
    debug_tcp_send(s);
}

static void ui_console_prompt(void)
{
    uart_vt_color(UART_COLOR_GREEN, UART_COLOR_BLACK, true);
    ui_console_write("ready> ");
    uart_vt_reset();
}

static void ui_print_ip(u32 ip)
{
    u32 a = (ip >> 24) & 0xFF;
    u32 b = (ip >> 16) & 0xFF;
    u32 c = (ip >> 8) & 0xFF;
    u32 d = ip & 0xFF;
    fb_printf("%u.%u.%u.%u", a, b, c, d);
    uart_puts("[");
    uart_hex(a); uart_putc('.');
    uart_hex(b); uart_putc('.');
    uart_hex(c); uart_putc('.');
    uart_hex(d); uart_puts("]");
}

static void ui_console_ip(u32 ip)
{
    fb_printf("%u.%u.%u.%u", (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
              (ip >> 8) & 0xFF, ip & 0xFF);
}

static void ui_console_route_table(void)
{
    struct net_route_entry routes[NET_ROUTE_MAX];
    u32 n = net_route_snapshot(routes, NET_ROUTE_MAX);
    ui_console_write("ROUTE DST MASK GW FLAGS PFX\n");
    for (u32 i = 0; i < n; i++) {
        fb_printf("%u ", i);
        ui_console_ip(routes[i].dst);
        ui_console_write(" ");
        ui_console_ip(routes[i].mask);
        ui_console_write(" ");
        ui_console_ip(routes[i].gateway);
        ui_console_write(" ");
        fb_printf("%u %u\n", routes[i].flags, routes[i].prefix_len);
    }
}

static void ui_console_mac(const u8 *mac)
{
    static const char hx[] = "0123456789ABCDEF";
    for (u32 i = 0; i < 6; i++) {
        if (i) ui_console_write(":");
        char b[3] = { hx[mac[i] >> 4], hx[mac[i] & 0xF], 0 };
        ui_console_write(b);
    }
}

static void ui_console_net_egress_trace(void)
{
    struct net_egress_snapshot e;
    net_egress_snapshot(&e);
    ui_console_write("egress resolves=");
    ui_console_u64_dec(e.resolve_calls);
    ui_console_write(" no_route=");
    ui_console_u64_dec(e.no_route);
    ui_console_write(" no_mac=");
    ui_console_u64_dec(e.no_mac);
    ui_console_write(" udp_attempts=");
    ui_console_u64_dec(e.udp_attempts);
    ui_console_write(" udp_ok=");
    ui_console_u64_dec(e.udp_ok);
    ui_console_write(" udp_fail=");
    ui_console_u64_dec(e.udp_fail);
    ui_console_write("\nlast dst=");
    ui_console_ip(e.last_dst_ip);
    ui_console_write(" next_hop=");
    ui_console_ip(e.last_next_hop);
    ui_console_write(" route=");
    ui_console_ip(e.last_route_dst);
    ui_console_write("/");
    ui_console_u32_dec(e.last_route_prefix);
    ui_console_write(" gw=");
    ui_console_ip(e.last_route_gateway);
    ui_console_write(" flags=");
    ui_console_u32_dec(e.last_route_flags);
    ui_console_write(" mac_source=");
    ui_console_write(net_egress_source_name(e.last_mac_source));
    ui_console_write(" mac=");
    ui_console_mac(e.last_mac);
    ui_console_write("\nlast_udp sport=");
    ui_console_u32_dec(e.last_udp_src_port);
    ui_console_write(" dport=");
    ui_console_u32_dec(e.last_udp_dst_port);
    ui_console_write(" len=");
    ui_console_u32_dec(e.last_udp_len);
    ui_console_write(" dst=");
    ui_console_ip(e.last_udp_dst_ip);
    ui_console_write(" next_hop=");
    ui_console_ip(e.last_udp_next_hop);
    ui_console_write(" mac_source=");
    ui_console_write(net_egress_source_name(e.last_udp_mac_source));
    ui_console_write(" mac=");
    ui_console_mac(e.last_udp_mac);
    ui_console_write(" ok=");
    ui_console_write(e.last_udp_ok ? "yes\n" : "no\n");
}

static void ui_console_arp_table(void)
{
    struct arp_snapshot_entry e[ARP_TABLE_SIZE];
    u32 n = arp_snapshot(e, ARP_TABLE_SIZE);
    ui_console_write("ARP IP MAC STATE RETRY CONS AGE_MS\n");
    for (u32 i = 0; i < n; i++) {
        ui_console_ip(e[i].ip);
        ui_console_write(" ");
        ui_console_mac(e[i].mac);
        ui_console_write(" ");
        fb_printf("%u %u %u %u\n", e[i].state, e[i].retries,
                  e[i].consistency, (u32)e[i].age_ms);
    }
}

static void ui_console_print_ps(void)
{
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    ui_console_write("PID      PPID     AFF  PRI       CPU%  MEM  ARENA used/high/bump/span#  STATE\n");
    for (u32 i = 0; i < n; i++) {
        if (snap[i].parent_pid == PROC_UI_KERNEL_PARENT_PID) {
            fb_printf("0x%x   -1       %u    %s   %u    %u       %s\n",
                      snap[i].pid, snap[i].affinity_core,
                      ui_priority_str(snap[i].priority_class), snap[i].cpu_percent,
                      snap[i].mem_kib, ui_proc_state_str(snap[i].state));
        } else {
            fb_printf("0x%x   0x%x   %u    %s   %u    %u       %s\n",
                      snap[i].pid, snap[i].parent_pid, snap[i].affinity_core,
                      ui_priority_str(snap[i].priority_class), snap[i].cpu_percent,
                      snap[i].mem_kib, ui_proc_state_str(snap[i].state));
        }
        uart_hex(snap[i].pid);
        uart_puts(" ppid=");
        if (snap[i].parent_pid == PROC_UI_KERNEL_PARENT_PID) uart_puts("-1");
        else uart_hex(snap[i].parent_pid);
        uart_puts(" aff=");
        uart_hex(snap[i].affinity_core);
        uart_puts(" pri=");
        uart_puts(ui_priority_str(snap[i].priority_class));
        uart_puts(" cpu=");
        uart_hex(snap[i].cpu_percent);
        uart_puts(" mem=");
        uart_hex(snap[i].mem_kib);
        uart_puts(" arena=");
        uart_hex(snap[i].arena_used_kib);
        uart_puts("/");
        uart_hex(snap[i].arena_high_kib);
        uart_puts(" bump=");
        uart_hex(snap[i].arena_bump_kib);
        uart_puts(" span=");
        uart_hex(snap[i].arena_span_kib);
        uart_puts("#");
        uart_hex(snap[i].arena_span_count);
        uart_puts(" state=");
        uart_puts(ui_proc_state_str(snap[i].state));
        uart_puts("\n");
    }
    ui_console_write("\nProcess graph:\n");
    for (u32 i = 0; i < n; i++) {
        if (snap[i].parent_pid != 0 && proc_ui_has_pid(snap, n, snap[i].parent_pid))
            continue;
        ui_console_write("  ");
        ui_console_hex_fixed(snap[i].pid, 8);
        ui_console_write(" ");
        ui_console_write(snap[i].image_path);
        ui_console_write("\n");
        for (u32 j = 0; j < n; j++) {
            if (snap[j].parent_pid != snap[i].pid)
                continue;
            ui_console_write("    -> ");
            ui_console_hex_fixed(snap[j].pid, 8);
            ui_console_write(" ");
            ui_console_write(snap[j].image_path);
            ui_console_write("\n");
        }
    }
}

struct fsinspect_ctx {
    u32 count;
    u32 max;
};
static struct fsinspect_ctx fs_ctx;

struct ui_dir_entry {
    u64 id;
    char name[128];
};
struct ui_dir_collect_ctx {
    struct ui_dir_entry *out;
    u32 max;
    u32 count;
};
static struct ui_dir_collect_ctx dir_collect_ctx;

static bool ui_path_split_parent(const char *path, char *parent, u32 parent_max, char *leaf, u32 leaf_max)
{
    if (!path || path[0] != '/' || !parent || !leaf) return false;
    u32 len = pios_strlen(path);
    if (len < 2) return false;
    while (len > 1 && path[len - 1] == '/') len--;
    if (len < 2) return false;
    i32 slash = -1;
    for (u32 i = 0; i < len; i++) if (path[i] == '/') slash = (i32)i;
    if (slash < 0 || (u32)slash >= len - 1) return false;
    u32 nlen = len - (u32)slash - 1;
    if (nlen + 1 > leaf_max) return false;
    for (u32 i = 0; i < nlen; i++) leaf[i] = path[(u32)slash + 1 + i];
    leaf[nlen] = 0;
    if (slash == 0) {
        if (parent_max < 2) return false;
        parent[0] = '/'; parent[1] = 0;
        return true;
    }
    if ((u32)slash + 1 > parent_max) return false;
    for (u32 i = 0; i < (u32)slash; i++) parent[i] = path[i];
    parent[slash] = 0;
    return true;
}

static bool ui_path_join(const char *base, const char *name, char *out, u32 out_max)
{
    if (!base || !name || !out || out_max < 2) return false;
    u32 bl = pios_strlen(base);
    u32 nl = pios_strlen(name);
    bool root = (bl == 1 && base[0] == '/');
    u32 need = bl + nl + (root ? 0 : 1) + 1;
    if (need > out_max) return false;
    u32 p = 0;
    for (u32 i = 0; i < bl; i++) out[p++] = base[i];
    if (!root) out[p++] = '/';
    for (u32 i = 0; i < nl; i++) out[p++] = name[i];
    out[p] = 0;
    return true;
}

static bool ui_fs_create_path(const char *path, bool is_dir)
{
    char parent[256];
    char leaf[128];
    if (!ui_path_split_parent(path, parent, sizeof(parent), leaf, sizeof(leaf)))
        return false;
    u64 parent_id = walfs_find(parent);
    if (!parent_id) return false;
    u32 flags = is_dir ? WALFS_DIR : WALFS_FILE;
    u64 id = walfs_create(parent_id, leaf, flags, 0644);
    return id != 0;
}

static bool ui_path_resolve(const char *in, char *out, u32 out_max)
{
    if (!in || !*in || !out || out_max < 2) return false;
    if (ui_streq(in, ".")) in = ui_cwd;
    if (in[0] == '/') {
        u32 n = pios_strlen(in);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i < n; i++) out[i] = in[i];
        while (n > 1 && out[n - 1] == '/') n--;
        out[n] = 0;
        return true;
    }
    if (ui_streq(in, "..")) {
        u32 n = pios_strlen(ui_cwd);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i < n; i++) out[i] = ui_cwd[i];
        if (n > 1) {
            while (n > 1 && out[n - 1] != '/') n--;
            if (n == 1) out[n] = 0;
            else out[n - 1] = 0;
        } else out[1] = 0;
        return true;
    }
    if (ui_cwd[0] == '/' && ui_cwd[1] == 0) {
        u32 nl = pios_strlen(in);
        if (nl + 2 > out_max) return false;
        out[0] = '/';
        for (u32 i = 0; i < nl; i++) out[i + 1] = in[i];
        out[nl + 1] = 0;
        return true;
    }
    u32 cl = pios_strlen(ui_cwd);
    u32 nl = pios_strlen(in);
    if (cl + nl + 2 > out_max) return false;
    u32 p = 0;
    for (u32 i = 0; i < cl; i++) out[p++] = ui_cwd[i];
    out[p++] = '/';
    for (u32 i = 0; i < nl; i++) out[p++] = in[i];
    out[p] = 0;
    return true;
}

static bool ui_has_suffix(const char *s, const char *suffix)
{
    if (!s || !suffix) return false;
    u32 sl = pios_strlen(s);
    u32 tl = pios_strlen(suffix);
    if (tl > sl) return false;
    for (u32 i = 0; i < tl; i++) {
        if (s[sl - tl + i] != suffix[i]) return false;
    }
    return true;
}

static bool ui_resolve_pis_path(const char *in, char *out, u32 out_max)
{
    char abs[256];
    if (!ui_path_resolve(in, abs, sizeof(abs))) return false;
    if (ui_has_suffix(abs, ".pis") || ui_has_suffix(abs, ".pbc")) {
        u32 n = pios_strlen(abs);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = abs[i];
        return true;
    }

    if (pios_strlen(abs) + 4 + 1 > out_max) return false;
    u32 p = 0;
    while (abs[p]) { out[p] = abs[p]; p++; }
    out[p++] = '.';
    out[p++] = 'p';
    out[p++] = 'i';
    out[p++] = 's';
    out[p] = 0;
    if (walfs_find(out)) return true;
    p -= 4;
    out[p] = 0;
    return true;
}

static bool ui_resolve_pix_path(const char *in, char *out, u32 out_max)
{
    char abs[256];
    if (!ui_path_resolve(in, abs, sizeof(abs))) return false;
    if (ui_has_suffix(abs, ".pix")) {
        u32 n = pios_strlen(abs);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = abs[i];
        return true;
    }
    if (pios_strlen(abs) + 4 + 1 > out_max) return false;
    u32 p = 0;
    while (abs[p]) { out[p] = abs[p]; p++; }
    out[p++] = '.';
    out[p++] = 'p';
    out[p++] = 'i';
    out[p++] = 'x';
    out[p] = 0;
    if (walfs_find(out)) return true;
    p -= 4;
    out[p] = 0;
    return true;
}

static bool ui_resolve_job_path(const char *in, char *out, u32 out_max, bool *is_script_out)
{
    char pix[256];
    char pis[256];
    if (is_script_out) *is_script_out = false;
    if (!ui_resolve_pix_path(in, pix, sizeof(pix))) return false;
    if (walfs_find(pix)) {
        u32 n = pios_strlen(pix);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = pix[i];
        return true;
    }
    if (!ui_resolve_pis_path(in, pis, sizeof(pis))) return false;
    if (walfs_find(pis)) {
        u32 n = pios_strlen(pis);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = pis[i];
        if (is_script_out) *is_script_out = ui_has_suffix(pis, ".pis");
        return true;
    }
    return false;
}

static void ui_dir_collect_cb(const struct walfs_dirent *entry)
{
    if (!entry || dir_collect_ctx.count >= dir_collect_ctx.max) return;
    struct ui_dir_entry *d = &dir_collect_ctx.out[dir_collect_ctx.count++];
    d->id = entry->child_id;
    for (u32 i = 0; i < 127; i++) {
        d->name[i] = (char)entry->name[i];
        if (entry->name[i] == 0) break;
    }
    d->name[127] = 0;
}

static void ui_fsinspect_cb(const struct walfs_dirent *entry)
{
    if (!entry || fs_ctx.count >= fs_ctx.max)
        return;
    struct walfs_inode ino;
    if (!walfs_stat(entry->child_id, &ino))
        return;
    fb_printf("  %s  id=0x%x size=%u %s\n",
              ino.name, (u32)ino.inode_id, (u32)ino.size,
              (ino.flags & WALFS_DIR) ? "<dir>" : "<file>");
    uart_puts("  ");
    uart_puts((const char *)ino.name);
    uart_puts(" id=");
    uart_hex((u32)ino.inode_id);
    uart_puts(" size=");
    uart_hex((u32)ino.size);
    uart_puts((ino.flags & WALFS_DIR) ? " <dir>\n" : " <file>\n");
    fs_ctx.count++;
}

static void ui_cmd_fsinspect(const char *path)
{
    if (!path || !*path) {
        ui_console_write("ERR: usage fsinspect <path>\n");
        return;
    }
    u64 id = walfs_find(path);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino)) {
        ui_console_write("ERR: stat failed\n");
        return;
    }
    fb_printf("path=%s id=0x%x size=%u flags=0x%x mode=0x%x\n",
              path, (u32)id, (u32)ino.size, ino.flags, ino.mode);
    uart_puts("path=");
    uart_puts(path);
    uart_puts(" id=");
    uart_hex((u32)id);
    uart_puts(" size=");
    uart_hex((u32)ino.size);
    uart_puts(" flags=");
    uart_hex(ino.flags);
    uart_puts(" mode=");
    uart_hex(ino.mode);
    uart_puts("\n");

    if (ino.flags & WALFS_DIR) {
        fs_ctx.count = 0;
        fs_ctx.max = 64;
        walfs_readdir(id, ui_fsinspect_cb);
        fb_printf("entries=%u\n", fs_ctx.count);
        uart_puts("entries=");
        uart_hex(fs_ctx.count);
        uart_puts("\n");
    }
}

/* WiFi support has been parked in spike/wifi/ — see GitHub issue.
 * The 'wifi' console command now returns an ERR stub. */

static void ui_cmd_usb(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "status")) {
        struct usb_device *dev = usb_get_device();
        const struct xhci_stats *st = xhci_get_stats();
        fb_printf("USB: dev=%s kbd=%s\n",
                  dev ? "yes" : "no",
                  usb_kbd_available() ? "ready" : "not-ready");
        if (dev) {
            fb_printf("USB: vid=%x pid=%x class=%x eps=%u\n",
                      dev->vendor_id, dev->product_id, dev->dev_class, dev->num_eps);
            for (u32 i = 0; i < dev->num_eps; i++) {
                fb_printf("  ep=%x attr=%x max=%u int=%u iface=%u cls=%x sub=%x proto=%x\n",
                          dev->eps[i].address, dev->eps[i].attributes,
                          dev->eps[i].max_packet, dev->eps[i].interval,
                          dev->eps[i].iface_number, dev->eps[i].iface_class,
                          dev->eps[i].iface_subclass, dev->eps[i].iface_protocol);
            }
        }
        fb_printf("xHCI: cmd=%u/%u timeout=%u xfer=%u/%u evt=%u stale=%u reset=%u stall=%u ringfull=%u\n",
                  st->cmd_completed, st->cmd_submitted, st->cmd_timeout,
                  st->xfer_ok, st->xfer_fail, st->evt_polled,
                  st->evt_stale_drained, st->ep_resets, st->ep_stalls,
                  st->ring_full);
        ui_console_write("usage: usb status|reinit|poll\n");
        return;
    }

    if (ui_streq(argv[1], "reinit")) {
        ui_console_write("USB: reinitializing xHCI...\n");
        if (usb_init())
            ui_console_write("OK: usb initialized\n");
        else
            ui_console_write("ERR: usb init failed\n");
        return;
    }

    if (ui_streq(argv[1], "poll")) {
        i32 c;
        u32 n = 0;
        while ((c = usb_kbd_try_getc()) >= 0) {
            char s[2] = { (char)c, 0 };
            ui_console_write(s);
            n++;
        }
        fb_printf("\nUSB: drained %u chars\n", n);
        return;
    }

    ui_console_write("ERR: usage usb status|reinit|poll\n");
}

static void ui_console_u32_dec(u32 v)
{
    char tmp[11];
    u32 n = 0;
    if (v == 0) {
        ui_console_write("0");
        return;
    }

    while (v && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n) {
        char s[2] = { tmp[--n], 0 };
        ui_console_write(s);
    }
}

static void ui_console_u64_dec(u64 v)
{
    char buf[24];
    int i = (int)sizeof(buf) - 1;
    buf[i] = 0;
    if (v == 0) {
        ui_console_write("0");
        return;
    }
    while (v && i > 0) {
        buf[--i] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    ui_console_write(&buf[i]);
}

static void ui_console_hex_fixed(u64 v, u32 digits)
{
    static const char hx[] = "0123456789ABCDEF";
    ui_console_write("0x");
    for (i32 i = (i32)((digits - 1U) * 4U); i >= 0; i -= 4) {
        char s[2] = { hx[(v >> (u32)i) & 0xFULL], 0 };
        ui_console_write(s);
    }
}

static void ui_cmd_mem(u32 argc, char **argv)
{
    if (argc >= 2 && !ui_streq(argv[1], "analyze")) {
        ui_console_write("ERR: usage mem analyze\n");
        return;
    }
    u64 k_start = (u64)(usize)&_start;
    u64 text_start = (u64)(usize)&__text_start;
    u64 text_end = (u64)(usize)&__text_end;
    u64 bss_start = (u64)(usize)&__bss_start;
    u64 bss_end = (u64)(usize)&__bss_end;
    u64 heap_start = (u64)(usize)&__heap_start;
    u32 image_len = http_running_kernel_image_len();
    struct proc_ui_entry snap[MAX_PROCS_PER_CORE + 1U];
    u32 n = proc_snapshot(snap, MAX_PROCS_PER_CORE + 1U);

    ui_console_write("mem kernel_start=");
    ui_console_hex_fixed(k_start, 16);
    ui_console_write(" image_bytes=");
    ui_console_u32_dec(image_len);
    ui_console_write(" slot_capacity=");
    ui_console_u32_dec(PIOS_STAGE2_ZONE_BYTES);
    ui_console_write(" slot_free=");
    ui_console_u32_dec(image_len < PIOS_STAGE2_ZONE_BYTES ? PIOS_STAGE2_ZONE_BYTES - image_len : 0);
    ui_console_write("\ntext=");
    ui_console_hex_fixed(text_start, 16);
    ui_console_write("..");
    ui_console_hex_fixed(text_end, 16);
    ui_console_write(" bss=");
    ui_console_hex_fixed(bss_start, 16);
    ui_console_write("..");
    ui_console_hex_fixed(bss_end, 16);
    ui_console_write(" heap_start=");
    ui_console_hex_fixed(heap_start, 16);
    ui_console_write("\nCORE RAM_BASE SLOT0 SLOT_END\n");
    for (u32 c = 0; c < 4; c++) {
        ui_console_u32_dec(c);
        ui_console_write(" ");
        ui_console_hex_fixed(core_ram_bases[c], 16);
        ui_console_write(" ");
        ui_console_hex_fixed(core_ram_bases[c] + PROC_SLOT_OFFSET, 16);
        ui_console_write(" ");
        ui_console_hex_fixed(core_ram_bases[c] + PROC_SLOT_OFFSET + ((u64)MAX_PROCS_PER_CORE * PROC_SLOT_SIZE), 16);
        ui_console_write("\n");
    }
    ui_console_write("PROC PID CORE STATE MEMK ACAP AUSED AHI ABUMP ASPAN SCNT IMAGE\n");
    for (u32 i = 0; i < n; i++) {
        ui_console_u32_dec(snap[i].pid);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].affinity_core);
        ui_console_write(" ");
        ui_console_write(ui_proc_state_str(snap[i].state));
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].mem_kib);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].arena_capacity_kib);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].arena_used_kib);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].arena_high_kib);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].arena_bump_kib);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].arena_span_kib);
        ui_console_write(" ");
        ui_console_u32_dec(snap[i].arena_span_count);
        ui_console_write(" ");
        ui_console_write(snap[i].image_path);
        ui_console_write("\n");
    }
}

static bool ui_mem_width(const char *s, u32 *width)
{
    if (!width) return false;
    if (!s) { *width = 4; return true; }
    if (ui_streq(s, "1") || ui_streq(s, "u8") || ui_streq(s, "byte")) { *width = 1; return true; }
    if (ui_streq(s, "2") || ui_streq(s, "16") || ui_streq(s, "half")) { *width = 2; return true; }
    if (ui_streq(s, "4") || ui_streq(s, "32") || ui_streq(s, "word")) { *width = 4; return true; }
    if (ui_streq(s, "8") || ui_streq(s, "64") || ui_streq(s, "qword")) { *width = 8; return true; }
    return false;
}

static u64 ui_mem_read(u64 addr, u32 width)
{
    if (width == 1) return *(volatile u8 *)(usize)addr;
    if (width == 2) return *(volatile u16 *)(usize)addr;
    if (width == 8) return *(volatile u64 *)(usize)addr;
    return *(volatile u32 *)(usize)addr;
}

static void ui_mem_write(u64 addr, u64 value, u32 width)
{
    if (width == 1) *(volatile u8 *)(usize)addr = (u8)value;
    else if (width == 2) *(volatile u16 *)(usize)addr = (u16)value;
    else if (width == 8) *(volatile u64 *)(usize)addr = value;
    else *(volatile u32 *)(usize)addr = (u32)value;
    dsb();
    isb();
}

static void ui_cmd_peek(u32 argc, char **argv)
{
    u64 addr = 0;
    u32 width = 4;
    if (argc < 2 || !ui_parse_u64(argv[1], &addr) ||
        (argc >= 3 && !ui_mem_width(argv[2], &width))) {
        ui_console_write("ERR: usage peek <addr> [1|2|4|8]\n");
        return;
    }
    u64 v = ui_mem_read(addr, width);
    ui_console_hex_fixed(addr, 16);
    ui_console_write(" = ");
    ui_console_hex_fixed(v, width * 2U);
    ui_console_write("\n");
}

static void ui_cmd_poke(u32 argc, char **argv)
{
    u64 addr = 0, value = 0;
    u32 width = 4;
    if (argc < 3 || !ui_parse_u64(argv[1], &addr) || !ui_parse_u64(argv[2], &value) ||
        (argc >= 4 && !ui_mem_width(argv[3], &width))) {
        ui_console_write("ERR: usage poke <addr> <value> [1|2|4|8]\n");
        return;
    }
    ui_mem_write(addr, value, width);
    ui_console_write("OK: ");
    ui_console_hex_fixed(addr, 16);
    ui_console_write(" <= ");
    ui_console_hex_fixed(value, width * 2U);
    ui_console_write("\n");
}

static void ui_cmd_dumpmem(u32 argc, char **argv)
{
    u64 addr = 0;
    u32 len = 256;
    if (argc < 2 || !ui_parse_u64(argv[1], &addr) ||
        (argc >= 3 && !ui_parse_u32(argv[2], &len))) {
        ui_console_write("ERR: usage dumpmem <addr> [bytes]\n");
        return;
    }
    if (len > 4096) len = 4096;
    for (u32 off = 0; off < len; off += 16) {
        ui_console_hex_fixed(addr + off, 16);
        ui_console_write(": ");
        for (u32 i = 0; i < 16; i++) {
            if (off + i < len) {
                u8 b = *(volatile u8 *)(usize)(addr + off + i);
                ui_console_hex_fixed(b, 2);
                ui_console_write(" ");
            } else {
                ui_console_write("   ");
            }
        }
        ui_console_write(" ");
        for (u32 i = 0; i < 16 && off + i < len; i++) {
            u8 b = *(volatile u8 *)(usize)(addr + off + i);
            char s[2] = { (b >= 32 && b < 127) ? (char)b : '.', 0 };
            ui_console_write(s);
        }
        ui_console_write("\n");
    }
}

static void ui_console_ip4(u32 ip)
{
    ui_console_u32_dec((ip >> 24) & 0xFF);
    ui_console_write(".");
    ui_console_u32_dec((ip >> 16) & 0xFF);
    ui_console_write(".");
    ui_console_u32_dec((ip >> 8) & 0xFF);
    ui_console_write(".");
    ui_console_u32_dec(ip & 0xFF);
}

static void ui_print_dma_diag(void)
{
    struct dma_diag_snapshot d;
    dma_diag_snapshot(&d);
    ui_console_write("DMA enabled=");
    ui_console_write(d.hw_memcpy_enabled ? "yes" : "no");
    ui_console_write(" mode=");
    ui_console_write(d.direct_mode ? "direct" : "cb");
    ui_console_write(" cbaddr=");
    ui_console_write(d.cbaddr_shifted ? "shifted" : "raw");
    ui_console_write(" selftests=");
    ui_console_u32_dec(d.selftest_runs);
    ui_console_write(" failures=");
    ui_console_u32_dec(d.selftest_failures);
    ui_console_write(" last_error=");
    ui_console_u32_dec(d.last_error);
    ui_console_write(" last_ch=");
    ui_console_u32_dec(d.last_channel);
    ui_console_write(" len=");
    ui_console_u32_dec(d.last_len);
    ui_console_write(" enable=");
    ui_console_hex_fixed(d.enable_reg, 8);
    ui_console_write("\nCH CS CBADDR TI SRC DST LEN DEBUG\n");
    for (u32 ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
        ui_console_u32_dec(ch);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].cs, 8);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].cbaddr, 8);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].ti, 8);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].src, 8);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].dst, 8);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].len, 8);
        ui_console_write(" ");
        ui_console_hex_fixed(d.channel[ch].debug, 8);
        ui_console_write("\n");
    }
    if (d.last_error == 4U) {
        ui_console_write("mismatch off=");
        ui_console_u32_dec(d.last_mismatch_off);
        ui_console_write(" got=");
        ui_console_u32_dec(d.last_got);
        ui_console_write(" expected=");
        ui_console_u32_dec(d.last_expected);
        ui_console_write("\n");
    }
}

static void ui_cmd_dma(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(dma_selftest() ? "DMA selftest OK\n" : "DMA selftest FAILED\n");
        ui_print_dma_diag();
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_dma_diag();
        return;
    }
    ui_console_write("ERR: usage dma status | dma selftest\n");
}

static void ui_cmd_keystore(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "derive")) {
        if (argc < 3) {
            ui_console_write("ERR: usage keystore derive <label>\n");
            return;
        }
        u32 fp = 0;
        if (!keystore_derive_fingerprint(argv[2], &fp)) {
            ui_console_write("ERR: derive failed\n");
            return;
        }
        ui_console_write("derive fingerprint=");
        ui_console_hex_fixed(fp, 8);
        ui_console_write("\n");
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        struct keystore_status st;
        keystore_status(&st);
        ui_console_write("keystore initialized=");
        ui_console_write(st.initialized ? "yes" : "no");
        ui_console_write(" sealed=");
        ui_console_write(st.sealed ? "yes" : "no");
        ui_console_write(" serial=");
        ui_console_write(st.board_serial_ok ? "ok" : "fallback");
        ui_console_write(" generation=");
        ui_console_u32_dec(st.generation);
        ui_console_write(" error=");
        ui_console_u32_dec(st.last_error);
        ui_console_write(" lba=");
        ui_console_u32_dec(st.user_records_lba);
        ui_console_write(" fingerprint=");
        ui_console_hex_fixed(st.fingerprint32, 8);
        ui_console_write("\n");
        return;
    }
    ui_console_write("ERR: usage keystore status | keystore derive <label>\n");
}

static void ui_print_tls_diag(void)
{
    struct tls_diag_snapshot t;
    tls_diag_snapshot(&t);
    ui_console_write("TLS kernel=enabled crypto=arm-aese+ghash-nibble bridge=picoweb-style active=");
    ui_console_u32_dec(t.active);
    ui_console_write(" established=");
    ui_console_u32_dec(t.established);
    ui_console_write(" hs_ok=");
    ui_console_u64_dec(t.handshakes_ok);
    ui_console_write(" hs_fail=");
    ui_console_u64_dec(t.handshake_failures);
    ui_console_write(" tx=");
    ui_console_u64_dec(t.records_tx);
    ui_console_write(" rx=");
    ui_console_u64_dec(t.records_rx);
    ui_console_write(" decrypt_fail=");
    ui_console_u64_dec(t.decrypt_failures);
    ui_console_write(" bridge_ok=");
    ui_console_u64_dec(t.bridge_parse_ok);
    ui_console_write(" bridge_more=");
    ui_console_u64_dec(t.bridge_parse_need_more);
    ui_console_write(" bridge_err=");
    ui_console_u64_dec(t.bridge_parse_error);
    ui_console_write(" selftests=");
    ui_console_u32_dec(t.selftests);
    ui_console_write(" failures=");
    ui_console_u32_dec(t.selftest_failures);
    ui_console_write(" last_error=");
    ui_console_u32_dec(t.last_error);
    ui_console_write("\n");
}

static void ui_cmd_tls(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(tls_selftest() ? "TLS selftest OK\n" : "TLS selftest FAILED\n");
        ui_print_tls_diag();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "bridge")) {
        static const u8 sample[] = "GET / HTTP/1.1\r\nHost: pios\r\nConnection: close\r\n\r\n";
        struct tls_bridge_request br;
        i32 r = tls_bridge_parse_request(sample, (u32)(sizeof(sample) - 1), &br);
        ui_console_write("TLS bridge result=");
        ui_console_u32_dec((u32)r);
        ui_console_write(" method=");
        ui_console_write(br.method);
        ui_console_write(" path=");
        ui_console_write(br.path);
        ui_console_write(" header_bytes=");
        ui_console_u32_dec(br.header_bytes);
        ui_console_write("\n");
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_tls_diag();
        return;
    }
    ui_console_write("ERR: usage tls status | tls selftest | tls bridge\n");
}

static void ui_cmd_brotli(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "selftest")) {
        ui_console_write(brotli_selftest() ? "Brotli selftest OK\n" : "Brotli selftest FAILED\n");
        return;
    }
    ui_console_write("ERR: usage brotli selftest\n");
}

static void ui_print_x509_status(void)
{
    struct x509_status xs;
    x509_status(&xs);
    ui_console_write("x509 initialized=");
    ui_console_write(xs.initialized ? "yes" : "no");
    ui_console_write(" key=");
    ui_console_write(xs.has_key ? "yes" : "no");
    ui_console_write(" p256_key=");
    ui_console_write(xs.has_p256_key ? "yes" : "no");
    ui_console_write(" cert=");
    ui_console_write(xs.has_cert ? "yes" : "no");
    ui_console_write(" tls_bound=");
    ui_console_write(xs.tls_bound ? "yes" : "no");
    ui_console_write(" der_ready=");
    ui_console_write(xs.der_ready ? "yes" : "no");
    ui_console_write(" der_len=");
    ui_console_u32_dec(xs.der_len);
    ui_console_write(" csr_ready=");
    ui_console_write(xs.csr_ready ? "yes" : "no");
    ui_console_write(" csr_len=");
    ui_console_u32_dec(xs.csr_len);
    ui_console_write(" csr_alg=");
    ui_console_write(xs.csr_alg == X509_CSR_ALG_P256 ? "p256" :
                     (xs.csr_alg == X509_CSR_ALG_ED25519 ? "ed25519" : "none"));
    ui_console_write(" generation=");
    ui_console_u32_dec(xs.generation);
    ui_console_write(" key_fp=");
    ui_console_hex_fixed(xs.key_fingerprint, 8);
    ui_console_write(" p256_fp=");
    ui_console_hex_fixed(xs.p256_key_fingerprint, 8);
    ui_console_write(" cert_fp=");
    ui_console_hex_fixed(xs.cert_fingerprint, 8);
    ui_console_write(" error=");
    ui_console_u32_dec(xs.last_error);
    ui_console_write("\nsubject=");
    ui_console_write(xs.subject);
    ui_console_write("\nissuer=");
    ui_console_write(xs.issuer);
    ui_console_write("\n");
}

static void ui_cmd_x509(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "generate")) {
        const char *cn = argc >= 3 ? argv[2] : "PIOS kernel dev";
        ui_console_write(x509_generate_dev_cert(cn) ? "X509 generate OK\n" : "X509 generate FAILED\n");
        ui_print_x509_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "csr")) {
        const char *cn = argc >= 3 ? argv[2] : "";
        ui_console_write(x509_generate_csr(cn) ? "X509 CSR OK\n" : "X509 CSR FAILED\n");
        ui_print_x509_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "p256")) {
        const char *cn = argc >= 3 ? argv[2] : "";
        ui_console_write(x509_generate_p256_csr(cn) ? "X509 P256 CSR OK\n" : "X509 P256 CSR FAILED\n");
        ui_print_x509_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "import-self")) {
        u32 cert_len = 0;
        const u8 *cert = x509_certificate_der(&cert_len);
        ui_console_write(x509_import_certificate_der(cert, cert_len) ? "X509 import-self OK\n" : "X509 import-self FAILED\n");
        ui_print_x509_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "bind")) {
        ui_console_write(x509_bind_tls() ? "X509 bind OK\n" : "X509 bind FAILED\n");
        ui_print_x509_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(x509_selftest() ? "X509 selftest OK\n" : "X509 selftest FAILED\n");
        ui_print_x509_status();
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_x509_status();
        return;
    }
    ui_console_write("ERR: usage x509 status | x509 generate [cn] | x509 csr [cn] | x509 p256 [cn] | x509 bind | x509 import-self | x509 selftest\n");
}

static void ui_print_acme_status(void)
{
    struct acme_status as;
    acme_status(&as);
    ui_console_write("acme initialized=");
    ui_console_write(as.initialized ? "yes" : "no");
    ui_console_write(" account_key=");
    ui_console_write(as.account_key ? "yes" : "no");
    ui_console_write(" csr_ready=");
    ui_console_write(as.csr_ready ? "yes" : "no");
    ui_console_write(" challenge_ready=");
    ui_console_write(as.challenge_ready ? "yes" : "no");
    ui_console_write(" state=");
    ui_console_u32_dec(as.state);
    ui_console_write(" account_fp=");
    ui_console_hex_fixed(as.account_fingerprint, 8);
    ui_console_write(" csr_len=");
    ui_console_u32_dec(as.csr_len);
    ui_console_write(" error=");
    ui_console_u32_dec(as.last_error);
    ui_console_write("\ndirectory=");
    ui_console_write(as.directory);
    ui_console_write("\ndomain=");
    ui_console_write(as.domain);
    ui_console_write("\ntoken=");
    ui_console_write(as.token);
    ui_console_write("\n");
}

static void ui_cmd_acme(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "prepare")) {
        const char *domain = argc >= 3 ? argv[2] : "";
        ui_console_write(acme_prepare_http01(domain) ? "ACME prepare OK\n" : "ACME prepare FAILED\n");
        ui_print_acme_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "challenge")) {
        const char *token = argc >= 3 ? argv[2] : "";
        const char *keyauth = argc >= 4 ? argv[3] : "";
        ui_console_write(acme_set_http01_challenge(token, keyauth) ? "ACME challenge OK\n" : "ACME challenge FAILED\n");
        ui_print_acme_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "csrhex")) {
        u32 csr_len = 0;
        const u8 *csr = x509_csr_der(&csr_len);
        if (!csr || csr_len == 0) {
            ui_console_write("ERR: no CSR ready; run acme prepare <domain>\n");
            return;
        }
        ui_console_write("csr_der_hex len=");
        ui_console_u32_dec(csr_len);
        ui_console_write("\n");
        for (u32 i = 0; i < csr_len; i++) {
            ui_console_hex_fixed(csr[i], 2);
            if ((i & 31U) == 31U) ui_console_write("\n");
        }
        ui_console_write("\n");
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "clear")) {
        acme_clear_http01_challenge();
        ui_console_write("ACME clear OK\n");
        ui_print_acme_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(acme_selftest() ? "ACME selftest OK\n" : "ACME selftest FAILED\n");
        ui_print_acme_status();
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_acme_status();
        return;
    }
    ui_console_write("ERR: usage acme status | acme prepare <domain> | acme csrhex | acme challenge <token> <keyauth> | acme clear | acme selftest\n");
}

static void ui_cmd_ksvc(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        bool ok = ksvc_mailbox_selftest(ksvc_debug_id) && ksvc_fault_policy_selftest(ksvc_debug_id);
        ui_console_write(ok ? "KSVC selftest OK\n" : "KSVC selftest FAILED\n");
        return;
    }
    if (argc >= 3 && (ui_streq(argv[1], "pause") || ui_streq(argv[1], "resume") ||
                      ui_streq(argv[1], "restart") || ui_streq(argv[1], "fault"))) {
        u32 id = 0;
        if (!ui_parse_u32(argv[2], &id)) {
            ui_console_write("ERR: invalid service id\n");
            return;
        }
        bool ok = false;
        if (ui_streq(argv[1], "pause")) ok = ksvc_pause((i32)id);
        else if (ui_streq(argv[1], "resume")) ok = ksvc_resume((i32)id);
        else if (ui_streq(argv[1], "restart")) ok = ksvc_restart((i32)id);
        else ok = ksvc_mark_error_code((i32)id, 0x4B534643U);
        ui_console_write(ok ? "OK: ksvc updated\n" : "ERR: ksvc update failed\n");
        return;
    }
    if (argc >= 2 && !ui_streq(argv[1], "status")) {
        ui_console_write("ERR: usage ksvc status | ksvc selftest | ksvc pause|resume|restart|fault <id>\n");
        return;
    }
    struct ksvc_snapshot_entry ks[KSVC_MAX_SERVICES];
    u32 n = ksvc_snapshot(ks, KSVC_MAX_SERVICES);
    ui_console_write("ID CORE PRI STATE      KIND CALLS ERR RST LERR PEND SENT RECV DROP LAST_T MAX_T TOTAL_T NAME\n");
    for (u32 i = 0; i < n; i++) {
        ui_console_hex_fixed(ks[i].id, 4);
        ui_console_write(" ");
        ui_console_u32_dec(ks[i].owner_core);
        ui_console_write(" ");
        ui_console_u32_dec(ks[i].priority);
        ui_console_write(" ");
        ui_console_write(ksvc_state_name(ks[i].state));
        ui_console_write(" ");
        ui_console_hex_fixed(ks[i].kind, 4);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].calls);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].errors);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].restarts);
        ui_console_write(" ");
        ui_console_u32_dec(ks[i].last_error);
        ui_console_write(" ");
        ui_console_u32_dec(ks[i].mailbox_pending);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].messages_sent);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].messages_recv);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].mailbox_drops);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].last_duration_ticks);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].max_duration_ticks);
        ui_console_write(" ");
        ui_console_u64_dec(ks[i].total_ticks);
        ui_console_write(" ");
        ui_console_write(ks[i].name);
        ui_console_write("\n");
    }
}

static void ui_cmd_irq(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(irq_diag_selftest() ? "IRQ selftest OK\n" : "IRQ selftest FAILED\n");
        return;
    }
    if (argc >= 3 && ui_streq(argv[1], "cntpns") && ui_streq(argv[2], "confirm")) {
        u64 before = 0, after = 0;
        u32 last = 0, d_ctlr = 0, c_ctlr = 0, unhandled = 0;
        bool ok = irq_cntpns_test(&before, &after, &last, &d_ctlr, &c_ctlr, &unhandled);
        ui_console_write(ok ? "IRQ cntpns OK" : "IRQ cntpns FAILED");
        ui_console_write(" timer_before=");
        ui_console_u64_dec(before);
        ui_console_write(" timer_after=");
        ui_console_u64_dec(after);
        ui_console_write(" last_intid=");
        ui_console_u32_dec(last);
        ui_console_write(" unhandled=");
        ui_console_u32_dec(unhandled);
        ui_console_write(" d_ctlr=");
        ui_console_hex_fixed(d_ctlr, 8);
        ui_console_write(" c_ctlr=");
        ui_console_hex_fixed(c_ctlr, 8);
        ui_console_write("\n");
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "probe")) {
        struct irq_gic_probe_snapshot p;
        irq_gic_probe_snapshot(&p);
        ui_console_write("irq probe current_driver_id=");
        ui_console_u32_dec(p.current_driver_id);
        ui_console_write(" count=");
        ui_console_u32_dec(p.count);
        ui_console_write("\nID D_BASE C_BASE D_CTLR D_TYPER D_IIDR C_CTLR C_PMR PLAUSIBLE\n");
        for (u32 i = 0; i < p.count; i++) {
            const struct irq_gic_probe_entry *e = &p.entries[i];
            ui_console_u32_dec(e->id);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicd_base, 16);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicc_base, 16);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicd_ctlr, 8);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicd_typer, 8);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicd_iidr, 8);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicc_ctlr, 8);
            ui_console_write(" ");
            ui_console_hex_fixed(e->gicc_pmr, 8);
            ui_console_write(e->plausible ? " yes\n" : " no\n");
        }
        return;
    }
    if (argc >= 2 && !ui_streq(argv[1], "status")) {
        ui_console_write("ERR: usage irq status | irq probe | irq selftest | irq cntpns confirm\n");
        return;
    }
    struct irq_diag_snapshot d;
    struct irq_hw_diag_snapshot hw;
    irq_diag_snapshot(&d);
    irq_hw_diag_snapshot(&hw);
    ui_console_write("irq total=");
    ui_console_u64_dec(d.total);
    ui_console_write(" handled=");
    ui_console_u64_dec(d.handled);
    ui_console_write(" unhandled=");
    ui_console_u64_dec(d.unhandled);
    ui_console_write(" spurious=");
    ui_console_u64_dec(d.spurious);
    ui_console_write(" timer=");
    ui_console_u64_dec(d.timer);
    ui_console_write(" last_intid=");
    ui_console_u32_dec(d.last_intid);
    ui_console_write(" last_core=");
    ui_console_u32_dec(d.last_core);
    ui_console_write(" last_tick=");
    ui_console_u64_dec(d.last_tick);
    ui_console_write("\nper_core=");
    for (u32 i = 0; i < 4; i++) {
        if (i) ui_console_write(",");
        ui_console_u64_dec(d.per_core[i]);
    }
    ui_console_write(" current_el=");
    ui_console_u64_dec((hw.current_el >> 2) & 3U);
    ui_console_write(" daif=");
    ui_console_hex_fixed(hw.daif, 8);
    ui_console_write(" vectors=");
    ui_console_write(hw.vectors_ready ? "ready" : "bad");
    ui_console_write(" gic=");
    ui_console_write(hw.gic_ready ? "ready" : "bad");
    ui_console_write(" timer=");
    ui_console_write(hw.timer_enabled ? "enabled" : "disabled");
    ui_console_write(" irq_masked=");
    ui_console_write(hw.irq_masked ? "yes" : "no");
    ui_console_write("\n");
}

/* ---- Opt-in CNTPNS PPI30 IRQ delivery test ----
 *
 * Wraps the dangerous GIC enable + timer fire path with the BCM2712
 * hardware watchdog so any wedge auto-resets the chip within 15 seconds
 * and A/B stage0 falls back to the known-good slot. After the spin
 * window we tear down everything we touched so leftover IRQ sources
 * can't wedge the kernel later (the previous attempt died ~14s after
 * the test returned because the timer kept firing).
 */
static u64 irq_cntpns_interval;
static volatile u32 irq_cntpns_singleshot_seen;

static void irq_cntpns_handler(void)
{
    u64 cval;
    __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(cval));
    cval += irq_cntpns_interval ? irq_cntpns_interval : 1000ULL;
    __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(cval));
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));
}

/* Single-shot handler: disables the timer entirely so the line drops and
 * no further IRQs fire. Bumps a counter we can read after the test.   */
static void irq_cntpns_singleshot_handler(void)
{
    /* IMASK=1 ENABLE=0 — drop the line definitively. */
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(2UL));
    irq_cntpns_singleshot_seen++;
    dsb();
}

static bool irq_cntpns_test(u64 *before_out, u64 *after_out, u32 *last_intid_out,
                            u32 *d_ctlr_out, u32 *c_ctlr_out, u32 *unhandled_out)
{
    const u64 gicd = 0x107FFF9000UL;
    const u64 gicc = 0x107FFFA000UL;
    const u32 intid = GIC_TIMER_NS_PHYS;
    const u32 reg = intid / 32U;
    const u32 bit = intid % 32U;
    const u32 mask = 1U << bit;
    struct irq_diag_snapshot before;
    struct irq_diag_snapshot after;
    irq_diag_snapshot(&before);

    /* Arm hw watchdog up front so any wedge auto-recovers. */
    watchdog_hw_arm_seconds(15);

    /* Snapshot pre-test GIC state so we can restore it. */
    __asm__ volatile("msr daifset, #2" ::: "memory");
    u64 old_gicd_base = gic_runtime_gicd_base();
    u64 old_gicc_base = gic_runtime_gicc_base();
    u32 old_pmr = mmio_read(gicc + 0x004);
    u32 old_d_ctlr = mmio_read(gicd + 0x000);
    u32 old_c_ctlr = mmio_read(gicc + 0x000);

    gic_select_bases(6, gicd, gicc);
    irq_register(intid, irq_cntpns_handler);

    u32 preg = intid / 4U;
    u32 pshift = (intid % 4U) * 8U;
    u32 pri = mmio_read(gicd + 0x400 + preg * 4U);
    u32 old_pri = pri;
    pri &= ~(0xFFU << pshift);
    pri |= (0x40U << pshift);
    mmio_write(gicd + 0x400 + preg * 4U, pri);

    mmio_write(gicd + 0x280 + reg * 4U, mask);   /* clear pending */
    mmio_write(gicd + 0x100 + reg * 4U, mask);   /* enable PPI 30 */
    mmio_write(gicc + 0x004, 0xF0U);             /* PMR */
    mmio_write(gicd + 0x000, old_d_ctlr | 1U);
    mmio_write(gicc + 0x000, old_c_ctlr | 1U);

    u64 freq, now;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
    irq_cntpns_interval = freq ? freq / 1000ULL : 1000ULL;
    if (irq_cntpns_interval == 0) irq_cntpns_interval = 1;
    __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + irq_cntpns_interval));
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));

    dsb();
    isb();
    __asm__ volatile("msr daifclr, #2" ::: "memory");

    u64 start = timer_monotonic_ms();
    while (timer_monotonic_ms() - start < 250ULL) {
        __asm__ volatile("yield");
    }

    /* Tear everything down so the kernel returns to a clean state. */
    __asm__ volatile("msr daifset, #2" ::: "memory");
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(2UL));   /* IMASK=1, ENABLE=0 */
    mmio_write(gicd + 0x180 + reg * 4U, mask);              /* disable PPI 30 */
    mmio_write(gicd + 0x280 + reg * 4U, mask);              /* clear pending */
    mmio_write(gicd + 0x400 + preg * 4U, old_pri);          /* restore priority */
    mmio_write(gicc + 0x004, old_pmr);                      /* restore PMR */
    mmio_write(gicd + 0x000, old_d_ctlr);                   /* restore GICD ctlr */
    mmio_write(gicc + 0x000, old_c_ctlr);                   /* restore GICC ctlr */
    irq_register(intid, NULL);
    gic_select_bases(1, old_gicd_base, old_gicc_base);
    dsb();
    isb();
    __asm__ volatile("msr daifclr, #2" ::: "memory");

    /* Test done, disable the hw watchdog so we don't reboot. */
    watchdog_hw_disable();

    irq_diag_snapshot(&after);
    if (before_out) *before_out = before.timer;
    if (after_out) *after_out = after.timer;
    if (last_intid_out) *last_intid_out = after.last_intid;
    if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
    if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);
    if (unhandled_out) *unhandled_out = (u32)(after.unhandled - before.unhandled);
    return after.timer > before.timer && after.last_intid == intid;
}

/* Step-by-step CNTPNS/PPI30 probe. Each `depth` value runs the setup up to
 * that step, samples GIC state, then tears the setup back down. If the
 * chip wedges at any step the hardware watchdog auto-resets within ~15s.
 *
 *   depth=0   read GICD/GICC state only (purely read)
 *   depth=1   also read GICC_IAR (acks any pending pre-existing interrupt)
 *   depth=2   also write PMR=0xF0
 *   depth=3   also write GICD_CTLR|=1, GICC_CTLR|=1 (no PPI enabled)
 *   depth=4   also configure + enable PPI 30 in distributor
 *   depth=5   also program CNTP_CVAL_EL0 (timer counter set, CTL=0)
 *   depth=6   also write CNTP_CTL_EL0=1 (timer asserts pending, DAIF masked)
 *   depth=7   also unmask DAIF for a brief spin, then re-mask
 *   depth=8   bare DAIF unmask only, no GIC writes (sees stale pending)
 *   depth=9   step 4 setup (GIC+PPI 30 enable) + DAIF unmask, no timer
 *   depth=10  full CNTV (PPI 27) test instead of CNTPNS
 *   depth=11  single-shot CNTPNS - unmask DAIF for ONE IRQ then re-mask
 *   depth=12  setup CNTPNS, wait 5ms with DAIF MASKED, snapshot HPPIR/ISPEND
 *   depth=13  fire CNTPNS, manually IAR+EOIR with DAIF masked (no vector)
 *   depth=14  asm-only IRQ vector path: mask CNTP, IAR, EOIR, eret
 */
static u32 irq_cntpns_step(u32 depth, u32 *d_ctlr_out, u32 *c_ctlr_out, u32 *pmr_out,
                           u32 *ispend_out, u32 *isenable_out, u32 *iar_out)
{
    const u64 gicd = 0x107FFF9000UL;
    const u64 gicc = 0x107FFFA000UL;
    const u32 intid = GIC_TIMER_NS_PHYS;
    const u32 reg = intid / 32U;
    const u32 bit = intid % 32U;
    const u32 mask = 1U << bit;

    watchdog_hw_arm_seconds(15);
    __asm__ volatile("msr daifset, #2" ::: "memory");

    /* The IRQ vector dispatches via gic_acknowledge/gic_end_of_interrupt
     * which use the runtime GIC base. We must repoint them at the real
     * Pi 5 GIC (0x107FFF9000/A000) before any IRQ can fire, otherwise
     * ack/EOI hit dead address space and the line stays asserted →
     * infinite IRQ loop. Snapshot the old base so we can restore. */
    u64 saved_gicd_base = gic_runtime_gicd_base();
    u64 saved_gicc_base = gic_runtime_gicc_base();
    u32 saved_gicid    = gic_runtime_id();
    gic_select_bases(6, gicd, gicc);

    u32 done = 0;

    /* Bare DAIF unmask probe. Skip ALL GIC writes — just see if any
     * stale pending interrupt fires when we open the gate. */
    if (depth == 8) {
        u32 d_ctlr_now = mmio_read(gicd + 0x000);
        u32 c_ctlr_now = mmio_read(gicc + 0x000);
        u32 pmr_now    = mmio_read(gicc + 0x004);
        u32 hppir = mmio_read(gicc + 0x018);
        if (d_ctlr_out)  *d_ctlr_out  = d_ctlr_now;
        if (c_ctlr_out)  *c_ctlr_out  = c_ctlr_now;
        if (pmr_out)     *pmr_out     = pmr_now;
        if (ispend_out)  *ispend_out  = hppir;
        if (isenable_out)*isenable_out= 0;
        if (iar_out)     *iar_out     = 0;
        dsb(); isb();
        __asm__ volatile("msr daifclr, #2" ::: "memory");
        u64 start = timer_monotonic_ms();
        while (timer_monotonic_ms() - start < 5ULL) {
            __asm__ volatile("yield");
        }
        __asm__ volatile("msr daifset, #2" ::: "memory");
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return 8;
    }

    /* GIC + PPI 30 enable + DAIF unmask, no timer source (no fire expected). */
    if (depth == 9) {
        u32 sav_d = mmio_read(gicd + 0x000);
        u32 sav_c = mmio_read(gicc + 0x000);
        u32 sav_pmr = mmio_read(gicc + 0x004);
        u32 sav_isena = mmio_read(gicd + 0x100 + reg * 4U);
        u32 sav_igroup = mmio_read(gicd + 0x080 + reg * 4U);
        u32 sav_pri = mmio_read(gicd + 0x400 + (intid / 4U) * 4U);
        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup | mask);
        u32 newp = sav_pri;
        newp &= ~(0xFFU << ((intid % 4U) * 8U));
        newp |= (0x40U << ((intid % 4U) * 8U));
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, newp);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x100 + reg * 4U, mask);
        mmio_write(gicc + 0x004, 0xF0U);
        mmio_write(gicd + 0x000, sav_d | 1U);
        mmio_write(gicc + 0x000, sav_c | 1U);
        if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
        if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);
        if (pmr_out)    *pmr_out    = mmio_read(gicc + 0x004);
        if (ispend_out) *ispend_out = mmio_read(gicd + 0x080 + reg * 4U);
        if (isenable_out) *isenable_out = mmio_read(gicd + 0x100 + reg * 4U);
        if (iar_out)    *iar_out    = mmio_read(gicc + 0x018);
        dsb(); isb();
        __asm__ volatile("msr daifclr, #2" ::: "memory");
        u64 start = timer_monotonic_ms();
        while (timer_monotonic_ms() - start < 5ULL) {
            __asm__ volatile("yield");
        }
        __asm__ volatile("msr daifset, #2" ::: "memory");
        /* teardown */
        mmio_write(gicd + 0x180 + reg * 4U, mask);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, sav_pri);
        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup);
        mmio_write(gicc + 0x004, sav_pmr);
        mmio_write(gicd + 0x000, sav_d);
        mmio_write(gicc + 0x000, sav_c);
        (void)sav_isena;
        dsb(); isb();
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return 9;
    }

    /* Full CNTV (PPI 27) test instead of CNTPNS. */
    if (depth == 10) {
        const u32 vintid = GIC_TIMER_VIRT;
        const u32 vreg = vintid / 32U;
        const u32 vbit = vintid % 32U;
        const u32 vmask = 1U << vbit;
        const u32 vpreg = vintid / 4U;
        const u32 vpshift = (vintid % 4U) * 8U;
        u32 sav_d = mmio_read(gicd + 0x000);
        u32 sav_c = mmio_read(gicc + 0x000);
        u32 sav_pmr = mmio_read(gicc + 0x004);
        u32 sav_isena = mmio_read(gicd + 0x100 + vreg * 4U);
        u32 sav_igroup = mmio_read(gicd + 0x080 + vreg * 4U);
        u32 sav_pri = mmio_read(gicd + 0x400 + vpreg * 4U);
        u64 sav_cval = 0, sav_ctl = 0;
        __asm__ volatile("mrs %0, cntv_cval_el0" : "=r"(sav_cval));
        __asm__ volatile("mrs %0, cntv_ctl_el0" : "=r"(sav_ctl));

        mmio_write(gicd + 0x080 + vreg * 4U, sav_igroup | vmask);
        u32 newp = sav_pri;
        newp &= ~(0xFFU << vpshift);
        newp |= (0x40U << vpshift);
        mmio_write(gicd + 0x400 + vpreg * 4U, newp);
        mmio_write(gicd + 0x280 + vreg * 4U, vmask);
        mmio_write(gicd + 0x100 + vreg * 4U, vmask);
        mmio_write(gicc + 0x004, 0xF0U);
        mmio_write(gicd + 0x000, sav_d | 1U);
        mmio_write(gicc + 0x000, sav_c | 1U);

        u64 freq, now;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        __asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
        u64 interval = freq ? freq / 1000ULL : 1000ULL;
        if (interval == 0) interval = 1;
        irq_cntpns_interval = interval;
        __asm__ volatile("msr cntv_cval_el0, %0" :: "r"(now + interval));
        irq_register(vintid, irq_cntpns_handler);
        __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(1UL));

        if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
        if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);
        if (pmr_out)    *pmr_out    = mmio_read(gicc + 0x004);
        if (ispend_out) *ispend_out = mmio_read(gicd + 0x080 + vreg * 4U);
        if (isenable_out) *isenable_out = mmio_read(gicd + 0x100 + vreg * 4U);

        dsb(); isb();
        __asm__ volatile("msr daifclr, #2" ::: "memory");
        u64 start = timer_monotonic_ms();
        while (timer_monotonic_ms() - start < 10ULL) {
            __asm__ volatile("yield");
        }
        __asm__ volatile("msr daifset, #2" ::: "memory");
        struct irq_diag_snapshot after_v;
        irq_diag_snapshot(&after_v);
        if (iar_out) *iar_out = after_v.last_intid;

        /* CNTV teardown */
        __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(2UL));
        irq_register(vintid, NULL);
        __asm__ volatile("msr cntv_cval_el0, %0" :: "r"(sav_cval));
        __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(sav_ctl));
        mmio_write(gicd + 0x180 + vreg * 4U, vmask);
        mmio_write(gicd + 0x280 + vreg * 4U, vmask);
        mmio_write(gicd + 0x400 + vpreg * 4U, sav_pri);
        mmio_write(gicd + 0x080 + vreg * 4U, sav_igroup);
        mmio_write(gicc + 0x004, sav_pmr);
        mmio_write(gicd + 0x000, sav_d);
        mmio_write(gicc + 0x000, sav_c);
        (void)sav_isena;
        dsb(); isb();
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return 10;
    }

    /* Single-shot CNTPNS test: arm, unmask DAIF for 1ms, mask, sample. */
    if (depth == 11) {
        u32 sav_d = mmio_read(gicd + 0x000);
        u32 sav_c = mmio_read(gicc + 0x000);
        u32 sav_pmr = mmio_read(gicc + 0x004);
        u32 sav_isena = mmio_read(gicd + 0x100 + reg * 4U);
        u32 sav_igroup = mmio_read(gicd + 0x080 + reg * 4U);
        u32 sav_pri = mmio_read(gicd + 0x400 + (intid / 4U) * 4U);
        u64 sav_cval = 0, sav_ctl = 0;
        __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(sav_cval));
        __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(sav_ctl));

        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup | mask);
        u32 newp = sav_pri;
        newp &= ~(0xFFU << ((intid % 4U) * 8U));
        newp |= (0x40U << ((intid % 4U) * 8U));
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, newp);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x100 + reg * 4U, mask);
        mmio_write(gicc + 0x004, 0xF0U);
        mmio_write(gicd + 0x000, sav_d | 1U);
        mmio_write(gicc + 0x000, sav_c | 1U);

        /* Sample initial state. */
        if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
        if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);
        if (pmr_out)    *pmr_out    = mmio_read(gicc + 0x004);
        if (ispend_out) *ispend_out = mmio_read(gicc + 0x018); /* HPPIR */
        if (isenable_out) *isenable_out = mmio_read(gicd + 0x100 + reg * 4U);

        /* True single-shot: handler masks CNTP immediately so the line
         * drops and only ONE IRQ ever fires. If even this wedges, the
         * IRQ vector path itself is broken for this GIC base. */
        irq_register(intid, irq_cntpns_singleshot_handler);
        irq_cntpns_singleshot_seen = 0;

        /* Arm timer 50us in the future with 50us interval. */
        u64 freq, now;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
        u64 ticks = freq ? (freq * 50ULL) / 1000000ULL : 50ULL;
        if (ticks == 0) ticks = 1;
        irq_cntpns_interval = ticks;
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + ticks));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));

        struct irq_diag_snapshot before;
        irq_diag_snapshot(&before);

        dsb(); isb();
        /* Brief 1ms unmask. */
        __asm__ volatile("msr daifclr, #2" ::: "memory");
        u64 t0 = timer_monotonic_ms();
        while (timer_monotonic_ms() - t0 < 1ULL) {
            __asm__ volatile("yield");
        }
        __asm__ volatile("msr daifset, #2" ::: "memory");

        /* Mask CNTP immediately so it doesn't keep firing. */
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(2UL));

        struct irq_diag_snapshot after;
        irq_diag_snapshot(&after);
        if (iar_out) *iar_out = (u32)(after.total - before.total);

        /* Teardown. */
        irq_register(intid, NULL);
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(sav_cval));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(sav_ctl));
        mmio_write(gicd + 0x180 + reg * 4U, mask);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, sav_pri);
        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup);
        mmio_write(gicc + 0x004, sav_pmr);
        mmio_write(gicd + 0x000, sav_d);
        mmio_write(gicc + 0x000, sav_c);
        (void)sav_isena;
        dsb(); isb();
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return 11;
    }

    /* CNTPNS armed, DAIF still masked the whole time. Wait 5ms then
     * snapshot HPPIR + ISPEND. Tells us whether the GIC actually latched
     * the timer interrupt and what group/priority it sees. */
    if (depth == 12) {
        u32 sav_d = mmio_read(gicd + 0x000);
        u32 sav_c = mmio_read(gicc + 0x000);
        u32 sav_pmr = mmio_read(gicc + 0x004);
        u32 sav_igroup = mmio_read(gicd + 0x080 + reg * 4U);
        u32 sav_pri = mmio_read(gicd + 0x400 + (intid / 4U) * 4U);
        u64 sav_cval = 0, sav_ctl = 0;
        __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(sav_cval));
        __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(sav_ctl));

        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup | mask);
        u32 newp = sav_pri;
        newp &= ~(0xFFU << ((intid % 4U) * 8U));
        newp |= (0x40U << ((intid % 4U) * 8U));
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, newp);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x100 + reg * 4U, mask);
        mmio_write(gicc + 0x004, 0xF0U);
        mmio_write(gicd + 0x000, sav_d | 1U);
        mmio_write(gicc + 0x000, sav_c | 1U);

        u64 freq, now;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
        u64 ticks = freq ? (freq * 50ULL) / 1000000ULL : 50ULL;
        if (ticks == 0) ticks = 1;
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + ticks));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));

        /* Spin 5ms with DAIF STILL MASKED. Timer fires, GIC latches it. */
        u64 t0 = timer_monotonic_ms();
        while (timer_monotonic_ms() - t0 < 5ULL) {
            __asm__ volatile("yield");
        }

        /* Snapshot post-fire state without unmasking IRQ. */
        u64 cntp_ctl_now = 0;
        __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(cntp_ctl_now));
        if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
        if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);
        if (pmr_out)    *pmr_out    = (u32)cntp_ctl_now;
        if (ispend_out) *ispend_out = mmio_read(gicd + 0x200 + reg * 4U); /* ISPENDR */
        if (isenable_out) *isenable_out = mmio_read(gicd + 0x100 + reg * 4U);
        if (iar_out)    *iar_out    = mmio_read(gicc + 0x018); /* HPPIR */

        /* Teardown. */
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(2UL));
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(sav_cval));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(sav_ctl));
        mmio_write(gicd + 0x180 + reg * 4U, mask);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, sav_pri);
        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup);
        mmio_write(gicc + 0x004, sav_pmr);
        mmio_write(gicd + 0x000, sav_d);
        mmio_write(gicc + 0x000, sav_c);
        dsb(); isb();
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return 12;
    }

    /* Fire CNTPNS, then manually IAR + EOIR with DAIF masked. No IRQ
     * vector involvement. If this clears the interrupt, GIC ack/EOI work
     * and the wedge is in our vector path. Output:
     *   ispend = ISPENDR before ack
     *   iar    = the IAR read (which acks)
     *   isenable = ISPENDR after EOI (should clear bit 30)
     *   pmr    = HPPIR after EOI (should return spurious 0x3FF) */
    if (depth == 13) {
        u32 sav_d = mmio_read(gicd + 0x000);
        u32 sav_c = mmio_read(gicc + 0x000);
        u32 sav_pmr = mmio_read(gicc + 0x004);
        u32 sav_igroup = mmio_read(gicd + 0x080 + reg * 4U);
        u32 sav_pri = mmio_read(gicd + 0x400 + (intid / 4U) * 4U);
        u64 sav_cval = 0, sav_ctl = 0;
        __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(sav_cval));
        __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(sav_ctl));

        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup | mask);
        u32 newp = sav_pri;
        newp &= ~(0xFFU << ((intid % 4U) * 8U));
        newp |= (0x40U << ((intid % 4U) * 8U));
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, newp);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x100 + reg * 4U, mask);
        mmio_write(gicc + 0x004, 0xF0U);
        mmio_write(gicd + 0x000, sav_d | 1U);
        mmio_write(gicc + 0x000, sav_c | 1U);

        u64 freq, now;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
        u64 ticks = freq ? (freq * 50ULL) / 1000000ULL : 50ULL;
        if (ticks == 0) ticks = 1;
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + ticks));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));

        /* Wait long enough for timer to assert. */
        u64 t0 = timer_monotonic_ms();
        while (timer_monotonic_ms() - t0 < 2ULL) {
            __asm__ volatile("yield");
        }

        u32 pre_ispend = mmio_read(gicd + 0x200 + reg * 4U);
        /* Mask CNTP BEFORE ack so it doesn't re-pend. */
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(2UL));
        u32 ack_iar = mmio_read(gicc + 0x00C);
        mmio_write(gicc + 0x010, ack_iar); /* EOI */
        dsb();
        u32 post_ispend = mmio_read(gicd + 0x200 + reg * 4U);
        u32 post_hppir = mmio_read(gicc + 0x018);

        if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
        if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);
        if (pmr_out)    *pmr_out    = post_hppir;
        if (ispend_out) *ispend_out = pre_ispend;
        if (isenable_out) *isenable_out = post_ispend;
        if (iar_out)    *iar_out    = ack_iar;

        /* Teardown. */
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(sav_cval));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(sav_ctl));
        mmio_write(gicd + 0x180 + reg * 4U, mask);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, sav_pri);
        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup);
        mmio_write(gicc + 0x004, sav_pmr);
        mmio_write(gicd + 0x000, sav_d);
        mmio_write(gicc + 0x000, sav_c);
        dsb(); isb();
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return 13;
    }

    /* Asm-only IRQ vector test. The normal irq_handler checks a global flag
     * before SAVE_CONTEXT; when armed, it masks CNTP, reads IAR, writes EOIR,
     * increments a counter, and erets without calling C. This isolates vector
     * entry + GIC CPU-interface delivery from the C save/dispatch path.
     * Output:
     *   iar = asm minimal handler count
     *   pmr = asm minimal handler last IAR
     *   ispend/isenable = ISPENDR before/after the unmask window */
    if (depth == 14) {
        u32 sav_d = mmio_read(gicd + 0x000);
        u32 sav_c = mmio_read(gicc + 0x000);
        u32 sav_pmr = mmio_read(gicc + 0x004);
        u32 sav_igroup = mmio_read(gicd + 0x080 + reg * 4U);
        u32 sav_pri = mmio_read(gicd + 0x400 + (intid / 4U) * 4U);
        u64 sav_cval = 0, sav_ctl = 0;
        __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(sav_cval));
        __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(sav_ctl));

        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup | mask);
        u32 newp = sav_pri;
        newp &= ~(0xFFU << ((intid % 4U) * 8U));
        newp |= (0x40U << ((intid % 4U) * 8U));
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, newp);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x100 + reg * 4U, mask);
        mmio_write(gicc + 0x004, 0xF0U);
        mmio_write(gicd + 0x000, sav_d | 1U);
        mmio_write(gicc + 0x000, sav_c | 1U);

        if (d_ctlr_out) *d_ctlr_out = mmio_read(gicd + 0x000);
        if (c_ctlr_out) *c_ctlr_out = mmio_read(gicc + 0x000);

        irq_vector_minimal_arm(true);

        u64 freq, now;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
        u64 ticks = freq ? (freq * 50ULL) / 1000000ULL : 50ULL;
        if (ticks == 0) ticks = 1;
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + ticks));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));
        dsb(); isb();

        if (ispend_out) *ispend_out = mmio_read(gicd + 0x200 + reg * 4U);
        __asm__ volatile("msr daifclr, #2" ::: "memory");
        u64 t0 = timer_monotonic_ms();
        while (timer_monotonic_ms() - t0 < 2ULL) {
            __asm__ volatile("yield");
        }
        __asm__ volatile("msr daifset, #2" ::: "memory");
        if (isenable_out) *isenable_out = mmio_read(gicd + 0x200 + reg * 4U);

        u32 min_count = 0, min_iar = 0;
        irq_vector_minimal_snapshot(&min_count, &min_iar);
        if (pmr_out) *pmr_out = min_iar;
        if (iar_out) *iar_out = min_count;

        irq_vector_minimal_arm(false);
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(sav_cval));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(sav_ctl));
        mmio_write(gicd + 0x180 + reg * 4U, mask);
        mmio_write(gicd + 0x280 + reg * 4U, mask);
        mmio_write(gicd + 0x400 + (intid / 4U) * 4U, sav_pri);
        mmio_write(gicd + 0x080 + reg * 4U, sav_igroup);
        mmio_write(gicc + 0x004, sav_pmr);
        mmio_write(gicd + 0x000, sav_d);
        mmio_write(gicc + 0x000, sav_c);
        dsb(); isb();
        gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
        watchdog_hw_disable();
        return depth;
    }

    u32 old_d_ctlr = mmio_read(gicd + 0x000);
    u32 old_c_ctlr = mmio_read(gicc + 0x000);
    u32 old_pmr    = mmio_read(gicc + 0x004);
    u32 old_ispend = mmio_read(gicd + 0x200 + reg * 4U);
    u32 old_isena  = mmio_read(gicd + 0x100 + reg * 4U);
    u32 old_igroup = mmio_read(gicd + 0x080 + reg * 4U);
    u32 old_gicd_typer = mmio_read(gicd + 0x004);

    if (d_ctlr_out)  *d_ctlr_out  = old_d_ctlr;
    if (c_ctlr_out)  *c_ctlr_out  = old_c_ctlr;
    if (pmr_out)     *pmr_out     = old_pmr;
    /* Pack diagnostics: ispend_out = igroup, isenable_out = isenable, iar_out = typer initially */
    if (ispend_out)  *ispend_out  = old_igroup;
    if (isenable_out)*isenable_out= old_isena;
    if (iar_out)     *iar_out     = old_gicd_typer;
    (void)old_ispend;
    done = 0;

    u32 old_pri = 0;
    u32 preg = intid / 4U;
    u32 pshift = (intid % 4U) * 8U;

    if (depth >= 1) {
        u32 iar = mmio_read(gicc + 0x00C);
        if (iar_out) *iar_out = iar;
        if ((iar & 0x3FF) != 0x3FF)
            mmio_write(gicc + 0x010, iar);
        done = 1;
    }
    if (depth >= 2) {
        mmio_write(gicc + 0x004, 0xF0U);
        done = 2;
    }
    if (depth >= 3) {
        mmio_write(gicd + 0x000, old_d_ctlr | 1U);
        mmio_write(gicc + 0x000, old_c_ctlr | 1U);
        done = 3;
    }
    if (depth >= 4) {
        /* Try to move PPI 30 to Group 1 NS before enabling. In non-secure
         * GIC-400 with security extensions enabled, IGROUPR is RAZ/WI from
         * non-secure unless GICD_CTLR.DS is set. We try anyway and rely on
         * the readback in the snapshot to tell us whether ATF left it
         * writable. Without Group 1 NS, the timer interrupt is delivered
         * to EL3 as FIQ instead of EL1 as IRQ. */
        u32 cur_group = mmio_read(gicd + 0x080 + reg * 4U);
        mmio_write(gicd + 0x080 + reg * 4U, cur_group | mask);
        old_pri = mmio_read(gicd + 0x400 + preg * 4U);
        u32 pri = old_pri;
        pri &= ~(0xFFU << pshift);
        pri |= (0x40U << pshift);
        mmio_write(gicd + 0x400 + preg * 4U, pri);
        mmio_write(gicd + 0x280 + reg * 4U, mask);     /* clear pending */
        mmio_write(gicd + 0x100 + reg * 4U, mask);     /* enable PPI 30 */
        /* Read back current registers as live snapshot. */
        if (iar_out) *iar_out = mmio_read(gicd + 0x080 + reg * 4U);  /* IGROUPR0 readback */
        if (ispend_out) *ispend_out = mmio_read(gicd + 0x100 + reg * 4U); /* ISENABLER readback */
        done = 4;
    }
    u64 prev_cval = 0, prev_ctl = 0;
    if (depth >= 5) {
        __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(prev_cval));
        __asm__ volatile("mrs %0, cntp_ctl_el0" : "=r"(prev_ctl));
        u64 freq, now;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
        u64 interval = freq ? freq / 1000ULL : 1000ULL;
        if (interval == 0) interval = 1;
        irq_cntpns_interval = interval;
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + interval));
        done = 5;
    }
    if (depth >= 6) {
        irq_register(intid, irq_cntpns_handler);
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));   /* ENABLE, no mask */
        done = 6;
    }
    if (depth >= 7) {
        dsb();
        isb();
        __asm__ volatile("msr daifclr, #2" ::: "memory");
        u64 start = timer_monotonic_ms();
        while (timer_monotonic_ms() - start < 5ULL) {
            __asm__ volatile("yield");
        }
        __asm__ volatile("msr daifset, #2" ::: "memory");
        done = 7;
    }

    /* Teardown — reverse order. */
    if (depth >= 6) {
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(2UL));   /* IMASK=1 */
        irq_register(intid, NULL);
    }
    if (depth >= 5) {
        __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(prev_cval));
        __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(prev_ctl));
    }
    if (depth >= 4) {
        mmio_write(gicd + 0x180 + reg * 4U, mask);     /* disable PPI 30 */
        mmio_write(gicd + 0x280 + reg * 4U, mask);     /* clear pending */
        mmio_write(gicd + 0x400 + preg * 4U, old_pri);
        mmio_write(gicd + 0x080 + reg * 4U, old_igroup); /* restore IGROUPR0 */
    }
    if (depth >= 3) {
        mmio_write(gicd + 0x000, old_d_ctlr);
        mmio_write(gicc + 0x000, old_c_ctlr);
    }
    if (depth >= 2) {
        mmio_write(gicc + 0x004, old_pmr);
    }

    dsb();
    isb();
    __asm__ volatile("msr daifclr, #2" ::: "memory");

    gic_select_bases(saved_gicid, saved_gicd_base, saved_gicc_base);
    watchdog_hw_disable();
    return done;
}

static void ui_cmd_abi(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(abi_selftest() ? "ABI selftest OK\n" : "ABI selftest FAILED\n");
        return;
    }
    if (argc >= 2 && !ui_streq(argv[1], "status")) {
        ui_console_write("ERR: usage abi status | abi selftest\n");
        return;
    }
    struct abi_status a;
    abi_status(&a);
    ui_console_write("abi stage=");
    ui_console_write(abi_stage_name(a.stage));
    ui_console_write(" direct_kpi=");
    ui_console_write(a.direct_kpi ? "yes" : "no");
    ui_console_write(" ksvc_registry=");
    ui_console_write(a.ksvc_registry ? "yes" : "no");
    ui_console_write(" ksvc_mailboxes=");
    ui_console_write(a.ksvc_mailboxes ? "yes" : "no");
    ui_console_write(" ksvc_callbacks=");
    ui_console_write(a.ksvc_callbacks ? "yes" : "no");
    ui_console_write(" svc_trap=");
    ui_console_write(a.svc_trap_ready ? "ready" : "pending");
    ui_console_write(" svc_calls=");
    ui_console_u64_dec(a.svc_calls);
    ui_console_write(" svc_bad=");
    ui_console_u64_dec(a.svc_bad_calls);
    ui_console_write(" entry_contract=");
    ui_console_write(a.el0_entry_contract ? "ready" : "pending");
    ui_console_write(" el0=");
    ui_console_write(a.el0_ready ? "ready" : "pending");
    ui_console_write(" ttbr_split=");
    ui_console_write(a.user_ttbr_split ? "yes" : "no");
    ui_console_write(" entry_flags=");
    ui_console_hex_fixed(a.el0_entry_flags, 8);
    ui_console_write(" el0_spsr=");
    ui_console_hex_fixed(a.el0_spsr, 8);
    ui_console_write(" kapi_size=");
    ui_console_u32_dec(a.kernel_api_version);
    ui_console_write(" pending_steps=");
    ui_console_u32_dec(a.pending_steps);
    ui_console_write("\n");
}

static void ui_print_tensor_status(void)
{
    struct tensor_status ts;
    tensor_status(&ts);
    ui_console_write("tensor v3d_available=");
    ui_console_write(ts.v3d_available ? "yes" : "no");
    ui_console_write(" dispatch=");
    ui_console_write(ts.v3d_dispatch_supported ? "yes" : "no");
    ui_console_write(" fallback=");
    ui_console_write(ts.qpu_fallback ? "yes" : "no");
    ui_console_write(" any_kernel=");
    ui_console_write(ts.any_kernel_bound ? "yes" : "no");
    ui_console_write(" ready_mask=");
    ui_console_hex_fixed(ts.ready_mask, 8);
    ui_console_write(" disabled_mask=");
    ui_console_hex_fixed(ts.disabled_mask, 8);
    ui_console_write(" ident0=");
    ui_console_hex_fixed(ts.ident0, 8);
    ui_console_write(" ident1=");
    ui_console_hex_fixed(ts.ident1, 8);
    ui_console_write(" ident2=");
    ui_console_hex_fixed(ts.ident2, 8);
    ui_console_write("\n");
}

static void ui_cmd_tensor(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(tensor_selftest() ? "Tensor selftest OK\n" : "Tensor selftest FAILED\n");
        ui_print_tensor_status();
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_tensor_status();
        return;
    }
    ui_console_write("ERR: usage qpu status | qpu selftest | tensor status | tensor selftest\n");
}

static void ui_print_dns_status(void)
{
    struct dns_async_status ds;
    dns_async_status(&ds);
    ui_console_write("dns state=");
    fb_printf("%u", ds.state);
    ui_console_write(" server=");
    ui_console_ip(ds.server_ip);
    ui_console_write(" result=");
    ui_console_ip(ds.result_ip);
    ui_console_write(" attempts=");
    fb_printf("%u", ds.attempts);
    ui_console_write(" error=");
    fb_printf("%u", ds.last_error);
    ui_console_write(" rx=");
    ui_console_u32_dec(ds.rx_total);
    ui_console_write(" server_rx=");
    ui_console_u32_dec(ds.rx_server);
    ui_console_write(" ignored=");
    ui_console_u32_dec(ds.rx_ignored);
    ui_console_write(" rejected=");
    ui_console_u32_dec(ds.rx_rejected);
    ui_console_write(" ok_rx=");
    ui_console_u32_dec(ds.rx_ok);
    ui_console_write("\nlast_rx=");
    ui_console_ip4(ds.last_rx_src_ip);
    ui_console_write(":");
    ui_console_u32_dec(ds.last_rx_src_port);
    ui_console_write(" -> ");
    ui_console_u32_dec(ds.last_rx_dst_port);
    ui_console_write(" len=");
    ui_console_u32_dec(ds.last_rx_len);
    ui_console_write("\nhostname=");
    ui_console_write(ds.hostname);
    ui_console_write("\n");
}

static void ui_cmd_dns(u32 argc, char **argv)
{
    if (argc >= 3 && ui_streq(argv[1], "resolve")) {
        ui_console_write(dns_resolve_async_start(argv[2]) ?
                         "DNS resolve started\n" :
                         "ERR: dns resolve start failed\n");
        ui_print_dns_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "flush")) {
        dns_cache_flush();
        ui_console_write("DNS cache flushed\n");
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_dns_status();
        return;
    }
    ui_console_write("ERR: usage dns resolve <hostname> | dns status | dns flush\n");
}

static void ui_console_print_netstat(void)
{
    tcp_snapshot_entry_t snap[TCP_MAX_CONNECTIONS];
    const tcp_diag_t *td = tcp_diag();
    u32 n = tcp_snapshot(snap, TCP_MAX_CONNECTIONS);
    u64 rx_drop = 0, tx_drop = 0;
    nic_filter_stats(&rx_drop, &tx_drop);
    ui_console_write("CONN ST        LOCAL                  REMOTE                 OWNER          PEND RX TX RETRY\n");
    for (u32 i = 0; i < n; i++) {
        tcp_snapshot_entry_t *e = &snap[i];
        ui_console_u32_dec((u32)e->conn);
        ui_console_write("    ");
        ui_console_write(tcp_state_name(e->state));
        ui_console_write(" ");
        ui_console_ip4(e->local_ip);
        ui_console_write(":");
        ui_console_u32_dec(e->local_port);
        ui_console_write("    ");
        if (e->remote_ip) {
            ui_console_ip4(e->remote_ip);
            ui_console_write(":");
            ui_console_u32_dec(e->remote_port);
        } else {
            ui_console_write("*:*");
        }

        ui_console_write("    ");
        ui_console_write(tcp_owner_label(e->local_port));
        ui_console_write("    ");
        ui_console_u32_dec(e->pending_count);
        ui_console_write(" ");
        ui_console_u32_dec(e->rx_used);
        ui_console_write(" ");
        ui_console_u32_dec(e->tx_used);
        ui_console_write(" ");
        ui_console_u32_dec(e->retries);
        ui_console_write("\n");
    }
    ui_console_write("syn=");
    ui_console_u32_dec((u32)td->syn_seen);
    ui_console_write(" synack=");
    ui_console_u32_dec((u32)td->synack_sent);
    ui_console_write(" accepted=");
    ui_console_u32_dec((u32)td->accepted);
    ui_console_write(" fw_rx_drop=");
    ui_console_u32_dec((u32)rx_drop);
    ui_console_write(" fw_tx_drop=");
    ui_console_u32_dec((u32)tx_drop);
    ui_console_write(" act_syn=");
    ui_console_u32_dec((u32)td->active_syn_sent);
    ui_console_write(" act_synack=");
    ui_console_u32_dec((u32)td->active_synack_seen);
    ui_console_write(" act_est=");
    ui_console_u32_dec((u32)td->active_established);
    ui_console_write(" act_rst=");
    ui_console_u32_dec((u32)td->active_rst);
    ui_console_write(" act_badack=");
    ui_console_u32_dec((u32)td->active_bad_ack);
    ui_console_write(" act_timeout=");
    ui_console_u32_dec((u32)td->active_timeout);
    ui_console_write("\nactive_last ");
    ui_console_ip4(td->active_last_local_ip);
    ui_console_write(":");
    ui_console_u32_dec(td->active_last_local_port);
    ui_console_write(" -> ");
    ui_console_ip4(td->active_last_remote_ip);
    ui_console_write(":");
    ui_console_u32_dec(td->active_last_remote_port);
    ui_console_write(" state=");
    ui_console_write(tcp_state_name(td->active_last_state));
    ui_console_write(" retries=");
    ui_console_u32_dec(td->active_last_retries);
    ui_console_write("\n");
}

static void ui_firewall_print_ip_spec(u32 flags, bool src, const nic_filter_rule_t *r)
{
    u32 exact = src ? NIC_FILTER_IP_FROM : NIC_FILTER_IP_TO;
    u32 range = src ? NIC_FILTER_IP_FROM_RANGE : NIC_FILTER_IP_TO_RANGE;
    u32 ip = src ? r->ip_from : r->ip_to;
    u32 mask = src ? r->ip_from_mask : r->ip_to_mask;
    u32 end = src ? r->ip_from_end : r->ip_to_end;
    if (flags & range) {
        ui_console_ip4(ip);
        ui_console_write("-");
        ui_console_ip4(end);
    } else if (flags & exact) {
        ui_console_ip4(ip);
        if (mask != 0) {
            ui_console_write("/");
            ui_console_ip4(mask);
        }
    } else {
        ui_console_write("any");
    }
}

static void ui_firewall_print_rule(u32 i, const nic_filter_rule_t *r)
{
    ui_console_u32_dec(i);
    ui_console_write(": ");
    ui_console_write(r->action == NIC_FILTER_ALLOW ? "allow " : "deny ");
    if (r->direction == NIC_FILTER_DIR_IN) ui_console_write("in ");
    else if (r->direction == NIC_FILTER_DIR_OUT) ui_console_write("out ");
    else ui_console_write("both ");
    if (r->flags & NIC_FILTER_ETHERTYPE) {
        if (r->ethertype == ETH_P_ARP) ui_console_write("arp ");
        else if (r->ethertype == ETH_P_IP) {
            if (r->flags & NIC_FILTER_IP_PROTO) {
                if (r->ip_proto == IP_PROTO_TCP) ui_console_write("tcp ");
                else if (r->ip_proto == IP_PROTO_UDP) ui_console_write("udp ");
                else if (r->ip_proto == IP_PROTO_ICMP) ui_console_write("icmp ");
                else ui_console_write("ip ");
            } else {
                ui_console_write("ip ");
            }
        }
    } else {
        ui_console_write("any ");
    }
    ui_console_write("src=");
    ui_firewall_print_ip_spec(r->flags, true, r);
    ui_console_write(" dst=");
    ui_firewall_print_ip_spec(r->flags, false, r);
    if (r->flags & NIC_FILTER_TCP_PORT_TO) {
        ui_console_write(" tcp_dport=");
        ui_console_u32_dec(r->tcp_port_to);
    }
    if (r->flags & NIC_FILTER_TCP_PORT_FROM) {
        ui_console_write(" tcp_sport=");
        ui_console_u32_dec(r->tcp_port_from);
    }
    if (r->flags & NIC_FILTER_UDP_PORT_TO) {
        ui_console_write(" udp_dport=");
        ui_console_u32_dec(r->udp_port_to);
    }
    if (r->flags & NIC_FILTER_UDP_PORT_FROM) {
        ui_console_write(" udp_sport=");
        ui_console_u32_dec(r->udp_port_from);
    }
    ui_console_write("\n");
}

static void ui_cmd_firewall(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("firewall list|reset|clear|remove <idx>|default <in allow|deny> <out allow|deny>\n");
        ui_console_write("firewall allow|deny <in|out|both> <tcp|udp|icmp|ip|arp> [port N|toport N|fromport N] [src SPEC] [dst SPEC]\n");
        ui_console_write("SPEC: any | a.b.c.d | a.b.c.d/prefix | a.b.c.d/mask | a.b.c.d-a.b.c.d\n");
        ui_console_write("Inbound default is deny; outbound default is allow. New rules insert first.\n");
        return;
    }

    if (ui_streq(argv[1], "list")) {
        u32 n = nic_filter_count();
        ui_console_write("firewall rules=");
        ui_console_u32_dec(n);
        ui_console_write(" (first match wins)\n");
        for (u32 i = 0; i < n; i++) {
            nic_filter_rule_t r;
            if (nic_filter_get(i, &r))
                ui_firewall_print_rule(i, &r);
        }
        return;
    }

    if (ui_streq(argv[1], "reset")) {
        net_firewall_install_defaults();
        ui_console_write("OK: firewall reset to inbound deny/outbound allow defaults\n");
        return;
    }

    if (ui_streq(argv[1], "clear")) {
        nic_filter_clear();
        nic_filter_set_default(false, true);
        ui_console_write("OK: firewall cleared; inbound deny, outbound allow\n");
        return;
    }

    if (ui_streq(argv[1], "remove")) {
        u32 idx = 0;
        if (argc < 3 || !ui_parse_u32(argv[2], &idx) || !nic_filter_remove(idx)) {
            ui_console_write("ERR: usage firewall remove <idx>\n");
            return;
        }
        ui_console_write("OK: firewall rule removed\n");
        return;
    }

    if (ui_streq(argv[1], "default")) {
        if (argc < 6 || !ui_streq(argv[2], "in") || !ui_streq(argv[4], "out")) {
            ui_console_write("ERR: usage firewall default in <allow|deny> out <allow|deny>\n");
            return;
        }
        bool in_allow = ui_streq(argv[3], "allow");
        bool out_allow = ui_streq(argv[5], "allow");
        if ((!in_allow && !ui_streq(argv[3], "deny")) ||
            (!out_allow && !ui_streq(argv[5], "deny"))) {
            ui_console_write("ERR: default policy must be allow|deny\n");
            return;
        }
        nic_filter_set_default(in_allow, out_allow);
        ui_console_write("OK: firewall defaults updated\n");
        return;
    }

    if (!ui_streq(argv[1], "allow") && !ui_streq(argv[1], "deny")) {
        ui_console_write("ERR: usage firewall help\n");
        return;
    }
    if (argc < 4) {
        ui_console_write("ERR: usage firewall allow|deny <in|out|both> <tcp|udp|icmp|ip|arp> ...\n");
        return;
    }

    nic_filter_rule_t r;
    simd_zero(&r, sizeof(r));
    r.action = ui_streq(argv[1], "allow") ? NIC_FILTER_ALLOW : NIC_FILTER_DROP;
    if (ui_streq(argv[2], "in")) r.direction = NIC_FILTER_DIR_IN;
    else if (ui_streq(argv[2], "out")) r.direction = NIC_FILTER_DIR_OUT;
    else if (ui_streq(argv[2], "both")) r.direction = NIC_FILTER_DIR_BOTH;
    else {
        ui_console_write("ERR: direction must be in|out|both\n");
        return;
    }

    if (ui_streq(argv[3], "tcp")) {
        r.flags |= NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_PROTO;
        r.ethertype = ETH_P_IP;
        r.ip_proto = IP_PROTO_TCP;
    } else if (ui_streq(argv[3], "udp")) {
        r.flags |= NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_PROTO;
        r.ethertype = ETH_P_IP;
        r.ip_proto = IP_PROTO_UDP;
    } else if (ui_streq(argv[3], "icmp")) {
        r.flags |= NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_PROTO;
        r.ethertype = ETH_P_IP;
        r.ip_proto = IP_PROTO_ICMP;
    } else if (ui_streq(argv[3], "ip")) {
        r.flags |= NIC_FILTER_ETHERTYPE;
        r.ethertype = ETH_P_IP;
    } else if (ui_streq(argv[3], "arp")) {
        r.flags |= NIC_FILTER_ETHERTYPE;
        r.ethertype = ETH_P_ARP;
    } else {
        ui_console_write("ERR: protocol must be tcp|udp|icmp|ip|arp\n");
        return;
    }

    for (u32 i = 4; i < argc; i++) {
        if (ui_streq(argv[i], "port") || ui_streq(argv[i], "toport")) {
            u32 p = 0;
            if (++i >= argc || !ui_parse_u32(argv[i], &p) || p > 65535U) {
                ui_console_write("ERR: invalid port\n");
                return;
            }
            if (r.ip_proto == IP_PROTO_TCP) {
                r.flags |= NIC_FILTER_TCP_PORT_TO;
                r.tcp_port_to = (u16)p;
            } else if (r.ip_proto == IP_PROTO_UDP) {
                r.flags |= NIC_FILTER_UDP_PORT_TO;
                r.udp_port_to = (u16)p;
            } else {
                ui_console_write("ERR: ports only valid for tcp/udp\n");
                return;
            }
        } else if (ui_streq(argv[i], "fromport")) {
            u32 p = 0;
            if (++i >= argc || !ui_parse_u32(argv[i], &p) || p > 65535U) {
                ui_console_write("ERR: invalid fromport\n");
                return;
            }
            if (r.ip_proto == IP_PROTO_TCP) {
                r.flags |= NIC_FILTER_TCP_PORT_FROM;
                r.tcp_port_from = (u16)p;
            } else if (r.ip_proto == IP_PROTO_UDP) {
                r.flags |= NIC_FILTER_UDP_PORT_FROM;
                r.udp_port_from = (u16)p;
            } else {
                ui_console_write("ERR: ports only valid for tcp/udp\n");
                return;
            }
        } else if (ui_streq(argv[i], "src")) {
            if (++i >= argc || !ui_parse_ip_spec(argv[i], &r.flags, true,
                                                 &r.ip_from, &r.ip_from_mask,
                                                 &r.ip_from_end)) {
                ui_console_write("ERR: invalid src spec\n");
                return;
            }
        } else if (ui_streq(argv[i], "dst")) {
            if (++i >= argc || !ui_parse_ip_spec(argv[i], &r.flags, false,
                                                 &r.ip_to, &r.ip_to_mask,
                                                 &r.ip_to_end)) {
                ui_console_write("ERR: invalid dst spec\n");
                return;
            }
        } else {
            ui_console_write("ERR: unknown firewall option\n");
            return;
        }
    }

    if (!nic_filter_add_front(&r)) {
        ui_console_write("ERR: firewall rule table full/invalid\n");
        return;
    }
    ui_console_write("OK: firewall rule inserted at index 0\n");
}

static void ui_cmd_netcfg(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "set")) {
        if (argc < 4) {
            ui_console_write("ERR: usage netcfg set <ip|mask|gw|dns> <a.b.c.d>\n");
            return;
        }
        u32 v = 0;
        if (!ui_parse_ip4(argv[3], &v)) {
            ui_console_write("ERR: invalid ip value\n");
            return;
        }
        if (ui_streq(argv[2], "ip")) ui_cfg_ip = v;
        else if (ui_streq(argv[2], "mask")) ui_cfg_mask = v;
        else if (ui_streq(argv[2], "gw")) ui_cfg_gw = v;
        else if (ui_streq(argv[2], "dns")) ui_cfg_dns = v;
        else {
            ui_console_write("ERR: key must be ip|mask|gw|dns\n");
            return;
        }
        ui_console_write("OK: value staged (run 'netcfg apply')\n");
        return;
    }

    if (argc >= 2 && ui_streq(argv[1], "apply")) {
        net_init(ui_cfg_ip, ui_cfg_gw, ui_cfg_mask, NULL);
        dns_init(ui_cfg_dns);
        net_services_listen();
        ui_cfg_dhcp = false;
        ui_console_write("OK: static net config applied\n");
        return;
    }

    if (argc >= 3 && ui_streq(argv[1], "dhcp")) {
        if (ui_streq(argv[2], "on")) {
            u32 timeout_ms = 5000;
            if (argc >= 4) {
                if (!ui_parse_u32(argv[3], &timeout_ms)) {
                    ui_console_write("ERR: invalid timeout\n");
                    return;
                }
            }
            ui_console_write("DHCP: requesting lease...\n");
            if (!dhcp_start(timeout_ms)) {
                ui_console_write("ERR: dhcp failed/timeout\n");
                return;
            }
            const dhcp_lease_t *lease = dhcp_get_lease();
            if (!lease) {
                ui_console_write("ERR: no lease\n");
                return;
            }
            ui_cfg_ip = lease->ip;
            ui_cfg_mask = lease->mask;
            ui_cfg_gw = lease->gateway;
            ui_cfg_dns = lease->dns;
            ui_cfg_dhcp = true;
            dns_init(ui_cfg_dns);
            ui_console_write("OK: dhcp lease applied\n");
            return;
        }
        if (ui_streq(argv[2], "off")) {
            ui_cfg_dhcp = false;
            ui_console_write("OK: dhcp disabled (run 'netcfg apply' for static)\n");
            return;
        }
        ui_console_write("ERR: usage netcfg dhcp <on|off> [timeout_ms]\n");
        return;
    }

    if (argc >= 2 && ui_streq(argv[1], "addnbr")) {
        if (argc < 4) {
            ui_console_write("ERR: usage netcfg addnbr <ip> <mac>\n");
            return;
        }
        u32 ip = 0;
        u8 mac[6];
        if (!ui_parse_ip4(argv[2], &ip) || !ui_parse_mac6(argv[3], mac)) {
            ui_console_write("ERR: invalid ip/mac\n");
            return;
        }
        net_add_neighbor(ip, mac);
        ui_console_write("OK: neighbor added\n");
        return;
    }

    if (argc >= 2 && ui_streq(argv[1], "routes")) {
        ui_console_route_table();
        return;
    }

    if (argc >= 2 && (ui_streq(argv[1], "neighbors") || ui_streq(argv[1], "arp"))) {
        ui_console_arp_table();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "trace")) {
        ui_console_net_egress_trace();
        return;
    }

    if (argc >= 2 && ui_streq(argv[1], "route")) {
        if (argc < 7 || !ui_streq(argv[2], "add")) {
            ui_console_write("ERR: usage netcfg route add <dst> <mask> <gw> <connected|via>\n");
            return;
        }
        u32 dst = 0, mask = 0, gw = 0;
        if (!ui_parse_ip4(argv[3], &dst) || !ui_parse_ip4(argv[4], &mask) ||
            !ui_parse_ip4(argv[5], &gw)) {
            ui_console_write("ERR: invalid route ip/mask/gw\n");
            return;
        }
        u8 flags = ui_streq(argv[6], "connected") ? NET_ROUTE_F_CONNECTED : 0;
        if (!ui_streq(argv[6], "connected") && !ui_streq(argv[6], "via")) {
            ui_console_write("ERR: route mode connected|via\n");
            return;
        }
        if (!net_route_add(dst, mask, gw, flags)) {
            ui_console_write("ERR: route add failed\n");
            return;
        }
        ui_console_write("OK: route added\n");
        return;
    }

    const net_stats_t *st = net_get_stats();
    u8 mac[6];
    nic_get_mac(mac);
    fb_printf("link=%s mode=%s\n", nic_link_up() ? "up" : "down", ui_cfg_dhcp ? "dhcp" : "static");
    fb_printf("ip=");
    ui_print_ip(net_get_our_ip());
    fb_puts(" mask=");
    ui_print_ip(ui_cfg_mask);
    fb_puts(" gw=");
    ui_print_ip(ui_cfg_gw);
    fb_puts(" dns=");
    ui_print_ip(ui_cfg_dns);
    fb_puts("\n");
    fb_printf("mac=%x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    fb_printf("tx=%X rx=%X udp_tx=%X udp_rx=%X drops=%X\n",
              st->tx_packets, st->rx_packets, st->udp_sent, st->udp_recv,
              st->drop_runt + st->drop_bad_cksum + st->drop_fragment + st->drop_ip_options +
              st->drop_bad_src + st->drop_not_for_us + st->drop_bad_proto +
              st->drop_icmp_ratelimit + st->drop_no_neighbor + st->drop_udp_malformed + st->drop_oversized);

    uart_puts("net link=");
    uart_puts(nic_link_up() ? "up" : "down");
    uart_puts(" mode=");
    uart_puts(ui_cfg_dhcp ? "dhcp" : "static");
    uart_puts(" ip=");
    uart_hex(net_get_our_ip());
    uart_puts(" mask=");
    uart_hex(ui_cfg_mask);
    uart_puts(" gw=");
    uart_hex(ui_cfg_gw);
    uart_puts(" dns=");
    uart_hex(ui_cfg_dns);
    uart_puts(" tx=");
    uart_hex((u32)st->tx_packets);
    uart_puts(" rx=");
    uart_hex((u32)st->rx_packets);
    uart_puts("\n");
    ui_console_route_table();
    ui_console_arp_table();
}

static void ui_cmd_disk(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "info")) {
        const sd_card_t *card = sd_get_card_info();
        fb_printf("disk type=%u rca=0x%x cap_bytes=%X\n", card->type, card->rca, card->capacity);
        uart_puts("disk type=");
        uart_hex(card->type);
        uart_puts(" rca=");
        uart_hex(card->rca);
        uart_puts(" cap=");
        uart_hex(card->capacity);
        uart_puts("\n");
        return;
    }
    if (ui_streq(argv[1], "sync")) {
        walfs_sync();
        ui_console_write("OK: walfs synced\n");
        return;
    }
    if (ui_streq(argv[1], "compact")) {
        bool ok = walfs_compact();
        ui_console_write(ok ? "OK: walfs compacted\n" : "ERR: compact failed\n");
        return;
    }
    if (ui_streq(argv[1], "verify")) {
        struct walfs_health h;
        bool ok = walfs_verify(&h);
        fb_printf("walfs verify: ok=%u super=%u head=%u rec=%u crc_err=%u hdr_err=%u open_tx=%u scan_end=%X\n",
                  ok ? 1U : 0U, h.super_ok ? 1U : 0U, h.wal_head_ok ? 1U : 0U,
                  h.valid_records, h.crc_errors, h.header_errors, h.open_tx ? 1U : 0U, h.scan_end);
        ui_console_write(ok ? "OK: walfs verify clean\n" : "ERR: walfs verify failed\n");
        return;
    }
    if (ui_streq(argv[1], "read")) {
        if (argc < 3) {
            ui_console_write("ERR: usage disk read <lba>\n");
            return;
        }
        u32 lba = 0;
        if (!ui_parse_u32(argv[2], &lba)) {
            ui_console_write("ERR: invalid lba\n");
            return;
        }
        ui_dump_sector(lba);
        return;
    }
    if (ui_streq(argv[1], "writezero")) {
        if (argc < 4 || !ui_streq(argv[3], "--force")) {
            ui_console_write("ERR: usage disk writezero <lba> --force\n");
            return;
        }
        u32 lba = 0;
        if (!ui_parse_u32(argv[2], &lba)) {
            ui_console_write("ERR: invalid lba\n");
            return;
        }
        static u8 z[SD_BLOCK_SIZE] ALIGNED(64);
        simd_zero(z, SD_BLOCK_SIZE);
        bool ok = sd_write_block(lba, z);
        ui_console_write(ok ? "OK: sector zeroed\n" : "ERR: write failed\n");
        return;
    }
    ui_console_write("ERR: usage disk [info|sync|compact|verify|read <lba>|writezero <lba> --force]\n");
}

static bool ui_parse_db_ref(u32 argc, char **argv, u32 start, u32 *card_out, u32 *rec_out, u32 *next_arg)
{
    if (!argv || !card_out || !rec_out || !next_arg || start >= argc)
        return false;
    u16 c16 = 0;
    u32 rec = 0;
    if (pios_addr_parse_picowal(argv[start], &c16, &rec)) {
        *card_out = c16;
        *rec_out = rec;
        *next_arg = start + 1;
        return true;
    }
    u32 card = 0;
    if (start + 1 >= argc)
        return false;
    if (!ui_parse_u32(argv[start], &card) || card > PICOWAL_CARD_MAX)
        return false;
    if (!ui_parse_u32(argv[start + 1], &rec) || rec > PICOWAL_RECORD_MAX)
        return false;
    *card_out = card;
    *rec_out = rec;
    *next_arg = start + 2;
    return true;
}

static void ui_cmd_addr(u32 argc, char **argv)
{
    if (argc < 2) {
        ui_console_write("addr <kind:pack/card[/tail]>\n");
        ui_console_write("Kinds: wal tcp udp stream dev file. Bare pack/card means wal:pack/card.\n");
        ui_console_write("Examples: wal:0/3 tcp:0/80 udp:0/7001 stream:1/42 dev:0/1/uart0 file:0/12/etc/init.pis\n");
        return;
    }
    struct pios_addr a;
    char canon[160];
    if (!pios_addr_parse(argv[1], &a) || !pios_addr_format(&a, canon, sizeof(canon))) {
        ui_console_write("ERR: invalid address\n");
        return;
    }
    ui_console_write("canonical=");
    ui_console_write(canon);
    ui_console_write("\nkind=");
    ui_console_write(pios_addr_kind_name(a.kind));
    ui_console_write(" pack=");
    ui_console_u32_dec(a.pack);
    ui_console_write(" card=");
    ui_console_u32_dec(a.card);
    if (a.kind == PIOS_ADDR_WAL) {
        u32 key = 0;
        if (picowal_db_pack_key((u16)a.pack, a.card, &key)) {
            ui_console_write(" dbkey=");
            ui_console_hex_fixed(key, 8);
        }
    } else if (a.kind == PIOS_ADDR_TCP || a.kind == PIOS_ADDR_UDP) {
        ui_console_write(" port=");
        ui_console_u32_dec(a.card);
    }
    if (a.tail[0]) {
        ui_console_write(" tail=");
        ui_console_write(a.tail);
    }
    ui_console_write("\n");
}

static void ui_cmd_db(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("db key <card> <record> | db key <wal:pack/card>\n");
        ui_console_write("db put <card> <record> <text...> | db put <wal:pack/card> <text...>\n");
        ui_console_write("db putf|getf <card> <record> <path> | db putf|getf <wal:pack/card> <path>\n");
        ui_console_write("db get|del <card> <record> | db get|del <wal:pack/card>\n");
        ui_console_write("db list <card|wal:pack/card>\n");
        ui_console_write("udp: port 7001 op={1:get,2:put,3:del,4:list} ver=1\n");
        return;
    }

    if (ui_streq(argv[1], "list")) {
        if (argc < 3) {
            ui_console_write("ERR: usage db list <card>\n");
            return;
        }
        u32 card = 0;
        u32 ignored = 0;
        u16 c16 = 0;
        if (pios_addr_parse_picowal(argv[2], &c16, &ignored)) {
            card = c16;
        } else if (!ui_parse_u32(argv[2], &card) || card > PICOWAL_CARD_MAX) {
            ui_console_write("ERR: card out of range (0..1023)\n");
            return;
        }
        u32 ids[64];
        u32 n = picowal_db_list((u16)card, ids, 64);
        fb_printf("db card=%u count=%u\n", card, n);
        for (u32 i = 0; i < n; i++)
            fb_printf("  rec=%u\n", ids[i]);
        return;
    }

    if (argc < 3) {
        ui_console_write("ERR: usage db <op> <card> <record> ... OR db <op> <wal:pack/card> ...\n");
        return;
    }

    u32 card = 0, rec = 0;
    u32 argi = 0;
    if (!ui_parse_db_ref(argc, argv, 2, &card, &rec, &argi)) {
        ui_console_write("ERR: invalid db address (use <card> <record> or wal:pack/card)\n");
        return;
    }

    if (ui_streq(argv[1], "key")) {
        u32 key = 0;
        if (!picowal_db_pack_key((u16)card, rec, &key)) {
            ui_console_write("ERR: key pack failed\n");
            return;
        }
        fb_printf("key=0x%x card=%u record=%u\n", key, card, rec);
        return;
    }

    if (ui_streq(argv[1], "del")) {
        if (!picowal_db_delete((u16)card, rec))
            ui_console_write("ERR: delete failed\n");
        else
            ui_console_write("OK: deleted\n");
        return;
    }

    if (ui_streq(argv[1], "put")) {
        if (argc <= argi) {
            ui_console_write("ERR: usage db put <addr> <text...>\n");
            return;
        }
        static u8 data[PICOWAL_DATA_MAX];
        u32 p = 0;
        for (u32 i = argi; i < argc; i++) {
            const char *s = argv[i];
            while (*s && p < PICOWAL_DATA_MAX) data[p++] = (u8)*s++;
            if (i + 1 < argc && p < PICOWAL_DATA_MAX) data[p++] = ' ';
        }
        if (p == 0 || p >= PICOWAL_DATA_MAX) {
            ui_console_write("ERR: payload too large\n");
            return;
        }
        i32 n = picowal_db_put((u16)card, rec, data, p);
        if (n < 0) ui_console_write("ERR: put failed\n");
        else fb_printf("OK: wrote %u bytes\n", (u32)n);
        return;
    }

    if (ui_streq(argv[1], "putf")) {
        if (argc <= argi) {
            ui_console_write("ERR: usage db putf <addr> <path>\n");
            return;
        }
        char abs[256];
        if (!ui_path_resolve(argv[argi], abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        u64 id = walfs_find(abs);
        if (!id) {
            ui_console_write("ERR: source file not found\n");
            return;
        }
        static u8 data[PICOWAL_DATA_MAX];
        i32 got = (i32)walfs_read(id, 0, data, PICOWAL_DATA_MAX);
        if (got <= 0) {
            ui_console_write("ERR: source read failed\n");
            return;
        }
        i32 n = picowal_db_put((u16)card, rec, data, (u32)got);
        if (n < 0) ui_console_write("ERR: putf failed\n");
        else fb_printf("OK: wrote %u bytes\n", (u32)n);
        return;
    }

    if (ui_streq(argv[1], "get")) {
        static u8 data[PICOWAL_DATA_MAX];
        i32 n = picowal_db_get((u16)card, rec, data, PICOWAL_DATA_MAX);
        if (n < 0) {
            ui_console_write("ERR: get failed\n");
            return;
        }
        fb_printf("db card=%u rec=%u len=%u\n", card, rec, (u32)n);
        for (i32 i = 0; i < n; i++) {
            char c = (char)data[i];
            if (c < 0x20 || c > 0x7E) c = '.';
            uart_putc(c);
            fb_putc(c);
        }
        uart_puts("\n");
        fb_putc('\n');
        return;
    }

    if (ui_streq(argv[1], "getf")) {
        if (argc <= argi) {
            ui_console_write("ERR: usage db getf <addr> <path>\n");
            return;
        }
        static u8 data[PICOWAL_DATA_MAX];
        i32 n = picowal_db_get((u16)card, rec, data, PICOWAL_DATA_MAX);
        if (n < 0) {
            ui_console_write("ERR: getf read failed\n");
            return;
        }
        char abs[256];
        if (!ui_path_resolve(argv[argi], abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        u64 id = walfs_find(abs);
        if (!id) {
            if (!ui_fs_create_path(abs, false)) {
                ui_console_write("ERR: target create failed\n");
                return;
            }
            id = walfs_find(abs);
        }
        if (!id || !walfs_write(id, 0, data, (u32)n)) {
            ui_console_write("ERR: target write failed\n");
            return;
        }
        fb_printf("OK: wrote file bytes=%u\n", (u32)n);
        return;
    }

    ui_console_write("ERR: unknown db op\n");
}

static void ui_dump_sector(u32 lba)
{
    static u8 sector[SD_BLOCK_SIZE] ALIGNED(64);
    if (!sd_read_block(lba, sector)) {
        ui_console_write("ERR: sd_read_block failed\n");
        return;
    }
    static const char hex[] = "0123456789ABCDEF";
    fb_printf("LBA 0x%x\n", lba);
    uart_puts("LBA ");
    uart_hex(lba);
    uart_puts("\n");
    for (u32 off = 0; off < SD_BLOCK_SIZE; off += 16) {
        fb_printf("%x: ", off);
        uart_hex(off);
        uart_puts(": ");
        for (u32 i = 0; i < 16; i++) {
            u8 b = sector[off + i];
            fb_putc(hex[(b >> 4) & 0xF]);
            fb_putc(hex[b & 0xF]);
            fb_putc(' ');
            uart_putc(hex[(b >> 4) & 0xF]);
            uart_putc(hex[b & 0xF]);
            uart_putc(' ');
        }
        fb_puts(" |");
        uart_puts(" |");
        for (u32 i = 0; i < 16; i++) {
            char c = (char)sector[off + i];
            if (c < 0x20 || c > 0x7E) c = '.';
            fb_putc(c);
            uart_putc(c);
        }
        fb_puts("|\n");
        uart_puts("|\n");
    }
}

static void ui_cmd_lsdir(const char *path)
{
    char abs[256];
    if (!path || !*path) path = ".";
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    ui_cmd_fsinspect(abs);
}

static void ui_cmd_mkdir(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage mkdir <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    if (walfs_find(abs)) {
        ui_console_write("OK: already exists\n");
        return;
    }
    if (!ui_fs_create_path(abs, true)) {
        ui_console_write("ERR: mkdir failed\n");
        return;
    }
    ui_console_write("OK: directory created\n");
}

static void ui_cmd_touch(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage touch <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        if (!ui_fs_create_path(abs, false)) {
            ui_console_write("ERR: touch create failed\n");
            return;
        }
        id = walfs_find(abs);
        if (!id) {
            ui_console_write("ERR: touch resolve failed\n");
            return;
        }
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: touch target not file\n");
        return;
    }
    ui_console_write("OK: touched\n");
}

static bool ui_copy_file_internal(const char *src, const char *dst)
{
    char src_abs[256];
    char dst_abs[256];
    if (!ui_path_resolve(src, src_abs, sizeof(src_abs))) return false;
    if (!ui_path_resolve(dst, dst_abs, sizeof(dst_abs))) return false;
    u64 src_id = walfs_find(src_abs);
    if (!src_id) return false;
    struct walfs_inode s;
    if (!walfs_stat(src_id, &s) || (s.flags & WALFS_DIR)) return false;

    u64 dst_id = walfs_find(dst_abs);
    if (!dst_id) {
        if (!ui_fs_create_path(dst_abs, false)) return false;
        dst_id = walfs_find(dst_abs);
        if (!dst_id) return false;
    }

    static u8 buf[WALFS_DATA_MAX] ALIGNED(64);
    u64 off = 0;
    while (off < s.size) {
        u32 chunk = (u32)((s.size - off) > WALFS_DATA_MAX ? WALFS_DATA_MAX : (s.size - off));
        u32 n = walfs_read(src_id, off, buf, chunk);
        if (n == 0 && chunk != 0) break;
        if (!walfs_write(dst_id, off, buf, n)) return false;
        off += n;
        if (n < chunk) break;
    }
    return true;
}

static bool ui_cpdir_recursive(const char *src, const char *dst, u32 depth)
{
    if (depth > 8) return false;
    char src_abs[256];
    char dst_abs[256];
    if (!ui_path_resolve(src, src_abs, sizeof(src_abs))) return false;
    if (!ui_path_resolve(dst, dst_abs, sizeof(dst_abs))) return false;
    u64 src_id = walfs_find(src_abs);
    if (!src_id) return false;
    struct walfs_inode s;
    if (!walfs_stat(src_id, &s) || !(s.flags & WALFS_DIR)) return false;

    if (!walfs_find(dst_abs) && !ui_fs_create_path(dst_abs, true))
        return false;

    struct ui_dir_entry entries[64];
    dir_collect_ctx.out = entries;
    dir_collect_ctx.max = 64;
    dir_collect_ctx.count = 0;
    walfs_readdir(src_id, ui_dir_collect_cb);

    for (u32 i = 0; i < dir_collect_ctx.count; i++) {
        char src_child[256];
        char dst_child[256];
        if (!ui_path_join(src_abs, entries[i].name, src_child, sizeof(src_child))) return false;
        if (!ui_path_join(dst_abs, entries[i].name, dst_child, sizeof(dst_child))) return false;
        struct walfs_inode ino;
        if (!walfs_stat(entries[i].id, &ino)) return false;
        if (ino.flags & WALFS_DIR) {
            if (!ui_cpdir_recursive(src_child, dst_child, depth + 1)) return false;
        } else {
            if (!ui_copy_file_internal(src_child, dst_child)) return false;
        }
    }
    return true;
}

static void ui_cmd_copy(const char *src, const char *dst)
{
    if (!src || !dst || !*src || !*dst) {
        ui_console_write("ERR: usage copy <src> <dst>\n");
        return;
    }
    if (!ui_copy_file_internal(src, dst)) {
        ui_console_write("ERR: copy failed\n");
        return;
    }
    ui_console_write("OK: copied\n");
}

static void ui_cmd_cpdir(const char *src, const char *dst)
{
    if (!src || !dst || !*src || !*dst) {
        ui_console_write("ERR: usage cpdir <src_dir> <dst_dir>\n");
        return;
    }
    if (!ui_cpdir_recursive(src, dst, 0)) {
        ui_console_write("ERR: cpdir failed\n");
        return;
    }
    ui_console_write("OK: directory copied\n");
}

static void ui_cmd_cat(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage cat <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: cat target not file\n");
        return;
    }
    static char buf[129];
    u64 off = 0;
    char last = 0;
    while (off < ino.size) {
        u32 want = (u32)((ino.size - off) > 128 ? 128 : (ino.size - off));
        u32 n = walfs_read(id, off, buf, want);
        if (n == 0) break;
        last = buf[n - 1];
        buf[n] = 0;
        ui_console_write(buf);
        off += n;
    }
    if (ino.size == 0 || last != '\n')
        ui_console_write("\n");
}

static void ui_cmd_stat(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage stat <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino)) {
        ui_console_write("ERR: stat failed\n");
        return;
    }
    fb_printf("path=%s id=0x%x size=%u flags=0x%x mode=0x%x\n",
              abs, (u32)id, (u32)ino.size, ino.flags, ino.mode);
    uart_puts("path=");
    uart_puts(abs);
    uart_puts(" id=");
    uart_hex((u32)id);
    uart_puts(" size=");
    uart_hex((u32)ino.size);
    uart_puts(" flags=");
    uart_hex(ino.flags);
    uart_puts(" mode=");
    uart_hex(ino.mode);
    uart_puts("\n");
}

static void ui_cmd_rm(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage rm <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    if (ui_streq(abs, "/")) {
        ui_console_write("ERR: refusing to remove root\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    if (!walfs_delete(id)) {
        ui_console_write("ERR: remove failed\n");
        return;
    }
    ui_console_write("OK: removed\n");
}

static void ui_stream_udp_cb(u32 src_ip, u16 src_port, u16 dst_port UNUSED, const u8 *data, u16 len)
{
    if (!ui_stream_udp.waiting || !data) return;
    if (len > sizeof(ui_stream_udp.data)) len = sizeof(ui_stream_udp.data);
    simd_memcpy(ui_stream_udp.data, data, len);
    ui_stream_udp.len = len;
    ui_stream_udp.src_ip = src_ip;
    ui_stream_udp.src_port = src_port;
    ui_stream_udp.ready = true;
}

static u16 ui_be16_read(const u8 *p)
{
    return (u16)(((u16)p[0] << 8) | (u16)p[1]);
}

static u16 ui_le16_read(const u8 *p)
{
    return (u16)((u16)p[0] | ((u16)p[1] << 8));
}

static u32 ui_be32_read(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | (u32)p[3];
}

static u32 ui_le32_read(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void ui_be16_write(u8 *p, u16 v)
{
    p[0] = (u8)(v >> 8);
    p[1] = (u8)v;
}

static void ui_be32_write(u8 *p, u32 v)
{
    p[0] = (u8)(v >> 24);
    p[1] = (u8)(v >> 16);
    p[2] = (u8)(v >> 8);
    p[3] = (u8)v;
}

static void ui_db_udp_cb(u32 src_ip, u16 src_port, u16 dst_port, const u8 *data, u16 len)
{
    if (!data || len < 12 || dst_port != PICOWAL_KV_UDP_PORT)
        return;

    u8 op = data[0];
    u8 ver = data[1];
    u16 card = ui_be16_read(&data[4]);
    u32 rec = ui_be32_read(&data[6]);
    u16 val_len = ui_be16_read(&data[10]);

    u8 out[1472];
    u16 out_len = 10;
    out[0] = 1; /* status: error by default */
    out[1] = op;
    ui_be16_write(&out[2], 0);
    ui_be16_write(&out[4], card);
    ui_be32_write(&out[6], rec);

    if (ver != UI_DB_UDP_VER) {
        out[0] = 2; /* bad request/version */
        net_send_udp(src_ip, PICOWAL_KV_UDP_PORT, src_port, out, out_len);
        return;
    }
    if (card > PICOWAL_CARD_MAX || rec > PICOWAL_RECORD_MAX) {
        out[0] = 2;
        net_send_udp(src_ip, PICOWAL_KV_UDP_PORT, src_port, out, out_len);
        return;
    }

    if (op == 1) { /* GET */
        i32 n = picowal_db_get(card, rec, &out[10], (u32)(sizeof(out) - 10));
        if (n >= 0) {
            out[0] = 0;
            ui_be16_write(&out[2], (u16)n);
            out_len = (u16)(10 + (u16)n);
        }
    } else if (op == 2) { /* PUT */
        if ((u32)len < 12U + (u32)val_len || val_len == 0 || val_len > PICOWAL_DATA_MAX) {
            out[0] = 2;
        } else {
            i32 n = picowal_db_put(card, rec, &data[12], val_len);
            if (n >= 0) {
                out[0] = 0;
                ui_be16_write(&out[2], (u16)n);
            }
        }
    } else if (op == 3) { /* DEL */
        if (picowal_db_delete(card, rec))
            out[0] = 0;
    } else if (op == 4) { /* LIST card */
        u32 ids[64];
        u32 n = picowal_db_list(card, ids, 64);
        u32 max_entries = ((u32)sizeof(out) - 10U) / 4U;
        if (n > max_entries) n = max_entries;
        for (u32 i = 0; i < n; i++)
            ui_be32_write(&out[10 + i * 4], ids[i]);
        out[0] = 0;
        ui_be16_write(&out[2], (u16)n);
        out_len = (u16)(10 + n * 4U);
        rec = 0;
        ui_be32_write(&out[6], rec);
    } else {
        out[0] = 2;
    }

    net_send_udp(src_ip, PICOWAL_KV_UDP_PORT, src_port, out, out_len);
}

static bool ui_env_key_eq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static bool ui_env_get(const char *key, const char **val_out)
{
    if (!key || !*key) return false;
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (ui_env[i].used && ui_env_key_eq(ui_env[i].key, key)) {
            if (val_out) *val_out = ui_env[i].val;
            return true;
        }
    }
    return false;
}

static void ui_env_set(const char *key, const char *val, bool persistent)
{
    if (!key || !*key || !val) return;
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (ui_env[i].used && ui_env_key_eq(ui_env[i].key, key)) {
            u32 j = 0;
            for (; key[j] && j + 1 < sizeof(ui_env[i].key); j++) ui_env[i].key[j] = key[j];
            ui_env[i].key[j] = 0;
            j = 0;
            for (; val[j] && j + 1 < sizeof(ui_env[i].val); j++) ui_env[i].val[j] = val[j];
            ui_env[i].val[j] = 0;
            ui_env[i].persistent = persistent;
            return;
        }
    }
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (!ui_env[i].used) {
            u32 j = 0;
            for (; key[j] && j + 1 < sizeof(ui_env[i].key); j++) ui_env[i].key[j] = key[j];
            ui_env[i].key[j] = 0;
            j = 0;
            for (; val[j] && j + 1 < sizeof(ui_env[i].val); j++) ui_env[i].val[j] = val[j];
            ui_env[i].val[j] = 0;
            ui_env[i].used = true;
            ui_env[i].persistent = persistent;
            return;
        }
    }
}

static bool ui_env_save(void)
{
    char out[2048];
    u32 p = 0;
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (!ui_env[i].used || !ui_env[i].persistent) continue;
        const char *k = ui_env[i].key;
        const char *v = ui_env[i].val;
        while (*k && p + 1 < sizeof(out)) out[p++] = *k++;
        if (p + 1 >= sizeof(out)) break;
        out[p++] = '=';
        while (*v && p + 1 < sizeof(out)) out[p++] = *v++;
        if (p + 1 >= sizeof(out)) break;
        out[p++] = '\n';
    }
    return picowal_db_put(0, 1, out, p) >= 0;
}

static bool ui_env_load(void)
{
    if (ui_env_loaded) return true;
    ui_env_loaded = true;
    char buf[2048];
    i32 n_i = picowal_db_get(0, 1, buf, sizeof(buf) - 1);
    if (n_i < 0) return true; /* no persisted config yet */
    u32 n = (u32)n_i;
    buf[n] = 0;
    u32 i = 0;
    while (i < n) {
        char key[32]; char val[128];
        u32 kp = 0, vp = 0;
        while (i < n && buf[i] != '=' && buf[i] != '\n' && kp + 1 < sizeof(key)) key[kp++] = buf[i++];
        key[kp] = 0;
        if (i < n && buf[i] == '=') i++;
        while (i < n && buf[i] != '\n' && vp + 1 < sizeof(val)) val[vp++] = buf[i++];
        val[vp] = 0;
        while (i < n && buf[i] != '\n') i++;
        if (i < n && buf[i] == '\n') i++;
        if (key[0]) ui_env_set(key, val, true);
    }
    return true;
}

static u32 ui_read_tty_line(char *out, u32 out_max, const char *prompt)
{
    if (!out || out_max < 2) return 0;
    if (prompt) ui_console_write(prompt);
    u32 n = 0;
    while (1) {
        i32 c = usb_kbd_try_getc();
        if (c < 0) c = uart_try_getc();
        if (c < 0) {
            net_poll();
            timer_delay_ms(1);
            continue;
        }
        if (c == '\r' || c == '\n') {
            out[n] = 0;
            ui_console_write("\n");
            return n;
        }
        if (c == '\b' || c == 127) {
            if (n > 0) {
                n--;
                ui_console_write("\b \b");
            }
            continue;
        }
        if (c < 0x20 || c > 0x7E) continue;
        if (n + 1 >= out_max) continue;
        out[n++] = (char)c;
        char e[2] = {(char)c, 0};
        ui_console_write(e);
    }
}

static u32 ui_edit_line_len(const char *s)
{
    u32 n = 0;
    while (s[n]) n++;
    return n;
}

static bool ui_edit_insert_line(char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX], u32 *line_count, u32 at)
{
    if (!line_count || *line_count >= UI_EDIT_MAX_LINES || at > *line_count) return false;
    for (u32 i = *line_count; i > at; i--) {
        simd_memcpy(lines[i], lines[i - 1], UI_EDIT_LINE_MAX);
    }
    lines[at][0] = 0;
    (*line_count)++;
    return true;
}

static bool ui_edit_delete_line(char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX], u32 *line_count, u32 at)
{
    if (!line_count || *line_count == 0 || at >= *line_count) return false;
    if (*line_count == 1) {
        lines[0][0] = 0;
        return true;
    }
    for (u32 i = at; i + 1 < *line_count; i++) {
        simd_memcpy(lines[i], lines[i + 1], UI_EDIT_LINE_MAX);
    }
    (*line_count)--;
    lines[*line_count][0] = 0;
    return true;
}

static bool ui_edit_save_file(const char *abs_path, char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX], u32 line_count)
{
    if (!abs_path || !*abs_path) return false;
    u64 id = walfs_find(abs_path);
    struct walfs_inode ino;
    if (id) {
        if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) return false;
        if (!walfs_delete(id)) return false;
        id = 0;
    }
    if (!id) {
        if (!ui_fs_create_path(abs_path, false)) return false;
        id = walfs_find(abs_path);
        if (!id) return false;
    }

    char out[UI_EDIT_MAX_LINES * UI_EDIT_LINE_MAX];
    u32 p = 0;
    if (line_count == 0) line_count = 1;
    for (u32 i = 0; i < line_count; i++) {
        const char *ln = lines[i];
        u32 len = ui_edit_line_len(ln);
        if (p + len > sizeof(out)) return false;
        for (u32 j = 0; j < len; j++) out[p++] = ln[j];
        if (i + 1 < line_count) {
            if (p + 1 > sizeof(out)) return false;
            out[p++] = '\n';
        }
    }
    if (p == 0) return true;
    return walfs_write(id, 0, out, p);
}

static void ui_edit_render(const char *abs_path,
                           char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX],
                           u32 line_count, u32 cur_line, u32 cur_col, u32 view_top,
                           bool insert_mode, bool dirty, const char *status)
{
    fb_clear(UI_SHELL_BG_COLOR);
    fb_set_color(UI_SHELL_TEXT_COLOR, UI_SHELL_BG_COLOR);
    fb_printf("PIOS edit.pix (kernel TUI)  %s\n", abs_path ? abs_path : "(null)");
    fb_printf("Ctrl+S save | Ctrl+Q exit | Ctrl+C copy line | Ctrl+X cut line | Ctrl+V paste line\n");
    fb_printf("Arrows move | Enter split line | Backspace/Delete erase | Insert toggles %s | %s%s\n",
              insert_mode ? "insert" : "overwrite", dirty ? "*" : "",
              status ? status : "");
    fb_printf("--------------------------------------------------------------------------------\n");

    fb_set_color(0x00FFFFFF, UI_SHELL_BG_COLOR);
    u32 rows = 20;
    for (u32 r = 0; r < rows; r++) {
        u32 li = view_top + r;
        if (li >= line_count) {
            fb_printf("~\n");
            continue;
        }
        bool sel = (li == cur_line);
        fb_set_color(sel ? 0x00FF9900 : 0x00FFFFFF, UI_SHELL_BG_COLOR);
        fb_printf("%c%u ", sel ? '>' : ' ', li + 1);
        char out[UI_EDIT_LINE_MAX + 2];
        u32 op = 0;
        u32 len = ui_edit_line_len(lines[li]);
        for (u32 i = 0; i < len && op + 1 < sizeof(out); i++) {
            if (sel && i == cur_col) out[op++] = '|';
            out[op++] = lines[li][i];
        }
        if (sel && cur_col >= len && op + 1 < sizeof(out)) out[op++] = '|';
        out[op] = 0;
        fb_printf("%s\n", out);
    }
}

static void ui_cmd_edit(const char *path)
{
    if (!path || !*path) {
        ui_console_write("ERR: usage edit <path>\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }

    char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX];
    simd_zero(lines, sizeof(lines));
    u32 line_count = 1;
    u32 cur_line = 0, cur_col = 0, view_top = 0;
    bool insert_mode = true;
    bool dirty = false;
    bool running = true;
    bool has_clip = false;
    bool redraw = true;
    char clip[UI_EDIT_LINE_MAX];
    char status[64];
    status[0] = 0;

    u64 id = walfs_find(abs);
    if (id) {
        struct walfs_inode ino;
        if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
            ui_console_write("ERR: edit target not file\n");
            return;
        }
        u32 max_read = (u32)((ino.size > (sizeof(lines) - 1)) ? (sizeof(lines) - 1) : ino.size);
        char buf[UI_EDIT_MAX_LINES * UI_EDIT_LINE_MAX];
        u32 n = walfs_read(id, 0, buf, max_read);
        buf[n] = 0;
        line_count = 0;
        u32 li = 0, cj = 0;
        for (u32 i = 0; i < n && li < UI_EDIT_MAX_LINES; i++) {
            char c = buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                lines[li][cj] = 0;
                li++;
                cj = 0;
                continue;
            }
            if (c < 0x20 || c > 0x7E) c = '.';
            if (cj + 1 < UI_EDIT_LINE_MAX) lines[li][cj++] = c;
        }
        if (li < UI_EDIT_MAX_LINES) {
            lines[li][cj] = 0;
            line_count = li + 1;
        }
        if (line_count == 0) line_count = 1;
    }

    while (running) {
        if (cur_line < view_top) view_top = cur_line;
        if (cur_line >= view_top + 20) view_top = cur_line - 19;

        if (redraw) {
            ui_edit_render(abs, lines, line_count, cur_line, cur_col, view_top, insert_mode, dirty, status);
            redraw = false;
        }

        i32 key;
        while ((key = usb_kbd_try_getkey()) >= 0) {
            u32 len = ui_edit_line_len(lines[cur_line]);
            if (key == USB_KBD_KEY_LEFT) {
                if (cur_col > 0) cur_col--;
            } else if (key == USB_KBD_KEY_RIGHT) {
                if (cur_col < len) cur_col++;
            } else if (key == USB_KBD_KEY_UP) {
                if (cur_line > 0) cur_line--;
                len = ui_edit_line_len(lines[cur_line]);
                if (cur_col > len) cur_col = len;
            } else if (key == USB_KBD_KEY_DOWN) {
                if (cur_line + 1 < line_count) cur_line++;
                len = ui_edit_line_len(lines[cur_line]);
                if (cur_col > len) cur_col = len;
            } else if (key == USB_KBD_KEY_HOME) {
                cur_col = 0;
            } else if (key == USB_KBD_KEY_END) {
                cur_col = ui_edit_line_len(lines[cur_line]);
            } else if (key == USB_KBD_KEY_INSERT) {
                insert_mode = !insert_mode;
            } else if (key == USB_KBD_KEY_DELETE) {
                if (cur_col < len) {
                    for (u32 i = cur_col; i < len; i++) lines[cur_line][i] = lines[cur_line][i + 1];
                    dirty = true;
                } else if (cur_line + 1 < line_count) {
                    u32 nlen = ui_edit_line_len(lines[cur_line + 1]);
                    if (len + nlen + 1 < UI_EDIT_LINE_MAX) {
                        for (u32 i = 0; i < nlen; i++) lines[cur_line][len + i] = lines[cur_line + 1][i];
                        lines[cur_line][len + nlen] = 0;
                        ui_edit_delete_line(lines, &line_count, cur_line + 1);
                        dirty = true;
                    }
                }
            } else if (key == USB_KBD_KEY_F3) {
                running = false;
            }
            status[0] = 0;
            redraw = true;
        }

        i32 c;
        while ((c = usb_kbd_try_getc()) >= 0) {
            u32 len = ui_edit_line_len(lines[cur_line]);
            if (c == 19) { /* Ctrl+S */
                if (ui_edit_save_file(abs, lines, line_count)) {
                    const char *ok = "saved";
                    for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                    dirty = false;
                } else {
                    const char *er = "save failed";
                    for (u32 i = 0; er[i] && i + 1 < sizeof(status); i++) status[i] = er[i], status[i + 1] = 0;
                }
                redraw = true;
                continue;
            }
            if (c == 17 || c == 27) { /* Ctrl+Q / Esc */
                running = false;
                break;
            }
            if (c == 3) { /* Ctrl+C copy line */
                simd_memcpy(clip, lines[cur_line], UI_EDIT_LINE_MAX);
                has_clip = true;
                const char *ok = "copied line";
                for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                redraw = true;
                continue;
            }
            if (c == 24) { /* Ctrl+X cut line */
                simd_memcpy(clip, lines[cur_line], UI_EDIT_LINE_MAX);
                has_clip = true;
                ui_edit_delete_line(lines, &line_count, cur_line);
                if (cur_line >= line_count) cur_line = line_count - 1;
                cur_col = 0;
                dirty = true;
                const char *ok = "cut line";
                for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                redraw = true;
                continue;
            }
            if (c == 22) { /* Ctrl+V paste line below */
                if (has_clip && ui_edit_insert_line(lines, &line_count, cur_line + 1)) {
                    simd_memcpy(lines[cur_line + 1], clip, UI_EDIT_LINE_MAX);
                    cur_line++;
                    cur_col = 0;
                    dirty = true;
                    const char *ok = "pasted line";
                    for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                } else {
                    const char *er = "paste failed";
                    for (u32 i = 0; er[i] && i + 1 < sizeof(status); i++) status[i] = er[i], status[i + 1] = 0;
                }
                redraw = true;
                continue;
            }
            if (c == '\r' || c == '\n') {
                if (line_count < UI_EDIT_MAX_LINES && ui_edit_insert_line(lines, &line_count, cur_line + 1)) {
                    if (cur_col < len) {
                        u32 tail = len - cur_col;
                        if (tail + 1 < UI_EDIT_LINE_MAX) {
                            for (u32 i = 0; i < tail; i++) lines[cur_line + 1][i] = lines[cur_line][cur_col + i];
                            lines[cur_line + 1][tail] = 0;
                            lines[cur_line][cur_col] = 0;
                        }
                    }
                    cur_line++;
                    cur_col = 0;
                    dirty = true;
                }
                redraw = true;
                continue;
            }
            if (c == '\b' || c == 127) {
                if (cur_col > 0) {
                    for (u32 i = cur_col - 1; i < len; i++) lines[cur_line][i] = lines[cur_line][i + 1];
                    cur_col--;
                    dirty = true;
                } else if (cur_line > 0) {
                    u32 plen = ui_edit_line_len(lines[cur_line - 1]);
                    if (plen + len + 1 < UI_EDIT_LINE_MAX) {
                        for (u32 i = 0; i < len; i++) lines[cur_line - 1][plen + i] = lines[cur_line][i];
                        lines[cur_line - 1][plen + len] = 0;
                        ui_edit_delete_line(lines, &line_count, cur_line);
                        cur_line--;
                        cur_col = plen;
                        dirty = true;
                    }
                }
                redraw = true;
                continue;
            }
            if (c < 0x20 || c > 0x7E) continue;
            if (insert_mode) {
                if (len + 1 >= UI_EDIT_LINE_MAX) continue;
                for (u32 i = len + 1; i > cur_col; i--) lines[cur_line][i] = lines[cur_line][i - 1];
                lines[cur_line][cur_col] = (char)c;
                cur_col++;
                dirty = true;
            } else {
                if (cur_col < len) {
                    lines[cur_line][cur_col++] = (char)c;
                    dirty = true;
                } else if (len + 1 < UI_EDIT_LINE_MAX) {
                    lines[cur_line][len] = (char)c;
                    lines[cur_line][len + 1] = 0;
                    cur_col++;
                    dirty = true;
                }
            }
            redraw = true;
        }

        while ((c = uart_try_getc()) >= 0) {
            if (c == 19) {
                if (ui_edit_save_file(abs, lines, line_count)) dirty = false;
                redraw = true;
            } else if (c == 17 || c == 27) {
                running = false;
            } else if (c == '\r' || c == '\n') {
                if (line_count < UI_EDIT_MAX_LINES && ui_edit_insert_line(lines, &line_count, cur_line + 1)) {
                    u32 len = ui_edit_line_len(lines[cur_line]);
                    if (cur_col < len) {
                        u32 tail = len - cur_col;
                        if (tail + 1 < UI_EDIT_LINE_MAX) {
                            for (u32 i = 0; i < tail; i++) lines[cur_line + 1][i] = lines[cur_line][cur_col + i];
                            lines[cur_line + 1][tail] = 0;
                            lines[cur_line][cur_col] = 0;
                        }
                    }
                    cur_line++;
                    cur_col = 0;
                    dirty = true;
                }
                redraw = true;
            } else if (c == '\b' || c == 127) {
                u32 len = ui_edit_line_len(lines[cur_line]);
                if (cur_col > 0) {
                    for (u32 i = cur_col - 1; i < len; i++) lines[cur_line][i] = lines[cur_line][i + 1];
                    cur_col--;
                    dirty = true;
                    redraw = true;
                }
            } else if (c >= 0x20 && c <= 0x7E) {
                u32 len = ui_edit_line_len(lines[cur_line]);
                if (len + 1 < UI_EDIT_LINE_MAX) {
                    for (u32 i = len + 1; i > cur_col; i--) lines[cur_line][i] = lines[cur_line][i - 1];
                    lines[cur_line][cur_col++] = (char)c;
                    dirty = true;
                    redraw = true;
                }
            }
        }
        net_poll();
        workq_drain(2);
        timer_delay_ms(1);
    }

    fb_clear(UI_SHELL_BG_COLOR);
    fb_set_color(UI_SHELL_TEXT_COLOR, UI_SHELL_BG_COLOR);
    ui_console_write("PIOS F3 Console (serial + HDMI)\n");
    ui_console_write("Type 'help' for commands.\n");
    ui_console_prompt();
}

static void ui_stream_emit(const u8 *data, u32 len, bool to_file, const char *path)
{
    if (!data) return;
    if (!to_file) {
        for (u32 i = 0; i < len; i++) {
            char c = (char)data[i];
            if (c < 0x20 && c != '\n' && c != '\r' && c != '\t') c = '.';
            char o[2] = {c, 0};
            ui_console_write(o);
        }
        if (len == 0 || data[len - 1] != '\n') ui_console_write("\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad output path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        if (!ui_fs_create_path(abs, false)) {
            ui_console_write("ERR: output create failed\n");
            return;
        }
        id = walfs_find(abs);
    }
    if (!id || !walfs_write(id, 0, data, len))
        ui_console_write("ERR: output write failed\n");
    else
        ui_console_write("OK: output saved\n");
}

static void http_append_sanitized_bytes(char *out, u32 *len, u32 max, const u8 *data, u32 n)
{
    if (!data) return;
    for (u32 i = 0; i < n; i++) {
        char c = (char)data[i];
        if (c < 0x20 && c != '\n' && c != '\r' && c != '\t') c = '.';
        char s[2] = { c, 0 };
        http_append(out, len, max, s);
    }
}

static bool ui_tcp_wait_established(tcp_conn_t c, u32 timeout_ms)
{
    for (u32 t = 0; t < timeout_ms; t++) {
        net_poll();
        u32 st = tcp_state(c);
        if (st == TCP_ESTABLISHED) return true;
        if (st == TCP_CLOSED) return false;
        timer_delay_ms(1);
    }
    return false;
}

static bool ui_tcp_write_all(tcp_conn_t c, const u8 *data, u32 len, u32 timeout_ms)
{
    u32 off = 0;
    u32 idle = 0;
    while (off < len && idle < timeout_ms) {
        u32 n = tcp_write(c, data + off, len - off);
        if (n > 0) {
            off += n;
            idle = 0;
        } else {
            idle++;
            timer_delay_ms(1);
        }
        net_poll();
    }
    return off == len;
}

static bool ui_http_fetch(bool use_tls, u32 dst_ip, u16 port,
                          const char *host, const char *path,
                          u32 timeout_ms, u8 *out, u32 out_max,
                          u32 *out_len, const char **err)
{
    char req[384];
    u32 req_len = 0;
    tcp_conn_t c;
    if (out_len) *out_len = 0;
    if (err) *err = NULL;
    if (!out || out_max == 0 || !host || !path || !path[0]) {
        if (err) *err = "bad args";
        return false;
    }

    http_append(req, &req_len, sizeof(req), "GET ");
    http_append(req, &req_len, sizeof(req), path);
    http_append(req, &req_len, sizeof(req), " HTTP/1.0\r\nHost: ");
    http_append(req, &req_len, sizeof(req), host);
    http_append(req, &req_len, sizeof(req), "\r\nConnection: close\r\n\r\n");

    c = tcp_connect(dst_ip, port);
    if (c < 0) {
        if (err) *err = "tcp connect failed";
        return false;
    }
    if (!ui_tcp_wait_established(c, timeout_ms)) {
        tcp_close(c);
        if (err) *err = "tcp connect timeout";
        return false;
    }

    if (use_tls) {
        tls_conn_t tc = tls_connect(c);
        if (tc < 0) {
            tcp_close(c);
            if (err) *err = "tls handshake failed";
            return false;
        }
        if (tls_write(tc, req, req_len) < 0) {
            tls_close(tc);
            if (err) *err = "tls write failed";
            return false;
        }
        i32 n = tls_read(tc, out, out_max);
        tls_close(tc);
        if (n < 0) {
            if (err) *err = "tls read failed";
            return false;
        }
        if (out_len) *out_len = (u32)n;
        return true;
    }

    if (!ui_tcp_write_all(c, (const u8 *)req, req_len, timeout_ms)) {
        tcp_close(c);
        if (err) *err = "tcp write timeout";
        return false;
    }

    u32 nout = 0;
    u32 idle = 0;
    while (idle < timeout_ms && nout < out_max) {
        net_poll();
        u32 avail = tcp_readable(c);
        if (avail > 0) {
            u32 want = out_max - nout;
            if (want > avail) want = avail;
            u32 n = tcp_read(c, out + nout, want);
            nout += n;
            idle = 0;
        } else {
            u32 st = tcp_state(c);
            if (st == TCP_CLOSED || st >= TCP_CLOSING) break;
            idle++;
            timer_delay_ms(1);
        }
    }
    tcp_close(c);
    if (out_len) *out_len = nout;
    return true;
}

static bool ui_http_client_parse_common(u32 argc, char **argv, bool use_tls,
                                        u32 *ip, u16 *port, const char **path,
                                        u32 *timeout_ms)
{
    u32 p = use_tls ? 443U : 80U;
    u32 to = 3000U;
    if (argc < 3 || !ui_streq(argv[1], "get"))
        return false;
    if (!ui_parse_ip4(argv[2], ip) && !dns_cache_lookup(argv[2], ip))
        return false;
    *path = (argc >= 4) ? argv[3] : "/";
    if ((*path)[0] != '/')
        return false;
    if (argc >= 5 && (!ui_parse_u32(argv[4], &p) || p > 65535U))
        return false;
    if (argc >= 6 && (!ui_parse_u32(argv[5], &to) || to == 0 || to > 30000U))
        return false;
    *port = (u16)p;
    *timeout_ms = to;
    return true;
}

static void ui_cmd_http_client(u32 argc, char **argv, bool use_tls)
{
    u32 ip = 0;
    u16 port = 0;
    u32 timeout_ms = 0;
    const char *path = NULL;
    u8 out[UI_STREAM_OUT_MAX];
    u32 out_len = 0;
    const char *err = NULL;

    if (!ui_http_client_parse_common(argc, argv, use_tls, &ip, &port, &path, &timeout_ms)) {
        ui_console_write(use_tls ?
            "ERR: usage https get <ip-or-cached-host> [path] [port] [timeout_ms]\n" :
            "ERR: usage http get <ip-or-cached-host> [path] [port] [timeout_ms]\n");
        return;
    }
    if (!ui_http_fetch(use_tls, ip, port, argv[2], path, timeout_ms,
                       out, sizeof(out), &out_len, &err)) {
        ui_console_write("ERR: ");
        ui_console_write(err ? err : "request failed");
        ui_console_write("\n");
        return;
    }
    ui_stream_emit(out, out_len, false, NULL);
}

static void ui_cmd_stream(u32 argc, char **argv)
{
    if (argc < 8 || !ui_streq(argv[4], "from")) {
        ui_console_write("ERR: stream <tcp|udp> <ip> <port> from <file|text|tty> <arg?> to <console|file> [path] [timeout_ms]\n");
        return;
    }
    char proto = argv[1][0];
    u32 dst_ip = 0, port = 0;
    if (!ui_parse_ip4(argv[2], &dst_ip) || !ui_parse_u32(argv[3], &port) || port > 65535) {
        ui_console_write("ERR: invalid ip/port\n");
        return;
    }
    u8 in_buf[UI_STREAM_IN_MAX];
    u32 in_len = 0;
    u32 i = 5;
    if (ui_streq(argv[i], "file")) {
        if (i + 1 >= argc) { ui_console_write("ERR: stream missing file path\n"); return; }
        char abs[256];
        if (!ui_path_resolve(argv[i + 1], abs, sizeof(abs))) { ui_console_write("ERR: bad file path\n"); return; }
        u64 id = walfs_find(abs);
        struct walfs_inode ino;
        if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) { ui_console_write("ERR: stream input file not found\n"); return; }
        in_len = (u32)((ino.size > UI_STREAM_IN_MAX) ? UI_STREAM_IN_MAX : ino.size);
        in_len = walfs_read(id, 0, in_buf, in_len);
        i += 2;
    } else if (ui_streq(argv[i], "text")) {
        if (i + 1 >= argc) { ui_console_write("ERR: stream missing text\n"); return; }
        const char *s = argv[i + 1];
        while (*s && in_len + 1 < sizeof(in_buf)) in_buf[in_len++] = (u8)*s++;
        i += 2;
    } else if (ui_streq(argv[i], "tty")) {
        char line[UI_STREAM_IN_MAX];
        in_len = ui_read_tty_line(line, sizeof(line), "tty> ");
        for (u32 j = 0; j < in_len; j++) in_buf[j] = (u8)line[j];
        i += 1;
    } else {
        ui_console_write("ERR: from must be file|text|tty\n");
        return;
    }
    if (i >= argc || !ui_streq(argv[i], "to") || i + 1 >= argc) {
        ui_console_write("ERR: stream missing to clause\n");
        return;
    }
    bool out_to_file = false;
    const char *out_path = NULL;
    if (ui_streq(argv[i + 1], "console")) {
        i += 2;
    } else if (ui_streq(argv[i + 1], "file")) {
        if (i + 2 >= argc) { ui_console_write("ERR: stream missing output file path\n"); return; }
        out_to_file = true;
        out_path = argv[i + 2];
        i += 3;
    } else {
        ui_console_write("ERR: to must be console|file\n");
        return;
    }
    u32 timeout_ms = 3000;
    if (i < argc && !ui_parse_u32(argv[i], &timeout_ms)) {
        ui_console_write("ERR: invalid timeout\n");
        return;
    }

    if (proto == 'u') {
        udp_recv_cb prev = net_swap_udp_callback(ui_stream_udp_cb);
        simd_zero(&ui_stream_udp, sizeof(ui_stream_udp));
        ui_stream_udp.waiting = true;
        bool sent = net_send_udp(dst_ip, 40000, (u16)port, in_buf, (u16)in_len);
        if (!sent) {
            net_set_udp_callback(prev);
            ui_console_write("ERR: udp send failed\n");
            return;
        }
        for (u32 t = 0; t < timeout_ms; t++) {
            net_poll();
            if (ui_stream_udp.ready) break;
            timer_delay_ms(1);
        }
        ui_stream_udp.waiting = false;
        net_set_udp_callback(prev);
        if (!ui_stream_udp.ready) {
            ui_console_write("ERR: udp recv timeout\n");
            return;
        }
        ui_stream_emit(ui_stream_udp.data, ui_stream_udp.len, out_to_file, out_path);
        return;
    }

    if (proto != 't') {
        ui_console_write("ERR: proto must be tcp|udp\n");
        return;
    }
    tcp_conn_t c = tcp_connect(dst_ip, (u16)port);
    if (c < 0) {
        ui_console_write("ERR: tcp connect failed\n");
        return;
    }
    bool up = false;
    for (u32 t = 0; t < timeout_ms; t++) {
        net_poll();
        u32 st = tcp_state(c);
        if (st == TCP_ESTABLISHED) { up = true; break; }
        if (st == TCP_CLOSED) break;
        timer_delay_ms(1);
    }
    if (!up) {
        tcp_close(c);
        ui_console_write("ERR: tcp connect timeout\n");
        return;
    }
    u32 off = 0;
    while (off < in_len) {
        u32 n = tcp_write(c, in_buf + off, in_len - off);
        if (n == 0) { net_poll(); timer_delay_ms(1); continue; }
        off += n;
        net_poll();
    }
    u8 out[UI_STREAM_OUT_MAX];
    u32 out_len = 0;
    u32 idle = 0;
    while (idle < 200 && out_len < sizeof(out)) {
        net_poll();
        u32 avail = tcp_readable(c);
        if (avail > 0) {
            u32 want = (u32)((sizeof(out) - out_len) < avail ? (sizeof(out) - out_len) : avail);
            u32 n = tcp_read(c, out + out_len, want);
            out_len += n;
            idle = 0;
        } else {
            idle++;
            timer_delay_ms(1);
        }
    }
    tcp_close(c);
    ui_stream_emit(out, out_len, out_to_file, out_path);
}

static void ui_cmd_mv(const char *src, const char *dst)
{
    if (!src || !dst || !*src || !*dst) {
        ui_console_write("ERR: usage mv <src> <dst>\n");
        return;
    }
    if (!ui_copy_file_internal(src, dst)) {
        ui_console_write("ERR: mv copy failed\n");
        return;
    }
    ui_cmd_rm(src);
}

static void ui_cmd_hexdump(const char *path, u32 max_bytes)
{
    if (!path || !*path) {
        ui_console_write("ERR: usage hexdump <path> [max_bytes]\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: hexdump file not found\n");
        return;
    }
    if (max_bytes == 0 || max_bytes > 1024) max_bytes = 256;
    u32 n = (u32)((ino.size < max_bytes) ? ino.size : max_bytes);
    u8 buf[1024];
    n = walfs_read(id, 0, buf, n);
    static const char hx[] = "0123456789ABCDEF";
    for (u32 off = 0; off < n; off += 16) {
        fb_printf("%x: ", off);
        uart_hex(off); uart_puts(": ");
        for (u32 i = 0; i < 16; i++) {
            if (off + i < n) {
                u8 b = buf[off + i];
                char o[4] = {hx[(b>>4)&0xF], hx[b&0xF], ' ', 0};
                ui_console_write(o);
            } else ui_console_write("   ");
        }
        ui_console_write("\n");
    }
}

static void ui_cmd_find(const char *base, const char *needle)
{
    char abs[256];
    if (!base || !*base || !needle || !*needle) {
        ui_console_write("ERR: usage find <dir> <needle>\n");
        return;
    }
    if (!ui_path_resolve(base, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: find dir not found\n");
        return;
    }
    struct ui_dir_entry entries[64];
    dir_collect_ctx.out = entries;
    dir_collect_ctx.max = 64;
    dir_collect_ctx.count = 0;
    walfs_readdir(id, ui_dir_collect_cb);
    for (u32 i = 0; i < dir_collect_ctx.count; i++) {
        bool hit = false;
        const char *n = entries[i].name;
        for (u32 p = 0; n[p]; p++) {
            u32 j = 0;
            while (needle[j] && n[p + j] == needle[j]) j++;
            if (!needle[j]) { hit = true; break; }
        }
        if (!hit) continue;
        char full[256];
        if (!ui_path_join(abs, entries[i].name, full, sizeof(full))) continue;
        ui_console_write(full);
        ui_console_write("\n");
    }
}

static void ui_cmd_df(void)
{
    const sd_card_t *card = sd_get_card_info();
    fb_printf("df: cap_bytes=%X (walfs usage telemetry pending)\n", card->capacity);
    uart_puts("df cap=");
    uart_hex(card->capacity);
    uart_puts("\n");
}

static void ui_cmd_mount(u32 argc, char **argv)
{
    if (argc >= 1 && ui_streq(argv[0], "mount")) {
        ui_console_write("OK: walfs always mounted at /\n");
        return;
    }
    if (argc >= 1 && ui_streq(argv[0], "umount")) {
        ui_console_write("ERR: umount unsupported on running kernel rootfs\n");
        return;
    }
    ui_console_write("ERR: usage mount|umount\n");
}

static void ui_cmd_env(u32 argc, char **argv)
{
    ui_env_load();
    if (argc < 2 || ui_streq(argv[1], "list")) {
        for (u32 i = 0; i < UI_ENV_MAX; i++) {
            if (!ui_env[i].used) continue;
            ui_console_write(ui_env[i].persistent ? "P " : "T ");
            ui_console_write(ui_env[i].key);
            ui_console_write("=");
            ui_console_write(ui_env[i].val);
            ui_console_write("\n");
        }
        return;
    }
    if ((ui_streq(argv[1], "set") || ui_streq(argv[1], "pset")) && argc >= 4) {
        bool p = ui_streq(argv[1], "pset");
        ui_env_set(argv[2], argv[3], p);
        if (p && !ui_env_save()) ui_console_write("ERR: env save failed\n");
        else ui_console_write("OK: env set\n");
        return;
    }
    if (ui_streq(argv[1], "get") && argc >= 3) {
        const char *v = NULL;
        if (ui_env_get(argv[2], &v)) {
            ui_console_write(v);
            ui_console_write("\n");
        } else ui_console_write("ERR: var not found\n");
        return;
    }
    if (ui_streq(argv[1], "unset") && argc >= 3) {
        for (u32 i = 0; i < UI_ENV_MAX; i++) {
            if (ui_env[i].used && ui_env_key_eq(ui_env[i].key, argv[2])) ui_env[i].used = false;
        }
        ui_env_save();
        ui_console_write("OK: env unset\n");
        return;
    }
    if (ui_streq(argv[1], "save")) {
        ui_console_write(ui_env_save() ? "OK: env saved\n" : "ERR: env save failed\n");
        return;
    }
    if (ui_streq(argv[1], "load")) {
        ui_env_loaded = false;
        ui_console_write(ui_env_load() ? "OK: env loaded\n" : "ERR: env load failed\n");
        return;
    }
    ui_console_write("ERR: env [list|get|set|pset|unset|save|load]\n");
}

static void ui_exec_subcommand(u32 start, u32 argc, char **argv, const char *placeholder)
{
    char cmd[256];
    u32 p = 0;
    for (u32 i = start; i < argc; i++) {
        const char *s = argv[i];
        if (placeholder && ui_streq(s, "{}")) s = placeholder;
        while (*s && p + 1 < sizeof(cmd)) cmd[p++] = *s++;
        if (i + 1 < argc && p + 1 < sizeof(cmd)) cmd[p++] = ' ';
    }
    cmd[p] = 0;
    if (p == 0) return;
    if (ui_script_depth > 8) {
        ui_console_write("ERR: script depth exceeded\n");
        return;
    }
    ui_script_depth++;
    ui_console_exec(cmd);
    ui_script_depth--;
}

static bool ui_cmp_values(const char *a, const char *op, const char *b)
{
    u32 av = 0, bv = 0;
    bool na = ui_parse_u32(a, &av);
    bool nb = ui_parse_u32(b, &bv);
    if (ui_streq(op, "==")) return ui_streq(a, b);
    if (ui_streq(op, "!=")) return !ui_streq(a, b);
    if (!na || !nb) return false;
    if (ui_streq(op, ">")) return av > bv;
    if (ui_streq(op, "<")) return av < bv;
    if (ui_streq(op, ">=")) return av >= bv;
    if (ui_streq(op, "<=")) return av <= bv;
    return false;
}

static void ui_cmd_if(u32 argc, char **argv)
{
    if (argc < 6) {
        ui_console_write("ERR: usage if <a> <op> <b> <cmd...>\n");
        return;
    }
    if (ui_cmp_values(argv[1], argv[2], argv[3]))
        ui_exec_subcommand(4, argc, argv, NULL);
}

static void ui_cmd_for(u32 argc, char **argv)
{
    if (argc < 5) {
        ui_console_write("ERR: usage for <start> <end> <cmd... with {}>\n");
        return;
    }
    u32 s = 0, e = 0;
    if (!ui_parse_u32(argv[1], &s) || !ui_parse_u32(argv[2], &e)) {
        ui_console_write("ERR: invalid range\n");
        return;
    }
    char repl[16];
    for (u32 i = s; i <= e; i++) {
        u32 n = 0, v = i;
        char rev[16];
        do { rev[n++] = (char)('0' + (v % 10)); v /= 10; } while (v && n < sizeof(rev));
        for (u32 j = 0; j < n; j++) repl[j] = rev[n - 1 - j];
        repl[n] = 0;
        ui_exec_subcommand(3, argc, argv, repl);
    }
}

static void ui_cmd_foreach(u32 argc, char **argv)
{
    if (argc < 5 || !ui_streq(argv[1], "file")) {
        ui_console_write("ERR: usage foreach file <path> <cmd... with {}>\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(argv[2], abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: foreach file not found\n");
        return;
    }
    char buf[1024];
    u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
    n = walfs_read(id, 0, buf, n);
    buf[n] = 0;
    char line[128];
    u32 lp = 0;
    for (u32 i = 0; i <= n; i++) {
        char c = (i < n) ? buf[i] : '\n';
        if (c == '\n' || c == '\r') {
            line[lp] = 0;
            if (lp) ui_exec_subcommand(3, argc, argv, line);
            lp = 0;
        } else if (lp + 1 < sizeof(line)) {
            line[lp++] = c;
        }
    }
}

static void ui_cmd_source(const char *path)
{
    char abs[256];
    if (!path || !*path || !ui_resolve_pis_path(path, abs, sizeof(abs))) {
        ui_console_write("ERR: usage source <path>\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: source file not found\n");
        return;
    }
    char buf[2048];
    u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
    n = walfs_read(id, 0, buf, n);
    buf[n] = 0;

    if (ui_has_suffix(abs, ".pbc")) {
        if (n < sizeof(struct picoscript_header)) {
            ui_console_write("ERR: bytecode too small\n");
            return;
        }
        const u8 *p = (const u8 *)buf;
        u32 magic = ui_le32_read(p);
        u16 version = ui_le16_read(p + 4);
        u16 count = ui_le16_read(p + 6);
        u32 image_bytes = ui_le32_read(p + 8);
        if (magic != PICOSCRIPT_MAGIC || version != PICOSCRIPT_VERSION ||
            image_bytes > (u32)n || image_bytes < sizeof(struct picoscript_header)) {
            ui_console_write("ERR: invalid PicoScript bytecode\n");
            return;
        }
        u32 off = sizeof(struct picoscript_header);
        char line[PICOSCRIPT_MAX_LINE + 1];
        for (u32 ci = 0; ci < count; ci++) {
            if (off + sizeof(struct picoscript_record) > image_bytes) {
                ui_console_write("ERR: truncated bytecode record\n");
                return;
            }
            u16 len = ui_le16_read(p + off);
            u8 flags = p[off + 2];
            off += sizeof(struct picoscript_record);
            if (len > PICOSCRIPT_MAX_LINE || off + len > image_bytes) {
                ui_console_write("ERR: invalid bytecode command\n");
                return;
            }
            for (u32 j = 0; j < len; j++)
                line[j] = (char)p[off + j];
            line[len] = 0;
            off += len;
            if ((flags & 1U) == 0 && len != 0 && line[0] != '#')
                ui_console_exec(line);
        }
        return;
    }

    char line[256];
    u32 lp = 0;
    for (u32 i = 0; i <= n; i++) {
        char c = (i < n) ? buf[i] : '\n';
        if (c == '\n' || c == '\r') {
            line[lp] = 0;
            if (lp && line[0] != '#') ui_console_exec(line);
            lp = 0;
        } else if (lp + 1 < sizeof(line)) {
            line[lp++] = c;
        }
    }
}

static bool ui_cap_manifest_validate_buf(const char *buf, u32 n, char *err, u32 err_max)
{
    if (!buf || n == 0) {
        if (err && err_max) { err[0] = 'e'; err[1] = 'm'; err[2] = 'p'; err[3] = 't'; err[4] = 'y'; err[5] = 0; }
        return false;
    }
    u32 i = 0;
    while (i < n) {
        u32 ls = i;
        while (i < n && buf[i] != '\n' && buf[i] != '\r') i++;
        u32 le = i;
        while (i < n && (buf[i] == '\n' || buf[i] == '\r')) i++;
        while (ls < le && (buf[ls] == ' ' || buf[ls] == '\t')) ls++;
        while (le > ls && (buf[le - 1] == ' ' || buf[le - 1] == '\t')) le--;
        if (le <= ls || buf[ls] == '#')
            continue;
        u32 eq = ls;
        while (eq < le && buf[eq] != '=') eq++;
        if (eq >= le) {
            if (err && err_max) { err[0]='b'; err[1]='a'; err[2]='d'; err[3]='='; err[4]=0; }
            return false;
        }
        u32 klen = eq - ls;
        if (klen == 0 || klen > 16) {
            if (err && err_max) { err[0]='k'; err[1]='e'; err[2]='y'; err[3]=0; }
            return false;
        }
        bool key_ok = false;
        if (klen == 7 && ui_strneq(&buf[ls], "capsule", 7)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "spawn", 5)) key_ok = true;
        else if (klen == 4 && ui_strneq(&buf[ls], "wait", 4)) key_ok = true;
        else if (klen == 6 && ui_strneq(&buf[ls], "nprocs", 6)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "group", 5)) key_ok = true;
        else if (klen == 3 && ui_strneq(&buf[ls], "vfs", 3)) key_ok = true;
        else if (klen == 2 && ui_strneq(&buf[ls], "fs", 2)) key_ok = true;
        else if (klen == 3 && ui_strneq(&buf[ls], "ipc", 3)) key_ok = true;
        else if (klen == 4 && ui_strneq(&buf[ls], "pipe", 4)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "cards", 5)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "ports", 5)) key_ok = true;
        else if (klen == 7 && ui_strneq(&buf[ls], "mem_kib", 7)) key_ok = true;
        else if (klen == 6 && ui_strneq(&buf[ls], "cpu_ms", 6)) key_ok = true;
        else if (klen == 8 && ui_strneq(&buf[ls], "ipc_objs", 8)) key_ok = true;
        else if (klen == 12 && ui_strneq(&buf[ls], "fs_write_kib", 12)) key_ok = true;
        if (!key_ok) {
            if (err && err_max) { err[0]='u'; err[1]='n'; err[2]='k'; err[3]=0; }
            return false;
        }
        u32 vlen = le - (eq + 1);
        if (vlen == 0) {
            if (err && err_max) { err[0]='v'; err[1]='a'; err[2]='l'; err[3]=0; }
            return false;
        }
    }
    if (err && err_max) err[0] = 0;
    return true;
}

static void ui_cmd_capsule(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("capsule ls | capsule status [id] | capsule check <path.cap>\n");
        return;
    }
    if (ui_streq(argv[1], "ls")) {
        struct proc_capsule_ui_entry e[UI_SNAPSHOT_MAX];
        u32 n = proc_capsule_snapshot(e, UI_SNAPSHOT_MAX);
        for (u32 i = 0; i < n; i++) {
            fb_printf("pid=0x%x core=%u cap=%u hash=0x%x grp=%s vfs=%s\n",
                      e[i].pid, e[i].affinity_core, e[i].capsule_id, e[i].capsule_hash,
                      e[i].group[0] ? e[i].group : "-", e[i].vfs_root[0] ? e[i].vfs_root : "-");
        }
        uart_puts("capsule ls done\n");
        return;
    }
    if (ui_streq(argv[1], "status")) {
        u32 id = 0;
        if (argc >= 3 && !ui_parse_u32(argv[2], &id)) {
            ui_console_write("ERR: invalid capsule id\n");
            return;
        }
        u64 st = 0, faults = 0;
        if (el2_hvc_call(EL2_HVC_STAGE2_STATUS, id, 0, 0, 0, &st) != 0) {
            ui_console_write("ERR: stage2 status unavailable\n");
            return;
        }
        (void)el2_hvc_call(EL2_HVC_STAGE2_FAULTS, 0, 0, 0, 0, &faults);
        fb_printf("capsule=%u st=0x%x faults=0x%x\n", id, st, faults);
        return;
    }
    if (ui_streq(argv[1], "check")) {
        if (argc < 3) {
            ui_console_write("ERR: usage capsule check <path.cap>\n");
            return;
        }
        char abs[256];
        if (!ui_path_resolve(argv[2], abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        u64 id = walfs_find(abs);
        struct walfs_inode ino;
        if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
            ui_console_write("ERR: manifest not found\n");
            return;
        }
        char buf[1024];
        u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
        n = walfs_read(id, 0, buf, n);
        buf[n] = 0;
        char err[16];
        if (!ui_cap_manifest_validate_buf(buf, n, err, sizeof(err))) {
            ui_console_write("ERR: invalid manifest ");
            ui_console_write(err);
            ui_console_write("\n");
            return;
        }
        ui_console_write("OK: manifest valid\n");
        return;
    }
    ui_console_write("ERR: unknown capsule subcommand\n");
}

static void ui_cmd_obs(u32 argc, char **argv)
{
    (void)argc; (void)argv;
    const net_stats_t *st = net_get_stats();
    struct proc_security_stats ps;
    proc_security_stats_snapshot(&ps);
    u64 faults = 0;
    (void)el2_hvc_call(EL2_HVC_STAGE2_FAULTS, 0, 0, 0, 0, &faults);
    u32 s2_fault_count = (u32)(faults & 0xFFFFFFFFULL);
    u32 s2_last_esr16 = (u32)((faults >> 32) & 0xFFFFULL);
    u32 s2_active = (u32)((faults >> 48) & 0xFFULL);
    u32 s2_last_fault_cap = (u32)((faults >> 56) & 0xFFULL);
    u64 net_drops = st->drop_runt + st->drop_bad_cksum + st->drop_fragment + st->drop_ip_options +
                    st->drop_bad_src + st->drop_not_for_us + st->drop_bad_proto +
                    st->drop_icmp_ratelimit + st->drop_no_neighbor + st->drop_udp_malformed +
                    st->drop_oversized;
    fb_printf("obs security: integ_chk=%X integ_fail=%X cap_kill=%X policy_deny=%X claim_deny=%X\n",
              ps.integrity_checks, ps.integrity_failures, ps.capsule_kills,
              ps.port_policy_denies, ps.port_claim_denies);
    fb_printf("obs net: tx=%X rx=%X udp_tx=%X udp_rx=%X drops=%X\n",
              st->tx_packets, st->rx_packets, st->udp_sent, st->udp_recv, net_drops);
    fb_printf("obs el2: s2_faults=%X last_esr16=0x%x active_cap=%u last_fault_cap=%u\n",
              s2_fault_count, s2_last_esr16, s2_active, s2_last_fault_cap);
    uart_puts("obs integ_chk="); uart_hex((u32)ps.integrity_checks);
    uart_puts(" integ_fail="); uart_hex((u32)ps.integrity_failures);
    uart_puts(" cap_kill="); uart_hex((u32)ps.capsule_kills);
    uart_puts(" policy_deny="); uart_hex((u32)ps.port_policy_denies);
    uart_puts(" claim_deny="); uart_hex((u32)ps.port_claim_denies);
    uart_puts(" net_drops="); uart_hex((u32)net_drops);
    uart_puts(" s2_faults="); uart_hex(s2_fault_count);
    uart_puts("\n");
}

static void ui_cmd_update(u32 argc, char **argv)
{
    if (!principal_has_cap(principal_current(), PRINCIPAL_ADMIN)) {
        ui_console_write("ERR: admin required\n");
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("update status | update stage <slot:0|1> [tries] | update success\n");
        return;
    }
    if (ui_streq(argv[1], "status")) {
        fb_printf("update slot=%u pending=%u prev=%u tries=%u committed=%u gen=%u\n",
                  g_boot_update_state.active_slot, g_boot_update_state.pending_slot,
                  g_boot_update_state.previous_slot, g_boot_update_state.tries_left,
                  g_boot_update_state.committed, g_boot_update_state.generation);
        return;
    }
    if (ui_streq(argv[1], "stage")) {
        if (argc < 3) {
            ui_console_write("ERR: usage update stage <slot:0|1> [tries]\n");
            return;
        }
        u32 slot = 0;
        if (!ui_parse_u32(argv[2], &slot) || slot > 1) {
            ui_console_write("ERR: slot must be 0 or 1\n");
            return;
        }
        u32 tries = 2;
        if (argc >= 4 && (!ui_parse_u32(argv[3], &tries) || tries == 0 || tries > 16)) {
            ui_console_write("ERR: tries must be 1..16\n");
            return;
        }
        struct boot_update_record rec = g_boot_update_state;
        rec.previous_slot = rec.active_slot;
        rec.active_slot = slot;
        rec.pending_slot = slot;
        rec.tries_left = tries;
        rec.committed = 0;
        rec.generation++;
        if (!boot_update_store(&rec)) {
            ui_console_write("ERR: update stage write failed\n");
            return;
        }
        g_boot_update_state = rec;
        ui_console_write("OK: update staged\n");
        return;
    }
    if (ui_streq(argv[1], "success")) {
        struct boot_update_record rec = g_boot_update_state;
        rec.previous_slot = rec.active_slot;
        rec.pending_slot = rec.active_slot;
        rec.tries_left = 0;
        rec.committed = 1;
        rec.generation++;
        if (!boot_update_store(&rec)) {
            ui_console_write("ERR: update success write failed\n");
            return;
        }
        g_boot_update_state = rec;
        ui_console_write("OK: update marked successful\n");
        return;
    }
    ui_console_write("ERR: unknown update subcommand\n");
}

static void ui_cmd_watchdog(u32 argc, char **argv)
{
    if (!principal_has_cap(principal_current(), PRINCIPAL_ADMIN)) {
        ui_console_write("ERR: admin required\n");
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("watchdog status | watchdog arm|disarm | watchdog timeout <ticks> | watchdog mode <halt|reboot> | watchdog trip | watchdog hw-arm <secs> | watchdog hw-pet | watchdog hw-disable | watchdog hw-trip confirm\n");
        return;
    }
    if (ui_streq(argv[1], "status")) {
        struct watchdog_status st;
        watchdog_status(&st);
        fb_printf("watchdog armed=%u mode=%s timeout=%u trips=%u last_core=%u\n",
                  st.armed ? 1U : 0U, st.reboot_on_trip ? "reboot" : "halt",
                  st.timeout_ticks, st.trip_count, st.last_trip_core);
        u32 rem = watchdog_hw_remaining_ticks();
        u32 rstc = watchdog_hw_rstc();
        fb_printf("hw remaining_ticks=0x%x (~%ums) rstc=0x%x\n",
                  rem, (u32)(((u64)rem * 1000ULL) >> 16), rstc);
        ui_console_write("watchdog armed=");
        ui_console_u32_dec(st.armed ? 1U : 0U);
        ui_console_write(" mode=");
        ui_console_write(st.reboot_on_trip ? "reboot" : "halt");
        ui_console_write(" timeout=");
        ui_console_u32_dec(st.timeout_ticks);
        ui_console_write(" trips=");
        ui_console_u64_dec(st.trip_count);
        ui_console_write(" hw_remaining=");
        ui_console_hex_fixed(rem, 8);
        ui_console_write(" hw_rstc=");
        ui_console_hex_fixed(rstc, 8);
        ui_console_write("\n");
        return;
    }
    if (ui_streq(argv[1], "hw-arm")) {
        u32 s = 15;
        if (argc >= 3 && !ui_parse_u32(argv[2], &s)) {
            ui_console_write("ERR: usage watchdog hw-arm <secs 1..15>\n");
            return;
        }
        watchdog_hw_arm_seconds(s);
        u32 rem = watchdog_hw_remaining_ticks();
        fb_printf("OK: hw watchdog armed s=%u remaining=0x%x\n", s, rem);
        ui_console_write("OK: hw watchdog armed s=");
        ui_console_u32_dec(s);
        ui_console_write(" remaining=");
        ui_console_hex_fixed(rem, 8);
        ui_console_write("\n");
        return;
    }
    if (ui_streq(argv[1], "hw-pet")) {
        watchdog_hw_pet();
        u32 rem = watchdog_hw_remaining_ticks();
        fb_printf("OK: hw watchdog petted remaining=0x%x\n", rem);
        ui_console_write("OK: hw watchdog petted remaining=");
        ui_console_hex_fixed(rem, 8);
        ui_console_write("\n");
        return;
    }
    if (ui_streq(argv[1], "hw-disable")) {
        watchdog_hw_disable();
        ui_console_write("OK: hw watchdog disabled\n");
        return;
    }
    if (ui_streq(argv[1], "hw-trip") && argc >= 3 && ui_streq(argv[2], "confirm")) {
        ui_console_write("OK: arming hw watchdog 2s and looping; expect auto-reset.\n");
        watchdog_hw_arm_seconds(2);
        for (;;) { __asm__ volatile("yield"); }
    }
    if (ui_streq(argv[1], "arm")) {
        watchdog_set_armed(true);
        ui_console_write("OK: watchdog armed\n");
        return;
    }
    if (ui_streq(argv[1], "disarm")) {
        watchdog_set_armed(false);
        ui_console_write("OK: watchdog disarmed\n");
        return;
    }
    if (ui_streq(argv[1], "timeout")) {
        u32 t = 0;
        if (argc < 3 || !ui_parse_u32(argv[2], &t)) {
            ui_console_write("ERR: usage watchdog timeout <ticks>\n");
            return;
        }
        watchdog_set_timeout(t);
        ui_console_write("OK: watchdog timeout updated\n");
        return;
    }
    if (ui_streq(argv[1], "mode")) {
        if (argc < 3) {
            ui_console_write("ERR: usage watchdog mode <halt|reboot>\n");
            return;
        }
        if (ui_streq(argv[2], "halt")) watchdog_set_reboot(false);
        else if (ui_streq(argv[2], "reboot")) watchdog_set_reboot(true);
        else { ui_console_write("ERR: mode must be halt|reboot\n"); return; }
        ui_console_write("OK: watchdog mode updated\n");
        return;
    }
    if (ui_streq(argv[1], "trip")) {
        watchdog_trip(CORE_NET, 0x41U);
    }
    ui_console_write("ERR: unknown watchdog subcommand\n");
}

static bool ui_batch_pid_active(i32 pid)
{
    if (pid <= 0) return false;
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    for (u32 i = 0; i < n; i++) {
        if ((i32)snap[i].pid == pid) return true;
    }
    return false;
}

static u32 ui_batch_running_count(void)
{
    u32 n = 0;
    for (u32 i = 0; i < UI_BATCH_MAX; i++)
        if (ui_batch_jobs[i].used && ui_batch_jobs[i].state == 1) n++;
    return n;
}

static u32 ui_batch_core_load(u32 core)
{
    u32 n = 0;
    for (u32 i = 0; i < UI_BATCH_MAX; i++) {
        if (!ui_batch_jobs[i].used || ui_batch_jobs[i].state != 1) continue;
        if ((((u32)ui_batch_jobs[i].pid) >> 16) == core) n++;
    }
    return n;
}

static struct ui_service_unit *ui_service_find(const char *name)
{
    if (!name || !name[0]) return NULL;
    for (u32 i = 0; i < UI_SVC_MAX; i++) {
        if (!ui_services[i].used) continue;
        if (ui_streq(ui_services[i].name, name)) return &ui_services[i];
    }
    return NULL;
}

static bool ui_service_pid_active(i32 pid)
{
    if (pid <= 0) return false;
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    for (u32 i = 0; i < n; i++) {
        if ((i32)snap[i].pid == pid) return true;
    }
    return false;
}

static bool ui_service_target_enabled(const struct ui_service_unit *s)
{
    if (!s || !s->used) return false;
    if (ui_service_target == UI_SVC_TARGET_ALL) return true;
    return s->target == ui_service_target;
}

static bool ui_service_dep_ready(const struct ui_service_unit *s)
{
    if (!s) return false;
    if (s->depends[0] == 0) return true;
    struct ui_service_unit *d = ui_service_find(s->depends);
    if (!d || !d->used) return false;
    return d->state == 1;
}

static void ui_service_tick(void)
{
    if (!ui_service_running) return;
    u64 now = timer_ticks();
    for (u32 i = 0; i < UI_SVC_MAX; i++) {
        struct ui_service_unit *s = &ui_services[i];
        if (!s->used) continue;
        if (s->state == 1 && s->pid > 0) {
            if (ui_service_pid_active(s->pid))
                continue;
            bool should_restart = (!s->stop_requested) &&
                                  (s->restart_policy == UI_SVC_RP_ALWAYS || s->restart_policy == UI_SVC_RP_ONFAIL);
            if (!should_restart) {
                s->state = s->stop_requested ? 0 : 3;
                s->pid = -1;
                s->stop_requested = false;
                continue;
            }
            if (s->window_start_ms == 0 || now - s->window_start_ms > 60000ULL) {
                s->window_start_ms = now;
                s->restarts = 0;
            }
            s->restarts++;
            s->pid = -1;
            if (s->restarts > s->max_restarts) {
                s->state = 3;
            } else {
                s->state = 2;
                s->next_action_ms = now + (u64)s->backoff_ms;
            }
        }
    }

    for (u32 i = 0; i < UI_SVC_MAX; i++) {
        struct ui_service_unit *s = &ui_services[i];
        if (!s->used) continue;
        if (!ui_service_target_enabled(s)) continue;
        if (!ui_service_dep_ready(s)) continue;
        if (s->state == 2 && now < s->next_action_ms) continue;
        if (s->state == 1 || s->state == 3) continue;
        i32 pid = proc_launch_on_core_as_prio(s->preferred_core, s->path, s->principal_id, s->priority_class);
        if (pid > 0) {
            s->pid = pid;
            s->state = 1;
            s->stop_requested = false;
            if (s->window_start_ms == 0) s->window_start_ms = now;
        } else {
            s->state = 2;
            s->next_action_ms = now + (u64)s->backoff_ms;
        }
    }
}

static void ui_batch_tick(void)
{
    ui_service_tick();
    u64 now = timer_ticks();
    for (u32 i = 0; i < UI_BATCH_MAX; i++) {
        struct ui_batch_job *j = &ui_batch_jobs[i];
        if (!j->used || j->state != 1) continue;
        if (!ui_batch_pid_active(j->pid)) {
            if (j->interval_ms > 0) {
                j->state = 0;
                j->next_due_ms = now + j->interval_ms;
            } else {
                j->state = 2;
            }
            j->pid = -1;
        }
    }

    if (!ui_batch_running) return;

    while (ui_batch_running_count() < ui_batch_parallel) {
        struct ui_batch_job *pick = NULL;
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            if (ui_batch_jobs[i].used && ui_batch_jobs[i].state == 0 &&
                ui_batch_jobs[i].next_due_ms <= now) {
                pick = &ui_batch_jobs[i];
                break;
            }
        }
        if (!pick) break;

        if (pick->is_script) {
            ui_cmd_source(pick->path);
            pick->attempts++;
            pick->last_err = 0;
            if (pick->interval_ms > 0) {
                pick->next_due_ms = now + pick->interval_ms;
                pick->state = 0;
            } else {
                pick->state = 2;
            }
            continue;
        }

        u32 core = pick->preferred_core;
        if (core != CORE_USERM && core != CORE_USER0 && core != CORE_USER1) {
            u32 l1 = ui_batch_core_load(CORE_USERM);
            u32 l2 = ui_batch_core_load(CORE_USER0);
            u32 l3 = ui_batch_core_load(CORE_USER1);
            core = CORE_USERM;
            if (l2 < l1) core = CORE_USER0;
            if (l3 < ui_batch_core_load(core)) core = CORE_USER1;
        }
        i32 pid = proc_launch_on_core_as_prio(core, pick->path, pick->principal_id, pick->priority_class);
        if (pid > 0) {
            pick->state = 1;
            pick->pid = pid;
            pick->attempts++;
            pick->last_err = 0;
        } else {
            pick->attempts++;
            pick->last_err = -1;
            if (pick->attempts > (pick->retries + 1))
                pick->state = 3;
        }
    }
}

static void ui_cmd_batch(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("batch add <path> [1|2|3|auto] [lazy|low|normal|high|realtime] [principal] [retries]\n");
        ui_console_write("batch at <delay_ms> <path> [1|2|3|auto] [lazy|low|normal|high|realtime] [principal] [retries]\n");
        ui_console_write("batch every <interval_ms> <path> [1|2|3|auto] [lazy|low|normal|high|realtime] [principal]\n");
        ui_console_write("batch run [parallel] | batch stop | batch status | batch list | batch clear\n");
        return;
    }
    if (ui_streq(argv[1], "add") || ui_streq(argv[1], "at") || ui_streq(argv[1], "every")) {
        u32 path_idx = 2;
        u32 delay_ms = 0;
        u32 interval_ms = 0;
        if (ui_streq(argv[1], "at")) {
            if (argc < 4 || !ui_parse_u32(argv[2], &delay_ms)) {
                ui_console_write("ERR: usage batch at <delay_ms> <path> [1|2|3|auto] [priority] [principal] [retries]\n");
                return;
            }
            path_idx = 3;
        } else if (ui_streq(argv[1], "every")) {
            if (argc < 4 || !ui_parse_u32(argv[2], &interval_ms) || interval_ms == 0) {
                ui_console_write("ERR: usage batch every <interval_ms> <path> [1|2|3|auto] [priority] [principal]\n");
                return;
            }
            path_idx = 3;
        }
        if (argc <= path_idx) {
            ui_console_write("ERR: batch path missing\n");
            return;
        }
        u32 slot = UI_BATCH_MAX;
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            if (!ui_batch_jobs[i].used || ui_batch_jobs[i].state >= 2) { slot = i; break; }
        }
        if (slot == UI_BATCH_MAX) {
            ui_console_write("ERR: batch queue full\n");
            return;
        }
        char path[128];
        bool is_script = false;
        if (!ui_resolve_job_path(argv[path_idx], path, sizeof(path), &is_script)) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        if (!walfs_find(path)) {
            ui_console_write("ERR: job target not found\n");
            return;
        }
        u32 core = 0;
        u32 argi = path_idx + 1;
        if (argc > argi) {
            if (argv[argi][0] == '1') core = CORE_USERM;
            else if (argv[argi][0] == '2') core = CORE_USER0;
            else if (argv[argi][0] == '3') core = CORE_USER1;
            else if (!ui_streq(argv[argi], "auto")) {
                ui_console_write("ERR: core must be 1|2|3|auto\n");
                return;
            }
            argi++;
        }
        u32 priority_class = PROC_PRIO_NORMAL;
        if (argc > argi) {
            u32 ptmp = 0;
            if (ui_parse_priority(argv[argi], &ptmp)) {
                priority_class = ptmp;
                argi++;
            }
        }
        u32 principal_id = principal_current();
        if (argc > argi) {
            if (!ui_parse_u32(argv[argi], &principal_id) || principal_id >= PRINCIPAL_MAX) {
                ui_console_write("ERR: invalid principal\n");
                return;
            }
            if (!principal_has_cap(principal_id, PRINCIPAL_EXEC)) {
                ui_console_write("ERR: principal missing EXEC capability\n");
                return;
            }
            argi++;
        }
        u32 retries = 0;
        if (argc > argi && !ui_parse_u32(argv[argi], &retries)) {
            ui_console_write("ERR: invalid retries\n");
            return;
        }
        struct ui_batch_job *j = &ui_batch_jobs[slot];
        simd_zero(j, sizeof(*j));
        j->used = true;
        j->id = ui_batch_next_id++;
        for (u32 i = 0; path[i] && i + 1 < sizeof(j->path); i++) j->path[i] = path[i];
        j->preferred_core = core;
        j->priority_class = priority_class;
        j->principal_id = principal_id;
        j->state = 0;
        j->pid = -1;
        j->retries = retries;
        j->interval_ms = interval_ms;
        j->next_due_ms = timer_ticks() + delay_ms;
        j->is_script = is_script;
        fb_printf("OK: batch id=%u queued\n", j->id);
        uart_puts("OK: batch id=");
        uart_hex(j->id);
        uart_puts(" queued\n");
        return;
    }
    if (ui_streq(argv[1], "run")) {
        if (argc >= 3) {
            u32 p = 0;
            if (!ui_parse_u32(argv[2], &p) || p == 0 || p > 8) {
                ui_console_write("ERR: parallel must be 1..8\n");
                return;
            }
            ui_batch_parallel = p;
        }
        ui_batch_running = true;
        ui_console_write("OK: batch scheduler running\n");
        return;
    }
    if (ui_streq(argv[1], "stop")) {
        ui_batch_running = false;
        ui_console_write("OK: batch scheduler paused\n");
        return;
    }
    if (ui_streq(argv[1], "clear")) {
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            if (ui_batch_jobs[i].used && ui_batch_jobs[i].state == 1 && ui_batch_jobs[i].pid > 0)
                proc_kill_pid((u32)ui_batch_jobs[i].pid, 0xFFFF3000U);
            ui_batch_jobs[i].used = false;
        }
        ui_console_write("OK: batch queue cleared\n");
        return;
    }
    if (ui_streq(argv[1], "status") || ui_streq(argv[1], "list")) {
        u32 q = 0, r = 0, d = 0, f = 0;
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            struct ui_batch_job *j = &ui_batch_jobs[i];
            if (!j->used) continue;
            if (j->state == 0) q++;
            else if (j->state == 1) r++;
            else if (j->state == 2) d++;
            else if (j->state == 3) f++;
            if (ui_streq(argv[1], "list")) {
                const char *st = (j->state == 0) ? "queued" :
                                 (j->state == 1) ? "running" :
                                 (j->state == 2) ? "done" :
                                 (j->state == 3) ? "failed" : "canceled";
                fb_printf("#%u %s type=%s core=%u pri=%s princ=%u pid=0x%x tries=%u/%u interval=%u %s\n",
                          j->id, st, j->is_script ? "pis" : "pix",
                          j->preferred_core ? j->preferred_core : 0,
                          ui_priority_str(j->priority_class),
                          j->principal_id, (u32)j->pid, j->attempts, j->retries + 1,
                          j->interval_ms, j->path);
                uart_puts("#"); uart_hex(j->id); uart_puts(" ");
                uart_puts(st); uart_puts(" path="); uart_puts(j->path); uart_puts("\n");
            }
        }
        fb_printf("batch: mode=%s parallel=%u queued=%u running=%u done=%u failed=%u\n",
                  ui_batch_running ? "run" : "pause", ui_batch_parallel, q, r, d, f);
        uart_puts("batch mode=");
        uart_puts(ui_batch_running ? "run" : "pause");
        uart_puts(" parallel="); uart_hex(ui_batch_parallel);
        uart_puts(" q="); uart_hex(q);
        uart_puts(" r="); uart_hex(r);
        uart_puts(" d="); uart_hex(d);
        uart_puts(" f="); uart_hex(f);
        uart_puts("\n");
        return;
    }
    ui_console_write("ERR: unknown batch subcommand\n");
}

static void ui_cmd_svc(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("svc add <name> <path> [dep|-] [target:default|rescue|all] [1|2|3] [priority] [principal] [restart] [max_restarts] [backoff_ms]\n");
        ui_console_write("svc start|stop|restart <name> | svc run|pause | svc target <default|rescue|all> | svc list | svc clear\n");
        return;
    }
    if (ui_streq(argv[1], "add")) {
        if (argc < 4) { ui_console_write("ERR: usage svc add <name> <path> ...\n"); return; }
        if (ui_service_find(argv[2])) { ui_console_write("ERR: service exists\n"); return; }
        i32 slot = -1;
        for (u32 i = 0; i < UI_SVC_MAX; i++) if (!ui_services[i].used) { slot = (i32)i; break; }
        if (slot < 0) { ui_console_write("ERR: service table full\n"); return; }

        char path[128];
        bool is_script = false;
        if (!ui_resolve_job_path(argv[3], path, sizeof(path), &is_script) || !walfs_find(path)) {
            ui_console_write("ERR: service path not found\n");
            return;
        }
        if (is_script) {
            ui_console_write("ERR: svc requires executable .pix path\n");
            return;
        }

        struct ui_service_unit *s = &ui_services[(u32)slot];
        simd_zero(s, sizeof(*s));
        s->used = true;
        for (u32 i = 0; argv[2][i] && i + 1 < sizeof(s->name); i++) s->name[i] = argv[2][i];
        for (u32 i = 0; path[i] && i + 1 < sizeof(s->path); i++) s->path[i] = path[i];
        s->target = UI_SVC_TARGET_DEFAULT;
        s->preferred_core = CORE_USERM;
        s->principal_id = principal_current();
        s->priority_class = PROC_PRIO_NORMAL;
        s->restart_policy = UI_SVC_RP_ONFAIL;
        s->max_restarts = 5;
        s->backoff_ms = 1000;
        s->state = 0;
        s->pid = -1;

        if (argc >= 5 && !ui_streq(argv[4], "-")) {
            for (u32 i = 0; argv[4][i] && i + 1 < sizeof(s->depends); i++) s->depends[i] = argv[4][i];
        }
        if (argc >= 6) {
            if (ui_streq(argv[5], "default")) s->target = UI_SVC_TARGET_DEFAULT;
            else if (ui_streq(argv[5], "rescue")) s->target = UI_SVC_TARGET_RESCUE;
            else if (ui_streq(argv[5], "all")) s->target = UI_SVC_TARGET_ALL;
        }
        if (argc >= 7) {
            if (argv[6][0] == '1') s->preferred_core = CORE_USERM;
            else if (argv[6][0] == '2') s->preferred_core = CORE_USER0;
            else if (argv[6][0] == '3') s->preferred_core = CORE_USER1;
        }
        if (argc >= 8) {
            u32 p = 0;
            if (ui_parse_priority(argv[7], &p)) s->priority_class = p;
        }
        if (argc >= 9) {
            u32 pr = 0;
            if (ui_parse_u32(argv[8], &pr) && pr < PRINCIPAL_MAX && principal_has_cap(pr, PRINCIPAL_EXEC))
                s->principal_id = pr;
        }
        if (argc >= 10) {
            if (ui_streq(argv[9], "never")) s->restart_policy = UI_SVC_RP_NEVER;
            else if (ui_streq(argv[9], "onfail")) s->restart_policy = UI_SVC_RP_ONFAIL;
            else if (ui_streq(argv[9], "always")) s->restart_policy = UI_SVC_RP_ALWAYS;
        }
        if (argc >= 11) { u32 v = 0; if (ui_parse_u32(argv[10], &v)) s->max_restarts = v; }
        if (argc >= 12) { u32 v = 0; if (ui_parse_u32(argv[11], &v)) s->backoff_ms = v; }
        ui_console_write("OK: service added\n");
        return;
    }
    if (ui_streq(argv[1], "run")) { ui_service_running = true; ui_console_write("OK: supervisor running\n"); return; }
    if (ui_streq(argv[1], "pause")) { ui_service_running = false; ui_console_write("OK: supervisor paused\n"); return; }
    if (ui_streq(argv[1], "target")) {
        if (argc < 3) { ui_console_write("ERR: usage svc target <default|rescue|all>\n"); return; }
        if (ui_streq(argv[2], "default")) ui_service_target = UI_SVC_TARGET_DEFAULT;
        else if (ui_streq(argv[2], "rescue")) ui_service_target = UI_SVC_TARGET_RESCUE;
        else if (ui_streq(argv[2], "all")) ui_service_target = UI_SVC_TARGET_ALL;
        else { ui_console_write("ERR: bad target\n"); return; }
        ui_console_write("OK: target updated\n");
        return;
    }
    if (ui_streq(argv[1], "clear")) {
        for (u32 i = 0; i < UI_SVC_MAX; i++) {
            if (!ui_services[i].used) continue;
            if (ui_services[i].pid > 0) (void)proc_kill_pid((u32)ui_services[i].pid, 0xFFFF4000U);
            ui_services[i].used = false;
        }
        ui_console_write("OK: services cleared\n");
        return;
    }
    if (ui_streq(argv[1], "list")) {
        for (u32 i = 0; i < UI_SVC_MAX; i++) {
            struct ui_service_unit *s = &ui_services[i];
            if (!s->used) continue;
            fb_printf("svc %-12s state=%u pid=%x dep=%s path=%s\n",
                      s->name, s->state, (u32)(s->pid > 0 ? s->pid : 0),
                      s->depends[0] ? s->depends : "-", s->path);
        }
        return;
    }
    if ((ui_streq(argv[1], "start") || ui_streq(argv[1], "stop") || ui_streq(argv[1], "restart")) && argc >= 3) {
        struct ui_service_unit *s = ui_service_find(argv[2]);
        if (!s) { ui_console_write("ERR: service not found\n"); return; }
        if (ui_streq(argv[1], "start")) {
            s->state = 0;
            s->stop_requested = false;
            s->next_action_ms = timer_ticks();
            ui_console_write("OK: service start queued\n");
        } else if (ui_streq(argv[1], "stop")) {
            s->stop_requested = true;
            if (s->pid > 0) (void)proc_kill_pid((u32)s->pid, 0xFFFF4001U);
            s->state = 0;
            s->pid = -1;
            ui_console_write("OK: service stopped\n");
        } else {
            s->stop_requested = true;
            if (s->pid > 0) (void)proc_kill_pid((u32)s->pid, 0xFFFF4002U);
            s->pid = -1;
            s->stop_requested = false;
            s->state = 0;
            s->next_action_ms = timer_ticks();
            ui_console_write("OK: service restart queued\n");
        }
        return;
    }
    ui_console_write("ERR: unknown svc subcommand\n");
}

static void ui_scheduler_exec_line(const char *line)
{
    if (!line || !*line) return;
    char cmd[320];
    u32 p = 0;
    const char *prefix = "batch ";
    for (u32 i = 0; prefix[i] && p + 1 < sizeof(cmd); i++) cmd[p++] = prefix[i];
    const char *s = line;
    bool known =
        (s[0]=='a'&&s[1]=='d'&&s[2]=='d'&&s[3]==' ') ||
        (s[0]=='a'&&s[1]=='t'&&s[2]==' ') ||
        (s[0]=='e'&&s[1]=='v'&&s[2]=='e'&&s[3]=='r'&&s[4]=='y'&&s[5]==' ') ||
        ui_streq(s, "run") || ui_streq(s, "stop") || ui_streq(s, "status") ||
        ui_streq(s, "list") || ui_streq(s, "clear") || ui_streq(s, "help");
    if (!known) {
        const char *add = "add ";
        for (u32 i = 0; add[i] && p + 1 < sizeof(cmd); i++) cmd[p++] = add[i];
    }
    while (*s && p + 1 < sizeof(cmd)) cmd[p++] = *s++;
    cmd[p] = 0;
    ui_console_exec(cmd);
}

static void ui_scheduler_feed_char(i32 c)
{
    if (c < 0) return;
    if (c == '\r' || c == '\n') {
        ui_sched_line[ui_sched_len] = 0;
        ui_scheduler_exec_line(ui_sched_line);
        ui_sched_len = 0;
        ui_last_render = 0;
        return;
    }
    if (c == '\b' || c == 127) {
        if (ui_sched_len > 0) ui_sched_len--;
        ui_last_render = 0;
        return;
    }
    if (c < 0x20 || c > 0x7E) return;
    if (ui_sched_len + 1 >= UI_CONSOLE_LINE_MAX) return;
    ui_sched_line[ui_sched_len++] = (char)c;
    ui_last_render = 0;
}

static void ui_render_scheduler(void)
{
    fb_clear(UI_SHELL_BG_COLOR);
    fb_set_color(BOOT_FG_PINK, UI_SHELL_BG_COLOR);
    fb_printf("PIOS Scheduler (F4)\n");
    fb_set_color(0x00FFFFFF, UI_SHELL_BG_COLOR);
    fb_printf("========================================\n");
    fb_printf("Type: <path> OR add/at/every/run/stop/status/list/clear then Enter\n");
    fb_printf("Examples: /bin/demo.pix | at 5000 /bin/demo.pix 2 0 1 | every 10000 /bin/job.pis auto 0\n");
    fb_printf("F3 console | F2 manager | F1 process view\n\n");

    u32 q = 0, r = 0, d = 0, f = 0;
    for (u32 i = 0; i < UI_BATCH_MAX; i++) {
        struct ui_batch_job *j = &ui_batch_jobs[i];
        if (!j->used) continue;
        if (j->state == 0) q++;
        else if (j->state == 1) r++;
        else if (j->state == 2) d++;
        else if (j->state == 3) f++;
    }
    fb_printf("mode=%s parallel=%u queued=%u running=%u done=%u failed=%u\n\n",
              ui_batch_running ? "run" : "pause", ui_batch_parallel, q, r, d, f);
    fb_printf("sched> %s", ui_sched_line);
}

static bool ui_console_help_topic(const char *topic)
{
    if (!topic)
        return false;
    if (ui_streq(topic, "status")) {
        ui_console_write("status\n  Show system/build/network summary.\n");
    } else if (ui_streq(topic, "ps") || ui_streq(topic, "processes")) {
        ui_console_write("ps\n  Show process list.\n");
        ui_console_write("kill <pid>\n  Kill process by numeric/hex pid.\n");
        ui_console_write("launch <path> [1|2|3] [lazy|low|normal|high|realtime]\n  Launch executable on a core.\n");
    } else if (ui_streq(topic, "netstat")) {
        ui_console_write("netstat\n  Show live TCP listeners/sessions, owners, buffers, retries, and firewall drops.\n");
    } else if (ui_streq(topic, "mem")) {
        ui_console_write("mem analyze\n  Show kernel image, raw-slot, per-core RAM, and process memory layout diagnostics.\n");
    } else if (ui_streq(topic, "http") || ui_streq(topic, "https")) {
        ui_console_write("http get <ip-or-cached-host> [path] [port] [timeout_ms]\n  Fetch plaintext HTTP over TCP; hostnames use DNS cache.\n");
        ui_console_write("https get <ip-or-cached-host> [path] [port] [timeout_ms]\n  Fetch through the PIOS kernel TLS-wrapper endpoint, not browser HTTPS.\n");
    } else if (ui_streq(topic, "dns")) {
        ui_console_write("dns resolve <hostname>\n  Start async A-record lookup.\n");
        ui_console_write("dns status\n  Poll resolver job status.\n");
        ui_console_write("dns flush\n  Flush DNS cache.\n");
    } else if (ui_streq(topic, "firewall")) {
        ui_console_write("firewall list\n  List rules.\n");
        ui_console_write("firewall reset\n  Restore inbound deny/outbound allow defaults plus service allows.\n");
        ui_console_write("firewall allow|deny <in|out|both> <tcp|udp|icmp|ip|arp> [port N|toport N|fromport N] [src SPEC] [dst SPEC]\n");
        ui_console_write("SPEC: any | a.b.c.d | a.b.c.d/prefix | a.b.c.d/mask | a.b.c.d-a.b.c.d\n");
    } else if (ui_streq(topic, "reboot")) {
        ui_console_write("reboot confirm\n  Reboot via PSCI SYSTEM_RESET; confirmation word is required.\n");
    } else if (ui_streq(topic, "bootctrl")) {
        ui_console_write("bootctrl status\nbootctrl clear-pending\nbootctrl reset-a confirm\nbootctrl test-invalid-b confirm\n  Show/repair/test stage0 A/B boot-control state without host raw-disk access.\n");
    } else if (ui_streq(topic, "watchdog")) {
        ui_console_write("watchdog status\nwatchdog arm|disarm\nwatchdog timeout <ticks>\nwatchdog mode <halt|reboot>\nwatchdog trip\nwatchdog hw-arm <secs>\nwatchdog hw-pet\nwatchdog hw-disable\nwatchdog hw-trip confirm\n  Hardware BCM2712 PM watchdog at 0x107D200000 (arms a chip-level auto-reset).\n");
    } else if (ui_streq(topic, "dma")) {
        ui_console_write("dma status\n  Show DMA enable state, CB address mode, selftest counters, and channel registers.\n");
        ui_console_write("dma selftest\n  Re-run the memcpy selftest and auto-select raw/shifted CB address mode if hardware passes.\n");
    } else if (ui_streq(topic, "keystore")) {
        ui_console_write("keystore status\n  Show sealed-root status, user-records LBA, and non-secret fingerprint.\n");
        ui_console_write("keystore derive <label>\n  Derive and print a non-secret fingerprint for a label.\n");
    } else if (ui_streq(topic, "tls")) {
        ui_console_write("tls status\n  Show kernel TLS connection, record, selftest, and bridge diagnostics.\n");
        ui_console_write("tls selftest\n  Verify client/server key agreement and AES-GCM record compatibility.\n");
        ui_console_write("tls bridge\n  Parse a sample plaintext HTTP request through the PicoWeb-style TLS bridge boundary.\n");
    } else if (ui_streq(topic, "brotli")) {
        ui_console_write("brotli selftest\n  Verify PIOS Brotli stored encoder and PicoWeb micro-Brotli decoder.\n");
    } else if (ui_streq(topic, "x509")) {
        ui_console_write("x509 status\n  Show kernel cert/key service state, DER/CSR lengths, and fingerprints.\n");
        ui_console_write("x509 generate [cn]\n  Generate a keystore-backed self-signed Ed25519 DER certificate.\n");
        ui_console_write("x509 csr [cn]\n  Generate a keystore-backed Ed25519 PKCS#10 CSR.\n");
        ui_console_write("x509 p256 [cn]\n  Generate a keystore-backed ECDSA P-256 PKCS#10 CSR for ACME.\n");
        ui_console_write("x509 bind\n  Mark the generated DER certificate as bound to kernel TLS.\n");
        ui_console_write("x509 import-self\n  Re-import the current DER certificate to exercise the import API.\n");
        ui_console_write("x509 selftest\n  Generate and bind a selftest DER certificate and CSR.\n");
    } else if (ui_streq(topic, "acme")) {
        ui_console_write("acme status\n  Show ACME account, CSR, and HTTP-01 challenge state.\n");
        ui_console_write("acme prepare <domain>\n  Generate an ACME-compatible P-256 CSR for the domain.\n");
        ui_console_write("acme csrhex\n  Export the prepared CSR DER as hex.\n");
        ui_console_write("acme challenge <token> <keyauth>\n  Arm /.well-known/acme-challenge/<token>.\n");
        ui_console_write("acme clear\n  Clear the current HTTP-01 challenge.\n");
        ui_console_write("acme selftest\n  Exercise CSR prep and challenge response state.\n");
    } else if (ui_streq(topic, "ksvc")) {
        ui_console_write("ksvc status\n  Show kernel service/plugin registry, mailbox counters, and runtime counters.\n");
        ui_console_write("ksvc selftest\n  Round-trip mailbox and fault/restart policy.\n");
        ui_console_write("ksvc pause|resume|restart|fault <id>\n  Update lifecycle state metadata for a registered service.\n");
    } else if (ui_streq(topic, "irq")) {
        ui_console_write("irq status\n  Show IRQ counters plus vector/GIC/timer register diagnostics.\n");
        ui_console_write("irq probe\n  Read plausible GIC distributor/interface windows without enabling IRQ-driven loops.\n");
        ui_console_write("irq selftest\n  Verify diagnostic invariants without enabling IRQ-driven service loops.\n");
        ui_console_write("irq cntpns confirm\n  Watchdog-protected CNTPNS/PPI30 delivery test. Auto-resets within 15s if it wedges.\n");
    } else if (ui_streq(topic, "abi")) {
        ui_console_write("abi status\n  Show current direct-KPI/ksvc/EL0 transition state.\n");
        ui_console_write("abi selftest\n  Verify ABI transition metadata invariants.\n");
    } else if (ui_streq(topic, "qpu") || ui_streq(topic, "tensor")) {
        ui_console_write("qpu status\n  Show V3D/QPU tensor dispatch diagnostics.\n");
        ui_console_write("tensor selftest\n  Verify safe NEON fallback tensor kernels.\n");
    } else if (ui_streq(topic, "files") || ui_streq(topic, "fs")) {
        ui_console_write("pwd | cd <path> | lsdir [path]\n");
        ui_console_write("mkdir <path> | touch <path> | cat <path> | stat <path> | rm <path>\n");
        ui_console_write("copy|cp <src> <dst> | cpdir <src_dir> <dst_dir> | mv <src> <dst>\n");
        ui_console_write("hexdump <path> [max_bytes] | find <dir> <needle> | df\n");
    } else if (ui_streq(topic, "disk")) {
        ui_console_write("disk info|sync|compact|verify|read <lba>|writezero <lba> --force\n");
    } else if (ui_streq(topic, "db")) {
        ui_console_write("db key <card> <record>\n");
        ui_console_write("db put|get|del <card> <record> ...\n");
        ui_console_write("db putf|getf <card> <record> <path>\n");
        ui_console_write("db list <card>\n");
    } else if (ui_streq(topic, "netcfg")) {
        ui_console_write("netcfg\n  Show current network config.\n");
        ui_console_write("netcfg set <ip|mask|gw|dns> <a.b.c.d>\nnetcfg apply\nnetcfg dhcp <on|off> [timeout_ms]\nnetcfg addnbr <ip> <mac>\nnetcfg routes|neighbors|trace\nnetcfg route add <dst> <mask> <gw> <connected|via>\n");
    } else if (ui_streq(topic, "stream")) {
        ui_console_write("stream <tcp|udp> <ip> <port> from <file|text|tty> <arg?> to <console|file> [path] [timeout_ms]\n");
        ui_console_write("http get <ip-or-cached-host> [path] [port] [timeout_ms]\nhttps get <ip-or-cached-host> [path] [port] [timeout_ms]\n");
    } else if (ui_streq(topic, "svc")) {
        ui_console_write("svc add <name> <path> [dep|-] [target] [1|2|3] [priority] [principal] [restart] [max] [backoff]\n");
        ui_console_write("svc start|stop|restart <name> | svc run|pause | svc target <default|rescue|all> | svc list | svc clear\n");
    } else if (ui_streq(topic, "batch")) {
        ui_console_write("batch add|at|every ...\nbatch run [parallel] | batch stop | batch status | batch list\n");
    } else if (ui_streq(topic, "env")) {
        ui_console_write("env list|get|set|pset|unset|save|load\n");
    } else if (ui_streq(topic, "update")) {
        ui_console_write("update status|stage <slot> [tries]|success\n");
    } else if (ui_streq(topic, "edit")) {
        ui_console_write("edit <path>\nedit.pix <path>\n  Ctrl+S save, Ctrl+Q exit.\n");
    } else if (ui_streq(topic, "usb")) {
        ui_console_write("usb status|reinit|poll\n");
    } else if (ui_streq(topic, "dev")) {
        ui_console_write("usb status|reinit|poll\ncapsule ...\nobs ...\nhexsec <lba>\n");
        ui_console_write("peek <addr> [1|2|4|8]\npoke <addr> <value> [1|2|4|8]\ndumpmem <addr> [bytes]\n");
    } else if (ui_streq(topic, "peek")) {
        ui_console_write("peek <addr> [1|2|4|8]\n  Read memory width in bytes. Example: peek 0x1000FFF000 4\n");
    } else if (ui_streq(topic, "poke")) {
        ui_console_write("poke <addr> <value> [1|2|4|8]\n  Write memory width in bytes. Dangerous: live kernel memory/device access.\n");
    } else if (ui_streq(topic, "dumpmem")) {
        ui_console_write("dumpmem <addr> [bytes]\n  Dump up to 4096 bytes from a memory address.\n");
    } else {
        return false;
    }
    return true;
}

static void ui_console_exec(char *line)
{
    char *argv[UI_CONSOLE_ARGV_MAX];
    u32 argc = 0;
    char *p = line;

    while (*p && argc < UI_CONSOLE_ARGV_MAX) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = 0;
    }
    if (argc == 0) return;

    if (ui_streq(argv[0], "help")) {
        if (argc < 2) {
            ui_console_write("PIOS help\n");
            ui_console_write("Run commands exactly as shown; category names are help topics, not prefixes.\n");
            ui_console_write("Examples: status | ps | netstat | addr wal:0/3 | firewall list | bootctrl status | reboot confirm\n");
            ui_console_write("Command help: help <command>, e.g. help status, help firewall, help reboot, help tls\n");
            ui_console_write("Category help: help core | help fs | help net | help svc | help dev\n");
        } else if (ui_streq(argv[1], "core")) {
            ui_console_write("Core/process commands:\n");
            ui_console_write("  status\n  time\n  ps\n  kill <pid>\n");
            ui_console_write("  mem analyze\n");
            ui_console_write("  launch|run <path> [1|2|3] [priority]\n");
            ui_console_write("  prio <pid> <lazy|low|normal|high|realtime>\n");
            ui_console_write("  affinity <pid> <1|2|3>\n");
            ui_console_write("  watchdog status|arm|disarm|timeout <ticks>|mode <halt|reboot>|trip\n");
            ui_console_write("  bootctrl status|clear-pending|reset-a confirm|test-invalid-b confirm\n");
            ui_console_write("  reboot confirm\n");
        } else if (ui_streq(argv[1], "fs")) {
            ui_console_write("Filesystem/storage commands:\n");
            ui_console_write("  pwd\n  cd <path>\n  lsdir [path]\n  mkdir <path>\n  touch <path>\n");
            ui_console_write("  copy|cp <src> <dst>\n  cpdir <src_dir> <dst_dir>\n  mv <src> <dst>\n");
            ui_console_write("  cat <path>\n  stat <path>\n  rm <path>\n  find <dir> <needle>\n");
            ui_console_write("  hexdump <path> [max_bytes] fsinspect <path>\n");
            ui_console_write("  df mount umount disk [info|sync|compact|verify|read|writezero]\n");
            ui_console_write("  db key|put|putf|get|getf|del|list\n");
        } else if (ui_streq(argv[1], "net")) {
            ui_console_write("Network/firewall commands:\n");
            ui_console_write("  netstat netcfg [set|apply|dhcp|addnbr]\n");
            ui_console_write("  firewall list|reset|clear|remove <idx>|default in <allow|deny> out <allow|deny>\n");
            ui_console_write("  firewall allow|deny <in|out|both> <tcp|udp|icmp|ip|arp> [port N] [src SPEC] [dst SPEC]\n");
            ui_console_write("  stream <tcp|udp> <ip> <port> from <file|text|tty> <arg?> to <console|file> [path] [timeout_ms]\n");
            ui_console_write("  http get <ip-or-cached-host> [path] [port] [timeout_ms] | https get <ip-or-cached-host> [path] [port] [timeout_ms]\n");
            ui_console_write("  dns resolve <host> | dns status | dns flush\n");
            ui_console_write("  TCP debug console: port 2323, then 'unlock pios'\n");
        } else if (ui_streq(argv[1], "svc")) {
            ui_console_write("Service/script commands:\n");
            ui_console_write("  batch add|at|every|run|stop|status|list\n");
            ui_console_write("  svc add|start|stop|restart|run|pause|target|list|clear\n");
            ui_console_write("  source <script[.pis]>  env [list|get|set|pset|unset|save|load]\n");
            ui_console_write("  if for foreach update status|stage <slot> [tries]|success\n");
        } else if (ui_streq(argv[1], "dev")) {
            ui_console_write("Device/debug commands:\n");
            ui_console_write("  dma status|selftest  usb status|reinit|poll  wifi disabled\n");
            ui_console_write("  capsule ...  obs ...  hexsec <lba>\n");
            ui_console_write("  keystore status|derive <label>  edit|edit.pix <path>  clear echo\n");
            ui_console_write("  addr <kind:pack/card[/tail]>\n");
        } else if (!ui_console_help_topic(argv[1])) {
            ui_console_write("ERR: unknown help topic. Try: help, help core, help status, help firewall\n");
        }
    } else if (ui_streq(argv[0], "pwd")) {
        ui_console_write(ui_cwd);
        ui_console_write("\n");
    } else if (ui_streq(argv[0], "cd")) {
        char abs[256];
        const char *target = (argc < 2) ? "/" : argv[1];
        if (!ui_path_resolve(target, abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
        } else {
            u64 id = walfs_find(abs);
            struct walfs_inode ino;
            if (!id || !walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR)) {
                ui_console_write("ERR: directory not found\n");
            } else {
                u32 n = pios_strlen(abs);
                for (u32 i = 0; i <= n; i++) ui_cwd[i] = abs[i];
                ui_console_write("OK: cwd updated\n");
            }
        }
    } else if (ui_streq(argv[0], "echo")) {
        if (argc >= 4 && ui_streq(argv[argc - 2], ">")) {
            char dst[256];
            if (!ui_path_resolve(argv[argc - 1], dst, sizeof(dst))) {
                ui_console_write("ERR: bad path\n");
                return;
            }
            u64 dst_id = walfs_find(dst);
            if (!dst_id) {
                if (!ui_fs_create_path(dst, false)) {
                    ui_console_write("ERR: echo redirect create failed\n");
                    return;
                }
                dst_id = walfs_find(dst);
            }
            if (!dst_id) {
                ui_console_write("ERR: echo redirect target missing\n");
                return;
            }
            char out[256];
            u32 p = 0;
            for (u32 i = 1; i + 2 < argc; i++) {
                const char *s = argv[i];
                while (*s && p + 1 < sizeof(out)) out[p++] = *s++;
                if (i + 3 < argc && p + 1 < sizeof(out)) out[p++] = ' ';
            }
            if (p + 1 < sizeof(out)) out[p++] = '\n';
            out[p] = 0;
            if (!walfs_write(dst_id, 0, out, p)) {
                ui_console_write("ERR: echo redirect write failed\n");
                return;
            }
            ui_console_write("OK: wrote file\n");
        } else {
            for (u32 i = 1; i < argc; i++) {
                ui_console_write(argv[i]);
                if (i + 1 < argc) ui_console_write(" ");
            }
            ui_console_write("\n");
        }
    } else if (ui_streq(argv[0], "clear")) {
        uart_vt_title("PIOS Admin Console");
        uart_vt_clear();
        uart_vt_home();
        if (ui_mode == UI_MODE_CONSOLE) {
            fb_clear(UI_SHELL_BG_COLOR);
            fb_set_color(UI_SHELL_TEXT_COLOR, UI_SHELL_BG_COLOR);
            ui_console_write("PIOS F3 Console (serial + HDMI)\n");
        }
    } else if (ui_streq(argv[0], "time")) {
        u64 t = timer_ticks();
        fb_printf("ticks=%X\n", t);
        uart_puts("ticks=");
        uart_hex(t);
        uart_puts("\n");
    } else if (ui_streq(argv[0], "addr")) {
        ui_cmd_addr(argc, argv);
    } else if (ui_streq(argv[0], "netstat")) {
        ui_console_print_netstat();
    } else if (ui_streq(argv[0], "mem")) {
        ui_cmd_mem(argc, argv);
    } else if (ui_streq(argv[0], "http")) {
        ui_cmd_http_client(argc, argv, false);
    } else if (ui_streq(argv[0], "https")) {
        ui_cmd_http_client(argc, argv, true);
    } else if (ui_streq(argv[0], "dns")) {
        ui_cmd_dns(argc, argv);
    } else if (ui_streq(argv[0], "ps")) {
        ui_console_print_ps();
    } else if (ui_streq(argv[0], "kill")) {
        if (argc < 2) {
            ui_console_write("ERR: usage kill <pid>\n");
        } else {
            u32 pid = 0;
            if (!ui_parse_u32(argv[1], &pid) || !proc_kill_pid(pid, 0xFFFF2000U)) {
                ui_console_write("ERR: kill failed\n");
            } else {
                ui_console_write("OK: killed\n");
            }
        }
    } else if (ui_streq(argv[0], "launch") || ui_streq(argv[0], "run")) {
        if (argc < 2) {
            ui_console_write("ERR: usage launch <path> [1|2|3]\n");
        } else {
            if (ui_streq(argv[0], "run")) {
                char sp[256];
                if (ui_resolve_pis_path(argv[1], sp, sizeof(sp)) && ui_has_suffix(sp, ".pis") && walfs_find(sp)) {
                    ui_cmd_source(sp);
                    return;
                }
            }
            u32 target_core = CORE_USERM;
            if (argc >= 3) {
                if (argv[2][0] == '1') target_core = CORE_USERM;
                else if (argv[2][0] == '2') target_core = CORE_USER0;
                else if (argv[2][0] == '3') target_core = CORE_USER1;
            }
            u32 prio = PROC_PRIO_NORMAL;
            if (argc >= 4) {
                if (!ui_parse_priority(argv[3], &prio)) {
                    ui_console_write("ERR: invalid priority\n");
                    return;
                }
            } else if (argc >= 3 && argv[2][0] != '1' && argv[2][0] != '2' && argv[2][0] != '3') {
                if (!ui_parse_priority(argv[2], &prio)) {
                    ui_console_write("ERR: invalid core/priority\n");
                    return;
                }
            }
            i32 pid = proc_launch_on_core_as_prio(target_core, argv[1], principal_current(), prio);
            if (pid <= 0) {
                ui_console_write("ERR: launch failed\n");
            } else {
                fb_printf("OK: pid=0x%x\n", (u32)pid);
                uart_puts("OK: pid=");
                uart_hex((u32)pid);
                uart_puts("\n");
            }
        }
    } else if (ui_streq(argv[0], "prio")) {
        if (argc < 3) {
            ui_console_write("ERR: usage prio <pid> <lazy|low|normal|high|realtime>\n");
        } else {
            u32 pid = 0, p = 0;
            if (!ui_parse_u32(argv[1], &pid) || !ui_parse_priority(argv[2], &p) || !proc_set_priority(pid, p))
                ui_console_write("ERR: prio update failed\n");
            else
                ui_console_write("OK: priority updated\n");
        }
    } else if (ui_streq(argv[0], "affinity")) {
        if (argc < 3) {
            ui_console_write("ERR: usage affinity <pid> <1|2|3>\n");
        } else {
            u32 pid = 0;
            u32 core = CORE_USERM;
            if (!ui_parse_u32(argv[1], &pid)) {
                ui_console_write("ERR: invalid pid\n");
            } else {
                if (argv[2][0] == '1') core = CORE_USERM;
                else if (argv[2][0] == '2') core = CORE_USER0;
                else if (argv[2][0] == '3') core = CORE_USER1;
                else {
                    ui_console_write("ERR: core must be 1|2|3\n");
                    return;
                }
                if (!proc_set_affinity(pid, core))
                    ui_console_write("ERR: affinity update failed\n");
                else
                    ui_console_write("OK: affinity updated\n");
            }
        }
    } else if (ui_streq(argv[0], "hexsec")) {
        if (argc < 2) {
            ui_console_write("ERR: usage hexsec <lba>\n");
        } else {
            u32 lba = 0;
            if (!ui_parse_u32(argv[1], &lba)) {
                ui_console_write("ERR: invalid lba\n");
            } else {
                ui_dump_sector(lba);
            }
        }
    } else if (ui_streq(argv[0], "peek")) {
        ui_cmd_peek(argc, argv);
    } else if (ui_streq(argv[0], "poke")) {
        ui_cmd_poke(argc, argv);
    } else if (ui_streq(argv[0], "dumpmem")) {
        ui_cmd_dumpmem(argc, argv);
    } else if (ui_streq(argv[0], "fsinspect")) {
        if (argc < 2) ui_cmd_fsinspect(ui_cwd);
        else {
            char abs[256];
            if (!ui_path_resolve(argv[1], abs, sizeof(abs))) ui_console_write("ERR: bad path\n");
            else ui_cmd_fsinspect(abs);
        }
    } else if (ui_streq(argv[0], "lsdir")) {
        if (argc < 2) ui_cmd_lsdir(ui_cwd);
        else ui_cmd_lsdir(argv[1]);
    } else if (ui_streq(argv[0], "mkdir")) {
        if (argc < 2) ui_console_write("ERR: usage mkdir <path>\n");
        else ui_cmd_mkdir(argv[1]);
    } else if (ui_streq(argv[0], "touch")) {
        if (argc < 2) ui_console_write("ERR: usage touch <path>\n");
        else ui_cmd_touch(argv[1]);
    } else if (ui_streq(argv[0], "copy")) {
        if (argc < 3) ui_console_write("ERR: usage copy <src> <dst>\n");
        else ui_cmd_copy(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "cp")) {
        if (argc < 3) ui_console_write("ERR: usage cp <src> <dst>\n");
        else ui_cmd_copy(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "cpdir")) {
        if (argc < 3) ui_console_write("ERR: usage cpdir <src_dir> <dst_dir>\n");
        else ui_cmd_cpdir(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "mv")) {
        if (argc < 3) ui_console_write("ERR: usage mv <src> <dst>\n");
        else ui_cmd_mv(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "cat")) {
        if (argc < 2) ui_console_write("ERR: usage cat <path>\n");
        else ui_cmd_cat(argv[1]);
    } else if (ui_streq(argv[0], "stat")) {
        if (argc < 2) ui_console_write("ERR: usage stat <path>\n");
        else ui_cmd_stat(argv[1]);
    } else if (ui_streq(argv[0], "rm")) {
        if (argc < 2) ui_console_write("ERR: usage rm <path>\n");
        else ui_cmd_rm(argv[1]);
    } else if (ui_streq(argv[0], "find")) {
        if (argc < 3) ui_console_write("ERR: usage find <dir> <needle>\n");
        else ui_cmd_find(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "hexdump")) {
        u32 max = 256;
        if (argc < 2) ui_console_write("ERR: usage hexdump <path> [max_bytes]\n");
        else {
            if (argc >= 3 && !ui_parse_u32(argv[2], &max)) {
                ui_console_write("ERR: invalid max_bytes\n");
            } else {
                ui_cmd_hexdump(argv[1], max);
            }
        }
    } else if (ui_streq(argv[0], "df")) {
        ui_cmd_df();
    } else if (ui_streq(argv[0], "mount")) {
        ui_cmd_mount(argc, argv);
    } else if (ui_streq(argv[0], "umount")) {
        ui_cmd_mount(argc, argv);
    } else if (ui_streq(argv[0], "stream")) {
        ui_cmd_stream(argc, argv);
    } else if (ui_streq(argv[0], "env")) {
        ui_cmd_env(argc, argv);
    } else if (ui_streq(argv[0], "if")) {
        ui_cmd_if(argc, argv);
    } else if (ui_streq(argv[0], "for")) {
        ui_cmd_for(argc, argv);
    } else if (ui_streq(argv[0], "foreach")) {
        ui_cmd_foreach(argc, argv);
    } else if (ui_streq(argv[0], "source")) {
        if (argc < 2) ui_console_write("ERR: usage source <path>\n");
        else ui_cmd_source(argv[1]);
    } else if (ui_streq(argv[0], "edit") || ui_streq(argv[0], "edit.pix")) {
        if (argc < 2) ui_console_write("ERR: usage edit <path>\n");
        else ui_cmd_edit(argv[1]);
    } else if (ui_streq(argv[0], "batch")) {
        ui_cmd_batch(argc, argv);
    } else if (ui_streq(argv[0], "svc")) {
        ui_cmd_svc(argc, argv);
    } else if (ui_streq(argv[0], "wifi")) {
        ui_console_write("ERR: wifi support removed (parked in spike/wifi/, see GitHub issue)\n");
    } else if (ui_streq(argv[0], "usb")) {
        ui_cmd_usb(argc, argv);
    } else if (ui_streq(argv[0], "dma")) {
        ui_cmd_dma(argc, argv);
    } else if (ui_streq(argv[0], "keystore")) {
        ui_cmd_keystore(argc, argv);
    } else if (ui_streq(argv[0], "tls")) {
        ui_cmd_tls(argc, argv);
    } else if (ui_streq(argv[0], "brotli")) {
        ui_cmd_brotli(argc, argv);
    } else if (ui_streq(argv[0], "x509")) {
        ui_cmd_x509(argc, argv);
    } else if (ui_streq(argv[0], "acme")) {
        ui_cmd_acme(argc, argv);
    } else if (ui_streq(argv[0], "ksvc")) {
        ui_cmd_ksvc(argc, argv);
    } else if (ui_streq(argv[0], "irq")) {
        ui_cmd_irq(argc, argv);
    } else if (ui_streq(argv[0], "abi")) {
        ui_cmd_abi(argc, argv);
    } else if (ui_streq(argv[0], "qpu") || ui_streq(argv[0], "tensor")) {
        ui_cmd_tensor(argc, argv);
    } else if (ui_streq(argv[0], "netcfg")) {
        ui_cmd_netcfg(argc, argv);
    } else if (ui_streq(argv[0], "firewall")) {
        ui_cmd_firewall(argc, argv);
    } else if (ui_streq(argv[0], "disk")) {
        ui_cmd_disk(argc, argv);
    } else if (ui_streq(argv[0], "db")) {
        ui_cmd_db(argc, argv);
    } else if (ui_streq(argv[0], "capsule")) {
        ui_cmd_capsule(argc, argv);
    } else if (ui_streq(argv[0], "obs")) {
        ui_cmd_obs(argc, argv);
    } else if (ui_streq(argv[0], "update")) {
        ui_cmd_update(argc, argv);
    } else if (ui_streq(argv[0], "watchdog")) {
        ui_cmd_watchdog(argc, argv);
    } else if (ui_streq(argv[0], "bootctrl")) {
        ui_cmd_bootctrl(argc, argv);
    } else if (ui_streq(argv[0], "reboot")) {
        if (argc >= 2 && ui_streq(argv[1], "confirm")) {
            ui_console_write("OK: rebooting via PSCI SYSTEM_RESET...\n");
            http_log_event("console-reboot", 0, 0);
            timer_delay_ms(100);
            watchdog_reboot_now(0x434F4E52U);
        } else {
            ui_console_write("ERR: usage reboot confirm\n");
        }
    } else {
        ui_console_write("ERR: unknown command\n");
    }
}

static void ui_console_feed_char(i32 c)
{
    if (c < 0) return;
    if (c == '\r' || c == '\n') {
        ui_console_write("\n");
        ui_console_line[ui_console_len] = 0;
        ui_console_exec(ui_console_line);
        ui_console_len = 0;
        ui_console_prompt();
        return;
    }
    if (c == '\b' || c == 127) {
        if (ui_console_len > 0) {
            ui_console_len--;
            ui_console_write("\b \b");
        }
        return;
    }
    if (c < 0x20 || c > 0x7E)
        return;
    if (ui_console_len + 1 >= UI_CONSOLE_LINE_MAX) {
        ui_console_write("\nERR: line too long\n");
        ui_console_len = 0;
        ui_console_prompt();
        return;
    }
    ui_console_line[ui_console_len++] = (char)c;
    char out[2] = { (char)c, 0 };
    ui_console_write(out);
}

static void ui_console_pump_input(void)
{
    i32 c;
    while ((c = usb_kbd_try_getc()) >= 0)
        ui_console_feed_char(c);
    while ((c = uart_try_getc()) >= 0)
        ui_console_feed_char(c);
}

static const char *ui_proc_state_str(u32 s)
{
    if (s == PROC_READY) return "ready";
    if (s == PROC_RUNNING) return "running";
    if (s == PROC_BLOCKED) return "blocked";
    if (s == PROC_DEAD) return "dead";
    return "unknown";
}

static void ui_render_process_view(void)
{
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    fb_clear(UI_SHELL_BG_COLOR);
    fb_set_color(BOOT_FG_PINK, UI_SHELL_BG_COLOR);
    fb_printf("PIOS Process View (F1 cycle, F2 manager)\n");
    fb_set_color(0x00FFFFFF, UI_SHELL_BG_COLOR);
    fb_printf("========================================\n\n");

    if (n == 0) {
        fb_printf("No active processes.\n");
        return;
    }
    if (ui_selected >= n)
        ui_selected = 0;

    struct proc_ui_entry *e = &snap[ui_selected];
    const char *keys[] = {
        "pid", "ppid", "state", "affinity", "cpu_pct", "mem_kib", "arena_cap_kib",
        "arena_used_kib", "arena_high_kib", "arena_bump_kib", "arena_span_kib",
        "span_count", "principal", "preemptions"
    };
    u32 vals[] = {
        e->pid, e->parent_pid, e->state, e->affinity_core, e->cpu_percent,
        e->mem_kib, e->arena_capacity_kib, e->arena_used_kib, e->arena_high_kib,
        e->arena_bump_kib, e->arena_span_kib, e->arena_span_count,
        e->principal_id, e->preemptions
    };

    fb_printf("Process %u / %u\n\n", ui_selected + 1, n);
    for (u32 i = 0; i < 14; i++) {
        if (i == 2) {
            fb_printf("%s: %s\n", keys[i], ui_proc_state_str(vals[i]));
        } else if (i == 0 || i == 1) {
            if (i == 1 && vals[i] == PROC_UI_KERNEL_PARENT_PID)
                fb_printf("%s: -1\n", keys[i]);
            else
                fb_printf("%s: 0x%x\n", keys[i], vals[i]);
        } else {
            fb_printf("%s: %u\n", keys[i], vals[i]);
        }
    }
    fb_printf("runtime_ticks: %X\n", e->runtime_ticks);
}

static void ui_render_process_manager(void)
{
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    fb_clear(UI_SHELL_BG_COLOR);
    fb_set_color(BOOT_FG_PINK, UI_SHELL_BG_COLOR);
    fb_printf("PIOS Process Manager (F2)\n");
    fb_set_color(0x00FFFFFF, UI_SHELL_BG_COLOR);
    fb_printf("========================================\n");
    fb_printf("Controls: F1 detail | K kill selected | L launch next | C open console\n\n");
    fb_printf("PID      PPID     AFF  CPU%%  MEM  ARENA  STATE\n");

    if (n == 0) {
        fb_printf("(no active processes)\n");
    } else {
        if (ui_selected >= n)
            ui_selected = 0;
        for (u32 i = 0; i < n; i++) {
            fb_set_color((i == ui_selected) ? 0x00FF9900 : 0x00FFFFFF, UI_SHELL_BG_COLOR);
            if (snap[i].parent_pid == PROC_UI_KERNEL_PARENT_PID) {
                fb_printf("0x%x   -1       %u    %u     %u   %u/%u  %s\n",
                          snap[i].pid, snap[i].affinity_core,
                          snap[i].cpu_percent, snap[i].mem_kib, snap[i].arena_used_kib,
                          snap[i].arena_high_kib, ui_proc_state_str(snap[i].state));
            } else {
                fb_printf("0x%x   0x%x   %u    %u     %u   %u/%u  %s\n",
                          snap[i].pid, snap[i].parent_pid, snap[i].affinity_core,
                          snap[i].cpu_percent, snap[i].mem_kib, snap[i].arena_used_kib,
                          snap[i].arena_high_kib, ui_proc_state_str(snap[i].state));
            }
        }
        fb_set_color(0x00FFFFFF, UI_SHELL_BG_COLOR);
    }

    if (ui_status_code == 1) fb_printf("\nstatus: killed selected pid\n");
    else if (ui_status_code == 2) fb_printf("\nstatus: launch request submitted\n");
    else if (ui_status_code == -1) fb_printf("\nstatus: action failed\n");
}

static void ui_handle_keys(void)
{
    i32 key;
    while ((key = usb_kbd_try_getkey()) >= 0) {
        if (key == USB_KBD_KEY_F1) {
            ui_mode = UI_MODE_PROC_VIEW;
            ui_selected++;
            ui_last_render = 0;
        } else if (key == USB_KBD_KEY_F2) {
            ui_mode = UI_MODE_PROC_MANAGER;
            ui_last_render = 0;
        } else if (key == USB_KBD_KEY_F3) {
            ui_mode = UI_MODE_CONSOLE;
            ui_console_len = 0;
            fb_clear(UI_SHELL_BG_COLOR);
            fb_set_color(UI_SHELL_TEXT_COLOR, UI_SHELL_BG_COLOR);
            ui_console_write("PIOS F3 Console (serial + HDMI)\n");
            ui_console_write("Type 'help' for commands.\n");
            ui_console_prompt();
            ui_last_render = 1;
        } else if (key == USB_KBD_KEY_F4) {
            ui_mode = UI_MODE_SCHEDULER;
            ui_sched_len = 0;
            ui_sched_line[0] = 0;
            ui_last_render = 0;
        }
    }

    if (ui_mode == UI_MODE_CONSOLE) {
        ui_console_pump_input();
        return;
    }

    if (ui_mode == UI_MODE_SCHEDULER) {
        i32 c;
        while ((c = usb_kbd_try_getc()) >= 0)
            ui_scheduler_feed_char(c);
        while ((c = uart_try_getc()) >= 0)
            ui_scheduler_feed_char(c);
        return;
    }

    if (ui_mode != UI_MODE_PROC_MANAGER)
        return;

    i32 c;
    while ((c = usb_kbd_try_getc()) >= 0) {
        if (c == 'k' || c == 'K') {
            struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
            u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
            if (n && ui_selected < n && proc_kill_pid(snap[ui_selected].pid, 0xFFFF1000U)) {
                ui_status_code = 1;
            } else {
                ui_status_code = -1;
            }
            ui_last_render = 0;
        } else if (c == 'l' || c == 'L') {
            ui_launch_idx++;
            if (ui_launch_idx >= (i32)(sizeof(ui_launch_candidates)/sizeof(ui_launch_candidates[0])))
                ui_launch_idx = 0;
            u32 core = (ui_launch_idx % 3 == 0) ? CORE_USERM :
                       (ui_launch_idx % 3 == 1) ? CORE_USER0 : CORE_USER1;
            i32 pid = proc_launch_on_core(core, ui_launch_candidates[ui_launch_idx]);
            ui_status_code = (pid > 0) ? 2 : -1;
            ui_last_render = 0;
        } else if (c == 'c' || c == 'C') {
            ui_mode = UI_MODE_CONSOLE;
            ui_console_len = 0;
            fb_clear(UI_SHELL_BG_COLOR);
            fb_set_color(UI_SHELL_TEXT_COLOR, UI_SHELL_BG_COLOR);
            ui_console_write("PIOS F3 Console (serial + HDMI)\n");
            ui_console_write("Type 'help' for commands.\n");
            ui_console_prompt();
            ui_last_render = 1;
            return;
        } else if (c == 'j' || c == 'J') {
            ui_selected++;
            ui_last_render = 0;
        } else if ((c == 'u' || c == 'U') && ui_selected > 0) {
            ui_selected--;
            ui_last_render = 0;
        }
    }
}

/* ---- Core entry points ---- */

/* Spinner state */
#define SPIN_GREEN   0x0000FF00
#define SPIN_YELLOW  0x00CCAA00
#define SPIN_PURPLE  0x00FF88CC
#define SPIN_RED     0x00FF2200

static u32 spin_color = SPIN_GREEN;
static u32 spin_counter;

/* Core 0 I/O reactor flags: timer IRQ marks these bits and wakes the
 * service loop. The HDMI dashboard decodes the last dispatched bitmask. */
#define CORE0_IO_NET     (1U << 0)
#define CORE0_IO_TCP     (1U << 1)
#define CORE0_IO_UART    (1U << 2)
#define CORE0_IO_USB     (1U << 3)
#define CORE0_IO_MAINT   (1U << 4)
#define CORE0_IO_DASH    (1U << 5)

static void spin_update(void) {
    static const char frames[] = "|/-\\";
    fb_set_cursor(126, 0);
    fb_set_color(spin_color, 0x00000000);
    fb_putc(frames[spin_counter & 3]);
    spin_counter++;
}

void spin_set_color(u32 color) {
    spin_color = color;
}

static bool dash_contains(const char *s, const char *needle)
{
    if (!s || !needle || !*needle)
        return false;
    for (u32 i = 0; s[i]; i++) {
        u32 j = 0;
        while (needle[j] && s[i + j] == needle[j])
            j++;
        if (!needle[j])
            return true;
    }
    return false;
}

static void dash_ip(u32 ip)
{
    fb_printf("%u.%u.%u.%u",
              (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
              (ip >> 8) & 0xFF, ip & 0xFF);
}

static void dash_core0_flags(u32 flags)
{
    bool any = false;
    if (flags & CORE0_IO_NET)   { fb_puts("NET");   any = true; }
    if (flags & CORE0_IO_TCP)   { if (any) fb_puts("|"); fb_puts("TCP");   any = true; }
    if (flags & CORE0_IO_UART)  { if (any) fb_puts("|"); fb_puts("UART");  any = true; }
    if (flags & CORE0_IO_USB)   { if (any) fb_puts("|"); fb_puts("USB");   any = true; }
    if (flags & CORE0_IO_MAINT) { if (any) fb_puts("|"); fb_puts("MAINT"); any = true; }
    if (flags & CORE0_IO_DASH)  { if (any) fb_puts("|"); fb_puts("DASH");  any = true; }
    if (!any) fb_puts("none");
}

static void hdmi_dashboard_render(void)
{
    static u64 last_ms;
    static u32 heartbeat;
    static bool layout_drawn;
    u64 now_ms = timer_monotonic_ms();
    if (now_ms < last_ms + 1000ULL)
        return;

    last_ms = now_ms;
    heartbeat++;

    u64 used = 0;
    for (u32 i = 0; i < 4; i++) {
        struct core_env *e = core_env_of(i);
        if (e->id == i && e->ram_base == (u8 *)(usize)core_ram_bases[i] &&
            e->ram_end == e->ram_base + CORE_PRIV_SIZE &&
            e->heap_ptr >= e->ram_base && e->heap_ptr <= e->ram_end)
            used += (u64)(usize)(e->heap_ptr - e->ram_base);
    }
    u64 total = CORE_PRIV_SIZE * 4ULL;
    u32 core_mem_kib[4];
    for (u32 i = 0; i < 4; i++)
        core_mem_kib[i] = http_core_ram_used_kib(i);

    u64 sched_wake = 0, sched_wfi = 0, sched_idle = 0, sched_total = 0;
    u32 sched_busy = 0, sched_flags = 0;
    core0_sched_snapshot(&sched_wake, &sched_wfi, &sched_idle, &sched_total,
                         &sched_busy, &sched_flags);

    nic_packet_counters_t pc;
    nic_packet_counters(&pc);
    tcp_snapshot_entry_t tcp[TCP_MAX_CONNECTIONS];
    u32 tcp_n = tcp_snapshot(tcp, TCP_MAX_CONNECTIONS);
    struct proc_ui_entry proc[UI_SNAPSHOT_MAX];
    u32 proc_n = proc_snapshot(proc, UI_SNAPSHOT_MAX);
    u32 kernel_mem_kib = proc_n ? proc[0].mem_kib : 0;
    u32 kernel_static_kib = proc_n ? proc[0].arena_span_kib : 0;
    u32 kernel_core_kib = proc_n ? proc[0].arena_used_kib : 0;
    u32 kernel_cap_kib = proc_n ? proc[0].arena_capacity_kib : 0;
    u32 ready = 0, running = 0, blocked = 0, dead = 0;
    for (u32 i = 0; i < proc_n; i++) {
        if (proc[i].state == PROC_READY) ready++;
        else if (proc[i].state == PROC_RUNNING) running++;
        else if (proc[i].state == PROC_BLOCKED) blocked++;
        else if (proc[i].state == PROC_DEAD) dead++;
    }

    u32 log_top = fb_rows() > 10 ? fb_rows() - 9 : 28;
    if (!layout_drawn) {
        fb_clear(0x00000000);
        fb_set_color(0x0000FF80, 0x00000000);
        fb_set_cursor(0, 0);
        fb_box(78, 4, "PIOS>");
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_set_cursor(0, 5);
        fb_box(78, 5, "SYSTEM");
        fb_set_color(0x00FFAA00, 0x00000000);
        fb_set_cursor(0, 11);
        fb_box(78, 7, "LISTENING PORTS");
        fb_set_color(0x0080FF80, 0x00000000);
        fb_set_cursor(0, 19);
        fb_box(78, 7, "PROCESSES");
        fb_set_color(0x00FF4040, 0x00000000);
        fb_set_cursor(0, log_top);
        fb_box(78, 8, "WARNINGS / ERRORS");
        layout_drawn = true;
    }

    fb_clear_row(1);
    fb_clear_row(2);
    fb_set_cursor(3, 1);
    fb_set_color(0x00FF80FF, 0x00000000);
    fb_puts(PIOS_BUILD_LABEL);
    fb_set_cursor(3, 2);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("uptime=");
    fb_printf("%u", (u32)(now_ms / 1000ULL));
    fb_puts("s hb=");
    fb_printf("%u", heartbeat);
    fb_puts(" ip=");
    dash_ip(net_get_our_ip());

    fb_clear_row(6);
    fb_clear_row(7);
    fb_clear_row(8);
    fb_set_cursor(3, 6);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("CPU0 busy=%u.%u%%  WFI=%u wake=%u flags=0x%x ",
              sched_busy / 10U, sched_busy % 10U,
              (u32)sched_wfi, (u32)sched_wake, sched_flags);
    dash_core0_flags(sched_flags);
    fb_set_cursor(3, 7);
    fb_printf("Kernel mem=%uK static=%uK core_alloc=%uK cap=%uK",
              kernel_mem_kib, kernel_static_kib, kernel_core_kib, kernel_cap_kib);
    fb_set_cursor(3, 8);
    fb_printf("RAM %uK/%uK  c0=%uK c1=%uK c2=%uK c3=%uK",
              (u32)(used / 1024ULL), (u32)(total / 1024ULL),
              core_mem_kib[0], core_mem_kib[1], core_mem_kib[2], core_mem_kib[3]);

    for (u32 r = 12; r < 17; r++) fb_clear_row(r);
    u32 row = 12;
    for (u32 i = 0; i < tcp_n && row < 17; i++) {
        if (tcp[i].state != TCP_LISTEN)
            continue;
        fb_set_cursor(3, row++);
        fb_puts("tcp/");
        fb_printf("%u", tcp[i].local_port);
        fb_puts("  ");
        fb_puts(tcp_owner_label(tcp[i].local_port));
        fb_puts("  pending=");
        fb_printf("%u", tcp[i].pending_count);
    }

    for (u32 r = 20; r < 25; r++) fb_clear_row(r);
    fb_set_cursor(3, 20);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("total=%u ready=%u running=%u blocked=%u dead=%u", proc_n, ready, running, blocked, dead);
    row = 21;
    for (u32 i = 0; i < proc_n && row < 25; i++) {
        fb_set_cursor(3, row++);
        if (proc[i].parent_pid == PROC_UI_KERNEL_PARENT_PID) {
            fb_printf("pid=0x%x ppid=-1 core=%u cpu=%u%% mem=%uK arena=%u/%uK %s %s",
                      proc[i].pid, proc[i].affinity_core,
                      proc[i].cpu_percent, proc[i].mem_kib, proc[i].arena_used_kib,
                      proc[i].arena_high_kib,
                      ui_proc_state_str(proc[i].state), proc[i].image_path);
        } else {
            fb_printf("pid=0x%x ppid=0x%x core=%u cpu=%u%% mem=%uK arena=%u/%uK %s %s",
                      proc[i].pid, proc[i].parent_pid, proc[i].affinity_core,
                      proc[i].cpu_percent, proc[i].mem_kib, proc[i].arena_used_kib,
                      proc[i].arena_high_kib,
                      ui_proc_state_str(proc[i].state), proc[i].image_path);
        }
    }

    for (u32 r = log_top + 1; r < log_top + 7; r++) fb_clear_row(r);
    row = log_top + 1;
    u32 shown = 0;
    u32 max_back = http_log_seq < HTTP_LOG_RING_SIZE ? http_log_seq : HTTP_LOG_RING_SIZE;
    for (u32 back = 0; back < max_back && shown < 6; back++) {
        u32 seq = http_log_seq - 1U - back;
        struct http_log_entry *e = &http_log_ring[seq % HTTP_LOG_RING_SIZE];
        if (e->seq != seq || !e->event)
            continue;
        if (!dash_contains(e->event, "error") &&
            !dash_contains(e->event, "fail") &&
            !dash_contains(e->event, "warn"))
            continue;
        fb_set_cursor(3, row++);
        fb_printf("%u t=%u %s a=%u b=%u", e->seq, e->tick_ms, e->event, e->a, e->b);
        shown++;
    }
    if (shown == 0) {
        fb_set_cursor(3, row);
        fb_set_color(0x0000FF80, 0x00000000);
        fb_puts("No warnings or errors in the hot log ring.");
    }
}

/* Core 0 I/O reactor: timer IRQ only marks work due and wakes the service
 * loop. Real I/O stays in thread context so IRQ latency remains bounded. */
static volatile u32 core0_io_flags;
static volatile u64 core0_io_wfi_count;
static volatile u64 core0_io_wake_count;
static volatile u64 core0_io_idle_ticks;
static volatile u64 core0_io_sched_start_ticks;
static volatile u32 core0_io_last_flags;

static inline u64 sched_counter_ticks(void)
{
    u64 cnt;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(cnt));
    return cnt;
}

static void core0_io_tick_hook(u32 core, u64 tick)
{
    if (core != CORE_NET)
        return;

    u32 flags = 0;
    if ((tick & 31U) == 0)
        flags |= CORE0_IO_NET | CORE0_IO_TCP | CORE0_IO_UART | CORE0_IO_USB;
    if ((tick % 100U) == 0)
        flags |= CORE0_IO_MAINT;
    if ((tick % 2000U) == 0)
        flags |= CORE0_IO_DASH;
    if (flags) {
        core0_io_flags |= flags;
        sev();
    }
}

static void core0_eth_irq_handler(void)
{
    if (core0_eth_irq_oneshot)
        gic_disable_irq(GIC_RP1_ETH_MSI);
    core0_eth_irq_last_mip = rp1_mip_host_status_l();
    core0_eth_irq_deferred_quench = true;
    core0_eth_irq_count++;
    core0_io_flags |= CORE0_IO_NET | CORE0_IO_TCP;
    sev();
}

static bool core0_eth_irq_drain_and_quench(bool host_route)
{
    const u32 eth_bit = 1U << RP1_INT_ETH;
    u32 passes = 0;
    bool clear = false;
    for (; passes < 8U; passes++) {
        net_poll();
        core0_eth_irq_last_macb_isr = macb_irq_ack_rx();
        if (!host_route) {
            core0_eth_irq_last_mip = rp1_eth_irq_ack();
            gic_clear_pending(GIC_RP1_ETH_MSI);
        }
        dsb();
        if ((rp1_irq_status_l() & eth_bit) == 0 &&
            (rp1_mip_host_status_l() & eth_bit) == 0) {
            clear = true;
            break;
        }
    }
    core0_eth_irq_quench_passes = passes < 8U ? passes + 1U : passes;
    return clear;
}

static u32 core0_io_take_flags(void)
{
    __asm__ volatile("msr daifset, #2" ::: "memory");
    dmb();
    u32 flags = core0_io_flags;
    core0_io_flags = 0;
    dmb();
    __asm__ volatile("msr daifclr, #2" ::: "memory");
    return flags;
}

static void core0_sched_snapshot(u64 *wake, u64 *wfi_count, u64 *idle_ticks,
                                 u64 *total_ticks, u32 *busy_permille,
                                 u32 *last_flags)
{
    u64 now = sched_counter_ticks();
    u64 start = core0_io_sched_start_ticks;
    u64 total = (start != 0 && now >= start) ? (now - start) : 0;
    u64 idle = core0_io_idle_ticks;
    if (idle > total)
        idle = total;
    u64 busy = total - idle;
    if (wake) *wake = core0_io_wake_count;
    if (wfi_count) *wfi_count = core0_io_wfi_count;
    if (idle_ticks) *idle_ticks = idle;
    if (total_ticks) *total_ticks = total;
    if (busy_permille) {
        while (total > 0x00FFFFFFFFFFFFFFULL) {
            total >>= 1;
            busy >>= 1;
        }
        *busy_permille = total ? (u32)((busy * 1000ULL) / total) : 0;
    }
    if (last_flags) *last_flags = core0_io_last_flags;
}

static bool ksvc_debug_poll(void *ctx)
{
    (void)ctx;
    debug_tcp_poll();
    return true;
}

static bool ksvc_dashboard_poll(void *ctx)
{
    (void)ctx;
    hdmi_dashboard_render();
    return true;
}

static bool ksvc_timer_poll(void *ctx)
{
    (void)ctx;
    arp_tick();
    tcp_tick();
    return true;
}

/* Core 0: Kernel services + network */
NORETURN void core0_main(void) {
    struct core_env *env = core_env_of(CORE_NET);
    ui_mode = UI_MODE_NONE;  /* HDMI stays on boot diags */
    ui_selected = 0;
    ui_last_render = 0;
    ui_launch_idx = -1;
    ui_status_code = 0;

    /* Clear the framebuffer once we hit the network poll loop so the
     * visual status strip (network heartbeat blocks) starts on a blank
     * canvas. Boot logs scrolled past, this gives us a clean diagnostic
     * surface for ICMP/heartbeat activity. */
    fb_clear(0x00000000);
    u32 stats_rows = fb_rows() / 2;
    if (stats_rows < 20) stats_rows = 20;
    fb_set_reserved_rows(stats_rows);
    fb_set_cursor(0, stats_rows);
    fb_set_color(0x0000FF80, 0x00000000);
    fb_puts("INIT COMPLETE\n");

    uart_puts("[sys] INIT COMPLETE\n");

    fb_set_color(0x00FFAA00, 0x00000000);
    fb_puts("[dma] late memcpy selftest...\n");
    uart_puts("[dma] late memcpy selftest...\n");
    if (dma_selftest()) {
        fb_puts("[dma] late memcpy selftest OK\n");
        uart_puts("[dma] late memcpy selftest OK\n");
    } else {
        fb_puts("[dma] late memcpy selftest FAILED (using NEON fallback)\n");
        uart_puts("[dma] late memcpy selftest FAILED (using NEON fallback)\n");
    }

    uart_vt_clear();
    uart_vt_home();
    uart_vt_color(UART_COLOR_CYAN, UART_COLOR_BLACK, true);
    ui_console_write("PIOS Serial Console is online\n");
    uart_vt_color(UART_COLOR_GREEN, UART_COLOR_BLACK, true);
    ui_console_write("Type Help for assistance!\n");
    uart_vt_reset();
    ui_console_prompt();

    timer_set_tick_hook(core0_io_tick_hook);
    core0_io_flags = CORE0_IO_NET | CORE0_IO_TCP | CORE0_IO_UART |
                     CORE0_IO_USB | CORE0_IO_MAINT | CORE0_IO_DASH;
    core0_io_sched_start_ticks = sched_counter_ticks();

    for (;;) {
        u32 flags = core0_io_take_flags();
        if (flags == 0) {
            core0_io_wfi_count++;
            u64 idle_start = sched_counter_ticks();
            wfi();
            u64 idle_end = sched_counter_ticks();
            if (idle_end >= idle_start)
                core0_io_idle_ticks += idle_end - idle_start;
            continue;
        }

        core0_io_wake_count++;
        core0_io_last_flags = flags;

        if (flags & CORE0_IO_NET) {
            u64 svc_start = ksvc_begin(ksvc_net_id);
            net_poll();
            dns_poll();
            if (core0_eth_irq_deferred_quench) {
                core0_eth_irq_drain_and_quench(false);
                core0_eth_irq_deferred_quench = false;
            }
            ksvc_end(ksvc_net_id, svc_start, false);
        }

        if (flags & CORE0_IO_TCP) {
            u64 svc_start = ksvc_begin(ksvc_tcp_id);
            echo_tcp_poll();
            ksvc_end(ksvc_tcp_id, svc_start, false);
            ksvc_run(ksvc_debug_id);
        }

        if (flags & (CORE0_IO_UART | CORE0_IO_USB)) {
            u64 svc_start = ksvc_begin(ksvc_ui_id);
            for (u32 i = 0; i < 16; i++) {
                i32 rx = uart_try_getc();
                if (rx < 0)
                    break;
                ui_console_feed_char(rx);
            }

            ui_handle_keys();
            ksvc_end(ksvc_ui_id, svc_start, false);
        }

        if (flags & CORE0_IO_MAINT) {
            ksvc_run(ksvc_timer_id);
        }

        if ((flags & CORE0_IO_DASH) && ui_mode == UI_MODE_NONE)
            ksvc_run(ksvc_dashboard_id);

        watchdog_hw_pet();
        env->poll_count++;
    }
}

/* Disk service loop helper (service core is CORE_DISK, currently core 0) */
static void disk_handle_request(u32 from_core) {
    struct fifo_msg msg;
    struct fifo_msg reply;
    /* Per-core block buffer in core 1's private RAM */
    static u8 block_buf[SD_BLOCK_SIZE] ALIGNED(64);

    if (!fifo_pop(CORE_DISK, from_core, &msg))
        return;

    reply.tag = msg.tag;
    reply.buffer = msg.buffer;
    reply.length = SD_BLOCK_SIZE;

    switch (msg.type) {
    case MSG_DISK_READ:
        if (sd_read_block(msg.param, block_buf)) {
            simd_memcpy((void *)(usize)msg.buffer, block_buf, SD_BLOCK_SIZE);
            reply.type   = MSG_DISK_DONE;
            reply.status = 0;
        } else {
            reply.type   = MSG_DISK_ERROR;
            reply.status = 1;
        }
        fifo_push(CORE_DISK, from_core, &reply);
        break;

    case MSG_DISK_WRITE:
        simd_memcpy(block_buf, (void *)(usize)msg.buffer, SD_BLOCK_SIZE);
        if (sd_write_block(msg.param, block_buf)) {
            reply.type   = MSG_DISK_DONE;
            reply.status = 0;
        } else {
            reply.type   = MSG_DISK_ERROR;
            reply.status = 1;
        }
        fifo_push(CORE_DISK, from_core, &reply);
        break;

    default:
        break;
    }
}

NORETURN void core1_main(void) {
    core_mark_online(CORE_USERM, 1);
    core_env_init(CORE_USERM);
    core_mark_online(CORE_USERM, 2);
    core_mark_online(CORE_USERM, 0x200U + core_id());
    proc_init();
    core_mark_online(CORE_USERM, 3);
    timer_init(PROC_PREEMPT_TIMER_HZ);
    core_mark_online(CORE_USERM, 4);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    core_mark_online(CORE_USERM, 5);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* Core 2: User core 0 - process scheduler */
NORETURN void core2_main(void) {
    core_mark_online(CORE_USER0, 1);
    core_env_init(CORE_USER0);
    core_mark_online(CORE_USER0, 2);
    core_mark_online(CORE_USER0, 0x200U + core_id());
    proc_init();
    core_mark_online(CORE_USER0, 3);
    timer_init(PROC_PREEMPT_TIMER_HZ);
    core_mark_online(CORE_USER0, 4);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    core_mark_online(CORE_USER0, 5);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* Core 3: User core 1 - process scheduler */
NORETURN void core3_main(void) {
    core_mark_online(CORE_USER1, 1);
    core_env_init(CORE_USER1);
    core_mark_online(CORE_USER1, 2);
    core_mark_online(CORE_USER1, 0x200U + core_id());
    proc_init();
    core_mark_online(CORE_USER1, 3);
    timer_init(PROC_PREEMPT_TIMER_HZ);
    core_mark_online(CORE_USER1, 4);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    core_mark_online(CORE_USER1, 5);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* ---- Boot diagnostics display ---- */

static void print_ip(u32 ip) {
    fb_printf("%d.%d.%d.%d",
        (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
        (ip >> 8) & 0xFF, ip & 0xFF);
}

static void boot_diag(bool sd_ok, bool walfs_ok, bool nic_ok, bool usb_ok) {
    /* Phase 7: Purple — PIOS operational.  Pink text on purple background. */
    fb_clear(BOOT_PURPLE);
    fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);

    fb_puts("PIOS ");
    fb_puts(PIOS_BUILD_LABEL);
    fb_puts(" - Pi 5 Bare Metal Microkernel\n");
    fb_set_color(BOOT_FG_WHITE, BOOT_PURPLE);
    fb_printf("========================================\n\n");

    fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);

    fb_printf("Core 0: Kernel/Net  [16MB @ 0x%x]\n", CORE0_RAM_BASE);
    fb_printf("Core 1: User        [16MB @ 0x%x]\n", CORE1_RAM_BASE);
    fb_printf("Core 2: User        [16MB @ 0x%x]\n", CORE2_RAM_BASE);
    fb_printf("Core 3: User        [16MB @ 0x%x]\n\n", CORE3_RAM_BASE);

    /* Network */
    if (nic_ok) {
        u8 mac[6];
        nic_get_mac(mac);
        fb_printf("NET:  Cadence MACB/GEM\n");
        fb_printf("  IP:   ");
        print_ip(MY_IP);
        fb_printf(" / ");
        print_ip(MY_MASK);
        fb_printf("\n  GW:   ");
        print_ip(MY_GW);
        fb_printf("\n  PHY:  %s\n", nic_link_up() ? "Link UP" : "Link DOWN");
        fb_printf("  MAC:  %x:%x:%x:%x:%x:%x\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        fb_set_color(BOOT_FG_WHITE, BOOT_PURPLE);
        fb_printf("NET:  MACB/GEM init FAILED\n");
        fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);
    }

    /* SD / Filesystem */
    if (sd_ok) {
        const sd_card_t *sd = sd_get_card_info();
        fb_printf("DISK: %s raw block  WALFS: %s\n",
            sd->type == 2 ? "SDHC/SDXC" : "SDSC",
            walfs_ok ? "OK" : "FAILED");
    } else {
        fb_set_color(BOOT_FG_WHITE, BOOT_PURPLE);
        fb_printf("DISK: NOT DETECTED\n");
        fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);
    }

    fb_printf("USB:  %s\n", usb_ok ? "xHCI online" : "not available");
    fb_printf("FIFO: 12ch SPSC  depth=%u  msg=%u bytes\n",
              FIFO_CAPACITY, FIFO_MSG_SIZE);
    fb_printf("SIMD: NEON memcpy/zero/checksum + CRC32\n\n");

    fb_set_color(BOOT_FG_WHITE, BOOT_PURPLE);
    fb_printf("System ready.\n\n");
    fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);
}

/*
 * ── CPU State Debugger ──
 * Ultra-simple: read one register, print it, repeat.
 * No macros, no helper functions, no optimisation surprises.
 */
void kernel_fb_early(void) {
    if (!fb_init(1920, 1080) && !fb_init(1280, 720) && !fb_init(1024, 768))
        return;

    u64 val;

    fb_set_color(0x0000FF00, 0x00000000);
    fb_puts("PIOS ");
    fb_puts(PIOS_BUILD_LABEL);
    fb_putc('\n');
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("FB addr = 0x%X\n", fb_get_phys_addr());
    fb_puts("Entering kernel_main...\n");
}

/*
 * ── EL2 Crash Handler (PiSOD) ──
 * Called from start.S el2_crash_vectors with:
 *   x0 = ESR_EL2, x1 = ELR_EL2, x2 = FAR_EL2, x3 = SPSR_EL2
 * Framebuffer is already initialised by kernel_fb_early().
 */
void kernel_el2_crash(u64 esr, u64 elr, u64 far, u64 spsr) {
    /* Purple screen, pink text — PiSOD */
    fb_clear(BOOT_PURPLE);
    fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);
    fb_puts("!! PIOS CRASH — EL2 EXCEPTION !!\n\n");

    fb_set_color(0x00FFFFFF, BOOT_PURPLE);
    fb_printf("ESR_EL2    = 0x%X\n", esr);
    fb_printf("ELR_EL2    = 0x%X\n", elr);
    fb_printf("FAR_EL2    = 0x%X\n", far);
    fb_printf("SPSR_EL2   = 0x%X\n", spsr);

    /* Decode ESR exception class */
    u32 ec = (u32)(esr >> 26) & 0x3F;
    fb_putc('\n');
    fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);
    fb_printf("EC = 0x%x  ", ec);
    switch (ec) {
    case 0x00: fb_puts("(Unknown reason)"); break;
    case 0x01: fb_puts("(Trapped WFI/WFE)"); break;
    case 0x07: fb_puts("(Trapped SIMD/FP)"); break;
    case 0x0E: fb_puts("(Illegal execution)"); break;
    case 0x15: fb_puts("(SVC from AArch64)"); break;
    case 0x16: fb_puts("(HVC from AArch64)"); break;
    case 0x17: fb_puts("(SMC from AArch64)"); break;
    case 0x18: fb_puts("(Trapped MSR/MRS)"); break;
    case 0x20: fb_puts("(Inst abort, lower EL)"); break;
    case 0x21: fb_puts("(Inst abort, same EL)"); break;
    case 0x22: fb_puts("(PC alignment)"); break;
    case 0x24: fb_puts("(Data abort, lower EL)"); break;
    case 0x25: fb_puts("(Data abort, same EL)"); break;
    case 0x26: fb_puts("(SP alignment)"); break;
    default:   fb_puts("(Other)"); break;
    }
    fb_putc('\n');

    /* Extra context */
    u64 val;
    fb_putc('\n');
    fb_set_color(0x00AAAAAA, BOOT_PURPLE);
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(val));
    fb_printf("CurrentEL  = 0x%X\n", val);
    __asm__ volatile("mov %0, sp" : "=r"(val));
    fb_printf("SP         = 0x%X\n", val);
    __asm__ volatile("mrs %0, SCTLR_EL2" : "=r"(val));
    fb_printf("SCTLR_EL2  = 0x%X\n", val);
    __asm__ volatile("mrs %0, HCR_EL2" : "=r"(val));
    fb_printf("HCR_EL2    = 0x%X\n", val);

    fb_putc('\n');
    fb_set_color(BOOT_FG_PINK, BOOT_PURPLE);
    fb_puts("System halted. Power cycle to reboot.\n");

    for (;;) __asm__ volatile("wfe");
}

/* ---- Main kernel entry ---- */

/* Draw register panel on the right side of screen (col 65+) */
static void reg_panel(u32 at_el1) {
    u32 col = 65;
    u32 row = 1;
    u64 val;

    uart_puts("[reg] CPU Regs\n");
    uart_puts("[reg] EL=");
    uart_hex(at_el1 ? 1 : 2);
    uart_puts(at_el1 ? " (kernel)\n" : " (hypervisor)\n");

    fb_set_cursor(col, row++);
    fb_set_color(0x00FF9900, 0x00000000);
    fb_puts("CPU Registers");

    fb_set_cursor(col, row++);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("EL = %d", at_el1 ? 1 : 2);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts(at_el1 ? " (kernel)" : " (hypervisor)");

    /* PC */
    __asm__ volatile("adr %0, ." : "=r"(val));
    fb_set_cursor(col, row++);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("PC     %X", val);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts(" Program ctr");
    uart_puts("[reg] PC=");
    uart_hex(val);
    uart_puts("\n");

    /* SP */
    __asm__ volatile("mov %0, sp" : "=r"(val));
    fb_set_cursor(col, row++);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("SP     %X", val);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts(" Stack ptr");
    uart_puts("[reg] SP=");
    uart_hex(val);
    uart_puts("\n");

    /* MPIDR */
    __asm__ volatile("mrs %0, MPIDR_EL1" : "=r"(val));
    fb_set_cursor(col, row++);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("MPIDR  %X", val);
    fb_set_cursor(col, row++);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_printf(" Core %u / Cluster %u",
        (u32)(val & 0xFF), (u32)((val >> 8) & 0xFF));
    uart_puts("[reg] MPIDR=");
    uart_hex(val);
    uart_puts(" c=");
    uart_hex((u32)(val & 0xFF));
    uart_puts(" cl=");
    uart_hex((u32)((val >> 8) & 0xFF));
    uart_puts("\n");

    if (at_el1) {
        /* ── EL1 registers ── */
        __asm__ volatile("mrs %0, SCTLR_EL1" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("SCTLR1 %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf(" MMU=%u DC=%u IC=%u",
            (u32)(val & 1), (u32)((val >> 2) & 1), (u32)((val >> 12) & 1));
        uart_puts("[reg] SCTLR1=");
        uart_hex(val);
        uart_puts(" MMU=");
        uart_hex((u32)(val & 1));
        uart_puts(" DC=");
        uart_hex((u32)((val >> 2) & 1));
        uart_puts(" IC=");
        uart_hex((u32)((val >> 12) & 1));
        uart_puts("\n");

        __asm__ volatile("mrs %0, CPACR_EL1" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("CPACR1 %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf(" NEON=%s", ((val >> 20) & 3) == 3 ? "enabled" : "TRAPPED");
        uart_puts("[reg] CPACR1=");
        uart_hex(val);
        uart_puts(" NEON=");
        uart_puts(((val >> 20) & 3) == 3 ? "enabled\n" : "TRAPPED\n");

        __asm__ volatile("mrs %0, TTBR0_EL1" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("TTBR0  %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_puts(val ? " Page table base" : " (none)");
        uart_puts("[reg] TTBR0=");
        uart_hex(val);
        uart_puts(val ? " pgtbl\n" : " none\n");

        __asm__ volatile("mrs %0, VBAR_EL1" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("VBAR1  %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_puts(val ? " Exception vectors" : " (none!)");
        uart_puts("[reg] VBAR1=");
        uart_hex(val);
        uart_puts(val ? " vec\n" : " none!\n");
    } else {
        /* ── EL2 registers ── */
        __asm__ volatile("mrs %0, SCTLR_EL2" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("SCTLR2 %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf(" MMU=%u DC=%u IC=%u",
            (u32)(val & 1), (u32)((val >> 2) & 1), (u32)((val >> 12) & 1));
        uart_puts("[reg] SCTLR2=");
        uart_hex(val);
        uart_puts(" MMU=");
        uart_hex((u32)(val & 1));
        uart_puts(" DC=");
        uart_hex((u32)((val >> 2) & 1));
        uart_puts(" IC=");
        uart_hex((u32)((val >> 12) & 1));
        uart_puts("\n");

        __asm__ volatile("mrs %0, HCR_EL2" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("HCR_EL2 %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf(" RW=%u VM=%u", (u32)((val >> 31) & 1), (u32)(val & 1));
        uart_puts("[reg] HCR2=");
        uart_hex(val);
        uart_puts(" RW=");
        uart_hex((u32)((val >> 31) & 1));
        uart_puts(" VM=");
        uart_hex((u32)(val & 1));
        uart_puts("\n");

        __asm__ volatile("mrs %0, CPTR_EL2" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("CPTR2  %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf(" NEON=%s", ((val >> 10) & 1) ? "TRAPPED" : "enabled");
        uart_puts("[reg] CPTR2=");
        uart_hex(val);
        uart_puts(" NEON=");
        uart_puts(((val >> 10) & 1) ? "TRAPPED\n" : "enabled\n");

        __asm__ volatile("mrs %0, VBAR_EL2" : "=r"(val));
        fb_set_cursor(col, row++);
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_printf("VBAR2  %X", val);
        fb_set_cursor(col, row++);
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_puts(val ? " EL2 vectors" : " (none)");
        uart_puts("[reg] VBAR2=");
        uart_hex(val);
        uart_puts(val ? " EL2 vec\n" : " none\n");
    }

    fb_set_cursor(col, row++);
    fb_set_color(0x00444444, 0x00000000);
    fb_puts("---------------------");
    uart_puts("[reg] ---\n");
}

#ifdef PIOS_ONEOFF_PROVISION
extern const u8 provision_payload_start[];
extern const u8 provision_payload_end[];

static u32 provision_read_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u32 provision_slot_lba(void)
{
    static u8 mbr[SD_BLOCK_SIZE] ALIGNED(64);
    if (!sd_read_block(0, mbr))
        return 2048;
    if (mbr[510] != 0x55 || mbr[511] != 0xAA)
        return 2048;
    u32 p2_start = provision_read_le32(&mbr[0x1CE + 8]);
    u32 p2_size = provision_read_le32(&mbr[0x1CE + 12]);
    if (p2_start == 0 || p2_size < PIOS_RESERVED_LBAS)
        return 2048;
    return p2_start;
}

static bool provision_write_payload_to_slot(void)
{
    static u8 block[SD_BLOCK_SIZE] ALIGNED(64);
    const u8 *payload = provision_payload_start;
    u32 len = (u32)(provision_payload_end - provision_payload_start);
    if (len == 0 || len > PIOS_STAGE2_ZONE_BYTES)
        return false;

    u32 lba = provision_slot_lba();
    uart_puts("[prov] oneoff slot LBA=");
    uart_hex(lba);
    uart_puts(" bytes=");
    uart_hex(len);
    uart_puts("\n");

    pios_fill_reserved_header(block, HOTPATCH_SLOT_MAGIC, len);
    if (!sd_write_block(lba, block))
        return false;

    for (u32 b = 0; b < (PIOS_STAGE2_ZONE_BYTES + SD_BLOCK_SIZE - 1U) / SD_BLOCK_SIZE; b++) {
        u32 off = b * SD_BLOCK_SIZE;
        u32 dst_lba = lba + (PIOS_STAGE2_OFFSET / SD_BLOCK_SIZE) + b;
        if (off + SD_BLOCK_SIZE <= len) {
            if (!sd_write_block(dst_lba, payload + off))
                return false;
        } else {
            simd_zero(block, sizeof(block));
            if (off < len)
                simd_memcpy(block, payload + off, len - off);
            if (!sd_write_block(dst_lba, block))
                return false;
        }
    }
    pios_bootctrl_init_good_at(lba, PIOS_BOOTCTRL_SLOT_A);
    return true;
}
#endif

void kernel_main(void) {
    bool usb_ok = false;
    bool fb_ok = true;  /* fb already init'd by kernel_fb_early */
    bool sd_ok = false;
    bool walfs_ok = false;
    bool nic_ok = false;

    watchdog_hw_arm_seconds(15);

    /* Stack, NEON, VBAR already set by start.S .Lel1_entry */

    /* Detect current EL */
    u64 cur_el;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(cur_el));
    cur_el = (cur_el >> 2) & 3;
    bool at_el1 = (cur_el == 1);

    /* ── Progress display (left side) + Register panel (right side) ── */
    bp_init();
    reg_panel(at_el1);
    bp_done(0, true);                     /* Firmware handoff */
    bp_done(1, true);                     /* VideoCore (fb from EL2) */

    /* Show FB physical address — needed for MMU mapping */
    {
        u64 fb_addr = fb_get_phys_addr();
        fb_set_cursor(1, bp_log_y);
        fb_set_color(0x0000CCFF, BOOT_BLACK);
        fb_printf("FB phys=0x%X  size=%u", fb_addr, fb_addr ? 1024*768*4 : 0);
        bp_log_y++;
    }

    if (at_el1) {
        bp_ok("[el2->el1] Transition OK!");
    } else {
        bp_warn("[boot] Running at EL2 (eret failed or skipped)");
    }

    /* ── EL1 inits (MMU already on from start.S) ── */
    if (at_el1) {
        /* Update shared vars for secondary cores */
        shared_ttbr0 = (u64)(usize)l1_table;
        shared_mair  = 0xBBFF4400UL;
        shared_tcr   = 0x200803519UL;
        bp_ok("[mmu] ON (start.S asm)");

        bp_log("[exc] exception_init...");
        exception_init();
        bp_ok("[exc] vectors installed");

        bp_log("[gic] gic_init...");
        gic_init();
        bp_ok("[gic] distributor + CPU iface ready");

        bp_log("[timer] timer_init(1000Hz)...");
        timer_init(1000);
        bp_ok("[timer] 1kHz tick running");

        bp_log("[wdog] watchdog_init(5s)...");
        watchdog_init(5000, false);
        bp_ok("[wdog] armed");

        bp_log("[irq] unmasking IRQs...");
        __asm__ volatile("msr daifclr, #2");
        bp_ok("[irq] IRQs live");

        reg_panel(at_el1);
    } else {
        bp_warn("[skip] all EL1 inits");
    }

    bp_log("[dma] dma_init...");
    dma_init();
    bp_ok("[dma] 6-channel engine ready");
    bp_done(1, true);
    watchdog_hw_pet();

    /* ── Phase 2+3: PCIe + RP1 + USB ── */
    bp_active(2);
    bp_log("[pcie] pcie_init...");
    if (pcie_init()) {
        bp_log("[pcie] RC online, calling rp1_init...");
        if (rp1_init()) {
            bp_ok("[pcie] RP1 BAR mapped OK");
            bp_done(2, true);
            bp_active(3);
            bp_log("[rp1] rp1_clk_init...");
            rp1_clk_init();
            bp_log("[rp1] rp1_gpio_init...");
            rp1_gpio_init();
            bp_log("[rp1] activity LED init...");
            ui_act_led_init();
            bp_log("[uart] uart_init (RP1 PL011)...");
            uart_init();
            uart_puts("\n[uart] RP1 UART online\n");
            bp_ok("[uart] RP1 UART online");
            bp_log("[usb] registering storage+kbd...");
            usb_storage_register();
            usb_kbd_register();
            bp_log("[usb] usb_init (xHCI)...");
            usb_ok = usb_init();
            if (usb_ok) bp_ok("[usb] xHCI online");
            else bp_err("[usb] xHCI FAILED");
            bp_done(3, true);
        } else {
            bp_err("[rp1] RP1 init FAILED"); bp_done(2, false); bp_done(3, false);
        }
    } else {
        bp_err("[pcie] PCIe init FAILED"); bp_done(2, false); bp_done(3, false);
    }
    watchdog_hw_pet();

    /* IPC */
    bp_log("[fifo] fifo_init_all...");
    fifo_init_all();
    bp_log("[ipc] ipc_queue_init...");
    ipc_queue_init();
    bp_log("[ipc] ipc_stream_init...");
    ipc_stream_init();
    ipc_proc_init();
    bp_log("[ipc] pipe_init...");
    pipe_init();
    bp_ok("[ipc] all channels ready");

    /* ── Phase 4: Filesystem ── */
    bp_active(4);
    bp_log("[sd] sd_init (EMMC2)...");
    sd_ok = sd_init();
    if (!sd_ok) {
        bp_err("[sd] SD init FAILED"); bp_done(4, false);
    } else {
        bp_ok("[sd] card detected OK");
#ifdef PIOS_ONEOFF_PROVISION
        bp_log("[prov] writing embedded second-stage slot...");
        if (provision_write_payload_to_slot())
            bp_ok("[prov] second-stage slot written");
        else
            bp_warn("[prov] second-stage slot write FAILED");
#endif
        bp_log("[cache] bcache_init...");
        bcache_init();
        watchdog_hw_pet();
        bp_log("[walfs] walfs_init...");
        walfs_ok = walfs_init();
        watchdog_hw_pet();
        if (walfs_ok) {
            bp_log("[walfs] walfs_verify...");
            struct walfs_health wh;
            if (!walfs_verify(&wh)) {
                /* Don't PiSOD — a corrupted superblock (e.g. from a
                 * watchdog reset that interrupted a write) used to brick
                 * the box because every reboot PiSODed before HTTP came
                 * up. Just log and continue; operator can issue
                 * `walfs format confirm` over the console to recover. */
                bp_warn("[walfs] verify FAILED — continuing (use 'walfs format confirm' to recover)");
                uart_puts("[walfs] verify FAILED super_ok=");
                uart_hex(wh.super_ok ? 1U : 0U);
                uart_puts(" wal_head_ok=");
                uart_hex(wh.wal_head_ok ? 1U : 0U);
                uart_puts(" crc_err=");
                uart_hex(wh.crc_errors);
                uart_puts(" hdr_err=");
                uart_hex(wh.header_errors);
                uart_puts(" scan_end=");
                uart_hex((u32)wh.scan_end);
                uart_puts("\n");
            }
            watchdog_hw_pet();
            bp_log("[walfs] principal_init...");
            principal_init();
            bp_log("[walfs] picowal_db_init...");
            if (!picowal_db_init()) {
                bp_warn("[walfs] picowal_db init FAILED — continuing");
            }
            watchdog_hw_pet();
            bp_log("[walfs] boot_policy_verify...");
            boot_policy_verify_or_seed();
            bp_log("[walfs] boot_measurements...");
            u64 el1_s = 0;
            u32 el1_len = 0;
            boot_measurements(NULL, NULL, &el1_s, &el1_len);
            /* Skip boot integrity arming during development —
             * the EL2 HVC call fails when kernel image changes
             * because the stored hash no longer matches. */
            uart_puts("[bt] skip EL2 integ arm (dev)\n");
            bp_warn("[walfs] integrity check skipped (dev)");
        } else {
            bp_err("[walfs] WALFS init FAILED");
        }
        if (walfs_ok) bp_ok("[fs] SD + WALFS online");
        else bp_warn("[fs] SD ok, WALFS failed");
        bp_log("[key] keystore_init...");
        if (keystore_init()) {
            bp_ok("[key] sealed root ready");
            bp_log("[x509] x509_init...");
            x509_init();
            if (x509_generate_dev_cert("PIOS kernel dev") && x509_bind_tls())
                bp_ok("[x509] dev certificate descriptor ready");
            else
                bp_warn("[x509] dev certificate descriptor unavailable");
            bp_log("[acme] acme_init...");
            if (acme_init())
                bp_ok("[acme] account state ready");
            else
                bp_warn("[acme] account state unavailable");
        } else {
            bp_warn("[key] keystore unavailable");
            x509_init();
            acme_init();
        }
        bp_done(4, sd_ok);
    }
    watchdog_hw_pet();

    /* ── Phase 5: NIC (Cadence MACB/GEM on RP1) ── */
    bp_active(5);
    bp_log("[nic] nic_init (Cadence GEM)...");
    nic_ok = nic_init();
    if (!nic_ok) {
        bp_err("[nic] MACB init FAILED"); bp_done(5, false);
    } else {
        bp_ok("[nic] MACB online");
        if (nic_link_up()) bp_ok("[nic] PHY link UP");
        else bp_warn("[nic] PHY link DOWN (use 'wifi init' for wireless)");
        bp_done(5, true);
    }

    /* Init network stack */
    bp_log("[net] net_init (static IP)...");
    net_init(MY_IP, MY_GW, MY_MASK, MY_GW_MAC);
    /* Pin the dev host PC as a static neighbor so we never need ARP
     * resolution to talk back to it, and unsolicited replies are valid. */
    net_add_neighbor(HOST_PC_IP, HOST_PC_MAC);
    (void)net_route_add(HOST_PC_IP, 0xFFFFFFFFU, 0, NET_ROUTE_F_CONNECTED);
    uart_puts("[net] static neighbor 192.168.218.9 -> 04:bf:1b:e1:d7:78\n");
    ui_cfg_ip = MY_IP;
    ui_cfg_mask = MY_MASK;
    ui_cfg_gw = MY_GW;
    ui_cfg_dns = MY_GW;
    ui_cfg_dhcp = false;
    dns_init(ui_cfg_dns);
    net_services_listen();
    uart_puts("[net] HTTP:80 HTTPS:443 DBG:2323\n");
    bp_ok("[net] IP stack ready");
    watchdog_hw_pet();

    /* GPU + Tensor */
    bp_log("[gpu] tensor_init...");
    tensor_init();
    bp_ok("[gpu] tensor compute ready");

    /* Core 0 environment */
    bp_log("[core] core_env_init...");
    core_env_init(CORE_NET);
    bp_log("[core] ksem_init_core...");
    ksem_init_core();
    bp_log("[core] workq_init_core...");
    workq_init_core();
    bp_log("[core] ksvc_init_core...");
    ksvc_init_core();
    ksvc_net_id = ksvc_register("net-poll", KSVC_KIND_POLL | KSVC_KIND_NETWORK, CORE_NET, 100);
    ksvc_tcp_id = ksvc_register("tcp-http-tls", KSVC_KIND_POLL | KSVC_KIND_NETWORK | KSVC_KIND_TLS, CORE_NET, 90);
    ksvc_debug_id = ksvc_register_poll("debug-console", KSVC_KIND_POLL | KSVC_KIND_CONSOLE, CORE_NET, 50, ksvc_debug_poll, NULL);
    ksvc_ui_id = ksvc_register("ui-input", KSVC_KIND_POLL | KSVC_KIND_UI, CORE_NET, 40);
    ksvc_dashboard_id = ksvc_register_poll("dashboard", KSVC_KIND_POLL | KSVC_KIND_UI, CORE_NET, 20, ksvc_dashboard_poll, NULL);
    ksvc_timer_id = ksvc_register_poll("tcp-timers", KSVC_KIND_POLL | KSVC_KIND_TIMER | KSVC_KIND_NETWORK, CORE_NET, 80, ksvc_timer_poll, NULL);
    bp_log("[core] module_init...");
    module_init();
    bp_ok("[core] env ready");

    /* Setup */
    if (walfs_ok) {
        bp_log("[setup] setup_run...");
        setup_run(fb_ok, nic_ok, usb_ok);
        bp_ok("[setup] done");
    }
    watchdog_hw_pet();

    /* ── Phase 6: Multicore ── */
    bp_active(6);
    if (at_el1) {
        bp_log("[smp] core_start_all (PSCI)...");
        core_start_all();
        bp_ok("[smp] cores 0-3 active");
        bp_done(6, true);
    } else {
        bp_warn("[skip] multicore (need EL1)");
        bp_done(6, false);
    }

    /* ── Phase 7: Ready — show system info on HDMI ── */
    bp_active(7);
    bp_done(7, true);
    bp_ok("[pios] System ready — serial console active");
    bp_spin_color = 0x00FF88CC;  /* purple/pink — OS running */

    /* System summary in the log area */
    {
        u32 max_rows = 768 / 8;
        if (bp_log_y + 6 < max_rows) {
            fb_set_cursor(1, bp_log_y++);
            fb_set_color(0x00444444, BOOT_BLACK);
            fb_puts("---------------------------------------------");

            u8 mac[6];
            nic_get_mac(mac);
            fb_set_cursor(1, bp_log_y++);
            fb_set_color(0x0000CCFF, BOOT_BLACK);
            fb_printf("MAC  %x:%x:%x:%x:%x:%x",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            fb_set_cursor(1, bp_log_y++);
            fb_printf("IP   %d.%d.%d.%d / %d.%d.%d.%d",
                (ui_cfg_ip >> 24) & 0xFF, (ui_cfg_ip >> 16) & 0xFF,
                (ui_cfg_ip >> 8) & 0xFF, ui_cfg_ip & 0xFF,
                (ui_cfg_mask >> 24) & 0xFF, (ui_cfg_mask >> 16) & 0xFF,
                (ui_cfg_mask >> 8) & 0xFF, ui_cfg_mask & 0xFF);

            fb_set_cursor(1, bp_log_y++);
            fb_printf("GW   %d.%d.%d.%d  DNS %d.%d.%d.%d",
                (ui_cfg_gw >> 24) & 0xFF, (ui_cfg_gw >> 16) & 0xFF,
                (ui_cfg_gw >> 8) & 0xFF, ui_cfg_gw & 0xFF,
                (ui_cfg_dns >> 24) & 0xFF, (ui_cfg_dns >> 16) & 0xFF,
                (ui_cfg_dns >> 8) & 0xFF, ui_cfg_dns & 0xFF);

            fb_set_cursor(1, bp_log_y++);
            fb_printf("PHY  %s  USB %s  SD %s",
                nic_ok ? (nic_link_up() ? "UP" : "DOWN") : "FAIL",
                usb_ok ? "OK" : "FAIL",
                sd_ok ? "OK" : "FAIL");

            fb_set_cursor(1, bp_log_y++);
            fb_set_color(BOOT_FG_PINK, BOOT_BLACK);
            fb_puts("Serial console active on RP1 UART0");
        }
    }

    /* HDMI stays on boot diags */

    if (at_el1) {
        core0_main();
    } else {
        for (;;) __asm__ volatile("wfe");
    }
}
