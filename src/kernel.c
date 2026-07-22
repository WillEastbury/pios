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
#include "pioscap.h"
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
#include "v3d.h"
#include "videocore.h"
#include "vc_display.h"
#include "walfs.h"
#include "bcache.h"
#include "principal.h"
#include "proc.h"
#include "dtrace.h"
#include "coredump.h"
#include "virtio_net.h"
#include "uhttp_bridge.h"
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
#include "lease.h"
#include "pipe.h"
#include "setup.h"
#include "ksem.h"
#include "workq.h"
#include "ksvc.h"
#include "picowal_db.h"
#include "el2.h"
#include "crypto.h"
#include "watchdog.h"
#include "stackprot.h"
#include "stack_canary.h"
#include "fat32.h"
#include "pios_addr.h"
#include "picoscript.h"
#include "mailbox.h"
#include "highmem.h"
#include "picovm.h"
#include "pixe_request.h"
#include "pixe_host.h"
#include "ide_assets.h"
#include "pico_hooks.h"
#include "keystore.h"
#include "tls.h"
#include "brotli.h"
#include "picocompress.h"
#include "picoweb.h"
#include "x509.h"
#include "acme.h"
#include "abi.h"
#include "mmio.h"
#include "capsule_store.h"
#include "platform.h"
#include "random.h"
#include "sts.h"
#include "sts_token.h"

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

#define MY_IP       IP4(192, 168, 0, 201)
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
#define ADMIN_HTTP_REQ_MAX 24576
#define ADMIN_HTTP_RESP_MAX 4096
#define ADMIN_SERVICE_WATCHDOG_MS 180000ULL
#define ADMIN_CLIENT_STALL_MS 10000ULL
#define HTTP_TX_CHUNK_MAX TCP_MSS

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
#define HTTP_ROUTE_PICOSCRIPT 11U
#define HTTP_ROUTE_FAVICON    12U
#define HTTP_ROUTE_STATIC     13U
#define HTTP_ROUTE_STATIC_PUT 14U
#define HTTP_ROUTE_CAPSULE    15U
#define HTTP_ROUTE_PCAP       16U

#define HTTP_ERR_NONE         0U
#define HTTP_ERR_RESP_TIMEOUT 1U
#define HTTP_ERR_REQ_TIMEOUT  2U
#define HTTP_ERR_BAD_STATE    3U
#define HTTP_ERR_HEADER_BIG   4U

static const char *http_event_name(u32 event)
{
    switch (event) {
    case HTTP_EVT_LISTEN: return "listen";
    case HTTP_EVT_ACCEPT: return "accept";
    case HTTP_EVT_RX: return "rx";
    case HTTP_EVT_COMPLETE: return "complete";
    case HTTP_EVT_BUILD_ENTER: return "build-enter";
    case HTTP_EVT_BUILD_EXIT: return "build-exit";
    case HTTP_EVT_TX: return "tx";
    case HTTP_EVT_CLOSE: return "close";
    case HTTP_EVT_ABORT: return "abort";
    case HTTP_EVT_RESET: return "reset";
    case HTTP_EVT_BAD_STATE: return "bad-state";
    case HTTP_EVT_STATUS_ENTER: return "status-enter";
    case HTTP_EVT_STATUS_EXIT: return "status-exit";
    case HTTP_EVT_ADMIN_BUILD: return "admin-build";
    default: return "?";
    }
}

static const char *http_route_name(u32 route)
{
    switch (route) {
    case HTTP_ROUTE_UNKNOWN: return "unknown";
    case HTTP_ROUTE_ROOT: return "root";
    case HTTP_ROUTE_STATUS: return "status";
    case HTTP_ROUTE_REBOOT: return "reboot";
    case HTTP_ROUTE_LOGS: return "logs";
    case HTTP_ROUTE_UPDATE: return "update";
    case HTTP_ROUTE_HOTPATCH: return "hotpatch";
    case HTTP_ROUTE_PLACEHOLDER: return "placeholder";
    case HTTP_ROUTE_NOT_FOUND: return "not-found";
    case HTTP_ROUTE_NETSTAT: return "netstat";
    case HTTP_ROUTE_ACME: return "acme";
    case HTTP_ROUTE_PICOSCRIPT: return "picoscript";
    case HTTP_ROUTE_FAVICON: return "favicon";
    case HTTP_ROUTE_STATIC: return "static";
    case HTTP_ROUTE_STATIC_PUT: return "static-put";
    case HTTP_ROUTE_CAPSULE: return "capsule";
    case HTTP_ROUTE_PCAP: return "pcap";
    default: return "?";
    }
}

static const char *http_error_name(u32 err)
{
    switch (err) {
    case HTTP_ERR_NONE: return "none";
    case HTTP_ERR_RESP_TIMEOUT: return "resp-timeout";
    case HTTP_ERR_REQ_TIMEOUT: return "req-timeout";
    case HTTP_ERR_BAD_STATE: return "bad-state";
    case HTTP_ERR_HEADER_BIG: return "header-too-big";
    default: return "?";
    }
}

static tcp_conn_t echo_listen_conn = -1;
static tcp_conn_t echo_client_conn = -1;
static tcp_conn_t http_listen_conn = -1;
static tcp_conn_t https_tls_listen_conn = -1;
static tcp_conn_t https_tls_tcp_conn = -1;
static tls_conn_t https_tls_conn = -1;
static u8 https_tls_req_buf[3072];   /* >= STS_TOKEN_MAX + request header overhead */
static u32 https_tls_req_len;
static bool https_tls_accepted;
static bool https_tls_response_sent;
static u64 https_tls_last_activity_ms;
static bool http_reboot_pending;
static i32 ksvc_net_id = -1;
static i32 ksvc_tcp_id = -1;
static i32 ksvc_debug_id = -1;
static i32 ksvc_ui_id = -1;
static i32 ksvc_dashboard_id = -1;
static i32 ksvc_timer_id = -1;
static u32 http_complete_tick;

/* ── Multi-connection HTTP server (:80) ──
 * The :80 server services a POOL of concurrent client connections instead of
 * one-at-a-time, so many simultaneous clients are accepted and advanced in
 * parallel (per poll). Each slot owns its full request/response state; the
 * existing per-request state machine runs on the current slot via the cursor
 * `Hc` (the http_* identifiers below are macros onto Hc->*), so the proven
 * serving logic stays byte-identical. The pool lives in highmem (~17KB/slot). */
#define HTTP_MAX_CONCURRENT 48
struct http_conn {
    tcp_conn_t client_conn;
    u8 req_buf[1024];
    u32 req_len;
    bool auth_checked;
    bool auth_ok;
    char resp_buf[16000];
    u32 resp_len;
    u32 resp_off;
    const u8 *static_body;
    u32 static_len;
    u32 static_off;
    u64 file_id;
    u32 file_len;
    u32 file_off;
    u8 file_chunk[HTTP_TX_CHUNK_MAX];
    u32 last_state;
    u32 last_readable;
    u32 last_writable;
    u32 last_write;
    bool prefix_dumped;
    bool req_prefix_dumped;
    u64 last_activity_ms;
    char last_req_prefix[25];
    char last_resp_prefix[25];
};
static struct http_conn *http_conns;             /* highmem pool [HTTP_MAX_CONCURRENT] */
static struct http_conn  http_conns_fallback[4]; /* if highmem unavailable */
static u32 http_conn_count;
static struct http_conn *Hc;                     /* current slot the state machine acts on */

#define http_client_conn       (Hc->client_conn)
#define http_req_buf           (Hc->req_buf)
#define http_req_len           (Hc->req_len)
#define http_auth_checked      (Hc->auth_checked)
#define http_auth_ok           (Hc->auth_ok)
#define http_resp_buf          (Hc->resp_buf)
#define http_resp_len          (Hc->resp_len)
#define http_resp_off          (Hc->resp_off)
#define http_static_body       (Hc->static_body)
#define http_static_len        (Hc->static_len)
#define http_static_off        (Hc->static_off)
#define http_file_id           (Hc->file_id)
#define http_file_len          (Hc->file_len)
#define http_file_off          (Hc->file_off)
#define http_file_chunk        (Hc->file_chunk)
#define http_last_state        (Hc->last_state)
#define http_last_readable     (Hc->last_readable)
#define http_last_writable     (Hc->last_writable)
#define http_last_write        (Hc->last_write)
#define http_prefix_dumped     (Hc->prefix_dumped)
#define http_req_prefix_dumped (Hc->req_prefix_dumped)
#define http_last_activity_ms  (Hc->last_activity_ms)
#define http_last_req_prefix   (Hc->last_req_prefix)
#define http_last_resp_prefix  (Hc->last_resp_prefix)
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
    /* Single-connection streaming OTA upload: the whole image arrives over ONE
     * POST body (no per-chunk connection churn that overruns the polling NIC),
     * streamed straight into the RAM staging buffer. */
    bool stream_mode;
    bool stream_reboot;
    u32  stream_total;
    u32  stream_received;
    u64  stream_wadv_ms;     /* last window re-advertise while waiting for tail */
    u64  stream_wadv_count;  /* total re-advertises sent this stream */
    u64  stream_last_drain;  /* sched_counter_ticks() at last drain entry */
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
/* OTA RAM staging: chunks land in this highmem buffer (fast memcpy) instead of
 * doing per-chunk blocking SD writes on core0 (which stall the polling NIC and
 * wedge the upload). The full image flushes to the SD slot once, at commit.
 * Falls back to direct per-chunk SD writes if highmem is unavailable. */
static u8 *ota_stage_buf;
static u32 ota_stage_cap;
static bool ota_stage_ready;

static void ota_update_reset_state(void)
{
    ota_update.active = false;
    ota_update.total = 0;
    ota_update.received = 0;
    ota_update.chunks = 0;
    ota_update.errors = 0;
    ota_update.last_error = NULL;
    ota_stage_ready = false;
}

/* ── UART-based firmware flash (no network required) ──
 * At 115200 baud (~11.5 KB/s raw), a FULL kernel image takes a couple of
 * minutes -- slow, but it works even when the NIC is completely wedged,
 * which is exactly the scenario this was built for.
 *
 * Two modes, both reusing the exact same commit path as the network OTA
 * flow (http_write_kernel_payload_range + http_write_kernel_slot_header +
 * pios_bootctrl_mark_pending):
 *
 * FULL mode (fallback, e.g. first-ever flash of a slot):
 *   "uartflash begin <total>", then repeated fixed-size binary chunks
 *   (UARTFLASH_CHUNK_SIZE bytes, final chunk shorter), each acked with
 *   "ACK <received> <checksum_hex>\n" (or "TIMEOUT <received>\n" if a chunk
 *   stalls mid-transfer -- resend from <received> either way).
 *
 * PATCH mode ("zap" -- delta update, per user request): since most builds
 * only change a small fraction of the image, sending the WHOLE thing every
 * time wastes most of a ~2-minute transfer on bytes that didn't change.
 * The diff itself is computed entirely on the BUILD/HOST side (which has
 * both the old and new images and real compute power for a proper
 * byte-level diff, e.g. Python's difflib) -- the device does no hashing or
 * comparison at all, it just:
 *   1. "uartflash patchbegin <base_len>" -- reads its OWN current target
 *      slot content back off the SD card as the patch base (ground truth,
 *      not trusting any host-side cache to be accurate).
 *   2. "uartflash patch <offset> <length>" (length <= UARTFLASH_CHUNK_SIZE),
 *      then <length> raw bytes -- overwrites that one changed region in the
 *      staged copy, acked with "PACK <offset> <checksum_hex>\n". Repeated
 *      once per changed region.
 *   3. "uartflash finalize <total>" -- sets the final image length (which
 *      may differ slightly from the base if size changed).
 * Then "uartflash commit [reboot]" in either mode, same as before.
 *
 * Function bodies are defined later in this file (after
 * http_append/ui_console_write), see uartflash_feed_byte / uartflash_poll /
 * ui_cmd_uartflash. */
#define UARTFLASH_CHUNK_SIZE   1024U
#define UARTFLASH_TIMEOUT_MS   3000ULL
enum uartflash_mode { UARTFLASH_MODE_NONE = 0, UARTFLASH_MODE_FULL, UARTFLASH_MODE_PATCH_REGION };
static enum uartflash_mode uartflash_mode;
static u32 uartflash_patch_offset;
static u32 uartflash_patch_length;
static bool uartflash_active;
static u32 uartflash_chunk_progress;
static u64 uartflash_last_byte_ms;

/* True only while genuinely mid-stream expecting raw bytes right now --
 * NOT simply "a uartflash session is open" (uartflash_active stays true
 * across the whole begin/patchbegin..commit/abort session, including gaps
 * between commands). Reactor code must divert incoming UART bytes to
 * uartflash_feed_byte() ONLY when this is true; otherwise text commands
 * like "uartflash status"/"commit"/"patch <off> <len>" sent *during* a
 * session (e.g. between two patch regions, or after a FULL transfer
 * finishes but before commit) would be silently swallowed as bogus binary
 * data instead of being parsed as console commands. */
static bool uartflash_expecting_bytes(void)
{
    if (!uartflash_active)
        return false;
    if (uartflash_mode == UARTFLASH_MODE_FULL)
        return ota_update.received < ota_update.total;
    if (uartflash_mode == UARTFLASH_MODE_PATCH_REGION)
        return true;
    return false; /* UARTFLASH_MODE_NONE: awaiting the next text command */
}

static tcp_conn_t debug_listen_conn = -1;
static tcp_conn_t debug_client_conn = -1;
static bool debug_tcp_unlocked;
static char debug_tcp_line[DEBUG_TCP_LINE_MAX];
static u32 debug_tcp_len;
static u32 debug_tcp_iac_skip;
static i32 debug_tcp_last_term_char = -1;
static bool debug_tcp_discard_line;
#define CORE0_ETH_IRQ_STALL_THRESHOLD       4U     /* consecutive non-clearing quenches before poll fallback */
#define CORE0_ETH_IRQ_FALLBACK_COOLDOWN_MS  5000ULL /* how long to stay poll-only before retrying IRQ mode */
static volatile u64 core0_eth_irq_count;
static volatile u32 core0_eth_irq_last_mip;
static volatile u32 core0_eth_irq_last_macb_isr;
static volatile bool core0_eth_irq_oneshot;
static volatile bool core0_eth_irq_deferred_quench;
static volatile u32 core0_eth_irq_quench_passes;
static volatile u32 core0_eth_irq_stall_streak;   /* consecutive quenches that failed to clear */
static volatile bool core0_eth_irq_poll_fallback; /* true: IRQ line masked, relying on poll only */
static volatile u64 core0_eth_irq_fallback_since_ms;
static volatile u32 core0_eth_irq_fallback_count;  /* lifetime fallback engagements (diagnostic) */
static volatile u32 core0_io_flags;
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
static u32 http_build_no_content_response(char *out, u32 max);
static u32 http_build_picoscript_response(char *out, u32 max, const u8 *req, u32 req_len);
static u32 http_build_static_file_response(char *out, u32 max, const u8 *req, u32 req_len);
static u32 http_build_pcap_response(char *out, u32 max, const u8 *req, u32 req_len);
static u32 http_build_static_upload_response(char *out, u32 max, const u8 *req, u32 req_len);
static u32 http_build_kernel_update_response(char *out, u32 max, const u8 *req, u32 req_len);
static void pios_bootctrl_mark_success(void);
static void core0_sched_snapshot(u64 *wake, u64 *wfi_count, u64 *idle_ticks,
                                 u64 *total_ticks, u32 *busy_permille,
                                 u32 *last_flags);
static void core0_eth_irq_handler(void);
static void core0_eth_irq_arm_host(bool oneshot);
static volatile u64 g_dash_snap_ticks;
static volatile u64 g_dash_render_ticks;
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
    debug_tcp_last_term_char = -1;
    debug_tcp_discard_line = false;
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
    if (debug_tcp_discard_line) {
        if (c == '\r' || c == '\n') {
            debug_tcp_discard_line = false;
            debug_tcp_last_term_char = c;
        }
        return;
    }
    if (c == '\r' || c == '\n') {
        if (debug_tcp_last_term_char >= 0 && debug_tcp_last_term_char != c) {
            debug_tcp_last_term_char = -1;
            return;
        }
        debug_tcp_last_term_char = c;
        debug_tcp_handle_line();
        return;
    }
    debug_tcp_last_term_char = -1;
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
        static const char locked_msg[] = "\r\nERR: line too long\r\ndebug> ";
        static const char unlocked_msg[] = "\r\nERR: line too long\r\npios> ";
        const char *msg = debug_tcp_unlocked ? unlocked_msg : locked_msg;
        (void)tcp_write(debug_client_conn, msg, pios_strlen(msg));
        debug_tcp_len = 0;
        debug_tcp_discard_line = true;
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
            debug_tcp_last_term_char = -1;
            debug_tcp_discard_line = false;
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
    debug_tcp_last_term_char = -1;
    debug_tcp_discard_line = false;

    echo_listen_conn = -1;
    http_listen_conn = tcp_listen(HTTP_TCP_PORT);
    https_tls_listen_conn = tcp_listen(HTTPS_TLS_TCP_PORT);
    debug_listen_conn = tcp_listen(DEBUG_TCP_PORT);
    admin_services_listen();
    uhttp_bridge_init();   /* userland HTTP on :81 (kernel-terminated TCP) */
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

static void http_append_bytes(char *out, u32 *len, u32 max, const u8 *data, u32 n)
{
    if (!out || !len || !data || max == 0)
        return;
    for (u32 i = 0; i < n && *len < max - 1; i++)
        out[(*len)++] = (char)data[i];
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
    char token[256];
    if (http_request_path_prefix_token(req, len, "/.well-known/acme-challenge/", token, sizeof(token)))
        return HTTP_ROUTE_ACME;
    if (http_request_path_is(req, len, "/api/status"))
        return HTTP_ROUTE_STATUS;
    if (http_request_path_is(req, len, "/api/netstat"))
        return HTTP_ROUTE_NETSTAT;
    if (http_request_path_prefix_token(req, len, "/static/", token, sizeof(token)))
        return HTTP_ROUTE_STATIC;
    if (http_request_path_is(req, len, "/picoscript") ||
        http_request_path_is(req, len, "/picoscript/") ||
        http_request_path_is(req, len, "/picoscript/index.html") ||
        http_request_path_is(req, len, "/picoscript/playground.html") ||
        http_request_path_is(req, len, "/picoscript/picowal.html") ||
        http_request_path_is(req, len, "/picoscript/config") ||
        http_request_path_is(req, len, "/picoscript/pico_hooks.js") ||
        http_request_path_is(req, len, "/picoscript/baremetal-binary.js"))
        return HTTP_ROUTE_PICOSCRIPT;
    if (http_request_path_is(req, len, "/favicon.ico") ||
        http_request_path_is(req, len, "/picoscript/favicon.ico"))
        return HTTP_ROUTE_FAVICON;
    if (http_request_path_is(req, len, "/api/admin/reboot"))
        return HTTP_ROUTE_REBOOT;
    if (http_request_path_is(req, len, "/api/admin/log-stream") ||
        http_request_path_is(req, len, "/api/logs") ||
        http_request_path_is(req, len, "/logs"))
        return HTTP_ROUTE_LOGS;
    if (http_request_path_is(req, len, "/api/admin/kernel-update"))
        return HTTP_ROUTE_UPDATE;
    if (http_request_path_is(req, len, "/api/admin/static-put"))
        return HTTP_ROUTE_STATIC_PUT;
    if (http_request_path_is(req, len, "/api/admin/hotpatch-kernel"))
        return HTTP_ROUTE_HOTPATCH;
    if (http_request_path_is(req, len, "/api/terminal") ||
        http_request_path_is(req, len, "/api/process") ||
        http_request_path_is(req, len, "/api/user") ||
        http_request_path_is(req, len, "/api/walfs"))
        return HTTP_ROUTE_PLACEHOLDER;
    if (http_request_path_is(req, len, "/api/capsule"))
        return HTTP_ROUTE_CAPSULE;
    if (http_request_path_is(req, len, "/api/admin/pcap"))
        return HTTP_ROUTE_PCAP;
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

struct perf_counter_snapshot {
    u32 cpu_permille[4];
    u32 cpu_total_permille;
    u32 cpu_clock_mhz;
    u64 sched_wake;
    u64 sched_wfi;
    u64 sched_idle_ticks;
    u64 sched_total_ticks;
    u32 sched_flags;
    u32 board_revision;
    bool board_revision_new_style;
    u32 board_mem_code;
    u32 board_model_code;
    u32 board_processor_code;
    u32 board_manufacturer_code;
    u32 board_pcb_revision;
    u64 installed_ram_bytes;
    u64 physical_ram_bytes;
    struct highmem_status highmem;
    u32 ram_total_kib;
    u32 ram_used_kib;
    u32 ram_kernel_kib;
    u32 ram_user_kib;
    u32 core_alloc_kib[4];
    u32 kernel_mem_kib;
    u32 kernel_static_kib;
    u32 kernel_core_kib;
    u32 kernel_cap_kib;
    u32 proc_total;
    u32 proc_ready;
    u32 proc_running;
    u32 proc_blocked;
    u32 proc_dead;
    u32 net_listen_count;
    u32 net_listen_pending;
    u64 nic_rx_bytes;
    u64 nic_tx_bytes;
    u32 nic_rx_mbps_x1000;
    u32 nic_tx_mbps_x1000;
    u32 nic_rx_peak_mbps_x1000;
    u32 nic_tx_peak_mbps_x1000;
    u32 nic_rx_wedge;
    u32 nic_rx_hole_recover;
    u32 nic_rx_idle;
    u32 nic_link_mbps;
    bool nic_link_full_duplex;
    u32 nic_rx_capacity_mbps;
    u32 nic_tx_capacity_mbps;
    u64 sd_read_bytes;
    u64 sd_write_bytes;
    u32 sd_read_mbps_x1000;
    u32 sd_write_mbps_x1000;
    u32 sd_read_last_mbps_x1000;
    u32 sd_write_last_mbps_x1000;
    u32 sd_read_peak_mbps_x1000;
    u32 sd_write_peak_mbps_x1000;
    bool walfs_mounted;
    bool walfs_super_ok;
    bool walfs_root_ok;
    bool walfs_legacy_present;
    u32 walfs_partition_lba;
    u32 walfs_base_lba;
    u64 walfs_partition_bytes;
    u64 walfs_region_bytes;
    u64 walfs_used_bytes;
    u64 walfs_free_bytes;
    u32 walfs_records;
    u64 dash_snap_ticks;
    u64 dash_render_ticks;
    u64 fb_blit_ticks;
};

struct board_revision_snapshot {
    u32 revision;
    bool new_style;
    u32 mem_code;
    u32 model_code;
    u32 processor_code;
    u32 manufacturer_code;
    u32 pcb_revision;
    u64 installed_ram_bytes;
};

static u64 board_revision_memory_bytes(u32 mem_code)
{
    static const u64 sizes[] = {
        256ULL << 20, 512ULL << 20, 1024ULL << 20, 2048ULL << 20,
        4096ULL << 20, 8192ULL << 20, 16384ULL << 20
    };
    return mem_code < (u32)(sizeof(sizes) / sizeof(sizes[0])) ? sizes[mem_code] : 0;
}

static void board_revision_snapshot(struct board_revision_snapshot *out)
{
    static bool probed;
    static struct board_revision_snapshot snap;
    static volatile u32 ALIGNED(16) mbox_rev[7];

    if (!out)
        return;
#if !PIOS_HAS_MAILBOX_FB
    if (!probed) {
        probed = true;
        snap.installed_ram_bytes = 512ULL << 20;
    }
    *out = snap;
    return;
#endif
    if (!probed) {
        probed = true;
        mbox_rev[0] = sizeof(mbox_rev);
        mbox_rev[1] = 0;
        mbox_rev[2] = TAG_GET_BOARD_REV;
        mbox_rev[3] = 4;
        mbox_rev[4] = 0;
        mbox_rev[5] = 0;
        mbox_rev[6] = TAG_END;
        if (mbox_call(MBOX_CH_PROP, mbox_rev) && (mbox_rev[4] & 0x80000000U)) {
            snap.revision = mbox_rev[5];
            snap.new_style = (snap.revision & (1U << 23)) != 0;
            if (snap.new_style) {
                snap.mem_code = (snap.revision >> 20) & 0x7U;
                snap.manufacturer_code = (snap.revision >> 16) & 0xFU;
                snap.processor_code = (snap.revision >> 12) & 0xFU;
                snap.model_code = (snap.revision >> 4) & 0xFFU;
                snap.pcb_revision = snap.revision & 0xFU;
                snap.installed_ram_bytes = board_revision_memory_bytes(snap.mem_code);
            }
        }
    }
    *out = snap;
}

static u64 perf_physical_ram_bytes(void)
{
#if !PIOS_HAS_MAILBOX_FB
    return 512ULL << 20;
#else
    static bool probed;
    static u64 bytes;
    static volatile u32 ALIGNED(16) mbox_mem[8];
    if (probed)
        return bytes;
    probed = true;
    mbox_mem[0] = sizeof(mbox_mem);
    mbox_mem[1] = 0;
    mbox_mem[2] = TAG_GET_ARM_MEMORY;
    mbox_mem[3] = 8;
    mbox_mem[4] = 0;
    mbox_mem[5] = 0;
    mbox_mem[6] = 0;
    mbox_mem[7] = TAG_END;
    if (mbox_call(MBOX_CH_PROP, mbox_mem) && (mbox_mem[4] & 0x80000000U))
        bytes = mbox_mem[6];
    return bytes;
#endif
}

static u64 perf_counter_delta(u64 now, u64 last)
{
    return now >= last ? now - last : now;
}

static u32 perf_mbps_x1000(u64 bytes, u64 elapsed_ms)
{
    if (elapsed_ms == 0)
        return 0;
    u64 v = (bytes * 8ULL) / elapsed_ms;
    return v > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (u32)v;
}

/*
 * Published cluster A76 core clock in MHz from the PMU cycle counter (true core
 * Hz). BCM2712 is one ARM frequency domain, so one reading covers all cores.
 * Written ONLY by core 0 from its main loop on a ~2.5s cadence; read by the
 * dashboard and /api/status snapshot, both on core 0 — single-writer, same-core
 * reader. 0 = not yet measured. (The serial-loop "effective throughput" and the
 * non-cacheable-execution penalty are reported on demand by `cpuclock`.)
 */
static volatile u32 g_cpu_clock_mhz;

static void perf_cpu_clock_measure(u32 window_us)
{
    u64 hz = cpu_clock_pmu_hz(window_us);
    if (!hz)
        hz = cpu_clock_estimate_hz(window_us);   /* fallback if PMU unavailable */
    g_cpu_clock_mhz = (u32)((hz + 500000ULL) / 1000000ULL);
}

static void perf_counter_snapshot(struct perf_counter_snapshot *p)
{
    static u64 rate_last_ms;
    static u64 rate_last_nic_rx;
    static u64 rate_last_nic_tx;
    static u64 rate_last_sd_reads;
    static u64 rate_last_sd_writes;
    static u32 rate_nic_rx;
    static u32 rate_nic_tx;
    static u32 rate_sd_read;
    static u32 rate_sd_write;
    static u32 peak_nic_rx;
    static u32 peak_nic_tx;

    if (!p)
        return;
    simd_zero(p, sizeof(*p));
    struct board_revision_snapshot br;
    board_revision_snapshot(&br);
    p->board_revision = br.revision;
    p->board_revision_new_style = br.new_style;
    p->board_mem_code = br.mem_code;
    p->board_model_code = br.model_code;
    p->board_processor_code = br.processor_code;
    p->board_manufacturer_code = br.manufacturer_code;
    p->board_pcb_revision = br.pcb_revision;
    p->installed_ram_bytes = br.installed_ram_bytes;
    p->physical_ram_bytes = perf_physical_ram_bytes();
    highmem_status(&p->highmem);
    p->ram_total_kib = (u32)((CORE_PRIV_SIZE * 4ULL) >> 10);
    for (u32 i = 0; i < 4; i++)
        p->core_alloc_kib[i] = http_core_ram_used_kib(i);

    core0_sched_snapshot(&p->sched_wake, &p->sched_wfi,
                         &p->sched_idle_ticks, &p->sched_total_ticks,
                         &p->cpu_permille[0], &p->sched_flags);
    struct proc_sched_core_snapshot pcs[3];
    u32 pcs_n = proc_sched_snapshot(pcs, 3);
    p->cpu_permille[1] = (pcs_n > 0) ? pcs[0].busy_permille : 0;
    p->cpu_permille[2] = (pcs_n > 1) ? pcs[1].busy_permille : 0;
    p->cpu_permille[3] = (pcs_n > 2) ? pcs[2].busy_permille : 0;
    p->cpu_total_permille = (p->cpu_permille[0] + p->cpu_permille[1] +
                             p->cpu_permille[2] + p->cpu_permille[3]) / 4U;
    p->cpu_clock_mhz = g_cpu_clock_mhz;

    struct proc_ui_entry proc[MAX_PROCS_PER_CORE + 1U];
    u32 proc_n = proc_snapshot(proc, MAX_PROCS_PER_CORE + 1U);
    p->proc_total = proc_n;
    if (proc_n > 0) {
        p->kernel_mem_kib = proc[0].mem_kib;
        p->kernel_static_kib = proc[0].arena_span_kib;
        p->kernel_core_kib = proc[0].arena_used_kib;
        p->kernel_cap_kib = proc[0].arena_capacity_kib;
        p->ram_kernel_kib = proc[0].mem_kib;
    }
    for (u32 i = 0; i < proc_n; i++) {
        if (i > 0)
            p->ram_user_kib += proc[i].mem_kib;
        if (proc[i].state == PROC_READY) p->proc_ready++;
        else if (proc[i].state == PROC_RUNNING) p->proc_running++;
        else if (proc[i].state == PROC_BLOCKED) p->proc_blocked++;
        else if (proc[i].state == PROC_DEAD) p->proc_dead++;
    }
    p->ram_used_kib = p->ram_kernel_kib + p->ram_user_kib;
    tcp_snapshot_entry_t tcp[TCP_MAX_CONNECTIONS];
    u32 tcp_n = tcp_snapshot(tcp, TCP_MAX_CONNECTIONS);
    for (u32 i = 0; i < tcp_n; i++) {
        if (tcp[i].state == TCP_LISTEN) {
            p->net_listen_count++;
            p->net_listen_pending += tcp[i].pending_count;
        }
    }

    nic_packet_counters_t nc;
    nic_packet_counters(&nc);
    const sd_stats_t *ss = sd_get_stats();
    u64 sd_reads = ss ? ss->reads : 0;
    u64 sd_writes = ss ? ss->writes : 0;
    p->nic_rx_bytes = nc.rx_bytes;
    p->nic_tx_bytes = nc.tx_bytes;
    p->sd_read_bytes = sd_reads * (u64)SD_BLOCK_SIZE;
    p->sd_write_bytes = sd_writes * (u64)SD_BLOCK_SIZE;

    u64 now_ms = timer_monotonic_ms();
    if (rate_last_ms == 0) {
        rate_last_ms = now_ms ? now_ms : 1;
        rate_last_nic_rx = nc.rx_bytes;
        rate_last_nic_tx = nc.tx_bytes;
        rate_last_sd_reads = sd_reads;
        rate_last_sd_writes = sd_writes;
    } else if (now_ms > rate_last_ms && now_ms - rate_last_ms >= 250ULL) {
        u64 dt = now_ms - rate_last_ms;
        rate_nic_rx = perf_mbps_x1000(perf_counter_delta(nc.rx_bytes, rate_last_nic_rx), dt);
        rate_nic_tx = perf_mbps_x1000(perf_counter_delta(nc.tx_bytes, rate_last_nic_tx), dt);
        rate_sd_read = perf_mbps_x1000(perf_counter_delta(sd_reads, rate_last_sd_reads) * (u64)SD_BLOCK_SIZE, dt);
        rate_sd_write = perf_mbps_x1000(perf_counter_delta(sd_writes, rate_last_sd_writes) * (u64)SD_BLOCK_SIZE, dt);
        if (rate_nic_rx > peak_nic_rx) peak_nic_rx = rate_nic_rx;
        if (rate_nic_tx > peak_nic_tx) peak_nic_tx = rate_nic_tx;
        rate_last_ms = now_ms;
        rate_last_nic_rx = nc.rx_bytes;
        rate_last_nic_tx = nc.tx_bytes;
        rate_last_sd_reads = sd_reads;
        rate_last_sd_writes = sd_writes;
    }
    p->nic_rx_mbps_x1000 = rate_nic_rx;
    p->nic_tx_mbps_x1000 = rate_nic_tx;
    p->nic_rx_peak_mbps_x1000 = peak_nic_rx;
    p->nic_tx_peak_mbps_x1000 = peak_nic_tx;
#if PIOS_HAS_GENET
    {
        struct macb_diag md;
        macb_diag(&md);
        p->nic_rx_wedge = md.rx_wedge;
        p->nic_rx_hole_recover = md.rx_hole_recover;
        p->nic_rx_idle = md.rx_idle;
    }
#else
    p->nic_rx_wedge = 0;
    p->nic_rx_hole_recover = 0;
    p->nic_rx_idle = 0;
#endif
    p->nic_link_mbps = nic_link_mbps();
    p->nic_link_full_duplex = nic_link_full_duplex();
    p->nic_rx_capacity_mbps = p->nic_link_mbps;
    p->nic_tx_capacity_mbps = p->nic_link_mbps;
    p->sd_read_mbps_x1000 = rate_sd_read;
    p->sd_write_mbps_x1000 = rate_sd_write;
    p->sd_read_last_mbps_x1000 = ss ? ss->read_last_mbps_x1000 : 0;
    p->sd_write_last_mbps_x1000 = ss ? ss->write_last_mbps_x1000 : 0;
    p->sd_read_peak_mbps_x1000 = ss ? ss->read_peak_mbps_x1000 : 0;
    p->sd_write_peak_mbps_x1000 = ss ? ss->write_peak_mbps_x1000 : 0;

    struct walfs_status_snapshot ws;
    walfs_status(&ws);
    p->walfs_mounted = ws.mounted;
    p->walfs_super_ok = ws.super_ok;
    p->walfs_root_ok = ws.root_ok;
    p->walfs_legacy_present = ws.legacy_present;
    p->walfs_partition_lba = ws.partition_lba;
    p->walfs_base_lba = ws.base_lba;
    p->walfs_partition_bytes = (u64)ws.partition_blocks * (u64)SD_BLOCK_SIZE;
    p->walfs_region_bytes = (u64)ws.region_blocks * (u64)SD_BLOCK_SIZE;
    p->walfs_used_bytes = ws.super_head;
    p->walfs_free_bytes = p->walfs_region_bytes > p->walfs_used_bytes ?
                          p->walfs_region_bytes - p->walfs_used_bytes : 0;
    p->walfs_records = ws.super_records;
    p->dash_snap_ticks = g_dash_snap_ticks;
    p->dash_render_ticks = g_dash_render_ticks;
    p->fb_blit_ticks = fb_last_blit_ticks();
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
    struct perf_counter_snapshot perf;
    perf_counter_snapshot(&perf);
    const struct videocore_probe *vc = videocore_probe_get();
    const struct vc_display_status *vcd = vc_display_status_get();
    http_append_json_metric(out, &len, max, "sched_wake", perf.sched_wake, true);
    http_append_json_metric(out, &len, max, "sched_wfi", perf.sched_wfi, true);
    http_append_json_metric(out, &len, max, "sched_busy_permille", perf.cpu_permille[0], true);
    http_append_json_metric(out, &len, max, "sched_flags", perf.sched_flags, false);
    http_append(out, &len, max, "},\"perf\":{");
    http_append_json_metric(out, &len, max, "cpu_total_permille", perf.cpu_total_permille, true);
    http_append_json_metric(out, &len, max, "cpu0_permille", perf.cpu_permille[0], true);
    http_append_json_metric(out, &len, max, "cpu1_permille", perf.cpu_permille[1], true);
    http_append_json_metric(out, &len, max, "cpu2_permille", perf.cpu_permille[2], true);
    http_append_json_metric(out, &len, max, "cpu3_permille", perf.cpu_permille[3], true);
    http_append_json_metric(out, &len, max, "cpu_clock_mhz", perf.cpu_clock_mhz, true);
    http_append_json_metric(out, &len, max, "board_revision", perf.board_revision, true);
    http_append_json_metric(out, &len, max, "board_revision_new_style", perf.board_revision_new_style ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "board_mem_code", perf.board_mem_code, true);
    http_append_json_metric(out, &len, max, "board_model_code", perf.board_model_code, true);
    http_append_json_metric(out, &len, max, "board_processor_code", perf.board_processor_code, true);
    http_append_json_metric(out, &len, max, "board_manufacturer_code", perf.board_manufacturer_code, true);
    http_append_json_metric(out, &len, max, "board_pcb_revision", perf.board_pcb_revision, true);
    http_append_json_metric(out, &len, max, "videocore_enabled", vc && vc->enabled ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_hvs_seen", vc && vc->hvs_seen ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_seen", vc && vc->v3d_seen ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_tech", vc ? vc->v3d_tech_version : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_cores", vc ? vc->v3d_core_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_slices", vc ? vc->v3d_slice_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_qpus_per_slice", vc ? vc->v3d_qpus_per_slice : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_mmu", vc && vc->v3d_has_mmu ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_v3d_l3c_kb", vc ? vc->v3d_l3c_kb : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_ready", vcd && vcd->ready ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_backend", vcd ? (u32)vcd->backend : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_native_probe", vcd && vcd->native_probe_ready ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_native_owner", vcd && vcd->native_owner ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_width", vcd ? vcd->width : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_height", vcd ? vcd->height : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_pitch", vcd ? vcd->pitch : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_scanout", vcd ? vcd->scanout_base : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_presents", vcd ? vcd->present_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_previous_backend", vcd ? (u32)vcd->previous_backend : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_takeover_attempts", vcd ? vcd->takeover_attempts : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_fallback_count", vcd ? vcd->fallback_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_takeover_status", vcd ? vcd->last_takeover_status : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_snapshot_count", vcd ? vcd->snapshot_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_hvs_control", vcd ? vcd->hvs_control : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_hvs_dl_status", vcd ? vcd->hvs_dl_status : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_status", vcd ? vcd->dlist_status : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_count", vcd ? vcd->dlist_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_format", vcd ? vcd->dlist_format : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_order", vcd ? vcd->dlist_order : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_stage_index", vcd ? vcd->dlist_stage_index : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_stage_count", vcd ? vcd->dlist_stage_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_stage_readback_ok", vcd ? vcd->dlist_stage_readback_ok : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_arm_channel", vcd ? vcd->dlist_arm_channel : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_arm_readback_ok", vcd ? vcd->dlist_arm_readback_ok : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_arm_lptrs", vcd ? vcd->dlist_arm_lptrs_after : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_restore_count", vcd ? vcd->dlist_restore_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_restore_readback_ok", vcd ? vcd->dlist_restore_readback_ok : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_dlist_restore_lptrs", vcd ? vcd->dlist_restore_lptrs : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_channel_status", vcd ? vcd->channel_reapply_status : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_channel_reapply_count", vcd ? vcd->channel_reapply_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_channel_reapply_ok", vcd ? vcd->channel_reapply_readback_ok : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_global_status", vcd ? vcd->global_reapply_status : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_global_reapply_count", vcd ? vcd->global_reapply_count : 0U, true);
    http_append_json_metric(out, &len, max, "videocore_display_global_reapply_ok", vcd ? vcd->global_reapply_readback_ok : 0U, true);
    http_append_json_metric(out, &len, max, "ram_installed_bytes", perf.installed_ram_bytes, true);
    http_append_json_metric(out, &len, max, "ram_installed_kib", perf.installed_ram_bytes >> 10, true);
    http_append_json_metric(out, &len, max, "ram_arm_visible_bytes", perf.physical_ram_bytes, true);
    http_append_json_metric(out, &len, max, "ram_arm_visible_kib", perf.physical_ram_bytes >> 10, true);
    http_append_json_metric(out, &len, max, "ram_physical_bytes", perf.physical_ram_bytes, true);
    http_append_json_metric(out, &len, max, "ram_physical_kib", perf.physical_ram_bytes >> 10, true);
    http_append_json_metric(out, &len, max, "ram_high_ready", perf.highmem.ready ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "ram_high_probe_ok", perf.highmem.probe_ok ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "ram_high_base", perf.highmem.base, true);
    http_append_json_metric(out, &len, max, "ram_high_limit", perf.highmem.limit, true);
    http_append_json_metric(out, &len, max, "ram_high_total_bytes", perf.highmem.total_bytes, true);
    http_append_json_metric(out, &len, max, "ram_high_used_bytes", perf.highmem.used_bytes, true);
    http_append_json_metric(out, &len, max, "ram_high_free_bytes", perf.highmem.free_bytes, true);
    http_append_json_metric(out, &len, max, "ram_high_probe_fail_addr", perf.highmem.probe_fail_addr, true);
    http_append_json_metric(out, &len, max, "ram_high_probe_lines", perf.highmem.probe_lines, true);
    http_append_json_metric(out, &len, max, "ram_high_alloc_count", perf.highmem.alloc_count, true);
    http_append_json_metric(out, &len, max, "ram_total_kib", perf.ram_total_kib, true);
    http_append_json_metric(out, &len, max, "ram_pool_total_kib", perf.ram_total_kib, true);
    http_append_json_metric(out, &len, max, "ram_used_kib", perf.ram_used_kib, true);
    http_append_json_metric(out, &len, max, "ram_kernel_kib", perf.ram_kernel_kib, true);
    http_append_json_metric(out, &len, max, "ram_user_kib", perf.ram_user_kib, true);
    http_append_json_metric(out, &len, max, "kernel_mem_kib", perf.kernel_mem_kib, true);
    http_append_json_metric(out, &len, max, "kernel_static_kib", perf.kernel_static_kib, true);
    http_append_json_metric(out, &len, max, "kernel_core_kib", perf.kernel_core_kib, true);
    http_append_json_metric(out, &len, max, "kernel_cap_kib", perf.kernel_cap_kib, true);
    http_append_json_metric(out, &len, max, "net_listen_count", perf.net_listen_count, true);
    http_append_json_metric(out, &len, max, "net_listen_pending", perf.net_listen_pending, true);
    http_append_json_metric(out, &len, max, "nic_rx_bytes", perf.nic_rx_bytes, true);
    http_append_json_metric(out, &len, max, "nic_tx_bytes", perf.nic_tx_bytes, true);
    http_append_json_metric(out, &len, max, "nic_rx_mbps_x1000", perf.nic_rx_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "nic_tx_mbps_x1000", perf.nic_tx_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "nic_rx_peak_mbps_x1000", perf.nic_rx_peak_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "nic_tx_peak_mbps_x1000", perf.nic_tx_peak_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "nic_rx_wedge", perf.nic_rx_wedge, true);
    http_append_json_metric(out, &len, max, "nic_rx_hole_recover", perf.nic_rx_hole_recover, true);
    http_append_json_metric(out, &len, max, "nic_rx_idle", perf.nic_rx_idle, true);
    http_append_json_metric(out, &len, max, "nic_link_mbps", perf.nic_link_mbps, true);
    http_append_json_metric(out, &len, max, "nic_link_full_duplex", perf.nic_link_full_duplex ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "nic_rx_capacity_mbps", perf.nic_rx_capacity_mbps, true);
    http_append_json_metric(out, &len, max, "nic_tx_capacity_mbps", perf.nic_tx_capacity_mbps, true);
    http_append_json_metric(out, &len, max, "sd_read_bytes", perf.sd_read_bytes, true);
    http_append_json_metric(out, &len, max, "sd_write_bytes", perf.sd_write_bytes, true);
    http_append_json_metric(out, &len, max, "sd_read_mbps_x1000", perf.sd_read_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "sd_write_mbps_x1000", perf.sd_write_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "sd_read_last_mbps_x1000", perf.sd_read_last_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "sd_write_last_mbps_x1000", perf.sd_write_last_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "sd_read_peak_mbps_x1000", perf.sd_read_peak_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "sd_write_peak_mbps_x1000", perf.sd_write_peak_mbps_x1000, true);
    http_append_json_metric(out, &len, max, "walfs_mounted", perf.walfs_mounted ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "walfs_super_ok", perf.walfs_super_ok ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "walfs_root_ok", perf.walfs_root_ok ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "walfs_legacy_present", perf.walfs_legacy_present ? 1U : 0U, true);
    http_append_json_metric(out, &len, max, "walfs_partition_lba", perf.walfs_partition_lba, true);
    http_append_json_metric(out, &len, max, "walfs_base_lba", perf.walfs_base_lba, true);
    http_append_json_metric(out, &len, max, "walfs_partition_bytes", perf.walfs_partition_bytes, true);
    http_append_json_metric(out, &len, max, "walfs_region_bytes", perf.walfs_region_bytes, true);
    http_append_json_metric(out, &len, max, "walfs_used_bytes", perf.walfs_used_bytes, true);
    http_append_json_metric(out, &len, max, "walfs_free_bytes", perf.walfs_free_bytes, true);
    http_append_json_metric(out, &len, max, "walfs_records", perf.walfs_records, true);
    http_append_json_metric(out, &len, max, "dash_snap_ticks", perf.dash_snap_ticks, true);
    http_append_json_metric(out, &len, max, "dash_render_ticks", perf.dash_render_ticks, true);
    http_append_json_metric(out, &len, max, "fb_blit_ticks", perf.fb_blit_ticks, false);
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
    http_append(out, &len, max, "},\"table\":{");
    {
        u32 cap = 0, inuse = 0, lsn = 0;
        tcp_table_stats(&cap, &inuse, &lsn);
        http_append_json_metric(out, &len, max, "capacity", cap, true);
        http_append_json_metric(out, &len, max, "inuse", inuse, true);
        http_append_json_metric(out, &len, max, "listeners", lsn, false);
    }
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

/* ---- PicoScript bytecode VM (picovm) -------------------------------------
 * Vendored from willeastbury/picoscript vm/picovm.c: the freestanding 16-opcode
 * VM that is byte-identical to the Python/JS reference VMs. This is the on-board
 * execution engine for compiled PicoScript "pixe" capsules. The `pixe` command
 * runs a bytecode program and reports steps/status/regs/out, so on-board
 * execution can be verified bit-for-bit against the off-board reference VM. */
static void pixe_append_i32(char *out, u32 *len, u32 max, i32 v)
{
    if (v < 0) {
        http_append(out, len, max, "-");
        http_append_u64(out, len, max, (u64)(-(i64)v));
    } else {
        http_append_u64(out, len, max, (u64)(u32)v);
    }
}

static int pixe_parse_hex32(const char *s, u32 *out)
{
    if (!s || !*s) return 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    u32 v = 0;
    int any = 0;
    while (*s) {
        char c = *s++;
        u32 d;
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return 0;
        v = (v << 4) | d;
        any = 1;
    }
    if (!any) return 0;
    *out = v;
    return 1;
}

static bool pixe_hex_nibble(char c, u8 *out)
{
    if (c >= '0' && c <= '9') { *out = (u8)(c - '0'); return true; }
    if (c >= 'a' && c <= 'f') { *out = (u8)(10 + c - 'a'); return true; }
    if (c >= 'A' && c <= 'F') { *out = (u8)(10 + c - 'A'); return true; }
    return false;
}

static bool pixe_parse_hex_byte_pair(const char *s, u8 *out)
{
    u8 hi = 0, lo = 0;
    if (!s || !pixe_hex_nibble(s[0], &hi) || !pixe_hex_nibble(s[1], &lo))
        return false;
    *out = (u8)((hi << 4) | lo);
    return true;
}

/* examples/sum.pc compiled with picoscript_build.py emit --as bytecode.
 * Reference VM result: 55 steps, status=200, regs=[1,10,55,11,55,0...], out empty. */
static const u32 pixe_sum_program[] = {
    0x50010000u, 0x4000000au, 0x41000000u, 0x50010000u, 0x40000000u,
    0x42000000u, 0x50010000u, 0x40000001u, 0x43000000u, 0xa3130004u,
    0x42210003u, 0x83000000u, 0x90000009u, 0x000080c8u, 0x0000a002u,
    0x20200001u, 0x44200000u, 0xc0000000u, 0xc0000000u
};

static void pixe_run_and_report(char *out, u32 *len, u32 max, const u32 *prog, int n)
{
    static pv_ctx vmctx;
    pv_init(&vmctx);
    long steps = pv_vm_run(&vmctx, prog, n);
    http_append(out, len, max, "pixe words=");
    http_append_u64(out, len, max, (u64)(u32)n);
    http_append(out, len, max, " steps=");
    http_append_u64(out, len, max, (u64)steps);
    http_append(out, len, max, " status=");
    pixe_append_i32(out, len, max, vmctx.http_status);
    http_append(out, len, max, " regs=");
    for (int i = 0; i < PV_NUM_REGS; i++) {
        if (i) http_append(out, len, max, ",");
        pixe_append_i32(out, len, max, vmctx.regs[i]);
    }
    http_append(out, len, max, " out=");
    if (vmctx.out_len == 0) {
        http_append(out, len, max, "(empty)");
    } else {
        for (int i = 0; i < vmctx.out_len; i++)
            http_append_hex8(out, len, max, vmctx.out[i]);
    }
    http_append(out, len, max, "\n");
}

static void http_append_tensor_tail(char *out, u32 *len, u32 max)
{
    http_append(out, len, max, " stage=");
    http_append_u64(out, len, max, (u32)tensor_tiny_last_stage());
    http_append(out, len, max, " status=");
    http_append_u64(out, len, max, (u32)tensor_tiny_last_status());
    http_append(out, len, max, " out=");
    http_append_u64(out, len, max, tensor_tiny_last_output_bits());
    http_append(out, len, max, " expect=");
    http_append_u64(out, len, max, tensor_tiny_last_expected_bits());
    struct v3d_csd_debug dbg;
    v3d_csd_debug_last(&dbg);
    http_append(out, len, max, " csd_st=");
    http_append_hex32(out, len, max, dbg.status_before);
    http_append(out, len, max, "/");
    http_append_hex32(out, len, max, dbg.status_after_kick);
    http_append(out, len, max, "/");
    http_append_hex32(out, len, max, dbg.status_after_wait);
    http_append(out, len, max, " cur=");
    http_append_hex32(out, len, max, dbg.current_cfg0);
    http_append(out, len, max, "/");
    http_append_hex32(out, len, max, dbg.current_cfg5);
    http_append(out, len, max, "/");
    http_append_hex32(out, len, max, dbg.current_cfg6);
    http_append(out, len, max, "\n");
}

static void http_bench_row(char *out, u32 *len, u32 max, const char *label,
                           u32 op, u32 m, u32 k, u32 p, u64 flops,
                           u32 reps_sn, u32 reps_v3d, bool include_v3d)
{
    u64 s_ns = tensor_bench(op, m, k, p, TENSOR_BENCH_SCALAR, reps_sn);
    u64 n_ns = tensor_bench(op, m, k, p, TENSOR_BENCH_NEON, reps_sn);
    http_append(out, len, max, label);
    http_append(out, len, max, " s_ns=");
    http_append_u64(out, len, max, s_ns);
    http_append(out, len, max, " n_ns=");
    http_append_u64(out, len, max, n_ns);
    http_append(out, len, max, " nx100=");
    http_append_u64(out, len, max, n_ns ? (s_ns * 100ULL) / n_ns : 0ULL);
    http_append(out, len, max, " ngf100=");
    http_append_u64(out, len, max, n_ns ? (flops * 100ULL) / n_ns : 0ULL);
    if (include_v3d) {
        u64 v_ns = tensor_bench(op, m, k, p, TENSOR_BENCH_V3D, reps_v3d);
        http_append(out, len, max, " v_ns=");
        http_append_u64(out, len, max, v_ns);
        http_append(out, len, max, " vx100=");
        http_append_u64(out, len, max, v_ns ? (s_ns * 100ULL) / v_ns : 0ULL);
        http_append(out, len, max, " vgf100=");
        http_append_u64(out, len, max, v_ns ? (flops * 100ULL) / v_ns : 0ULL);
    }
    http_append(out, len, max, "\n");
}

/* ---- PicoSTS console command (shared helpers + deterministic test harness) ----
 *
 * The `sts` console command exercises the real sts_login/sts_validate/upsert
 * code paths deterministically so QEMU tests can cover login/validate/tamper/
 * scope-denial without needing a TLS client. Test provisioning (fixed secret +
 * known users) is compiled ONLY on the QEMU platform, so production/hardware
 * builds never ship known credentials. No token bytes are ever printed. */
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#define STS_TEST_PROVISION 1
#else
#define STS_TEST_PROVISION 0
#endif

static char g_sts_test_token[STS_TOKEN_MAX];
static u32  g_sts_test_token_len;

static const char *sts_err_name(i32 rc)
{
    switch (rc) {
    case STS_OK:            return "ok";
    case STS_ERR_INPUT:     return "invalid_input";
    case STS_ERR_AUDIENCE:  return "audience_denied";
    case STS_ERR_AUTH:      return "auth_failed";
    case STS_ERR_FORBIDDEN: return "forbidden";
    case STS_ERR_SCOPE:     return "scope_denied";
    case STS_ERR_NOSECRET:  return "no_signing_secret";
    case STS_ERR_NOTIME:    return "clock_unset";
    case STS_ERR_EXPIRED:   return "token_expired";
    case STS_ERR_FULL:      return "store_full";
    default:                return "internal_error";
    }
}

static u32 sts_next_arg(const char *s, u32 i, char *out, u32 cap)
{
    while (s[i] == ' ') i++;
    u32 o = 0;
    while (s[i] && s[i] != ' ') { if (o + 1U < cap) out[o++] = s[i]; i++; }
    out[o] = 0;
    return i;
}

/* Forward decl: the console `sts users` harness drives the real HTTP users
 * handler (bearer admin gate + pagination) through a synthesized TLS request. */
static u32 http_build_sts_response(char *out, u32 max, const u8 *req, u32 req_len, bool via_tls);

static void http_exec_sts_command(char *out, u32 *len_ptr, u32 max, const char *args)
{
    u32 len = *len_ptr;
    char sub[16];
    u32 i = sts_next_arg(args, 0, sub, sizeof(sub));

    if (sub[0] == 0 || http_streq(sub, "status")) {
        struct sts_user_public snap[STS_MAX_USERS];
        u32 n = sts_list(snap, STS_MAX_USERS);
        http_append(out, &len, max, "sts status secret=");
        http_append(out, &len, max, sts_has_secret() ? "yes" : "no");
        http_append(out, &len, max, " users=");
        http_append_u64(out, &len, max, n);
        http_append(out, &len, max, " utc=");
        http_append(out, &len, max, timer_utc_ms() ? "set" : "unset");
        http_append(out, &len, max, " rng=");
        http_append(out, &len, max, crypto_random_status());
        http_append(out, &len, max, "\n");
    }
#if STS_TEST_PROVISION
    else if (http_streq(sub, "testusers")) {
        u8 salt_a[STS_SALT_LEN], salt_u[STS_SALT_LEN];
        for (u32 k = 0; k < STS_SALT_LEN; k++) { salt_a[k] = (u8)(0xA0U + k); salt_u[k] = (u8)(0x50U + k); }
        i32 r1 = sts_upsert_user("admin1", "pw-admin-123", salt_a,
                                 (u16)(1U << 0), (u16)((1U<<0)|(1U<<1)|(1U<<2)), STS_FLAG_ADMIN);
        i32 r2 = sts_upsert_user("user1", "pw-user-123", salt_u,
                                 (u16)(1U << 0), (u16)(1U << 1), 0);
        http_append(out, &len, max, "sts testusers admin1=");
        http_append(out, &len, max, sts_err_name(r1));
        http_append(out, &len, max, " user1=");
        http_append(out, &len, max, sts_err_name(r2));
        http_append(out, &len, max, "\n");
    } else if (http_streq(sub, "testsecret")) {
        u8 secret[32];
        for (u32 k = 0; k < 32; k++) secret[k] = (u8)(0x40U + k);
        bool ok = sts_provision_secret(secret, sizeof(secret));
        if (timer_utc_ms() == 0U) (void)timer_set_utc_ms(1700000000000ULL);
        http_append(out, &len, max, ok ? "sts testsecret ok\n" : "sts testsecret FAIL\n");
    }
#endif
    else if (http_streq(sub, "gensecret")) {
        u8 secret[32];
        if (!crypto_random_bytes(secret, sizeof(secret))) {
            http_append(out, &len, max, "sts gensecret FAIL rng=");
            http_append(out, &len, max, crypto_random_status());
            http_append(out, &len, max, "\n");
        } else {
            bool ok = sts_provision_secret(secret, sizeof(secret));
            for (u32 k = 0; k < sizeof(secret); k++) secret[k] = 0;
            http_append(out, &len, max, ok ? "sts gensecret ok\n" : "sts gensecret FAIL provision\n");
        }
    } else if (http_streq(sub, "login")) {
        char user[64], pass[128], aud[64], tenant[64], scope_s[160];
        i = sts_next_arg(args, i, user, sizeof(user));
        i = sts_next_arg(args, i, pass, sizeof(pass));
        i = sts_next_arg(args, i, aud, sizeof(aud));
        i = sts_next_arg(args, i, tenant, sizeof(tenant));
        i = sts_next_arg(args, i, scope_s, sizeof(scope_s));
        if (tenant[0] == 0) { tenant[0] = 'd'; tenant[1] = 'e'; tenant[2] = 'm'; tenant[3] = 'o'; tenant[4] = 0; }
        u16 req_scope = 0;
        bool scope_ok = true;
        if (scope_s[0]) {
            u32 j = 0;
            while (scope_s[j]) {
                while (scope_s[j] == '+' || scope_s[j] == ',') j++;
                u32 st = j;
                while (scope_s[j] && scope_s[j] != '+' && scope_s[j] != ',') j++;
                if (j > st) {
                    i32 si = sts_scope_index(&scope_s[st], j - st);
                    if (si < 0) { scope_ok = false; break; }
                    req_scope |= (u16)(1U << (u32)si);
                }
            }
        }
        char scope_out[256];
        g_sts_test_token_len = 0;
        i32 rc = scope_ok
            ? sts_login(user, pass, aud, tenant, req_scope, 0,
                        g_sts_test_token, sizeof(g_sts_test_token), &g_sts_test_token_len,
                        scope_out, sizeof(scope_out))
            : STS_ERR_SCOPE;
        for (u32 k = 0; k < sizeof(pass); k++) pass[k] = 0;
        if (rc == STS_OK) {
            http_append(out, &len, max, "sts login ok scope=");
            http_append(out, &len, max, scope_out);
            http_append(out, &len, max, " tokenlen=");
            http_append_u64(out, &len, max, g_sts_test_token_len);
            http_append(out, &len, max, "\n");
        } else {
            http_append(out, &len, max, "sts login FAIL err=");
            http_append(out, &len, max, sts_err_name(rc));
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(sub, "validate") || http_streq(sub, "tamper") || http_streq(sub, "authz")) {
        bool is_authz = http_streq(sub, "authz");
        char need_scope[32];
        need_scope[0] = 0;
        if (is_authz) i = sts_next_arg(args, i, need_scope, sizeof(need_scope));
        char aud[64];
        i = sts_next_arg(args, i, aud, sizeof(aud));
        if (g_sts_test_token_len == 0) {
            http_append(out, &len, max, "sts ");
            http_append(out, &len, max, sub);
            http_append(out, &len, max, " FAIL no_token\n");
        } else {
            char token[STS_TOKEN_MAX];
            u32 tl = g_sts_test_token_len;
            for (u32 k = 0; k < tl; k++) token[k] = g_sts_test_token[k];
            token[tl] = 0;
            if (http_streq(sub, "tamper")) {
                /* Flip the first signature character. Mutating the final
                 * base64url character can change only unused padding bits and
                 * decode to the same signature bytes. */
                u32 sig = 0;
                for (u32 k = 0; k < tl; k++)
                    if (token[k] == '.')
                        sig = k + 1U;
                if (sig < tl)
                    token[sig] = (token[sig] == 'A') ? 'B' : 'A';
            }
            char tenant[64];
            u16 scope_mask = 0;
            i32 rc = sts_validate(token, tl, aud, tenant, sizeof(tenant), &scope_mask);
            if (http_streq(sub, "tamper")) {
                http_append(out, &len, max, (rc == STS_OK) ? "sts tamper VALID(BAD)\n" : "sts tamper invalid(ok)\n");
            } else if (is_authz) {
                i32 si = sts_scope_index(need_scope, pios_strlen(need_scope));
                bool allow = (rc == STS_OK) && (si >= 0) && (scope_mask & (u16)(1U << (u32)si));
                http_append(out, &len, max, allow ? "sts authz allow scope=" : "sts authz deny scope=");
                http_append(out, &len, max, need_scope);
                if (rc != STS_OK) { http_append(out, &len, max, " err="); http_append(out, &len, max, sts_err_name(rc)); }
                http_append(out, &len, max, "\n");
            } else {
                if (rc == STS_OK) {
                    char scope_str[256];
                    (void)sts_scope_mask_to_string(scope_mask, scope_str, sizeof(scope_str));
                    http_append(out, &len, max, "sts validate valid tenant=");
                    http_append(out, &len, max, tenant);
                    http_append(out, &len, max, " scope=");
                    http_append(out, &len, max, scope_str);
                    http_append(out, &len, max, "\n");
                } else {
                    http_append(out, &len, max, "sts validate invalid err=");
                    http_append(out, &len, max, sts_err_name(rc));
                    http_append(out, &len, max, "\n");
                }
            }
        }
    } else if (http_streq(sub, "users")) {
        /* Deterministic coverage for the /api/sts/users bearer-admin gate and
         * pagination: synthesize a TLS GET carrying the last-issued test token
         * as the Authorization bearer, then drive the REAL HTTP handler. */
        char off_s[12], lim_s[12];
        i = sts_next_arg(args, i, off_s, sizeof(off_s));
        i = sts_next_arg(args, i, lim_s, sizeof(lim_s));
        if (g_sts_test_token_len == 0) {
            http_append(out, &len, max, "sts users FAIL no_token\n");
        } else {
            static char req[STS_TOKEN_MAX + 160];
            static char resp[1280];
            u32 rq = 0;
            http_append(req, &rq, sizeof(req), "GET /api/sts/users");
            if (off_s[0] || lim_s[0]) {
                http_append(req, &rq, sizeof(req), "?offset=");
                http_append(req, &rq, sizeof(req), off_s[0] ? off_s : "0");
                http_append(req, &rq, sizeof(req), "&limit=");
                http_append(req, &rq, sizeof(req), lim_s[0] ? lim_s : "0");
            }
            http_append(req, &rq, sizeof(req), " HTTP/1.0\r\nAuthorization: Bearer ");
            http_append_bytes(req, &rq, sizeof(req), (const u8 *)g_sts_test_token, g_sts_test_token_len);
            http_append(req, &rq, sizeof(req), "\r\n\r\n");
            u32 rl = http_build_sts_response(resp, sizeof(resp), (const u8 *)req, rq, true);
            http_append(out, &len, max, "sts users bytes=");
            http_append_u64(out, &len, max, rl);
            http_append(out, &len, max, "\n");
            http_append_bytes(out, &len, max, (const u8 *)resp, rl);
            http_append(out, &len, max, "\n");
        }
    } else {
        http_append(out, &len, max,
            "usage: sts status | sts login <user> <pass> <aud> <tenant> [scope+scope] | "
            "sts validate <aud> | sts tamper <aud> | sts authz <scope> <aud> | "
            "sts users [offset] [limit] | sts gensecret"
#if STS_TEST_PROVISION
            " | sts testusers | sts testsecret"
#endif
            "\n");
    }
    *len_ptr = len;
}

/* The single, shared implementation of every "terminal" command (~150+
 * commands: status/netstat/macbdiag/rp1 irq/stackdiag/processes/... -- the
 * full diagnostic surface). Both the HTTP /api/terminal handler and the
 * UART/F3 console (ui_console_exec()'s fallback, see below) dispatch into
 * this ONE function so the two front-ends never drift out of sync and
 * never duplicate command logic. Appends output to out/len/max using the
 * same http_append() idiom used throughout the codebase; callers own the
 * buffer and any header/prefix that precedes it. */
static void http_exec_terminal_command(char *out, u32 *len_ptr, u32 max, char *cmd)
{
    u32 len = *len_ptr;
    if (http_streq(cmd, "help")) {
        http_append(out, &len, max,
            "PIOS terminal help\n"
            "Run commands exactly as shown; category names are help topics, not command prefixes.\n"
            "Examples: status | ps | netstat | ls / | firewall list | addr wal:0/3 | bootctrl status | reboot confirm\n"
            "Diagnostics: walfs verify | walfs compact | watchdog | crypto selftest | arp probe | nic dump on | nic counters | picocompress selftest | picoweb selftest\n"
            "Command help: help status | help netstat | help firewall | help reboot | help peek | help walfs | help cachestats\n"
            "Category help on UART/TCP console: help core | help fs | help net | help svc | help dev\n");
    } else if (http_starts_with(cmd, "help ")) {
        const char *topic = cmd + 5;
        if (http_streq(topic, "status")) {
            http_append(out, &len, max, "status\n  Show system/build/network summary.\n");
        } else if (http_streq(topic, "ps") || http_streq(topic, "processes")) {
            http_append(out, &len, max, "processes\n  Show process snapshot with PPID, arena/span telemetry, and process graph roots/children.\n");
        } else if (http_streq(topic, "netstat")) {
            http_append(out, &len, max, "netstat\n  Show live TCP listeners/sessions, owners, buffers, retries, and firewall drops.\n");
        } else if (http_streq(topic, "rxdiag")) {
            http_append(out, &len, max, "rxdiag\n  Correlate MAC DMA drain, NIC filtering, NET dispatch, poll cadence, and Ethernet IRQ handoff.\n");
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
            http_append(out, &len, max, "ls [absolute-path] | fsinspect [absolute-path] | walfs status | walfs verify | walfs compact | walfs format confirm\n  WALFS listing/status, integrity verify, non-destructive compact, plus confirmed reserved-base format.\n");
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
        } else if (http_streq(topic, "picocompress")) {
            http_append(out, &len, max, "picocompress selftest\n  Verify upstream-compatible 508-byte block compression/decompression and encoder stats.\n");
        } else if (http_streq(topic, "picoweb")) {
            http_append(out, &len, max, "picoweb selftest\n  Verify upstream-compatible method+pattern route dispatch, route params, headers, and status text.\n");
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
        } else if (http_streq(topic, "walfs") || http_streq(topic, "disk")) {
            http_append(out, &len, max, "walfs verify | walfs compact | walfs status | walfs format confirm\n  Verify WAL metadata/record-chain integrity, compact the WAL (non-destructive), or status.\n");
        } else if (http_streq(topic, "crypto")) {
            http_append(out, &len, max, "crypto selftest\n  Run AES-GCM + nibble-table GHASH crypto selftest.\n");
        } else if (http_streq(topic, "cachestats")) {
            http_append(out, &len, max, "cachestats\n  Show WAL inode/path, DNS, and ARP LRU cache hit/miss/evict telemetry.\n");
        } else if (http_streq(topic, "watchdog")) {
            http_append(out, &len, max, "watchdog | watchdog status\n  Show watchdog armed/mode/timeout, trip count, last trip core, and hw remaining ticks.\n");
        } else if (http_streq(topic, "arp")) {
            http_append(out, &len, max, "arp probe\n  Send a gratuitous ARP (TX-path test) and report request/learn/conflict counters.\n");
        } else if (http_streq(topic, "nic")) {
            http_append(out, &len, max, "nic dump <on|off> | nic counters | nic offload\n  Toggle raw packet dump or show counters/offload capability and checksum telemetry.\n");
        } else {
            http_append(out, &len, max, "ERR: unknown help topic. Try help status, help netstat, help firewall, help reboot, help dma, help tls, help brotli, help walfs, help cachestats\n");
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
    } else if (http_streq(cmd, "picocompress") || http_streq(cmd, "picocompress selftest")) {
        http_append(out, &len, max, pc_selftest() ? "Picocompress selftest OK\n" : "Picocompress selftest FAILED\n");
    } else if (http_streq(cmd, "picoweb") || http_streq(cmd, "picoweb selftest")) {
        http_append(out, &len, max, picoweb_selftest() ? "PicoWeb selftest OK\n" : "PicoWeb selftest FAILED\n");
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
    } else if (http_streq(cmd, "cpuclock") || http_streq(cmd, "cpu clock")) {
        u64 frq;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frq));
        u64 hz = cpu_clock_estimate_hz(30000U);
        u64 pmu_hz = cpu_clock_pmu_hz(20000U);
        if (pmu_hz)
            g_cpu_clock_mhz = (u32)((pmu_hz + 500000ULL) / 1000000ULL);
        http_append(out, &len, max, "cpuclock core=");
        http_append_u64(out, &len, max, core_id() & 3U);
        http_append(out, &len, max, " pmu_khz=");
        http_append_u64(out, &len, max, (pmu_hz + 500ULL) / 1000ULL);
        http_append(out, &len, max, " eff_khz=");
        http_append_u64(out, &len, max, (hz + 500ULL) / 1000ULL);
        http_append(out, &len, max, " timer_mhz=");
        http_append_u64(out, &len, max, frq / 1000000ULL);
        http_append(out, &len, max, " penalty_x=");
        http_append_u64(out, &len, max, hz ? (pmu_hz / hz) : 0ULL);
        http_append(out, &len, max, " (pmu=clock eff=throughput gap=nc-exec)\n");
    } else if (http_streq(cmd, "mmustat")) {
        u64 sctlr, ttbr0;
        __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
        __asm__ volatile("mrs %0, ttbr0_el1" : "=r"(ttbr0));
        http_append(out, &len, max, "mmustat sctlr=");
        http_append_hex32(out, &len, max, (u32)sctlr);
        http_append(out, &len, max, " M=");
        http_append_u64(out, &len, max, sctlr & 1U);
        http_append(out, &len, max, " C=");
        http_append_u64(out, &len, max, (sctlr >> 2) & 1U);
        http_append(out, &len, max, " I=");
        http_append_u64(out, &len, max, (sctlr >> 12) & 1U);
        http_append(out, &len, max, " ttbr0=");
        http_append_hex32(out, &len, max, (u32)ttbr0);
        http_append(out, &len, max, "\n");
        /* AT S1E1R translate a few VAs; PAR_EL1[63:56]=memattr (0xFF=WB,
         * 0x44=Normal-NC, 0x00=Device), bit0=fault. */
        u64 probe[3];
        probe[0] = (u64)(usize)&cpu_clock_estimate_hz; /* kernel .text */
        u64 stackv = 0; probe[1] = (u64)(usize)&stackv; /* core-0 stack */
        probe[2] = 0x0059f000ULL;                       /* tensor .bss pool */
        const char *pn[3] = { "text", "stack", "bss_pool" };
        for (u32 i = 0; i < 3; i++) {
            u64 par;
            __asm__ volatile("at s1e1r, %1\n isb\n mrs %0, par_el1"
                             : "=r"(par) : "r"(probe[i]) : "memory");
            http_append(out, &len, max, pn[i]);
            http_append(out, &len, max, " va=");
            http_append_hex32(out, &len, max, (u32)probe[i]);
            http_append(out, &len, max, " par=");
            http_append_hex32(out, &len, max, (u32)(par >> 32));
            http_append(out, &len, max, ":");
            http_append_hex32(out, &len, max, (u32)par);
            http_append(out, &len, max, (par & 1U) ? " FAULT" : "");
            http_append(out, &len, max, " attr=");
            http_append_hex32(out, &len, max, (u32)((par >> 56) & 0xFFU));
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "cacheregs")) {
        /* Architectural cache identification (all EL1-readable). CCSIDR per level
         * via CSSELR. CPUECTLR/CPUACTLR are A76 IMP-DEF; EL1 access is enabled by
         * Pi5 TF-A in practice. If a read faults the recoverable PiSOD will show
         * it and this command can be trimmed. */
        u64 ctr, clidr, midr, sctlr, l1d, l1i, l2;
        __asm__ volatile("mrs %0, midr_el1"  : "=r"(midr));
        __asm__ volatile("mrs %0, ctr_el0"   : "=r"(ctr));
        __asm__ volatile("mrs %0, clidr_el1" : "=r"(clidr));
        __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
        __asm__ volatile("msr csselr_el1, %1\n isb\n mrs %0, ccsidr_el1"
                         : "=r"(l1d) : "r"(0UL) : "memory"); /* L1D: level0,data  */
        __asm__ volatile("msr csselr_el1, %1\n isb\n mrs %0, ccsidr_el1"
                         : "=r"(l1i) : "r"(1UL) : "memory"); /* L1I: level0,instr */
        __asm__ volatile("msr csselr_el1, %1\n isb\n mrs %0, ccsidr_el1"
                         : "=r"(l2)  : "r"(2UL) : "memory"); /* L2:  level1,data  */
        u64 cpuectlr = 0, cpuactlr = 0;
        __asm__ volatile("mrs %0, s3_0_c15_c1_4" : "=r"(cpuectlr)); /* CPUECTLR */
        __asm__ volatile("mrs %0, s3_0_c15_c1_0" : "=r"(cpuactlr)); /* CPUACTLR */
        http_append(out, &len, max, "cacheregs midr=");
        http_append_hex32(out, &len, max, (u32)midr);
        http_append(out, &len, max, " ctr=");
        http_append_hex32(out, &len, max, (u32)ctr);
        http_append(out, &len, max, " clidr=");
        http_append_hex32(out, &len, max, (u32)clidr);
        http_append(out, &len, max, " sctlr=");
        http_append_hex32(out, &len, max, (u32)sctlr);
        http_append(out, &len, max, "\n L1D_ccsidr=");
        http_append_hex32(out, &len, max, (u32)l1d);
        http_append(out, &len, max, " L1I_ccsidr=");
        http_append_hex32(out, &len, max, (u32)l1i);
        http_append(out, &len, max, " L2_ccsidr=");
        http_append_hex32(out, &len, max, (u32)l2);
        http_append(out, &len, max, "\n CPUECTLR=");
        http_append_hex32(out, &len, max, (u32)(cpuectlr >> 32));
        http_append(out, &len, max, ":");
        http_append_hex32(out, &len, max, (u32)cpuectlr);
        http_append(out, &len, max, " CPUACTLR=");
        http_append_hex32(out, &len, max, (u32)(cpuactlr >> 32));
        http_append(out, &len, max, ":");
        http_append_hex32(out, &len, max, (u32)cpuactlr);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "lease")) {
        /* P0: prove the lease fabric (lifecycle, poison, MMU grant/revoke,
         * zero-copy handle, copy-once, TLS-direct-write) runs live. */
        bool ok = lease_selftest();
        http_append(out, &len, max, ok ? "lease selftest PASS\n" : "lease selftest FAIL\n");
        struct lease_stats st;
        lease_get_stats(&st);
        http_append(out, &len, max, "lease pool slots=");
        http_append_u64(out, &len, max, st.slots_total);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "rxholedump")) {
#if !PIOS_HAS_GENET
        http_append(out, &len, max,
            "rxholedump unavailable: macb/GEM not active on this platform\n");
#else
        struct macb_hole_snapshot hs;
        if (!macb_rx_hole_snapshot(&hs)) {
            http_append(out, &len, max, "rxholedump none\n");
        } else {
            http_append(out, &len, max, "rxholedump seq=");
            http_append_u64(out, &len, max, hs.sequence);
            http_append(out, &len, max, " stuck=");
            http_append_u64(out, &len, max, hs.stuck_idx);
            http_append(out, &len, max, " RBQP=");
            http_append_hex32(out, &len, max, hs.rbqp);
            http_append(out, &len, max, " RBQPH=");
            http_append_hex32(out, &len, max, hs.rbqph);
            http_append(out, &len, max, " RSR=");
            http_append_hex32(out, &len, max, hs.rsr);
            http_append(out, &len, max, " NCR=");
            http_append_hex32(out, &len, max, hs.ncr);
            http_append(out, &len, max, " RBQP_STOP=");
            http_append_hex32(out, &len, max, hs.rbqp_stopped);
            http_append(out, &len, max, "\nmode DMACFG=");
            http_append_hex32(out, &len, max, hs.dmacfg);
            http_append(out, &len, max, " DCFG10=");
            http_append_hex32(out, &len, max, hs.dcfg10);
            http_append(out, &len, max, " RXBDCTRL=");
            http_append_hex32(out, &len, max, hs.rxbdctrl);
            http_append(out, &len, max, " prefetch_desc=");
            http_append_u64(out, &len, max, hs.prefetch_descs);
            http_append(out, &len, max, " trailing=");
            http_append_u64(out, &len, max, hs.trailing_bytes);
            http_append(out, &len, max, "\ncache line=");
            http_append_hex32(out, &len, max, hs.cache_line);
            http_append(out, &len, max, " flags=");
            http_append_hex32(out, &len, max, hs.cache_probe_flags);
            http_append(out, &len, max, "\nstuck idx=");
            http_append_u64(out, &len, max, hs.stuck.idx);
            http_append(out, &len, max, " expected=");
            http_append_hex32(out, &len, max, hs.expected_addr);
            http_append(out, &len, max, " words=");
            http_append_hex32(out, &len, max, hs.stuck.addr);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.stuck.ctrl);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.stuck.addr_hi);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.stuck.word3);
            http_append(out, &len, max, "\nstopped words=");
            http_append_hex32(out, &len, max, hs.stopped.addr);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.stopped.ctrl);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.stopped.addr_hi);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.stopped.word3);
            http_append(out, &len, max, "\nafter_ivac words=");
            http_append_hex32(out, &len, max, hs.after_ivac.addr);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.after_ivac.ctrl);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.after_ivac.addr_hi);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, hs.after_ivac.word3);
            http_append(out, &len, max, "\n");
            for (u32 i = 0; i < hs.follow_count; i++) {
                const struct macb_desc_snapshot *d = &hs.follow[i];
                http_append(out, &len, max, "owned idx=");
                http_append_u64(out, &len, max, d->idx);
                http_append(out, &len, max, " words=");
                http_append_hex32(out, &len, max, d->addr);
                http_append(out, &len, max, "/");
                http_append_hex32(out, &len, max, d->ctrl);
                http_append(out, &len, max, "/");
                http_append_hex32(out, &len, max, d->addr_hi);
                http_append(out, &len, max, "/");
                http_append_hex32(out, &len, max, d->word3);
                http_append(out, &len, max, "\n");
            }
        }
#endif
    } else if (http_streq(cmd, "macbdiag")) {
        /* P1 live wedge diagnosis: poll this while ramping load. If rx_owned
         * climbs to 32 the RX ring is full (overrun); if rx_recv/tx_send stop
         * climbing the MAC engine has stalled; RSR/TSR carry overrun/HRESP. */
#if !PIOS_HAS_GENET
        /* macb is not the active NIC on this platform (e.g. QEMU uses
         * virtio-net), so macb_init() never ran — its RX ring pointer/size are
         * uninitialised and scanning them would wedge core 0. Fail closed and
         * point at the right command instead. */
        http_append(out, &len, max,
            "macbdiag unavailable: macb/GEM not active on this platform (use 'vnetdiag')\n");
#else
        struct macb_diag md;
        macb_diag(&md);
        http_append(out, &len, max, "macbdiag rx_owned=");
        http_append_u64(out, &len, max, md.rx_owned);
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, md.ring_size);
        http_append(out, &len, max, " contig=");
        http_append_u64(out, &len, max, md.rx_contig_owned);
        http_append(out, &len, max, " after_gap=");
        http_append_u64(out, &len, max, md.rx_owned_after_gap);
        http_append(out, &len, max, " first_after=");
        http_append_u64(out, &len, max, md.rx_first_owned_distance);
        http_append(out, &len, max, " rx_idx=");
        http_append_u64(out, &len, max, md.rx_idx);
        http_append(out, &len, max, " tx_idx=");
        http_append_u64(out, &len, max, md.tx_idx);
        http_append(out, &len, max, " rx_recv=");
        http_append_u64(out, &len, max, md.rx_recv);
        http_append(out, &len, max, " tx_send=");
        http_append_u64(out, &len, max, md.tx_send);
        http_append(out, &len, max, " NSR=");
        http_append_hex32(out, &len, max, md.nsr);
        http_append(out, &len, max, " RSR=");
        http_append_hex32(out, &len, max, md.rsr);
        http_append(out, &len, max, " TSR=");
        http_append_hex32(out, &len, max, md.tsr);
        http_append(out, &len, max, " RBQP=");
        http_append_hex32(out, &len, max, md.rbqp);
        http_append(out, &len, max, " TBQP=");
        http_append_hex32(out, &len, max, md.tbqp);
        http_append(out, &len, max, " ETH_CFG_STAT=");
        http_append_hex32(out, &len, max, md.eth_cfg_stat);
        http_append(out, &len, max, " rx_recover=");
        http_append_u64(out, &len, max, md.rx_recover);
        http_append(out, &len, max, " tx_drop=");
        http_append_u64(out, &len, max, md.tx_drop);
        http_append(out, &len, max, " tx_recover=");
        http_append_u64(out, &len, max, md.tx_recover);
        http_append(out, &len, max, " rx_live_recover=");
        http_append_u64(out, &len, max, md.rx_live_recover);
        http_append(out, &len, max, " rx_hole_recover=");
        http_append_u64(out, &len, max, md.rx_hole_recover);
        http_append(out, &len, max, " rx_wedge=");
        http_append_u64(out, &len, max, md.rx_wedge);
        http_append(out, &len, max, " rx_idle=");
        http_append_u64(out, &len, max, md.rx_idle);
        http_append(out, &len, max, " tx_pause=");
        http_append_u64(out, &len, max, md.tx_pause);
        http_append(out, &len, max, " rx_pause=");
        http_append_u64(out, &len, max, md.rx_pause);
        http_append(out, &len, max, "\n");
#endif
    } else if (http_starts_with(cmd, "dtrace")) {
        /* In-memory diagnostic trace control + dump.
         *   dtrace            -> status
         *   dtrace on|off     -> enable/disable capture
         *   dtrace clear      -> reset rings
         *   dtrace mask <hex> -> set category bitmask (0=all)
         *   dtrace dump [n]   -> dump n most-recent records (default 256) */
        const char *arg = cmd + 6;
        while (*arg == ' ') arg++;
        if (http_starts_with(arg, "on")) {
            dtrace_set_enabled(true);
            len += dtrace_status(out + len, max - len);
        } else if (http_starts_with(arg, "off")) {
            dtrace_set_enabled(false);
            len += dtrace_status(out + len, max - len);
        } else if (http_starts_with(arg, "clear")) {
            dtrace_clear();
            http_append(out, &len, max, "dtrace cleared\n");
        } else if (http_starts_with(arg, "mask")) {
            const char *h = arg + 4;
            while (*h == ' ') h++;
            u32 m = 0;
            if (h[0] == '0' && (h[1] == 'x' || h[1] == 'X')) h += 2;
            while ((*h >= '0' && *h <= '9') || (*h >= 'a' && *h <= 'f') || (*h >= 'A' && *h <= 'F')) {
                u32 d = (*h <= '9') ? (u32)(*h - '0') : ((*h | 0x20) - 'a' + 10U);
                m = (m << 4) | d; h++;
            }
            dtrace_set_mask(m);
            len += dtrace_status(out + len, max - len);
        } else if (http_starts_with(arg, "dump")) {
            const char *n = arg + 4;
            while (*n == ' ') n++;
            u32 cnt = 0;
            while (*n >= '0' && *n <= '9') { cnt = cnt * 10U + (u32)(*n - '0'); n++; }
            if (cnt == 0) cnt = 256;
            len += dtrace_status(out + len, max - len);
            len += dtrace_dump(out + len, max - len, cnt);
        } else {
            len += dtrace_status(out + len, max - len);
        }
    } else if (http_streq(cmd, "schedquanta") || http_streq(cmd, "starvation")) {
        /* Scheduling quantum + per-core CPU/starvation snapshot. The quantum is
         * the preemption budget; busy_permille shows per-core CPU utilisation,
         * and a core pinned near 1000 (100%) with low wake_count is starving. */
        http_append(out, &len, max, "quantum_ms=");
        http_append_u64(out, &len, max, PROC_PREEMPT_QUANTUM_MS);
        http_append(out, &len, max, " timer_hz=");
        http_append_u64(out, &len, max, PROC_PREEMPT_TIMER_HZ);
        http_append(out, &len, max, " quantum_ticks=");
        http_append_u64(out, &len, max,
            ((u64)PROC_PREEMPT_TIMER_HZ * PROC_PREEMPT_QUANTUM_MS + 999U) / 1000U);
        http_append(out, &len, max, "\nCORE BUSY%o IDLE WAKE PREEMPT SOFT STARVING\n");
        struct proc_sched_core_snapshot ss[NUM_CORES];
        u32 n = proc_sched_snapshot(ss, NUM_CORES);
        for (u32 i = 0; i < n; i++) {
            http_append_u64(out, &len, max, ss[i].core);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ss[i].busy_permille);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ss[i].idle_count);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ss[i].wake_count);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ss[i].preemptions);
            http_append(out, &len, max, " ");
            http_append_u64(out, &len, max, ss[i].soft_events);
            http_append(out, &len, max, " ");
            /* Starving = pinned busy with almost no wakes (stuck in a phase). */
            http_append(out, &len, max,
                (ss[i].busy_permille >= 950U && ss[i].wake_count < 4U) ? "YES" : "no");
            http_append(out, &len, max, "\n");
        }
    } else if (http_starts_with(cmd, "cdump")) {
        /* Live core-dump snapshot + compare (distinct from the crash 'coredump'):
         *   cdump snap a|b   -> capture CPU/system regs + region CRCs into a slot
         *   cdump show a|b   -> print a slot
         *   cdump diff       -> list reg/region differences between A and B */
        const char *arg = cmd + 5;
        while (*arg == ' ') arg++;
        if (http_starts_with(arg, "snap")) {
            const char *s = arg + 4; while (*s == ' ') s++;
            u32 slot = (*s == 'b' || *s == 'B') ? COREDUMP_SLOT_B : COREDUMP_SLOT_A;
            coredump_take(slot);
            http_append(out, &len, max, "captured slot ");
            if (len < max) out[len++] = (char)('A' + slot);
            http_append(out, &len, max, "\n");
        } else if (http_starts_with(arg, "show")) {
            const char *s = arg + 4; while (*s == ' ') s++;
            u32 slot = (*s == 'b' || *s == 'B') ? COREDUMP_SLOT_B : COREDUMP_SLOT_A;
            len += coredump_format(slot, out + len, max - len);
        } else if (http_starts_with(arg, "diff")) {
            len += coredump_diff(out + len, max - len);
        } else {
            http_append(out, &len, max, "usage: cdump snap a|b | show a|b | diff\n");
        }
    } else if (http_starts_with(cmd, "selftest")) {
        /* Unified self-test battery: run the QEMU-safe pure in-kernel selftests
         * and report a PASS/FAIL summary. Used by the QEMU smoke harness as a
         * pre-deploy gate. Hardware-dependent tests (dma, v3d/tensor variants,
         * and the cert tests x509/acme which need on-device entropy/time) are
         * excluded here — they have their own on-hardware commands. NOTE: the
         * tensor V3D variants (tensor_vector16/etc) can HANG with no NIC/V3D on
         * QEMU, so they must never be in this QEMU-facing battery. */
        struct { const char *name; bool (*fn)(void); } bat[] = {
            { "crypto",      crypto_selftest },
            { "tls",         tls_selftest },
            { "brotli",      brotli_selftest },
            { "picocompress",pc_selftest },
            { "picoweb",     picoweb_selftest },
            { "lease",       lease_selftest },
            { "abi",         abi_selftest },
            { "irq_diag",    irq_diag_selftest },
            { "proc_svc",    proc_svc_selftest },
            { "proc_entry",  proc_entry_contract_selftest },
            { "tensor_neon", tensor_selftest },
            { "el2_stage2",  el2_stage2_selftest },
            { "sts",         sts_selftest },
        };
        u32 nt = (u32)(sizeof(bat) / sizeof(bat[0]));
        u32 passed = 0;
        for (u32 i = 0; i < nt; i++) {
            bool ok = bat[i].fn ? bat[i].fn() : false;
            if (ok) passed++;
            http_append(out, &len, max, ok ? "PASS " : "FAIL ");
            http_append(out, &len, max, bat[i].name);
            http_append(out, &len, max, "\n");
        }
        http_append(out, &len, max, "selftest summary ");
        http_append_u64(out, &len, max, passed);
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, nt);
        http_append(out, &len, max, (passed == nt) ? " ALL PASS\n" : " SOME FAILED\n");
    } else if (http_streq(cmd, "vnetdiag")) {
        /* virtio-net TX/RX drop visibility (QEMU). tx_drop>0 means the TX ring
         * filled and a segment was lost -> peer retransmit (~1s latency stall);
         * rx_starve>0 means net_poll fell behind and the device ring filled. */
        u64 tx_ok = 0, tx_drop = 0, rx_ok = 0, rx_starve = 0;
        virtio_net_counters(&tx_ok, &tx_drop, &rx_ok, &rx_starve);
        http_append(out, &len, max, "vnetdiag tx_ok=");
        http_append_u64(out, &len, max, tx_ok);
        http_append(out, &len, max, " tx_drop=");
        http_append_u64(out, &len, max, tx_drop);
        http_append(out, &len, max, " rx_ok=");
        http_append_u64(out, &len, max, rx_ok);
        http_append(out, &len, max, " rx_starve=");
        http_append_u64(out, &len, max, rx_starve);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "stackdiag")) {
        /* Data-plane health across every layer that carries constant traffic:
         * IP stack, TCP, kernel TLS, the port FIFO forwarding bridges, and the
         * inter-core FIFOs. If any of these "falls over" under load this is the
         * single place to see it. Read-only; safe to poll while ramping load. */
        const net_stats_t *ns = net_get_stats();
        const tcp_diag_t *td = tcp_diag();
        u32 tcap = 0, tinuse = 0, tlisten = 0;
        tcp_table_stats(&tcap, &tinuse, &tlisten);
        struct tls_diag_snapshot ts;
        tls_diag_snapshot(&ts);

        http_append(out, &len, max, "IP   rx=");
        http_append_u64(out, &len, max, ns->rx_packets);
        http_append(out, &len, max, " tx=");
        http_append_u64(out, &len, max, ns->tx_packets);
        http_append(out, &len, max, " icmp=");
        http_append_u64(out, &len, max, ns->icmp_echo_replies);
        http_append(out, &len, max, " udp_rx=");
        http_append_u64(out, &len, max, ns->udp_recv);
        http_append(out, &len, max, " udp_tx=");
        http_append_u64(out, &len, max, ns->udp_sent);
        http_append(out, &len, max, " | drop runt=");
        http_append_u64(out, &len, max, ns->drop_runt);
        http_append(out, &len, max, " cksum=");
        http_append_u64(out, &len, max, ns->drop_bad_cksum);
        http_append(out, &len, max, " frag=");
        http_append_u64(out, &len, max, ns->drop_fragment);
        http_append(out, &len, max, " ipopt=");
        http_append_u64(out, &len, max, ns->drop_ip_options);
        http_append(out, &len, max, " badsrc=");
        http_append_u64(out, &len, max, ns->drop_bad_src);
        http_append(out, &len, max, " notforus=");
        http_append_u64(out, &len, max, ns->drop_not_for_us);
        http_append(out, &len, max, " badproto=");
        http_append_u64(out, &len, max, ns->drop_bad_proto);
        http_append(out, &len, max, " noneigh=");
        http_append_u64(out, &len, max, ns->drop_no_neighbor);
        http_append(out, &len, max, " oversz=");
        http_append_u64(out, &len, max, ns->drop_oversized);
        http_append(out, &len, max, " udpcksum=");
        http_append_u64(out, &len, max, ns->drop_udp_bad_cksum);
        http_append(out, &len, max, "\n");

        http_append(out, &len, max, "TCP  syn=");
        http_append_u64(out, &len, max, td->syn_seen);
        http_append(out, &len, max, " synack=");
        http_append_u64(out, &len, max, td->synack_sent);
        http_append(out, &len, max, " accept=");
        http_append_u64(out, &len, max, td->accepted);
        http_append(out, &len, max, " txseg=");
        http_append_u64(out, &len, max, td->tx_segments);
        http_append(out, &len, max, " txnomac=");
        http_append_u64(out, &len, max, td->tx_no_mac);
        http_append(out, &len, max, " txfail=");
        http_append_u64(out, &len, max, td->tx_send_fail);
        http_append(out, &len, max, " short=");
        http_append_u64(out, &len, max, td->in_short);
        http_append(out, &len, max, " badcsum=");
        http_append_u64(out, &len, max, td->bad_checksum);
        http_append(out, &len, max, " badhdr=");
        http_append_u64(out, &len, max, td->bad_header);
        http_append(out, &len, max, " nolisten=");
        http_append_u64(out, &len, max, td->no_listener);
        http_append(out, &len, max, " ackseen=");
        http_append_u64(out, &len, max, td->ack_cookie_seen);
        http_append(out, &len, max, " queued=");
        http_append_u64(out, &len, max, td->pending_queued);
        http_append(out, &len, max, " cookiebad=");
        http_append_u64(out, &len, max, td->ack_cookie_bad);
        http_append(out, &len, max, " pendfull=");
        http_append_u64(out, &len, max, td->pending_full);
        http_append(out, &len, max, " | table cap=");
        http_append_u64(out, &len, max, tcap);
        http_append(out, &len, max, " inuse=");
        http_append_u64(out, &len, max, tinuse);
        http_append(out, &len, max, " listen=");
        http_append_u64(out, &len, max, tlisten);
        http_append(out, &len, max, "\n");

        http_append(out, &len, max, "TLS  active=");
        http_append_u64(out, &len, max, ts.active);
        http_append(out, &len, max, " est=");
        http_append_u64(out, &len, max, ts.established);
        http_append(out, &len, max, " hs_ok=");
        http_append_u64(out, &len, max, ts.handshakes_ok);
        http_append(out, &len, max, " hs_fail=");
        http_append_u64(out, &len, max, ts.handshake_failures);
        http_append(out, &len, max, " rec_rx=");
        http_append_u64(out, &len, max, ts.records_rx);
        http_append(out, &len, max, " rec_tx=");
        http_append_u64(out, &len, max, ts.records_tx);
        http_append(out, &len, max, " decfail=");
        http_append_u64(out, &len, max, ts.decrypt_failures);
        http_append(out, &len, max, " closes=");
        http_append_u64(out, &len, max, ts.closes);
        http_append(out, &len, max, " br_ok=");
        http_append_u64(out, &len, max, ts.bridge_parse_ok);
        http_append(out, &len, max, " br_err=");
        http_append_u64(out, &len, max, ts.bridge_parse_error);
        http_append(out, &len, max, " last_err=");
        http_append_hex32(out, &len, max, ts.last_error);
        http_append(out, &len, max, "\n");

        for (u32 bi = 0; bi < UHTTP_BRIDGE_COUNT; bi++) {
            i32 lc = -1; u32 st = 0, rq = 0, rs = 0, reqs = 0, mg = 0, pid = 0;
            uhttp_bridge_state_idx(bi, &lc, &st, &rq, &rs, &reqs, &mg, &pid);
            http_append(out, &len, max, "FWD  bridge");
            http_append_u64(out, &len, max, bi);
            http_append(out, &len, max, " port=");
            http_append_u64(out, &len, max, bi == 0 ? UHTTP_PORT : UHTTP_NATIVE_PORT);
            http_append(out, &len, max, " state=");
            http_append_u64(out, &len, max, st);
            http_append(out, &len, max, " req=");
            http_append_u64(out, &len, max, rq);
            http_append(out, &len, max, " resp=");
            http_append_u64(out, &len, max, rs);
            http_append(out, &len, max, " lag=");
            http_append_u64(out, &len, max, (rq >= rs) ? (rq - rs) : 0);
            http_append(out, &len, max, " reqs=");
            http_append_u64(out, &len, max, reqs);
            http_append(out, &len, max, " pid=");
            http_append_u64(out, &len, max, pid);
            http_append(out, &len, max, mg == UHTTP_BRIDGE_MAGIC ? " magic=ok\n" : " magic=BAD\n");
        }

        /* Inter-core FIFO occupancy: find the deepest active ring (constant
         * cross-core traffic for disk/net/socket/IPC). A ring pinned near
         * FIFO_CAPACITY means a consumer core stopped draining. */
        u32 fmax = 0, fmd = 0, fms = 0;
        for (u32 d = 0; d < 4U; d++) {
            for (u32 s = 0; s < 4U; s++) {
                if (d == s) continue;
                u32 fc = fifo_count(d, s);
                if (fc > fmax) { fmax = fc; fmd = d; fms = s; }
            }
        }
        http_append(out, &len, max, "FIFO max_depth=");
        http_append_u64(out, &len, max, fmax);
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, FIFO_CAPACITY);
        http_append(out, &len, max, " (dst");
        http_append_u64(out, &len, max, fmd);
        http_append(out, &len, max, "<-src");
        http_append_u64(out, &len, max, fms);
        http_append(out, &len, max, ")\n");
    } else if (http_streq(cmd, "cachediag")) {
        /* milli-cycles per op = cyc*1000/ops, so 1000 == 1.0 cyc/op. If nop and
         * load are both ~hundreds-of-thousands (i.e. ~hundreds cyc/op) the caches
         * are not allocating (every access is a DRAM round-trip). */
        u64 r[5] = {0,0,0,0,0};
        cpu_pmu_microbench(r);
        http_append(out, &len, max, "cachediag spin/i=");
        http_append_u64(out, &len, max, (r[0] * 1000ULL) / 2000000ULL);
        http_append(out, &len, max, " nop/i=");
        http_append_u64(out, &len, max, (r[1] * 1000ULL) / 6400000ULL);
        http_append(out, &len, max, " | load_lo=");
        http_append_u64(out, &len, max, r[2] / 1000ULL);
        http_append(out, &len, max, " load_hi=");
        http_append_u64(out, &len, max, r[3] / 1000ULL);
        http_append(out, &len, max, " load_dram=");
        http_append_u64(out, &len, max, r[4] / 1000ULL);
        http_append(out, &len, max, " cyc/load (lo~=hi~=dram => caches dead; hi<<lo => remap bug)\n");
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
               http_streq(cmd, "rp1 irq arm-host6-1") || http_streq(cmd, "rp1 irq source-diag") ||
               http_starts_with(cmd, "rp1 irq storm")) {
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
        } else if (http_starts_with(cmd, "rp1 irq storm")) {
            /* Synthetic IRQ-storm harness: fire the same MIP INT_RAISE edge
             * used by "raise-eth" in a tight, bounded loop, entirely
             * independent of real MAC/network activity. Interrupts stay
             * unmasked across the loop (this handler doesn't mask them), so
             * each raise is serviced synchronously by core0_eth_irq_handler
             * before the next iteration -- the worst-case back-to-back
             * inter-arrival gap the real IRQ-arm race and the poll-fallback
             * backstop are meant to survive. Isolates "does pure IRQ rate
             * alone reproduce/stress the problem" from "does it need real RX
             * ring backlog too". Bounded to 100000 to keep this HTTP request
             * from hanging indefinitely on a misbehaving build. */
            const char *narg = cmd + 13;
            while (*narg == ' ') narg++;
            u32 n = 0;
            while (*narg >= '0' && *narg <= '9') { n = n * 10U + (u32)(*narg - '0'); narg++; }
            if (n == 0) n = 1000;
            if (n > 100000U) n = 100000U;
            u64 before = core0_eth_irq_count;
            u32 stall_before = core0_eth_irq_stall_streak;
            u32 fallback_before = core0_eth_irq_fallback_count;
            bool poll_fallback_before = core0_eth_irq_poll_fallback;
            for (u32 i = 0; i < n; i++)
                rp1_eth_irq_raise_test();
            http_append(out, &len, max, "rp1 eth irq storm n=");
            http_append_u64(out, &len, max, n);
            http_append(out, &len, max, " count_before=");
            http_append_u64(out, &len, max, before);
            http_append(out, &len, max, " count_after=");
            http_append_u64(out, &len, max, core0_eth_irq_count);
            http_append(out, &len, max, " stall_streak_before=");
            http_append_u64(out, &len, max, stall_before);
            http_append(out, &len, max, " stall_streak_after=");
            http_append_u64(out, &len, max, core0_eth_irq_stall_streak);
            http_append(out, &len, max, " poll_fallback_before=");
            http_append(out, &len, max, poll_fallback_before ? "1" : "0");
            http_append(out, &len, max, " poll_fallback_after=");
            http_append(out, &len, max, core0_eth_irq_poll_fallback ? "1" : "0");
            http_append(out, &len, max, " fallback_count_before=");
            http_append_u64(out, &len, max, fallback_before);
            http_append(out, &len, max, " fallback_count_after=");
            http_append_u64(out, &len, max, core0_eth_irq_fallback_count);
            http_append(out, &len, max, "\n");
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
            core0_eth_irq_arm_host(http_streq(cmd, "rp1 irq arm-host6-1"));
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
    } else if (http_starts_with(cmd, "sgi ")) {
        /* GIC SGI inter-core wake doorbell probe (Phase 1: prove delivery).
         *   sgi test <core0-3> [n]  fire n SGIs at target core, report recv delta
         *   sgi stat                report per-core delivery counters */
        char *argv[4];
        u32 argc = http_split_args(cmd, argv, 4);
        if (argc >= 2 && http_streq(argv[1], "test")) {
            u32 tgt = 1U, n = 1U;
            if (argc >= 3) (void)http_parse_u32(argv[2], &tgt);
            if (argc >= 4) (void)http_parse_u32(argv[3], &n);
            if (tgt > 3U) tgt = 3U;
            if (n == 0U) n = 1U;
            if (n > 100000U) n = 100000U;
            u32 before = proc_sgi_wake_count(tgt);
            for (u32 i = 0; i < n; i++) {
                gic_send_sgi((u8)(1U << tgt), GIC_SGI_WAKE);
                timer_delay_us(20);
            }
            timer_delay_us(1000);
            u32 after = proc_sgi_wake_count(tgt);
            http_append(out, &len, max, "sgi test core=");
            http_append_u64(out, &len, max, tgt);
            http_append(out, &len, max, " sent=");
            http_append_u64(out, &len, max, n);
            http_append(out, &len, max, " recv_before=");
            http_append_u64(out, &len, max, before);
            http_append(out, &len, max, " recv_after=");
            http_append_u64(out, &len, max, after);
            http_append(out, &len, max, " delta=");
            http_append_u64(out, &len, max, after - before);
            http_append(out, &len, max, "\n");
        } else if (argc >= 2 && http_streq(argv[1], "stat")) {
            http_append(out, &len, max, "sgi recv[0..3]=");
            for (u32 c = 0; c < 4U; c++) {
                http_append_u64(out, &len, max, proc_sgi_wake_count(c));
                http_append(out, &len, max, c < 3U ? "," : "\n");
            }
            http_append(out, &len, max, "fifo irq ready[0..3]=");
            for (u32 c = 0; c < 4U; c++) {
                http_append_u64(out, &len, max, fifo_irq_ready(c) ? 1U : 0U);
                http_append(out, &len, max, c < 3U ? "," : "\n");
            }
            http_append(out, &len, max, "fifo irq sent[0..3]=");
            for (u32 c = 0; c < 4U; c++) {
                http_append_u64(out, &len, max, fifo_irq_sent(c));
                http_append(out, &len, max, c < 3U ? "," : "\n");
            }
        } else {
            http_append(out, &len, max, "usage: sgi test <core0-3> [n] | sgi stat\n");
        }
    } else if (http_starts_with(cmd, "membench ")) {
        char *argv[3];
        u32 argc = http_split_args(cmd, argv, 3);
        u64 addr = 0;
        u32 count = 1000000U;
        if (argc >= 2 && http_parse_u64(argv[1], &addr) &&
            (argc < 3 || http_parse_u32(argv[2], &count))) {
            volatile u32 *p = (volatile u32 *)(usize)addr;
            u64 t0; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t0));
            for (u32 i = 0; i < count; i++)
                p[i & 1023U] = i;            /* 4KB window: cache-resident if cacheable */
            u64 t1; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t1));
            u32 acc = 0;
            u64 t2; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t2));
            for (u32 i = 0; i < count; i++)
                acc += p[i & 1023U];         /* read pass over the same window */
            u64 t3; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t3));
            u64 wdt = t1 - t0, rdt = t3 - t2;
            /* Pure-compute loop (no memory) to separate CPU clock from memory
             * latency: if this is also ~100ns/iter the CPU is slow-clocked. */
            u32 x = count | 1U;
            u64 t4; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t4));
            for (u32 i = 0; i < count; i++)
                x = x * 1664525U + 1013904223U;
            u64 t5; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t5));
            __asm__ volatile("" : : "r"(x));   /* keep x live */
            u64 cdt = t5 - t4;
            /* Resolve the EFFECTIVE memory attribute the CPU uses for this
             * address: AT S1E1R does a stage-1 EL1 read translation, PAR_EL1
             * bits [63:56] = MAIR attribute byte (0xFF=WB cacheable,
             * 0x44=Normal non-cacheable, 0x00=Device), [9:8]=shareability. */
            u64 par = 0;
            __asm__ volatile("at s1e1r, %1; isb; mrs %0, par_el1"
                             : "=r"(par) : "r"(addr) : "memory");
            http_append(out, &len, max, "membench addr=0x");
            http_append_hex64(out, &len, max, addr);
            http_append(out, &len, max, " count=");
            http_append_u64(out, &len, max, count);
            http_append(out, &len, max, " write_ps=");
            http_append_u64(out, &len, max, count ? (wdt * 18518ULL) / count : 0);
            http_append(out, &len, max, " read_ps=");
            http_append_u64(out, &len, max, count ? (rdt * 18518ULL) / count : 0);
            http_append(out, &len, max, " compute_ps=");
            http_append_u64(out, &len, max, count ? (cdt * 18518ULL) / count : 0);
            http_append(out, &len, max, " acc=");
            http_append_u64(out, &len, max, acc + x);
            http_append(out, &len, max, " par=0x");
            http_append_hex64(out, &len, max, par);
            http_append(out, &len, max, " attr=0x");
            http_append_hex32(out, &len, max, (u32)((par >> 56) & 0xFFU));
            http_append(out, &len, max, " sh=");
            http_append_u64(out, &len, max, (par >> 8) & 3U);
            http_append(out, &len, max, "\n");
        } else {
            http_append(out, &len, max, "usage membench <addr> [count]\n");
        }
    } else if (http_streq(cmd, "clockdiag")) {
        /* Directly measure the effective CPU clock with the PMU cycle counter
         * against the architected timer (no instruction-timing inference), read
         * the firmware throttle/temperature status (rules out under-voltage /
         * thermal as the cause), then RE-ASSERT the max ARM clock at runtime and
         * re-measure (a real fix attempt — tells us if the boot-time set was too
         * early to stick). */
        u64 cf; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(cf));
        if (!cf) cf = 54000000ULL;
        u64 pmcr; __asm__ volatile("mrs %0, pmcr_el0" : "=r"(pmcr));
        __asm__ volatile("msr pmcr_el0, %0" :: "r"(pmcr | 0x45ULL)); /* E|C|LC */
        __asm__ volatile("msr pmcntenset_el0, %0" :: "r"(1ULL << 31));
        __asm__ volatile("isb");
        u64 win = cf / 200ULL;   /* ~5ms measurement window */
        u64 c0, t0, c1, t1, now;
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t0));
        __asm__ volatile("mrs %0, pmccntr_el0" : "=r"(c0));
        do { __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now)); } while (now - t0 < win);
        __asm__ volatile("mrs %0, pmccntr_el0" : "=r"(c1));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t1));
        u64 cpu_hz1 = (t1 > t0) ? ((c1 - c0) * cf) / (t1 - t0) : 0;
        u32 thr = fb_get_throttled();
        u32 tmc = fb_get_temperature_mc();
        u32 arm = fb_get_arm_clock();
        u32 core = fb_get_clock_rate_id(4);
        u32 reassert = fb_set_arm_clock_max();
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t0));
        __asm__ volatile("mrs %0, pmccntr_el0" : "=r"(c0));
        do { __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now)); } while (now - t0 < win);
        __asm__ volatile("mrs %0, pmccntr_el0" : "=r"(c1));
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t1));
        u64 cpu_hz2 = (t1 > t0) ? ((c1 - c0) * cf) / (t1 - t0) : 0;
        http_append(out, &len, max, "clockdiag cpu_hz=");
        http_append_u64(out, &len, max, cpu_hz1);
        http_append(out, &len, max, " cpu_hz_after_reassert=");
        http_append_u64(out, &len, max, cpu_hz2);
        http_append(out, &len, max, " fw_arm_hz=");
        http_append_u64(out, &len, max, arm);
        http_append(out, &len, max, " fw_core_hz=");
        http_append_u64(out, &len, max, core);
        http_append(out, &len, max, " reassert_hz=");
        http_append_u64(out, &len, max, reassert);
        http_append(out, &len, max, " throttled=0x");
        http_append_hex32(out, &len, max, thr);
        http_append(out, &len, max, " temp_mC=");
        http_append_u64(out, &len, max, tmc);
        http_append(out, &len, max, " cntfrq=");
        http_append_u64(out, &len, max, cf);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "pixe") || http_streq(cmd, "pixe selftest")) {
        /* Run the embedded compiled sum.pc and report VM state; verifies the
         * on-board picovm is byte-identical to the off-board reference VM. */
        pixe_run_and_report(out, &len, max, pixe_sum_program,
                            (int)(sizeof(pixe_sum_program) / sizeof(pixe_sum_program[0])));
    } else if (http_streq(cmd, "pixe req")) {
        /* EL1 protocol component proof: decode a canned HTTP request, split it
         * into endpoint request-context spans, bind the principal, and dump
         * the result. This is the kernel-side half of the web pipeline. */
        static char pixe_req_buf[1024];
        u32 rn = pixe_request_selftest(pixe_req_buf, sizeof(pixe_req_buf));
        if (rn >= sizeof(pixe_req_buf)) rn = sizeof(pixe_req_buf) - 1;
        pixe_req_buf[rn] = 0;
        http_append(out, &len, max, pixe_req_buf);
    } else if (http_streq(cmd, "pixe endpoint")) {
        /* Endpoint-contract proof: build the canned request context (EL1 side),
         * then run the embedded echo endpoint (Context.GetVerb/GetBody -> Io.Write)
         * against it and seal the response. Verifies the read+emit+seal contract
         * (bound context -> picovm host hooks -> response) byte-for-byte on HW.
         * Actual EL0 entry remains gated on the pending eret/SVC scheduler path. */
        static char pixe_ep_buf[1024];
        u32 rn = pixe_host_selftest(pixe_ep_buf, sizeof(pixe_ep_buf));
        if (rn >= sizeof(pixe_ep_buf)) rn = sizeof(pixe_ep_buf) - 1;
        pixe_ep_buf[rn] = 0;
        http_append(out, &len, max, pixe_ep_buf);
    } else if (http_starts_with(cmd, "pixe endpoint ")) {
        /* Run a card-loaded endpoint program against the canned request context:
         * pixe endpoint <card> <record>. Ties the card pipeline to the EL0 host so
         * a compiled endpoint persisted as a picowal card runs against a bound
         * request context. */
        char *pargv[6];
        u32 pargc = http_split_args(cmd, pargv, 6);
        u32 card = 0, rec = 0;
        if (pargc >= 4 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            card <= PICOWAL_CARD_MAX && rec <= PICOWAL_RECORD_MAX) {
            static u8 pixe_ep_blob[PICOWAL_DATA_MAX];
            i32 n = picowal_db_get((u16)card, rec, pixe_ep_blob, PICOWAL_DATA_MAX);
            if (n < 0) {
                http_append(out, &len, max, "ERR: pixe endpoint card get failed\n");
            } else if ((n & 3) != 0) {
                http_append(out, &len, max, "ERR: card payload not a multiple of 4 bytes\n");
            } else {
                static u32 pixe_ep_prog[PICOWAL_DATA_MAX / 4];
                static char pixe_ep_buf2[1024];
                int nwords = n / 4;
                for (int i = 0; i < nwords; i++)
                    pixe_ep_prog[i] = (u32)pixe_ep_blob[i * 4 + 0] |
                                      ((u32)pixe_ep_blob[i * 4 + 1] << 8) |
                                      ((u32)pixe_ep_blob[i * 4 + 2] << 16) |
                                      ((u32)pixe_ep_blob[i * 4 + 3] << 24);
                u32 rn2 = pixe_host_run_report(pixe_ep_buf2, sizeof(pixe_ep_buf2),
                                               pixe_ep_prog, nwords);
                if (rn2 >= sizeof(pixe_ep_buf2)) rn2 = sizeof(pixe_ep_buf2) - 1;
                pixe_ep_buf2[rn2] = 0;
                http_append(out, &len, max, pixe_ep_buf2);
            }
        } else {
            http_append(out, &len, max, "usage pixe endpoint <card> <record>\n");
        }
    } else if (http_starts_with(cmd, "pixe run ")) {
        /* Run an inline bytecode program: pixe run <hexword> [hexword...] */
        static u32 pixe_prog[64];
        char *pargv[34];
        u32 pargc = http_split_args(cmd, pargv, 34);
        int n = 0;
        bool ok = true;
        for (u32 i = 2; i < pargc && n < 64; i++) {
            if (!pixe_parse_hex32(pargv[i], &pixe_prog[n])) { ok = false; break; }
            n++;
        }
        if (ok && n > 0)
            pixe_run_and_report(out, &len, max, pixe_prog, n);
        else
            http_append(out, &len, max, "usage pixe run <hexword> [hexword...]\n");
    } else if (http_starts_with(cmd, "pixe save ")) {
        /* Persist the embedded sum program to a picowal card as raw little-endian
         * bytecode, proving the capsule persist->load->execute round-trip. */
        char *pargv[6];
        u32 pargc = http_split_args(cmd, pargv, 6);
        u32 card = 0, rec = 0;
        if (pargc >= 4 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            card <= PICOWAL_CARD_MAX && rec <= PICOWAL_RECORD_MAX) {
            static u8 pixe_save_blob[sizeof(pixe_sum_program)];
            u32 nwords = (u32)(sizeof(pixe_sum_program) / sizeof(pixe_sum_program[0]));
            for (u32 i = 0; i < nwords; i++) {
                u32 w = pixe_sum_program[i];
                pixe_save_blob[i * 4 + 0] = (u8)(w & 0xFFU);
                pixe_save_blob[i * 4 + 1] = (u8)((w >> 8) & 0xFFU);
                pixe_save_blob[i * 4 + 2] = (u8)((w >> 16) & 0xFFU);
                pixe_save_blob[i * 4 + 3] = (u8)((w >> 24) & 0xFFU);
            }
            i32 n = picowal_db_put((u16)card, rec, pixe_save_blob, nwords * 4U);
            if (n < 0) {
                http_append(out, &len, max, "ERR: pixe save put failed\n");
            } else {
                http_append(out, &len, max, "pixe saved card=");
                http_append_u64(out, &len, max, card);
                http_append(out, &len, max, " record=");
                http_append_u64(out, &len, max, rec);
                http_append(out, &len, max, " bytes=");
                http_append_u64(out, &len, max, (u32)n);
                http_append(out, &len, max, "\n");
            }
        } else {
            http_append(out, &len, max, "usage pixe save <card> <record>\n");
        }
    } else if (http_starts_with(cmd, "pixe card ")) {
        /* Load raw little-endian bytecode from a picowal card and execute it.
         * This is the capsule-as-card execution path. */
        char *pargv[6];
        u32 pargc = http_split_args(cmd, pargv, 6);
        u32 card = 0, rec = 0;
        if (pargc >= 4 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            card <= PICOWAL_CARD_MAX && rec <= PICOWAL_RECORD_MAX) {
            static u8 pixe_card_blob[PICOWAL_DATA_MAX];
            i32 n = picowal_db_get((u16)card, rec, pixe_card_blob, PICOWAL_DATA_MAX);
            if (n < 0) {
                http_append(out, &len, max, "ERR: pixe card get failed\n");
            } else if ((n & 3) != 0) {
                http_append(out, &len, max, "ERR: card payload not a multiple of 4 bytes\n");
            } else {
                static u32 pixe_card_prog[PICOWAL_DATA_MAX / 4];
                int nwords = n / 4;
                for (int i = 0; i < nwords; i++)
                    pixe_card_prog[i] = (u32)pixe_card_blob[i * 4 + 0] |
                                        ((u32)pixe_card_blob[i * 4 + 1] << 8) |
                                        ((u32)pixe_card_blob[i * 4 + 2] << 16) |
                                        ((u32)pixe_card_blob[i * 4 + 3] << 24);
                pixe_run_and_report(out, &len, max, pixe_card_prog, nwords);
            }
        } else {
            http_append(out, &len, max, "usage pixe card <card> <record>\n");
        }
    } else if (http_starts_with(cmd, "pixe put ")) {
        /* Chunked host upload of compiled bytecode into a picowal card via
         * read-modify-write: pixe put <card> <record> <wordoffset> <hexword>...
         * wordoffset 0 starts a fresh program (truncates any existing record);
         * subsequent ascending chunks extend it. Self-contained and retry-safe so
         * the host tool can stream a program larger than the 128B command buffer. */
        char *pargv[34];
        u32 pargc = http_split_args(cmd, pargv, 34);
        u32 card = 0, rec = 0, woff = 0;
        if (pargc >= 5 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            http_parse_u32(pargv[4], &woff) && card <= PICOWAL_CARD_MAX &&
            rec <= PICOWAL_RECORD_MAX && woff < (PICOWAL_DATA_MAX / 4)) {
            static u8 pixe_put_buf[PICOWAL_DATA_MAX];
            u32 old_len = 0;
            if (woff != 0) {
                i32 g = picowal_db_get((u16)card, rec, pixe_put_buf, PICOWAL_DATA_MAX);
                if (g > 0) old_len = (u32)g;
            }
            /* Zero-fill any gap between the existing content and this chunk. */
            for (u32 z = old_len; z < woff * 4U; z++) pixe_put_buf[z] = 0;
            u32 count = 0;
            bool ok = true;
            for (u32 i = 5; i < pargc; i++) {
                u32 w;
                if (!pixe_parse_hex32(pargv[i], &w)) { ok = false; break; }
                u32 byte_off = (woff + count) * 4U;
                if (byte_off + 4U > PICOWAL_DATA_MAX) { ok = false; break; }
                pixe_put_buf[byte_off + 0] = (u8)(w & 0xFFU);
                pixe_put_buf[byte_off + 1] = (u8)((w >> 8) & 0xFFU);
                pixe_put_buf[byte_off + 2] = (u8)((w >> 16) & 0xFFU);
                pixe_put_buf[byte_off + 3] = (u8)((w >> 24) & 0xFFU);
                count++;
            }
            if (!ok || count == 0) {
                http_append(out, &len, max, "usage pixe put <card> <record> <wordoffset> <hexword>...\n");
            } else {
                u32 new_len = (woff + count) * 4U;
                if (new_len < old_len) new_len = old_len;
                i32 n = picowal_db_put((u16)card, rec, pixe_put_buf, new_len);
                if (n < 0) {
                    http_append(out, &len, max, "ERR: pixe put failed\n");
                } else {
                    http_append(out, &len, max, "pixe put card=");
                    http_append_u64(out, &len, max, card);
                    http_append(out, &len, max, " record=");
                    http_append_u64(out, &len, max, rec);
                    http_append(out, &len, max, " offset=");
                    http_append_u64(out, &len, max, woff);
                    http_append(out, &len, max, " count=");
                    http_append_u64(out, &len, max, count);
                    http_append(out, &len, max, " total_words=");
                    http_append_u64(out, &len, max, (u32)n / 4U);
                    http_append(out, &len, max, " total_bytes=");
                    http_append_u64(out, &len, max, (u32)n);
                    http_append(out, &len, max, "\n");
                }
            }
        } else {
            http_append(out, &len, max, "usage pixe put <card> <record> <wordoffset> <hexword>...\n");
        }
    } else if (http_starts_with(cmd, "pixe srcput ")) {
        /* Chunked source-text upload:
         * pixe srcput <card> <record> <byteoffset> <hexbytes>
         * byteoffset 0 starts/truncates; subsequent chunks extend. */
        char *pargv[6];
        u32 pargc = http_split_args(cmd, pargv, 6);
        u32 card = 0, rec = 0, off = 0;
        if (pargc >= 6 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            http_parse_u32(pargv[4], &off) && card <= PICOWAL_CARD_MAX &&
            rec <= PICOWAL_RECORD_MAX && off < PICOWAL_DATA_MAX) {
            static u8 src_buf[PICOWAL_DATA_MAX];
            u32 old_len = 0;
            if (off != 0) {
                i32 g = picowal_db_get((u16)card, rec, src_buf, PICOWAL_DATA_MAX);
                if (g > 0) old_len = (u32)g;
            }
            for (u32 z = old_len; z < off && z < PICOWAL_DATA_MAX; z++) src_buf[z] = 0;
            const char *hex = pargv[5];
            u32 hex_len = pios_strlen(hex);
            bool ok = (hex_len != 0 && (hex_len & 1U) == 0);
            u32 count = hex_len / 2U;
            if (off + count > PICOWAL_DATA_MAX) ok = false;
            for (u32 i = 0; ok && i < count; i++)
                ok = pixe_parse_hex_byte_pair(hex + i * 2U, &src_buf[off + i]);
            if (!ok) {
                http_append(out, &len, max, "usage pixe srcput <card> <record> <byteoffset> <hexbytes>\n");
            } else {
                u32 new_len = off + count;
                if (new_len < old_len) new_len = old_len;
                i32 n = picowal_db_put((u16)card, rec, src_buf, new_len);
                if (n < 0) {
                    http_append(out, &len, max, "ERR: pixe srcput failed\n");
                } else {
                    http_append(out, &len, max, "pixe srcput card=");
                    http_append_u64(out, &len, max, card);
                    http_append(out, &len, max, " record=");
                    http_append_u64(out, &len, max, rec);
                    http_append(out, &len, max, " offset=");
                    http_append_u64(out, &len, max, off);
                    http_append(out, &len, max, " count=");
                    http_append_u64(out, &len, max, count);
                    http_append(out, &len, max, " total_bytes=");
                    http_append_u64(out, &len, max, (u32)n);
                    http_append(out, &len, max, "\n");
                }
            }
        } else {
            http_append(out, &len, max, "usage pixe srcput <card> <record> <byteoffset> <hexbytes>\n");
        }
    } else if (http_starts_with(cmd, "pixe srcget ")) {
        char *pargv[6];
        u32 pargc = http_split_args(cmd, pargv, 6);
        u32 card = 0, rec = 0;
        if (pargc >= 4 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            card <= PICOWAL_CARD_MAX && rec <= PICOWAL_RECORD_MAX) {
            static u8 src_get_buf[PICOWAL_DATA_MAX];
            i32 n = picowal_db_get((u16)card, rec, src_get_buf, PICOWAL_DATA_MAX);
            if (n < 0) {
                http_append(out, &len, max, "ERR: pixe srcget failed\n");
            } else {
                http_append(out, &len, max, "pixe source card=");
                http_append_u64(out, &len, max, card);
                http_append(out, &len, max, " record=");
                http_append_u64(out, &len, max, rec);
                http_append(out, &len, max, " bytes=");
                http_append_u64(out, &len, max, (u32)n);
                http_append(out, &len, max, "\n");
                http_append_bytes(out, &len, max, src_get_buf, (u32)n);
                http_append(out, &len, max, "\n");
            }
        } else {
            http_append(out, &len, max, "usage pixe srcget <card> <record>\n");
        }
    } else if (http_starts_with(cmd, "pixe srchex ")) {
        char *pargv[6];
        u32 pargc = http_split_args(cmd, pargv, 6);
        u32 card = 0, rec = 0;
        if (pargc >= 4 && http_parse_u32(pargv[2], &card) && http_parse_u32(pargv[3], &rec) &&
            card <= PICOWAL_CARD_MAX && rec <= PICOWAL_RECORD_MAX) {
            static u8 src_get_buf[PICOWAL_DATA_MAX];
            i32 n = picowal_db_get((u16)card, rec, src_get_buf, PICOWAL_DATA_MAX);
            if (n < 0) {
                http_append(out, &len, max, "ERR: pixe srchex failed\n");
            } else {
                http_append(out, &len, max, "pixe srchex card=");
                http_append_u64(out, &len, max, card);
                http_append(out, &len, max, " record=");
                http_append_u64(out, &len, max, rec);
                http_append(out, &len, max, " bytes=");
                http_append_u64(out, &len, max, (u32)n);
                http_append(out, &len, max, "\n");
                for (i32 i = 0; i < n; i++)
                    http_append_hex8(out, &len, max, src_get_buf[i]);
                http_append(out, &len, max, "\n");
            }
        } else {
            http_append(out, &len, max, "usage pixe srchex <card> <record>\n");
        }
    } else if (http_streq(cmd, "uhttp")) {
        for (u32 bi = 0; bi < UHTTP_BRIDGE_COUNT; bi++) {
            i32 bl = 0; u32 bst = 0, brq = 0, brs = 0, breq = 0, bmagic = 0, bpid = 0;
            uhttp_bridge_state_idx(bi, &bl, &bst, &brq, &brs, &breq, &bmagic, &bpid);
            http_append(out, &len, max, "bridge");
            http_append_u64(out, &len, max, bi);
            http_append(out, &len, max, " listen=");
            http_append_u64(out, &len, max, (u32)bl);
            http_append(out, &len, max, " state=");
            http_append_u64(out, &len, max, bst);
            http_append(out, &len, max, " req_seq=");
            http_append_u64(out, &len, max, brq);
            http_append(out, &len, max, " resp_seq=");
            http_append_u64(out, &len, max, brs);
            http_append(out, &len, max, " reqs=");
            http_append_u64(out, &len, max, breq);
            http_append(out, &len, max, " magic=0x");
            http_append_hex32(out, &len, max, bmagic);
            http_append(out, &len, max, " pid=");
            http_append_u64(out, &len, max, bpid);
            http_append(out, &len, max, "\n");
        }
        i32 lc = 0; u32 st = 0, rq = 0, rs = 0, reqs = 0, magic = 0, pid = 0;
        uhttp_bridge_state(&lc, &st, &rq, &rs, &reqs, &magic, &pid);
        http_append(out, &len, max, "uhttp listen=");
        http_append_u64(out, &len, max, (u64)(u32)lc);
        http_append(out, &len, max, " state=");
        http_append_u64(out, &len, max, st);
        http_append(out, &len, max, " req_seq=");
        http_append_u64(out, &len, max, rq);
        http_append(out, &len, max, " resp_seq=");
        http_append_u64(out, &len, max, rs);
        http_append(out, &len, max, " reqs_total=");
        http_append_u64(out, &len, max, reqs);
        http_append(out, &len, max, " magic=0x");
        http_append_hex32(out, &len, max, magic);
        http_append(out, &len, max, " httpd_pid=");
        http_append_u64(out, &len, max, pid);
        u32 wp = 0, wd = 0, wf = 0;
        proc_rwake_stats(&wp, &wd, &wf);
        http_append(out, &len, max, " wake_posted=");
        http_append_u64(out, &len, max, wp);
        http_append(out, &len, max, " wake_drained=");
        http_append_u64(out, &len, max, wd);
        http_append(out, &len, max, " wake_full=");
        http_append_u64(out, &len, max, wf);
        http_append(out, &len, max, " dbg_loops=");
        http_append_u64(out, &len, max, uhttp_bridge()->dbg_loops);
        http_append(out, &len, max, " dbg_phase=");
        http_append_u64(out, &len, max, uhttp_bridge()->dbg_phase);
        http_append(out, &len, max, " hb_core2=");
        http_append_u64(out, &len, max, proc_sched_loops(CORE_USER0));
        http_append(out, &len, max, " hb_core3=");
        http_append_u64(out, &len, max, proc_sched_loops(CORE_USER1));
        {
            u32 ce = 0, cx = 0, lp = 0;
            proc_sched_ctx_stats(CORE_USER0, &ce, &cx, &lp);
            http_append(out, &len, max, " c2_ctxin=");
            http_append_u64(out, &len, max, ce);
            http_append(out, &len, max, " c2_ctxout=");
            http_append_u64(out, &len, max, cx);
            http_append(out, &len, max, " c2_lastpid=");
            http_append_u64(out, &len, max, lp);
            http_append(out, &len, max, " c2_stage=");
            http_append_u64(out, &len, max, proc_sched_stage_get(CORE_USER0));
            proc_sched_ctx_stats(CORE_USER1, &ce, &cx, &lp);
            http_append(out, &len, max, " c3_ctxin=");
            http_append_u64(out, &len, max, ce);
            http_append(out, &len, max, " c3_ctxout=");
            http_append_u64(out, &len, max, cx);
            http_append(out, &len, max, " c3_lastpid=");
            http_append_u64(out, &len, max, lp);
            http_append(out, &len, max, " c3_stage=");
            http_append_u64(out, &len, max, proc_sched_stage_get(CORE_USER1));
        }
        {
            u32 di = 0, dp = 0, dz = 0, dc = 0, dn = 0, ds = 0;
            proc_rwake_dbg(&di, &dp, &dz, &dc, &dn, &ds);
            http_append(out, &len, max, " d_iters=");
            http_append_u64(out, &len, max, di);
            http_append(out, &len, max, " d_pid=");
            http_append_u64(out, &len, max, dp);
            http_append(out, &len, max, " d_zero=");
            http_append_u64(out, &len, max, dz);
            http_append(out, &len, max, " d_calls=");
            http_append_u64(out, &len, max, dc);
            http_append(out, &len, max, " d_noslot=");
            http_append_u64(out, &len, max, dn);
            http_append(out, &len, max, " d_state=");
            http_append_u64(out, &len, max, ds);
        }
        {
            u32 ls = 0, lp = 0, dn = 0, wf = 0;
            proc_rwake_live(CORE_USER0, &ls, &lp, &dn, &wf);
            http_append(out, &len, max, " c2_curstate=");
            http_append_u64(out, &len, max, ls);
            http_append(out, &len, max, " c2_curpid=");
            http_append_u64(out, &len, max, lp);
            http_append(out, &len, max, " c2_disp=");
            http_append_u64(out, &len, max, dn);
            http_append(out, &len, max, " c2_wfe=");
            http_append_u64(out, &len, max, wf);
            proc_rwake_live(CORE_USER1, &ls, &lp, &dn, &wf);
            http_append(out, &len, max, " c3_curstate=");
            http_append_u64(out, &len, max, ls);
            http_append(out, &len, max, " c3_curpid=");
            http_append_u64(out, &len, max, lp);
            http_append(out, &len, max, " c3_disp=");
            http_append_u64(out, &len, max, dn);
            http_append(out, &len, max, " c3_wfe=");
            http_append_u64(out, &len, max, wf);
        }
        {
            u32 resc = 0, rsn = 0, pe = 0, pel = 0, pb = 0, pr = 0;
            proc_rwake_park_dbg(CORE_USER0, &resc, &rsn, &pe, &pel, &pb, &pr);
            http_append(out, &len, max, " rescued=");
            http_append_u64(out, &len, max, resc);
            http_append(out, &len, max, " c2_desched=");
            http_append_u64(out, &len, max, rsn);
            http_append(out, &len, max, " park_enter=");
            http_append_u64(out, &len, max, pe);
            http_append(out, &len, max, " park_early=");
            http_append_u64(out, &len, max, pel);
            http_append(out, &len, max, " park_block=");
            http_append_u64(out, &len, max, pb);
            http_append(out, &len, max, " park_resume=");
            http_append_u64(out, &len, max, pr);
        }
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "fb info")) {
        u32 db = 0, size = 0, pitch = 0;
        fb_debug_info(&db, &size, &pitch);
        u64 cntfrq; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(cntfrq));
        http_append(out, &len, max, "fb double_buffer=");
        http_append_u64(out, &len, max, db);
        http_append(out, &len, max, " size=");
        http_append_u64(out, &len, max, size);
        http_append(out, &len, max, " back_size=");
        http_append_u64(out, &len, max, (u64)FB_BACK_SIZE);
        http_append(out, &len, max, " pitch=");
        http_append_u64(out, &len, max, pitch);
        http_append(out, &len, max, " cntfrq=");
        http_append_u64(out, &len, max, cntfrq);
        http_append(out, &len, max, " dash_snap_ticks=");
        http_append_u64(out, &len, max, g_dash_snap_ticks);
        http_append(out, &len, max, " dash_render_ticks=");
        http_append_u64(out, &len, max, g_dash_render_ticks);
        http_append(out, &len, max, " blit_ticks=");
        http_append_u64(out, &len, max, fb_last_blit_ticks());
        u64 sctlr, tcr, cel;
        __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
        __asm__ volatile("mrs %0, tcr_el1" : "=r"(tcr));
        __asm__ volatile("mrs %0, currentel" : "=r"(cel));
        http_append(out, &len, max, " sctlr=0x");
        http_append_hex64(out, &len, max, sctlr);
        http_append(out, &len, max, " (M=");
        http_append_u64(out, &len, max, sctlr & 1U);
        http_append(out, &len, max, " C=");
        http_append_u64(out, &len, max, (sctlr >> 2) & 1U);
        http_append(out, &len, max, " I=");
        http_append_u64(out, &len, max, (sctlr >> 12) & 1U);
        http_append(out, &len, max, ") tcr=0x");
        http_append_hex64(out, &len, max, tcr);
        http_append(out, &len, max, " EL=");
        http_append_u64(out, &len, max, (cel >> 2) & 3U);
        http_append(out, &len, max, " arm_clock=");
        http_append_u64(out, &len, max, fb_get_arm_clock());
        http_append(out, &len, max, "\n");
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
        /* SW reactor (SEV/WFE) + ETH RX IRQ + hardware watchdog liveness. */
        u64 wk = 0, wf = 0, it = 0, tt = 0; u32 bp = 0, lf = 0;
        core0_sched_snapshot(&wk, &wf, &it, &tt, &bp, &lf);
        struct watchdog_status wd;
        watchdog_status(&wd);
        http_append(out, &len, max, "sw wake=");
        http_append_u64(out, &len, max, wk);
        http_append(out, &len, max, " wfi=");
        http_append_u64(out, &len, max, wf);
        http_append(out, &len, max, " busy_permille=");
        http_append_u64(out, &len, max, bp);
        http_append(out, &len, max, " | eth_irq=");
        http_append_u64(out, &len, max, core0_eth_irq_count);
        http_append(out, &len, max, " eth_quench_passes=");
        http_append_u64(out, &len, max, core0_eth_irq_quench_passes);
        http_append(out, &len, max, "\nwdog armed=");
        http_append(out, &len, max, wd.armed ? "yes" : "no");
        http_append(out, &len, max, " reset_in_s=");
        http_append_u64(out, &len, max, watchdog_hw_remaining_ticks() >> 16);
        http_append(out, &len, max, " trips=");
        http_append_u64(out, &len, max, wd.trip_count);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "coredump") || http_streq(cmd, "crash")) {
        struct exception_crash_record cr;
        if (!exception_crash_snapshot(&cr)) {
            http_append(out, &len, max, "coredump none\n");
        } else {
            http_append(out, &len, max, "coredump magic=0x");
            http_append_hex32(out, &len, max, cr.magic);
            http_append(out, &len, max, " version=");
            http_append_u64(out, &len, max, cr.version);
            http_append(out, &len, max, " kind=");
            http_append_u64(out, &len, max, cr.kind);
            http_append(out, &len, max, " core=");
            http_append_u64(out, &len, max, cr.core);
            http_append(out, &len, max, " el=");
            http_append_u64(out, &len, max, cr.current_el);
            http_append(out, &len, max, " ec=0x");
            http_append_hex32(out, &len, max, cr.ec);
            http_append(out, &len, max, "\npid=");
            http_append_u64(out, &len, max, cr.pid);
            http_append(out, &len, max, " capsule=");
            http_append_u64(out, &len, max, cr.capsule);
            http_append(out, &len, max, " generation=");
            http_append_u64(out, &len, max, cr.process_generation);
            http_append(out, &len, max, " owner=");
            http_append_u64(out, &len, max, cr.owner_principal);
            http_append(out, &len, max, "\ndesc_id=");
            http_append_u64(out, &len, max, cr.descriptor_id);
            http_append(out, &len, max, " desc_gen=");
            http_append_u64(out, &len, max, cr.descriptor_generation);
            http_append(out, &len, max, " desc_owner=");
            http_append_u64(out, &len, max, cr.descriptor_owner);
            http_append(out, &len, max, " fifo_seq=");
            http_append_u64(out, &len, max, cr.last_fifo_seq);
            http_append(out, &len, max, "\nesr=0x");
            http_append_hex64(out, &len, max, cr.esr);
            http_append(out, &len, max, " elr=0x");
            http_append_hex64(out, &len, max, cr.elr);
            http_append(out, &len, max, " far=0x");
            http_append_hex64(out, &len, max, cr.far);
            http_append(out, &len, max, "\nsp=0x");
            http_append_hex64(out, &len, max, cr.sp);
            http_append(out, &len, max, " ttbr0=0x");
            http_append_hex64(out, &len, max, cr.ttbr0);
            http_append(out, &len, max, " syndrome=0x");
            http_append_hex64(out, &len, max, cr.syndrome);
            http_append(out, &len, max, " ticks=");
            http_append_u64(out, &len, max, cr.ticks);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "crashlba")) {
        /* Read the SD-persisted crash record (survives the watchdog reset that
         * clears DRAM). After a wedge-crash+reboot, this reveals where core0
         * faulted: elr=faulting PC, far=fault address, ec=exception class. */
        struct exception_crash_record cr;
        if (!exception_crash_sd_read(&cr)) {
            http_append(out, &len, max, "crashlba none (no persisted crash on SD)\n");
        } else {
            http_append(out, &len, max, "crashlba kind=");
            http_append_u64(out, &len, max, cr.kind);
            http_append(out, &len, max, " core=");
            http_append_u64(out, &len, max, cr.core);
            http_append(out, &len, max, " el=");
            http_append_u64(out, &len, max, cr.current_el);
            http_append(out, &len, max, " ec=0x");
            http_append_hex32(out, &len, max, cr.ec);
            http_append(out, &len, max, "\nesr=0x");
            http_append_hex64(out, &len, max, cr.esr);
            http_append(out, &len, max, " elr=0x");
            http_append_hex64(out, &len, max, cr.elr);
            http_append(out, &len, max, " far=0x");
            http_append_hex64(out, &len, max, cr.far);
            http_append(out, &len, max, "\nsp=0x");
            http_append_hex64(out, &len, max, cr.sp);
            http_append(out, &len, max, " ttbr0=0x");
            http_append_hex64(out, &len, max, cr.ttbr0);
            http_append(out, &len, max, " pid=");
            http_append_u64(out, &len, max, cr.pid);
            http_append(out, &len, max, " ticks=");
            http_append_u64(out, &len, max, cr.ticks);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "crashlba clear")) {
        exception_crash_sd_clear();
        http_append(out, &len, max, "crashlba cleared\n");
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
    } else if (http_streq(cmd, "el0") || http_streq(cmd, "el0 probe")) {
        u32 seen = 0, pid = 0, spsr = 0, exits = 0;
        u64 arg = 0, elr = 0;
        proc_el0_probe_snapshot(&seen, &pid, &spsr, &arg, &elr, &exits);
        http_append(out, &len, max, "el0 probe seen=");
        http_append_u64(out, &len, max, seen);
        http_append(out, &len, max, " pid=");
        http_append_u64(out, &len, max, pid);
        http_append(out, &len, max, " spsr=0x");
        http_append_hex32(out, &len, max, spsr);
        http_append(out, &len, max, " arg=0x");
        http_append_hex64(out, &len, max, arg);
        http_append(out, &len, max, " elr=0x");
        http_append_hex64(out, &len, max, elr);
        http_append(out, &len, max, " exits=");
        http_append_u64(out, &len, max, exits);
        i32 lst = 0; u32 lpid = 0, lslot = 0, ecnt = 0, epid = 0, fpid = 0;
        u64 lbase = 0, epc = 0, esp = 0, fesr = 0, felr = 0, ffar = 0, fl1e = 0, fl2e = 0, fl3e = 0;
        u64 par0w = 0, par0r = 0, par1w = 0;
        proc_el0_diag_snapshot(&lst, &lpid, &lslot, &lbase, &ecnt, &epid,
                               &epc, &esp, &fpid, &fesr, &felr, &ffar, &fl1e, &fl2e, &fl3e,
                               &par0w, &par0r, &par1w);
        http_append(out, &len, max, " launch=");
        if (lst < 0) {
            http_append(out, &len, max, "-");
            http_append_u64(out, &len, max, (u32)(-lst));
        } else {
            http_append_u64(out, &len, max, (u32)lst);
        }
        http_append(out, &len, max, " lpid=");
        http_append_u64(out, &len, max, lpid);
        http_append(out, &len, max, " slot=");
        http_append_u64(out, &len, max, lslot);
        http_append(out, &len, max, " base=0x");
        http_append_hex64(out, &len, max, lbase);
        http_append(out, &len, max, " enter=");
        http_append_u64(out, &len, max, ecnt);
        http_append(out, &len, max, " epid=");
        http_append_u64(out, &len, max, epid);
        http_append(out, &len, max, " epc=0x");
        http_append_hex64(out, &len, max, epc);
        http_append(out, &len, max, " esp=0x");
        http_append_hex64(out, &len, max, esp);
        http_append(out, &len, max, " fpid=");
        http_append_u64(out, &len, max, fpid);
        http_append(out, &len, max, " fesr=0x");
        http_append_hex64(out, &len, max, fesr);
        http_append(out, &len, max, " felr=0x");
        http_append_hex64(out, &len, max, felr);
        http_append(out, &len, max, " ffar=0x");
        http_append_hex64(out, &len, max, ffar);
        http_append(out, &len, max, " l1e=0x");
        http_append_hex64(out, &len, max, fl1e);
        http_append(out, &len, max, " l2e=0x");
        http_append_hex64(out, &len, max, fl2e);
        http_append(out, &len, max, " l3e=0x");
        http_append_hex64(out, &len, max, fl3e);
        http_append(out, &len, max, " par0w=0x");
        http_append_hex64(out, &len, max, par0w);
        http_append(out, &len, max, " par0r=0x");
        http_append_hex64(out, &len, max, par0r);
        http_append(out, &len, max, " par1w=0x");
        http_append_hex64(out, &len, max, par1w);
        http_append(out, &len, max, "\n");
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
        http_append(out, &len, max, " fifo_irq_delta=");
        http_append_u64(out, &len, max, b.fifo_irq_delta);
        http_append(out, &len, max, " cross_fifo_ticks=");
        http_append_u64(out, &len, max, b.cross_fifo_ticks);
        http_append(out, &len, max, " cross_batch_ticks=");
        http_append_u64(out, &len, max, b.cross_batch_ticks);
        http_append(out, &len, max, " micro_full_ticks=");
        http_append_u64(out, &len, max, b.cross_micro_full_ticks);
        http_append(out, &len, max, " micro_part_ticks=");
        http_append_u64(out, &len, max, b.cross_micro_partial_ticks);
        http_append(out, &len, max, " ring_batch_ticks=");
        http_append_u64(out, &len, max, b.cross_ring_batch_ticks);
        http_append(out, &len, max, " span_ring_ticks=");
        http_append_u64(out, &len, max, b.cross_span_ring_ticks);
        http_append(out, &len, max, " span_all_ticks=");
        http_append_u64(out, &len, max, b.cross_span_all_ticks);
        http_append(out, &len, max, " rt_base_ticks=");
        http_append_u64(out, &len, max, b.span_rt_base_ticks);
        http_append(out, &len, max, " rt_ish_ticks=");
        http_append_u64(out, &len, max, b.span_rt_ish_ticks);
        http_append(out, &len, max, " rt_acqrel_ticks=");
        http_append_u64(out, &len, max, b.span_rt_acqrel_ticks);
        http_append(out, &len, max, " rt_asm_ticks=");
        http_append_u64(out, &len, max, b.span_rt_asm_ticks);
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
            http_append(out, &len, max, " cross_batch_per=");
            http_append_u64(out, &len, max, b.cross_batch_ticks / b.iterations);
            http_append(out, &len, max, " micro_full_per=");
            http_append_u64(out, &len, max, b.cross_micro_full_ticks / b.iterations);
            http_append(out, &len, max, " micro_part_per=");
            http_append_u64(out, &len, max, b.cross_micro_partial_ticks / b.iterations);
            http_append(out, &len, max, " ring_batch_per=");
            http_append_u64(out, &len, max, b.cross_ring_batch_ticks / b.iterations);
            http_append(out, &len, max, " span_ring_per=");
            http_append_u64(out, &len, max, b.cross_span_ring_ticks / b.iterations);
            http_append(out, &len, max, " span_all_per=");
            http_append_u64(out, &len, max, b.cross_span_all_ticks / b.iterations);
            http_append(out, &len, max, " rt_base_per=");
            http_append_u64(out, &len, max, b.span_rt_base_ticks / b.iterations);
            http_append(out, &len, max, " rt_ish_per=");
            http_append_u64(out, &len, max, b.span_rt_ish_ticks / b.iterations);
            http_append(out, &len, max, " rt_acqrel_per=");
            http_append_u64(out, &len, max, b.span_rt_acqrel_ticks / b.iterations);
            http_append(out, &len, max, " rt_asm_per=");
            http_append_u64(out, &len, max, b.span_rt_asm_ticks / b.iterations);
            http_append(out, &len, max, " sev_per=");
            http_append_u64(out, &len, max, b.sev_ticks / b.iterations);
        }
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "cohdiag") || http_starts_with(cmd, "cohdiag ")) {
        u32 iters = 2000;
        u32 ccore = CORE_USER1;
        if (http_starts_with(cmd, "cohdiag ")) {
            const char *p = cmd + 8;
            u32 v = 0;
            if (http_parse_u32(p, &v) && v > 0) iters = v;
            while (*p == ' ') p++;
            while (*p && *p != ' ') p++;
            while (*p == ' ') p++;
            if (*p) {
                u32 c = 0;
                if (http_parse_u32(p, &c)) ccore = c;
            }
        }
        struct proc_cohdiag_result r;
        bool ok = proc_cohdiag(iters, ccore, &r);
        http_append(out, &len, max, ok ? "cohdiag OK" : "cohdiag ERR");
        http_append(out, &len, max, " consumer_core=");
        http_append_u64(out, &len, max, r.consumer_core);
        http_append(out, &len, max, " seqs=");
        http_append_u64(out, &len, max, r.producer_seqs);
        http_append(out, &len, max, " wb_safe=");
        http_append_u64(out, &len, max, r.wb_safe);
        http_append(out, &len, max, " | B attr fifo=0x");
        http_append_hex32(out, &len, max, r.attr_fifo);
        http_append(out, &len, max, "/sh");
        http_append_u64(out, &len, max, r.sh_fifo);
        http_append(out, &len, max, " dma_net=0x");
        http_append_hex32(out, &len, max, r.attr_dma_net);
        http_append(out, &len, max, "/sh");
        http_append_u64(out, &len, max, r.sh_dma_net);
        http_append(out, &len, max, " ipc=0x");
        http_append_hex32(out, &len, max, r.attr_ipc);
        http_append(out, &len, max, "/sh");
        http_append_u64(out, &len, max, r.sh_ipc);
        http_append(out, &len, max, " wb=0x");
        http_append_hex32(out, &len, max, r.attr_wb);
        http_append(out, &len, max, "/sh");
        http_append_u64(out, &len, max, r.sh_wb);
        http_append(out, &len, max, " nc=0x");
        http_append_hex32(out, &len, max, r.attr_nc);
        http_append(out, &len, max, "/sh");
        http_append_u64(out, &len, max, r.sh_nc);
        http_append(out, &len, max, " code=0x");
        http_append_hex32(out, &len, max, r.attr_code);
        http_append(out, &len, max, "/sh");
        http_append_u64(out, &len, max, r.sh_code);
        http_append(out, &len, max, " par_fault=");
        http_append_u64(out, &len, max, r.par_fault);
        http_append(out, &len, max, " | A wb_acq[chk=");
        http_append_u64(out, &len, max, r.wb_checks);
        http_append(out, &len, max, " mis=");
        http_append_u64(out, &len, max, r.wb_mismatch);
        http_append(out, &len, max, " tear=");
        http_append_u64(out, &len, max, r.wb_tears);
        http_append(out, &len, max, " to=");
        http_append_u64(out, &len, max, r.wb_timeout);
        http_append(out, &len, max, "] wb_noacq[chk=");
        http_append_u64(out, &len, max, r.wb_noacq_checks);
        http_append(out, &len, max, " mis=");
        http_append_u64(out, &len, max, r.wb_noacq_mismatch);
        http_append(out, &len, max, " tear=");
        http_append_u64(out, &len, max, r.wb_noacq_tears);
        http_append(out, &len, max, " to=");
        http_append_u64(out, &len, max, r.wb_noacq_timeout);
        http_append(out, &len, max, "] nc[chk=");
        http_append_u64(out, &len, max, r.nc_checks);
        http_append(out, &len, max, " mis=");
        http_append_u64(out, &len, max, r.nc_mismatch);
        http_append(out, &len, max, " tear=");
        http_append_u64(out, &len, max, r.nc_tears);
        http_append(out, &len, max, " to=");
        http_append_u64(out, &len, max, r.nc_timeout);
        http_append(out, &len, max, "] | C nc_seq=");
        http_append_u64(out, &len, max, r.nc_seq_ps);
        http_append(out, &len, max, "ps nc_scat=");
        http_append_u64(out, &len, max, r.nc_scatter_ps);
        http_append(out, &len, max, "ps wb_seq=");
        http_append_u64(out, &len, max, r.wb_seq_ps);
        http_append(out, &len, max, "ps wb_scat=");
        http_append_u64(out, &len, max, r.wb_scatter_ps);
        http_append(out, &len, max, "ps wb_hot=");
        http_append_u64(out, &len, max, r.wb_hot_ps);
        http_append(out, &len, max, "ps\n");
    } else if (http_streq(cmd, "v3d reset soft") ||
               http_streq(cmd, "qpu reset soft") ||
               http_streq(cmd, "tensor reset soft")) {
        watchdog_hw_arm_seconds(5);
        v3d_status_t st = v3d_soft_reset();
        watchdog_hw_disable();
        struct v3d_reset_debug r;
        v3d_reset_debug_last(&r);
        http_append(out, &len, max, st == V3D_STATUS_OK ? "v3d reset soft OK" : "v3d reset soft FAILED");
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)st);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, r.err_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.err_after_clear);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.err_after_reset);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, r.core_int_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.hub_int_before);
        http_append(out, &len, max, " -> ");
        http_append_hex32(out, &len, max, r.core_int_after);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.hub_int_after);
        http_append(out, &len, max, " sms=");
        http_append_hex32(out, &len, max, r.sms_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.sms_after);
        http_append(out, &len, max, " mmu=");
        http_append_hex32(out, &len, max, r.mmu_ctl_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.mmuc_before);
        http_append(out, &len, max, " -> ");
        http_append_hex32(out, &len, max, r.mmu_ctl_after);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.mmuc_after);
        http_append(out, &len, max, " pm=");
        http_append_hex32(out, &len, max, r.pm_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.pm_asserted);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.pm_after);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "v3d reset pm confirm") ||
               http_streq(cmd, "qpu reset pm confirm") ||
               http_streq(cmd, "tensor reset pm confirm")) {
        watchdog_hw_arm_seconds(5);
        v3d_status_t st = v3d_pm_reset();
        watchdog_hw_disable();
        struct v3d_reset_debug r;
        v3d_reset_debug_last(&r);
        http_append(out, &len, max, st == V3D_STATUS_OK ? "v3d reset pm OK" : "v3d reset pm FAILED");
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)st);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, r.err_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.err_after_clear);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.err_after_reset);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, r.core_int_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.hub_int_before);
        http_append(out, &len, max, " -> ");
        http_append_hex32(out, &len, max, r.core_int_after);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.hub_int_after);
        http_append(out, &len, max, " sms=");
        http_append_hex32(out, &len, max, r.sms_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.sms_after);
        http_append(out, &len, max, " mmu=");
        http_append_hex32(out, &len, max, r.mmu_ctl_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.mmuc_before);
        http_append(out, &len, max, " -> ");
        http_append_hex32(out, &len, max, r.mmu_ctl_after);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.mmuc_after);
        http_append(out, &len, max, " pm=");
        http_append_hex32(out, &len, max, r.pm_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.pm_asserted);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, r.pm_after);
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
        http_append(out, &len, max, " native=");
        http_append(out, &len, max, ts.v3d_native_probe_ok ? "yes" : "no");
        http_append(out, &len, max, " nself=");
        http_append(out, &len, max, ts.v3d_native_selftest_ok ? "yes" : "no");
        http_append(out, &len, max, " ncomp=");
        http_append(out, &len, max, ts.v3d_native_compute_enabled ? "yes" : "no");
        http_append(out, &len, max, " nmmu=");
        http_append(out, &len, max, ts.v3d_native_mmu_ready ? "yes" : "no");
        http_append(out, &len, max, " tiny=");
        http_append(out, &len, max, ts.v3d_native_tiny_kernels_ready ? "yes" : "no");
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
        http_append(out, &len, max, " tv=");
        http_append_u64(out, &len, max, ts.v3d_tech_version);
        http_append(out, &len, max, " cores=");
        http_append_u64(out, &len, max, ts.v3d_core_count);
        http_append(out, &len, max, " qps=");
        http_append_u64(out, &len, max, ts.v3d_qpus_per_slice);
        http_append(out, &len, max, " slices=");
        http_append_u64(out, &len, max, ts.v3d_slice_count);
        http_append(out, &len, max, " csd=");
        http_append_u64(out, &len, max, ts.v3d_csd_status);
        http_append(out, &len, max, " nstat=");
        http_append_u64(out, &len, max, (u32)ts.v3d_native_selftest_status);
        http_append(out, &len, max, " mmuctl=");
        http_append_u64(out, &len, max, ts.v3d_mmu_ctl);
        http_append(out, &len, max, " mmuc=");
        http_append_u64(out, &len, max, ts.v3d_mmuc_control);
        http_append(out, &len, max, " tiny_ready=");
        http_append_u64(out, &len, max, ts.v3d_tiny_ready_mask);
        http_append(out, &len, max, " tiny_ver=");
        http_append_u64(out, &len, max, ts.v3d_tiny_verified_mask);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display global reapply dryrun") ||
               http_streq(cmd, "vc display global reapply dry-run") ||
               http_streq(cmd, "vc display hvs reapply dryrun") ||
               http_streq(cmd, "vc display hvs reapply dry-run") ||
               http_streq(cmd, "videocore display global reapply dryrun") ||
               http_streq(cmd, "videocore display global reapply dry-run") ||
               http_streq(cmd, "vc display global reapply") ||
               http_streq(cmd, "vc display hvs reapply") ||
               http_streq(cmd, "videocore display global reapply")) {
        bool dry = http_streq(cmd, "vc display global reapply dryrun") ||
                   http_streq(cmd, "vc display global reapply dry-run") ||
                   http_streq(cmd, "vc display hvs reapply dryrun") ||
                   http_streq(cmd, "vc display hvs reapply dry-run") ||
                   http_streq(cmd, "videocore display global reapply dryrun") ||
                   http_streq(cmd, "videocore display global reapply dry-run");
        bool ok = vc_display_global_reapply(dry);
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display global reapply OK " :
                                         "vc_display global reapply FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_global_status_name(d->global_reapply_status) : "none");
        http_append(out, &len, max, " control=");
        http_append_hex32(out, &len, max, d ? d->global_reapply_control : 0U);
        http_append(out, &len, max, " rb=");
        http_append_hex32(out, &len, max, d ? d->global_reapply_rb_control : 0U);
        http_append(out, &len, max, " readback=");
        http_append(out, &len, max, d && d->global_reapply_readback_ok ? "ok" : (dry ? "dryrun" : "bad"));
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display channel reapply dryrun") ||
               http_streq(cmd, "vc display channel reapply dry-run") ||
               http_streq(cmd, "videocore display channel reapply dryrun") ||
               http_streq(cmd, "videocore display channel reapply dry-run") ||
               http_streq(cmd, "vc display channel reapply") ||
               http_streq(cmd, "videocore display channel reapply")) {
        bool dry = http_streq(cmd, "vc display channel reapply dryrun") ||
                   http_streq(cmd, "vc display channel reapply dry-run") ||
                   http_streq(cmd, "videocore display channel reapply dryrun") ||
                   http_streq(cmd, "videocore display channel reapply dry-run");
        bool ok = vc_display_channel_reapply(dry);
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display channel reapply OK " :
                                         "vc_display channel reapply FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_channel_status_name(d->channel_reapply_status) : "none");
        http_append(out, &len, max, " ch=");
        http_append_u64(out, &len, max, d ? d->channel_reapply_channel : 0U);
        http_append(out, &len, max, " ctrl=");
        http_append_hex32(out, &len, max, d ? d->channel_reapply_ctrl0 : 0U);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, d ? d->channel_reapply_ctrl1 : 0U);
        http_append(out, &len, max, " cob=");
        http_append_hex32(out, &len, max, d ? d->channel_reapply_cob : 0U);
        http_append(out, &len, max, " rb=");
        http_append_hex32(out, &len, max, d ? d->channel_reapply_rb_ctrl0 : 0U);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, d ? d->channel_reapply_rb_ctrl1 : 0U);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, d ? d->channel_reapply_rb_cob : 0U);
        http_append(out, &len, max, " readback=");
        http_append(out, &len, max, d && d->channel_reapply_readback_ok ? "ok" : (dry ? "dryrun" : "bad"));
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display dlist stage") ||
               http_streq(cmd, "videocore display dlist stage")) {
        bool ok = vc_display_dlist_stage();
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display dlist stage OK " :
                                         "vc_display dlist stage FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_dlist_status_name(d->dlist_status) : "none");
        http_append(out, &len, max, " index=");
        http_append_u64(out, &len, max, d ? d->dlist_stage_index : 0U);
        http_append(out, &len, max, " count=");
        http_append_u64(out, &len, max, d ? d->dlist_stage_count : 0U);
        http_append(out, &len, max, " readback=");
        http_append(out, &len, max, d && d->dlist_stage_readback_ok ? "ok" : "bad");
        http_append(out, &len, max, "\n  readback=");
        if (d) {
            for (u32 i = 0; i < d->dlist_stage_count && i < 16U; i++) {
                if (i)
                    http_append(out, &len, max, ",");
                http_append_hex32(out, &len, max, d->dlist_readback[i]);
            }
        }
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display dlist arm dryrun") ||
               http_streq(cmd, "vc display dlist arm dry-run") ||
               http_streq(cmd, "videocore display dlist arm dryrun") ||
               http_streq(cmd, "videocore display dlist arm dry-run") ||
               http_streq(cmd, "vc display dlist arm") ||
               http_streq(cmd, "videocore display dlist arm")) {
        bool dry = http_streq(cmd, "vc display dlist arm dryrun") ||
                   http_streq(cmd, "vc display dlist arm dry-run") ||
                   http_streq(cmd, "videocore display dlist arm dryrun") ||
                   http_streq(cmd, "videocore display dlist arm dry-run");
        bool ok = vc_display_dlist_arm(dry);
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display dlist arm OK " :
                                         "vc_display dlist arm FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_dlist_status_name(d->dlist_status) : "none");
        http_append(out, &len, max, " ch=");
        http_append_u64(out, &len, max, d ? d->dlist_arm_channel : 0U);
        http_append(out, &len, max, " before=");
        http_append_hex32(out, &len, max, d ? d->dlist_arm_lptrs_before : 0U);
        http_append(out, &len, max, " after=");
        http_append_hex32(out, &len, max, d ? d->dlist_arm_lptrs_after : 0U);
        http_append(out, &len, max, " readback=");
        http_append(out, &len, max, d && d->dlist_arm_readback_ok ? "ok" : (dry ? "dryrun" : "bad"));
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display dlist dryrun") ||
               http_streq(cmd, "vc display dlist dry-run") ||
               http_streq(cmd, "videocore display dlist dryrun") ||
               http_streq(cmd, "videocore display dlist dry-run")) {
        bool ok = vc_display_dlist_dryrun();
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display dlist dryrun OK " :
                                         "vc_display dlist dryrun FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_dlist_status_name(d->dlist_status) : "none");
        http_append(out, &len, max, " count=");
        http_append_u64(out, &len, max, d ? d->dlist_count : 0U);
        http_append(out, &len, max, " format=");
        http_append_u64(out, &len, max, d ? d->dlist_format : 0U);
        http_append(out, &len, max, " order=");
        http_append_u64(out, &len, max, d ? d->dlist_order : 0U);
        http_append(out, &len, max, " scanout=");
        http_append_u64(out, &len, max, d ? d->scanout_base : 0U);
        http_append(out, &len, max, "\n  words=");
        if (d) {
            for (u32 i = 0; i < d->dlist_count && i < 16U; i++) {
                if (i)
                    http_append(out, &len, max, ",");
                http_append_hex32(out, &len, max, d->dlist_words[i]);
            }
        }
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display snapshot") ||
               http_streq(cmd, "videocore display snapshot")) {
        bool ok = vc_display_snapshot();
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display snapshot OK " :
                                         "vc_display snapshot FAILED ");
        http_append(out, &len, max, "count=");
        http_append_u64(out, &len, max, d ? d->snapshot_count : 0U);
        http_append(out, &len, max, " hvs_ver=");
        http_append_hex32(out, &len, max, d ? d->snapshot_hvs_version : 0U);
        http_append(out, &len, max, " hvs_id=");
        http_append_hex32(out, &len, max, d ? d->snapshot_hvs_id : 0U);
        http_append(out, &len, max, " control=");
        http_append_hex32(out, &len, max, d ? d->hvs_control : 0U);
        http_append(out, &len, max, " fetcher=");
        http_append_hex32(out, &len, max, d ? d->hvs_fetcher_status : 0U);
        http_append(out, &len, max, " fetch=");
        http_append_hex32(out, &len, max, d ? d->hvs_fetch_status : 0U);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, d ? d->hvs_handle_error : 0U);
        http_append(out, &len, max, " dlstat=");
        http_append_hex32(out, &len, max, d ? d->hvs_dl_status : 0U);
        http_append(out, &len, max, "\n");
        for (u32 ch = 0; ch < 3U; ch++) {
            http_append(out, &len, max, "  ch");
            http_append_u64(out, &len, max, ch);
            http_append(out, &len, max, " ctrl=");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_ctrl0[ch] : 0U);
            http_append(out, &len, max, "/");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_ctrl1[ch] : 0U);
            http_append(out, &len, max, " status=");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_status[ch] : 0U);
            http_append(out, &len, max, " dl=");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_dl[ch] : 0U);
            http_append(out, &len, max, " lptrs=");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_lptrs[ch] : 0U);
            http_append(out, &len, max, " cob=");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_cob[ch] : 0U);
            http_append(out, &len, max, " run=");
            http_append_hex32(out, &len, max, d ? d->hvs_disp_run[ch] : 0U);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "vc display takeover dryrun") ||
               http_streq(cmd, "vc display takeover dry-run") ||
               http_streq(cmd, "videocore display takeover dryrun") ||
               http_streq(cmd, "videocore display takeover dry-run")) {
        bool ok = vc_display_takeover_current(true);
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display takeover dryrun OK " :
                                         "vc_display takeover dryrun FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_takeover_status_name(d->last_takeover_status) : "none");
        http_append(out, &len, max, " backend=");
        http_append(out, &len, max, d ? vc_display_backend_name(d->backend) : "none");
        http_append(out, &len, max, " native_owner=");
        http_append(out, &len, max, d && d->native_owner ? "yes" : "no");
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display takeover") ||
               http_streq(cmd, "videocore display takeover")) {
        bool ok = vc_display_takeover_current(false);
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display takeover OK " :
                                         "vc_display takeover FAILED ");
        http_append(out, &len, max, "status=");
        http_append(out, &len, max, d ? vc_display_takeover_status_name(d->last_takeover_status) : "none");
        http_append(out, &len, max, " backend=");
        http_append(out, &len, max, d ? vc_display_backend_name(d->backend) : "none");
        http_append(out, &len, max, " native_owner=");
        http_append(out, &len, max, d && d->native_owner ? "yes" : "no");
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display fallback") ||
               http_streq(cmd, "videocore display fallback")) {
        bool ok = vc_display_fallback();
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, ok ? "vc_display fallback OK " :
                                         "vc_display fallback FAILED ");
        http_append(out, &len, max, "backend=");
        http_append(out, &len, max, d ? vc_display_backend_name(d->backend) : "none");
        http_append(out, &len, max, " native_owner=");
        http_append(out, &len, max, d && d->native_owner ? "yes" : "no");
        http_append(out, &len, max, " restore=");
        http_append_hex32(out, &len, max, d ? d->dlist_restore_lptrs : 0U);
        http_append(out, &len, max, "/");
        http_append(out, &len, max, d && d->dlist_restore_readback_ok ? "ok" : "none");
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "vc display") || http_streq(cmd, "videocore display") ||
               http_streq(cmd, "display status")) {
        const struct vc_display_status *d = vc_display_status_get();
        http_append(out, &len, max, "vc_display ready=");
        http_append(out, &len, max, d && d->ready ? "yes" : "no");
        http_append(out, &len, max, " backend=");
        http_append(out, &len, max, d ? vc_display_backend_name(d->backend) : "none");
        http_append(out, &len, max, " native_probe=");
        http_append(out, &len, max, d && d->native_probe_ready ? "yes" : "no");
        http_append(out, &len, max, " native_owner=");
        http_append(out, &len, max, d && d->native_owner ? "yes" : "no");
        http_append(out, &len, max, " hvs=");
        http_append(out, &len, max, d && d->hvs_seen ? "yes" : "no");
        http_append(out, &len, max, " v3d=");
        http_append(out, &len, max, d && d->v3d_seen ? "yes" : "no");
        http_append(out, &len, max, " mode=");
        http_append_u64(out, &len, max, d ? d->width : 0U);
        http_append(out, &len, max, "x");
        http_append_u64(out, &len, max, d ? d->height : 0U);
        http_append(out, &len, max, " pitch=");
        http_append_u64(out, &len, max, d ? d->pitch : 0U);
        http_append(out, &len, max, " scanout=");
        http_append_u64(out, &len, max, d ? d->scanout_base : 0U);
        http_append(out, &len, max, " presents=");
        http_append_u64(out, &len, max, d ? d->present_count : 0U);
        http_append(out, &len, max, " takeover=");
        http_append(out, &len, max, d ? vc_display_takeover_status_name(d->last_takeover_status) : "none");
        http_append(out, &len, max, " attempts=");
        http_append_u64(out, &len, max, d ? d->takeover_attempts : 0U);
        http_append(out, &len, max, " fallbacks=");
        http_append_u64(out, &len, max, d ? d->fallback_count : 0U);
        http_append(out, &len, max, " snapshots=");
        http_append_u64(out, &len, max, d ? d->snapshot_count : 0U);
        http_append(out, &len, max, " dlist=");
        http_append(out, &len, max, d ? vc_display_dlist_status_name(d->dlist_status) : "none");
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, d ? d->dlist_count : 0U);
        http_append(out, &len, max, " staged=");
        http_append_u64(out, &len, max, d ? d->dlist_stage_index : 0U);
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, d ? d->dlist_stage_count : 0U);
        http_append(out, &len, max, " arm=");
        http_append_hex32(out, &len, max, d ? d->dlist_arm_lptrs_after : 0U);
        http_append(out, &len, max, " restore=");
        http_append_hex32(out, &len, max, d ? d->dlist_restore_lptrs : 0U);
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, d ? d->dlist_restore_count : 0U);
        http_append(out, &len, max, " chan=");
        http_append(out, &len, max, d ? vc_display_channel_status_name(d->channel_reapply_status) : "none");
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, d ? d->channel_reapply_count : 0U);
        http_append(out, &len, max, " global=");
        http_append(out, &len, max, d ? vc_display_global_status_name(d->global_reapply_status) : "none");
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, d ? d->global_reapply_count : 0U);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tensor tiny noop") || http_streq(cmd, "qpu tiny noop")) {
        bool ok = tensor_tiny_noop_proof();
        http_append(out, &len, max, ok ? "Tensor tiny noop proof OK" : "Tensor tiny noop proof FAILED");
        http_append(out, &len, max, " stage=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_stage());
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_status());
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        http_append(out, &len, max, " csd_st=");
        http_append_hex32(out, &len, max, dbg.status_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_kick);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_wait);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, dbg.core_int_sts);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.hub_int_sts);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, dbg.err_stat);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tensor tiny store") || http_streq(cmd, "qpu tiny store") ||
               http_streq(cmd, "tensor tiny store-only") || http_streq(cmd, "qpu tiny store-only")) {
        bool ok = tensor_tiny_store_proof();
        http_append(out, &len, max, ok ? "Tensor tiny store proof OK" : "Tensor tiny store proof FAILED");
        http_append(out, &len, max, " stage=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_stage());
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_status());
        http_append(out, &len, max, " out=");
        http_append_u64(out, &len, max, tensor_tiny_last_output_bits());
        http_append(out, &len, max, " expect=");
        http_append_u64(out, &len, max, tensor_tiny_last_expected_bits());
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        http_append(out, &len, max, " csd_st=");
        http_append_hex32(out, &len, max, dbg.status_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_kick);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_wait);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, dbg.core_int_sts);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.hub_int_sts);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, dbg.err_stat);
        http_append(out, &len, max, " gmp=");
        http_append_hex32(out, &len, max, dbg.gmp_status);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.gmp_cfg);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.gmp_vio_addr);
        http_append(out, &len, max, " cur=");
        http_append_hex32(out, &len, max, dbg.current_cfg0);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.current_cfg5);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.current_cfg6);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tensor tiny loadstore") || http_streq(cmd, "qpu tiny loadstore") ||
               http_streq(cmd, "tensor tiny load-store") || http_streq(cmd, "qpu tiny load-store")) {
        bool ok = tensor_tiny_load_store_proof();
        http_append(out, &len, max, ok ? "Tensor tiny load-store proof OK" : "Tensor tiny load-store proof FAILED");
        http_append(out, &len, max, " stage=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_stage());
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_status());
        http_append(out, &len, max, " out=");
        http_append_u64(out, &len, max, tensor_tiny_last_output_bits());
        http_append(out, &len, max, " expect=");
        http_append_u64(out, &len, max, tensor_tiny_last_expected_bits());
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        http_append(out, &len, max, " csd_st=");
        http_append_hex32(out, &len, max, dbg.status_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_kick);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_wait);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, dbg.core_int_sts);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.hub_int_sts);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, dbg.err_stat);
        http_append(out, &len, max, " gmp=");
        http_append_hex32(out, &len, max, dbg.gmp_status);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.gmp_cfg);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.gmp_vio_addr);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tensor tiny memory") || http_streq(cmd, "qpu tiny memory") ||
               http_streq(cmd, "tensor tiny memory proof") || http_streq(cmd, "qpu tiny memory proof")) {
        bool ok = tensor_tiny_memory_proof();
        http_append(out, &len, max, ok ? "Tensor tiny memory proof OK" : "Tensor tiny memory proof FAILED");
        http_append(out, &len, max, " stage=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_stage());
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_status());
        http_append(out, &len, max, " out=");
        http_append_u64(out, &len, max, tensor_tiny_last_output_bits());
        http_append(out, &len, max, " expect=");
        http_append_u64(out, &len, max, tensor_tiny_last_expected_bits());
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        http_append(out, &len, max, " csd_st=");
        http_append_hex32(out, &len, max, dbg.status_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_kick);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_wait);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, dbg.core_int_sts);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.hub_int_sts);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, dbg.err_stat);
        http_append(out, &len, max, " gmp=");
        http_append_hex32(out, &len, max, dbg.gmp_status);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.gmp_cfg);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.gmp_vio_addr);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "tensor vector16") || http_streq(cmd, "qpu vector16") ||
               http_streq(cmd, "tensor vector16 selftest") || http_streq(cmd, "qpu vector16 selftest")) {
        bool ok = tensor_vector16_selftest();
        http_append(out, &len, max, ok ? "Tensor vector16 selftest OK" : "Tensor vector16 selftest FAILED");
        http_append_tensor_tail(out, &len, max);
    } else if (http_streq(cmd, "tensor vector128") || http_streq(cmd, "qpu vector128") ||
               http_streq(cmd, "tensor vectorN") || http_streq(cmd, "qpu vectorN")) {
        bool ok = tensor_vectorn_selftest(128U);
        http_append(out, &len, max, ok ? "Tensor vector128 selftest OK" : "Tensor vector128 selftest FAILED");
        http_append_tensor_tail(out, &len, max);
    } else if (http_streq(cmd, "tensor matvec128") || http_streq(cmd, "qpu matvec128")) {
        bool ok = tensor_matvec128_selftest();
        http_append(out, &len, max, ok ? "Tensor matvec128 selftest OK" : "Tensor matvec128 selftest FAILED");
        http_append_tensor_tail(out, &len, max);
    } else if (http_streq(cmd, "tensor bench") || http_streq(cmd, "qpu bench") ||
               http_streq(cmd, "tensor bench v3d") || http_streq(cmd, "qpu bench v3d")) {
        bool with_v3d = http_streq(cmd, "tensor bench v3d") ||
                        http_streq(cmd, "qpu bench v3d");
        http_append(out, &len, max,
                    with_v3d ? "tensor bench (scalar/neon/v3d, per-iter ns; gflops are wall-clock)\n"
                             : "tensor bench (scalar/neon, per-iter ns; add 'v3d' for GPU)\n");
        http_append(out, &len, max, "clk_mhz=");
        http_append_u64(out, &len, max, g_cpu_clock_mhz);
        http_append(out, &len, max, "\n");
        http_bench_row(out, &len, max, "add  n=1024", TENSOR_BENCH_ADD, 1U, 1024U, 1U, 1024ULL, 1000U, 32U, with_v3d);
        http_bench_row(out, &len, max, "mul  n=1024", TENSOR_BENCH_MUL, 1U, 1024U, 1U, 1024ULL, 1000U, 32U, with_v3d);
        http_bench_row(out, &len, max, "relu n=1024", TENSOR_BENCH_RELU, 1U, 1024U, 1U, 1024ULL, 1000U, 32U, with_v3d);
    } else if (http_streq(cmd, "tensor matmul64") || http_streq(cmd, "qpu matmul64")) {
        bool ok = tensor_matmul64_selftest();
        http_append(out, &len, max, ok ? "Tensor matmul64 selftest OK" : "Tensor matmul64 selftest FAILED");
        http_append_tensor_tail(out, &len, max);
    } else if (http_streq(cmd, "tensor tiny") || http_streq(cmd, "qpu tiny") ||
               http_streq(cmd, "tensor tiny selftest") || http_streq(cmd, "qpu tiny selftest")) {
        bool ok = tensor_tiny_selftest();
        http_append(out, &len, max, ok ? "Tensor tiny selftest OK" : "Tensor tiny selftest FAILED");
        http_append(out, &len, max, " stage=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_stage());
        http_append(out, &len, max, " status=");
        http_append_u64(out, &len, max, (u32)tensor_tiny_last_status());
        http_append(out, &len, max, " out=");
        http_append_u64(out, &len, max, tensor_tiny_last_output_bits());
        http_append(out, &len, max, " expect=");
        http_append_u64(out, &len, max, tensor_tiny_last_expected_bits());
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        http_append(out, &len, max, " csd_st=");
        http_append_hex32(out, &len, max, dbg.status_before);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_kick);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.status_after_wait);
        http_append(out, &len, max, " int=");
        http_append_hex32(out, &len, max, dbg.core_int_sts);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.hub_int_sts);
        http_append(out, &len, max, " err=");
        http_append_hex32(out, &len, max, dbg.err_stat);
        http_append(out, &len, max, " mmu=");
        http_append_hex32(out, &len, max, dbg.mmu_ctl);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.mmu_illegal_addr);
        http_append(out, &len, max, " vio=");
        http_append_hex32(out, &len, max, dbg.mmu_vio_addr);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.mmu_vio_id);
        http_append(out, &len, max, " cur=");
        http_append_hex32(out, &len, max, dbg.current_cfg0);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.current_cfg5);
        http_append(out, &len, max, "/");
        http_append_hex32(out, &len, max, dbg.current_cfg6);
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
    } else if (http_streq(cmd, "sts") || http_starts_with(cmd, "sts ")) {
        http_exec_sts_command(out, &len, max, http_streq(cmd, "sts") ? "" : cmd + 4);
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
    } else if (http_streq(cmd, "capsule") || http_starts_with(cmd, "capsule ")) {
        char *argv[10];
        u32 argc = http_split_args(cmd, argv, 10);
        if (argc < 2 || http_streq(argv[1], "help")) {
            http_append(out, &len, max,
                "capsule status <pack> | capsule realize <pack> | capsule list <pack> | capsule get|gethex <pack> <card>\n"
                "capsule puthex <pack> <card> <byteoffset> <hexbytes> | capsule del <pack> <card>\n"
                "capsule import <pack> <adapter_card> [manifest_rec source_rec bytecode_rec]\n");
        } else if (http_streq(argv[1], "status") || http_streq(argv[1], "manifest")) {
            u32 pack = 0;
            if (argc < 3 || !http_parse_u32(argv[2], &pack)) {
                http_append(out, &len, max, "ERR: usage capsule status <pack>\n");
            } else {
                struct capsule_manifest m;
                char err[64];
                if (!capsule_store_load_manifest(pack, &m, err, sizeof(err))) {
                    http_append(out, &len, max, "ERR: ");
                    http_append(out, &len, max, err);
                    http_append(out, &len, max, "\n");
                } else {
                    http_append(out, &len, max, "capsule pack=");
                    http_append_u64(out, &len, max, pack);
                    http_append(out, &len, max, " name=");
                    http_append(out, &len, max, m.name);
                    http_append(out, &len, max, " cards=");
                    http_append_u64(out, &len, max, m.cards_lo);
                    http_append(out, &len, max, "-");
                    http_append_u64(out, &len, max, m.cards_hi);
                    http_append(out, &len, max, " processes=");
                    http_append_u64(out, &len, max, m.process_count);
                    http_append(out, &len, max, " fifos=");
                    http_append_u64(out, &len, max, m.fifo_count);
                    http_append(out, &len, max, "\n");
                    for (u32 i = 0; i < m.process_count; i++) {
                        struct capsule_process *p = &m.processes[i];
                        http_append(out, &len, max, "process ");
                        http_append(out, &len, max, p->name);
                        http_append(out, &len, max, " source=");
                        http_append_u64(out, &len, max, p->source);
                        http_append(out, &len, max, " bytecode=");
                        http_append_u64(out, &len, max, p->bytecode);
                        if (p->io[0]) {
                            http_append(out, &len, max, " io=");
                            http_append(out, &len, max, p->io);
                        }
                        if (p->entry[0]) {
                            http_append(out, &len, max, " entry=");
                            http_append(out, &len, max, p->entry);
                        }
                        http_append(out, &len, max, "\n");
                    }
                    for (u32 i = 0; i < m.fifo_count; i++) {
                        struct capsule_fifo *f = &m.fifos[i];
                        http_append(out, &len, max, "ipc_fifo ");
                        http_append(out, &len, max, f->name);
                        http_append(out, &len, max, " from=");
                        http_append(out, &len, max, f->from);
                        http_append(out, &len, max, " to=");
                        http_append(out, &len, max, f->to);
                        http_append(out, &len, max, " depth=");
                        http_append_u64(out, &len, max, f->depth);
                        http_append(out, &len, max, " frame_max=");
                        http_append_u64(out, &len, max, f->frame_max);
                        http_append(out, &len, max, "\n");
                    }
                }
            }
        } else if (http_streq(argv[1], "realize")) {
            u32 pack = 0;
            if (argc < 3 || !http_parse_u32(argv[2], &pack)) {
                http_append(out, &len, max, "ERR: usage capsule realize <pack>\n");
            } else {
                struct capsule_manifest m;
                char err[64];
                if (!capsule_store_load_manifest(pack, &m, err, sizeof(err))) {
                    http_append(out, &len, max, "ERR: ");
                    http_append(out, &len, max, err);
                    http_append(out, &len, max, "\n");
                } else {
                    u32 wrote = 0;
                    for (u32 i = 0; i < m.fifo_count; i++) {
                        struct capsule_fifo *f = &m.fifos[i];
                        char desc[256];
                        u32 dl = 0;
                        http_append(desc, &dl, sizeof(desc), "ipc_fifo = ");
                        http_append(desc, &dl, sizeof(desc), f->name);
                        http_append(desc, &dl, sizeof(desc), "\n  from = ");
                        http_append(desc, &dl, sizeof(desc), f->from);
                        http_append(desc, &dl, sizeof(desc), "\n  to = ");
                        http_append(desc, &dl, sizeof(desc), f->to);
                        http_append(desc, &dl, sizeof(desc), "\n  depth = ");
                        http_append_u64(desc, &dl, sizeof(desc), f->depth);
                        http_append(desc, &dl, sizeof(desc), "\n  frame_max = ");
                        http_append_u64(desc, &dl, sizeof(desc), f->frame_max);
                        http_append(desc, &dl, sizeof(desc), "\n");
                        if (dl > 0 && capsule_store_write(pack, 20000U + i, desc, dl))
                            wrote++;
                    }
                    http_append(out, &len, max, "capsule realize pack=");
                    http_append_u64(out, &len, max, pack);
                    http_append(out, &len, max, " fifo_cards=");
                    http_append_u64(out, &len, max, wrote);
                    http_append(out, &len, max, "\n");
                }
            }
        } else if (http_streq(argv[1], "list")) {
            u32 pack = 0;
            if (argc < 3 || !http_parse_u32(argv[2], &pack)) {
                http_append(out, &len, max, "ERR: usage capsule list <pack>\n");
            } else {
                u32 cards[64];
                u32 n = capsule_store_list(pack, cards, 64);
                http_append(out, &len, max, "capsule pack=");
                http_append_u64(out, &len, max, pack);
                http_append(out, &len, max, " count=");
                http_append_u64(out, &len, max, n);
                http_append(out, &len, max, "\n");
                for (u32 i = 0; i < n; i++) {
                    http_append(out, &len, max, "card=");
                    http_append_u64(out, &len, max, cards[i]);
                    http_append(out, &len, max, " role=");
                    http_append(out, &len, max, capsule_card_role(cards[i]));
                    http_append(out, &len, max, "\n");
                }
            }
        } else if (http_streq(argv[1], "get") || http_streq(argv[1], "gethex")) {
            u32 pack = 0, card = 0;
            if (argc < 4 || !http_parse_u32(argv[2], &pack) || !http_parse_u32(argv[3], &card)) {
                http_append(out, &len, max, "ERR: usage capsule get|gethex <pack> <card>\n");
            } else {
                static u8 cbuf[CAPSULE_STORE_CARD_MAX_BYTES];
                i32 n = capsule_store_read(pack, card, cbuf, sizeof(cbuf));
                if (n < 0) {
                    http_append(out, &len, max, "ERR: capsule card read failed\n");
                } else {
                    http_append(out, &len, max, "capsule pack=");
                    http_append_u64(out, &len, max, pack);
                    http_append(out, &len, max, " card=");
                    http_append_u64(out, &len, max, card);
                    http_append(out, &len, max, " bytes=");
                    http_append_u64(out, &len, max, (u32)n);
                    http_append(out, &len, max, "\n");
                    if (http_streq(argv[1], "gethex")) {
                        for (i32 i = 0; i < n; i++) http_append_hex8(out, &len, max, cbuf[i]);
                    } else {
                        http_append_sanitized_bytes(out, &len, max, cbuf, (u32)n);
                    }
                    http_append(out, &len, max, "\n");
                }
            }
        } else if (http_streq(argv[1], "puthex")) {
            u32 pack = 0, card = 0, off = 0;
            if (argc < 6 || !http_parse_u32(argv[2], &pack) || !http_parse_u32(argv[3], &card) ||
                !http_parse_u32(argv[4], &off)) {
                http_append(out, &len, max, "ERR: usage capsule puthex <pack> <card> <byteoffset> <hexbytes>\n");
            } else {
                static u8 cbuf[CAPSULE_STORE_CARD_MAX_BYTES];
                u32 old_len = 0;
                if (off != 0) {
                    i32 got = capsule_store_read(pack, card, cbuf, sizeof(cbuf));
                    if (got > 0) old_len = (u32)got;
                }
                for (u32 z = old_len; z < off && z < sizeof(cbuf); z++) cbuf[z] = 0;
                const char *hex = argv[5];
                u32 hex_len = pios_strlen(hex);
                bool ok = (hex_len != 0 && (hex_len & 1U) == 0);
                u32 count = hex_len / 2U;
                if (off > sizeof(cbuf) || count > sizeof(cbuf) - off) ok = false;
                for (u32 i = 0; ok && i < count; i++)
                    ok = pixe_parse_hex_byte_pair(hex + i * 2U, &cbuf[off + i]);
                if (!ok) {
                    http_append(out, &len, max, "ERR: bad hex/range\n");
                } else {
                    u32 new_len = off + count;
                    if (new_len < old_len) new_len = old_len;
                    if (!capsule_store_write(pack, card, cbuf, new_len)) {
                        http_append(out, &len, max, "ERR: capsule card write failed\n");
                    } else {
                        http_append(out, &len, max, "OK: capsule puthex pack=");
                        http_append_u64(out, &len, max, pack);
                        http_append(out, &len, max, " card=");
                        http_append_u64(out, &len, max, card);
                        http_append(out, &len, max, " bytes=");
                        http_append_u64(out, &len, max, new_len);
                        http_append(out, &len, max, "\n");
                    }
                }
            }
        } else if (http_streq(argv[1], "del")) {
            u32 pack = 0, card = 0;
            if (argc < 4 || !http_parse_u32(argv[2], &pack) || !http_parse_u32(argv[3], &card))
                http_append(out, &len, max, "ERR: usage capsule del <pack> <card>\n");
            else
                http_append(out, &len, max, capsule_store_delete(pack, card) ? "OK: deleted\n" : "ERR: delete failed\n");
        } else if (http_streq(argv[1], "import")) {
            u32 pack = 0, adapter = 0, manifest_rec = 10, source_rec = capsule_source_for(1), code_rec = capsule_code_for(1);
            if (argc < 4 || !http_parse_u32(argv[2], &pack) || !http_parse_u32(argv[3], &adapter) ||
                adapter > PICOWAL_CARD_MAX) {
                http_append(out, &len, max, "ERR: usage capsule import <pack> <adapter_card> [manifest_rec source_rec bytecode_rec]\n");
            } else {
                if (argc >= 5) (void)http_parse_u32(argv[4], &manifest_rec);
                if (argc >= 6) (void)http_parse_u32(argv[5], &source_rec);
                if (argc >= 7) (void)http_parse_u32(argv[6], &code_rec);
                static u8 ibuf[PICOWAL_DATA_MAX];
                u32 wrote = 0;
                i32 n = picowal_db_get((u16)adapter, manifest_rec, ibuf, sizeof(ibuf));
                if (n > 0 && capsule_store_write(pack, 0, ibuf, (u32)n)) wrote++;
                n = picowal_db_get((u16)adapter, source_rec, ibuf, sizeof(ibuf));
                if (n > 0 && capsule_store_write(pack, capsule_source_for(1), ibuf, (u32)n)) wrote++;
                n = picowal_db_get((u16)adapter, code_rec, ibuf, sizeof(ibuf));
                if (n <= 0 && code_rec != 0)
                    n = picowal_db_get((u16)adapter, 0, ibuf, sizeof(ibuf));
                if (n > 0 && capsule_store_write(pack, capsule_code_for(1), ibuf, (u32)n)) wrote++;
                http_append(out, &len, max, "capsule import pack=");
                http_append_u64(out, &len, max, pack);
                http_append(out, &len, max, " adapter=");
                http_append_u64(out, &len, max, adapter);
                http_append(out, &len, max, " wrote=");
                http_append_u64(out, &len, max, wrote);
                http_append(out, &len, max, " manifest=");
                http_append_u64(out, &len, max, manifest_rec);
                http_append(out, &len, max, " source=");
                http_append_u64(out, &len, max, source_rec);
                http_append(out, &len, max, " bytecode=");
                http_append_u64(out, &len, max, code_rec);
                if (code_rec != 0)
                    http_append(out, &len, max, " fallback=0");
                http_append(out, &len, max, "\n");
            }
        } else {
            http_append(out, &len, max, "ERR: unknown capsule subcommand\n");
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
    } else if (http_streq(cmd, "walfs verify") || http_streq(cmd, "disk verify") ||
               http_streq(cmd, "fs verify")) {
        struct walfs_health h;
        bool ok = walfs_verify(&h);
        http_append(out, &len, max, "walfs verify ok=");
        http_append(out, &len, max, ok ? "yes" : "no");
        http_append(out, &len, max, " super=");
        http_append(out, &len, max, h.super_ok ? "ok" : "bad");
        http_append(out, &len, max, " wal_head=");
        http_append(out, &len, max, h.wal_head_ok ? "ok" : "bad");
        http_append(out, &len, max, " valid_records=");
        http_append_u64(out, &len, max, h.valid_records);
        http_append(out, &len, max, " crc_errors=");
        http_append_u64(out, &len, max, h.crc_errors);
        http_append(out, &len, max, " header_errors=");
        http_append_u64(out, &len, max, h.header_errors);
        http_append(out, &len, max, " open_tx=");
        http_append(out, &len, max, h.open_tx ? "yes" : "no");
        http_append(out, &len, max, " scan_end=");
        http_append_u64(out, &len, max, h.scan_end);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "walfs sync") || http_streq(cmd, "disk sync")) {
        walfs_sync();
        struct walfs_health h;
        bool ok = walfs_verify(&h);
        http_append(out, &len, max, "walfs sync ok=");
        http_append(out, &len, max, ok ? "yes" : "no");
        http_append(out, &len, max, " super=");
        http_append(out, &len, max, h.super_ok ? "ok" : "bad");
        http_append(out, &len, max, " wal_head=");
        http_append(out, &len, max, h.wal_head_ok ? "ok" : "bad");
        http_append(out, &len, max, " valid_records=");
        http_append_u64(out, &len, max, h.valid_records);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "walfs compact") || http_streq(cmd, "disk compact")) {
        bool ok = walfs_compact();
        http_append(out, &len, max, ok ? "walfs compact OK\n" : "walfs compact FAILED\n");
        if (ok) {
            struct walfs_status_snapshot ws;
            walfs_status(&ws);
            http_append(out, &len, max, "new head=");
            http_append_u64(out, &len, max, ws.super_head);
            http_append(out, &len, max, " records=");
            http_append_u64(out, &len, max, ws.super_records);
            http_append(out, &len, max, "\n");
        }
    } else if (http_streq(cmd, "watchdog") || http_streq(cmd, "watchdog status")) {
        struct watchdog_status st;
        watchdog_status(&st);
        http_append(out, &len, max, "watchdog armed=");
        http_append(out, &len, max, st.armed ? "yes" : "no");
        http_append(out, &len, max, " mode=");
        http_append(out, &len, max, st.reboot_on_trip ? "reboot" : "halt");
        http_append(out, &len, max, " timeout_ticks=");
        http_append_u64(out, &len, max, st.timeout_ticks);
        http_append(out, &len, max, " trips=");
        http_append_u64(out, &len, max, st.trip_count);
        http_append(out, &len, max, " last_core=");
        http_append_u64(out, &len, max, st.last_trip_core);
        http_append(out, &len, max, " hw_remaining_ticks=");
        http_append_u64(out, &len, max, watchdog_hw_remaining_ticks());
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "crypto selftest") || http_streq(cmd, "crypto")) {
        bool ok = crypto_selftest();
        http_append(out, &len, max, ok ? "crypto selftest OK (AES-GCM + GHASH nibble table)\n"
                                       : "crypto selftest FAILED\n");
    } else if (http_streq(cmd, "arp probe")) {
        arp_probe();
        const arp_stats_t *ast = arp_get_stats();
        http_append(out, &len, max, "arp probe sent requests_sent=");
        http_append_u64(out, &len, max, ast->requests_sent);
        http_append(out, &len, max, " learned=");
        http_append_u64(out, &len, max, ast->learned);
        http_append(out, &len, max, " conflicts=");
        http_append_u64(out, &len, max, ast->conflicts);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "nic dump on") || http_streq(cmd, "nic dump off")) {
        bool on = http_streq(cmd, "nic dump on");
        nic_set_packet_dump(on);
        http_append(out, &len, max, on ? "nic packet dump ENABLED\n" : "nic packet dump disabled\n");
    } else if (http_streq(cmd, "nic counters")) {
        nic_packet_counters_t c;
        nic_packet_counters(&c);
        http_append(out, &len, max, "nic processed=");
        http_append_u64(out, &len, max, c.processed);
        http_append(out, &len, max, " dropped=");
        http_append_u64(out, &len, max, c.dropped);
        http_append(out, &len, max, " firewalled=");
        http_append_u64(out, &len, max, c.firewalled);
        http_append(out, &len, max, " rate_limited=");
        http_append_u64(out, &len, max, c.rate_limited);
        http_append(out, &len, max, " rx_arp_not_us=");
        http_append_u64(out, &len, max, c.rx_arp_not_us);
        http_append(out, &len, max, " flood_blocked=");
        http_append_u64(out, &len, max, c.flood_blocked);
        /* Direction-specific totals avoid the legacy mixed RX/TX counters. */
        http_append(out, &len, max, " rx_total=");
        http_append_u64(out, &len, max, c.rx_total);
        http_append(out, &len, max, " tx_total=");
        http_append_u64(out, &len, max, c.tx_total);
        http_append(out, &len, max, " tx_csum_hw=");
        http_append_u64(out, &len, max, c.tx_csum_offloaded);
        http_append(out, &len, max, " tx_csum_sw=");
        http_append_u64(out, &len, max, c.tx_csum_software);
        http_append(out, &len, max, " rx_csum_trusted=");
        http_append_u64(out, &len, max, c.rx_csum_trusted);
        http_append(out, &len, max, " rx_csum_untrusted=");
        http_append_u64(out, &len, max, c.rx_csum_untrusted);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "rxdiag")) {
        nic_packet_counters_t nc;
        const net_stats_t *ns = net_get_stats();
        nic_packet_counters(&nc);

#if PIOS_HAS_GENET
        struct macb_diag md;
        macb_diag(&md);
        http_append(out, &len, max, "MAC  rx_recv=");
        http_append_u64(out, &len, max, md.rx_recv);
        http_append(out, &len, max, " owned=");
        http_append_u64(out, &len, max, md.rx_owned);
        http_append(out, &len, max, "/");
        http_append_u64(out, &len, max, md.ring_size);
        http_append(out, &len, max, " recover=");
        http_append_u64(out, &len, max, md.rx_recover);
        http_append(out, &len, max, " live_recover=");
        http_append_u64(out, &len, max, md.rx_live_recover);
        http_append(out, &len, max, " hole_recover=");
        http_append_u64(out, &len, max, md.rx_hole_recover);
        http_append(out, &len, max, " contig=");
        http_append_u64(out, &len, max, md.rx_contig_owned);
        http_append(out, &len, max, " after_gap=");
        http_append_u64(out, &len, max, md.rx_owned_after_gap);
#else
        http_append(out, &len, max, "MAC  unavailable (non-GEM platform)");
#endif
        http_append(out, &len, max, "\nNIC  rx_accept=");
        http_append_u64(out, &len, max, nc.rx_total);
        http_append(out, &len, max, " arp_early_drop=");
        http_append_u64(out, &len, max, nc.rx_arp_not_us);
        http_append(out, &len, max, " filter_drop=");
        http_append_u64(out, &len, max, nc.rx_filter_drop);
        http_append(out, &len, max, "\nNET  ingress=");
        http_append_u64(out, &len, max, ns->rx_packets);
        http_append(out, &len, max, " dispatched=");
        http_append_u64(out, &len, max, ns->rx_dispatched);
        http_append(out, &len, max, " unsupported=");
        http_append_u64(out, &len, max, ns->rx_unsupported);
        http_append(out, &len, max, " polls=");
        http_append_u64(out, &len, max, ns->poll_calls);
        http_append(out, &len, max, " empty=");
        http_append_u64(out, &len, max, ns->poll_empty);
        http_append(out, &len, max, " budget_hits=");
        http_append_u64(out, &len, max, ns->poll_budget_hits);
        http_append(out, &len, max, " last_frames=");
        http_append_u64(out, &len, max, ns->poll_last_frames);
        http_append(out, &len, max, "\nIRQ  eth=");
        http_append_u64(out, &len, max, core0_eth_irq_count);
        http_append(out, &len, max, " pending_flags=");
        http_append_hex32(out, &len, max, core0_io_flags);
        http_append(out, &len, max, " deferred=");
        http_append_u64(out, &len, max, core0_eth_irq_deferred_quench ? 1U : 0U);
        http_append(out, &len, max, " quench_passes=");
        http_append_u64(out, &len, max, core0_eth_irq_quench_passes);
        http_append(out, &len, max, " stall_streak=");
        http_append_u64(out, &len, max, core0_eth_irq_stall_streak);
        http_append(out, &len, max, " fallback=");
        http_append_u64(out, &len, max, core0_eth_irq_poll_fallback ? 1U : 0U);
        http_append(out, &len, max, " fallback_count=");
        http_append_u64(out, &len, max, core0_eth_irq_fallback_count);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "arp status")) {
        /* Full ARP subsystem diagnostics: reply/request rate-limit drops
         * (arp.c's ARP_REPLY_INTERVAL_MS is a GLOBAL, not per-source, 100ms
         * window -- drop_ratelimit climbing under concurrent LAN ARP traffic
         * is the leading suspect for the "board unreachable but MAC/DMA and
         * firewall both look healthy" wedge symptom), spoof drops, and
         * conflicts, alongside the early nic.c broadcast-not-for-us filter
         * counter (rx_arp_not_us above in 'nic counters') for the OTHER
         * candidate explanation. Both counters together let a single
         * 'inspect' snapshot distinguish which path is actually dropping
         * legitimate ARP traffic during a live wedge. */
        const arp_stats_t *a = arp_get_stats();
        http_append(out, &len, max, "arp requests_sent=");
        http_append_u64(out, &len, max, a->requests_sent);
        http_append(out, &len, max, " replies_sent=");
        http_append_u64(out, &len, max, a->replies_sent);
        http_append(out, &len, max, " learned=");
        http_append_u64(out, &len, max, a->learned);
        http_append(out, &len, max, " drop_spoof=");
        http_append_u64(out, &len, max, a->drop_spoof);
        http_append(out, &len, max, " drop_ratelimit=");
        http_append_u64(out, &len, max, a->drop_ratelimit);
        http_append(out, &len, max, " conflicts=");
        http_append_u64(out, &len, max, a->conflicts);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "break") || http_starts_with(cmd, "break ")) {
        /* Stop-the-world debug freeze: request a freeze on every OTHER core
         * (never core_id() itself -- freezing your own console core would
         * strand the very command session you're using to inspect/resume
         * it). "break <core>" targets a single core instead. The target
         * actually stops on whatever interrupt next fires there (typically
         * the periodic timer) -- see irq_dispatch() in exception.c. */
        u32 self = core_id() & 3U;
        char *sp = cmd + 5;
        while (*sp == ' ') sp++;
        u32 target = 0xFFFFFFFFU;
        if (*sp)
            (void)http_parse_u32(sp, &target);
        u32 requested = 0;
        for (u32 core = 0; core < DEBUG_FREEZE_MAX_CORES; core++) {
            if (core == self) continue;
            if (target != 0xFFFFFFFFU && core != target) continue;
            debug_freeze_request(core);
            requested++;
        }
        http_append(out, &len, max, "break: requested freeze on ");
        http_append_u64(out, &len, max, requested);
        http_append(out, &len, max, " core(s) (self=core");
        http_append_u64(out, &len, max, self);
        http_append(out, &len, max, " stays live); poll 'freeze status' until frozen=1, then 'regs <core>'\n");
    } else if (http_streq(cmd, "resume") || http_starts_with(cmd, "resume ")) {
        char *sp = cmd + 6;
        while (*sp == ' ') sp++;
        u32 target = 0xFFFFFFFFU;
        if (*sp)
            (void)http_parse_u32(sp, &target);
        u32 cleared = 0;
        for (u32 core = 0; core < DEBUG_FREEZE_MAX_CORES; core++) {
            if (target != 0xFFFFFFFFU && core != target) continue;
            debug_freeze_clear(core);
            cleared++;
        }
        http_append(out, &len, max, "resume: cleared freeze request on ");
        http_append_u64(out, &len, max, cleared);
        http_append(out, &len, max, " core(s)\n");
    } else if (http_streq(cmd, "freeze status")) {
        for (u32 core = 0; core < DEBUG_FREEZE_MAX_CORES; core++) {
            http_append(out, &len, max, "core");
            http_append_u64(out, &len, max, core);
            http_append(out, &len, max, ": requested=");
            http_append(out, &len, max, debug_freeze_is_requested(core) ? "1" : "0");
            http_append(out, &len, max, " frozen=");
            http_append(out, &len, max, debug_freeze_is_frozen(core) ? "1" : "0");
            http_append(out, &len, max, "\n");
        }
    } else if (http_starts_with(cmd, "regs")) {
        char *sp = cmd + 4;
        while (*sp == ' ') sp++;
        u32 core = 0;
        if (!*sp || !http_parse_u32(sp, &core)) {
            http_append(out, &len, max, "usage: regs <core>\n");
        } else {
            struct debug_freeze_slot snap;
            if (!debug_freeze_snapshot(core, &snap)) {
                http_append(out, &len, max, "core");
                http_append_u64(out, &len, max, core);
                http_append(out, &len, max, " is not frozen (use 'break' first, then wait for frozen=1)\n");
            } else {
                http_append(out, &len, max, "core");
                http_append_u64(out, &len, max, core);
                http_append(out, &len, max, " frozen at tick=");
                http_append_u64(out, &len, max, snap.freeze_tick);
                http_append(out, &len, max, " last_intid=");
                http_append_u64(out, &len, max, snap.last_intid);
                http_append(out, &len, max, "\nelr=");
                http_append_hex64(out, &len, max, snap.elr);
                http_append(out, &len, max, " spsr=");
                http_append_hex64(out, &len, max, snap.spsr);
                http_append(out, &len, max, "\n");
                for (u32 i = 0; i < 31U; i++) {
                    http_append(out, &len, max, "x");
                    http_append_u64(out, &len, max, i);
                    http_append(out, &len, max, "=");
                    http_append_hex64(out, &len, max, snap.x[i]);
                    http_append(out, &len, max, (i % 4U == 3U) ? "\n" : " ");
                }
                http_append(out, &len, max, "\n");
            }
        }
    } else if (http_streq(cmd, "nic offload")) {
        nic_offload_status_t s;
        nic_offload_status(&s);
        http_append(out, &len, max, "nic offload tx_csum_cap=");
        http_append(out, &len, max, s.tx_checksum_capable ? "yes" : "no");
        http_append(out, &len, max, " tx_csum=");
        http_append(out, &len, max, s.tx_checksum_enabled ? "on" : "off");
        http_append(out, &len, max, " rx_csum_cap=");
        http_append(out, &len, max, s.rx_checksum_capable ? "yes" : "no");
        http_append(out, &len, max, " rx_csum=");
        http_append(out, &len, max, s.rx_checksum_enabled ? "on" : "off");
        http_append(out, &len, max, " tso_cap=");
        http_append(out, &len, max, s.tso_capable ? "yes" : "no");
        http_append(out, &len, max, " tso=");
        http_append(out, &len, max, s.tso_enabled ? "on" : "off");
        http_append(out, &len, max, " tx_hw=");
        http_append_u64(out, &len, max, s.tx_csum_offloaded);
        http_append(out, &len, max, " tx_sw=");
        http_append_u64(out, &len, max, s.tx_csum_software);
        http_append(out, &len, max, " rx_trusted=");
        http_append_u64(out, &len, max, s.rx_csum_trusted);
        http_append(out, &len, max, " rx_untrusted=");
        http_append_u64(out, &len, max, s.rx_csum_untrusted);
        http_append(out, &len, max, " ncfgr=");
        http_append_hex32(out, &len, max, s.mac_ncfgr);
        http_append(out, &len, max, " dmacfg=");
        http_append_hex32(out, &len, max, s.mac_dmacfg);
        http_append(out, &len, max, "\n");
    } else if (http_streq(cmd, "cachestats")) {
        u64 ih = 0, im = 0, ph = 0, pm = 0, dh = 0, dm = 0, dev = 0, ah = 0, am = 0, aev = 0;
        walfs_cache_stats(&ih, &im, &ph, &pm);
        dns_cache_stats(&dh, &dm, &dev);
        arp_cache_stats(&ah, &am, &aev);
        http_append(out, &len, max, "cache walfs inode_hit=");
        http_append_u64(out, &len, max, ih);
        http_append(out, &len, max, " inode_miss=");
        http_append_u64(out, &len, max, im);
        http_append(out, &len, max, " path_hit=");
        http_append_u64(out, &len, max, ph);
        http_append(out, &len, max, " path_miss=");
        http_append_u64(out, &len, max, pm);
        http_append(out, &len, max, "\ncache dns hit=");
        http_append_u64(out, &len, max, dh);
        http_append(out, &len, max, " miss=");
        http_append_u64(out, &len, max, dm);
        http_append(out, &len, max, " evict=");
        http_append_u64(out, &len, max, dev);
        http_append(out, &len, max, "\ncache arp hit=");
        http_append_u64(out, &len, max, ah);
        http_append(out, &len, max, " miss=");
        http_append_u64(out, &len, max, am);
        http_append(out, &len, max, " evict=");
        http_append_u64(out, &len, max, aev);
        http_append(out, &len, max, "\n");
    } else {
        http_append(out, &len, max, "unknown command\n");
    }
    *len_ptr = len;
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
    else
        http_exec_terminal_command(out, &len, max, cmd);
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

static bool http_capsule_query_u32(const u8 *req, u32 req_len, const char *key, u32 *out)
{
    char tmp[16];
    return http_query_value(req, req_len, "/api/capsule", key, tmp, sizeof(tmp)) &&
           http_parse_u32(tmp, out);
}

static void http_capsule_json_error(char *out, u32 *len, u32 max, const char *msg)
{
    http_append(out, len, max, "{\"ok\":false,\"error\":");
    http_append_json_string(out, len, max, msg);
    http_append(out, len, max, "}\n");
}

static void http_append_json_hex_bytes(char *out, u32 *len, u32 max, const u8 *data, u32 n)
{
    http_append(out, len, max, "\"");
    for (u32 i = 0; i < n; i++)
        http_append_hex8(out, len, max, data[i]);
    http_append(out, len, max, "\"");
}

static void http_capsule_append_manifest_json(char *out, u32 *len, u32 max,
                                              u32 pack, const struct capsule_manifest *m)
{
    http_append(out, len, max, "{\"ok\":true,\"op\":\"status\",\"pack\":");
    http_append_u64(out, len, max, pack);
    http_append(out, len, max, ",\"name\":");
    http_append_json_string(out, len, max, m->name);
    http_append(out, len, max, ",\"cardsLo\":");
    http_append_u64(out, len, max, m->cards_lo);
    http_append(out, len, max, ",\"cardsHi\":");
    http_append_u64(out, len, max, m->cards_hi);
    http_append(out, len, max, ",\"processes\":[");
    for (u32 i = 0; i < m->process_count; i++) {
        const struct capsule_process *p = &m->processes[i];
        if (i) http_append(out, len, max, ",");
        http_append(out, len, max, "{\"name\":");
        http_append_json_string(out, len, max, p->name);
        http_append(out, len, max, ",\"source\":");
        http_append_u64(out, len, max, p->source);
        http_append(out, len, max, ",\"bytecode\":");
        http_append_u64(out, len, max, p->bytecode);
        http_append(out, len, max, ",\"io\":");
        http_append_json_string(out, len, max, p->io);
        http_append(out, len, max, ",\"entry\":");
        http_append_json_string(out, len, max, p->entry);
        http_append(out, len, max, "}");
    }
    http_append(out, len, max, "],\"fifos\":[");
    for (u32 i = 0; i < m->fifo_count; i++) {
        const struct capsule_fifo *f = &m->fifos[i];
        if (i) http_append(out, len, max, ",");
        http_append(out, len, max, "{\"name\":");
        http_append_json_string(out, len, max, f->name);
        http_append(out, len, max, ",\"from\":");
        http_append_json_string(out, len, max, f->from);
        http_append(out, len, max, ",\"to\":");
        http_append_json_string(out, len, max, f->to);
        http_append(out, len, max, ",\"depth\":");
        http_append_u64(out, len, max, f->depth);
        http_append(out, len, max, ",\"frameMax\":");
        http_append_u64(out, len, max, f->frame_max);
        http_append(out, len, max, "}");
    }
    http_append(out, len, max, "]}\n");
}

static u32 http_build_capsule_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char op[16];
    u32 pack = 0;
    u32 card = 0;
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");

    if (!http_query_value(req, req_len, "/api/capsule", "op", op, sizeof(op))) {
        u32 o = 0;
        http_append(op, &o, sizeof(op), "status");
    }
    if (!http_capsule_query_u32(req, req_len, "pack", &pack)) {
        http_capsule_json_error(out, &len, max, "missing pack");
        return len;
    }

    if (http_streq(op, "status") || http_streq(op, "manifest")) {
        struct capsule_manifest m;
        char err[64];
        if (!capsule_store_load_manifest(pack, &m, err, sizeof(err))) {
            http_capsule_json_error(out, &len, max, err);
            return len;
        }
        http_capsule_append_manifest_json(out, &len, max, pack, &m);
        return len;
    }

    if (http_streq(op, "list")) {
        u32 cards[96];
        u32 n = capsule_store_list(pack, cards, 96);
        http_append(out, &len, max, "{\"ok\":true,\"op\":\"list\",\"pack\":");
        http_append_u64(out, &len, max, pack);
        http_append(out, &len, max, ",\"cards\":[");
        for (u32 i = 0; i < n; i++) {
            if (i) http_append(out, &len, max, ",");
            http_append(out, &len, max, "{\"card\":");
            http_append_u64(out, &len, max, cards[i]);
            http_append(out, &len, max, ",\"role\":");
            http_append_json_string(out, &len, max, capsule_card_role(cards[i]));
            http_append(out, &len, max, "}");
        }
        http_append(out, &len, max, "],\"count\":");
        http_append_u64(out, &len, max, n);
        http_append(out, &len, max, "}\n");
        return len;
    }

    if (http_streq(op, "realize")) {
        struct capsule_manifest m;
        char err[64];
        if (!capsule_store_load_manifest(pack, &m, err, sizeof(err))) {
            http_capsule_json_error(out, &len, max, err);
            return len;
        }
        u32 wrote = 0;
        for (u32 i = 0; i < m.fifo_count; i++) {
            const struct capsule_fifo *f = &m.fifos[i];
            char desc[256];
            u32 dl = 0;
            http_append(desc, &dl, sizeof(desc), "ipc_fifo = ");
            http_append(desc, &dl, sizeof(desc), f->name);
            http_append(desc, &dl, sizeof(desc), "\n  from = ");
            http_append(desc, &dl, sizeof(desc), f->from);
            http_append(desc, &dl, sizeof(desc), "\n  to = ");
            http_append(desc, &dl, sizeof(desc), f->to);
            http_append(desc, &dl, sizeof(desc), "\n  depth = ");
            http_append_u64(desc, &dl, sizeof(desc), f->depth);
            http_append(desc, &dl, sizeof(desc), "\n  frame_max = ");
            http_append_u64(desc, &dl, sizeof(desc), f->frame_max);
            http_append(desc, &dl, sizeof(desc), "\n");
            if (dl > 0 && capsule_store_write(pack, 20000U + i, desc, dl))
                wrote++;
        }
        http_append(out, &len, max, "{\"ok\":true,\"op\":\"realize\",\"pack\":");
        http_append_u64(out, &len, max, pack);
        http_append(out, &len, max, ",\"fifoCards\":");
        http_append_u64(out, &len, max, wrote);
        http_append(out, &len, max, "}\n");
        return len;
    }

    if (!http_capsule_query_u32(req, req_len, "card", &card)) {
        http_capsule_json_error(out, &len, max, "missing card");
        return len;
    }

    if (http_streq(op, "get") || http_streq(op, "gethex")) {
        char off_s[16];
        char max_s[16];
        u32 off = 0;
        u32 want = 2048;
        if (http_query_value(req, req_len, "/api/capsule", "offset", off_s, sizeof(off_s)))
            (void)http_parse_u32(off_s, &off);
        if (http_query_value(req, req_len, "/api/capsule", "max", max_s, sizeof(max_s)))
            (void)http_parse_u32(max_s, &want);
        if (want == 0 || want > 4096U) want = 2048;
        static u8 cbuf[4096];
        u32 total = 0;
        i32 n = capsule_store_read_at(pack, card, off, cbuf, want, &total);
        if (n < 0) {
            http_capsule_json_error(out, &len, max, "card read failed");
            return len;
        }
        http_append(out, &len, max, "{\"ok\":true,\"op\":\"get\",\"pack\":");
        http_append_u64(out, &len, max, pack);
        http_append(out, &len, max, ",\"card\":");
        http_append_u64(out, &len, max, card);
        http_append(out, &len, max, ",\"role\":");
        http_append_json_string(out, &len, max, capsule_card_role(card));
        http_append(out, &len, max, ",\"offset\":");
        http_append_u64(out, &len, max, off);
        http_append(out, &len, max, ",\"bytes\":");
        http_append_u64(out, &len, max, (u32)n);
        http_append(out, &len, max, ",\"totalBytes\":");
        http_append_u64(out, &len, max, total);
        http_append(out, &len, max, ",\"truncated\":");
        http_append(out, &len, max, (((u64)off + (u32)n) < total) ? "true" : "false");
        http_append(out, &len, max, ",\"encoding\":\"hex\",\"data\":");
        http_append_json_hex_bytes(out, &len, max, cbuf, (u32)n);
        http_append(out, &len, max, "}\n");
        return len;
    }

    if (http_streq(op, "puthex")) {
        char off_s[16];
        char hex[2048];
        u32 off = 0;
        if (http_query_value(req, req_len, "/api/capsule", "offset", off_s, sizeof(off_s)))
            (void)http_parse_u32(off_s, &off);
        if (!http_query_value(req, req_len, "/api/capsule", "hex", hex, sizeof(hex))) {
            http_capsule_json_error(out, &len, max, "missing hex");
            return len;
        }
        static u8 cbuf[CAPSULE_STORE_CARD_MAX_BYTES];
        u32 old_len = 0;
        if (off != 0) {
            i32 got = capsule_store_read(pack, card, cbuf, sizeof(cbuf));
            if (got > 0) old_len = (u32)got;
        }
        for (u32 z = old_len; z < off && z < sizeof(cbuf); z++) cbuf[z] = 0;
        u32 hex_len = pios_strlen(hex);
        bool ok = (hex_len != 0 && (hex_len & 1U) == 0);
        u32 count = hex_len / 2U;
        if (off > sizeof(cbuf) || count > sizeof(cbuf) - off) ok = false;
        for (u32 i = 0; ok && i < count; i++)
            ok = pixe_parse_hex_byte_pair(hex + i * 2U, &cbuf[off + i]);
        if (!ok) {
            http_capsule_json_error(out, &len, max, "bad hex/range");
            return len;
        }
        u32 new_len = off + count;
        if (new_len < old_len) new_len = old_len;
        if (!capsule_store_write(pack, card, cbuf, new_len)) {
            http_capsule_json_error(out, &len, max, "card write verify failed");
            return len;
        }
        http_append(out, &len, max, "{\"ok\":true,\"op\":\"puthex\",\"pack\":");
        http_append_u64(out, &len, max, pack);
        http_append(out, &len, max, ",\"card\":");
        http_append_u64(out, &len, max, card);
        http_append(out, &len, max, ",\"bytes\":");
        http_append_u64(out, &len, max, new_len);
        http_append(out, &len, max, ",\"verified\":true}\n");
        return len;
    }

    if (http_streq(op, "del")) {
        bool ok = capsule_store_delete(pack, card);
        http_append(out, &len, max, "{\"ok\":");
        http_append(out, &len, max, ok ? "true" : "false");
        http_append(out, &len, max, ",\"op\":\"del\",\"pack\":");
        http_append_u64(out, &len, max, pack);
        http_append(out, &len, max, ",\"card\":");
        http_append_u64(out, &len, max, card);
        if (!ok) http_append(out, &len, max, ",\"error\":\"delete failed\"");
        http_append(out, &len, max, "}\n");
        return len;
    }

    http_capsule_json_error(out, &len, max, "unknown op");
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
           http_query_value(req, req_len, "/api/admin/kernel-stream", key, out, out_max) ||
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

    /* Measures how long this call blocks core0's single reactor loop --
     * since CORE_DISK == CORE_NET == 0 (include/core.h), net_poll()/TCP/UART
     * cannot run for the whole duration of this synchronous multi-block SD
     * flush. Emitted unconditionally (not threshold-gated) under
     * DTRACE_CAT_OTA since a full commit is a rare, high-value event to
     * always capture when tracing is on. */
    u64 dt_sd_start;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(dt_sd_start));

    u32 base_lba = walfs_partition_lba() + (slot_offset / SD_BLOCK_SIZE);
    u32 pos = offset;
    u32 written = 0;
    u32 since_pet = 0;
    while (written < len) {
        u32 sector_off = pos % SD_BLOCK_SIZE;
        u32 n = SD_BLOCK_SIZE - sector_off;
        if (n > len - written)
            n = len - written;
        u32 lba = base_lba + (pos / SD_BLOCK_SIZE);
        if (sector_off == 0 && n == SD_BLOCK_SIZE) {
            if (!sd_write_block(lba, data + written)) {
                u64 dt_sd_end;
                __asm__ volatile("mrs %0, cntpct_el0" : "=r"(dt_sd_end));
                DTRACE(DTRACE_CAT_OTA, DT_OTA_SD_BLOCK_DUR, written,
                       dt_sd_end - dt_sd_start, slot_offset, 0);
                return false;
            }
        } else {
            if (!sd_read_block(lba, block))
                simd_zero(block, sizeof(block));
            simd_memcpy(block + sector_off, data + written, n);
            if (!sd_write_block(lba, block)) {
                u64 dt_sd_end;
                __asm__ volatile("mrs %0, cntpct_el0" : "=r"(dt_sd_end));
                DTRACE(DTRACE_CAT_OTA, DT_OTA_SD_BLOCK_DUR, written,
                       dt_sd_end - dt_sd_start, slot_offset, 0);
                return false;
            }
        }
        pos += n;
        written += n;
        /* Pet the 5s hardware watchdog during a large flush (e.g. the OTA commit
         * writes ~1.2MB = thousands of blocks); without this the bulk write
         * would trip the watchdog and reboot mid-write, corrupting the slot. */
        if (++since_pet >= 64U) {
            since_pet = 0;
            watchdog_hw_pet();
        }
    }
    {
        u64 dt_sd_end;
        __asm__ volatile("mrs %0, cntpct_el0" : "=r"(dt_sd_end));
        DTRACE(DTRACE_CAT_OTA, DT_OTA_SD_BLOCK_DUR, written,
               dt_sd_end - dt_sd_start, slot_offset, 0);
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
        if (e->event && http_streq(e->event, "http-error")) {
            u32 ev = (e->a >> 24) & 0xFFU;
            u32 route = (e->a >> 16) & 0xFFU;
            u32 arg = e->a & 0xFFFFU;
            http_append(out, &len, max, " detail event=");
            http_append(out, &len, max, http_event_name(ev));
            http_append(out, &len, max, " route=");
            http_append(out, &len, max, http_route_name(route));
            if (ev == HTTP_EVT_ABORT) {
                http_append(out, &len, max, " error=");
                http_append(out, &len, max, http_error_name(arg));
                http_append(out, &len, max, " conn=");
                http_append_u64(out, &len, max, e->b);
            } else if (ev == HTTP_EVT_BAD_STATE) {
                http_append(out, &len, max, " state=");
                http_append_u64(out, &len, max, arg);
                http_append(out, &len, max, " idle_ms=");
                http_append_u64(out, &len, max, e->b);
            } else {
                http_append(out, &len, max, " arg=");
                http_append_u64(out, &len, max, arg);
            }
        }
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

/* Pre-allocate the OTA RAM staging buffer at boot so chunk uploads are pure
 * memcpy (no per-chunk SD writes that stall the core0 NIC poll). One-time
 * highmem allocation; if highmem is unavailable, OTA falls back to direct SD
 * writes (slower, may stall). Call once after highmem_init. */
static void ota_staging_init(void)
{
    if (ota_stage_buf)
        return;
    struct highmem_status hm;
    highmem_status(&hm);
    if (hm.ready) {
        ota_stage_buf = (u8 *)highmem_alloc(PIOS_STAGE2_ZONE_BYTES, 64);
        if (ota_stage_buf)
            ota_stage_cap = PIOS_STAGE2_ZONE_BYTES;
        return;
    }
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* QEMU has no Pi highmem allocator, so the OTA stream would always be
     * rejected ("stream unavailable") and the core0-starvation stall could
     * never be reproduced in a VM. Provide a static staging buffer here so the
     * upload stream runs end to end; the stall manifests mid-stream, before any
     * SD commit, so this is enough to debug it. Gated to QEMU so the Pi5 .bss
     * is not bloated by the ~2MB zone. */
    static u8 qemu_ota_stage[PIOS_STAGE2_ZONE_BYTES] ALIGNED(64);
    ota_stage_buf = qemu_ota_stage;
    ota_stage_cap = (u32)sizeof(qemu_ota_stage);
#endif
}

/* Allocate the multi-connection HTTP :80 pool from highmem at boot (each slot
 * is ~17KB; HTTP_MAX_CONCURRENT of them). Falls back to a tiny static pool if
 * highmem is unavailable. Call once after highmem_init. */
static void http_conns_init(void)
{
    if (http_conns)
        return;
    struct highmem_status hm;
    highmem_status(&hm);
    if (hm.ready) {
        http_conns = (struct http_conn *)highmem_alloc(
            (u64)HTTP_MAX_CONCURRENT * sizeof(struct http_conn), 64);
        if (http_conns)
            http_conn_count = HTTP_MAX_CONCURRENT;
    }
    if (!http_conns) {
        http_conns = http_conns_fallback;
        http_conn_count = (u32)(sizeof(http_conns_fallback) / sizeof(http_conns_fallback[0]));
    }
    for (u32 i = 0; i < http_conn_count; i++)
        http_conns[i].client_conn = -1;
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
        /* Acquire (once) a highmem staging buffer so chunk uploads avoid
         * per-chunk SD writes that stall the NIC. */
        if (!error && !ota_stage_buf) {
            struct highmem_status hm;
            highmem_status(&hm);
            if (hm.ready) {
                ota_stage_buf = (u8 *)highmem_alloc(capacity, 64);
                if (ota_stage_buf)
                    ota_stage_cap = capacity;
            }
        }
        ota_stage_ready = (ota_stage_buf != NULL && total <= ota_stage_cap);
        if (!error && !http_write_kernel_slot_header(ota_update.target_slot_offset, total, false)) {
            error = "failed to invalidate slot header";
        } else if (!error) {
            ota_update.active = true;
            ota_update.total = total;
            ota_update.received = 0;
            ota_update.chunks = 0;
            ota_update.last_error = NULL;
            ok = true;
            http_log_event("ota-begin", total, ota_stage_ready ? 1U : 0U);
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
                if (ota_stage_ready) {
                    /* Fast path: stage in RAM, no SD write -> NIC keeps polling. */
                    if (ota_update.received + new_len <= ota_stage_cap) {
                        simd_memcpy(ota_stage_buf + ota_update.received,
                                    req + body_off + skip, new_len);
                        written = new_len;
                        ok = true;
                    } else {
                        error = "staged chunk exceeds buffer";
                        ok = false;
                    }
                } else {
                    ok = http_write_kernel_payload_range(ota_update.target_slot_offset,
                                                        ota_update.received,
                                                        req + body_off + skip,
                                                        new_len, &written);
                }
                if (ok && written == new_len) {
                    ota_update.received += written;
                    written = body_len;
                } else if (!error) {
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
    } else if (http_streq(action, "commit") || http_streq(action, "writeandreboot")) {
        /* writeandreboot = the user's flow: after blocks are uploaded (RAM-staged)
         * and verified (received==total), flush the staged image to the SD slot in
         * ONE pass and reboot into it. core0 is unresponsive only during this final
         * flush, which is fine because the board is about to reboot anyway (and the
         * SD-write loop pets the watchdog). The A/B header is written valid LAST and
         * the slot is marked pending (health-gated), so a failed flush is safe. */
        bool force_reboot = http_streq(action, "writeandreboot");
        if (!ota_update.active) {
            error = "no OTA update active";
        } else if (total != 0 && total != ota_update.total) {
            error = "total does not match active OTA update";
        } else if (ota_update.received != ota_update.total) {
            error = "OTA image incomplete";
        } else if (ota_stage_ready &&
                   !http_write_kernel_payload_range(ota_update.target_slot_offset, 0,
                                                    ota_stage_buf, ota_update.total, &written)) {
            error = "failed to flush staged image to slot";
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
            if (force_reboot ||
                (http_update_query_value(req, req_len, "reboot", reboot, sizeof(reboot)) &&
                 http_streq(reboot, "1")))
                http_reboot_pending = true;
        }
    } else if (http_streq(action, "cancel")) {
        ota_update.active = false;
        ota_update.last_error = "cancelled";
        ok = true;
        http_log_event("ota-cancel", ota_update.received, ota_update.total);
    } else if (http_streq(action, "reset")) {
        u32 old_received = ota_update.received;
        u32 old_total = ota_update.total;
        ota_update_reset_state();
        ok = true;
        http_log_event("ota-reset", old_received, old_total);
    } else {
        error = "unknown action; use status, begin, chunk, commit, writeandreboot, cancel, reset, or self";
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
    http_append(out, &len, max, ",\"staged\":");
    http_append(out, &len, max, (ota_stage_buf != NULL) ? "true" : "false");
    http_append(out, &len, max, ",\"stageReady\":");
    http_append(out, &len, max, ota_stage_ready ? "true" : "false");
    http_append(out, &len, max, ",\"stageCap\":");
    http_append_u64(out, &len, max, ota_stage_cap);
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
        "<style>body{margin:0;background:var(--cp-bg);color:var(--cp-text);font-family:'Segoe UI',Aptos,Calibri,-apple-system,BlinkMacSystemFont,sans-serif}.wrap{padding:24px}.top{display:flex;justify-content:space-between;gap:16px;align-items:flex-end}.tabs{display:flex;gap:6px;flex-wrap:wrap;margin:16px 0}.tabs button,.bt{border:1px solid var(--cp-border);background:var(--cp-surface);color:var(--cp-text);border-radius:.625rem;padding:8px 12px}.tabs button.act,.bt.primary{background:var(--cp-accent);color:var(--cp-accent-fg);border-color:var(--cp-accent)}.tabs button:hover,.bt:hover{border-color:var(--cp-border-strong)}.tools{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:0 0 12px}.tools input{width:88px}.tools label{color:var(--cp-text-muted)}input{background:var(--cp-surface);color:var(--cp-text);border:1px solid var(--cp-border);border-radius:.625rem;padding:7px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}.cd,.tile{background:var(--cp-surface);border:1px solid var(--cp-border);border-radius:16px;box-shadow:0 0 2px rgba(0,0,0,.12),0 1px 2px rgba(0,0,0,.14);padding:16px;margin:12px 0}.tile{margin:0}.muted{color:var(--cp-text-muted)}.pill{display:inline-block;border:1px solid var(--cp-border);border-radius:.625rem;padding:2px 7px;background:var(--cp-accent-soft)}a{color:var(--cp-link)}pre,code,.nt{font-family:Consolas,'Courier New',Courier,monospace}pre{white-space:pre-wrap;overflow:auto}.nt{width:100%;border-collapse:collapse;font-size:13px}.nt th,.nt td{border-bottom:1px solid var(--cp-border);padding:7px 8px;text-align:left;vertical-align:top}.nt th{color:var(--cp-text-muted);background:var(--cp-surface-soft)}.ok{color:var(--cp-success)}.warn{color:var(--cp-warning)}.bad{color:var(--cp-danger)}.term{background:var(--cp-bg-elevated);color:var(--cp-success);border:1px solid var(--cp-success);border-radius:.625rem;overflow:hidden;font-family:Consolas,'Courier New',Courier,monospace}.bar{background:var(--cp-surface-soft);padding:8px 12px;border-bottom:1px solid var(--cp-border);letter-spacing:.08em}.screen{height:420px;overflow:auto;padding:14px;white-space:pre-wrap;font-size:14px;line-height:1.35}.prompt{display:flex;gap:8px;align-items:center;border-top:1px solid var(--cp-border);background:var(--cp-surface-soft);padding:10px 12px}.prompt input{flex:1;border:0;outline:0;font-family:Consolas,'Courier New',Courier,monospace}.pico-frame{width:100%;height:70vh;border:1px solid var(--cp-border);border-radius:16px;background:var(--cp-surface)}.cursor{animation:blink 1s steps(1) infinite}@keyframes blink{50%{opacity:0}}</style></head><body><div class='wrap'><div class='top'><div><h1>PIOS Admin Console</h1><p id='sub' class='muted'>Structured tabs; manual refresh.</p></div><div class='muted'>SECOND STAGE</div></div><div class='tabs'><button class='act' data-t='overview'>Overview</button><button data-t='processes'>Processes</button><button data-t='netstat'>Netstat</button><button data-t='graphs'>Graphs</button><button data-t='system'>System</button><button data-t='users'>Users</button><button data-t='logs'>Logs</button><button data-t='walfs'>WALFS</button><button data-t='picoscript'>PicoScript</button><button data-t='firewall'>Firewall</button><button data-t='terminal'>Terminal</button><button data-t='admin'>Admin</button></div><div id='app'></div></div>");
    http_append(out, &len, max,
        "<script>let tab='overview',samples=[],hist=['PIOS remote terminal ready. Type Help for assistance!'],auto=false,ms=3000,timer=0;const app=document.getElementById('app'),sub=document.getElementById('sub');function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}function bust(u){return u+(u.includes('?')?'&':'?')+'_='+Date.now()}async function txt(u){let r=await fetch(bust(u));return await r.text()}async function js(u){let r=await fetch(bust(u));return await r.json()}function card(t,h){return `<div class=cd><h2>${t}</h2>${h}</div>`}function tools(){return `<div class=tools><button class='bt primary' id=rf>Refresh</button><label><input id=ar type=checkbox ${auto?'checked':''}> auto-refresh</label><input id=ms type=number min=250 step=250 value='${ms}'><span class=muted>ms</span></div>`}function bind(){let r=document.getElementById('rf'),a=document.getElementById('ar'),m=document.getElementById('ms');if(r)r.onclick=draw;if(a)a.onchange=()=>{auto=a.checked;sync()};if(m)m.onchange=()=>{ms=Math.max(250,parseInt(m.value||'3000'));m.value=ms;sync()};sync()}function sync(){if(timer){clearInterval(timer);timer=0}if(auto&&tab!=='terminal')timer=setInterval(draw,ms)}function table(h,rows){let b=rows.length?rows.map(r=>'<tr>'+h.map((_,i)=>'<td>'+esc(r[i]||'')+'</td>').join('')+'</tr>').join(''):`<tr><td colspan='${h.length}' class=muted>No rows</td></tr>`;return '<table class=nt><thead><tr>'+h.map(x=>'<th>'+esc(x)+'</th>').join('')+'</tr></thead><tbody>'+b+'</tbody></table>'}function kv(o){return table(['Key','Value'],Object.keys(o).map(k=>[k,o[k]]))}function splitRows(t){return t.trim().split('\\n').filter(x=>x&&x[0]>='0'&&x[0]<='9').map(x=>x.trim().split(/\\s+/))}function spark(a){let m=Math.max(1,...a);return a.map(v=>String(v).padStart(4,' ')+ ' '+ '#'.repeat(Math.min(40,Math.round(v*40/m)))).join('\\n')}async function overview(){let d=await js('/api/status'),dg=d.diag||{};app.innerHTML=card('Overview',tools()+`<div class=grid><div class=tile><b>Build</b><p>${esc(d.build)}</p><span class=pill>${esc(d.version)}</span></div><div class=tile><b>Network</b><p>${esc(d.ip)}</p><p class=muted>${esc(d.mode)}</p></div><div class=tile><b>Uptime</b><p>${esc(d.uptime)} seconds</p></div><div class=tile><b>HTTP diagnostics</b>${kv(dg)}</div></div>`);bind()}async function system(){let r=await txt('/api/terminal?cmd=status'),rows=[];r.split(/\\s+/).forEach(p=>{let i=p.indexOf('=');if(i>0)rows.push([p.slice(0,i),p.slice(i+1)])});app.innerHTML=card('System',tools()+`<div class=grid><div class=tile><b>${esc((r.split('\\n')[0]||'PIOS'))}</b></div><div class=tile>${table(['Metric','Value'],rows)}</div></div><pre>${esc(r)}</pre>`);bind()}async function netstat(){let n=await js('/api/netstat');app.innerHTML=card('Netstat',tools()+table(n.cols,n.rows)+`<p class=muted>syn=${n.diag.syn} accepted=${n.diag.accepted} fw_rx_drop=${n.fw.rxDrop}</p>`);bind()}async function graphs(){let s=await js('/api/status'),n=await js('/api/netstat');samples.push([s.uptime,n.count,n.diag.syn,n.fw.rxDrop]);while(samples.length>48)samples.shift();app.innerHTML=card('Graphs',tools()+table(['uptime','conns','syn','fw_rx_drop'],samples)+`<div class=grid><div class=tile><h3>Connections</h3><pre>${esc(spark(samples.map(x=>x[1])))}</pre></div><div class=tile><h3>Firewall RX drops</h3><pre>${esc(spark(samples.map(x=>x[3])))}</pre></div></div>`);bind()}async function processes(){let r=await txt('/api/terminal?cmd=processes'),p=r.split('\\n\\nGRAPH\\n'),rows=splitRows(p[0]);app.innerHTML=card('Processes',tools()+table(['PID','PPID','Core','State','Pri','CPU','MemK','ArenaCap','ArenaUsed','ArenaHigh','Bump','SpanK','Span#','Image'],rows)+`<h3>Process graph</h3><pre>${esc(p[1]||'')}</pre>`);bind()}async function users(){let r=await txt('/api/terminal?cmd=users');app.innerHTML=card('Users',tools()+table(['ID','Flags','Name'],splitRows(r)));bind()}async function logs(){let a=await txt('/api/admin/log-stream?tail=24'),b=await txt('/api/logs');app.innerHTML=card('Logs',tools()+`<div class=grid><div class=tile><h3>Operator tail</h3><pre>${esc(a)}</pre></div><div class=tile><h3>Process logs</h3><pre>${esc(b)}</pre></div></div>`);bind()}async function walfs(){let d=await js('/api/walfs?path=/'),body='';if(d.entries)body=table(['ID','Name','Size','Flags'],d.entries.map(e=>[e.id,e.name,e.size,e.flags]));else body=kv(d);app.innerHTML=card('WALFS',tools()+body);bind()}function picoscript(){sync();app.innerHTML=card('PicoScript Editor',`<p class=muted>Full PicoScript playground/editor served from this PIOS image. <a href='/picoscript' target='_blank'>Open in new tab</a></p><iframe class=pico-frame src='/picoscript'></iframe>`)}async function firewall(){let r=await txt('/api/terminal?cmd=firewall%20list'),rows=[];r.split('\\n').forEach(l=>{let m=l.match(/^(\\d+):\\s+(\\S+)\\s+(\\S+)\\s+(.*)$/);if(m)rows.push([m[1],m[2],m[3],m[4]])});app.innerHTML=card('Firewall',tools()+`<p class=muted>${esc(r.split('\\n')[0]||'')}</p>`+table(['#','Action','Dir','Match'],rows)+`<h3>Mutation examples</h3><pre>firewall allow in tcp port 2323 src 192.168.218.9\\nfirewall deny in tcp port 80 src 192.168.218.0/24\\nfirewall reset</pre>`);bind()}function term(){sync();app.innerHTML=card('Terminal',`<div class=term><div class=bar>PIOS // SECOND STAGE LOADER // REMOTE CONSOLE</div><div class=screen id=screen></div><div class=prompt><span>ready&gt;</span><input id=cmd autocomplete=off spellcheck=false autofocus><span class=cursor>_</span></div></div>`);const sc=document.getElementById('screen'),cmd=document.getElementById('cmd');function paint(){sc.textContent=hist.join('\\n');sc.scrollTop=sc.scrollHeight}async function run(){let c=cmd.value.trim();cmd.value='';if(!c)return;if(c==='clear'){hist=[];paint();return}hist.push('ready> '+c);try{hist.push(await txt('/api/terminal?cmd='+encodeURIComponent(c)))}catch(e){hist.push('ERR '+e)}while(hist.length>160)hist.shift();paint()}cmd.onkeydown=e=>{if(e.key==='Enter')run()};paint();cmd.focus()}function admin(){app.innerHTML=card('Admin',tools()+`<div class=grid><div class=tile><b>Logs</b><p><a href='/api/admin/log-stream?tail=24'>Tail log stream</a></p></div><div class=tile><b>OTA</b><p><a href='/api/admin/kernel-update?confirm=1'>Kernel update status</a></p></div><div class=tile><b>Reboot</b><p><a href='/api/admin/reboot?confirm=1'>Queue hot reboot</a></p></div></div>`);bind()}async function draw(){sub.textContent='Loading '+tab+'...';try{if(tab==='overview')await overview();else if(tab==='system')await system();else if(tab==='netstat')await netstat();else if(tab==='graphs')await graphs();else if(tab==='processes')await processes();else if(tab==='users')await users();else if(tab==='logs')await logs();else if(tab==='walfs')await walfs();else if(tab==='picoscript')picoscript();else if(tab==='firewall')await firewall();else if(tab==='terminal')term();else if(tab==='admin')admin();else app.innerHTML=card(tab,'unknown tab')}catch(e){app.innerHTML=card('Error',`<pre>${esc(e)}</pre>`)}sub.textContent='Tab '+tab+(auto&&tab!=='terminal'?` // auto ${ms}ms`:'')}document.querySelectorAll('[data-t]').forEach(b=>b.onclick=()=>{tab=b.dataset.t;document.querySelectorAll('[data-t]').forEach(x=>x.classList.toggle('act',x===b));draw()});draw()</script></body></html>");
    return len;
}

/* ---- PicoSTS hosted auth/z endpoints (vendored contract, WALFS-backed) ----
 *
 * health + jwks are public (HTTP or HTTPS); login/validate/whoami/users are
 * TLS-only and fail closed on plaintext. Credentials/tenants arrive as
 * query/form parameters for this first slice (documented limitation) and are
 * NEVER logged: no request-body echo, no secret in any http_log_event, and the
 * password buffer is scrubbed before returning. HS256 is symmetric so /jwks
 * intentionally exposes no key material. */
static void http_sts_json_header(char *out, u32 *len, u32 max, const char *status)
{
    http_append(out, len, max, "HTTP/1.0 ");
    http_append(out, len, max, status);
    http_append(out, len, max,
        "\r\nContent-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
}

/* Extract a bearer token from the Authorization header (NUL-terminated).
 * Returns the token length or 0 (absent/oversized => fail closed). */
/* True only for an HTTP POST request line (method is case-sensitive). */
static bool http_method_is_post(const u8 *req, u32 len)
{
    return len >= 5U && req[0] == 'P' && req[1] == 'O' && req[2] == 'S' &&
           req[3] == 'T' && (req[4] == ' ' || req[4] == '\t');
}

/* Extract a single request header value by case-insensitive name. name_lc must
 * be lowercase and include the trailing ':'. Trims surrounding whitespace and
 * fails closed (returns 0) on overflow. The caller must never log the result
 * for secret headers such as X-PIOS-Password. */
static u32 http_header_value_ci(const u8 *req, u32 len, const char *name_lc,
                                char *out, u32 cap)
{
    if (!out || cap == 0U) return 0;
    out[0] = 0;
    u32 nlen = 0;
    while (name_lc[nlen]) nlen++;
    if (nlen == 0U) return 0;
    for (u32 i = 0; i + nlen <= len; i++) {
        if (i != 0 && req[i - 1] != '\n') continue;
        if (!http_match_ci(&req[i], name_lc)) continue;
        i += nlen;
        while (i < len && (req[i] == ' ' || req[i] == '\t')) i++;
        u32 o = 0;
        while (i < len && req[i] != '\r' && req[i] != '\n') {
            if (o + 1U >= cap) return 0;   /* fail closed on overflow */
            out[o++] = (char)req[i++];
        }
        while (o > 0U && (out[o - 1] == ' ' || out[o - 1] == '\t')) o--;
        out[o] = 0;
        return o;
    }
    return 0;
}

static u32 http_bearer_token(const u8 *req, u32 len, char *out, u32 cap)
{
    static const char auth_hdr[] = "authorization:";
    static const char bearer[] = "bearer ";
    if (!out || cap == 0U) return 0;
    out[0] = 0;
    for (u32 i = 0; i + sizeof(auth_hdr) - 1 < len; i++) {
        if (i != 0 && req[i - 1] != '\n') continue;
        if (!http_match_ci(&req[i], auth_hdr)) continue;
        i += (u32)sizeof(auth_hdr) - 1;
        while (i < len && (req[i] == ' ' || req[i] == '\t')) i++;
        if (i + sizeof(bearer) - 1 >= len || !http_match_ci(&req[i], bearer)) return 0;
        i += (u32)sizeof(bearer) - 1;
        u32 o = 0;
        while (i < len && req[i] != '\r' && req[i] != '\n' && req[i] != ' ' && req[i] != '\t') {
            if (o + 1U >= cap) return 0;   /* fail closed on overflow */
            out[o++] = (char)req[i++];
        }
        out[o] = 0;
        return o;
    }
    return 0;
}

/* Space/plus-separated scope string -> policy bitmask (unknown scope => fail). */
static bool http_sts_parse_scopes(const char *s, u16 *mask_out)
{
    u16 mask = 0;
    u32 i = 0;
    while (s[i]) {
        while (s[i] == ' ' || s[i] == '+') i++;
        u32 start = i;
        while (s[i] && s[i] != ' ' && s[i] != '+') i++;
        if (i > start) {
            i32 si = sts_scope_index(&s[start], i - start);
            if (si < 0) return false;
            mask |= (u16)(1U << (u32)si);
        }
    }
    *mask_out = mask;
    return true;
}

/* One TLS record is the hard transport limit for a :443 response (mirrors
 * tls.c TLS_MAX_RECORD). The users listing is paginated to stay under it. */
#define STS_TLS_RECORD_MAX  1024U
/* Page cap for /api/sts/users so a full page + envelope + headers always fits
 * one TLS record; a per-record size guard additionally enforces the bound. */
#define STS_USERS_PAGE_MAX  4U

static u32 http_build_sts_response(char *out, u32 max, const u8 *req, u32 req_len, bool via_tls)
{
    u32 len = 0;

    /* ---- public: health ---- */
    if (http_request_path_is(req, req_len, "/api/sts/health")) {
        u64 utc = timer_utc_ms();
        http_sts_json_header(out, &len, max, "200 OK");
        http_append(out, &len, max, "{\"service\":\"sts\",\"has_secret\":");
        http_append(out, &len, max, sts_has_secret() ? "true" : "false");
        http_append(out, &len, max, ",\"utc_set\":");
        http_append(out, &len, max, utc ? "true" : "false");
        http_append(out, &len, max, ",\"rng_available\":");
        http_append(out, &len, max, crypto_random_available() ? "true" : "false");
        http_append(out, &len, max, ",\"rng\":");
        http_append_json_string(out, &len, max, crypto_random_status());
        http_append(out, &len, max, ",\"tls\":");
        http_append(out, &len, max, via_tls ? "true" : "false");
        http_append(out, &len, max, "}\n");
        return len;
    }

    /* ---- public: JWKS (HS256 symmetric => no public keys, ever) ---- */
    if (http_request_path_is(req, req_len, "/api/sts/jwks")) {
        http_sts_json_header(out, &len, max, "200 OK");
        http_append(out, &len, max, "{\"keys\":[],\"alg\":\"HS256\",\"note\":\"symmetric; no public key material\"}\n");
        return len;
    }

    /* ---- everything below is sensitive: TLS-only, fail closed ---- */
    if (!via_tls) {
        http_sts_json_header(out, &len, max, "403 Forbidden");
        http_append(out, &len, max, "{\"ok\":false,\"error\":\"tls_required\"}\n");
        return len;
    }

    /* ---- login: password -> scoped token (mirrors POST /sts/login) ---- */
    if (http_request_path_is(req, req_len, "/api/sts/login")) {
        if (!http_method_is_post(req, req_len)) {
            http_sts_json_header(out, &len, max, "405 Method Not Allowed");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"method_not_allowed\"}\n");
            return len;
        }
        char user[64], pass[128], aud[64], tenant[64], scope_s[160], ttl_s[16];
        char token[STS_TOKEN_MAX];
        char scope_out[256];
        u32 tlen = 0;
        u16 req_scope = 0;
        u32 ttl = 0;
        /* Non-secret identifiers stay bounded query params; the password is
         * carried only in the X-PIOS-Password header (TLS-only path) and is
         * never taken from the URL and never logged. */
        u32 plen = http_header_value_ci(req, req_len, "x-pios-password:", pass, sizeof(pass));
        if (plen == 0U ||
            !http_query_value(req, req_len, "/api/sts/login", "user", user, sizeof(user)) ||
            !http_query_value(req, req_len, "/api/sts/login", "aud", aud, sizeof(aud))) {
            for (u32 k = 0; k < sizeof(pass); k++) pass[k] = 0;
            http_sts_json_header(out, &len, max, "400 Bad Request");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"missing_params\"}\n");
            return len;
        }
        if (!http_query_value(req, req_len, "/api/sts/login", "tenant", tenant, sizeof(tenant)) || tenant[0] == 0) {
            tenant[0] = 'd'; tenant[1] = 'e'; tenant[2] = 'f'; tenant[3] = 'a';
            tenant[4] = 'u'; tenant[5] = 'l'; tenant[6] = 't'; tenant[7] = 0;
        }
        if (http_query_value(req, req_len, "/api/sts/login", "scope", scope_s, sizeof(scope_s)) && scope_s[0]) {
            if (!http_sts_parse_scopes(scope_s, &req_scope)) {
                for (u32 k = 0; k < sizeof(pass); k++) pass[k] = 0;
                http_sts_json_header(out, &len, max, "403 Forbidden");
                http_append(out, &len, max, "{\"ok\":false,\"error\":\"scope_denied\"}\n");
                return len;
            }
        }
        if (http_query_value(req, req_len, "/api/sts/login", "ttl", ttl_s, sizeof(ttl_s)))
            (void)http_parse_u32(ttl_s, &ttl);

        i32 rc = sts_login(user, pass, aud, tenant, req_scope, ttl,
                           token, sizeof(token), &tlen, scope_out, sizeof(scope_out));
        for (u32 k = 0; k < sizeof(pass); k++) pass[k] = 0;   /* scrub secret */

        if (rc == STS_OK) {
            http_sts_json_header(out, &len, max, "200 OK");
            http_append(out, &len, max, "{\"ok\":true,\"token\":\"");
            http_append(out, &len, max, token);
            http_append(out, &len, max, "\",\"scope\":");
            http_append_json_string(out, &len, max, scope_out);
            http_append(out, &len, max, "}\n");
        } else {
            http_sts_json_header(out, &len, max, (rc == STS_ERR_AUTH) ? "401 Unauthorized" : "403 Forbidden");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"");
            http_append(out, &len, max, sts_err_name(rc));
            http_append(out, &len, max, "\"}\n");
        }
        return len;
    }

    /* ---- validate: token + audience -> claims (mirrors POST /sts/validate) ---- */
    if (http_request_path_is(req, req_len, "/api/sts/validate")) {
        if (!http_method_is_post(req, req_len)) {
            http_sts_json_header(out, &len, max, "405 Method Not Allowed");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"method_not_allowed\"}\n");
            return len;
        }
        char token[STS_TOKEN_MAX], aud[64], tenant[64];
        u16 scope_mask = 0;
        /* Token is only accepted via Authorization: Bearer, never in the URL. */
        u32 tlen = http_bearer_token(req, req_len, token, sizeof(token));
        if (tlen == 0 ||
            !http_query_value(req, req_len, "/api/sts/validate", "aud", aud, sizeof(aud))) {
            http_sts_json_header(out, &len, max, "400 Bad Request");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"missing_params\"}\n");
            return len;
        }
        i32 rc = sts_validate(token, tlen, aud, tenant, sizeof(tenant), &scope_mask);
        if (rc == STS_OK) {
            char scope_str[256];
            (void)sts_scope_mask_to_string(scope_mask, scope_str, sizeof(scope_str));
            http_sts_json_header(out, &len, max, "200 OK");
            http_append(out, &len, max, "{\"active\":true,\"tenant\":");
            http_append_json_string(out, &len, max, tenant);
            http_append(out, &len, max, ",\"scope\":");
            http_append_json_string(out, &len, max, scope_str);
            http_append(out, &len, max, "}\n");
        } else {
            http_sts_json_header(out, &len, max, "200 OK");
            http_append(out, &len, max, "{\"active\":false,\"error\":\"");
            http_append(out, &len, max, sts_err_name(rc));
            http_append(out, &len, max, "\"}\n");
        }
        return len;
    }

    /* ---- whoami: token-gated authorization boundary (requires sts.validate) --- */
    if (http_request_path_is(req, req_len, "/api/sts/whoami")) {
        char token[STS_TOKEN_MAX], tenant[64];
        u16 scope_mask = 0;
        u32 tlen = http_bearer_token(req, req_len, token, sizeof(token));
        if (tlen == 0) {
            http_sts_json_header(out, &len, max, "401 Unauthorized");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"missing_bearer\"}\n");
            return len;
        }
        i32 rc = sts_validate(token, tlen, "wave-sts", tenant, sizeof(tenant), &scope_mask);
        u16 need = (u16)(1U << 1);   /* sts.validate */
        if (rc != STS_OK || !(scope_mask & need)) {
            http_sts_json_header(out, &len, max, "403 Forbidden");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"");
            http_append(out, &len, max, (rc != STS_OK) ? sts_err_name(rc) : "scope_denied");
            http_append(out, &len, max, "\"}\n");
            return len;
        }
        char scope_str[256];
        (void)sts_scope_mask_to_string(scope_mask, scope_str, sizeof(scope_str));
        http_sts_json_header(out, &len, max, "200 OK");
        http_append(out, &len, max, "{\"ok\":true,\"tenant\":");
        http_append_json_string(out, &len, max, tenant);
        http_append(out, &len, max, ",\"scope\":");
        http_append_json_string(out, &len, max, scope_str);
        http_append(out, &len, max, "}\n");
        return len;
    }

    /* ---- admin: list users ---- */
    if (http_request_path_is(req, req_len, "/api/sts/users")) {
        /* HTTP_AUTH_ENABLED is 0, so http_admin_authorized() cannot gate this
         * route. Require a real STS bearer token for audience "wave-sts" that
         * carries the sts.admin scope. Missing bearer => 401; invalid/expired
         * token => 401; valid token without sts.admin => 403. Fail closed. */
        char token[STS_TOKEN_MAX], tenant[64];
        u16 scope_mask = 0;
        u32 tlen = http_bearer_token(req, req_len, token, sizeof(token));
        if (tlen == 0) {
            http_sts_json_header(out, &len, max, "401 Unauthorized");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"missing_bearer\"}\n");
            return len;
        }
        i32 arc = sts_validate(token, tlen, "wave-sts", tenant, sizeof(tenant), &scope_mask);
        if (arc != STS_OK) {
            http_sts_json_header(out, &len, max, "401 Unauthorized");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"");
            http_append(out, &len, max, sts_err_name(arc));
            http_append(out, &len, max, "\"}\n");
            return len;
        }
        i32 admin_idx = sts_scope_index("sts.admin", 9);
        if (admin_idx < 0 || !(scope_mask & (u16)(1U << (u32)admin_idx))) {
            http_sts_json_header(out, &len, max, "403 Forbidden");
            http_append(out, &len, max, "{\"ok\":false,\"error\":\"scope_denied\"}\n");
            return len;
        }

        /* Bounded pagination. limit is clamped to STS_USERS_PAGE_MAX; a
         * per-record size guard then guarantees the complete response (headers
         * + JSON) never exceeds one STS_TLS_RECORD_MAX TLS record. */
        u32 offset = 0, limit = STS_USERS_PAGE_MAX;
        char num[16];
        if (http_query_value(req, req_len, "/api/sts/users", "offset", num, sizeof(num)))
            (void)http_parse_u32(num, &offset);
        if (http_query_value(req, req_len, "/api/sts/users", "limit", num, sizeof(num))) {
            u32 l = 0;
            if (http_parse_u32(num, &l) && l != 0U && l < limit) limit = l;
        }

        static struct sts_user_public snap[STS_MAX_USERS];
        u32 total = sts_list(snap, STS_MAX_USERS);
        u32 start = (offset < total) ? offset : total;

        http_sts_json_header(out, &len, max, "200 OK");
        http_append(out, &len, max, "{\"ok\":true,\"total\":");
        http_append_u64(out, &len, max, total);
        http_append(out, &len, max, ",\"offset\":");
        http_append_u64(out, &len, max, start);
        http_append(out, &len, max, ",\"users\":[");

        u32 emitted = 0;
        u32 idx = start;
        for (; idx < total && emitted < limit; idx++) {
            char rec[320];
            u32 rl = 0;
            char scope_str[160];
            (void)sts_scope_mask_to_string(snap[idx].scope_mask, scope_str, sizeof(scope_str));
            if (emitted) http_append(rec, &rl, sizeof(rec), ",");
            http_append(rec, &rl, sizeof(rec), "{\"username\":");
            http_append_json_string(rec, &rl, sizeof(rec), snap[idx].username);
            http_append(rec, &rl, sizeof(rec), ",\"admin\":");
            http_append(rec, &rl, sizeof(rec), (snap[idx].flags & STS_FLAG_ADMIN) ? "true" : "false");
            http_append(rec, &rl, sizeof(rec), ",\"disabled\":");
            http_append(rec, &rl, sizeof(rec), (snap[idx].flags & STS_FLAG_DISABLED) ? "true" : "false");
            http_append(rec, &rl, sizeof(rec), ",\"scope\":");
            http_append_json_string(rec, &rl, sizeof(rec), scope_str);
            http_append(rec, &rl, sizeof(rec), "}");
            /* Reserve room for the closing  ],"limit":N,"next":NNN}\n. */
            if (len + rl + 32U > STS_TLS_RECORD_MAX) break;
            http_append_bytes(out, &len, max, (const u8 *)rec, rl);
            emitted++;
        }
        http_append(out, &len, max, "],\"limit\":");
        http_append_u64(out, &len, max, limit);
        http_append(out, &len, max, ",\"next\":");
        if (idx < total) http_append_u64(out, &len, max, idx);
        else http_append(out, &len, max, "null");
        http_append(out, &len, max, "}\n");
        return len;
    }

    http_sts_json_header(out, &len, max, "404 Not Found");
    http_append(out, &len, max, "{\"ok\":false,\"error\":\"unknown_sts_route\"}\n");
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
    if (route == HTTP_ROUTE_PICOSCRIPT) {
        len = http_build_picoscript_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_STATIC) {
        len = http_build_static_file_response(out, max, req, req_len);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 200);
        return len;
    }
    if (route == HTTP_ROUTE_FAVICON) {
        len = http_build_no_content_response(out, max);
        http_diag.build_len = len;
        http_trace(HTTP_EVT_BUILD_EXIT, route, len, 204);
        return len;
    }
    /* PicoSTS endpoints. This dispatcher only ever runs on the plaintext :80
     * server, so via_tls is always false here: health/jwks answer, and the
     * sensitive login/validate/whoami/users routes fail closed with 403
     * tls_required. The HTTPS :443 path routes these with via_tls=true. */
    if (http_request_path_is(req, req_len, "/api/sts/health") ||
        http_request_path_is(req, req_len, "/api/sts/jwks") ||
        http_request_path_is(req, req_len, "/api/sts/login") ||
        http_request_path_is(req, req_len, "/api/sts/validate") ||
        http_request_path_is(req, req_len, "/api/sts/whoami") ||
        http_request_path_is(req, req_len, "/api/sts/users")) {
        len = http_build_sts_response(out, max, req, req_len, false);
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
    if (route == HTTP_ROUTE_PCAP) {
        len = http_build_pcap_response(out, max, req, req_len);
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
    if (http_request_path_is(req, req_len, "/api/capsule")) {
        len = http_build_capsule_response(out, max, req, req_len);
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

static u32 http_build_no_content_response(char *out, u32 max)
{
    u32 len = 0;
    http_append(out, &len, max,
        "HTTP/1.0 204 No Content\r\n"
        "Cache-Control: max-age=86400\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n\r\n");
    return len;
}

static u32 http_build_picoscript_config_response(char *out, u32 max)
{
    u32 len = 0;
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max,
        "{\"ok\":true,"
        "\"ide_prefix\":\"/picoscript/\","
        "\"capsule_prefix\":\"/api/capsule\","
        "\"walfs_prefix\":\"/api/walfs\","
        "\"terminal_prefix\":\"/api/terminal\","
        "\"picosts_enabled\":false,"
        "\"build\":\"");
    http_append(out, &len, max, PIOS_BUILD_STAMP);
    http_append(out, &len, max, "\",\"version\":\"");
    http_append(out, &len, max, PIOS_VERSION);
    http_append(out, &len, max, "\",\"hook_table_version\":\"0x");
    http_append_hex32(out, &len, max, (u32)PV_HOOK_TABLE_VERSION);
    http_append(out, &len, max, "\"}\n");
    return len;
}

static u32 http_build_picoscript_asset_response(char *out, u32 max,
                                                const u8 *body, u32 body_len,
                                                const char *content_type)
{
    u32 len = 0;
    http_static_body = body;
    http_static_len = body_len;
    http_static_off = 0;
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: ");
    http_append(out, &len, max, content_type);
    http_append(out, &len, max,
        "\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: ");
    http_append_u64(out, &len, max, http_static_len);
    http_append(out, &len, max,
        "\r\n"
        "Connection: close\r\n\r\n");
    return len;
}

static u32 http_build_picoscript_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    if (http_request_path_is(req, req_len, "/picoscript/config"))
        return http_build_picoscript_config_response(out, max);
    if (http_request_path_is(req, req_len, "/picoscript/picowal.html"))
        return http_build_picoscript_asset_response(out, max,
            IDE_PICOWAL_HTML, IDE_PICOWAL_HTML_LEN, "text/html; charset=utf-8");
    if (http_request_path_is(req, req_len, "/picoscript/pico_hooks.js"))
        return http_build_picoscript_asset_response(out, max,
            IDE_PICO_HOOKS_JS, IDE_PICO_HOOKS_JS_LEN, "application/javascript; charset=utf-8");
    if (http_request_path_is(req, req_len, "/picoscript/baremetal-binary.js"))
        return http_build_picoscript_asset_response(out, max,
            IDE_BAREMETAL_BINARY_JS, IDE_BAREMETAL_BINARY_JS_LEN, "application/javascript; charset=utf-8");
    /* Portal (root/index.html/playground.html and any other /picoscript request). */
    return http_build_picoscript_asset_response(out, max,
        IDE_HTML, IDE_HTML_LEN, "text/html; charset=utf-8");
}

static bool http_static_path_from_req(const u8 *req, u32 req_len, char *out, u32 out_max)
{
    char rel[256];
    if (!http_request_path_prefix_token(req, req_len, "/static/", rel, sizeof(rel)))
        return false;
    if (!out || out_max < 18)
        return false;
    u32 p = 0;
    static const char base[] = "/var/www/static/";
    for (u32 i = 0; base[i]; i++) out[p++] = base[i];
    for (u32 i = 0; rel[i]; i++) {
        char c = rel[i];
        if (c == '\\' || c == '\r' || c == '\n' || c == '\t')
            return false;
        if (c == '.' && rel[i + 1] == '.')
            return false;
        if ((u8)c < 0x20)
            return false;
        if (p + 1 >= out_max)
            return false;
        out[p++] = c;
    }
    out[p] = 0;
    return p > (u32)(sizeof(base) - 1);
}

static bool http_has_suffix(const char *s, const char *suffix)
{
    if (!s || !suffix) return false;
    u32 sl = pios_strlen(s);
    u32 tl = pios_strlen(suffix);
    if (tl > sl) return false;
    for (u32 i = 0; i < tl; i++)
        if (s[sl - tl + i] != suffix[i]) return false;
    return true;
}

static const char *http_content_type_for_path(const char *path)
{
    if (http_has_suffix(path, ".js")) return "application/javascript; charset=utf-8";
    if (http_has_suffix(path, ".css")) return "text/css; charset=utf-8";
    if (http_has_suffix(path, ".html")) return "text/html; charset=utf-8";
    if (http_has_suffix(path, ".json")) return "application/json; charset=utf-8";
    if (http_has_suffix(path, ".wasm")) return "application/wasm";
    if (http_has_suffix(path, ".woff2")) return "font/woff2";
    if (http_has_suffix(path, ".ttf")) return "font/ttf";
    if (http_has_suffix(path, ".svg")) return "image/svg+xml";
    if (http_has_suffix(path, ".ico")) return "image/x-icon";
    return "application/octet-stream";
}

static u32 http_build_static_file_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char path[256];
    if (!http_static_path_from_req(req, req_len, path, sizeof(path))) {
        http_append(out, &len, max,
            "HTTP/1.0 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nbad static path\n");
        return len;
    }
    u64 id = walfs_find(path);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        http_append(out, &len, max,
            "HTTP/1.0 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nstatic asset not found\n");
        return len;
    }
    http_file_id = id;
    http_file_len = ino.size > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (u32)ino.size;
    http_file_off = 0;
    http_append(out, &len, max, "HTTP/1.0 200 OK\r\nContent-Type: ");
    http_append(out, &len, max, http_content_type_for_path(path));
    http_append(out, &len, max, "\r\nCache-Control: max-age=3600\r\nContent-Length: ");
    http_append_u64(out, &len, max, http_file_len);
    http_append(out, &len, max, "\r\nConnection: close\r\n\r\n");
    return len;
}

/* On-device packet capture -- see pioscap.h. No host-side capture driver
 * required: exports a standard PCAP file that Wireshark can open directly.
 * GET /api/admin/pcap?action=status|start|stop|clear|dump (default status). */
static u32 http_build_pcap_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char action[16];
    action[0] = 0;
    (void)http_query_value(req, req_len, "/api/admin/pcap", "action", action, sizeof(action));

    if (http_streq(action, "start")) {
        pioscap_enable(true);
        http_append(out, &len, max,
            "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
            "Connection: close\r\n\r\n{\"ok\":true,\"action\":\"start\"}\n");
        return len;
    }
    if (http_streq(action, "stop")) {
        pioscap_enable(false);
        http_append(out, &len, max,
            "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
            "Connection: close\r\n\r\n{\"ok\":true,\"action\":\"stop\"}\n");
        return len;
    }
    if (http_streq(action, "clear")) {
        pioscap_clear();
        http_append(out, &len, max,
            "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
            "Connection: close\r\n\r\n{\"ok\":true,\"action\":\"clear\"}\n");
        return len;
    }
    if (http_streq(action, "dump")) {
        /* Paginated: the ring (4096 entries) is far bigger than one HTTP
         * response can hold. ?start=N selects the page starting at logical
         * entry N (oldest-first); only start=0 carries the 24-byte PCAP
         * global header. Response headers X-Pcap-Written/X-Pcap-Total tell
         * the caller how many entries this page held and how many exist in
         * total, so a client can loop start += written until start >= total
         * and concatenate the raw bytes into one .pcap file. */
        u32 start_index = 0;
        {
            char start_str[16];
            if (http_query_value(req, req_len, "/api/admin/pcap", "start", start_str, sizeof(start_str)))
                (void)http_parse_u32(start_str, &start_index);
        }
        static u8 pcap_scratch[15800];
        u32 written_count = 0, total_count = 0;
        u32 pcap_len = pioscap_export_paged(pcap_scratch, sizeof(pcap_scratch),
                                            start_index, PIOSCAP_PAGE_MAX,
                                            &written_count, &total_count);
        http_append(out, &len, max,
            "HTTP/1.0 200 OK\r\nContent-Type: application/vnd.tcpdump.pcap\r\n"
            "Content-Disposition: attachment; filename=\"pios_capture.pcap\"\r\n"
            "Cache-Control: no-store\r\nX-Pcap-Start: ");
        http_append_u64(out, &len, max, start_index);
        http_append(out, &len, max, "\r\nX-Pcap-Written: ");
        http_append_u64(out, &len, max, written_count);
        http_append(out, &len, max, "\r\nX-Pcap-Total: ");
        http_append_u64(out, &len, max, total_count);
        http_append(out, &len, max, "\r\nContent-Length: ");
        http_append_u64(out, &len, max, pcap_len);
        http_append(out, &len, max, "\r\nConnection: close\r\n\r\n");
        http_append_bytes(out, &len, max, pcap_scratch, pcap_len);
        return len;
    }

    /* Default: status */
    struct pioscap_status st;
    pioscap_status(&st);
    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\n\r\n");
    http_append(out, &len, max, "{\"enabled\":");
    http_append(out, &len, max, st.enabled ? "true" : "false");
    http_append(out, &len, max, ",\"frozen\":");
    http_append(out, &len, max, st.frozen ? "true" : "false");
    http_append(out, &len, max, ",\"freezeReason\":");
    http_append_json_string(out, &len, max, st.freeze_reason ? st.freeze_reason : "");
    http_append(out, &len, max, ",\"freezeTsMs\":");
    http_append_u64(out, &len, max, st.freeze_ts_ms);
    http_append(out, &len, max, ",\"count\":");
    http_append_u64(out, &len, max, st.count);
    http_append(out, &len, max, ",\"capacity\":");
    http_append_u64(out, &len, max, st.capacity);
    http_append(out, &len, max, ",\"dropped\":");
    http_append_u64(out, &len, max, st.dropped);
    http_append(out, &len, max, ",\"totalRx\":");
    http_append_u64(out, &len, max, st.total_rx);
    http_append(out, &len, max, ",\"totalTx\":");
    http_append_u64(out, &len, max, st.total_tx);
    http_append(out, &len, max,
        ",\"usage\":\"?action=start|stop|clear|status|dump[&start=N]\"}\n");
    return len;
}

static bool http_walfs_static_path_safe(const char *path)
{
    static const char prefix[] = "/var/www/static/";
    if (!path)
        return false;
    for (u32 i = 0; prefix[i]; i++)
        if (path[i] != prefix[i])
            return false;
    for (u32 i = 0; path[i]; i++) {
        char c = path[i];
        if (c == '\\' || c == '\r' || c == '\n' || c == '\t' || (u8)c < 0x20)
            return false;
        if (c == '.' && path[i + 1] == '.')
            return false;
    }
    return true;
}

static bool http_walfs_parent_leaf(const char *path, char *parent, u32 parent_max,
                                   char *leaf, u32 leaf_max)
{
    if (!path || path[0] != '/' || !parent || !leaf)
        return false;
    u32 len = pios_strlen(path);
    if (len < 2)
        return false;
    i32 slash = -1;
    for (u32 i = 0; i < len; i++)
        if (path[i] == '/') slash = (i32)i;
    if (slash < 0 || (u32)slash >= len - 1)
        return false;
    u32 nlen = len - (u32)slash - 1U;
    if (nlen + 1U > leaf_max)
        return false;
    for (u32 i = 0; i < nlen; i++)
        leaf[i] = path[(u32)slash + 1U + i];
    leaf[nlen] = 0;
    if (slash == 0) {
        if (parent_max < 2)
            return false;
        parent[0] = '/';
        parent[1] = 0;
        return true;
    }
    if ((u32)slash + 1U > parent_max)
        return false;
    for (u32 i = 0; i < (u32)slash; i++)
        parent[i] = path[i];
    parent[slash] = 0;
    return true;
}

static bool http_walfs_ensure_dir_path(const char *path)
{
    if (!path || path[0] != '/')
        return false;
    if (path[1] == 0)
        return true;
    char cur[256];
    u32 p = 0;
    cur[p++] = '/';
    cur[p] = 0;
    u64 parent_id = WALFS_ROOT_INODE;
    const char *s = path + 1;
    while (*s) {
        char seg[128];
        u32 n = 0;
        while (*s && *s != '/') {
            if (n + 1U >= sizeof(seg))
                return false;
            seg[n++] = *s++;
        }
        seg[n] = 0;
        if (*s == '/') s++;
        if (n == 0)
            return false;
        if (p > 1) {
            if (p + 1U >= sizeof(cur)) return false;
            cur[p++] = '/';
        }
        if (p + n + 1U >= sizeof(cur))
            return false;
        for (u32 i = 0; i < n; i++) cur[p++] = seg[i];
        cur[p] = 0;
        u64 id = walfs_find(cur);
        if (!id) {
            id = walfs_create(parent_id, seg, WALFS_DIR, 0755);
            if (!id)
                return false;
        } else {
            struct walfs_inode ino;
            if (!walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR))
                return false;
        }
        parent_id = id;
    }
    return true;
}

static bool http_walfs_ensure_file(const char *path, u64 *id_out)
{
    char parent[256], leaf[128];
    if (id_out) *id_out = 0;
    if (!http_walfs_static_path_safe(path) ||
        !http_walfs_parent_leaf(path, parent, sizeof(parent), leaf, sizeof(leaf)) ||
        !http_walfs_ensure_dir_path(parent))
        return false;
    u64 id = walfs_find(path);
    if (!id) {
        u64 parent_id = walfs_find(parent);
        if (!parent_id)
            return false;
        id = walfs_create(parent_id, leaf, WALFS_FILE, 0644);
        if (!id)
            return false;
    } else {
        struct walfs_inode ino;
        if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR))
            return false;
    }
    if (id_out) *id_out = id;
    return true;
}

static bool http_walfs_recreate_file(const char *path, u64 *id_out)
{
    char parent[256], leaf[128];
    if (id_out) *id_out = 0;
    if (!http_walfs_static_path_safe(path) ||
        !http_walfs_parent_leaf(path, parent, sizeof(parent), leaf, sizeof(leaf)) ||
        !http_walfs_ensure_dir_path(parent))
        return false;

    u64 old_id = walfs_find(path);
    if (old_id) {
        struct walfs_inode ino;
        if (!walfs_stat(old_id, &ino) || (ino.flags & WALFS_DIR))
            return false;
        if (!walfs_delete(old_id))
            return false;
    }

    u64 parent_id = walfs_find(parent);
    if (!parent_id)
        return false;
    u64 id = walfs_create(parent_id, leaf, WALFS_FILE, 0644);
    if (!id)
        return false;
    if (id_out) *id_out = id;
    return true;
}

static u32 http_build_static_upload_response(char *out, u32 max, const u8 *req, u32 req_len)
{
    u32 len = 0;
    char path[256], confirm[8], tmp[16];
    u32 offset = 0, total = 0;
    u32 body_off = http_header_body_offset(req, req_len);
    u32 body_len = body_off && body_off <= req_len ? req_len - body_off : 0;
    u32 content_len = 0;
    bool has_content_len = body_off && http_content_length(req, body_off, &content_len);
    bool ok = false;
    const char *error = NULL;
    u64 id = 0;

    path[0] = 0; confirm[0] = 0; tmp[0] = 0;
    (void)http_query_value(req, req_len, "/api/admin/static-put", "path", path, sizeof(path));
    (void)http_query_value(req, req_len, "/api/admin/static-put", "confirm", confirm, sizeof(confirm));
    if (http_query_value(req, req_len, "/api/admin/static-put", "offset", tmp, sizeof(tmp)))
        (void)http_parse_u32(tmp, &offset);
    tmp[0] = 0;
    if (http_query_value(req, req_len, "/api/admin/static-put", "total", tmp, sizeof(tmp)))
        (void)http_parse_u32(tmp, &total);

    if (!http_streq(confirm, "1")) error = "requires confirm=1";
    else if (!path[0] || !http_walfs_static_path_safe(path)) error = "bad static path";
    else if (!has_content_len || body_len != content_len) error = "incomplete body";
    else if (body_len > WALFS_DATA_MAX) error = "chunk too large";
    else if (total != 0 && (offset > total || body_len > total - offset)) error = "chunk exceeds total";
    else {
        if (offset == 0) {
            if (!http_walfs_recreate_file(path, &id)) {
                error = "recreate failed";
            } else {
                ok = walfs_replace(id, req + body_off, body_len);
                if (!ok) error = "replace failed";
            }
        } else {
            if (!http_walfs_ensure_file(path, &id)) {
                error = "create/find failed";
            } else {
                ok = walfs_write(id, offset, req + body_off, body_len);
                if (!ok) error = "write failed";
            }
        }
    }

    http_append(out, &len, max,
        "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n{\"ok\":");
    http_append(out, &len, max, ok ? "true" : "false");
    http_append(out, &len, max, ",\"path\":");
    http_append_json_string(out, &len, max, path);
    http_append(out, &len, max, ",\"offset\":");
    http_append_u64(out, &len, max, offset);
    http_append(out, &len, max, ",\"bytes\":");
    http_append_u64(out, &len, max, body_len);
    if (total) {
        http_append(out, &len, max, ",\"total\":");
        http_append_u64(out, &len, max, total);
    }
    if (!ok && error) {
        http_append(out, &len, max, ",\"error\":");
        http_append_json_string(out, &len, max, error);
    }
    http_append(out, &len, max, "}\n");
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
    http_static_body = NULL;
    http_static_len = 0;
    http_static_off = 0;
    http_file_id = 0;
    http_file_len = 0;
    http_file_off = 0;
    http_last_write = 0;
}

static void http_abort_client(void)
{
    http_trace(HTTP_EVT_ABORT, http_diag.route, http_diag.error,
               http_client_conn >= 0 ? (u32)http_client_conn : 0xFFFFFFFFU);
    if (http_client_conn >= 0)
        tcp_abort(http_client_conn);
    /* Multi-connection: abort only THIS slot's connection, never tcp_purge_port
     * (which would nuke every other in-flight :80 connection). */
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
    if (http_request_path_is(req, req_len, "/api/admin/static-put"))
        return http_build_static_upload_response(out, max, req, req_len);
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
        if (svc->stream_mode) {
            DTRACE(DTRACE_CAT_OTA, DT_OTA_WATCHDOG, svc->stream_received,
                   now - svc->last_activity_ms, svc->stream_total, svc->port);
            pioscap_notify_event("ota-stall-watchdog");
        }
        admin_service_restart(svc);
    }

    if (svc->client_conn < 0 && svc->listen_conn >= 0) {
        svc->client_conn = tcp_accept(svc->listen_conn);
        if (svc->client_conn >= 0) {
            svc->req_len = 0;
            svc->resp_len = 0;
            svc->resp_off = 0;
            svc->stream_mode = false;
            svc->stream_received = 0;
            svc->stream_total = 0;
            svc->stream_reboot = false;
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

    /* ── Streaming OTA upload ── pump the single large POST body straight into
     * the RAM staging buffer. One connection => no per-chunk churn that overruns
     * the polling NIC. When the whole image is staged, flush it to the SD slot
     * and (optionally) reboot into it. */
    if (svc->stream_mode) {
        u32 readable = tcp_readable(svc->client_conn);
        /* REACTOR_GAP: trace how long since the last time we ran this drain block.
         * A large gap during OTA is the starvation signature we are hunting. */
        u64 t_now; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t_now));
        if (svc->stream_last_drain) {
            u64 gap = t_now - svc->stream_last_drain;
            u64 cntfrq; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(cntfrq));
            if (gap > cntfrq / 2000U) /* >~0.5ms = interesting */
                DTRACE(DTRACE_CAT_OTA, DT_OTA_REACTOR_GAP, gap,
                       svc->stream_received, svc->stream_total, 0);
        }
        svc->stream_last_drain = t_now;
        /* Tight drain: while the stream is active, pump the NIC RX + drain this
         * connection in a bounded inner loop so the upload runs at wire speed
         * regardless of the core0 reactor cadence or :80 load. The board has no
         * other active services during OTA, so dedicating core0 to the stream
         * here is the right tradeoff. Bounded by stage space and a spin budget so
         * it always returns to the reactor (watchdog pets, dashboard, etc). */
        u32 drain_spins = 0;
        while (ota_stage_buf && svc->stream_received < svc->stream_total &&
               drain_spins < 4096U) {
            drain_spins++;
            if (readable == 0) {
                net_poll();   /* ingest more inbound frames into the TCP rx ring */
                readable = tcp_readable(svc->client_conn);
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
                /* Unlike the main core0 reactor (which pairs every net_poll()
                 * with a macb_rx_recover()/macb_rx_liveness_recover() check),
                 * this tight drain loop used to call net_poll() alone. A
                 * genuine hardware RX overrun triggered by the sustained
                 * burst of a bulk transfer would then sit un-recovered until
                 * the loop exhausted its spin budget (or the connection was
                 * abandoned) and control finally returned to the outer
                 * reactor -- long enough that the peer's TCP stack gave up
                 * and reset the connection. Recover inline instead, and log
                 * enough context (drain_spins, stream progress, lifetime
                 * recovery count) to read back on the next capture/dtrace
                 * dump without needing to catch it live. */
                if (readable == 0) {
                    bool recovered = macb_rx_recover();
                    if (!recovered)
                        recovered = macb_rx_hole_recover();
                    if (!recovered)
                        recovered = macb_rx_liveness_recover(timer_monotonic_ms());
                    if (recovered) {
                        struct macb_diag md_post;
                        macb_diag(&md_post);
                        DTRACE(DTRACE_CAT_OTA, DT_OTA_RX_RECOVER, svc->stream_received,
                               svc->stream_total, drain_spins,
                               md_post.rx_recover + md_post.rx_live_recover);
                        http_log_event("ota-rx-recover", svc->stream_received, drain_spins);
                        net_poll();
                        readable = tcp_readable(svc->client_conn);
                    }
                }
#endif
                if (readable == 0)
                    break;    /* genuinely nothing available right now */
            }
            u32 want = svc->stream_total - svc->stream_received;
            if (want > readable) want = readable;
            if (svc->stream_received + want > ota_stage_cap)
                want = ota_stage_cap - svc->stream_received;
            u32 n = tcp_read(svc->client_conn, ota_stage_buf + svc->stream_received, want);
            if (n == 0)
                break;
            svc->stream_received += n;
            ota_update.received = svc->stream_received;
            svc->last_activity_ms = now;
            readable = tcp_readable(svc->client_conn);
            if ((drain_spins & 0xFFU) == 0)
                watchdog_hw_pet();
        }
        DTRACE(DTRACE_CAT_OTA, DT_OTA_CHUNK, svc->stream_received, svc->stream_total,
               drain_spins, tcp_readable(svc->client_conn));
        if (drain_spins >= 4096U)
            DTRACE(DTRACE_CAT_OTA, DT_OTA_DRAIN_SPIN, svc->stream_received, drain_spins,
                   svc->stream_total, 0);
        if (svc->stream_received < svc->stream_total && readable == 0) {
            /* Waiting for more inbound data (typically the final sub-MSS tail).
             * Periodically re-advertise our receive window so a single lost
             * window-update ACK self-heals: otherwise the peer waits forever for
             * a window it never heard reopened, the (now-empty) rx ring gives
             * tcp_read nothing to drain so nothing re-ACKs, and the 10s stall
             * watchdog resets the connection ~248 bytes short of a 1.2MB upload. */
            if (now - svc->stream_wadv_ms > 200ULL) {
                tcp_advertise_window(svc->client_conn);
                svc->stream_wadv_ms = now;
                svc->stream_wadv_count++;
                DTRACE(DTRACE_CAT_OTA, DT_OTA_WADV,
                       svc->stream_received, svc->stream_total,
                       (u64)tcp_readable(svc->client_conn),
                       svc->stream_wadv_count);
            }
        }
        if (svc->stream_received >= svc->stream_total) {
            bool fok = ota_stage_buf &&
                http_write_kernel_payload_range(ota_update.target_slot_offset, 0,
                                                ota_stage_buf, svc->stream_total, NULL) &&
                http_write_kernel_slot_header(ota_update.target_slot_offset,
                                              svc->stream_total, true);
            svc->resp_len = 0;
            http_append(svc->resp, &svc->resp_len, sizeof(svc->resp),
                "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n");
            if (fok) {
                ota_update.active = false;
                ota_update.commits++;
                DTRACE(DTRACE_CAT_OTA, DT_OTA_COMMIT, svc->stream_total, ota_update.commits, 0, 0);
                pios_bootctrl_mark_pending(ota_update.target_slot);
                http_log_event("ota-stream-commit", svc->stream_total, ota_update.commits);
                http_append(svc->resp, &svc->resp_len, sizeof(svc->resp),
                    "{\"ok\":true,\"streamed\":true,\"reboot\":");
                http_append(svc->resp, &svc->resp_len, sizeof(svc->resp),
                    svc->stream_reboot ? "true}\n" : "false}\n");
                if (svc->stream_reboot)
                    http_reboot_pending = true;
            } else {
                ota_update.errors++;
                ota_update.last_error = "stream flush failed";
                http_append(svc->resp, &svc->resp_len, sizeof(svc->resp),
                    "{\"ok\":false,\"error\":\"stream flush failed\"}\n");
            }
            svc->resp_off = 0;
            svc->stream_mode = false;
        } else if (now - svc->last_activity_ms > 10000ULL) {
            /* stalled mid-stream: abort (slot left uncommitted = safe) */
            ota_update.active = false;
            ota_update.last_error = "stream stalled";
            svc->stream_mode = false;
            admin_service_clear_client(svc, true);
        }
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
        /* Switch to streaming as soon as the HEADERS are complete (a multi-MB
         * body would never fit svc->req, so we must not wait for the full
         * request). http_header_body_offset() != 0 means headers are done. */
        u32 body_off = http_header_body_offset(svc->req, svc->req_len);
        bool is_stream = body_off != 0 && svc == &admin_update_svc &&
                         http_request_path_is(svc->req, svc->req_len, "/api/admin/kernel-stream") &&
                         http_update_confirmed(svc->req, svc->req_len);
        if (is_stream) {
            u32 total = http_update_query_u32_default(svc->req, svc->req_len, "total", 0);
            char reboot[8];
            bool want_reboot = http_update_query_value(svc->req, svc->req_len, "reboot",
                                                       reboot, sizeof(reboot)) &&
                               http_streq(reboot, "1");
            ota_staging_init();
            if (total == 0 || total > ota_stage_cap || !ota_stage_buf) {
                svc->resp_len = 0;
                http_append(svc->resp, &svc->resp_len, sizeof(svc->resp),
                    "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
                    "{\"ok\":false,\"error\":\"stream unavailable or bad total\"}\n");
                svc->resp_off = 0;
            } else {
                ota_update.target_slot = pios_bootctrl_target_slot();
                ota_update.target_slot_offset = pios_boot_slot_offset(ota_update.target_slot);
                ota_update.total = total;
                ota_update.received = 0;
                ota_update.active = true;
                ota_update.last_error = NULL;
                http_write_kernel_slot_header(ota_update.target_slot_offset, total, false);
                svc->stream_mode = true;
                svc->stream_total = total;
                svc->stream_reboot = want_reboot;
                svc->stream_received = 0;
                svc->stream_wadv_ms = 0;
                svc->stream_wadv_count = 0;
                svc->stream_last_drain = 0;
                DTRACE(DTRACE_CAT_OTA, DT_OTA_BEGIN, total, want_reboot ? 1U : 0U, svc->port, 0);
                http_log_event("ota-stream-begin", total, want_reboot ? 1U : 0U);
                /* Any body bytes already read in with the headers go to staging. */
                if (body_off < svc->req_len) {
                    u32 have = svc->req_len - body_off;
                    if (have > total) have = total;
                    simd_memcpy(ota_stage_buf, svc->req + body_off, have);
                    svc->stream_received = have;
                    ota_update.received = have;
                }
            }
            svc->last_activity_ms = now;
        } else if (http_request_complete(svc->req, svc->req_len) ||
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
            if (chunk > HTTP_TX_CHUNK_MAX) chunk = HTTP_TX_CHUNK_MAX;
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
                /* Accumulate the (possibly multi-segment, POST-bodied) request
                 * across polls until it is structurally complete. A single
                 * gated read cannot hold a POST login carrying an
                 * X-PIOS-Password header plus body, so append each readable
                 * chunk and only dispatch once http_request_complete() is true.
                 * Fail closed if the buffer fills before the request completes. */
                if (tcp_readable(https_tls_tcp_conn) > 0 &&
                    https_tls_req_len < sizeof(https_tls_req_buf)) {
                    i32 rn = tls_read(https_tls_conn,
                                      https_tls_req_buf + https_tls_req_len,
                                      sizeof(https_tls_req_buf) - https_tls_req_len);
                    if (rn > 0) {
                        https_tls_req_len += (u32)rn;
                        https_tls_last_activity_ms = timer_monotonic_ms();
                    }
                }

                bool buf_full = https_tls_req_len >= sizeof(https_tls_req_buf);
                bool complete = https_tls_req_len > 0 &&
                    http_request_complete(https_tls_req_buf, https_tls_req_len);

                if (!complete && !buf_full) {
                    /* Still waiting for the rest of the request. Drop the
                     * connection if the peer goes quiet for too long. */
                    if ((timer_monotonic_ms() - https_tls_last_activity_ms) > 5000ULL) {
                        tls_close(https_tls_conn);
                        https_tls_tcp_conn = -1;
                        https_tls_conn = -1;
                    }
                } else {
                    /* Route PicoSTS endpoints over TLS (via_tls=true) so the
                     * TLS-only login/validate/whoami/users routes are reachable
                     * only here, never on plaintext :80. Non-STS paths keep the
                     * minimal health banner. */
                    static char https_resp[8192];
                    char sts_tail[64];
                    i32 wn = -1;
                    if (buf_full && !complete) {
                        /* Oversized / incomplete request -> fail closed. */
                        static const char toobig[] =
                            "HTTP/1.0 431 Request Header Fields Too Large\r\n"
                            "Content-Type: application/json\r\n"
                            "Connection: close\r\n\r\n"
                            "{\"ok\":false,\"error\":\"request_too_large\"}\n";
                        wn = tls_write(https_tls_conn, toobig, sizeof(toobig) - 1);
                    } else if (http_request_path_prefix_token(https_tls_req_buf,
                                   https_tls_req_len, "/api/sts", sts_tail, sizeof(sts_tail))) {
                        u32 rlen = http_build_sts_response(https_resp, sizeof(https_resp),
                                                           https_tls_req_buf, https_tls_req_len, true);
                        /* tls_write cannot safely split a record: a response
                         * larger than one TLS record would be rejected by
                         * tls_write anyway, so fail closed with a small 500
                         * rather than emitting a truncated/oversized frame. */
                        if (rlen == 0U || rlen > STS_TLS_RECORD_MAX) {
                            static const char too_big[] =
                                "HTTP/1.0 500 Internal Server Error\r\n"
                                "Content-Type: application/json\r\n"
                                "Connection: close\r\n\r\n"
                                "{\"ok\":false,\"error\":\"response_too_large\"}\n";
                            wn = tls_write(https_tls_conn, too_big, sizeof(too_big) - 1);
                        } else {
                            wn = tls_write(https_tls_conn, https_resp, rlen);
                        }
                    } else {
                        static const char resp[] =
                            "HTTP/1.0 200 OK\r\n"
                            "Content-Type: text/plain\r\n"
                            "Content-Length: 20\r\n"
                            "Connection: close\r\n\r\n"
                            "PIOS kernel TLS 443\n";
                        wn = tls_write(https_tls_conn, resp, sizeof(resp) - 1);
                    }
                    if (wn > 0) {
                        https_tls_response_sent = true;
                        https_tls_last_activity_ms = timer_monotonic_ms();
                        http_log_event("tls443-response", HTTPS_TLS_TCP_PORT, (u32)wn);
                        tls_close(https_tls_conn);
                        https_tls_tcp_conn = -1;
                        https_tls_conn = -1;
                    } else {
                        /* tls_write cannot partially queue a record, so a
                         * non-positive result is terminal: reset the sole
                         * connection immediately instead of retrying forever. */
                        http_log_event("tls443-write-fail", HTTPS_TLS_TCP_PORT, (u32)wn);
                        tls_close(https_tls_conn);
                        https_tls_tcp_conn = -1;
                        https_tls_conn = -1;
                    }
                }
            } else if ((timer_monotonic_ms() - https_tls_last_activity_ms) > 1000ULL) {
                tls_close(https_tls_conn);
                https_tls_tcp_conn = -1;
                https_tls_conn = -1;
            }
        }
    }

    /* HTTP :80 — accept new clients into free pool slots (up to a batch per
     * poll), then below we service ALL active slots so many connections are
     * handled concurrently instead of one-at-a-time. */
    if (http_listen_conn >= 0) {
        for (u32 a = 0; a < 4U; a++) {
            struct http_conn *slot = NULL;
            for (u32 i = 0; i < http_conn_count; i++) {
                if (http_conns[i].client_conn < 0) { slot = &http_conns[i]; break; }
            }
            if (!slot)
                break;                       /* pool full */
            tcp_conn_t c = tcp_accept(http_listen_conn);
            if (c < 0)
                break;                       /* no more pending connections */
            Hc = slot;
            http_client_conn = c;
            http_diag.accepts++;
            http_diag.conn = (u32)c;
            http_diag.route = HTTP_ROUTE_UNKNOWN;
            http_diag.error = HTTP_ERR_NONE;
            http_trace(HTTP_EVT_ACCEPT, HTTP_ROUTE_UNKNOWN, (u32)c, http_diag.accepts);
            http_req_len = 0;
            http_auth_checked = false;
            http_auth_ok = false;
            http_resp_len = 0;
            http_resp_off = 0;
            http_last_write = 0;
            http_prefix_dumped = false;
            http_req_prefix_dumped = false;
            http_static_body = NULL;
            http_static_len = 0;
            http_static_off = 0;
            http_file_id = 0;
            http_file_len = 0;
            http_file_off = 0;
            http_last_req_prefix[0] = 0;
            http_last_resp_prefix[0] = 0;
            http_last_activity_ms = timer_monotonic_ms();
            if (HTTP_DIAG_VERBOSE) uart_puts("[http] accepted\n");
        }
    }
    /* Service active slots round-robin, BOUNDED to a few per poll so this loop
     * returns quickly and core0's net_poll (RX drain) — which runs after it in
     * the core0 reactor — isn't starved. Without this bound, serving many
     * connections per poll overloads single-core0 and overruns the RX ring. */
    static u32 http_rr;
    u32 http_served = 0, http_scanned = 0;
    while (http_scanned < http_conn_count && http_served < 6U) {
        u32 hi = (http_rr + http_scanned) % http_conn_count;
        http_scanned++;
        if (http_conns[hi].client_conn < 0)
            continue;
        Hc = &http_conns[hi];
        http_served++;
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
                    if (chunk > HTTP_TX_CHUNK_MAX) chunk = HTTP_TX_CHUNK_MAX;
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
            } else if (http_resp_len > 0 && http_static_body && http_static_off < http_static_len) {
                u32 writable = tcp_writable(http_client_conn);
                u32 remain = http_static_len - http_static_off;
                if (writable > 0) {
                    u32 chunk = remain < writable ? remain : writable;
                    if (chunk > 512) chunk = 512;
                    u32 n = tcp_write(http_client_conn, http_static_body + http_static_off, chunk);
                    http_diag.write_calls++;
                    http_last_write = n;
                    if (n == 0) {
                        http_diag.write_zero++;
                    } else {
                        http_diag.write_bytes += n;
                        http_last_activity_ms = timer_monotonic_ms();
                    }
                    http_static_off += n;
                }
            } else if (http_resp_len > 0 && http_file_id && http_file_off < http_file_len) {
                u32 writable = tcp_writable(http_client_conn);
                u32 remain = http_file_len - http_file_off;
                if (writable > 0) {
                    u32 chunk = remain < writable ? remain : writable;
                    if (chunk > sizeof(http_file_chunk)) chunk = sizeof(http_file_chunk);
                    u32 got = walfs_read(http_file_id, http_file_off, http_file_chunk, chunk);
                    if (got == 0) {
                        http_diag.error = HTTP_ERR_RESP_TIMEOUT;
                        http_abort_client();
                    } else {
                        u32 n = tcp_write(http_client_conn, http_file_chunk, got);
                        http_diag.write_calls++;
                        http_last_write = n;
                        if (n == 0) {
                            http_diag.write_zero++;
                        } else {
                            http_diag.write_bytes += n;
                            http_last_activity_ms = timer_monotonic_ms();
                        }
                        http_file_off += n;
                    }
                }
            }

            if (http_resp_len > 0 && http_resp_off >= http_resp_len &&
                (!http_static_body || http_static_off >= http_static_len) &&
                (!http_file_id || http_file_off >= http_file_len)) {
                if (tcp_tx_pending(http_client_conn) == 0) {
                    http_diag.closes++;
                    http_complete_tick++;
                    http_trace(HTTP_EVT_CLOSE, http_diag.route, http_resp_len, http_resp_off);
                    if (HTTP_DIAG_VERBOSE) uart_puts("[http] close complete\n");
                    if (http_reboot_pending) {
                        timer_delay_ms(250);
                        watchdog_reboot_now(0x48545450U);
                    }
                    http_reset_client(true);
                } else {
                    http_last_activity_ms = timer_monotonic_ms();
                }
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
                if (http_req_len == 0) {
                    /* Browsers may open speculative/preconnect sockets and never
                     * send a request line. Close those quietly; only partial
                     * requests are actionable request timeouts. */
                    http_diag.closes++;
                    http_trace(HTTP_EVT_CLOSE, HTTP_ROUTE_UNKNOWN, 0,
                               http_client_conn >= 0 ? (u32)http_client_conn : 0xFFFFFFFFU);
                    http_reset_client(true);
                } else {
                    http_diag.aborts++;
                    http_diag.error = HTTP_ERR_REQ_TIMEOUT;
                    http_abort_client();
                }
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
    if (http_conn_count)
        http_rr = (http_rr + http_scanned) % http_conn_count;
    /* admin_services_poll() is intentionally NOT called here anymore. It used to
     * be nested at the end of this heavy :80 handler, which starved the OTA
     * admin drain when the multi-connection :80 pool was busy. It is now driven
     * directly from the core0 reactor (before this handler) so OTA streaming is
     * never starved by :80 load. */
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
static void ui_cmd_uartflash(u32 argc, char **argv);
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
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_log_y = BP_LIST_ROW + BP_COUNT + 2;
    uart_puts("\n[bt] PIOS ");
    uart_puts(PIOS_BUILD_LABEL);
    uart_puts(" Boot\n");
    for (u32 i = 0; i < BP_COUNT; i++)
        bp_uart_phase(i, "pending");
    return;
#endif
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
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_uart_phase(phase, "active");
    return;
#endif
    u32 row = BP_LIST_ROW + phase;
    fb_set_cursor(4, row);
    fb_set_color(BOOT_FG_WHITE, BOOT_BLACK);
    fb_puts(bp_names[phase]);
    fb_set_cursor(BP_STAT_COL, row);
    fb_set_color(BOOT_FG_WHITE, BOOT_BLACK);
    fb_puts("[..]");
    bp_uart_phase(phase, "active");
    fb_present();
}

/* Mark a phase as done: show [OK] or [!!] */
static void bp_done(u32 phase, bool ok) {
    if (phase >= BP_COUNT) return;
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_uart_phase(phase, ok ? "ok" : "failed");
    return;
#endif
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
    fb_present();
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
    fb_present();
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
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_uart_line("[diag] ", msg);
    return;
#endif
    fb_set_color(BOOT_FG_LOG, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[diag] ", msg);
    fb_present();
}

/* Green log — success */
static void bp_ok(const char *msg) {
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_uart_line("[ok] ", msg);
    return;
#endif
    fb_set_color(BOOT_FG_OK, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[ok] ", msg);
    fb_present();
}

/* Red log — error */
static void bp_err(const char *msg) {
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_uart_line("[err] ", msg);
    return;
#endif
    fb_set_color(BOOT_FG_FAIL, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[err] ", msg);
    fb_present();
}

/* Yellow log — warning */
static void bp_warn(const char *msg) {
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_uart_line("[warn] ", msg);
    return;
#endif
    fb_set_color(BOOT_YELLOW, BOOT_BLACK);
    bp_timestamp();
    fb_puts(msg);
    bp_log_y++;
    bp_uart_line("[warn] ", msg);
    fb_present();
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

static void ui_console_hdmi_reset(void)
{
    u32 rows = fb_rows();
    u32 cols = fb_cols();
    if (rows == 0 || cols == 0)
        return;

    u32 panel_top = rows / 2U;
    if (panel_top + 3U >= rows)
        panel_top = 0;

    fb_set_color(UI_SHELL_TEXT_COLOR, UI_SHELL_BG_COLOR);
    for (u32 row = panel_top; row < rows; row++)
        fb_clear_row(row);

    fb_set_cursor(0, panel_top);
    fb_puts(" PIOS TERMINAL | UART + TCP/2323 + HDMI");
    fb_set_cursor(0, panel_top + 1U);
    fb_hline(cols);

    fb_set_reserved_rows(panel_top + 2U);
    fb_set_cursor(0, panel_top + 2U);
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

static void ui_console_ip(u32 ip)
{
    /* Route via ui_console_write() (UART + HDMI + TCP) instead of the old
     * fb_printf()-only implementation, which left this address invisible
     * over the TCP/2323 debug console and physical UART alike -- same
     * class of bug fixed for "dtrace dump"/"hexsec"/"capsule". */
    ui_console_u32_dec((ip >> 24) & 0xFF);
    ui_console_write(".");
    ui_console_u32_dec((ip >> 16) & 0xFF);
    ui_console_write(".");
    ui_console_u32_dec((ip >> 8) & 0xFF);
    ui_console_write(".");
    ui_console_u32_dec(ip & 0xFF);
}

static void ui_console_route_table(void)
{
    struct net_route_entry routes[NET_ROUTE_MAX];
    u32 n = net_route_snapshot(routes, NET_ROUTE_MAX);
    ui_console_write("ROUTE DST MASK GW FLAGS PFX\n");
    for (u32 i = 0; i < n; i++) {
        ui_console_u32_dec(i);
        ui_console_write(" ");
        ui_console_ip(routes[i].dst);
        ui_console_write(" ");
        ui_console_ip(routes[i].mask);
        ui_console_write(" ");
        ui_console_ip(routes[i].gateway);
        ui_console_write(" ");
        ui_console_u32_dec(routes[i].flags);
        ui_console_write(" ");
        ui_console_u32_dec(routes[i].prefix_len);
        ui_console_write("\n");
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
        ui_console_u32_dec(e[i].state);
        ui_console_write(" ");
        ui_console_u32_dec(e[i].retries);
        ui_console_write(" ");
        ui_console_u32_dec(e[i].consistency);
        ui_console_write(" ");
        ui_console_u32_dec((u32)e[i].age_ms);
        ui_console_write("\n");
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

static void ui_cmd_picocompress(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "selftest")) {
        ui_console_write(pc_selftest() ? "Picocompress selftest OK\n" : "Picocompress selftest FAILED\n");
        return;
    }
    ui_console_write("ERR: usage picocompress selftest\n");
}

static void ui_cmd_picoweb(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "selftest")) {
        ui_console_write(picoweb_selftest() ? "PicoWeb selftest OK\n" : "PicoWeb selftest FAILED\n");
        return;
    }
    ui_console_write("ERR: usage picoweb selftest\n");
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
    ui_console_write(" native=");
    ui_console_write(ts.v3d_native_probe_ok ? "yes" : "no");
    ui_console_write(" nself=");
    ui_console_write(ts.v3d_native_selftest_ok ? "yes" : "no");
    ui_console_write(" ncomp=");
    ui_console_write(ts.v3d_native_compute_enabled ? "yes" : "no");
    ui_console_write(" nmmu=");
    ui_console_write(ts.v3d_native_mmu_ready ? "yes" : "no");
    ui_console_write(" tiny=");
    ui_console_write(ts.v3d_native_tiny_kernels_ready ? "yes" : "no");
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
    ui_console_write(" tv=");
    ui_console_hex_fixed(ts.v3d_tech_version, 8);
    ui_console_write(" cores=");
    ui_console_hex_fixed(ts.v3d_core_count, 8);
    ui_console_write(" qps=");
    ui_console_hex_fixed(ts.v3d_qpus_per_slice, 8);
    ui_console_write(" slices=");
    ui_console_hex_fixed(ts.v3d_slice_count, 8);
    ui_console_write(" csd=");
    ui_console_hex_fixed(ts.v3d_csd_status, 8);
    ui_console_write(" nstat=");
    ui_console_hex_fixed((u32)ts.v3d_native_selftest_status, 8);
    ui_console_write(" mmuctl=");
    ui_console_hex_fixed(ts.v3d_mmu_ctl, 8);
    ui_console_write(" mmuc=");
    ui_console_hex_fixed(ts.v3d_mmuc_control, 8);
    ui_console_write(" tiny_ready=");
    ui_console_hex_fixed(ts.v3d_tiny_ready_mask, 8);
    ui_console_write(" tiny_ver=");
    ui_console_hex_fixed(ts.v3d_tiny_verified_mask, 8);
    ui_console_write("\n");
}

static void ui_cmd_tensor(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        ui_console_write(tensor_selftest() ? "Tensor selftest OK\n" : "Tensor selftest FAILED\n");
        ui_print_tensor_status();
        return;
    }
    if (argc >= 3 && ui_streq(argv[1], "tiny") && ui_streq(argv[2], "noop")) {
        bool ok = tensor_tiny_noop_proof();
        ui_console_write(ok ? "Tensor tiny noop proof OK stage=" : "Tensor tiny noop proof FAILED stage=");
        ui_console_hex_fixed((u32)tensor_tiny_last_stage(), 8);
        ui_console_write(" status=");
        ui_console_hex_fixed((u32)tensor_tiny_last_status(), 8);
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        ui_console_write(" csd_st=");
        ui_console_hex_fixed(dbg.status_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_kick, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_wait, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(dbg.core_int_sts, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.hub_int_sts, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(dbg.err_stat, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    if (argc >= 3 && ui_streq(argv[1], "tiny") &&
        (ui_streq(argv[2], "store") || ui_streq(argv[2], "store-only"))) {
        bool ok = tensor_tiny_store_proof();
        ui_console_write(ok ? "Tensor tiny store proof OK stage=" : "Tensor tiny store proof FAILED stage=");
        ui_console_hex_fixed((u32)tensor_tiny_last_stage(), 8);
        ui_console_write(" status=");
        ui_console_hex_fixed((u32)tensor_tiny_last_status(), 8);
        ui_console_write(" out=");
        ui_console_hex_fixed(tensor_tiny_last_output_bits(), 8);
        ui_console_write(" expect=");
        ui_console_hex_fixed(tensor_tiny_last_expected_bits(), 8);
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        ui_console_write(" csd_st=");
        ui_console_hex_fixed(dbg.status_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_kick, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_wait, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(dbg.core_int_sts, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.hub_int_sts, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(dbg.err_stat, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    if (argc >= 3 && ui_streq(argv[1], "tiny") &&
        (ui_streq(argv[2], "loadstore") || ui_streq(argv[2], "load-store"))) {
        bool ok = tensor_tiny_load_store_proof();
        ui_console_write(ok ? "Tensor tiny load-store proof OK stage=" : "Tensor tiny load-store proof FAILED stage=");
        ui_console_hex_fixed((u32)tensor_tiny_last_stage(), 8);
        ui_console_write(" status=");
        ui_console_hex_fixed((u32)tensor_tiny_last_status(), 8);
        ui_console_write(" out=");
        ui_console_hex_fixed(tensor_tiny_last_output_bits(), 8);
        ui_console_write(" expect=");
        ui_console_hex_fixed(tensor_tiny_last_expected_bits(), 8);
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        ui_console_write(" csd_st=");
        ui_console_hex_fixed(dbg.status_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_kick, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_wait, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(dbg.core_int_sts, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.hub_int_sts, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(dbg.err_stat, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    if (argc >= 3 && ui_streq(argv[1], "tiny") &&
        (ui_streq(argv[2], "memory") || ui_streq(argv[2], "mem"))) {
        bool ok = tensor_tiny_memory_proof();
        ui_console_write(ok ? "Tensor tiny memory proof OK stage=" : "Tensor tiny memory proof FAILED stage=");
        ui_console_hex_fixed((u32)tensor_tiny_last_stage(), 8);
        ui_console_write(" status=");
        ui_console_hex_fixed((u32)tensor_tiny_last_status(), 8);
        ui_console_write(" out=");
        ui_console_hex_fixed(tensor_tiny_last_output_bits(), 8);
        ui_console_write(" expect=");
        ui_console_hex_fixed(tensor_tiny_last_expected_bits(), 8);
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        ui_console_write(" csd_st=");
        ui_console_hex_fixed(dbg.status_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_kick, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_wait, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(dbg.core_int_sts, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.hub_int_sts, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(dbg.err_stat, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "tiny")) {
        bool ok = tensor_tiny_selftest();
        ui_console_write(ok ? "Tensor tiny selftest OK stage=" : "Tensor tiny selftest FAILED stage=");
        ui_console_hex_fixed((u32)tensor_tiny_last_stage(), 8);
        ui_console_write(" status=");
        ui_console_hex_fixed((u32)tensor_tiny_last_status(), 8);
        ui_console_write(" out=");
        ui_console_hex_fixed(tensor_tiny_last_output_bits(), 8);
        ui_console_write(" expect=");
        ui_console_hex_fixed(tensor_tiny_last_expected_bits(), 8);
        struct v3d_csd_debug dbg;
        v3d_csd_debug_last(&dbg);
        ui_console_write(" csd_st=");
        ui_console_hex_fixed(dbg.status_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_kick, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.status_after_wait, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(dbg.core_int_sts, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.hub_int_sts, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(dbg.err_stat, 8);
        ui_console_write(" mmu=");
        ui_console_hex_fixed(dbg.mmu_ctl, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.mmu_illegal_addr, 8);
        ui_console_write(" vio=");
        ui_console_hex_fixed(dbg.mmu_vio_addr, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.mmu_vio_id, 8);
        ui_console_write(" cur=");
        ui_console_hex_fixed(dbg.current_cfg0, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.current_cfg5, 8);
        ui_console_write("/");
        ui_console_hex_fixed(dbg.current_cfg6, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "status")) {
        ui_print_tensor_status();
        return;
    }
    if (argc >= 3 && ui_streq(argv[1], "reset") && ui_streq(argv[2], "soft")) {
        watchdog_hw_arm_seconds(5);
        v3d_status_t st = v3d_soft_reset();
        watchdog_hw_disable();
        struct v3d_reset_debug r;
        v3d_reset_debug_last(&r);
        ui_console_write(st == V3D_STATUS_OK ? "v3d reset soft OK status=" : "v3d reset soft FAILED status=");
        ui_console_hex_fixed((u32)st, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(r.err_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.err_after_clear, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.err_after_reset, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(r.core_int_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.hub_int_before, 8);
        ui_console_write("->");
        ui_console_hex_fixed(r.core_int_after, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.hub_int_after, 8);
        ui_console_write(" sms=");
        ui_console_hex_fixed(r.sms_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.sms_after, 8);
        ui_console_write(" mmu=");
        ui_console_hex_fixed(r.mmu_ctl_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.mmuc_before, 8);
        ui_console_write("->");
        ui_console_hex_fixed(r.mmu_ctl_after, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.mmuc_after, 8);
        ui_console_write(" pm=");
        ui_console_hex_fixed(r.pm_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.pm_asserted, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.pm_after, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    if (argc >= 4 && ui_streq(argv[1], "reset") && ui_streq(argv[2], "pm") &&
        ui_streq(argv[3], "confirm")) {
        watchdog_hw_arm_seconds(5);
        v3d_status_t st = v3d_pm_reset();
        watchdog_hw_disable();
        struct v3d_reset_debug r;
        v3d_reset_debug_last(&r);
        ui_console_write(st == V3D_STATUS_OK ? "v3d reset pm OK status=" : "v3d reset pm FAILED status=");
        ui_console_hex_fixed((u32)st, 8);
        ui_console_write(" err=");
        ui_console_hex_fixed(r.err_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.err_after_clear, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.err_after_reset, 8);
        ui_console_write(" int=");
        ui_console_hex_fixed(r.core_int_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.hub_int_before, 8);
        ui_console_write("->");
        ui_console_hex_fixed(r.core_int_after, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.hub_int_after, 8);
        ui_console_write(" sms=");
        ui_console_hex_fixed(r.sms_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.sms_after, 8);
        ui_console_write(" mmu=");
        ui_console_hex_fixed(r.mmu_ctl_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.mmuc_before, 8);
        ui_console_write("->");
        ui_console_hex_fixed(r.mmu_ctl_after, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.mmuc_after, 8);
        ui_console_write(" pm=");
        ui_console_hex_fixed(r.pm_before, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.pm_asserted, 8);
        ui_console_write("/");
        ui_console_hex_fixed(r.pm_after, 8);
        ui_console_write("\n");
        ui_print_tensor_status();
        return;
    }
    ui_console_write("ERR: usage qpu status | qpu reset soft | qpu reset pm confirm | qpu selftest | qpu tiny | tensor status | tensor selftest | tensor tiny\n");
}

static void ui_print_dns_status(void)
{
    struct dns_async_status ds;
    dns_async_status(&ds);
    ui_console_write("dns state=");
    ui_console_u32_dec(ds.state);
    ui_console_write(" server=");
    ui_console_ip(ds.server_ip);
    ui_console_write(" result=");
    ui_console_ip(ds.result_ip);
    ui_console_write(" attempts=");
    ui_console_u32_dec(ds.attempts);
    ui_console_write(" error=");
    ui_console_u32_dec(ds.last_error);
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
    /* Render via ui_console_write() (UART + HDMI + TCP) instead of the old
     * split fb_printf()/uart_puts() pair -- the fb_printf() side never
     * reached UART/TCP at all, and the separate uart_puts() side was a
     * reduced-field duplicate that never reached HDMI/TCP; same class of
     * bug fixed for "dtrace dump"/"hexsec"/"capsule"/"watchdog". */
    ui_console_write("net link=");
    ui_console_write(nic_link_up() ? "up" : "down");
    ui_console_write(" mode=");
    ui_console_write(ui_cfg_dhcp ? "dhcp" : "static");
    ui_console_write(" ip=");
    ui_console_ip(net_get_our_ip());
    ui_console_write(" mask=");
    ui_console_ip(ui_cfg_mask);
    ui_console_write(" gw=");
    ui_console_ip(ui_cfg_gw);
    ui_console_write(" dns=");
    ui_console_ip(ui_cfg_dns);
    ui_console_write("\nmac=");
    for (u32 i = 0; i < 6; i++) {
        if (i) ui_console_write(":");
        ui_console_hex_fixed(mac[i], 2);
    }
    ui_console_write("\ntx=");
    ui_console_hex_fixed(st->tx_packets, 8);
    ui_console_write(" rx=");
    ui_console_hex_fixed(st->rx_packets, 8);
    ui_console_write(" udp_tx=");
    ui_console_hex_fixed(st->udp_sent, 8);
    ui_console_write(" udp_rx=");
    ui_console_hex_fixed(st->udp_recv, 8);
    ui_console_write(" drops=");
    ui_console_hex_fixed(st->drop_runt + st->drop_bad_cksum + st->drop_fragment + st->drop_ip_options +
                          st->drop_bad_src + st->drop_not_for_us + st->drop_bad_proto +
                          st->drop_icmp_ratelimit + st->drop_no_neighbor + st->drop_udp_malformed + st->drop_oversized,
                          8);
    ui_console_write("\n");
    ui_console_route_table();
    ui_console_arp_table();
}

static void ui_cmd_disk(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "info")) {
        /* Render via ui_console_write() (UART + HDMI + TCP) instead of the
         * old direct fb_printf()/uart_puts() pair, which was invisible over
         * the TCP debug console -- same class of bug fixed for "dtrace dump"
         * and "hexsec". */
        const sd_card_t *card = sd_get_card_info();
        char line[128];
        u32 len = 0;
        http_append(line, &len, sizeof(line), "disk type=");
        http_append_u64(line, &len, sizeof(line), card->type);
        http_append(line, &len, sizeof(line), " rca=0x");
        http_append_hex32(line, &len, sizeof(line), card->rca);
        http_append(line, &len, sizeof(line), " cap_bytes=0x");
        http_append_hex32(line, &len, sizeof(line), (u32)card->capacity);
        http_append(line, &len, sizeof(line), "\n");
        line[len] = 0;
        ui_console_write(line);
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
        char line[192];
        u32 len = 0;
        http_append(line, &len, sizeof(line), "walfs verify: ok=");
        http_append_u64(line, &len, sizeof(line), ok ? 1U : 0U);
        http_append(line, &len, sizeof(line), " super=");
        http_append_u64(line, &len, sizeof(line), h.super_ok ? 1U : 0U);
        http_append(line, &len, sizeof(line), " head=");
        http_append_u64(line, &len, sizeof(line), h.wal_head_ok ? 1U : 0U);
        http_append(line, &len, sizeof(line), " rec=");
        http_append_u64(line, &len, sizeof(line), h.valid_records);
        http_append(line, &len, sizeof(line), " crc_err=");
        http_append_u64(line, &len, sizeof(line), h.crc_errors);
        http_append(line, &len, sizeof(line), " hdr_err=");
        http_append_u64(line, &len, sizeof(line), h.header_errors);
        http_append(line, &len, sizeof(line), " open_tx=");
        http_append_u64(line, &len, sizeof(line), h.open_tx ? 1U : 0U);
        http_append(line, &len, sizeof(line), " scan_end=0x");
        http_append_hex32(line, &len, sizeof(line), h.scan_end);
        http_append(line, &len, sizeof(line), "\n");
        line[len] = 0;
        ui_console_write(line);
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
        ui_console_write("db card=");
        ui_console_u32_dec(card);
        ui_console_write(" count=");
        ui_console_u32_dec(n);
        ui_console_write("\n");
        for (u32 i = 0; i < n; i++) {
            ui_console_write("  rec=");
            ui_console_u32_dec(ids[i]);
            ui_console_write("\n");
        }
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
        ui_console_write("key=0x");
        ui_console_hex_fixed(key, 8);
        ui_console_write(" card=");
        ui_console_u32_dec(card);
        ui_console_write(" record=");
        ui_console_u32_dec(rec);
        ui_console_write("\n");
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
        if (n < 0) {
            ui_console_write("ERR: put failed\n");
        } else {
            ui_console_write("OK: wrote ");
            ui_console_u32_dec((u32)n);
            ui_console_write(" bytes\n");
        }
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
        if (n < 0) {
            ui_console_write("ERR: putf failed\n");
        } else {
            ui_console_write("OK: wrote ");
            ui_console_u32_dec((u32)n);
            ui_console_write(" bytes\n");
        }
        return;
    }

    if (ui_streq(argv[1], "get")) {
        static u8 data[PICOWAL_DATA_MAX];
        static char printable[PICOWAL_DATA_MAX + 1];
        i32 n = picowal_db_get((u16)card, rec, data, PICOWAL_DATA_MAX);
        if (n < 0) {
            ui_console_write("ERR: get failed\n");
            return;
        }
        ui_console_write("db card=");
        ui_console_u32_dec(card);
        ui_console_write(" rec=");
        ui_console_u32_dec(rec);
        ui_console_write(" len=");
        ui_console_u32_dec((u32)n);
        ui_console_write("\n");
        for (i32 i = 0; i < n; i++) {
            char c = (char)data[i];
            if (c < 0x20 || c > 0x7E) c = '.';
            printable[i] = c;
        }
        printable[n] = 0;
        ui_console_write(printable);
        ui_console_write("\n");
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
        ui_console_write("OK: wrote file bytes=");
        ui_console_u32_dec((u32)n);
        ui_console_write("\n");
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
    /* Render into a line buffer and fan out via ui_console_write() (UART +
     * HDMI console + TCP/2323) instead of writing straight to uart_puts()/
     * fb_printf(): the old direct-to-UART/HDMI calls made "hexsec" invisible
     * to anyone using the TCP debug console, the same class of bug fixed for
     * "dtrace dump". */
    static const char hex[] = "0123456789ABCDEF";
    char line[96];
    u32 len = 0;
    http_append(line, &len, sizeof(line), "LBA 0x");
    http_append_hex32(line, &len, sizeof(line), lba);
    http_append(line, &len, sizeof(line), "\n");
    line[len] = 0;
    ui_console_write(line);
    for (u32 off = 0; off < SD_BLOCK_SIZE; off += 16) {
        len = 0;
        http_append_hex32(line, &len, sizeof(line), off);
        http_append(line, &len, sizeof(line), ": ");
        for (u32 i = 0; i < 16; i++) {
            u8 b = sector[off + i];
            line[len++] = hex[(b >> 4) & 0xF];
            line[len++] = hex[b & 0xF];
            line[len++] = ' ';
        }
        line[len++] = '|';
        for (u32 i = 0; i < 16; i++) {
            char c = (char)sector[off + i];
            if (c < 0x20 || c > 0x7E) c = '.';
            line[len++] = c;
        }
        line[len++] = '|';
        line[len++] = '\n';
        line[len] = 0;
        ui_console_write(line);
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

    ui_console_hdmi_reset();
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
            ui_console_write("pid=");
            ui_console_hex_fixed(e[i].pid, 8);
            ui_console_write(" core=");
            ui_console_u32_dec(e[i].affinity_core);
            ui_console_write(" cap=");
            ui_console_u32_dec(e[i].capsule_id);
            ui_console_write(" hash=");
            ui_console_hex_fixed(e[i].capsule_hash, 8);
            ui_console_write(" grp=");
            ui_console_write(e[i].group[0] ? e[i].group : "-");
            ui_console_write(" vfs=");
            ui_console_write(e[i].vfs_root[0] ? e[i].vfs_root : "-");
            ui_console_write("\n");
        }
        ui_console_write("capsule ls done\n");
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
        ui_console_write("capsule=");
        ui_console_u32_dec(id);
        ui_console_write(" st=");
        ui_console_hex_fixed(st, 16);
        ui_console_write(" faults=");
        ui_console_hex_fixed(faults, 16);
        ui_console_write("\n");
        {
            u32 fc = 0, active = 0, last_cap = 0;
            u64 esr = 0, elr = 0, far_ipa = 0, sp = 0;
            bool faulted_el0 = false;
            u32 core = core_id();
            if (el2_stage2_fault_detail(core, &fc, &esr, &elr, &far_ipa, &sp,
                                        &faulted_el0, &active, &last_cap)) {
                ui_console_write("core=");
                ui_console_u32_dec(core);
                ui_console_write(" active=");
                ui_console_u32_dec(active);
                ui_console_write(" last_fault_cap=");
                ui_console_u32_dec(last_cap);
                ui_console_write(" count=");
                ui_console_u32_dec(fc);
                ui_console_write("\nesr=");
                ui_console_hex_fixed(esr, 16);
                ui_console_write(" elr=");
                ui_console_hex_fixed(elr, 16);
                ui_console_write(" far_ipa=");
                ui_console_hex_fixed(far_ipa, 16);
                ui_console_write(faulted_el0 ? " sp_el0=" : " sp_el1=");
                ui_console_hex_fixed(sp, 16);
                ui_console_write("\n");
            }
        }
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

static void uartflash_send_line(const char *prefix, u32 a, u32 b, bool has_b)
{
    char line[64];
    u32 l = 0;
    http_append(line, &l, sizeof(line), prefix);
    http_append_u64(line, &l, sizeof(line), a);
    if (has_b) {
        http_append(line, &l, sizeof(line), " ");
        char hex[16];
        u32 hl = 0;
        for (i32 shift = 28; shift >= 0; shift -= 4) {
            u32 nib = (b >> shift) & 0xFU;
            hex[hl++] = (char)(nib < 10 ? '0' + nib : 'a' + nib - 10);
        }
        hex[hl] = 0;
        http_append(line, &l, sizeof(line), hex);
    }
    http_append(line, &l, sizeof(line), "\n");
    ui_console_write(line);
}

/* Reads `len` bytes (rounded up to SD block granularity) from the given
 * slot's raw storage into ota_stage_buf, used as the patch base for
 * PATCH-mode transfers: the device reads its OWN actual flash content as
 * ground truth rather than trusting any host-side cache to be accurate. */
static bool uartflash_read_slot_into_stage(u32 slot_offset, u32 len)
{
    if (!ota_stage_buf || len > ota_stage_cap)
        return false;
    u32 lba_base = walfs_partition_lba() + (slot_offset / SD_BLOCK_SIZE);
    u32 nblocks = (len + SD_BLOCK_SIZE - 1U) / SD_BLOCK_SIZE;
    for (u32 i = 0; i < nblocks; i++) {
        if (!sd_read_block(lba_base + i, ota_stage_buf + (u64)i * SD_BLOCK_SIZE))
            return false;
    }
    return true;
}

/* Called once per received UART byte while a transfer is in progress
 * (bypasses the normal ASCII-filtering console line editor, which would
 * corrupt binary data). FULL mode writes straight into the staging buffer
 * at the cumulative offset; PATCH_REGION mode writes at the fixed
 * offset/length given by the preceding "uartflash patch" command. */
static void uartflash_feed_byte(u8 b)
{
    if (!uartflash_active || !ota_stage_buf)
        return;
    if (uartflash_mode == UARTFLASH_MODE_FULL) {
        if (ota_update.received >= ota_update.total)
            return;
        u32 remaining = ota_update.total - ota_update.received;
        u32 want = remaining < UARTFLASH_CHUNK_SIZE ? remaining : UARTFLASH_CHUNK_SIZE;
        ota_stage_buf[ota_update.received + uartflash_chunk_progress] = b;
        uartflash_chunk_progress++;
        uartflash_last_byte_ms = timer_monotonic_ms();
        if (uartflash_chunk_progress >= want) {
            u32 sum = 0;
            for (u32 i = 0; i < want; i++)
                sum = (sum * 31U) + ota_stage_buf[ota_update.received + i];
            ota_update.received += want;
            ota_update.chunks++;
            uartflash_chunk_progress = 0;
            uartflash_send_line("ACK ", ota_update.received, sum, true);
        }
    } else if (uartflash_mode == UARTFLASH_MODE_PATCH_REGION) {
        if ((u64)uartflash_patch_offset + uartflash_chunk_progress >= ota_stage_cap)
            return; /* out-of-bounds safety net; the "patch" command already validates this */
        ota_stage_buf[uartflash_patch_offset + uartflash_chunk_progress] = b;
        uartflash_chunk_progress++;
        uartflash_last_byte_ms = timer_monotonic_ms();
        if (uartflash_chunk_progress >= uartflash_patch_length) {
            u32 sum = 0;
            for (u32 i = 0; i < uartflash_patch_length; i++)
                sum = (sum * 31U) + ota_stage_buf[uartflash_patch_offset + i];
            ota_update.chunks++;
            uartflash_chunk_progress = 0;
            uartflash_mode = UARTFLASH_MODE_NONE; /* region done; awaiting next "uartflash patch" */
            uartflash_send_line("PACK ", uartflash_patch_offset, sum, true);
        }
    }
}

/* Called every reactor tick while a transfer is in progress: detects a
 * stalled chunk (host stopped sending mid-chunk, e.g. dropped bytes or a
 * crashed sender) and tells the host to resend from the last fully-acked
 * offset, rather than waiting forever with partial, uncounted bytes sitting
 * in the staging buffer. */
static void uartflash_poll(void)
{
    if (!uartflash_active || uartflash_chunk_progress == 0)
        return;
    if (timer_monotonic_ms() - uartflash_last_byte_ms > UARTFLASH_TIMEOUT_MS) {
        uartflash_chunk_progress = 0;
        if (uartflash_mode == UARTFLASH_MODE_PATCH_REGION) {
            uartflash_mode = UARTFLASH_MODE_NONE;
            uartflash_send_line("TIMEOUT ", uartflash_patch_offset, 0, false);
        } else {
            uartflash_send_line("TIMEOUT ", ota_update.received, 0, false);
        }
    }
}

static void ui_cmd_uartflash(u32 argc, char **argv)
{
    if (!principal_has_cap(principal_current(), PRINCIPAL_ADMIN)) {
        ui_console_write("ERR: admin required\n");
        return;
    }
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("uartflash begin <total> | uartflash patchbegin <base_len> | "
                         "uartflash patch <offset> <length> | uartflash finalize <total> | "
                         "uartflash commit [reboot] | uartflash abort | uartflash status\n");
        return;
    }
    if (ui_streq(argv[1], "begin")) {
        if (uartflash_active) {
            ui_console_write("ERR: a transfer is already in progress; uartflash abort first\n");
            return;
        }
        u32 total = 0;
        if (argc < 3 || !ui_parse_u32(argv[2], &total) || total == 0 ||
            total > PIOS_STAGE2_ZONE_BYTES) {
            ui_console_write("ERR: usage uartflash begin <total>; total must fit the raw payload slot\n");
            return;
        }
        if (!ota_stage_buf) {
            struct highmem_status hm;
            highmem_status(&hm);
            if (hm.ready)
                ota_stage_buf = (u8 *)highmem_alloc(PIOS_STAGE2_ZONE_BYTES, 64);
            if (ota_stage_buf)
                ota_stage_cap = PIOS_STAGE2_ZONE_BYTES;
        }
        if (!ota_stage_buf || total > ota_stage_cap) {
            ui_console_write("ERR: no staging buffer available (highmem not ready)\n");
            return;
        }
        ota_update.target_slot = pios_bootctrl_target_slot();
        ota_update.target_slot_offset = pios_boot_slot_offset(ota_update.target_slot);
        ota_update.active = true;
        ota_update.total = total;
        ota_update.received = 0;
        ota_update.chunks = 0;
        ota_update.errors = 0;
        ota_stage_ready = true;
        uartflash_active = true;
        uartflash_mode = UARTFLASH_MODE_FULL;
        uartflash_chunk_progress = 0;
        uartflash_last_byte_ms = timer_monotonic_ms();
        http_log_event("uartflash-begin", total, ota_update.target_slot);
        char line[80];
        u32 l = 0;
        http_append(line, &l, sizeof(line), "READY chunk=");
        http_append_u64(line, &l, sizeof(line), UARTFLASH_CHUNK_SIZE);
        http_append(line, &l, sizeof(line), " slot=");
        http_append_u64(line, &l, sizeof(line), ota_update.target_slot);
        http_append(line, &l, sizeof(line), "\n");
        ui_console_write(line);
        return;
    }
    if (ui_streq(argv[1], "patchbegin")) {
        if (uartflash_active) {
            ui_console_write("ERR: a transfer is already in progress; uartflash abort first\n");
            return;
        }
        u32 base_len = 0;
        if (argc < 3 || !ui_parse_u32(argv[2], &base_len) || base_len == 0 ||
            base_len > PIOS_STAGE2_ZONE_BYTES) {
            ui_console_write("ERR: usage uartflash patchbegin <base_len>\n");
            return;
        }
        if (!ota_stage_buf) {
            struct highmem_status hm;
            highmem_status(&hm);
            if (hm.ready)
                ota_stage_buf = (u8 *)highmem_alloc(PIOS_STAGE2_ZONE_BYTES, 64);
            if (ota_stage_buf)
                ota_stage_cap = PIOS_STAGE2_ZONE_BYTES;
        }
        if (!ota_stage_buf) {
            ui_console_write("ERR: no staging buffer available (highmem not ready)\n");
            return;
        }
        ota_update.target_slot = pios_bootctrl_target_slot();
        ota_update.target_slot_offset = pios_boot_slot_offset(ota_update.target_slot);
        /* Read the CURRENT target-slot content back off the SD card as the
         * patch base. Ground truth from the device's own flash, not a
         * host-side cache -- if the host's assumption about what's on the
         * slot is stale/wrong, the patch offsets would corrupt an
         * unrelated region, so this only ever reads real, current bytes. */
        if (!uartflash_read_slot_into_stage(ota_update.target_slot_offset, base_len)) {
            ui_console_write("ERR: failed to read current slot content as patch base\n");
            return;
        }
        ota_update.active = true;
        ota_update.total = base_len;   /* provisional; "finalize" sets the real final length */
        ota_update.received = base_len;
        ota_update.chunks = 0;
        ota_update.errors = 0;
        ota_stage_ready = true;
        uartflash_active = true;
        uartflash_mode = UARTFLASH_MODE_NONE;
        uartflash_chunk_progress = 0;
        http_log_event("uartflash-patchbegin", base_len, ota_update.target_slot);
        ui_console_write("OK: patch base loaded\n");
        return;
    }
    if (ui_streq(argv[1], "patch")) {
        if (!uartflash_active) {
            ui_console_write("ERR: uartflash patchbegin first\n");
            return;
        }
        u32 poff = 0, plen = 0;
        if (argc < 4 || !ui_parse_u32(argv[2], &poff) || !ui_parse_u32(argv[3], &plen) ||
            plen == 0 || plen > UARTFLASH_CHUNK_SIZE || (u64)poff + plen > ota_stage_cap) {
            ui_console_write("ERR: usage uartflash patch <offset> <length>; length must be <= 1024\n");
            return;
        }
        uartflash_mode = UARTFLASH_MODE_PATCH_REGION;
        uartflash_patch_offset = poff;
        uartflash_patch_length = plen;
        uartflash_chunk_progress = 0;
        uartflash_last_byte_ms = timer_monotonic_ms();
        ui_console_write("OK: awaiting patch bytes\n");
        return;
    }
    if (ui_streq(argv[1], "finalize")) {
        if (!uartflash_active) {
            ui_console_write("ERR: uartflash patchbegin first\n");
            return;
        }
        u32 total = 0;
        if (argc < 3 || !ui_parse_u32(argv[2], &total) || total == 0 || total > ota_stage_cap) {
            ui_console_write("ERR: usage uartflash finalize <total>\n");
            return;
        }
        ota_update.total = total;
        ota_update.received = total;
        uartflash_mode = UARTFLASH_MODE_NONE;
        ui_console_write("OK: finalized\n");
        return;
    }
    if (ui_streq(argv[1], "status")) {
        char line[96];
        u32 l = 0;
        http_append(line, &l, sizeof(line), "uartflash active=");
        http_append(line, &l, sizeof(line), uartflash_active ? "1" : "0");
        http_append(line, &l, sizeof(line), " received=");
        http_append_u64(line, &l, sizeof(line), ota_update.received);
        http_append(line, &l, sizeof(line), " total=");
        http_append_u64(line, &l, sizeof(line), ota_update.total);
        http_append(line, &l, sizeof(line), " chunks=");
        http_append_u64(line, &l, sizeof(line), ota_update.chunks);
        http_append(line, &l, sizeof(line), "\n");
        ui_console_write(line);
        return;
    }
    if (ui_streq(argv[1], "abort")) {
        uartflash_active = false;
        uartflash_mode = UARTFLASH_MODE_NONE;
        uartflash_chunk_progress = 0;
        ota_update_reset_state();
        ui_console_write("OK: uartflash aborted\n");
        return;
    }
    if (ui_streq(argv[1], "commit")) {
        if (!uartflash_active || !ota_update.active) {
            ui_console_write("ERR: no uartflash transfer active\n");
            return;
        }
        if (ota_update.received != ota_update.total) {
            ui_console_write("ERR: transfer incomplete; use uartflash status to check progress\n");
            return;
        }
        u32 written = 0;
        if (!ota_stage_ready ||
            !http_write_kernel_payload_range(ota_update.target_slot_offset, 0,
                                             ota_stage_buf, ota_update.total, &written)) {
            ui_console_write("ERR: failed to flush staged image to slot\n");
            return;
        }
        if (!http_write_kernel_slot_header(ota_update.target_slot_offset,
                                           ota_update.received, true)) {
            ui_console_write("ERR: failed to commit slot header\n");
            return;
        }
        ota_update.active = false;
        ota_update.commits++;
        uartflash_active = false;
        pios_bootctrl_mark_pending(ota_update.target_slot);
        http_log_event("uartflash-commit", ota_update.received, ota_update.commits);
        bool reboot = argc >= 3 && ui_streq(argv[2], "reboot");
        ui_console_write(reboot ? "OK: committed, rebooting...\n" : "OK: committed\n");
        if (reboot) {
            timer_delay_ms(100);
            watchdog_reboot_now(0x55415246U); /* "UARF" */
        }
        return;
    }
    ui_console_write("ERR: unknown uartflash subcommand\n");
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
        u32 rem = watchdog_hw_remaining_ticks();
        u32 rstc = watchdog_hw_rstc();
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
    } else if (ui_streq(topic, "picocompress")) {
        ui_console_write("picocompress selftest\n  Verify upstream-compatible 508-byte block codec and encoder stats.\n");
    } else if (ui_streq(topic, "picoweb")) {
        ui_console_write("picoweb selftest\n  Verify upstream-compatible route dispatch, params, headers, and status text.\n");
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

static void ui_cmd_crypto(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "selftest")) {
        bool ok = crypto_selftest();
        ui_console_write(ok ? "OK: crypto selftest passed (AES-GCM + GHASH nibble table)\n"
                            : "ERR: crypto selftest FAILED\n");
        return;
    }
    ui_console_write("usage: crypto selftest\n");
}

static void ui_cmd_arp(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "probe")) {
        arp_probe();
        const arp_stats_t *st = arp_get_stats();
        ui_console_write("OK: arp probe sent requests_sent=");
        ui_console_u64_dec(st->requests_sent);
        ui_console_write(" learned=");
        ui_console_u64_dec(st->learned);
        ui_console_write(" conflicts=");
        ui_console_u64_dec(st->conflicts);
        ui_console_write("\n");
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "status")) {
        const arp_stats_t *st = arp_get_stats();
        ui_console_write("arp requests_sent=");
        ui_console_u64_dec(st->requests_sent);
        ui_console_write(" replies_sent=");
        ui_console_u64_dec(st->replies_sent);
        ui_console_write(" learned=");
        ui_console_u64_dec(st->learned);
        ui_console_write(" drop_spoof=");
        ui_console_u64_dec(st->drop_spoof);
        ui_console_write(" drop_ratelimit=");
        ui_console_u64_dec(st->drop_ratelimit);
        ui_console_write(" conflicts=");
        ui_console_u64_dec(st->conflicts);
        ui_console_write("\n");
        return;
    }
    ui_console_write("usage: arp probe | arp status\n");
}

static void ui_cmd_nic(u32 argc, char **argv)
{
    if (argc >= 3 && ui_streq(argv[1], "dump")) {
        bool on = ui_streq(argv[2], "on") || ui_streq(argv[2], "1");
        nic_set_packet_dump(on);
        ui_console_write(on ? "OK: nic packet dump ENABLED\n"
                            : "OK: nic packet dump disabled\n");
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "counters")) {
        nic_packet_counters_t c;
        nic_packet_counters(&c);
        ui_console_write("nic processed=");
        ui_console_u64_dec(c.processed);
        ui_console_write(" dropped=");
        ui_console_u64_dec(c.dropped);
        ui_console_write(" firewalled=");
        ui_console_u64_dec(c.firewalled);
        ui_console_write(" rate_limited=");
        ui_console_u64_dec(c.rate_limited);
        ui_console_write(" rx_arp_not_us=");
        ui_console_u64_dec(c.rx_arp_not_us);
        ui_console_write(" flood_blocked=");
        ui_console_u64_dec(c.flood_blocked);
        /* Direction-specific totals avoid the legacy mixed RX/TX counters. */
        ui_console_write(" rx_total=");
        ui_console_u64_dec(c.rx_total);
        ui_console_write(" tx_total=");
        ui_console_u64_dec(c.tx_total);
        ui_console_write(" tx_csum_hw=");
        ui_console_u64_dec(c.tx_csum_offloaded);
        ui_console_write(" tx_csum_sw=");
        ui_console_u64_dec(c.tx_csum_software);
        ui_console_write(" rx_csum_trusted=");
        ui_console_u64_dec(c.rx_csum_trusted);
        ui_console_write(" rx_csum_untrusted=");
        ui_console_u64_dec(c.rx_csum_untrusted);
        ui_console_write("\n");
        return;
    }
    if (argc >= 2 && ui_streq(argv[1], "offload")) {
        nic_offload_status_t s;
        nic_offload_status(&s);
        ui_console_write("nic offload tx_csum_cap=");
        ui_console_write(s.tx_checksum_capable ? "yes" : "no");
        ui_console_write(" tx_csum=");
        ui_console_write(s.tx_checksum_enabled ? "on" : "off");
        ui_console_write(" rx_csum_cap=");
        ui_console_write(s.rx_checksum_capable ? "yes" : "no");
        ui_console_write(" rx_csum=");
        ui_console_write(s.rx_checksum_enabled ? "on" : "off");
        ui_console_write(" tso_cap=");
        ui_console_write(s.tso_capable ? "yes" : "no");
        ui_console_write(" tso=");
        ui_console_write(s.tso_enabled ? "on" : "off");
        ui_console_write(" tx_hw=");
        ui_console_u64_dec(s.tx_csum_offloaded);
        ui_console_write(" tx_sw=");
        ui_console_u64_dec(s.tx_csum_software);
        ui_console_write(" rx_trusted=");
        ui_console_u64_dec(s.rx_csum_trusted);
        ui_console_write(" rx_untrusted=");
        ui_console_u64_dec(s.rx_csum_untrusted);
        ui_console_write(" ncfgr=");
        ui_console_hex_fixed(s.mac_ncfgr, 8);
        ui_console_write(" dmacfg=");
        ui_console_hex_fixed(s.mac_dmacfg, 8);
        ui_console_write("\n");
        return;
    }
    ui_console_write("usage: nic dump <on|off> | nic counters | nic offload\n");
}

/* Fallback for the UART/F3 console: when ui_console_exec()'s own argv-based
 * dispatch doesn't recognise argv[0], reconstruct a flat space-separated
 * command string and hand it to the SAME shared handler the HTTP
 * /api/terminal endpoint uses (http_exec_terminal_command()) -- this is
 * what closes the gap for commands like "macbdiag"/"rp1 irq"/"stackdiag"
 * that previously only existed over HTTP, without duplicating any of
 * their ~150-command logic. Uses static buffers (not stack) since the
 * reconstructed command / rendered output can be sizeable and this runs
 * on the shared core0 console context, not a recursive/re-entrant path. */
static void ui_console_exec_shared_fallback(u32 argc, char **argv)
{
    static char joined[UI_CONSOLE_LINE_MAX];
    static char rendered[8192];
    u32 jl = 0;
    for (u32 i = 0; i < argc; i++) {
        if (i > 0 && jl + 1 < sizeof(joined))
            joined[jl++] = ' ';
        u32 tl = 0;
        while (argv[i][tl] && jl + 1 < sizeof(joined)) {
            joined[jl++] = argv[i][tl++];
        }
    }
    joined[jl] = 0;

    u32 len = 0;
    http_exec_terminal_command(rendered, &len, sizeof(rendered), joined);
    rendered[len < sizeof(rendered) ? len : sizeof(rendered) - 1] = 0;
    if (ui_streq(rendered, "unknown command\n"))
        ui_console_write("ERR: unknown command\n");
    else
        ui_console_write(rendered);
}

static void ui_cmd_cachestats(u32 argc, char **argv)
{
    (void)argc; (void)argv;
    u64 ih = 0, im = 0, ph = 0, pm = 0, dh = 0, dm = 0, dev = 0, ah = 0, am = 0, aev = 0;
    walfs_cache_stats(&ih, &im, &ph, &pm);
    dns_cache_stats(&dh, &dm, &dev);
    arp_cache_stats(&ah, &am, &aev);
    ui_console_write("cache walfs inode_hit=");
    ui_console_u64_dec(ih);
    ui_console_write(" inode_miss=");
    ui_console_u64_dec(im);
    ui_console_write(" path_hit=");
    ui_console_u64_dec(ph);
    ui_console_write(" path_miss=");
    ui_console_u64_dec(pm);
    ui_console_write("\ncache dns hit=");
    ui_console_u64_dec(dh);
    ui_console_write(" miss=");
    ui_console_u64_dec(dm);
    ui_console_write(" evict=");
    ui_console_u64_dec(dev);
    ui_console_write("\ncache arp hit=");
    ui_console_u64_dec(ah);
    ui_console_write(" miss=");
    ui_console_u64_dec(am);
    ui_console_write(" evict=");
    ui_console_u64_dec(aev);
    ui_console_write("\n");
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
            ui_console_write("  crypto selftest  arp probe  nic dump <on|off>|counters  cachestats\n");
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
            ui_console_hdmi_reset();
            ui_console_write("PIOS F3 Console (serial + HDMI)\n");
        }
    } else if (ui_streq(argv[0], "time")) {
        if (argc < 2 || ui_streq(argv[1], "time")) {
            ui_console_write("ERR: usage time <command>\n");
        } else {
            char nested[UI_CONSOLE_LINE_MAX];
            u32 pos = 0;
            for (u32 i = 1; i < argc; i++) {
                const char *s = argv[i];
                while (*s && pos + 1U < sizeof(nested))
                    nested[pos++] = *s++;
                if (i + 1U < argc && pos + 1U < sizeof(nested))
                    nested[pos++] = ' ';
            }
            nested[pos] = 0;
            u64 start = read_cntvct();
            ui_console_exec(nested);
            u64 end = read_cntvct();
            ui_console_write("elapsed_ticks=");
            ui_console_u64_dec(end >= start ? end - start : 0);
            ui_console_write("\n");
        }
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
    } else if (ui_streq(argv[0], "picocompress")) {
        ui_cmd_picocompress(argc, argv);
    } else if (ui_streq(argv[0], "picoweb")) {
        ui_cmd_picoweb(argc, argv);
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
    } else if (ui_streq(argv[0], "vc") || ui_streq(argv[0], "videocore")) {
        if (argc >= 4 && ui_streq(argv[1], "display") &&
            (ui_streq(argv[2], "global") || ui_streq(argv[2], "hvs")) &&
            ui_streq(argv[3], "reapply")) {
            bool dry = argc >= 5 && (ui_streq(argv[4], "dryrun") || ui_streq(argv[4], "dry-run"));
            bool ok = vc_display_global_reapply(dry);
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display global reapply OK status=" :
                                  "vc_display global reapply FAILED status=");
            ui_console_write(d ? vc_display_global_status_name(d->global_reapply_status) : "none");
            ui_console_write(" control=");
            ui_console_hex_fixed(d ? d->global_reapply_control : 0U, 8);
            ui_console_write(" rb=");
            ui_console_hex_fixed(d ? d->global_reapply_rb_control : 0U, 8);
            ui_console_write(" readback=");
            ui_console_write(d && d->global_reapply_readback_ok ? "ok" : (dry ? "dryrun" : "bad"));
            ui_console_write("\n");
        } else if (argc >= 4 && ui_streq(argv[1], "display") && ui_streq(argv[2], "channel") &&
            ui_streq(argv[3], "reapply")) {
            bool dry = argc >= 5 && (ui_streq(argv[4], "dryrun") || ui_streq(argv[4], "dry-run"));
            bool ok = vc_display_channel_reapply(dry);
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display channel reapply OK status=" :
                                  "vc_display channel reapply FAILED status=");
            ui_console_write(d ? vc_display_channel_status_name(d->channel_reapply_status) : "none");
            ui_console_write(" ch=");
            ui_console_u32_dec(d ? d->channel_reapply_channel : 0U);
            ui_console_write(" ctrl=");
            ui_console_hex_fixed(d ? d->channel_reapply_ctrl0 : 0U, 8);
            ui_console_write("/");
            ui_console_hex_fixed(d ? d->channel_reapply_ctrl1 : 0U, 8);
            ui_console_write(" cob=");
            ui_console_hex_fixed(d ? d->channel_reapply_cob : 0U, 8);
            ui_console_write(" rb=");
            ui_console_hex_fixed(d ? d->channel_reapply_rb_ctrl0 : 0U, 8);
            ui_console_write("/");
            ui_console_hex_fixed(d ? d->channel_reapply_rb_ctrl1 : 0U, 8);
            ui_console_write("/");
            ui_console_hex_fixed(d ? d->channel_reapply_rb_cob : 0U, 8);
            ui_console_write(" readback=");
            ui_console_write(d && d->channel_reapply_readback_ok ? "ok" : (dry ? "dryrun" : "bad"));
            ui_console_write("\n");
        } else if (argc >= 4 && ui_streq(argv[1], "display") && ui_streq(argv[2], "dlist") &&
            ui_streq(argv[3], "arm")) {
            bool dry = argc >= 5 && (ui_streq(argv[4], "dryrun") || ui_streq(argv[4], "dry-run"));
            bool ok = vc_display_dlist_arm(dry);
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display dlist arm OK status=" :
                                  "vc_display dlist arm FAILED status=");
            ui_console_write(d ? vc_display_dlist_status_name(d->dlist_status) : "none");
            ui_console_write(" ch=");
            ui_console_u32_dec(d ? d->dlist_arm_channel : 0U);
            ui_console_write(" before=");
            ui_console_hex_fixed(d ? d->dlist_arm_lptrs_before : 0U, 8);
            ui_console_write(" after=");
            ui_console_hex_fixed(d ? d->dlist_arm_lptrs_after : 0U, 8);
            ui_console_write(" readback=");
            ui_console_write(d && d->dlist_arm_readback_ok ? "ok" : (dry ? "dryrun" : "bad"));
            ui_console_write("\n");
        } else if (argc >= 4 && ui_streq(argv[1], "display") && ui_streq(argv[2], "dlist") &&
            ui_streq(argv[3], "stage")) {
            bool ok = vc_display_dlist_stage();
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display dlist stage OK status=" :
                                  "vc_display dlist stage FAILED status=");
            ui_console_write(d ? vc_display_dlist_status_name(d->dlist_status) : "none");
            ui_console_write(" index=");
            ui_console_u32_dec(d ? d->dlist_stage_index : 0U);
            ui_console_write(" count=");
            ui_console_u32_dec(d ? d->dlist_stage_count : 0U);
            ui_console_write(" readback=");
            ui_console_write(d && d->dlist_stage_readback_ok ? "ok" : "bad");
            ui_console_write("\n  readback=");
            if (d) {
                for (u32 i = 0; i < d->dlist_stage_count && i < 16U; i++) {
                    if (i)
                        ui_console_write(",");
                    ui_console_hex_fixed(d->dlist_readback[i], 8);
                }
            }
            ui_console_write("\n");
        } else if (argc >= 4 && ui_streq(argv[1], "display") && ui_streq(argv[2], "dlist") &&
            (ui_streq(argv[3], "dryrun") || ui_streq(argv[3], "dry-run"))) {
            bool ok = vc_display_dlist_dryrun();
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display dlist dryrun OK status=" :
                                  "vc_display dlist dryrun FAILED status=");
            ui_console_write(d ? vc_display_dlist_status_name(d->dlist_status) : "none");
            ui_console_write(" count=");
            ui_console_u32_dec(d ? d->dlist_count : 0U);
            ui_console_write(" format=");
            ui_console_u32_dec(d ? d->dlist_format : 0U);
            ui_console_write(" order=");
            ui_console_u32_dec(d ? d->dlist_order : 0U);
            ui_console_write("\n  words=");
            if (d) {
                for (u32 i = 0; i < d->dlist_count && i < 16U; i++) {
                    if (i)
                        ui_console_write(",");
                    ui_console_hex_fixed(d->dlist_words[i], 8);
                }
            }
            ui_console_write("\n");
        } else if (argc >= 3 && ui_streq(argv[1], "display") && ui_streq(argv[2], "snapshot")) {
            bool ok = vc_display_snapshot();
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display snapshot OK count=" : "vc_display snapshot FAILED count=");
            ui_console_u32_dec(d ? d->snapshot_count : 0U);
            ui_console_write(" hvs_ver=");
            ui_console_hex_fixed(d ? d->snapshot_hvs_version : 0U, 8);
            ui_console_write(" hvs_id=");
            ui_console_hex_fixed(d ? d->snapshot_hvs_id : 0U, 8);
            ui_console_write(" control=");
            ui_console_hex_fixed(d ? d->hvs_control : 0U, 8);
            ui_console_write(" fetcher=");
            ui_console_hex_fixed(d ? d->hvs_fetcher_status : 0U, 8);
            ui_console_write(" fetch=");
            ui_console_hex_fixed(d ? d->hvs_fetch_status : 0U, 8);
            ui_console_write(" err=");
            ui_console_hex_fixed(d ? d->hvs_handle_error : 0U, 8);
            ui_console_write(" dlstat=");
            ui_console_hex_fixed(d ? d->hvs_dl_status : 0U, 8);
            ui_console_write("\n");
            for (u32 ch = 0; ch < 3U; ch++) {
                ui_console_write("  ch");
                ui_console_u32_dec(ch);
                ui_console_write(" ctrl=");
                ui_console_hex_fixed(d ? d->hvs_disp_ctrl0[ch] : 0U, 8);
                ui_console_write("/");
                ui_console_hex_fixed(d ? d->hvs_disp_ctrl1[ch] : 0U, 8);
                ui_console_write(" status=");
                ui_console_hex_fixed(d ? d->hvs_disp_status[ch] : 0U, 8);
                ui_console_write(" dl=");
                ui_console_hex_fixed(d ? d->hvs_disp_dl[ch] : 0U, 8);
                ui_console_write(" lptrs=");
                ui_console_hex_fixed(d ? d->hvs_disp_lptrs[ch] : 0U, 8);
                ui_console_write(" cob=");
                ui_console_hex_fixed(d ? d->hvs_disp_cob[ch] : 0U, 8);
                ui_console_write(" run=");
                ui_console_hex_fixed(d ? d->hvs_disp_run[ch] : 0U, 8);
                ui_console_write("\n");
            }
        } else if (argc >= 3 && ui_streq(argv[1], "display") && ui_streq(argv[2], "takeover")) {
            bool dry = argc >= 4 && (ui_streq(argv[3], "dryrun") || ui_streq(argv[3], "dry-run"));
            bool ok = vc_display_takeover_current(dry);
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display takeover OK status=" : "vc_display takeover FAILED status=");
            ui_console_write(d ? vc_display_takeover_status_name(d->last_takeover_status) : "none");
            ui_console_write(" backend=");
            ui_console_write(d ? vc_display_backend_name(d->backend) : "none");
            ui_console_write(" native_owner=");
            ui_console_write(d && d->native_owner ? "yes" : "no");
            ui_console_write(" restore=");
            ui_console_hex_fixed(d ? d->dlist_restore_lptrs : 0U, 8);
            ui_console_write("/");
            ui_console_write(d && d->dlist_restore_readback_ok ? "ok" : "none");
            ui_console_write("\n");
        } else if (argc >= 3 && ui_streq(argv[1], "display") && ui_streq(argv[2], "fallback")) {
            bool ok = vc_display_fallback();
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write(ok ? "vc_display fallback OK backend=" : "vc_display fallback FAILED backend=");
            ui_console_write(d ? vc_display_backend_name(d->backend) : "none");
            ui_console_write(" native_owner=");
            ui_console_write(d && d->native_owner ? "yes" : "no");
            ui_console_write("\n");
        } else if (argc >= 2 && ui_streq(argv[1], "display")) {
            const struct vc_display_status *d = vc_display_status_get();
            ui_console_write("vc_display ready=");
            ui_console_write(d && d->ready ? "yes" : "no");
            ui_console_write(" backend=");
            ui_console_write(d ? vc_display_backend_name(d->backend) : "none");
            ui_console_write(" native_probe=");
            ui_console_write(d && d->native_probe_ready ? "yes" : "no");
            ui_console_write(" native_owner=");
            ui_console_write(d && d->native_owner ? "yes" : "no");
            ui_console_write(" hvs=");
            ui_console_write(d && d->hvs_seen ? "yes" : "no");
            ui_console_write(" v3d=");
            ui_console_write(d && d->v3d_seen ? "yes" : "no");
            ui_console_write(" mode=");
            ui_console_u32_dec(d ? d->width : 0U);
            ui_console_write("x");
            ui_console_u32_dec(d ? d->height : 0U);
            ui_console_write(" pitch=");
            ui_console_u32_dec(d ? d->pitch : 0U);
            ui_console_write(" scanout=");
            ui_console_hex_fixed((u32)(d ? d->scanout_base : 0U), 8);
            ui_console_write(" presents=");
            ui_console_u32_dec(d ? d->present_count : 0U);
            ui_console_write(" takeover=");
            ui_console_write(d ? vc_display_takeover_status_name(d->last_takeover_status) : "none");
            ui_console_write(" attempts=");
            ui_console_u32_dec(d ? d->takeover_attempts : 0U);
            ui_console_write(" fallbacks=");
            ui_console_u32_dec(d ? d->fallback_count : 0U);
            ui_console_write(" snapshots=");
            ui_console_u32_dec(d ? d->snapshot_count : 0U);
            ui_console_write(" dlist=");
            ui_console_write(d ? vc_display_dlist_status_name(d->dlist_status) : "none");
            ui_console_write("/");
            ui_console_u32_dec(d ? d->dlist_count : 0U);
            ui_console_write(" staged=");
            ui_console_u32_dec(d ? d->dlist_stage_index : 0U);
            ui_console_write("/");
            ui_console_u32_dec(d ? d->dlist_stage_count : 0U);
            ui_console_write(" arm=");
            ui_console_hex_fixed(d ? d->dlist_arm_lptrs_after : 0U, 8);
            ui_console_write(" restore=");
            ui_console_hex_fixed(d ? d->dlist_restore_lptrs : 0U, 8);
            ui_console_write("/");
            ui_console_u32_dec(d ? d->dlist_restore_count : 0U);
            ui_console_write(" chan=");
            ui_console_write(d ? vc_display_channel_status_name(d->channel_reapply_status) : "none");
            ui_console_write("/");
            ui_console_u32_dec(d ? d->channel_reapply_count : 0U);
            ui_console_write(" global=");
            ui_console_write(d ? vc_display_global_status_name(d->global_reapply_status) : "none");
            ui_console_write("/");
            ui_console_u32_dec(d ? d->global_reapply_count : 0U);
            ui_console_write("\n");
        } else {
            ui_console_write("ERR: usage vc display | vc display snapshot | vc display global|channel reapply [dryrun] | vc display dlist dryrun|stage|arm [dryrun] | vc display takeover [dryrun] | vc display fallback\n");
        }
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
    } else if (ui_streq(argv[0], "uartflash")) {
        ui_cmd_uartflash(argc, argv);
    } else if (ui_streq(argv[0], "watchdog")) {
        ui_cmd_watchdog(argc, argv);
    } else if (ui_streq(argv[0], "bootctrl")) {
        ui_cmd_bootctrl(argc, argv);
    } else if (ui_streq(argv[0], "crypto")) {
        ui_cmd_crypto(argc, argv);
    } else if (ui_streq(argv[0], "arp")) {
        ui_cmd_arp(argc, argv);
    } else if (ui_streq(argv[0], "nic")) {
        ui_cmd_nic(argc, argv);
    } else if (ui_streq(argv[0], "cachestats")) {
        ui_cmd_cachestats(argc, argv);
    } else if (ui_streq(argv[0], "dtrace")) {
        /* dtrace [on|off|clear|mask <hex>|dump [n]]  — in-memory trace control */
        if (argc >= 2 && ui_streq(argv[1], "on")) {
            dtrace_set_enabled(true);
        } else if (argc >= 2 && ui_streq(argv[1], "off")) {
            dtrace_set_enabled(false);
        } else if (argc >= 2 && ui_streq(argv[1], "clear")) {
            dtrace_clear();
            ui_console_write("dtrace cleared\n");
        } else if (argc >= 3 && ui_streq(argv[1], "mask")) {
            u32 m = 0;
            const char *h = argv[2];
            if (h[0] == '0' && (h[1] == 'x' || h[1] == 'X')) h += 2;
            while ((*h >= '0' && *h <= '9') || ((*h | 0x20) >= 'a' && (*h | 0x20) <= 'f')) {
                u32 d = (*h <= '9') ? (u32)(*h - '0') : ((*h | 0x20) - 'a' + 10U);
                m = (m << 4) | d; h++;
            }
            dtrace_set_mask(m);
        } else if (argc >= 2 && ui_streq(argv[1], "dump")) {
            /* Render into a shared buffer and fan out via ui_console_write()
             * (UART + HDMI console + TCP/2323) instead of writing straight to
             * the physical UART: the old dtrace_dump_uart() path made "dtrace
             * dump" invisible to anyone using the TCP debug console or the
             * HDMI terminal, which defeats the point of a unified console. */
            static char dump_buf[8192];
            u32 n = 256;
            if (argc >= 3) (void)ui_parse_u32(argv[2], &n);
            u32 dl = dtrace_dump(dump_buf, sizeof(dump_buf) - 1U, n);
            dump_buf[dl] = 0;
            ui_console_write(dump_buf);
        }
        {
            char sb[160];
            u32 sl = dtrace_status(sb, sizeof(sb) - 1U);
            sb[sl] = 0;
            ui_console_write(sb);
        }
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
        ui_console_exec_shared_fallback(argc, argv);
    }
}

static i32 ui_console_last_term_char = -1;  /* value of the last \r or \n
                                          processed as a line terminator (-1
                                          if none pending), so the OPPOSITE
                                          half of a \r\n (or \n\r) pair
                                          arriving as the very next byte can
                                          be swallowed instead of being
                                          treated as a second, empty line --
                                          while a REPEATED same terminator
                                          (e.g. two bare \r from pressing
                                          Enter twice) still executes twice. */

static void ui_console_feed_char(i32 c)
{
    if (c < 0) return;
    if (c == '\r' || c == '\n') {
        /* A CRLF (or LFCR) pair is ONE line terminator, not two. Without
         * this, a client that sends "\r\n" (the common convention) causes
         * the first byte to execute the command -- which can synchronously
         * flip the console into a raw byte-consuming mode (e.g. uartflash's
         * PATCH_REGION/FULL transfer) -- and the second byte then gets
         * misrouted as the FIRST byte of that raw stream instead of being
         * recognised as terminator noise, corrupting the transfer by
         * exactly one byte. Swallow the paired opposite half here instead;
         * a repeated identical terminator (e.g. two bare \r) still executes
         * as two separate (empty) lines. */
        if (ui_console_last_term_char >= 0 && ui_console_last_term_char != c) {
            ui_console_last_term_char = -1;
            return;
        }
        ui_console_last_term_char = c;
        ui_console_write("\n");
        ui_console_line[ui_console_len] = 0;
        ui_console_exec(ui_console_line);
        ui_console_len = 0;
        ui_console_prompt();
        return;
    }
    ui_console_last_term_char = -1;
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
            ui_console_hdmi_reset();
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
            ui_console_hdmi_reset();
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
#define CORE0_IO_CPUCLK  (1U << 6)

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

static inline u64 dash_now_ticks(void) { u64 c; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(c)); return c; }

static void dash_blank_line(u32 col, u32 row, u32 width)
{
    if (width == 0)
        return;
    fb_set_cursor(col, row);
    for (u32 i = 0; i < width; i++)
        fb_putc(' ');
}

static void dash_put_trunc(const char *s, u32 max_chars)
{
    u32 i = 0;
    while (s && s[i] && i < max_chars) {
        fb_putc(s[i]);
        i++;
    }
}

static bool dash_streq(const char *a, const char *b)
{
    if (!a || !b)
        return false;
    u32 i = 0;
    while (a[i] && b[i]) {
        if (a[i] != b[i])
            return false;
        i++;
    }
    return a[i] == b[i];
}

static void dash_clear_body(u32 col, u32 row, u32 width, u32 height)
{
    if (width < 3 || height < 3)
        return;
    for (u32 r = row + 1; r + 1 < row + height; r++)
        dash_blank_line(col + 1, r, width - 2);
}

static void dash_draw_window(u32 col, u32 row, u32 width, u32 height,
                             const char *title, u32 color)
{
    u32 max_cols = fb_cols();
    u32 max_rows = fb_rows();
    if (col >= max_cols || row >= max_rows)
        return;
    if (col + width > max_cols)
        width = max_cols - col;
    if (row + height > max_rows)
        height = max_rows - row;
    if (width < 4 || height < 2)
        return;

    fb_set_color(color, 0x00000000);
    fb_box_at(col, row, width, height, title);
}

static void dash_put_permille_pct(u32 permille)
{
    u32 whole = permille / 10U;
    u32 frac = permille % 10U;
    if (frac)
        fb_printf("%u.%u%%", whole, frac);
    else
        fb_printf("%u%%", whole);
}

/* Diagnostics-panel helper: "label=y" in on_color when set, dim grey when clear.
 * Used for decoded status bits where the caller decides which colour "set"
 * means (red for fault bits, green for healthy bits like link-up). */
static void dash_flag(const char *label, bool on, u32 on_color)
{
    fb_set_color(0x00AAAAAA, 0x00000000);
    fb_puts(label);
    fb_set_color(on ? on_color : 0x00606060, 0x00000000);
    fb_puts(on ? "y" : "n");
}

/* Diagnostics-panel helper: grey "label=" then a white unsigned value. */
static void dash_kv_u32(const char *label, u32 val)
{
    fb_set_color(0x00AAAAAA, 0x00000000);
    fb_puts(label);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("%u", val);
}

static const struct proc_ui_entry *
dash_proc_for_pid(const struct proc_ui_entry *proc, u32 proc_n, u32 pid)
{
    for (u32 i = 0; i < proc_n; i++) {
        if (proc[i].pid != pid)
            continue;
        return &proc[i];
    }
    return (const struct proc_ui_entry *)0;
}

static const struct proc_capsule_ui_entry *
dash_capsule_for_pid(const struct proc_capsule_ui_entry *caps, u32 caps_n, u32 pid)
{
    for (u32 i = 0; i < caps_n; i++) {
        if (caps[i].pid == pid)
            return &caps[i];
    }
    return (const struct proc_capsule_ui_entry *)0;
}

static bool dash_capsule_same_group(const struct proc_capsule_ui_entry *a,
                                    const struct proc_capsule_ui_entry *b)
{
    if (!a || !b)
        return false;
    return a->capsule_id == b->capsule_id &&
           a->capsule_hash == b->capsule_hash &&
           dash_streq(a->group, b->group);
}

static bool dash_bridge_for_port(u16 port, u32 *bridge_out, u32 *pid_out,
                                 u32 *core_out)
{
    u32 idx = 0xFFFFFFFFU;
    if (port == UHTTP_PORT)
        idx = 0U;
#if UHTTP_BRIDGE_COUNT > 1
    else if (port == UHTTP_NATIVE_PORT)
        idx = 1U;
#endif
    if (idx >= UHTTP_BRIDGE_COUNT)
        return false;
    i32 listen = 0;
    u32 state = 0, req = 0, resp = 0, total = 0, magic = 0, pid = 0;
    uhttp_bridge_state_idx(idx, &listen, &state, &req, &resp, &total, &magic, &pid);
    (void)listen; (void)state; (void)req; (void)resp; (void)total; (void)magic;
    if (bridge_out) *bridge_out = idx;
    if (pid_out) *pid_out = pid;
    if (core_out) *core_out = uhttp_bridge_target_core(idx);
    return true;
}

struct dash_listener_info {
    u16 port;
    u32 bridge;
    u32 pid;
    u32 core;
    u32 principal_id;
    u32 cpu_percent;
    u32 mem_kib;
    bool has_bridge;
    bool has_proc;
    const char *image;
    const char *owner;
    const char *principal;
    const struct proc_capsule_ui_entry *capsule;
};

static const char *dash_principal_name(const struct principal_ui_entry *users,
                                       u32 users_n, u32 principal_id)
{
    for (u32 i = 0; i < users_n; i++) {
        if (users[i].id == principal_id)
            return users[i].name;
    }
    return principal_id == PRINCIPAL_ROOT ? "root" : "?";
}

static u32 dash_listener_group(const struct dash_listener_info *info)
{
    if (info->has_proc)
        return 2U; /* user */
    if (info->port == ADMIN_STATUS_TCP_PORT ||
        info->port == ADMIN_REBOOT_TCP_PORT ||
        info->port == ADMIN_UPDATE_TCP_PORT)
        return 1U; /* admin */
    return 0U;     /* kernel */
}

static const char *dash_group_name(u32 group)
{
    if (group == 0U) return "KERNEL";
    if (group == 1U) return "ADMIN";
    return "USER";
}

static struct dash_listener_info dash_listener_info_for(
    const tcp_snapshot_entry_t *tcp,
    const struct proc_ui_entry *proc, u32 proc_n,
    const struct proc_capsule_ui_entry *caps, u32 caps_n,
    const struct principal_ui_entry *users, u32 users_n)
{
    struct dash_listener_info info = {0};
    info.port = tcp->local_port;
    info.owner = tcp_owner_label(tcp->local_port);
    info.image = info.owner;
    info.principal_id = PRINCIPAL_ROOT;
    info.principal = "root";
    info.has_bridge = dash_bridge_for_port(tcp->local_port, &info.bridge, &info.pid,
                                           &info.core);
    if (info.pid) {
        const struct proc_ui_entry *p = dash_proc_for_pid(proc, proc_n, info.pid);
        if (p) {
            info.has_proc = true;
            info.core = p->affinity_core;
            info.principal_id = p->principal_id;
            info.principal = dash_principal_name(users, users_n, p->principal_id);
            info.cpu_percent = p->cpu_percent;
            info.mem_kib = p->mem_kib;
            info.image = p->image_path;
        }
        info.capsule = dash_capsule_for_pid(caps, caps_n, info.pid);
    }
    return info;
}

static bool dash_info_same_capsule(const struct dash_listener_info *a,
                                   const struct dash_listener_info *b)
{
    if (!a || !b)
        return false;
    if (dash_listener_group(a) != dash_listener_group(b))
        return false;
    if (a->has_proc != b->has_proc)
        return false;
    if (!a->has_proc)
        return true;
    if (!a->capsule || !b->capsule)
        return a->capsule == b->capsule;
    return dash_capsule_same_group(a->capsule, b->capsule);
}

static void dash_draw_group_header(u32 row, u32 col, u32 group)
{
    fb_set_cursor(col, row);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts(dash_group_name(group));
}

static void dash_draw_capsule_header(u32 row, u32 col,
                                     const struct proc_capsule_ui_entry *cap,
                                     bool kernel_group)
{
    fb_set_cursor(col, row);
    fb_set_color(0x00FFAA00, 0x00000000);
    fb_puts("CAPSULE ");
    if (kernel_group) {
        fb_puts("kernel/platform");
        return;
    }
    if (!cap) {
        fb_puts("unknown");
        return;
    }
    if (cap->group[0])
        dash_put_trunc(cap->group, 28U);
    else if (cap->capsule_id == PROC_CAPSULE_ID_NONE)
        fb_puts("none");
    else {
        fb_puts("id=");
        fb_printf("%u", cap->capsule_id);
    }
    fb_puts(" hash=0x");
    fb_printf("%x", cap->capsule_hash);
}

static void dash_put_mib(u32 kib)
{
    u32 mib = kib >> 10;
    u32 frac = ((kib & 1023U) * 10U) >> 10;
    if (frac)
        fb_printf("%u.%uM", mib, frac);
    else
        fb_printf("%uM", mib);
}

static void dash_put_bytes_mb(u64 bytes)
{
    fb_printf("%uMB", (u32)(bytes >> 20));
}

static void dash_put_mbps(u32 mbps_x1000)
{
    fb_printf("%u", mbps_x1000 / 1000U);
    u32 frac = mbps_x1000 % 1000U;
    if (frac) {
        fb_putc('.');
        fb_putc((char)('0' + (frac / 100U) % 10U));
        fb_putc((char)('0' + (frac / 10U) % 10U));
        fb_putc((char)('0' + frac % 10U));
    }
}

static void dash_put_uptime_breakdown(u64 sec)
{
    static const u32 min = 60U, hour = 60U * 60U, day = 24U * 60U * 60U;
    static const u32 week = 7U * 24U * 60U * 60U;
    static const u32 month = 30U * 24U * 60U * 60U;
    static const u32 year = 365U * 24U * 60U * 60U;
    u32 y = (u32)(sec / year); sec %= year;
    u32 mo = (u32)(sec / month); sec %= month;
    u32 w = (u32)(sec / week); sec %= week;
    u32 d = (u32)(sec / day); sec %= day;
    u32 h = (u32)(sec / hour); sec %= hour;
    u32 m = (u32)(sec / min);
    fb_printf(" (%uY %uM %uW %uD %uH %uM)", y, mo, w, d, h, m);
}

static u32 dash_prefix_len_from_mask(u32 mask)
{
    u32 n = 0;
    for (i32 bit = 31; bit >= 0; bit--) {
        if (mask & (1U << (u32)bit))
            n++;
        else
            break;
    }
    return n;
}

static void dash_put_active(bool active)
{
    fb_set_color(active ? 0x0000FF80 : 0x00FFAA00, 0x00000000);
    fb_puts(active ? "ACTIVE" : "OFF");
    fb_set_color(0x00FFFFFF, 0x00000000);
}

static void dash_hw_row(u32 row, u32 c_dev, u32 c_active, u32 c_load,
                        u32 c_ram, u32 c_caps, const char *dev,
                        bool active, const char *load, const char *ram,
                        const char *caps)
{
    fb_set_cursor(c_dev, row);
    fb_set_color(0x0000CCFF, 0x00000000);
    dash_put_trunc(dev, c_active > c_dev ? c_active - c_dev - 1U : 10U);
    fb_set_cursor(c_active, row);
    dash_put_active(active);
    fb_set_cursor(c_load, row);
    fb_set_color(0x00FFFFFF, 0x00000000);
    dash_put_trunc(load, c_ram > c_load ? c_ram - c_load - 1U : 20U);
    fb_set_cursor(c_ram, row);
    dash_put_trunc(ram, c_caps > c_ram ? c_caps - c_ram - 1U : 12U);
    fb_set_cursor(c_caps, row);
    dash_put_trunc(caps, 44U);
}

static void dash_hw_row_u64_hex(u32 row, u32 c_dev, u32 c_active, u32 c_load,
                                u32 c_ram, u32 c_caps, const char *dev,
                                bool active, const char *prefix, u64 addr,
                                const char *ram, const char *caps)
{
    char load[32];
    u32 n = 0;
    while (prefix && *prefix && n + 1U < sizeof(load))
        load[n++] = *prefix++;
    if (n + 3U < sizeof(load)) {
        static const char hex[] = "0123456789ABCDEF";
        load[n++] = '0';
        load[n++] = 'x';
        bool seen = false;
        for (i32 sh = 60; sh >= 0 && n + 1U < sizeof(load); sh -= 4) {
            u8 v = (u8)((addr >> (u32)sh) & 0xFULL);
            if (v || seen || sh == 0) {
                load[n++] = hex[v];
                seen = true;
            }
        }
    }
    load[n] = 0;
    dash_hw_row(row, c_dev, c_active, c_load, c_ram, c_caps, dev, active, load, ram, caps);
}

static void dash_draw_listener_row(u32 row, u32 c_pid, u32 c_core, u32 c_user,
                                   u32 c_cpu, u32 c_mem, u32 c_proc,
                                   u32 c_fifo, u32 proc_width, u32 fifo_width,
                                   const struct dash_listener_info *info)
{
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_set_cursor(c_pid, row);
    if (info->has_proc)
        fb_printf("%u", info->pid);
    else
        fb_puts("kern");
    fb_set_cursor(c_core, row);
    fb_printf("%u", (info->has_proc || info->has_bridge) ? info->core : 0U);
    fb_set_cursor(c_user, row);
    dash_put_trunc(info->principal ? info->principal : "?", 10U);
    fb_set_cursor(c_cpu, row);
    if (info->has_proc)
        fb_printf("%u%%", info->cpu_percent);
    else
        fb_puts("-");
    fb_set_cursor(c_mem, row);
    if (info->has_proc)
        dash_put_mib(info->mem_kib);
    else
        fb_puts("-");
    fb_set_cursor(c_proc, row);
    dash_put_trunc(info->image ? info->image : "-", proc_width);
    fb_set_cursor(c_fifo, row);
    if (info->has_bridge) {
        fb_puts("uhttp bridge");
        fb_printf("%u", info->bridge);
        fb_puts(" fifo -> tcp/");
        fb_printf("%u", info->port);
    } else {
        dash_put_trunc(info->owner, fifo_width);
        fb_puts(" -> tcp/");
        fb_printf("%u", info->port);
    }
}

static void hdmi_dashboard_render(void)
{
    static u64 last_ms;
    static bool layout_drawn;
    u64 now_ms = timer_monotonic_ms();
    /* Self-throttle floor (900ms) sits just under the 1Hz core0_io_tick_hook
     * DASH cadence so sub-ms beat between tick-count and monotonic-ms never
     * skips a second, while still preventing pathological back-to-back renders. */
    if (last_ms != 0 && now_ms < last_ms + 900ULL)
        return;
    last_ms = now_ms;
    u64 t_dash0 = dash_now_ticks();

    struct perf_counter_snapshot perf;
    perf_counter_snapshot(&perf);

    tcp_snapshot_entry_t tcp[TCP_MAX_CONNECTIONS];
    u32 tcp_n = tcp_snapshot(tcp, TCP_MAX_CONNECTIONS);
    struct proc_ui_entry proc[UI_SNAPSHOT_MAX];
    u32 proc_n = proc_snapshot(proc, UI_SNAPSHOT_MAX);
    struct proc_capsule_ui_entry caps[UI_SNAPSHOT_MAX];
    u32 caps_n = proc_capsule_snapshot(caps, UI_SNAPSHOT_MAX);
    struct principal_ui_entry users[PRINCIPAL_MAX];
    u32 users_n = principal_snapshot(users, PRINCIPAL_MAX);
    u32 screen_cols = fb_cols();
    u32 screen_rows = fb_rows();
    bool wide = screen_cols >= 180U;
    u32 header_col = 0, header_row = 0, header_w = wide ? (screen_cols - 2U) : 78U, header_h = 5U;
    u32 hw_col = 0U;
    u32 hw_row = header_row + header_h + 1U;
    u32 hw_w = wide ? header_w : 78U;
    u32 hw_h = 15U;
    u32 tns_col = 0U;
    u32 tns_row = hw_row + hw_h + 1U;
    u32 tns_w = wide ? header_w : 78U;
    u32 tns_h = 5U;     /* border + 3 content rows: column header, NEON, V3D/QPU */
    u32 map_col = 0U;
    u32 map_row = tns_row + tns_h + 1U;
    u32 map_w = wide ? header_w : 78U;
    u32 log_col = 0;
    u32 log_h = wide ? 11U : 12U;
    u32 log_top = screen_rows > log_h + map_row + 5U ? screen_rows - log_h - 1U :
                  (wide ? 28U : (screen_rows > 10U ? screen_rows - 9U : 28U));
    u32 log_w = header_w;
    if (log_top + log_h >= screen_rows && screen_rows > log_h + 1U)
        log_top = screen_rows - log_h - 1U;
    u32 map_h = log_top > map_row + 3U ? log_top - map_row - 1U : (wide ? 16U : 13U);

    /* Right-hand diagnostics column. On wide screens the HARDWARE / TENSOR /
     * MAP boxes leave a large unused gutter on the right. Carve a fixed-width
     * column there for live NIC/MAC, DMA, and FIFO/lease-arena diagnostics —
     * the one channel that stays readable when the network path wedges. */
    u32 diag_col = 0U, diag_w = 0U;
    if (wide && header_w >= 170U) {
        diag_w = 52U;
        u32 left_w = header_w - diag_w - 1U;
        if (left_w < 116U) {            /* keep HW capability strings intact */
            left_w = 116U;
            diag_w = header_w - left_w - 1U;
        }
        hw_w = left_w;
        tns_w = left_w;
        map_w = left_w;
        diag_col = left_w + 1U;
    }

    u64 t_dash1 = dash_now_ticks();
    g_dash_snap_ticks = t_dash1 - t_dash0;
    if (!layout_drawn) {
        fb_clear(0x00000000);
        dash_draw_window(header_col, header_row, header_w, header_h, "PIOS WORKBENCH", 0x0000FF80);
        dash_draw_window(hw_col, hw_row, hw_w, hw_h, "HARDWARE / CAPABILITIES", 0x0000CCFF);
        dash_draw_window(tns_col, tns_row, tns_w, tns_h, "TENSOR / AI ACCELERATION", 0x00FF80FF);
        dash_draw_window(map_col, map_row, map_w, map_h, "NETWORK / PROCESS MAP", 0x00FFAA00);
        dash_draw_window(log_col, log_top, log_w, log_h, "WARNINGS / ERRORS", 0x00FF4040);
        if (diag_w) {
            dash_draw_window(diag_col, hw_row, diag_w, hw_h, "NIC / MAC RX-TX", 0x0000FFCC);
            dash_draw_window(diag_col, tns_row, diag_w, tns_h, "DMA ENGINE", 0x00FFCC66);
            dash_draw_window(diag_col, map_row, diag_w, map_h, "FIFO / LEASE ARENAS", 0x0066FF66);
        }
        layout_drawn = true;
    }

    dash_clear_body(header_col, header_row, header_w, header_h);
    u32 h0 = header_col + 3U;
    u32 h1 = header_col + (header_w / 3U) + 2U;
    u32 h2 = header_col + ((header_w * 2U) / 3U) + 1U;
    u32 ip = net_get_our_ip();
    u32 mask = net_get_netmask();

    fb_set_cursor(h0, header_row + 1);
    fb_set_color(0x00FF80FF, 0x00000000);
    fb_puts("VERSION: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts(PIOS_VERSION);
    fb_set_cursor(h1, header_row + 1);
    fb_set_color(0x0000FF80, 0x00000000);
    fb_puts("UPTIME: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    u64 uptime_s = now_ms / 1000ULL;
    fb_printf("%u", (u32)uptime_s);
    fb_puts("s");
    dash_put_uptime_breakdown(uptime_s);
    fb_set_cursor(h2, header_row + 1);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("IP: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    dash_ip(ip);
    fb_puts("/");
    fb_printf("%u", dash_prefix_len_from_mask(mask));

    fb_set_cursor(h0, header_row + 2);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("CPU: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("Total=");
    dash_put_permille_pct(perf.cpu_total_permille);
    fb_puts(" 0=");
    dash_put_permille_pct(perf.cpu_permille[0]);
    fb_puts(" 1=");
    dash_put_permille_pct(perf.cpu_permille[1]);
    fb_puts(" 2=");
    dash_put_permille_pct(perf.cpu_permille[2]);
    fb_puts(" 3=");
    dash_put_permille_pct(perf.cpu_permille[3]);
    fb_puts(" clk=");
    if (perf.cpu_clock_mhz)
        fb_printf("%uMHz", perf.cpu_clock_mhz);
    else
        fb_puts("?");
    fb_set_cursor(h1, header_row + 2);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("RAM: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("Inst=");
    dash_put_bytes_mb(perf.installed_ram_bytes);
    fb_puts(" ARM=");
    dash_put_bytes_mb(perf.physical_ram_bytes);
    fb_puts(" Pool=");
    fb_printf("%uMB", perf.ram_total_kib >> 10);
    fb_puts(" High=");
    if (perf.highmem.ready)
        dash_put_bytes_mb(perf.highmem.total_bytes);
    else
        fb_puts("off");
    fb_puts(" Used=");
    fb_printf("%uMB", perf.ram_used_kib >> 10);
    fb_puts(" K=");
    fb_printf("%uMB", perf.ram_kernel_kib >> 10);
    fb_puts(" U=");
    fb_printf("%uMB", perf.ram_user_kib >> 10);
    fb_set_cursor(h2, header_row + 2);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("BOARD: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("rev=0x");
    fb_printf("%x", perf.board_revision);
    fb_puts(" model=0x");
    fb_printf("%x", perf.board_model_code);
    fb_puts(" pcb=");
    fb_printf("%u", perf.board_pcb_revision);
    fb_puts(" ram=");
    dash_put_bytes_mb(perf.installed_ram_bytes);

    fb_set_cursor(h0, header_row + 3);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("NET: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("Rx=");
    dash_put_mbps(perf.nic_rx_mbps_x1000);
    fb_puts("/");
    dash_put_mbps(perf.nic_rx_capacity_mbps * 1000U);
    fb_puts(" Tx=");
    dash_put_mbps(perf.nic_tx_mbps_x1000);
    fb_puts("/");
    dash_put_mbps(perf.nic_tx_capacity_mbps * 1000U);
    fb_puts(" Mb/s ");
    if (perf.nic_link_mbps) {
        fb_printf("%u", perf.nic_link_mbps);
        fb_puts(perf.nic_link_full_duplex ? "FD" : "HD");
    } else {
        fb_puts("down");
    }
    fb_set_cursor(h1, header_row + 3);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("WALFS: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts(perf.walfs_mounted ? "mount " : "down ");
    fb_puts("reg=");
    dash_put_bytes_mb(perf.walfs_region_bytes);
    fb_puts(" used=");
    dash_put_bytes_mb(perf.walfs_used_bytes);
    fb_puts(" rec=");
    fb_printf("%u", perf.walfs_records);
    fb_set_cursor(h2, header_row + 3);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_puts("SD: ");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("R=");
    dash_put_mbps(perf.sd_read_mbps_x1000);
    fb_puts("/");
    dash_put_mbps(perf.sd_read_peak_mbps_x1000);
    fb_puts(" W=");
    dash_put_mbps(perf.sd_write_mbps_x1000);
    fb_puts("/");
    dash_put_mbps(perf.sd_write_peak_mbps_x1000);
    fb_puts(" Mb/s");

    dash_clear_body(hw_col, hw_row, hw_w, hw_h);
    u32 hw_r = hw_row + 1U;
    const u32 hw_end = hw_row + hw_h - 1U;
    const u32 hw_dev = hw_col + 3U;
    const u32 hw_active = wide ? hw_col + 18U : hw_col + 15U;
    const u32 hw_load = wide ? hw_col + 29U : hw_col + 24U;
    const u32 hw_ram = wide ? hw_col + 58U : hw_col + 45U;
    const u32 hw_caps = wide ? hw_col + 74U : hw_col + 56U;
    fb_set_cursor(hw_dev, hw_r);
    fb_set_color(0x00AAAAAA, 0x00000000);
    fb_puts("HARDWARE");
    fb_set_cursor(hw_active, hw_r);
    fb_puts("STATE");
    fb_set_cursor(hw_load, hw_r);
    fb_puts("LOAD / MMIO");
    fb_set_cursor(hw_ram, hw_r);
    fb_puts("RAM");
    fb_set_cursor(hw_caps, hw_r);
    fb_puts("CAPABILITY / DRIVER");
    hw_r++;
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "CPU cores", true, "EL1 ", PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT ? 0x40080000ULL : 0x00080000ULL,
                            "4x16M", "AArch64 NEON CRC AES/SHA timers");
    const struct videocore_probe *vc = videocore_probe_get();
    const struct vc_display_status *vcd = vc_display_status_get();
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "VideoCore", PIOS_HAS_MAILBOX_FB || (vc && vc->enabled && (vc->hvs_seen || vc->v3d_seen)),
                            "HVS ", 0x107C580000ULL,
                            vcd && vcd->ready ? vc_display_backend_name(vcd->backend) : "0",
                            vc && vc->v3d_seen ? "VC display drv + V3D/QPU/MMU probe" :
                            (PIOS_HAS_MAILBOX_FB ? "mailbox firmware services" : "not present"));
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "Storage", sd_get_card_info() && sd_get_card_info()->capacity != 0,
                            PIOS_HAS_SD ? "EMMC2 " : "blk ", PIOS_HAS_SD ? PIOS_EMMC2_BASE : 0ULL,
                            "bcache", PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT ?
                            (sd_qemu_virtio_blk_ready() ? "virtio-blk + WALFS" : "RAM block + WALFS") :
                            "SDHCI/SDIO + WALFS");
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "NIC", PIOS_HAS_GENET && perf.nic_link_mbps != 0,
                            "MMIO ", PIOS_GENET_BASE, "rings", PIOS_HAS_GENET ? "GENET/MACB Ethernet" : "no active NIC backend");
    if (hw_r < hw_end)
        dash_hw_row(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                    "WiFi/BT", false, "not loaded", "0", "CYW/BT parked; no active driver");
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "USB/HID", PIOS_HAS_RP1, "RP1 ", PIOS_RP1_BAR_BASE,
                            "xhci", PIOS_HAS_RP1 ? "xHCI + USB HID keyboard" : "not present");
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "RP1 bridge", PIOS_HAS_RP1 && PIOS_HAS_PCIE, "BAR ", PIOS_RP1_BAR_BASE,
                            "regs", PIOS_HAS_RP1 ? "PCIe southbridge, MIP IRQs" : "not present");
    if (hw_r < hw_end)
        dash_hw_row_u64_hex(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                            "GPIO", PIOS_HAS_RP1, "RP1 ", PIOS_RP1_BAR_BASE,
                            "regs", PIOS_HAS_RP1 ? "54 GPIO / 28 header pins" : "not present");
    if (hw_r < hw_end)
        dash_hw_row(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                    "I2C / SPI", false, PIOS_HAS_RP1 ? "RP1 ctrl" : "not present",
                    "0", PIOS_HAS_RP1 ? "HW present; driver pending" : "not present");
    if (hw_r < hw_end)
        dash_hw_row(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                    "SDIO", PIOS_HAS_SD, PIOS_HAS_SD ? "EMMC2" : "virtio/mmio",
                    "dma/buf", PIOS_HAS_SD ? "SDIO 4-bit path" : "platform block path");
    if (hw_r < hw_end)
        dash_hw_row(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                    "QSPI", false, PIOS_HAS_RP1 ? "RP1/QSPI" : "not present",
                    "0", "HW planned; driver pending");
    if (hw_r < hw_end)
        dash_hw_row(hw_r++, hw_dev, hw_active, hw_load, hw_ram, hw_caps,
                    "Framebuffer", PIOS_HAS_MAILBOX_FB || PIOS_HAS_BOOTINFO_FB,
                    vcd && vcd->ready ? vc_display_backend_name(vcd->backend) :
                    (PIOS_HAS_MAILBOX_FB ? "mailbox FB" : (PIOS_HAS_BOOTINFO_FB ? "UEFI GOP" : "none")),
                    (PIOS_HAS_MAILBOX_FB || PIOS_HAS_BOOTINFO_FB) ? "scanout+back" : "0",
                    (PIOS_HAS_MAILBOX_FB || PIOS_HAS_BOOTINFO_FB) ? "workbench dashboard" : "serial console only");

    /* TENSOR / AI ACCELERATION section: which compute backends accelerate the
     * tensor primitives, their live state, and the terminal commands that drive
     * them. NEON is always present (ARMv8.2 SIMD, fp32); V3D/QPU state is read
     * live from tensor_status(). */
    dash_clear_body(tns_col, tns_row, tns_w, tns_h);
    struct tensor_status tns;
    tensor_status(&tns);
    {
        u32 tns_r = tns_row + 1U;
        const u32 t_back = tns_col + 3U;
        const u32 t_state = wide ? (tns_col + 16U) : (tns_col + 13U);
        const u32 t_detail = wide ? (tns_col + 26U) : (tns_col + 21U);
        const u32 t_detail_w = tns_w > (t_detail - tns_col) + 2U
                                   ? tns_w - (t_detail - tns_col) - 2U : 20U;
        bool v3d_ready = tns.v3d_native_compute_enabled && tns.any_kernel_bound;
        fb_set_color(0x00AAAAAA, 0x00000000);
        fb_set_cursor(t_back, tns_r);
        fb_puts("BACKEND");
        fb_set_cursor(t_state, tns_r);
        fb_puts("STATE");
        fb_set_cursor(t_detail, tns_r);
        fb_puts("ACCELERATED OPS / COMMANDS");
        tns_r++;
        fb_set_color(0x0000FF80, 0x00000000);
        fb_set_cursor(t_back, tns_r);
        fb_puts("CPU NEON");
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_set_cursor(t_state, tns_r);
        fb_puts("ready");
        fb_set_cursor(t_detail, tns_r);
        dash_put_trunc("add mul scale dot relu softmax matmul matvec (fp32 SIMD)  cmd: tensor bench",
                       t_detail_w);
        tns_r++;
        fb_set_color(0x0000CCFF, 0x00000000);
        fb_set_cursor(t_back, tns_r);
        fb_puts("V3D QPU");
        fb_set_color(v3d_ready ? 0x0000FF80 : (tns.v3d_available ? 0x00FFAA00 : 0x00FF4040),
                     0x00000000);
        fb_set_cursor(t_state, tns_r);
        fb_puts(v3d_ready ? "ready" : (tns.v3d_available ? "probe" : "off"));
        fb_set_color(0x00FFFFFF, 0x00000000);
        fb_set_cursor(t_detail, tns_r);
        dash_put_trunc("vector16/N add+mul, matvecN, matmul via QPU CSD  cmd: tensor vectorN | bench v3d",
                       t_detail_w);
    }

    dash_clear_body(map_col, map_row, map_w, map_h);
    u32 row = map_row + 1U;
    const u32 map_end = map_row + map_h - 1U;
    const u32 c_pid = map_col + 3U;
    const u32 c_core = wide ? (map_col + 10U) : (map_col + 9U);
    const u32 c_user = wide ? (map_col + 16U) : (map_col + 14U);
    const u32 c_cpu = wide ? (map_col + 30U) : (map_col + 24U);
    const u32 c_mem = wide ? (map_col + 38U) : (map_col + 31U);
    const u32 c_proc = wide ? (map_col + 48U) : (map_col + 39U);
    const u32 c_fifo = wide ? (map_col + 96U) : (map_col + 56U);
    const u32 proc_width = wide ? 44U : 15U;
    const u32 fifo_width = map_w > c_fifo - map_col + 2U ? map_w - (c_fifo - map_col) - 2U : 20U;
    fb_set_cursor(c_pid, row);
    fb_set_color(0x00AAAAAA, 0x00000000);
    fb_puts("PID");
    fb_set_cursor(c_core, row);
    fb_puts("CORE");
    fb_set_cursor(c_user, row);
    fb_puts("USER");
    fb_set_cursor(c_cpu, row);
    fb_puts("CPU");
    fb_set_cursor(c_mem, row);
    fb_puts("RAM");
    fb_set_cursor(c_proc, row);
    fb_puts("PROCESS");
    fb_set_cursor(c_fifo, row);
    fb_puts("FIFO / PORT");
    row++;
    struct dash_listener_info listeners[TCP_MAX_CONNECTIONS];
    u32 listener_n = 0;
    for (u32 i = 0; i < tcp_n && listener_n < TCP_MAX_CONNECTIONS; i++) {
        if (tcp[i].state != TCP_LISTEN)
            continue;
        listeners[listener_n++] = dash_listener_info_for(&tcp[i], proc, proc_n, caps, caps_n,
                                                         users, users_n);
    }
    u32 shown_listeners = 0;
    for (u32 group = 0; group < 3U && row < map_end; group++) {
        bool group_header_drawn = false;
        for (u32 i = 0; i < listener_n && row < map_end; i++) {
            if (dash_listener_group(&listeners[i]) != group)
                continue;
            bool capsule_seen = false;
            for (u32 j = 0; j < i; j++) {
                if (dash_listener_group(&listeners[j]) == group &&
                    dash_info_same_capsule(&listeners[j], &listeners[i])) {
                    capsule_seen = true;
                    break;
                }
            }
            if (capsule_seen)
                continue;
            if (!group_header_drawn) {
                dash_draw_group_header(row++, map_col + 2U, group);
                group_header_drawn = true;
                if (row >= map_end)
                    break;
            }
            dash_draw_capsule_header(row++, map_col + 4U,
                                     listeners[i].capsule,
                                     !listeners[i].has_proc);
            for (u32 k = 0; k < listener_n && row < map_end; k++) {
                if (dash_listener_group(&listeners[k]) != group ||
                    !dash_info_same_capsule(&listeners[k], &listeners[i]))
                    continue;
                dash_draw_listener_row(row++, c_pid, c_core, c_user, c_cpu, c_mem,
                                       c_proc, c_fifo, proc_width, fifo_width,
                                       &listeners[k]);
                shown_listeners++;
            }
        }
    }
    if (shown_listeners == 0) {
        fb_set_cursor(map_col + 3U, row);
        fb_set_color(0x00AAAAAA, 0x00000000);
        fb_puts("No listening sockets yet.");
    }

    dash_clear_body(log_col, log_top, log_w, log_h);
    row = log_top + 1;
    u32 shown = 0;
    u32 max_back = http_log_seq < HTTP_LOG_RING_SIZE ? http_log_seq : HTTP_LOG_RING_SIZE;
    for (u32 back = 0; back < max_back && shown + 2U < log_h; back++) {
        u32 seq = http_log_seq - 1U - back;
        struct http_log_entry *e = &http_log_ring[seq % HTTP_LOG_RING_SIZE];
        if (e->seq != seq || !e->event)
            continue;
        if (!dash_contains(e->event, "error") &&
            !dash_contains(e->event, "fail") &&
            !dash_contains(e->event, "warn"))
            continue;
        fb_set_cursor(log_col + 3U, row++);
        if (dash_streq(e->event, "http-error")) {
            u32 ev = (e->a >> 24) & 0xFFU;
            u32 route = (e->a >> 16) & 0xFFU;
            u32 arg = e->a & 0xFFFFU;
            fb_printf("%u t=%u http-error event=%s route=%s",
                      e->seq, e->tick_ms, http_event_name(ev), http_route_name(route));
            if (ev == HTTP_EVT_ABORT)
                fb_printf(" error=%s conn=%u", http_error_name(arg), e->b);
            else if (ev == HTTP_EVT_BAD_STATE)
                fb_printf(" state=%u idle_ms=%u", arg, e->b);
            else
                fb_printf(" arg=%u b=%u", arg, e->b);
        } else {
            fb_printf("%u t=%u %s a=%u b=%u", e->seq, e->tick_ms, e->event, e->a, e->b);
        }
        shown++;
    }
    if (shown == 0) {
        fb_set_cursor(log_col + 3U, row);
        fb_set_color(0x0000FF80, 0x00000000);
        fb_puts("No warnings or errors in the hot log ring.");
    }

    /* ===== Right-hand diagnostics column ===== */
    if (diag_w) {
        const u32 C_RED = 0x00FF4040, C_GRN = 0x0000FF80,
                  C_WHT = 0x00FFFFFF, C_GRY = 0x00AAAAAA, C_YEL = 0x00FFCC66;
        u32 dc = diag_col + 2U;

        /* ---- NIC / MAC RX-TX ---- */
        struct macb_diag md;
        macb_diag(&md);
        nic_packet_counters_t nc;
        nic_packet_counters(&nc);
        bool bna    = (md.rsr & (1U << 0)) != 0;   /* RX Buffer Not Available */
        bool rx_ovr = (md.rsr & (1U << 2)) != 0;   /* RX overrun */
        bool rx_hno = (md.rsr & (1U << 3)) != 0;   /* RX HRESP not OK */
        bool tx_bex = (md.tsr & (1U << 4)) != 0;   /* TX buffers exhausted */
        bool tx_und = (md.tsr & (1U << 6)) != 0;   /* TX underrun */
        bool tx_hno = (md.tsr & (1U << 8)) != 0;   /* TX HRESP not OK */
        bool rx_en  = (md.ncr & (1U << 2)) != 0;   /* RX enabled */
        bool tx_en  = (md.ncr & (1U << 3)) != 0;   /* TX enabled */
        bool link   = perf.nic_link_mbps != 0U;    /* negotiated PHY/MDIO state; GEM NSR bit0 is not the link source */
        bool axi_err = (md.eth_cfg_stat & 0x30U) != 0U; /* ARLEN/AWLEN illegal = real AXI bus fault */
        bool ring_full = md.ring_size && md.rx_owned >= (md.ring_size - (md.ring_size / 8U));
        bool wedged = bna || rx_ovr || rx_hno || tx_bex || tx_und || tx_hno ||
                      axi_err || ring_full || !rx_en || !tx_en;
        dash_clear_body(diag_col, hw_row, diag_w, hw_h);
        u32 dr = hw_row + 1U;
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("STATE: ");
        fb_set_color(wedged ? C_RED : C_GRN, 0x00000000);
        fb_puts(wedged ? "** WEDGED **" : "OK");
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("RX ring owned=");
        fb_set_color(ring_full ? C_RED : C_WHT, 0x00000000);
        fb_printf("%u", md.rx_owned);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("/");
        fb_printf("%u", md.ring_size);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("RSR=0x");
        fb_set_color(C_WHT, 0x00000000);
        fb_printf("%x ", md.rsr);
        dash_flag("BNA=", bna, C_RED);
        fb_puts(" ");
        dash_flag("OVR=", rx_ovr, C_RED);
        fb_puts(" ");
        dash_flag("HNO=", rx_hno, C_RED);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("TSR=0x");
        fb_set_color(C_WHT, 0x00000000);
        fb_printf("%x ", md.tsr);
        dash_flag("UND=", tx_und, C_RED);
        fb_puts(" ");
        dash_flag("BEX=", tx_bex, C_RED);
        fb_puts(" ");
        dash_flag("HNO=", tx_hno, C_RED);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("NCR=0x");
        fb_set_color(C_WHT, 0x00000000);
        fb_printf("%x ", md.ncr);
        dash_flag("RE=", rx_en, C_GRN);
        fb_puts(" ");
        dash_flag("TE=", tx_en, C_GRN);
        fb_puts(" ");
        dash_flag("LINK=", link, C_GRN);
        fb_set_cursor(dc, dr++);
        dash_kv_u32("rx_idx=", md.rx_idx);
        fb_puts(" ");
        dash_kv_u32("tx_idx=", md.tx_idx);
        fb_set_cursor(dc, dr++);
        dash_kv_u32("rx_recv=", md.rx_recv);
        fb_puts(" ");
        dash_kv_u32("tx_send=", md.tx_send);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("rx_recover=");
        fb_set_color(md.rx_recover ? C_YEL : C_WHT, 0x00000000);
        fb_printf("%u", md.rx_recover);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" tx_drop=");
        fb_set_color(md.tx_drop ? C_YEL : C_WHT, 0x00000000);
        fb_printf("%u", md.tx_drop);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" tx_rec=");
        fb_set_color(md.tx_recover ? C_YEL : C_WHT, 0x00000000);
        fb_printf("%u", md.tx_recover);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("rx_wedge=");
        fb_set_color(md.rx_wedge ? C_YEL : C_WHT, 0x00000000);
        fb_printf("%u", md.rx_wedge);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" rx_live_rec=");
        fb_set_color(md.rx_live_recover ? C_YEL : C_WHT, 0x00000000);
        fb_printf("%u", md.rx_live_recover);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" hole=");
        fb_set_color(md.rx_hole_recover ? C_YEL : C_WHT, 0x00000000);
        fb_printf("%u", md.rx_hole_recover);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("RBQP=0x");
        fb_set_color(C_WHT, 0x00000000);
        fb_printf("%x", md.rbqp);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" TBQP=0x");
        fb_set_color(C_WHT, 0x00000000);
        fb_printf("%x", md.tbqp);
        fb_set_cursor(dc, dr++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("ETH_CFG_STAT=0x");
        fb_set_color(axi_err ? C_RED : C_WHT, 0x00000000);
        fb_printf("%x", md.eth_cfg_stat);
        if (axi_err) {
            fb_set_color(C_RED, 0x00000000);
            fb_puts(" AXI!");
        }
        fb_set_cursor(dc, dr++);
        dash_kv_u32("pkt proc=", (u32)nc.processed);
        fb_puts(" ");
        dash_kv_u32("drop=", (u32)nc.dropped);
        fb_set_cursor(dc, dr++);
        dash_kv_u32("fw=", (u32)nc.firewalled);
        fb_puts(" ");
        dash_kv_u32("rl=", (u32)nc.rate_limited);
        fb_puts(" ");
        dash_kv_u32("flood=", (u32)nc.flood_blocked);

        /* ---- DMA ENGINE ---- */
        struct dma_diag_snapshot dd;
        dma_diag_snapshot(&dd);
        dash_clear_body(diag_col, tns_row, diag_w, tns_h);
        u32 er = tns_row + 1U;
        fb_set_cursor(dc, er++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("hw=");
        fb_set_color(dd.hw_memcpy_enabled ? C_GRN : C_RED, 0x00000000);
        fb_puts(dd.hw_memcpy_enabled ? "on" : "off");
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" mode=");
        fb_set_color(C_WHT, 0x00000000);
        fb_puts(dd.direct_mode ? "direct" : "cb");
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" shift=");
        fb_set_color(C_WHT, 0x00000000);
        fb_puts(dd.cbaddr_shifted ? "y" : "n");
        fb_set_cursor(dc, er++);
        dash_kv_u32("selftest run=", dd.selftest_runs);
        fb_puts(" ");
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("probe_fail=");
        fb_set_color(C_WHT, 0x00000000);   /* mode-probe failures are expected; last_err is the real signal */
        fb_printf("%u", dd.selftest_failures);
        fb_set_cursor(dc, er++);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts("last_err=0x");
        fb_set_color(dd.last_error ? C_RED : C_WHT, 0x00000000);
        fb_printf("%x", dd.last_error);
        fb_set_color(C_GRY, 0x00000000);
        fb_puts(" en=0x");
        fb_set_color(C_WHT, 0x00000000);
        fb_printf("%x", dd.enable_reg);

        /* ---- FIFO / LEASE ARENAS ---- */
        struct lease_stats ls;
        lease_get_stats(&ls);
        dash_clear_body(diag_col, map_row, diag_w, map_h);
        u32 fr = map_row + 1U;
        const u32 fend = map_row + map_h - 1U;
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            fb_set_color(C_YEL, 0x00000000);
            fb_puts("LEASE FABRIC");
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            dash_kv_u32("arenas=", ls.arenas);
            fb_puts(" ");
            fb_set_color(C_GRY, 0x00000000);
            fb_puts("slots=");
            fb_set_color(C_WHT, 0x00000000);
            fb_printf("%u/%u", ls.slots_live, ls.slots_total);
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            dash_kv_u32("acq=", ls.acquires);
            fb_puts(" ");
            dash_kv_u32("rel=", ls.releases);
            fb_puts(" ");
            dash_kv_u32("xfer=", ls.transfers);
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            dash_kv_u32("copy=", ls.copies);
            fb_puts(" ");
            dash_kv_u32("grant=", ls.grants);
            fb_puts(" ");
            dash_kv_u32("rvk=", ls.revokes);
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            fb_set_color(C_GRY, 0x00000000);
            fb_puts("reject mmu=");
            fb_set_color(ls.mmu_rejects ? C_RED : C_WHT, 0x00000000);
            fb_printf("%u", ls.mmu_rejects);
            fb_set_color(C_GRY, 0x00000000);
            fb_puts(" stale=");
            fb_set_color(ls.stale_rejects ? C_RED : C_WHT, 0x00000000);
            fb_printf("%u", ls.stale_rejects);
            fb_set_color(C_GRY, 0x00000000);
            fb_puts(" st=");
            fb_set_color(ls.state_rejects ? C_RED : C_WHT, 0x00000000);
            fb_printf("%u", ls.state_rejects);
        }
        if (fr + 1U < fend)
            fr++;   /* spacer */
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            fb_set_color(C_YEL, 0x00000000);
            fb_puts("FIFO pending (dst<-src)");
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            dash_kv_u32("0<-1=", fifo_count(0U, 1U));
            fb_puts(" ");
            dash_kv_u32("0<-2=", fifo_count(0U, 2U));
            fb_puts(" ");
            dash_kv_u32("0<-3=", fifo_count(0U, 3U));
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            dash_kv_u32("1<-0=", fifo_count(1U, 0U));
            fb_puts(" ");
            dash_kv_u32("2<-0=", fifo_count(2U, 0U));
            fb_puts(" ");
            dash_kv_u32("3<-0=", fifo_count(3U, 0U));
        }

        /* ---- NET STACK HEALTH ---- IP / TCP / TLS / port-FIFO forwarding.
         * Constant-traffic layers; a red counter here means that layer is
         * dropping or backing up under load. Guarded so it clips on small fb. */
        {
            const net_stats_t *ns = net_get_stats();
            const tcp_diag_t *td = tcp_diag();
            u32 tcap = 0, tinuse = 0, tlisten = 0;
            tcp_table_stats(&tcap, &tinuse, &tlisten);
            struct tls_diag_snapshot ts;
            tls_diag_snapshot(&ts);
            if (fr + 1U < fend)
                fr++;   /* spacer */
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_YEL, 0x00000000);
                fb_puts("NET STACK HEALTH");
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("IP rx=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)ns->rx_packets);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" tx=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)ns->tx_packets);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" cksm=");
                fb_set_color(ns->drop_bad_cksum ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)ns->drop_bad_cksum);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" ovsz=");
                fb_set_color(ns->drop_oversized ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)ns->drop_oversized);
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("TCP tbl=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u/%u", tinuse, tcap);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" nolis=");
                fb_set_color(td->no_listener ? C_YEL : C_WHT, 0x00000000);
                fb_printf("%u", (u32)td->no_listener);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" pfull=");
                fb_set_color(td->pending_full ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)td->pending_full);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" txfail=");
                fb_set_color(td->tx_send_fail ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)td->tx_send_fail);
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("TLS rx=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)ts.records_rx);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" tx=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)ts.records_tx);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" dec!=");
                fb_set_color(ts.decrypt_failures ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)ts.decrypt_failures);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" err=");
                fb_set_color(ts.last_error ? C_RED : C_WHT, 0x00000000);
                fb_printf("%x", ts.last_error);
            }
            for (u32 bi = 0; bi < UHTTP_BRIDGE_COUNT && fr < fend; bi++) {
                i32 lc = -1; u32 st = 0, rq = 0, rs = 0, rqs = 0, mg = 0, pid = 0;
                uhttp_bridge_state_idx(bi, &lc, &st, &rq, &rs, &rqs, &mg, &pid);
                u32 lag = (rq >= rs) ? (rq - rs) : 0U;
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("FWD :");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", bi == 0U ? UHTTP_PORT : UHTTP_NATIVE_PORT);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" req=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", rqs);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" lag=");
                fb_set_color(lag > 1U ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", lag);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(mg == UHTTP_BRIDGE_MAGIC ? " ok" : " BAD");
                fb_set_color(mg == UHTTP_BRIDGE_MAGIC ? C_GRN : C_RED, 0x00000000);
            }
        }

        /* ---- INTERRUPTS / WATCHDOG / TIMER ---- the liveness layer. SW = the
         * core0 reactor wake/sleep (SEV/WFE) counts; HW = GIC IRQ totals incl.
         * the ETH RX IRQ that drives the network plane; WDOG = hardware
         * watchdog arm state + seconds-to-reset + trips. If HW IRQ or ETH stop
         * climbing while traffic flows, the board has fallen back to poll-only
         * (or wedged); a shrinking WDOG remaining with no pets = imminent reset. */
        {
            struct irq_diag_snapshot id;
            irq_diag_snapshot(&id);
            struct watchdog_status wd;
            watchdog_status(&wd);
            u64 wk = 0, wf = 0, it = 0, tt = 0; u32 bp = 0, lf = 0;
            core0_sched_snapshot(&wk, &wf, &it, &tt, &bp, &lf);
            u32 wdog_rem = watchdog_hw_remaining_ticks();
            if (fr + 1U < fend)
                fr++;   /* spacer */
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_YEL, 0x00000000);
                fb_puts("INTERRUPTS / WDOG / TIMER");
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("SW wake=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)wk);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" wfi=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)wf);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" busy=");
                fb_set_color(bp > 900U ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", bp);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("pm");
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("HW irq=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)id.total);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" tmr=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)id.timer);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" spur=");
                fb_set_color(id.spurious ? C_YEL : C_WHT, 0x00000000);
                fb_printf("%u", (u32)id.spurious);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" unh=");
                fb_set_color(id.unhandled ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)id.unhandled);
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("ETH irq=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", (u32)core0_eth_irq_count);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" qpass=");
                fb_set_color(core0_eth_irq_quench_passes > 4U ? C_YEL : C_WHT, 0x00000000);
                fb_printf("%u", core0_eth_irq_quench_passes);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" lastID=");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", id.last_intid);
            }
            if (fr < fend) {
                fb_set_cursor(dc, fr++);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("WDOG ");
                fb_set_color(wd.armed ? C_GRN : C_RED, 0x00000000);
                fb_puts(wd.armed ? "armed" : "OFF");
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" rst_in=");
                /* RSTC watchdog ticks run at ~65536 Hz on BCM2712. */
                fb_set_color(wdog_rem < (65536U) ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", wdog_rem >> 16);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("s trip=");
                fb_set_color(wd.trip_count ? C_RED : C_WHT, 0x00000000);
                fb_printf("%u", (u32)wd.trip_count);
            }
        }

        if (fr + 1U < fend)
            fr++;   /* spacer */
        if (fr < fend) {
            /* OTA state: READY (idle) -> UPLOADING (blocks streaming in) ->
             * ARMED (whole image staged, about to flash+reboot). */
            fb_set_cursor(dc, fr++);
            fb_set_color(C_YEL, 0x00000000);
            fb_puts("OTA: ");
            bool ota_up = ota_update.active && ota_update.received < ota_update.total;
            bool ota_armed = ota_update.active && ota_update.total != 0 &&
                             ota_update.received >= ota_update.total;
            if (ota_armed) {
                fb_set_color(C_RED, 0x00000000);
                fb_puts("ARMED");
            } else if (ota_up) {
                fb_set_color(C_YEL, 0x00000000);
                fb_puts("UPLOADING");
            } else {
                fb_set_color(C_GRN, 0x00000000);
                fb_puts("READY");
            }
            if (ota_update.total) {
                fb_set_color(C_GRY, 0x00000000);
                fb_puts(" ");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%u", ota_update.received >> 10);
                fb_set_color(C_GRY, 0x00000000);
                fb_puts("/");
                fb_set_color(C_WHT, 0x00000000);
                fb_printf("%uKB", ota_update.total >> 10);
            }
        }
        if (fr < fend) {
            fb_set_cursor(dc, fr++);
            dash_kv_u32("commits=", ota_update.commits);
            fb_puts(" ");
            fb_set_color(C_GRY, 0x00000000);
            fb_puts("staged=");
            fb_set_color((ota_stage_buf != NULL) ? C_GRN : C_RED, 0x00000000);
            fb_puts((ota_stage_buf != NULL) ? "y" : "n");
        }
    }

    g_dash_render_ticks = dash_now_ticks() - t_dash1;
}

/* Core 0 I/O reactor: timer IRQ only marks work due and wakes the service
 * loop. Real I/O stays in thread context so IRQ latency remains bounded. */
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
    /* UART/USB stay responsive (~31Hz, cheap polls). DASH is framebuffer-heavy
     * but cheap now that caches are alive, so refresh it at 1Hz (see offset
     * note below). While a uartflash transfer is active, service every tick
     * instead: the PL011 hardware RX FIFO is shallow (16/32 bytes), and at
     * 115200 baud (~11.5 KB/s) a 31-tick gap between services can accumulate
     * far more than that, silently dropping bytes on overflow. */
    if ((tick & 31U) == 0 || uartflash_active)
        flags |= CORE0_IO_UART | CORE0_IO_USB;
    /* CORE0_IO_NET is now driven PURELY by the ETH IRQ handler
     * (core0_eth_irq_handler sets it directly) plus the deferred-quench
     * consumer's poll-fallback backstop -- deliberately NOT forced here
     * anymore -- ONLY on real RP1/GEM hardware, where core0_eth_irq_arm_host()
     * actually arms that IRQ path at boot (see PIOS_HAS_RP1 && PIOS_HAS_GENET
     * gating there). The previous unconditional 128Hz force existed only
     * because RX IRQ delivery was previously suspected under load; that hypothesis
     * was based on an unverified "check-then-arm" race theory that RP1SPEC.md
     * 6.2 actually contradicts (IACK-while-still-asserted is documented to
     * generate a fresh MSI), and the REAL bug was a software lost-wakeup race
     * in how core0_eth_irq_deferred_quench was consumed (see the fixed
     * clear-before-drain loop below) -- not an inherent IRQ reliability
     * problem. Brute-force timer polling was masking that bug rather than
     * fixing it, at the cost of never trusting/exercising the IRQ path under
     * real load. Genuine hardware-wedge detection (RSR.BNA/OVR latch, RX
     * silence) does NOT depend on this and still runs unconditionally on the
     * CORE0_IO_MAINT cadence below, so a total IRQ failure (as opposed to a
     * missed individual wake) still self-heals.
     * On platforms without that IRQ path (e.g. QEMU's virtio-net, or any
     * build where PIOS_HAS_RP1/PIOS_HAS_GENET is 0), nothing ever arms an RX
     * interrupt at all -- so CORE0_IO_NET must still be forced periodically
     * there, exactly as before, or RX is never drained. */
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
    if ((tick & 7U) == 0) {
        flags |= CORE0_IO_TCP;
        /* Exception to "purely IRQ-driven": while the poll-only livelock
         * fallback is engaged, the GIC ETH line is deliberately masked (see
         * core0_eth_irq_drain_and_quench), so nothing else will ever set
         * CORE0_IO_NET again -- including the cooldown-based re-arm check
         * that lives inside that same branch and is what clears the
         * fallback. Without this, engaging the fallback would be a
         * permanent deadlock: masked IRQ + no periodic force = RX never
         * drains again and the board never leaves fallback mode. Restore
         * the fast poll cadence ONLY for the duration of this genuinely
         * degraded window; normal healthy operation stays purely
         * event-driven. */
        if (core0_eth_irq_poll_fallback)
            flags |= CORE0_IO_NET;
    }
#else
    if ((tick & 7U) == 0)
        flags |= CORE0_IO_NET | CORE0_IO_TCP;
#endif
    if ((tick % 100U) == 0)
        flags |= CORE0_IO_MAINT;
    /* Dashboard renders at 1Hz. The CPU-clock perf measurement also samples at
     * 1Hz, but offset by half a second (tick%1000==500) so its spin/PMU window
     * never lands on the same tick as the framebuffer-heavy render: this spreads
     * core0 load across the second and guarantees each render reads a sample at
     * most ~500ms old. Add future periodic perf captures on their own offset
     * (e.g. ==250/==750) to keep one heavy job per tick. */
    if ((tick % 1000U) == 0)
        flags |= CORE0_IO_DASH;
    if ((tick % 1000U) == 500U)
        flags |= CORE0_IO_CPUCLK;
    if (flags) {
        core0_io_flags |= flags;
        sev();
    }
}

/* Single unified per-core timer tick hook. Every core registers THIS one hook
 * (core 0 in core0_main, user cores in their main() after proc_preempt_init),
 * so there is exactly one tick_hook per core and the two responsibilities can
 * never overwrite each other's slot:
 *   - proc_timer_tick():   preemption accounting   (user cores; no-op on core 0)
 *   - core0_io_tick_hook(): reactor IO-flag marking (core 0; no-op elsewhere) */
static void pios_tick_hook(u32 core, u64 tick)
{
    proc_timer_tick(core, tick);
    core0_io_tick_hook(core, tick);
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

/* Arm RP1 Ethernet RX → GIC HOST6 delivery to core 0 (the proven sequence,
 * shared by the boot auto-arm and the `rp1 irq arm-host6` console command).
 * Continuous mode (oneshot=false) leaves the GIC line enabled; the handler
 * defers a drain+quench that W1C-clears MACB ISR.RCOMP so the next packet
 * re-triggers without storming. */
static void core0_eth_irq_arm_host(bool oneshot)
{
    __asm__ volatile("msr daifset, #2" ::: "memory");
    core0_eth_irq_oneshot = oneshot;
    irq_register(GIC_RP1_ETH_MSI, core0_eth_irq_handler);
    gic_set_group1(GIC_RP1_ETH_MSI);
    gic_set_priority(GIC_RP1_ETH_MSI, 0x40);
    gic_set_target(GIC_RP1_ETH_MSI, 1);   /* CPU0 bitmask = CORE_NET */
    gic_set_edge_triggered(GIC_RP1_ETH_MSI);
    gic_clear_pending(GIC_RP1_ETH_MSI);
    macb_irq_ack_rx();
    rp1_eth_host_arm();
    macb_irq_enable_rx();
    dsb();
    isb();
    gic_enable_irq(GIC_RP1_ETH_MSI);
    __asm__ volatile("msr daifclr, #2" ::: "memory");
}

static bool core0_eth_irq_drain_and_quench(bool host_route)
{
    (void)host_route;
    const u32 eth_bit = 1U << RP1_INT_ETH;
    u32 passes = 0;
    bool clear = false;
    /* Edge model: drain RX until the RP1 raw ETH source de-asserts (MACB has
     * no more received frames), clearing the MACB ISR/RSR each pass. The MIP
     * host status is non-latching in edge mode, so we no longer try to clear
     * it; the GIC edge was already completed by EOI in irq_dispatch. */
    for (; passes < 8U; passes++) {
        net_poll();
        core0_eth_irq_last_macb_isr = macb_irq_ack_rx();
        dsb();
        core0_eth_irq_last_mip = rp1_mip_host_status_l();
        if ((rp1_irq_status_l() & eth_bit) == 0) {
            clear = true;
            break;
        }
    }
    core0_eth_irq_quench_passes = passes < 8U ? passes + 1U : passes;
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
    {
        struct macb_diag md;
        macb_diag(&md);
        DTRACE(DTRACE_CAT_REACTOR, DT_RX_IRQ_QUENCH, core0_eth_irq_count,
               core0_eth_irq_quench_passes, clear ? 1U : 0U, md.rx_recv);
    }
#endif
    if (clear) {
        /* Per RP1SPEC.md 6.2 (MSIx configuration registers): "If a peripheral
         * interrupt is still asserted at the time the IACK register is
         * written, a new MSIx write is generated." So a frame that completes
         * DMA in the gap between our "raw line is low" observation and the
         * IACK write is NOT lost -- RP1 guarantees IACK-while-asserted
         * produces a fresh MSI. There is no check-then-arm race to close
         * here; a single unconditional rearm after the drain loop is the
         * documented-correct sequence. (An earlier version of this function
         * added a bounded retry loop around this rearm based on an
         * inferred-not-verified "edge only fires for transitions after
         * arming" model; that model contradicts the datasheet, and the retry
         * loop could itself exit on its last iteration without a final
         * rearm if the line was still high -- a real regression. Removed.) */
        rp1_eth_irq_rearm();
        core0_eth_irq_stall_streak = 0;
        return true;
    }

    /* Did NOT catch up within the 8-pass budget: the raw ETH interrupt
     * source is still asserted, meaning more frames arrived faster than we
     * could drain. Re-arming unconditionally here (the previous behaviour)
     * risks a receive livelock under sustained overload: IRQ fires, drain
     * falls behind, re-arm anyway, IRQ fires again almost immediately,
     * repeat -- burning core0's time in IRQ entry/exit rather than making
     * steady draining progress (see DT_RX_IRQ_QUENCH; live testing showed
     * rx_owned climbing into the mid-400s while RBQP kept advancing, i.e.
     * hardware still receiving but software never catching up). After a
     * few consecutive non-clearing quenches, mask the IRQ line instead of
     * re-arming and fall back to poll-only: the main reactor's net_poll()
     * plus macb_rx_recover()/macb_rx_liveness_recover() pairing (see
     * CORE0_IO_NET handling) is the exact same drain path either way, but
     * without an interrupt able to re-trigger before it has finished. */
    core0_eth_irq_stall_streak++;
    if (core0_eth_irq_stall_streak >= CORE0_ETH_IRQ_STALL_THRESHOLD &&
        !core0_eth_irq_poll_fallback) {
        core0_eth_irq_poll_fallback = true;
        core0_eth_irq_fallback_since_ms = timer_monotonic_ms();
        core0_eth_irq_fallback_count++;
        gic_disable_irq(GIC_RP1_ETH_MSI);
        DTRACE(DTRACE_CAT_REACTOR, DT_RX_IRQ_QUENCH, core0_eth_irq_count,
               0xFFFFFFFFU /* sentinel: fallback engaged */,
               core0_eth_irq_fallback_count, 0);
        http_log_event("eth-irq-poll-fallback", core0_eth_irq_stall_streak,
                       core0_eth_irq_fallback_count);
        return false;
    }
    if (!core0_eth_irq_poll_fallback)
        rp1_eth_irq_rearm();
    return false;
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
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
    /* Guaranteed-cadence hardware-wedge safety net, independent of RX IRQ
     * activity. CORE0_IO_NET (which also runs these two checks inline) is now
     * purely event-driven off the ETH IRQ handler -- if the IRQ path ever
     * fails completely (a genuine hardware wedge, as opposed to a single
     * missed software wake), CORE0_IO_NET would never fire again and these
     * checks would never run from there. Running them here too, on the
     * unconditional CORE0_IO_MAINT tick, preserves self-healing for a real
     * MAC/DMA fault regardless of whether IRQ-driven wake is currently
     * working at all. */
    if (!macb_rx_recover() && !macb_rx_hole_recover())
        macb_rx_liveness_recover(timer_monotonic_ms());
#endif
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
#if PIOS_HAS_DMA
    fb_puts("[dma] late memcpy selftest...\n");
    uart_puts("[dma] late memcpy selftest...\n");
    if (dma_selftest()) {
        fb_puts("[dma] late memcpy selftest OK\n");
        uart_puts("[dma] late memcpy selftest OK\n");
    } else {
        fb_puts("[dma] late memcpy selftest FAILED (using NEON fallback)\n");
        uart_puts("[dma] late memcpy selftest FAILED (using NEON fallback)\n");
    }
#else
    fb_puts("[dma] late memcpy selftest skipped on this platform\n");
    uart_puts("[dma] late memcpy selftest skipped on this platform\n");
#endif

    uart_vt_clear();
    uart_vt_home();
    uart_vt_color(UART_COLOR_CYAN, UART_COLOR_BLACK, true);
    ui_console_write("PIOS Serial Console is online\n");
    uart_vt_color(UART_COLOR_GREEN, UART_COLOR_BLACK, true);
    ui_console_write("Type Help for assistance!\n");
    uart_vt_reset();
    ui_console_prompt();

    timer_set_tick_hook(pios_tick_hook);
    core0_io_flags = CORE0_IO_NET | CORE0_IO_TCP | CORE0_IO_UART |
                     CORE0_IO_USB | CORE0_IO_MAINT | CORE0_IO_DASH | CORE0_IO_CPUCLK;
    core0_io_sched_start_ticks = sched_counter_ticks();
#if PIOS_HAS_BOOTINFO_FB
    bool dash_fb_ok = fb_init(1920, 1080) || fb_init(1280, 720) || fb_init(1024, 768);
    uart_puts(dash_fb_ok ? "[fb] UEFI GOP framebuffer online\n" :
                           "[fb] UEFI GOP framebuffer unavailable\n");
    /* Only paint the dashboard if a framebuffer actually initialised; on a
     * headless QEMU boot (no GOP bootinfo) there is no scanout to render to. */
    if (dash_fb_ok && ui_mode == UI_MODE_NONE) {
        hdmi_dashboard_render();
        fb_present();
    }
#else
    if (ui_mode == UI_MODE_NONE) {
        hdmi_dashboard_render();
        fb_present();
    }
#endif

    /* Auto-arm RP1 Ethernet RX → GIC HOST6 so inbound packets wake core 0 via
     * interrupt instead of relying solely on the periodic poll. The 31 Hz NET
     * poll is retained as a safety net until IRQ delivery is proven under load. */
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
    core0_eth_irq_arm_host(false);
#endif

    /* Diagnostic-trace phase threshold: only record reactor phases that take
     * longer than ~50us of work, so the trace ring captures the rare long
     * (starving) phases instead of being flooded by empty 128Hz polls. */
    u64 dt_phase_thresh;
    { u64 f; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f)); dt_phase_thresh = f / 20000U; }

    u64 stack_canary_last_check = 0;
    for (;;) {
        /* Placed at the very top of the loop, before the idle branch's
         * `continue`, so this runs every iteration regardless of whether
         * core0 is idle or servicing IO -- self-rate-limited the same way
         * watchdog_poll() throttles itself, since this is NOT wired through
         * watchdog_poll() (see stack_canary.h/.c comment history: that
         * function turned out to have zero callers anywhere in the tree,
         * and resurrecting it would also reactivate its own dormant, never-
         * fed g_wdog liveness-trip path -- watchdog_touch() likewise has no
         * callers, so every core's last_touch would sit at its init value
         * forever, and watchdog_poll() would spuriously trip ~5s after
         * every boot. Call stack_canary_check() directly instead.) */
        u64 now_ticks = timer_ticks();
        if (now_ticks - stack_canary_last_check >= 100ULL) {
            stack_canary_last_check = now_ticks;
            stack_canary_check();
        }

        u32 flags = core0_io_take_flags();
        if (flags == 0) {
            watchdog_hw_pet();
            core0_io_wfi_count++;
            u64 idle_start = sched_counter_ticks();
            /* WFI can return immediately while an event/interrupt condition is
             * still observable on this Pi5 path. Drain a stale SEV event, then
             * sleep until the next timer/IRQ/SEV so core0 does not hot-spin. */
            wfe();
            wfe();
            u64 idle_end = sched_counter_ticks();
            if (idle_end >= idle_start)
                core0_io_idle_ticks += idle_end - idle_start;
            continue;
        }

        core0_io_wake_count++;
        core0_io_last_flags = flags;

        if (flags & CORE0_IO_NET) {
            u64 svc_start = ksvc_begin(ksvc_net_id);
            u64 dt_t0 = sched_counter_ticks();
            u32 got = net_poll();
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
            /* Self-heal a latched RX-overrun stall (BNA/OVR). A burst that
             * outran the drain leaves GEM with the ring full and RX DMA halted;
             * without this the NIC stays wedged (rx_idx frozen) even after the
             * load stops. Recover, then drain the freshly-restarted ring. */
            if (macb_rx_recover())
                got += net_poll();
            else if (macb_rx_hole_recover())
                got += net_poll();
            /* Also self-heal a non-BNA/OVR RX halt (which the status check above
             * cannot see): if the RX-liveness watchdog (activity-gated) fires,
             * rebuild the ring and re-drain. */
            else if (macb_rx_liveness_recover(timer_monotonic_ms()))
                got += net_poll();
#endif
            /* Chain CORE0_IO_TCP directly off real MAC-layer completion
             * (hardirq -> softirq handoff), rather than waiting for TCP's own
             * independent periodic tick: if net_poll() actually delivered new
             * frames this pass, the TCP-level connection/app-layer services
             * (admin/OTA, echo, :81 bridge) have new segment data to react to
             * right now, and the periodic tick could be up to 8 ticks away.
             * This mirrors what core0_eth_irq_handler already does for the
             * IRQ-driven wake path (it sets both bits together in one step);
             * this closes the same gap for the poll/deferred-quench-driven
             * path. TCP's own periodic tick (see core0_io_tick_hook) is still
             * needed independently for TX-side pumping when there's pending
             * output but no new RX to chain off of. */
            if (got > 0)
                core0_io_flags |= CORE0_IO_TCP;
            dns_poll();
            /* Consume the deferred-quench request clear-before-work, not
             * clear-after: the old order (drain, then unconditionally clear)
             * has a real lost-wakeup window -- core0_eth_irq_handler runs on
             * this same core and can fire between the drain call returning
             * and the "= false" store, setting the flag true only for it to
             * be immediately clobbered back to false by mainline, silently
             * dropping that request. Clearing first and looping while the
             * flag keeps getting re-set (by a fresh IRQ arriving mid-drain)
             * closes that window: any such IRQ causes another drain pass
             * instead of being lost. */
            while (core0_eth_irq_deferred_quench) {
                core0_eth_irq_deferred_quench = false;
                core0_eth_irq_drain_and_quench(false);
            }
#if PIOS_HAS_RP1 && PIOS_HAS_GENET
            /* Recovered from a poll-only livelock fallback: after a cooldown
             * (during which this same net_poll()/macb_rx_recover() pairing
             * above is what's been draining the ring, IRQ-free), try
             * IRQ-driven wake again. If the overload was transient this
             * restores lower-latency wake; if it recurs immediately the
             * stall-streak counter will just re-trip the fallback. */
            if (core0_eth_irq_poll_fallback &&
                timer_monotonic_ms() - core0_eth_irq_fallback_since_ms > CORE0_ETH_IRQ_FALLBACK_COOLDOWN_MS) {
                core0_eth_irq_poll_fallback = false;
                core0_eth_irq_stall_streak = 0;
                core0_eth_irq_arm_host(false);
                http_log_event("eth-irq-poll-fallback-end", core0_eth_irq_fallback_count, 0);
            }
#endif
            u64 dt_net = sched_counter_ticks() - dt_t0;
            if (dt_net > dt_phase_thresh)
                DTRACE(DTRACE_CAT_REACTOR, DT_RX_PHASE_NET, dt_net, flags, 0, 0);
            ksvc_end(ksvc_net_id, svc_start, false);
        }

        if (flags & CORE0_IO_TCP) {
            u64 svc_start = ksvc_begin(ksvc_tcp_id);
            /* Drain the admin services (incl. the OTA upload stream) FIRST and
             * independently of the heavy multi-connection :80 handler, so a bulk
             * OTA upload is never starved of core0 time by :80 load. */
            u64 dt_a0 = sched_counter_ticks();
            admin_services_poll();
            u64 dt_a1 = sched_counter_ticks();
            if (dt_a1 - dt_a0 > dt_phase_thresh)
                DTRACE(DTRACE_CAT_REACTOR, DT_RX_PHASE_ADMIN, dt_a1 - dt_a0, 0, 0, 0);
            echo_tcp_poll();
            u64 dt_echo = sched_counter_ticks() - dt_a1;
            if (dt_echo > dt_phase_thresh)
                DTRACE(DTRACE_CAT_REACTOR, DT_RX_PHASE_ECHO, dt_echo, 0, 0, 0);
            u64 dt_http0 = sched_counter_ticks();
            uhttp_bridge_poll();   /* userland :81 request/response pump */
            u64 dt_http = sched_counter_ticks() - dt_http0;
            if (dt_http > dt_phase_thresh)
                DTRACE(DTRACE_CAT_REACTOR, DT_RX_PHASE_HTTP, dt_http, 0, 0, 0);
            ksvc_end(ksvc_tcp_id, svc_start, false);
            ksvc_run(ksvc_debug_id);
        }

        if (flags & (CORE0_IO_UART | CORE0_IO_USB)) {
            u64 svc_start = ksvc_begin(ksvc_ui_id);
            if (uartflash_active) {
                /* Drain the WHOLE FIFO this tick for throughput regardless of
                 * whether we're mid-region or between commands right now --
                 * the PL011 hardware RX FIFO is shallow and a slow drain
                 * risks losing bytes either way. Route each byte based on
                 * whether we're genuinely expecting raw data RIGHT NOW:
                 * mid-transfer bytes go to uartflash_feed_byte(), but bytes
                 * arriving between regions (e.g. a "uartflash
                 * status"/"commit"/next "patch <off> <len>" command) must
                 * still reach the normal text console parser, or they'd be
                 * silently swallowed as bogus binary data. Checking this
                 * per-byte (not once before the loop) matters: the last byte
                 * of a chunk/region can flip expecting-bytes from true to
                 * false mid-drain, and the very next byte in the same FIFO
                 * burst could already be the start of a text command. */
                i32 rx;
                while ((rx = uart_try_getc()) >= 0) {
                    /* A command that just executed (via '\r' or '\n') may
                     * have synchronously flipped us into a raw-byte-
                     * consuming mode (uartflash begin/patch). If the client
                     * sent the conventional "\r\n" pair, the second half
                     * arrives here as the very next byte and would
                     * otherwise be misrouted as the FIRST byte of that raw
                     * stream, corrupting the transfer by one byte. Swallow
                     * the paired opposite terminator before the mode check
                     * even runs; a repeated identical terminator (e.g. two
                     * bare \r) still reaches ui_console_feed_char()
                     * untouched and executes as two separate lines. */
                    if ((rx == '\r' || rx == '\n') &&
                        ui_console_last_term_char >= 0 &&
                        ui_console_last_term_char != rx) {
                        ui_console_last_term_char = -1;
                        continue;
                    }
                    if (uartflash_expecting_bytes())
                        uartflash_feed_byte((u8)rx);
                    else
                        ui_console_feed_char(rx);
                }
                uartflash_poll();
            } else {
                for (u32 i = 0; i < 16; i++) {
                    i32 rx = uart_try_getc();
                    if (rx < 0)
                        break;
                    ui_console_feed_char(rx);
                }
            }

            ui_handle_keys();
            ksvc_end(ksvc_ui_id, svc_start, false);
        }

        if (flags & CORE0_IO_MAINT) {
            ksvc_run(ksvc_timer_id);
        }

        if ((flags & CORE0_IO_DASH) && ui_mode == UI_MODE_NONE)
            ksvc_run(ksvc_dashboard_id);

        if (flags & CORE0_IO_CPUCLK)
            perf_cpu_clock_measure(2000U);

        fb_present();   /* flush any dirty back-buffer rows to the scanout */
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
    timer_set_tick_hook(pios_tick_hook);
    core_mark_online(CORE_USERM, 5);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* Core 2: User core 0 - process scheduler */
/* Embedded userland HTTP server flat binaries (src/user_httpd_payload.S). */
extern const u8 user_httpd_vm_start[];
extern const u8 user_httpd_vm_end[];
extern const u8 user_httpd_native_start[];
extern const u8 user_httpd_native_end[];
extern const u8 user_el0_probe_start[];
extern const u8 user_el0_probe_end[];
extern const u8 user_el0_pico_start[];
extern const u8 user_el0_pico_end[];

/* Physical slot bases for the kernel-embedded EL0 HTTP workers, derived from
 * the platform core RAM map so the identical launch path works on Pi5 and
 * QEMU. proc_exec_from_mem_el0() recomputes the slot from
 * (physical_base - core_ram_base - PROC_SLOT_OFFSET) / PROC_SLOT_SIZE and
 * refuses a mismatch, so each value must equal slot_base(slot) for its core.
 * core 2 uses slot 0; core 3 deliberately uses slot 1. The linked base is the
 * high EL0 image VA (mmu.c USER_HIGH_LINK_BASE). */
#define HTTPD_VM_EL0_LINK_BASE   0x2001000000ULL
#define HTTPD_VM_EL0_CORE2_SLOT  0U
#define HTTPD_VM_EL0_CORE3_SLOT  1U
#define HTTPD_VM_EL0_CORE2_PHYS  \
    (CORE2_RAM_BASE + PROC_SLOT_OFFSET + (u64)HTTPD_VM_EL0_CORE2_SLOT * PROC_SLOT_SIZE)
#define HTTPD_VM_EL0_CORE3_PHYS  \
    (CORE3_RAM_BASE + PROC_SLOT_OFFSET + (u64)HTTPD_VM_EL0_CORE3_SLOT * PROC_SLOT_SIZE)
#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
_Static_assert(HTTPD_VM_EL0_CORE2_PHYS == 0x02900000ULL,
               "core2 EL0 slot base must stay 0x02900000 on Pi5");
_Static_assert(HTTPD_VM_EL0_CORE3_PHYS == 0x03B00000ULL,
               "core3 EL0 slot base must stay 0x03B00000 on Pi5");
#endif
NORETURN void core2_main(void) {
    core_mark_online(CORE_USER0, 1);
    core_env_init(CORE_USER0);
    core_mark_online(CORE_USER0, 2);
    core_mark_online(CORE_USER0, 0x200U + core_id());
    proc_init();
    core_mark_online(CORE_USER0, 3);
    timer_init(PROC_PREEMPT_TIMER_HZ);
    core_mark_online(CORE_USER0, 4);
    proc_mark_core_hosts_process(CORE_USER0);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    timer_set_tick_hook(pios_tick_hook);
    core_mark_online(CORE_USER0, 5);
    /* Launch the PicoScript VM-backed HTTP benchmark worker on port 82. */
    proc_exec_from_mem_el0("user/httpd-vm-el0", user_httpd_vm_start,
                           (u32)(usize)(user_httpd_vm_end - user_httpd_vm_start),
                           HTTPD_VM_EL0_LINK_BASE, HTTPD_VM_EL0_CORE2_PHYS, PROC_PRIO_NORMAL, core_id());
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
    proc_mark_core_hosts_process(CORE_USER1);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    timer_set_tick_hook(pios_tick_hook);
    core_mark_online(CORE_USER1, 5);
    /* Launch a second PicoScript VM-backed HTTP worker on bridge 1 / port 83. */
    proc_exec_from_mem_el0("user/httpd-vm1-el0", user_httpd_native_start,
                           (u32)(usize)(user_httpd_native_end - user_httpd_native_start),
                           HTTPD_VM_EL0_LINK_BASE, HTTPD_VM_EL0_CORE3_PHYS, PROC_PRIO_NORMAL, core_id());
    {
        u64 sctlr;
        __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
        sctlr |= (1ULL << 23); /* SPAN: do not force PAN on EL1 exception entry */
        __asm__ volatile("msr sctlr_el1, %0\nisb" :: "r"(sctlr) : "memory");
    }
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
#if !PIOS_HAS_MAILBOX_FB
    uart_puts("PIOS ");
    uart_puts(PIOS_BUILD_LABEL);
    uart_puts(PIOS_HAS_BOOTINFO_FB ? " - platform framebuffer deferred\n" :
                                      " - platform framebuffer skipped\n");
    return;
#endif
    /* Ramp the A76 to the firmware's max clock before anything else — bare-metal
     * Pi 5 otherwise runs at a low default, making the whole system ~10-100x
     * slower (slow FB/IPC/HTTP, high idle). */
#if PIOS_HAS_MAILBOX_FB
    fb_set_arm_clock_max();
#endif
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

    fb_present();   /* force the crash frame to the scanout before halting */

    for (;;) __asm__ volatile("wfe");
}

/* ---- Main kernel entry ---- */

/* Draw register panel on the right side of screen (col 65+) */
static void reg_panel(u32 at_el1) {
#if !PIOS_HAS_MAILBOX_FB
    uart_puts("[reg] CPU Regs skipped on platform without framebuffer; EL=");
    uart_hex(at_el1 ? 1 : 2);
    uart_puts("\n");
    return;
#endif
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
    /* Seed the stack-protector canary as early as possible, before any
     * deeper subsystem init runs (see stackprot.h). */
    stackprot_init();

    bool usb_ok = false;
    bool fb_ok = PIOS_HAS_BOOTINFO_FB ? false : true;  /* Pi FB is early; QEMU GOP is deferred. */
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
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    bp_log_y = BP_LIST_ROW + BP_COUNT + 2;
    uart_puts("\n[bt] PIOS ");
    uart_puts(PIOS_BUILD_LABEL);
    uart_puts(" Boot (serial progress; dashboard after ready)\n");
#else
    bp_init();
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
    uart_puts("[bt] serial progress ready\n");
#endif
    reg_panel(at_el1);
    bp_done(0, true);                     /* Firmware handoff */
    bp_done(1, true);                     /* VideoCore (fb from EL2) */
#endif

    /* Show FB physical address — needed for MMU mapping */
#if !(PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB)
    {
        u64 fb_addr = fb_get_phys_addr();
        fb_set_cursor(1, bp_log_y);
        fb_set_color(0x0000CCFF, BOOT_BLACK);
        fb_printf("FB phys=0x%X  size=%u", fb_addr, fb_addr ? 1024*768*4 : 0);
        bp_log_y++;
    }
#endif

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

        /* Phase 1 cache-coherency cleanup: make per-core RAM + HDMI back
         * buffer WB cacheable + Inner-Shareable (DSU-coherent); keep DMA /
         * FIFO / IPC / scanout NC. Must run before core_start_all() so
         * secondaries inherit the rebuilt l1_table via shared_ttbr0.
         *
         * RE-ENABLED (PIOS_ENABLE_CACHE_REMAP=1): the first-1GB remap is now
         * break-before-make compliant — mmu_enable_caching() builds a fresh
         * l1_table_cached off to the side and installs it via a single atomic
         * TTBR0_EL1 swap + full TLB flush (no in-place block->table edit of the
         * live table), eliminating the TLB-conflict PiSOD. A/B health-gated OTA
         * still backstops a bad boot. */
#ifndef PIOS_ENABLE_CACHE_REMAP
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#define PIOS_ENABLE_CACHE_REMAP 0
#else
#define PIOS_ENABLE_CACHE_REMAP 1
#endif
#endif
#if PIOS_ENABLE_CACHE_REMAP
        mmu_enable_caching();
        bp_ok("[mmu] cache map: code+core-RAM+FB WB-IS, bss/stacks/DMA/FIFO/IPC NC");
#else
        bp_warn("[mmu] cache remap GATED (PIOS_ENABLE_CACHE_REMAP=0)");
#endif

        {
#if PIOS_HAS_MAILBOX_FB
            struct board_revision_snapshot br;
            board_revision_snapshot(&br);
            bool high_ok = highmem_init(br.installed_ram_bytes, perf_physical_ram_bytes());
            bp_log(high_ok ? "[ram] highmem 1-4GiB probe OK" :
                             "[ram] highmem unavailable/probe failed");
#else
            bp_warn("[ram] highmem probe skipped on non-Pi platform");
#endif
        }

        bp_log("[exc] exception_init...");
        exception_init();
        bp_ok("[exc] vectors installed");

        /* g_boot_el/g_el2_active (el2.c) were never populated before this:
         * el2_init() had no caller anywhere in the tree, so despite
         * el2_boot_el_state being correctly recorded by boot assembly
         * (vectors.S) when the core genuinely starts at real EL2,
         * el2_hvc_call() always took the software-only fallback path
         * (a plain C function call, never a real `hvc` trap), and
         * el2_stage2_program_hw()'s read_current_el()!=2 check always
         * failed silently -- meaning VTTBR_EL2/HCR_EL2 were NEVER actually
         * programmed, on any platform, regardless of capsule bookkeeping
         * reporting success. Found via rubber-duck review. This activates
         * that dormant hardware path for the first time; see AGENTS.md
         * continuation note before trusting stage-2 as a real boundary on
         * physical Pi5 hardware -- it has only been exercised on QEMU here. */
        el2_init();
        bp_log(el2_active() ? "[el2] real EL2 active, stage-2 HW path live"
                             : "[el2] not at EL2 (boot_el != 2); stage-2 stays software-only");

        bp_log("[gic] gic_init...");
        gic_init();
        bp_ok("[gic] distributor + CPU iface ready");

        bp_log("[timer] timer_init(1000Hz)...");
        timer_init(1000);
        bp_ok("[timer] 1kHz tick running");

        bp_log("[wdog] watchdog_init(5s)...");
        watchdog_init(5000, false);
        bp_ok("[wdog] armed");

        stack_canary_init();

        bp_log("[irq] unmasking IRQs...");
        __asm__ volatile("msr daifclr, #2");
        bp_ok("[irq] IRQs live");

        reg_panel(at_el1);
    } else {
        bp_warn("[skip] all EL1 inits");
    }

    bp_log("[dma] dma_init...");
#if PIOS_HAS_DMA
    dma_init();
    bp_ok("[dma] 6-channel engine ready");
#else
    bp_warn("[dma] skipped on this platform");
#endif
    bp_done(1, true);
    watchdog_hw_pet();

    /* ── Phase 2+3: PCIe + RP1 + USB ── */
    bp_active(2);
#if PIOS_HAS_PCIE && PIOS_HAS_RP1
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
#else
    bp_warn("[pcie] skipped on this platform");
    bp_done(2, true);
    bp_done(3, true);
#endif
    watchdog_hw_pet();

    /* IPC */
    bp_log("[fifo] fifo_init_all...");
    fifo_init_all();
    dtrace_init();
    coredump_init();
    bp_log("[ipc] ipc_queue_init...");
    ipc_queue_init();
    bp_log("[ipc] ipc_stream_init...");
    ipc_stream_init();
    ipc_proc_init();
    lease_init();   /* lease fabric (P0): descriptor-ownership pool init */
    ota_staging_init();   /* pre-allocate OTA RAM staging so uploads don't block core0 on SD */
    http_conns_init();    /* allocate the multi-connection HTTP :80 pool */
    bp_log("[ipc] pipe_init...");
    pipe_init();
    bp_ok("[ipc] all channels ready");

    /* ── Phase 4: Filesystem ── */
    bp_active(4);
    bp_log(PIOS_HAS_SD ? "[sd] sd_init (EMMC2)..." : "[sd] sd_init (platform block)...");
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
        bp_log("[rng] crypto_random_init...");
        crypto_random_init();
        if (crypto_random_available())
            bp_ok("[rng] trusted entropy source online");
        else
            bp_warn("[rng] no trusted entropy source (secrets require explicit provisioning)");
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
            /* STS user store is WALFS-backed; mount it now that WALFS +
             * principal identity are online. Fails closed internally (no
             * secret / unset clock gate at issue time). */
            bp_log("[sts] sts_init...");
            if (sts_init())
                bp_ok("[sts] token service mounted");
            else
                bp_warn("[sts] token service mount FAILED");
        } else {
            bp_err("[walfs] WALFS init FAILED");
        }
        if (walfs_ok) bp_ok("[fs] SD + WALFS online");
        else bp_warn("[fs] SD ok, WALFS failed");
        bp_log("[key] keystore_init...");
        if (keystore_init()) {            bp_ok("[key] sealed root ready");
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

    /* ── Phase 5: NIC (runtime backend probe: macb / virtio-net / ...) ── */
    bp_active(5);
    bp_log("[nic] nic_init (probe backends)...");
    nic_ok = nic_init();
    if (!nic_ok) {
        bp_warn("[nic] no NIC hardware detected");
        bp_done(5, true);
    } else {
        uart_puts("[nic] active backend: ");
        uart_puts(nic_active_name());
        uart_puts("\n");
        bp_ok("[nic] online");
        if (nic_link_up()) bp_ok("[nic] link UP");
        else bp_warn("[nic] link DOWN");
        bp_done(5, true);
    }

    /* Init network stack — only if a NIC was activated. */
    if (nic_ok) {
        bp_log("[net] net_init (static IP)...");
        net_init(MY_IP, MY_GW, MY_MASK, MY_GW_MAC);
        /* Pin the dev host PC as a static neighbor so we never need ARP
         * resolution to talk back to it, and unsolicited replies are valid. */
        net_add_neighbor(HOST_PC_IP, HOST_PC_MAC);
        (void)net_route_add(HOST_PC_IP, 0xFFFFFFFFU, 0, NET_ROUTE_F_CONNECTED);
        uart_puts("[net] static neighbor 192.168.218.9 -> 04:bf:1b:e1:d7:78\n");
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
        /* QEMU user-mode (SLIRP) assigns its gateway a deterministic MAC of
         * 52:55:<gateway-ipv4>. Pin it so the stack never has to ARP-resolve the
         * gateway: SLIRP answers each ARP request with a single reply, which
         * otherwise loses the race against the anti-spoof 2-reply consistency
         * requirement (arp.c ARP_CONSISTENCY_COUNT) and the SYN-retransmit/curl
         * timeout, so inbound connections never get a SYN-ACK. This is a QEMU
         * test-harness convenience; real hardware resolves the gateway via ARP. */
        {
            u8 gwmac[6] = { 0x52, 0x55,
                            (u8)(MY_GW >> 24), (u8)(MY_GW >> 16),
                            (u8)(MY_GW >> 8),  (u8)MY_GW };
            net_add_neighbor(MY_GW, gwmac);
            uart_puts("[net] QEMU SLIRP gateway MAC pinned\n");
        }
#endif
        ui_cfg_ip = MY_IP;
        ui_cfg_mask = MY_MASK;
        ui_cfg_gw = MY_GW;
        ui_cfg_dns = MY_GW;
        ui_cfg_dhcp = false;
        dns_init(ui_cfg_dns);
        net_services_listen();
        uart_puts("[net] HTTP:80 HTTPS:443 DBG:2323\n");
        bp_ok("[net] IP stack ready");
    } else {
        uart_puts("[net] no NIC detected — network disabled\n");
        bp_warn("[net] disabled (no NIC)");
    }
    watchdog_hw_pet();

    /* Native VideoCore visibility probe, then GPU + Tensor. */
    bp_log("[vc] native probe...");
    videocore_init();
    vc_display_init();
    vc_display_probe_native();
    bp_ok(PIOS_ENABLE_NATIVE_VIDEOCORE ? "[vc] native probe complete" :
                                          "[vc] native probe disabled");
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
        /* Initialise the single-instance shared process tables on core 0 BEFORE
         * launching the secondaries, so cores 1-3 never race to zero procs[] and
         * a late core cannot wipe a process another core just created. */
        proc_init_shared();
    }
    if (at_el1 && PIOS_HAS_PSCI_SECONDARIES) {
        bp_log("[smp] core_start_all (PSCI)...");
        core_start_all();
        bp_ok("[smp] cores 0-3 active");
        bp_done(6, true);
    } else if (at_el1) {
        bp_warn("[skip] multicore secondaries on this platform");
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

            fb_set_cursor(1, bp_log_y++);
            fb_set_color(0x0000CCFF, BOOT_BLACK);
            if (nic_ok) {
                u8 mac[6];
                nic_get_mac(mac);
                fb_printf("MAC  %x:%x:%x:%x:%x:%x",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            } else {
                fb_puts("MAC  unavailable on this platform");
            }

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
            fb_puts(PIOS_HAS_RP1 ? "Serial console active on RP1 UART0" :
                                    "Serial console active on platform UART");
        }
    }

    /* HDMI stays on boot diags */

    if (at_el1) {
        core0_main();
    } else {
        for (;;) __asm__ volatile("wfe");
    }
}
