/*
 * dns.c - Minimal hardened DNS stub resolver
 *
 * Sends A-record queries over UDP port 53, parses responses,
 * caches results in a 16-entry LRU table.
 *
 * Wire format: RFC 1035 §4
 * Hardening: TXID random, source port random, strict validation,
 *            TTL clamping, reject TC=1, single-question enforcement.
 */

#include "dns.h"
#include "net.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"
#include "lru.h"

/* ---- DNS wire format constants ---- */

#define DNS_PORT        53
#define DNS_TYPE_A      1
#define DNS_CLASS_IN    1
#define DNS_HDR_SIZE    12

/* DNS header flags */
#define DNS_QR          (1U << 15)  /* response */
#define DNS_RD          (1U << 8)   /* recursion desired */
#define DNS_TC          (1U << 9)   /* truncated */
#define DNS_RCODE_MASK  0x000F

/* ---- Hardening limits ---- */

#define MIN_TTL         60
#define MAX_TTL         86400
#define QUERY_TIMEOUT   3000    /* ms */
#define MAX_RETRIES     3
#define SRC_PORT_BASE   49152

/* ---- Cache (LRU with TTL) ---- */

static struct lru_cache dns_cache;
static u32 dns_server;
static u16 txid_counter;

/* ---- Response state (set by UDP callback) ---- */

static volatile bool got_response;
static u32 resp_ip;
static u16 expected_txid;
static u32 resp_ttl;
static struct dns_async_status async_status;
static u16 async_src_port;
static u16 async_txid;
static u32 async_query_len;
static u64 async_deadline;
static bool async_callback_installed;

#define DNS_ERR_NONE       0U
#define DNS_ERR_NO_SERVER  1U
#define DNS_ERR_BAD_HOST   2U
#define DNS_ERR_SEND       3U
#define DNS_ERR_TIMEOUT    4U

/* ---- Helpers ---- */

static u32 str_hash(const char *s) {
    u32 h = 5381;
    while (*s)
        h = ((h << 5) + h) + (u8)*s++;
    return h;
}

static void str_copy(char *dst, u32 cap, const char *src)
{
    u32 i = 0;
    if (!dst || cap == 0) return;
    if (src) {
        while (i + 1 < cap && src[i]) {
            dst[i] = src[i];
            i++;
        }
    }
    dst[i] = 0;
}

static u16 gen_txid(void) {
    u64 seed = timer_ticks() + (++txid_counter);
    return (u16)(hw_crc32c(&seed, sizeof(seed)) & 0xFFFF);
}

static u16 gen_src_port(void) {
    u64 seed = timer_ticks() ^ 0xDEAD;
    return (u16)(SRC_PORT_BASE + (hw_crc32c(&seed, sizeof(seed)) & 0x3FFF));
}

/* ---- QNAME encoding ---- */

static u32 encode_qname(u8 *buf, u32 max_len, const char *hostname) {
    u32 off = 0;
    const char *p = hostname;

    while (*p) {
        const char *dot = p;
        while (*dot && *dot != '.') dot++;
        u32 label_len = (u32)(dot - p);
        if (label_len == 0 || label_len > 63) return 0;
        if (p[0] == '-' || p[label_len - 1] == '-') return 0;
        /* 1 byte length prefix + label + 1 byte root terminator must fit */
        if (off + 1 + label_len + 1 > max_len) return 0;
        buf[off++] = (u8)label_len;
        for (u32 i = 0; i < label_len; i++) {
            u8 c = (u8)p[i];
            bool ok = ((c >= 'a' && c <= 'z') ||
                       (c >= 'A' && c <= 'Z') ||
                       (c >= '0' && c <= '9') ||
                       c == '-');
            if (!ok) return 0;
            buf[off++] = c;
        }
        p = (*dot == '.') ? dot + 1 : dot;
    }
    if (off + 1 > max_len) return 0;
    buf[off++] = 0; /* root label */
    return off;
}

/* ---- Query construction ---- */

static u8 query_buf[256] ALIGNED(64);

static u32 build_query(u16 txid, const char *hostname) {
    /* Hostname length check per RFC 1035 */
    u32 hostname_len = 0;
    while (hostname[hostname_len]) hostname_len++;
    if (hostname_len > 253) return 0;

    u8 *p = query_buf;

    /* Header */
    p[0] = (u8)(txid >> 8); p[1] = (u8)txid;
    u16 flags = DNS_RD;
    p[2] = (u8)(flags >> 8); p[3] = (u8)flags;
    p[4] = 0; p[5] = 1;   /* QDCOUNT = 1 */
    p[6] = 0; p[7] = 0;   /* ANCOUNT = 0 */
    p[8] = 0; p[9] = 0;   /* NSCOUNT = 0 */
    p[10] = 0; p[11] = 0; /* ARCOUNT = 0 */

    /* Question - encode_qname must fit within remaining query_buf space */
    u32 qname_max = sizeof(query_buf) - DNS_HDR_SIZE - 4; /* 4 = QTYPE+QCLASS */
    u32 qname_len = encode_qname(p + DNS_HDR_SIZE, qname_max, hostname);
    if (qname_len == 0) return 0;

    u32 off = DNS_HDR_SIZE + qname_len;
    if (off + 4 > sizeof(query_buf)) return 0;
    p[off++] = 0; p[off++] = DNS_TYPE_A;   /* QTYPE = A */
    p[off++] = 0; p[off++] = DNS_CLASS_IN; /* QCLASS = IN */

    return off;
}

/* ---- Response parsing ---- */

static u16 read_u16_be(const u8 *p) {
    return ((u16)p[0] << 8) | p[1];
}

static u32 read_u32_be(const u8 *p) {
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

/* Skip a DNS name (handles compression pointers) */
static u32 skip_name(const u8 *buf, u32 off, u32 len) {
    u32 hops = 0;
    while (off < len && hops < 128) {
        u8 label = buf[off];
        if (label == 0) { off++; break; }
        if ((label & 0xC0) == 0xC0) {
            if (off + 1 >= len) return len; /* truncated pointer */
            off += 2;
            break;
        }
        if (label & 0xC0) return len;   /* reserved label type — reject */
        if (off + 1 + label > len) return len; /* bounds check */
        off += 1 + label;
        hops++;
    }
    if (hops >= 128) return len;
    return off;
}

static void parse_response(const u8 *data, u16 len) {
    if (len < DNS_HDR_SIZE) {
        async_status.rx_rejected++;
        return;
    }

    u16 txid = read_u16_be(data);
    u16 flags = read_u16_be(data + 2);
    u16 qdcount = read_u16_be(data + 4);
    u16 ancount = read_u16_be(data + 6);

    /* Strict TXID match */
    if (txid != expected_txid) {
        async_status.rx_rejected++;
        return;
    }

    /* Must be a response */
    if (!(flags & DNS_QR)) {
        async_status.rx_rejected++;
        return;
    }

    /* Reject truncated */
    if (flags & DNS_TC) {
        async_status.rx_rejected++;
        return;
    }

    /* Check RCODE == 0 (no error) */
    if (flags & DNS_RCODE_MASK) {
        async_status.rx_rejected++;
        return;
    }

    /* Single-question enforcement */
    if (qdcount != 1) {
        async_status.rx_rejected++;
        return;
    }

    /* Skip question section */
    u32 off = DNS_HDR_SIZE;
    off = skip_name(data, off, len);
    if (off + 4 > len) {
        async_status.rx_rejected++;
        return;
    }
    off += 4; /* QTYPE + QCLASS */

    /* Parse answers, find first A record */
    for (u16 i = 0; i < ancount && off < len; i++) {
        off = skip_name(data, off, len);
        if (off + 10 > len) {
            async_status.rx_rejected++;
            return;
        }

        u16 atype = read_u16_be(data + off);
        u16 aclass = read_u16_be(data + off + 2);
        u32 attl = read_u32_be(data + off + 4);
        u16 rdlen = read_u16_be(data + off + 8);
        off += 10;

        if (off + rdlen > len) {
            async_status.rx_rejected++;
            return;
        }

        if (atype == DNS_TYPE_A && aclass == DNS_CLASS_IN && rdlen == 4) {
            resp_ip = read_u32_be(data + off);
            /* TTL clamping */
            if (attl < MIN_TTL) attl = MIN_TTL;
            if (attl > MAX_TTL) attl = MAX_TTL;
            resp_ttl = attl;
            got_response = true;
            async_status.rx_ok++;
            return;
        }
        off += rdlen;
    }
    async_status.rx_rejected++;
}

/* ---- UDP callback ---- */

static udp_recv_cb prev_callback;

static void dns_udp_handler(u32 src_ip, u16 src_port, u16 dst_port,
                             const u8 *data, u16 len) {
    async_status.rx_total++;
    async_status.last_rx_src_ip = src_ip;
    async_status.last_rx_src_port = src_port;
    async_status.last_rx_dst_port = dst_port;
    async_status.last_rx_len = len;

    /* Source IP validation: only accept from our DNS server */
    if (src_ip != dns_server || src_port != DNS_PORT) {
        async_status.rx_ignored++;
        if (prev_callback)
            prev_callback(src_ip, src_port, dst_port, data, len);
        return;
    }

    async_status.rx_server++;
    parse_response(data, len);
}

/* ---- Cache operations (via LRU) ---- */

static u32 *cache_lookup(u32 hash) {
    struct lru_entry *e = lru_get(&dns_cache, (const u8 *)&hash, sizeof(hash));
    if (!e) return NULL;
    return (u32 *)e->value;
}

static void cache_insert(u32 hash, u32 ip, u32 ttl_sec) {
    (void)ttl_sec; /* TTL handled by LRU cache-wide ttl_ms */
    lru_put(&dns_cache, (const u8 *)&hash, sizeof(hash), &ip, sizeof(ip));
}

/* ---- Public API ---- */

void dns_init(u32 server_ip) {
    dns_server = server_ip;
    txid_counter = 0;
    lru_init(&dns_cache, NULL, 60000); /* 60s TTL */
    simd_zero(&async_status, sizeof(async_status));
    async_status.state = DNS_ASYNC_IDLE;
    async_status.server_ip = server_ip;
    async_callback_installed = false;
    uart_puts("[dns] Server: ");
    uart_hex(server_ip);
    uart_puts("\n");
}

void dns_cache_flush(void) {
    lru_flush(&dns_cache);
}

void dns_cache_stats(u64 *hits, u64 *misses, u64 *evictions) {
    u64 h = 0, m = 0, ev = 0, ex = 0;
    lru_stats(&dns_cache, &h, &m, &ev, &ex);
    if (hits) *hits = h;
    if (misses) *misses = m;
    if (evictions) *evictions = ev;
}

bool dns_cache_lookup(const char *hostname, u32 *ip_out)
{
    if (!hostname || !ip_out)
        return false;
    u32 hash = str_hash(hostname);
    u32 *cached_ip = cache_lookup(hash);
    if (!cached_ip)
        return false;
    *ip_out = *cached_ip;
    return true;
}

static void dns_async_finish(u32 state, u32 err)
{
    if (async_callback_installed) {
        net_set_udp_callback(prev_callback);
        async_callback_installed = false;
    }
    async_status.state = state;
    async_status.last_error = err;
}

static bool dns_async_send_query(void)
{
    bool sent = net_send_udp(dns_server, async_src_port, DNS_PORT,
                             query_buf, (u16)async_query_len);
    async_status.attempts++;
    async_deadline = timer_monotonic_ms() + QUERY_TIMEOUT;
    if (!sent)
        async_status.last_error = DNS_ERR_SEND;
    return sent;
}

bool dns_resolve_async_start(const char *hostname)
{
    u32 cached = 0;
    if (!dns_server) {
        async_status.last_error = DNS_ERR_NO_SERVER;
        return false;
    }
    if (!hostname || !hostname[0]) {
        async_status.last_error = DNS_ERR_BAD_HOST;
        return false;
    }
    if (dns_cache_lookup(hostname, &cached)) {
        simd_zero(&async_status, sizeof(async_status));
        async_status.state = DNS_ASYNC_DONE;
        async_status.server_ip = dns_server;
        async_status.result_ip = cached;
        str_copy(async_status.hostname, sizeof(async_status.hostname), hostname);
        return true;
    }

    async_txid = gen_txid();
    async_src_port = gen_src_port();
    expected_txid = async_txid;
    got_response = false;
    resp_ip = 0;
    resp_ttl = 0;
    async_query_len = build_query(async_txid, hostname);
    if (async_query_len == 0) {
        async_status.last_error = DNS_ERR_BAD_HOST;
        return false;
    }

    if (async_callback_installed)
        net_set_udp_callback(prev_callback);
    prev_callback = net_swap_udp_callback(dns_udp_handler);
    async_callback_installed = true;

    simd_zero(&async_status, sizeof(async_status));
    async_status.state = DNS_ASYNC_RUNNING;
    async_status.server_ip = dns_server;
    str_copy(async_status.hostname, sizeof(async_status.hostname), hostname);
    (void)dns_async_send_query();
    return true;
}

void dns_poll(void)
{
    if (async_status.state != DNS_ASYNC_RUNNING)
        return;
    if (got_response) {
        async_status.result_ip = resp_ip;
        cache_insert(str_hash(async_status.hostname), resp_ip, resp_ttl);
        dns_async_finish(DNS_ASYNC_DONE, DNS_ERR_NONE);
        return;
    }
    if (timer_monotonic_ms() < async_deadline)
        return;
    if (async_status.attempts >= MAX_RETRIES) {
        dns_async_finish(DNS_ASYNC_FAILED, DNS_ERR_TIMEOUT);
        return;
    }
    (void)dns_async_send_query();
}

void dns_async_status(struct dns_async_status *out)
{
    if (!out) return;
    *out = async_status;
}

bool dns_resolve(const char *hostname, u32 *ip_out) {
    if (!dns_server) return false;

    /* Check cache */
    u32 hash = str_hash(hostname);
    u32 *cached_ip = cache_lookup(hash);
    if (cached_ip) {
        *ip_out = *cached_ip;
        return true;
    }

    u16 txid = gen_txid();
    u16 src_port = gen_src_port();
    expected_txid = txid;
    got_response = false;
    resp_ip = 0;
    resp_ttl = 0;

    u32 qlen = build_query(txid, hostname);
    if (qlen == 0) return false;

    /* Install our UDP handler temporarily, saving previous callback */
    prev_callback = net_swap_udp_callback(dns_udp_handler);

    for (u32 retry = 0; retry < MAX_RETRIES; retry++) {
        net_send_udp(dns_server, src_port, DNS_PORT, query_buf, (u16)qlen);

        /* Poll for response */
        u64 deadline = timer_monotonic_ms() + QUERY_TIMEOUT;
        while (timer_monotonic_ms() < deadline) {
            net_poll();
            if (got_response) {
                *ip_out = resp_ip;
                cache_insert(hash, resp_ip, resp_ttl);
                net_set_udp_callback(prev_callback);
                return true;
            }
            timer_delay_us(100);
        }
    }

    /* Restore previous UDP callback on failure */
    net_set_udp_callback(prev_callback);

    uart_puts("[dns] Resolve failed: ");
    uart_puts(hostname);
    uart_puts("\n");
    return false;
}
