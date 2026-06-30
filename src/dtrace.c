/*
 * dtrace.c - in-memory diagnostic trace ring. See include/dtrace.h.
 */

#include "dtrace.h"
#include "uart.h"
#include "simd.h"

#define DTRACE_RING_CAP   512U          /* records per core (power of two) */
#define DTRACE_RING_MASK  (DTRACE_RING_CAP - 1U)

struct dtrace_ring {
    volatile u64 head;                  /* monotonic write counter (one writer) */
    u8 _pad[56];
    struct dtrace_rec recs[DTRACE_RING_CAP];
} ALIGNED(64);

/* One owner ring per core; each core writes only its own slot. */
static struct dtrace_ring g_dtrace_rings[NUM_CORES] ALIGNED(64);

volatile u32 g_dtrace_active_mask;      /* 0 == disabled */
static volatile u32 g_dtrace_mask = DTRACE_CAT_ALL;
static volatile u32 g_dtrace_on;

static inline u64 dt_now(void)
{
    u64 c;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(c));
    return c;
}

static inline u32 cat_to_log2(u32 cat)
{
    /* cat is a single bit; return its index. */
    u32 i = 0;
    while (i < 31U && ((cat >> i) & 1U) == 0U)
        i++;
    return i;
}

void dtrace_init(void)
{
    simd_zero(g_dtrace_rings, sizeof(g_dtrace_rings));
    g_dtrace_mask = DTRACE_CAT_ALL;
    g_dtrace_on = 0;
    g_dtrace_active_mask = 0;
    dsb();
}

static void dtrace_recompute(void)
{
    g_dtrace_active_mask = g_dtrace_on ? g_dtrace_mask : 0U;
    dsb();
}

void dtrace_set_enabled(bool on)
{
    g_dtrace_on = on ? 1U : 0U;
    dtrace_recompute();
}

bool dtrace_enabled(void) { return g_dtrace_on != 0U; }

void dtrace_set_mask(u32 mask)
{
    g_dtrace_mask = mask ? mask : DTRACE_CAT_ALL;
    dtrace_recompute();
}

u32 dtrace_mask(void) { return g_dtrace_mask; }

void dtrace_clear(void)
{
    for (u32 c = 0; c < NUM_CORES; c++) {
        g_dtrace_rings[c].head = 0;
    }
    dsb();
}

void dtrace_emit(u32 cat, u16 ev, u64 a0, u64 a1, u64 a2, u64 a3)
{
    u32 c = core_id() & (NUM_CORES - 1U);
    struct dtrace_ring *r = &g_dtrace_rings[c];
    u64 h = r->head;
    struct dtrace_rec *rec = &r->recs[h & DTRACE_RING_MASK];
    rec->ts = dt_now();
    rec->core = (u8)c;
    rec->cat_log2 = (u8)cat_to_log2(cat);
    rec->ev = ev;
    rec->_pad = 0;
    rec->a0 = a0;
    rec->a1 = a1;
    rec->a2 = a2;
    rec->a3 = a3;
    /* Publish payload before advancing head so a cross-core dump reader never
     * sees a half-written record. */
    dmb_ishst();
    r->head = h + 1U;
}

/* ---- formatting helpers (bounded, no libc) ---- */

static void put(char *out, u32 *len, u32 max, const char *s)
{
    while (*s && *len < max)
        out[(*len)++] = *s++;
}

static void put_u64(char *out, u32 *len, u32 max, u64 v)
{
    char tmp[20];
    u32 n = 0;
    if (v == 0) { if (*len < max) out[(*len)++] = '0'; return; }
    while (v && n < sizeof(tmp)) { tmp[n++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (n) { if (*len < max) out[(*len)++] = tmp[--n]; else break; }
}

static void put_hex(char *out, u32 *len, u32 max, u64 v)
{
    static const char hx[] = "0123456789ABCDEF";
    put(out, len, max, "0x");
    bool started = false;
    for (i32 sh = 60; sh >= 0; sh -= 4) {
        u32 nib = (u32)((v >> sh) & 0xFU);
        if (nib || started || sh == 0) {
            started = true;
            if (*len < max) out[(*len)++] = hx[nib];
        }
    }
}

static const char *cat_name(u8 log2)
{
    switch (1U << log2) {
    case DTRACE_CAT_TCP:     return "TCP";
    case DTRACE_CAT_MAC:     return "MAC";
    case DTRACE_CAT_FIFO:    return "FIFO";
    case DTRACE_CAT_SCHED:   return "SCHED";
    case DTRACE_CAT_IRQ:     return "IRQ";
    case DTRACE_CAT_OTA:     return "OTA";
    case DTRACE_CAT_REACTOR: return "RCTR";
    default:                 return "?";
    }
}

static const char *ev_name(u8 log2, u16 ev)
{
    switch (1U << log2) {
    case DTRACE_CAT_TCP:
        switch (ev) {
        case DT_TCP_ACCEPT: return "accept"; case DT_TCP_ESTAB: return "estab";
        case DT_TCP_RX: return "rx"; case DT_TCP_TX: return "tx";
        case DT_TCP_ACK: return "ack"; case DT_TCP_WNDUPD: return "wndupd";
        case DT_TCP_CLOSE: return "close"; case DT_TCP_RESET: return "reset";
        } break;
    case DTRACE_CAT_MAC:
        switch (ev) {
        case DT_MAC_RX: return "rx"; case DT_MAC_TX: return "tx";
        case DT_MAC_TXRECLAIM: return "txreclaim"; case DT_MAC_RXRECOVER: return "rxrecover";
        case DT_MAC_TXRECOVER: return "txrecover";
        } break;
    case DTRACE_CAT_FIFO:
        switch (ev) { case DT_FIFO_PUSH: return "push"; case DT_FIFO_POP: return "pop"; } break;
    case DTRACE_CAT_SCHED:
        switch (ev) {
        case DT_SCHED_SWITCH: return "switch"; case DT_SCHED_PARK: return "park";
        case DT_SCHED_WAKE: return "wake"; case DT_SCHED_PREEMPT: return "preempt";
        } break;
    case DTRACE_CAT_IRQ:
        switch (ev) { case DT_IRQ_DISPATCH: return "dispatch"; } break;
    case DTRACE_CAT_OTA:
        switch (ev) {
        case DT_OTA_BEGIN: return "begin"; case DT_OTA_CHUNK: return "chunk";
        case DT_OTA_DRAIN_SPIN: return "drain"; case DT_OTA_WATCHDOG: return "watchdog";
        case DT_OTA_COMMIT: return "commit"; case DT_OTA_RESET: return "reset";
        case DT_OTA_WADV: return "wadv"; case DT_OTA_REACTOR_GAP: return "gap";
        } break;
    case DTRACE_CAT_REACTOR:
        switch (ev) {
        case DT_RX_PHASE_NET: return "net"; case DT_RX_PHASE_TCP: return "tcp";
        case DT_RX_PHASE_ADMIN: return "admin"; case DT_RX_PHASE_ECHO: return "echo";
        case DT_RX_PHASE_HTTP: return "http";
        case DT_RX_WAKE: return "wake";
        } break;
    }
    return "?";
}

/* Per-core read cursors for the 4-way merge. */
struct dt_cursor { u64 idx; u64 end; };

static bool cursor_peek(u32 core, struct dt_cursor *cu, struct dtrace_rec *out)
{
    if (cu->idx >= cu->end)
        return false;
    *out = g_dtrace_rings[core].recs[cu->idx & DTRACE_RING_MASK];
    return true;
}

static void format_rec(char *out, u32 *len, u32 max, const struct dtrace_rec *r, u64 t0)
{
    /* +<rel_ticks> c<core> CAT/ev a0 a1 a2 a3 */
    put(out, len, max, "+");
    put_u64(out, len, max, r->ts - t0);
    put(out, len, max, " c");
    put_u64(out, len, max, r->core);
    put(out, len, max, " ");
    put(out, len, max, cat_name(r->cat_log2));
    put(out, len, max, "/");
    put(out, len, max, ev_name(r->cat_log2, r->ev));
    put(out, len, max, " ");
    put_hex(out, len, max, r->a0);
    put(out, len, max, " ");
    put_hex(out, len, max, r->a1);
    put(out, len, max, " ");
    put_hex(out, len, max, r->a2);
    put(out, len, max, " ");
    put_hex(out, len, max, r->a3);
    put(out, len, max, "\n");
}

u32 dtrace_dump(char *out, u32 max, u32 max_records)
{
    u32 len = 0;
    struct dt_cursor cu[NUM_CORES];
    u64 oldest = ~0ULL;

    /* Each ring holds the most-recent min(head,CAP) records in ts order. */
    for (u32 c = 0; c < NUM_CORES; c++) {
        u64 h = g_dtrace_rings[c].head;
        u64 count = h < DTRACE_RING_CAP ? h : DTRACE_RING_CAP;
        cu[c].idx = h - count;
        cu[c].end = h;
        if (count) {
            struct dtrace_rec r;
            if (cursor_peek(c, &cu[c], &r) && r.ts < oldest)
                oldest = r.ts;
        }
    }
    if (oldest == ~0ULL)
        oldest = 0;

    if (max_records == 0 || max_records > NUM_CORES * DTRACE_RING_CAP)
        max_records = NUM_CORES * DTRACE_RING_CAP;

    for (u32 emitted = 0; emitted < max_records; emitted++) {
        /* 4-way merge: pick the oldest unconsumed record across all rings. */
        i32 best = -1;
        u64 best_ts = ~0ULL;
        struct dtrace_rec br;
        for (u32 c = 0; c < NUM_CORES; c++) {
            struct dtrace_rec r;
            if (cursor_peek(c, &cu[c], &r) && r.ts < best_ts) {
                best_ts = r.ts; best = (i32)c; br = r;
            }
        }
        if (best < 0)
            break;
        cu[best].idx++;
        if (len + 96U >= max)        /* leave headroom for one record */
            break;
        format_rec(out, &len, max, &br, oldest);
    }
    return len;
}

void dtrace_dump_uart(u32 max_records)
{
    struct dt_cursor cu[NUM_CORES];
    u64 oldest = ~0ULL;
    for (u32 c = 0; c < NUM_CORES; c++) {
        u64 h = g_dtrace_rings[c].head;
        u64 count = h < DTRACE_RING_CAP ? h : DTRACE_RING_CAP;
        cu[c].idx = h - count;
        cu[c].end = h;
        if (count) {
            struct dtrace_rec r;
            if (cursor_peek(c, &cu[c], &r) && r.ts < oldest) oldest = r.ts;
        }
    }
    if (oldest == ~0ULL) oldest = 0;
    if (max_records == 0 || max_records > NUM_CORES * DTRACE_RING_CAP)
        max_records = NUM_CORES * DTRACE_RING_CAP;

    for (u32 emitted = 0; emitted < max_records; emitted++) {
        i32 best = -1; u64 best_ts = ~0ULL; struct dtrace_rec br;
        for (u32 c = 0; c < NUM_CORES; c++) {
            struct dtrace_rec r;
            if (cursor_peek(c, &cu[c], &r) && r.ts < best_ts) { best_ts = r.ts; best = (i32)c; br = r; }
        }
        if (best < 0) break;
        cu[best].idx++;
        char line[128];
        u32 l = 0;
        format_rec(line, &l, sizeof(line) - 1U, &br, oldest);
        line[l] = 0;
        uart_puts(line);
    }
}

u32 dtrace_status(char *out, u32 max)
{
    u32 len = 0;
    put(out, &len, max, "dtrace ");
    put(out, &len, max, g_dtrace_on ? "ON" : "OFF");
    put(out, &len, max, " mask=");
    put_hex(out, &len, max, g_dtrace_mask);
    put(out, &len, max, " cap=");
    put_u64(out, &len, max, DTRACE_RING_CAP);
    put(out, &len, max, "/core counts=[");
    for (u32 c = 0; c < NUM_CORES; c++) {
        if (c) put(out, &len, max, ",");
        put_u64(out, &len, max, g_dtrace_rings[c].head);
    }
    put(out, &len, max, "]\n");
    return len;
}
