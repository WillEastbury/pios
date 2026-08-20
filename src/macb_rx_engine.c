/*
 * macb_rx_engine.c - Cadence GEM (RP1, BCM2712 / Raspberry Pi 5) RX descriptor
 * engine.
 *
 * The receive ring state machine for the MACB driver, kept in its own
 * translation unit so it can be reasoned about and fuzzed independently. The
 * driver (src/macb.c) owns the fixed DMA_NET ring/buffer layout and the
 * PHY/NCFGR bring-up and hands them to macb_rx_engine_init(); after that this
 * engine is the exclusive owner of the RX OWN bits, the software cursor,
 * RBQP/RBQPH, NCR.RE for RX restart, and the RSR BNA/OVR/HRESP fault bits.
 *
 * Design rationale
 * ----------------
 * 1. Memory model. The descriptor ring and RX buffers live in DMA_NET, which is
 *    identity-mapped Normal Non-Cacheable. Because the memory is non-cacheable
 *    there is nothing to flush or invalidate: this file issues NO cache
 *    maintenance of any kind. Correctness comes purely from ordering. All
 *    descriptor and payload accesses are volatile and CPU<->GEM ordering is
 *    enforced with FULL-SYSTEM AArch64 barriers: dmb sy for acquire/release
 *    ordering and dsb sy for publication/completion. DMA_NET is mapped Inner
 *    Shareable while the GEM is an external PCIe DMA master on RP1, so the
 *    integrated engine deliberately uses the widest (system) domain rather than
 *    mixing a barrier-domain experiment into the rewrite.
 *
 * 2. Ownership. word0 bit0 is OWN: 0 = GEM owns (may fill), 1 = software owns a
 *    completed descriptor. The producer (GEM) writes ctrl+payload before setting
 *    OWN; the consumer observes OWN, executes a DMA acquire, then reads
 *    ctrl+payload. On release the consumer finishes the copy and field reset,
 *    executes a DMA release, then publishes word0 last. Diagnostic snapshots of
 *    completed (OWN=1) descriptors observe the same acquire ordering.
 *
 * 3. Release is never a read-modify-write of word0. The exact word0 (low buffer
 *    address, plus WRAP only on the final descriptor, OWN clear) is rebuilt from
 *    the descriptor index and stored as the single final release store, so a
 *    torn/corrupt address word can never survive a release and a stray CPU store
 *    cannot partially overwrite a DMA ownership update.
 *
 * 4. Recovery is layered and fail-closed. recover_status classifies BNA/OVR/
 *    HRESP; HRESP rebuilds immediately, while a BNA/OVR with zero owned
 *    descriptors arms a latch-independent persistence candidate (active flag +
 *    rx_recv baseline + start time) and only rebuilds if the engine stays halted
 *    for >=20 ms, never for a stale BNA latch. recover_ordering_hole detects the
 *    "impossible hole" (cursor OWN=0 while >=4 later descriptors are OWN=1),
 *    requires it to persist >=20 ms, snapshots it, then rebuilds. recover_
 *    liveness only acts on a real wedge (proved RX progress, then 90 s of RX
 *    silence while TX advances). All destructive rebuilds share one primitive
 *    that disables NCR.RE, proves it disabled, republishes the whole ring while
 *    disabled, reprograms RBQP/RBQPH, clears RSR, re-enables RE, and resets the
 *    cursor. init likewise verifies RE is actually clear before writing the ring.
 *    Lifetime counters and the last captured hole image survive rebuilds.
 *
 * 5. Every scan/loop is bounded by ring_count or a fixed spin bound. RBQP is
 *    never treated as the software cursor; RX is managed strictly by OWN bits.
 */

#include "macb_rx_engine.h"
#include "mmio.h"
#include "timer.h"
#include "simd.h"
#include "pioscap.h"
#include "dtrace.h"

/* ======================================================================== */
/* Hardware constants (GEM register block; matches src/macb.c MACB_BASE).   */
/* ======================================================================== */

#define GEM_BASE      0x1F00100000ull

#define GEM_NCR       0x0000u
#define GEM_DMACFG    0x0010u
#define GEM_RBQP      0x0018u
#define GEM_RSR       0x0020u
#define GEM_RBQPH     0x04D4u
#define GEM_RXBDCTRL  0x04D0u
#define GEM_DCFG10    0x02A4u

#define GEM_NCR_RE    (1u << 2)

#define GEM_RSR_BNA   (1u << 0)
#define GEM_RSR_REC   (1u << 1)
#define GEM_RSR_OVR   (1u << 2)
#define GEM_RSR_HRESP (1u << 3)
#define GEM_RSR_ALL   0x0000000Fu
#define GEM_RSR_FAULT (GEM_RSR_BNA | GEM_RSR_OVR | GEM_RSR_HRESP)

#define GEM_DMACFG_RXEXT  (1u << 28)
#define GEM_DMACFG_TXEXT  (1u << 29)
#define GEM_DMACFG_ADDR64 (1u << 30)

/* word0: [31:2] low buffer address, bit1 WRAP, bit0 OWN/USED. */
#define RX_W0_OWN        0x00000001u
#define RX_W0_WRAP       0x00000002u
#define RX_W0_ADDR_MASK  0xFFFFFFFCu

/* word1: [12:0] length, bit14 SOF, bit15 EOF, [23:22] checksum status. */
#define RX_CTRL_LEN_MASK   0x00001FFFu
#define RX_CTRL_SOF        0x00004000u
#define RX_CTRL_EOF        0x00008000u
#define RX_CTRL_CSUM_SHIFT 22u
#define RX_CTRL_CSUM_MASK  0x3u
#define RX_CSUM_L4         0x2u   /* trusted result must include bit value 2 */


/* ======================================================================== */
/* Tunable, spec-derived thresholds                                         */
/* ======================================================================== */

#define RX_MIN_BUFFER_SIZE     1518u    /* one full standard Ethernet frame + FCS */
#define RX_HOLE_MIN_LATER      4u       /* >=4 later OWN=1 => impossible hole      */
#define RX_HOLE_HOLD_MS        20u      /* hole must persist this long             */
#define RX_STATUS_CONFIRM_MS   20u      /* stall must persist this long            */
#define RX_LIVE_WEDGE_MS       90000u   /* RX silence that (with demand) is a wedge*/
#define RX_LIVE_DISARM_MS      30000u   /* quiet RX+TX disarms the detector        */
#define RX_LIVE_IDLE_MS        15000u   /* informational idle, once per window     */
#define RX_LIVE_BACKOFF_MIN_MS 30000u
#define RX_LIVE_BACKOFF_MAX_MS 180000u
#define RX_RE_DISABLE_SPINS    100000u  /* bounded poll proving NCR.RE cleared     */

/* ======================================================================== */
/* Barriers. No cache ops.                                                  */
/*                                                                          */
/* Full-system domain (dmb sy / dsb sy): DMA_NET is mapped Inner Shareable  */
/* while the GEM is an external PCIe DMA master on RP1, so the integrated    */
/* engine orders against the widest domain rather than experimenting with   */
/* inner/outer scoping on the live receive path.                            */
/* ======================================================================== */

static inline void rx_dma_acquire(void)
{
    /* After observing OWN, order it before ctrl/payload reads. */
    __asm__ volatile("dmb sy" ::: "memory");
}

static inline void rx_dma_release(void)
{
    /* Order prior payload reads and field resets before the OWN-clear store. */
    __asm__ volatile("dmb sy" ::: "memory");
}

static inline void rx_dma_publish(void)
{
    /* Complete descriptor writes before handing the ring to the engine. */
    __asm__ volatile("dsb sy" ::: "memory");
}

/* ======================================================================== */
/* Private engine state (all owned by the single software writer, core 0).  */
/* Isolated on its own cache line: the diagnostics counters and cursor must  */
/* not share a line with unrelated mutable driver globals.                   */
/* ======================================================================== */

struct macb_rx_state {
    bool initialized;

    volatile u32 *ring;   /* descriptor ring base, viewed as u32 words */
    u8 *buffers;
    u32 ring_count;
    u32 buffer_size;
    u64 ring_dma;
    u64 buffers_dma;
    u32 trailing_bytes;
    bool checksum_enabled;

    u32 cursor;           /* software RX cursor (reported as rx_idx) */

    /* Lifetime diagnostic counters (preserved across rebuilds). */
    u32 rx_recv;
    u32 rx_recover;
    u32 rx_hole_recover;
    u32 rx_live_recover;
    u32 rx_wedge;
    u32 rx_idle;

    /* Status-recovery persistence candidate (independent of the RSR latch,
     * which reads clean once BNA/OVR is written-one-to-clear even while the
     * engine stays halted). Armed on a fresh BNA/OVR with zero owned. */
    bool status_cand_active;
    u32  status_cand_rx_recv;   /* rx_recv baseline captured when the candidate armed */
    u64  status_cand_since_ms;  /* arm time */

    /* Ordering-hole candidate tracking + last captured image. */
    bool hole_candidate_valid;
    u32  hole_candidate_idx;
    u64  hole_candidate_since_ms;
    u32  hole_sequence;
    struct macb_rx_hole_image hole;

    /* Liveness detector. */
    bool live_armed;
    u32  live_rx_mark;
    u64  live_rx_mark_ms;
    u32  live_tx_mark;
    bool live_idle_counted;
    u64  live_last_recover_ms;
    u64  live_backoff_ms;

    /* Instant of the most recent destructive rebuild from any path. */
    u64  last_recover_ms;
};

static struct macb_rx_state g ALIGNED(64);

/* ======================================================================== */
/* Small helpers                                                            */
/* ======================================================================== */

static inline u32 gem_rd(u32 off)          { return mmio_read(GEM_BASE + off); }
static inline void gem_wr(u32 off, u32 v)  { mmio_write(GEM_BASE + off, v); }

static inline volatile u32 *rx_desc(u32 i)
{
    return g.ring + (usize)i * 4u; /* 4 u32 words == 16-byte stride */
}

static inline u32 rx_ring_low(void)
{
    return (u32)g.ring_dma;
}

/* Exact word0 for descriptor i: low buffer address, WRAP only on the last
 * descriptor, OWN clear. This is the canonical value used for both publish and
 * release, so no descriptor is ever released via read-modify-write. */
static u32 rx_expected_word0(u32 i)
{
    u32 addr = (u32)(g.buffers_dma + (u64)i * (u64)g.buffer_size);
    u32 w0 = addr & RX_W0_ADDR_MASK;
    if (i == g.ring_count - 1u)
        w0 |= RX_W0_WRAP;
    return w0; /* OWN clear */
}

static void rx_capture_desc(struct macb_rx_desc_image *out, u32 index)
{
    volatile u32 *d = rx_desc(index);
    out->index = index;
    out->addr = d[0];
    rx_dma_acquire();
    out->ctrl = d[1];
    out->addr_hi = d[2];
    out->word3 = d[3];
}

/* Deterministic single-descriptor release: reset fields, release barrier, then
 * publish the rebuilt word0 (OWN clear) as the final store. */
static void rx_release_descriptor(u32 i)
{
    volatile u32 *d = rx_desc(i);
    d[1] = 0u;
    d[2] = (u32)(g.buffers_dma >> 32);
    d[3] = 0u;
    rx_dma_release();
    d[0] = rx_expected_word0(i);
}

static void rx_advance(void)
{
    u32 next = g.cursor + 1u;
    if (next >= g.ring_count)
        next = 0u;
    g.cursor = next;
}

/* Publish the entire ring (caller guarantees NCR.RE is disabled): all non-owner
 * fields first, one release barrier, then every word0, then a completion barrier. */
static void rx_publish_ring(void)
{
    u32 n = g.ring_count;
    u32 i;

    for (i = 0; i < n; i++) {
        volatile u32 *d = rx_desc(i);
        d[1] = 0u;
        d[2] = (u32)(g.buffers_dma >> 32);
        d[3] = 0u;
    }
    rx_dma_publish();
    for (i = 0; i < n; i++)
        rx_desc(i)[0] = rx_expected_word0(i);
    rx_dma_publish();
}

/* ---- One bounded ownership scan describing ring topology from the cursor. */
struct rx_scan {
    u32 total_owned;
    u32 contig_owned;
    u32 owned_after_gap;
    u32 first_owned_distance;   /* distance to first OWN after the first gap; == ring_count when none */
    bool cursor_owned;
};

static void rx_scan_ring(struct rx_scan *r)
{
    u32 n = g.ring_count;
    u32 idx = g.cursor;
    bool gap_seen = false;
    bool contig = true;
    bool first_after = false;
    u32 k;

    r->total_owned = 0u;
    r->contig_owned = 0u;
    r->owned_after_gap = 0u;
    r->first_owned_distance = n;   /* sentinel: no owned descriptor after a gap */
    r->cursor_owned = false;

    for (k = 0; k < n; k++) {
        u32 w0 = rx_desc(idx)[0];
        bool owned = (w0 & RX_W0_OWN) != 0u;

        if (k == 0u)
            r->cursor_owned = owned;
        if (owned)
            r->total_owned++;
        if (contig) {
            if (owned)
                r->contig_owned++;
            else
                contig = false;
        }
        if (!owned) {
            gap_seen = true;
        } else if (gap_seen) {
            r->owned_after_gap++;
            if (!first_after) {
                first_after = true;
                r->first_owned_distance = k;
            }
        }

        idx++;
        if (idx >= n)
            idx = 0u;
    }
}

/* ======================================================================== */
/* Destructive rebuild primitive (shared by all recovery paths)             */
/* ======================================================================== */

static bool rx_re_wait_disabled(void)
{
    u32 s;
    for (s = 0; s < RX_RE_DISABLE_SPINS; s++) {
        if ((gem_rd(GEM_NCR) & GEM_NCR_RE) == 0u)
            return true;
    }
    return false;
}

/* Returns true only if the ring was safely rebuilt with reception re-enabled.
 * Never rewrites the ring while NCR.RE is still set (fail closed). Lifetime
 * counters and the stored hole image are preserved. */
static bool rx_destructive_rebuild(void)
{
    bool disabled;

    gem_wr(GEM_NCR, gem_rd(GEM_NCR) & ~GEM_NCR_RE);
    rx_dma_publish();
    disabled = rx_re_wait_disabled();
    if (!disabled) {
        gem_wr(GEM_NCR, gem_rd(GEM_NCR) & ~GEM_NCR_RE);
        rx_dma_publish();
        disabled = rx_re_wait_disabled();
    }
    if (!disabled) {
        DTRACE(DTRACE_CAT_MAC, DT_MAC_REDISABLEFAIL, gem_rd(GEM_NCR), 0u, 0u, 0u);
        return false; /* refuse to rewrite a live ring */
    }

    rx_publish_ring();

    gem_wr(GEM_RBQP, rx_ring_low());
    gem_wr(GEM_RBQPH, (u32)(g.ring_dma >> 32));
    rx_dma_publish();

    gem_wr(GEM_RSR, GEM_RSR_ALL);            /* write-one-to-clear latched bits */
    gem_wr(GEM_NCR, gem_rd(GEM_NCR) | GEM_NCR_RE);
    rx_dma_publish();

    g.cursor = 0u;
    g.status_cand_active = false;
    g.hole_candidate_valid = false;
    g.last_recover_ms = timer_monotonic_ms();
    return true;
}

/* ======================================================================== */
/* Init                                                                     */
/* ======================================================================== */

bool macb_rx_engine_init(const struct macb_rx_config *config)
{
    u32 dmacfg;

    if (!config || !config->ring || !config->buffers)
        return false;
    if (config->ring_count < 2u)
        return false;
    if (config->buffer_size < RX_MIN_BUFFER_SIZE)
        return false;
    if (config->ring_dma == 0ULL || config->buffers_dma == 0ULL)
        return false;
    if (((usize)config->ring & 63u) != 0u)          /* 64-byte aligned ring     */
        return false;
    if (((usize)config->buffers & 3u) != 0u)        /* 4-byte aligned base       */
        return false;
    if ((config->buffer_size & 3u) != 0u)           /* keep every buffer 4-aligned */
        return false;

    /* Arithmetic overflow guards. */
    if (config->ring_count > (0xFFFFFFFFu / 16u))               /* ring byte span  */
        return false;
    if (config->ring_count > (0xFFFFFFFFu / config->buffer_size)) /* buffer span    */
        return false;
    {
        u64 base_lo = config->buffers_dma & 0xFFFFFFFFull;
        u64 span = (u64)config->ring_count * (u64)config->buffer_size;
        if (base_lo + span > 0x100000000ull)        /* word0 holds low 32 bits    */
            return false;
    }
    if ((config->ring_dma >> 32) != (config->buffers_dma >> 32))
        return false;

    /* Confirm the live descriptor mode: 64-bit addressing on, extended (6-word)
     * timestamp descriptors off. Otherwise the 4-word layout is invalid; fail. */
    dmacfg = gem_rd(GEM_DMACFG);
    if ((dmacfg & GEM_DMACFG_ADDR64) == 0u)
        return false;
    if (dmacfg & (GEM_DMACFG_RXEXT | GEM_DMACFG_TXEXT))
        return false;

    /* Do not trust the caller's "reception disabled" claim: verify NCR.RE is
     * clear before writing the ring so we never rewrite a live ring. If it is
     * still set, clear it and prove it cleared; if it cannot be proven disabled,
     * fail closed. */
    if (gem_rd(GEM_NCR) & GEM_NCR_RE) {
        gem_wr(GEM_NCR, gem_rd(GEM_NCR) & ~GEM_NCR_RE);
        rx_dma_publish();
        if (!rx_re_wait_disabled())
            return false;
    }

    /* Commit configuration. */
    g.ring = (volatile u32 *)config->ring;
    g.buffers = config->buffers;
    g.ring_count = config->ring_count;
    g.buffer_size = config->buffer_size;
    g.ring_dma = config->ring_dma;
    g.buffers_dma = config->buffers_dma;
    g.trailing_bytes = config->trailing_bytes;
    g.checksum_enabled = config->checksum_enabled;

    /* Reset all private counters, cursors, candidates, liveness and hole image. */
    g.cursor = 0u;
    g.rx_recv = 0u;
    g.rx_recover = 0u;
    g.rx_hole_recover = 0u;
    g.rx_live_recover = 0u;
    g.rx_wedge = 0u;
    g.rx_idle = 0u;
    g.status_cand_active = false;
    g.status_cand_rx_recv = 0u;
    g.status_cand_since_ms = 0u;
    g.hole_candidate_valid = false;
    g.hole_candidate_idx = 0u;
    g.hole_candidate_since_ms = 0u;
    g.hole_sequence = 0u;
    g.hole = (struct macb_rx_hole_image){0};
    g.live_armed = false;
    g.live_rx_mark = 0u;
    g.live_rx_mark_ms = 0u;
    g.live_tx_mark = 0u;
    g.live_idle_counted = false;
    g.live_last_recover_ms = 0u;
    g.live_backoff_ms = 0u;
    g.last_recover_ms = 0u;

    /* Reception is proven disabled above. Publish the whole ring first. */
    rx_publish_ring();

    /* Program the queue pointer while reception is disabled. */
    gem_wr(GEM_RBQP, rx_ring_low());
    gem_wr(GEM_RBQPH, (u32)(g.ring_dma >> 32));
    rx_dma_publish();

    /* Clear any latched status, then enable reception. */
    gem_wr(GEM_RSR, GEM_RSR_ALL);
    gem_wr(GEM_NCR, gem_rd(GEM_NCR) | GEM_NCR_RE);
    rx_dma_publish();

    g.initialized = true;
    return true;
}

/* ======================================================================== */
/* Receive                                                                  */
/* ======================================================================== */

bool macb_rx_engine_recv(u8 *frame, u32 frame_capacity, u32 *frame_length,
                         bool *checksum_trusted)
{
    u32 i;
    volatile u32 *d;
    u32 w0, ctrl, length;
    bool sof, eof, ok;

    if (!g.initialized)
        return false;
    if (!frame || !frame_length || !checksum_trusted)
        return false;
    if (frame_capacity == 0u)
        return false;

    i = g.cursor;
    d = rx_desc(i);
    w0 = d[0];
    if ((w0 & RX_W0_OWN) == 0u)
        return false;               /* GEM still owns it; do not advance */

    rx_dma_acquire();               /* acquire before reading ctrl/payload */
    ctrl = d[1];

    length = ctrl & RX_CTRL_LEN_MASK;
    sof = (ctrl & RX_CTRL_SOF) != 0u;
    eof = (ctrl & RX_CTRL_EOF) != 0u;

    ok = sof && eof
         && (length > 0u)
         && (length <= g.buffer_size)
         && (length <= frame_capacity);

    if (ok) {
        const u8 *buf = g.buffers + (usize)i * (usize)g.buffer_size;
        u32 csum = (ctrl >> RX_CTRL_CSUM_SHIFT) & RX_CTRL_CSUM_MASK;

        simd_memcpy(frame, buf, (usize)length);
        *frame_length = length;
        *checksum_trusted = g.checksum_enabled && ((csum & RX_CSUM_L4) != 0u);

        rx_release_descriptor(i);
        rx_advance();
        g.rx_recv++;
        return true;
    }

    /* Malformed completion: do not copy, but release the one descriptor so it
     * can never block the ordered ring, then advance. */
    *frame_length = 0u;
    *checksum_trusted = false;
    rx_release_descriptor(i);
    rx_advance();
    DTRACE(DTRACE_CAT_MAC, DT_MAC_RXMALFORMED, i, ctrl, length, 0u);
    return false;
}

/* ======================================================================== */
/* Status recovery (exclusive owner of BNA/OVR/HRESP)                       */
/* ======================================================================== */

bool macb_rx_engine_recover_status(void)
{
    u32 rsr, latched;
    struct rx_scan s;
    u64 now;

    if (!g.initialized)
        return false;

    rsr = gem_rd(GEM_RSR);
    latched = rsr & GEM_RSR_FAULT;

    /* HRESP / bus failure -> immediate, fail-closed destructive rebuild. */
    if (latched & GEM_RSR_HRESP) {
        DTRACE(DTRACE_CAT_MAC, DT_MAC_RXHRESP, rsr, g.cursor, g.rx_recover, 0u);
        g.status_cand_active = false;
        if (rx_destructive_rebuild()) {   /* also resets candidate state */
            g.rx_recover++;
            pioscap_notify_event("rx-hresp-recover");
            return true;
        }
        return false;
    }

    /* Evaluate an armed stall candidate INDEPENDENTLY of the current latch.
     * Writing BNA/OVR to clear makes RSR read back clean even while the engine
     * stays halted, so the candidate, not the latch, drives the decision. */
    if (g.status_cand_active) {
        /* Cancel if RX made progress since the candidate armed: engine is live. */
        if (g.rx_recv != g.status_cand_rx_recv) {
            g.status_cand_active = false;
        } else {
            rx_scan_ring(&s);
            /* Cancel if any descriptor is now software-owned: the normal drain
             * or the ordering-hole detector owns that case. Clear a fresh latch. */
            if (s.total_owned > 0u) {
                g.status_cand_active = false;
                if (latched)
                    gem_wr(GEM_RSR, latched);
                return false;
            }
            /* Still zero owned and no RX progress: a genuinely halted engine. */
            now = timer_monotonic_ms();
            if ((now - g.status_cand_since_ms) >= RX_STATUS_CONFIRM_MS) {
                DTRACE(DTRACE_CAT_MAC, DT_MAC_RXRECOVER,
                       g.rx_recover + 1u, rsr, g.cursor, g.status_cand_rx_recv);
                DTRACE(DTRACE_CAT_MAC, DT_MAC_RXRECOVER_PATTERN,
                       s.contig_owned, s.owned_after_gap, g.cursor,
                       s.first_owned_distance);
                if (rx_destructive_rebuild()) {   /* resets status_cand_active */
                    g.rx_recover++;
                    pioscap_notify_event("rx-overrun-recover");
                    return true;
                }
                return false;   /* RE stuck: keep candidate, retry next call */
            }
            /* Not confirmed yet: keep the candidate; clear any fresh latch. */
            if (latched)
                gem_wr(GEM_RSR, latched);
            return false;
        }
    }

    /* No active candidate. Cheap common path when nothing is latched. */
    if (latched == 0u)
        return false;

    /* BNA/OVR latched (no HRESP). Classify by ownership distribution. */
    rx_scan_ring(&s);

    /* Software-owned descriptors exist (a frame to drain, or a ring exhausted by
     * backpressure). Clear the latched status so the GEM resumes once
     * descriptors free, and let the drain / ordering-hole detector handle it. */
    if (s.total_owned > 0u) {
        gem_wr(GEM_RSR, latched);
        return false;
    }

    /* Fresh BNA/OVR with zero software-owned descriptors. Do NOT equate a stale
     * bit with "destroy the ring": clear the latch and arm a persistence
     * candidate keyed on rx_recv and time, evaluated on subsequent calls above. */
    gem_wr(GEM_RSR, latched);
    g.status_cand_active = true;
    g.status_cand_rx_recv = g.rx_recv;
    g.status_cand_since_ms = timer_monotonic_ms();
    return false;
}

/* ======================================================================== */
/* Ordering-hole recovery                                                   */
/* ======================================================================== */

static void rx_capture_hole(u32 stuck)
{
    struct macb_rx_hole_image *h = &g.hole;
    u32 idx, k, n;

    *h = (struct macb_rx_hole_image){0};
    h->valid = true;
    h->sequence = ++g.hole_sequence;
    h->stuck_idx = stuck;
    h->expected_addr = rx_expected_word0(stuck);
    h->rbqp = gem_rd(GEM_RBQP);
    h->rbqph = gem_rd(GEM_RBQPH);
    h->rsr = gem_rd(GEM_RSR);
    h->ncr = gem_rd(GEM_NCR);
    h->dmacfg = gem_rd(GEM_DMACFG);
    h->dcfg10 = gem_rd(GEM_DCFG10);
    h->rxbdctrl = gem_rd(GEM_RXBDCTRL);
    u32 prefetch_field = (h->dcfg10 >> 8) & 0xFU;
    h->prefetch_descs = prefetch_field ? (2U << (prefetch_field - 1U)) : 0U;
    h->trailing_bytes = g.trailing_bytes;

    rx_capture_desc(&h->stuck, stuck);

    n = 0u;
    idx = stuck + 1u;
    if (idx >= g.ring_count)
        idx = 0u;
    for (k = 1u; k < g.ring_count && n < MACB_RX_CAPTURE_OWNED; k++) {
        volatile u32 *e = rx_desc(idx);
        u32 w0 = e[0];
        if (w0 & RX_W0_OWN) {
            rx_dma_acquire();   /* OWN observed: acquire before reading payload words */
            h->later_owned[n].index = idx;
            h->later_owned[n].addr = w0;
            h->later_owned[n].ctrl = e[1];
            h->later_owned[n].addr_hi = e[2];
            h->later_owned[n].word3 = e[3];
            n++;
        }

        idx++;
        if (idx >= g.ring_count)
            idx = 0u;
    }
    h->later_owned_count = n;
}

static void rx_probe_stopped_cache_line(u32 stuck)
{
    struct macb_rx_hole_image *h = &g.hole;
    h->cache_probe_flags = 0;

    gem_wr(GEM_NCR, gem_rd(GEM_NCR) & ~GEM_NCR_RE);
    rx_dma_publish();
    if (!rx_re_wait_disabled())
        return;

    h->cache_probe_flags |= MACB_RX_CACHE_PROBE_STOPPED;
    h->rbqp_stopped = gem_rd(GEM_RBQP);
    rx_capture_desc(&h->stopped, stuck);
    if (h->stopped.addr & RX_W0_OWN)
        h->cache_probe_flags |= MACB_RX_CACHE_PROBE_OWN_BEFORE_IVAC;

    usize line = (usize)rx_desc(stuck) & ~(usize)63U;
    h->cache_line = (u32)line;
    __asm__ volatile("dc ivac, %0" :: "r"(line) : "memory");
    __asm__ volatile("dsb sy\nisb" ::: "memory");
    rx_capture_desc(&h->after_ivac, stuck);

    if (h->stopped.addr != h->after_ivac.addr ||
        h->stopped.ctrl != h->after_ivac.ctrl ||
        h->stopped.addr_hi != h->after_ivac.addr_hi ||
        h->stopped.word3 != h->after_ivac.word3)
        h->cache_probe_flags |= MACB_RX_CACHE_PROBE_WORDS_CHANGED;
    if (h->after_ivac.addr & RX_W0_OWN)
        h->cache_probe_flags |= MACB_RX_CACHE_PROBE_OWN_AFTER_IVAC;
}

bool macb_rx_engine_recover_ordering_hole(void)
{
    u32 cur, w0;
    struct rx_scan s;
    u64 now;

    if (!g.initialized)
        return false;

    cur = g.cursor;
    w0 = rx_desc(cur)[0];
    if ((w0 & RX_W0_OWN) != 0u) {
        g.hole_candidate_valid = false; /* completed frame, not a hole */
        return false;
    }

    rx_scan_ring(&s);

    /* Impossible hole requires at least four later OWN=1 descriptors. */
    if (s.owned_after_gap < RX_HOLE_MIN_LATER) {
        g.hole_candidate_valid = false;
        return false;
    }

    /* Revalidate the current descriptor after the scan. */
    w0 = rx_desc(cur)[0];
    if ((w0 & RX_W0_OWN) != 0u) {
        g.hole_candidate_valid = false;
        return false;
    }

    /* Require the same cursor to remain an impossible hole for >=20 ms. */
    now = timer_monotonic_ms();
    if (!g.hole_candidate_valid || g.hole_candidate_idx != cur) {
        g.hole_candidate_valid = true;
        g.hole_candidate_idx = cur;
        g.hole_candidate_since_ms = now;
        return false;
    }
    if ((now - g.hole_candidate_since_ms) < RX_HOLE_HOLD_MS)
        return false;

    /* Persist the snapshot immediately before recovery, then rebuild. */
    rx_capture_hole(cur);
    rx_probe_stopped_cache_line(cur);
    DTRACE(DTRACE_CAT_MAC, DT_MAC_RXHOLERECOVER, cur, s.owned_after_gap,
           s.first_owned_distance, g.rx_hole_recover);

    /* The bare invalidate deliberately discards any hidden CPU cache line so
     * the following read observes PoC. Never resume this probed ring: if such
     * a line was dirty, sibling descriptor releases may have been discarded.
     * Rebuild every descriptor while RX remains stopped, preserving the
     * before/after image as evidence. */
    if (rx_destructive_rebuild()) {
        g.rx_hole_recover++;
        pioscap_notify_event("rx-descriptor-hole");
        g.hole_candidate_valid = false;
        return true;
    }
    return false; /* RE stuck: keep candidate and retry next call */
}

/* ======================================================================== */
/* Liveness recovery                                                        */
/* ======================================================================== */

bool macb_rx_engine_recover_liveness(u64 now_ms, u32 tx_progress_counter)
{
    u32 rx;
    u64 silence;
    bool tx_progressed;

    if (!g.initialized)
        return false;

    rx = g.rx_recv;

    /* RX progressed since last observation: (re)arm and restart the window. */
    if (rx != g.live_rx_mark) {
        g.live_armed = true;
        g.live_rx_mark = rx;
        g.live_rx_mark_ms = now_ms;
        g.live_tx_mark = tx_progress_counter;
        g.live_idle_counted = false;
        return false;
    }

    /* No wedge semantics until RX has proved progress at least once. */
    if (!g.live_armed)
        return false;

    /* A status/hole recovery during this window counts as "already handled":
     * rebase the window (and TX baseline) to the recovery instant exactly once. */
    if (g.last_recover_ms > g.live_rx_mark_ms) {
        g.live_rx_mark_ms = g.last_recover_ms;
        g.live_tx_mark = tx_progress_counter;
        g.live_idle_counted = false;
    }

    silence = (now_ms > g.live_rx_mark_ms) ? (now_ms - g.live_rx_mark_ms) : 0u;
    tx_progressed = (tx_progress_counter != g.live_tx_mark);

    /* Informational 15 s idle, once per window. */
    if (silence >= RX_LIVE_IDLE_MS && !g.live_idle_counted) {
        g.rx_idle++;
        g.live_idle_counted = true;
    }

    /* Fully quiet RX+TX for 30 s is an ordinary quiet period: disarm. */
    if (!tx_progressed && silence >= RX_LIVE_DISARM_MS) {
        g.live_armed = false;
        g.live_rx_mark_ms = now_ms;
        g.live_tx_mark = tx_progress_counter;
        g.live_idle_counted = false;
        return false;
    }

    /* Genuine wedge: armed, 90 s of RX silence, and TX advanced (unmet demand). */
    if (silence >= RX_LIVE_WEDGE_MS && tx_progressed) {
        bool ok;
        u64 next;

        /* Bounded recovery backoff, up to 180 s. */
        if (g.live_last_recover_ms != 0u &&
            (now_ms - g.live_last_recover_ms) < g.live_backoff_ms)
            return false;

        DTRACE(DTRACE_CAT_MAC, DT_MAC_RXLIVERECOVER, g.rx_wedge + 1u,
               (u32)silence, tx_progress_counter, g.rx_recv);
        g.rx_wedge++;
        ok = rx_destructive_rebuild();
        if (ok) {
            g.rx_live_recover++;
            pioscap_notify_event("rx-liveness-wedge");
        }

        next = (g.live_backoff_ms == 0u) ? RX_LIVE_BACKOFF_MIN_MS
                                         : (g.live_backoff_ms * 2u);
        if (next > RX_LIVE_BACKOFF_MAX_MS)
            next = RX_LIVE_BACKOFF_MAX_MS;
        g.live_backoff_ms = next;
        g.live_last_recover_ms = now_ms;

        /* A successful rebuild starts a fresh liveness window. If RE could not
         * be stopped, keep the detector armed so it retries after the bounded
         * backoff instead of requiring a wedged RX engine to recover itself. */
        if (ok) {
            g.live_armed = false;
            g.live_rx_mark_ms = now_ms;
            g.live_tx_mark = tx_progress_counter;
            g.live_idle_counted = false;
        }
        return ok;
    }

    return false;
}

/* ======================================================================== */
/* Diagnostics                                                              */
/* ======================================================================== */

void macb_rx_engine_diag(struct macb_rx_diag *out)
{
    struct rx_scan s;

    if (!out)
        return;

    *out = (struct macb_rx_diag){0};
    if (!g.initialized)
        return;

    rx_scan_ring(&s); /* read-only: never mutates ring or hardware */

    out->rx_idx = g.cursor;
    out->rx_owned = s.total_owned;
    out->rx_contig_owned = s.contig_owned;
    out->rx_owned_after_gap = s.owned_after_gap;
    out->rx_first_owned_distance = s.first_owned_distance;
    out->rx_recv = g.rx_recv;
    out->rx_recover = g.rx_recover;
    out->rx_hole_recover = g.rx_hole_recover;
    out->rx_live_recover = g.rx_live_recover;
    out->rx_wedge = g.rx_wedge;
    out->rx_idle = g.rx_idle;
    out->rsr = gem_rd(GEM_RSR);
    out->ncr = gem_rd(GEM_NCR);
    out->rbqp = gem_rd(GEM_RBQP);
    out->rbqph = gem_rd(GEM_RBQPH);
}

bool macb_rx_engine_last_hole(struct macb_rx_hole_image *out)
{
    if (!out)
        return false;
    if (!g.hole.valid)
        return false;      /* no capture yet */
    *out = g.hole;         /* preserved across ring recovery */
    return true;
}

/* ======================================================================== */
/* Runtime offload state                                                    */
/* ======================================================================== */

void macb_rx_engine_set_checksum_enabled(bool enable)
{
    /* recv() gates *checksum_trusted on this flag; keep it consistent with the
     * driver's NCFGR.RXCOEN, which macb_set_rx_checksum_offload() can toggle at
     * runtime. Ownership is core 0 only, so a plain store is sufficient. */
    g.checksum_enabled = enable;
}
