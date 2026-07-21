#pragma once
#include "types.h"
#include "core.h"

/*
 * dtrace - in-memory diagnostic trace ring.
 *
 * A per-core, lock-free ring of fixed 64-byte records that captures the
 * lifecycle of network packets, FIFO traffic, scheduler switches and IRQs as
 * a wireshark-with-procmon style timeline, buffered in RAM and dumped on
 * demand over HTTP or UART. Designed to add near-zero cost when disabled and
 * to never perturb scheduling when enabled.
 *
 * Invariants (see .github/copilot-instructions.md):
 *  - One writer per ring: each core only ever writes g_dtrace_rings[core_id].
 *    core 0 reads every ring at dump time (a 4-way merge of per-core streams,
 *    each already in monotonic-timestamp order).
 *  - Each ring is an ALIGNED(64) owner record with 64-byte stride.
 *  - No dynamic string formatting at emit: records store raw u64 args; names
 *    are decoded only when a dump is formatted.
 *  - The enable/mask gate is a single volatile word; emit is a load+and+branch.
 *    Counters are diagnostics, not synchronization.
 */

/* ---- Categories (bitmask) ---- */
#define DTRACE_CAT_TCP      (1U << 0)
#define DTRACE_CAT_MAC      (1U << 1)
#define DTRACE_CAT_FIFO     (1U << 2)
#define DTRACE_CAT_SCHED    (1U << 3)
#define DTRACE_CAT_IRQ      (1U << 4)
#define DTRACE_CAT_OTA      (1U << 5)
#define DTRACE_CAT_REACTOR  (1U << 6)
#define DTRACE_CAT_ALL      0xFFFFFFFFU

/* ---- Event ids (per category; kept compact) ---- */
/* TCP */
#define DT_TCP_ACCEPT       1
#define DT_TCP_ESTAB        2
#define DT_TCP_RX           3   /* a0=conn a1=seq a2=len */
#define DT_TCP_TX           4   /* a0=conn a1=seq a2=len */
#define DT_TCP_ACK          5   /* a0=conn a1=ack a2=wnd */
#define DT_TCP_WNDUPD       6   /* a0=conn a1=wnd */
#define DT_TCP_CLOSE        7
#define DT_TCP_RESET        8
/* MAC */
#define DT_MAC_RX           1   /* a0=len a1=rx_recv */
#define DT_MAC_TX           2   /* a0=len a1=tx_send */
#define DT_MAC_TXRECLAIM    3
#define DT_MAC_RXRECOVER    4
#define DT_MAC_TXRECOVER    5
#define DT_MAC_RXLIVERECOVER 6  /* a0=rx_wedge_count a1=idle_ms a2=streak a3=tx_send_count */
#define DT_MAC_RXRECOVER_PATTERN 7 /* pre-rebuild OWN-bit snapshot; a0=contig_owned run
                                    * starting at rx_idx (NUM_RX = genuinely saturated),
                                    * a1=owned_after_gap (nonzero = hole/anomaly: a NOT-owned
                                    * descriptor followed by more owned ones -- points at a
                                    * false/stale overrun rather than real traffic volume),
                                    * a2=rx_idx, a3=first_owned_distance */
#define DT_MAC_RXHOLERECOVER 8 /* ordered-ring hole: a0=stuck rx_idx
                                * a1=owned_after_gap a2=first distance a3=count */
#define DT_MAC_RXMALFORMED   9 /* malformed RX descriptor released (bad SOF/EOF
                                * or length): a0=idx a1=ctrl a2=len */
#define DT_MAC_RXHRESP      10 /* HRESP/bus-error RX rebuild (fail-closed):
                                * a0=rsr a1=rx_idx a2=rx_recover */
#define DT_MAC_REDISABLEFAIL 11 /* NCR.RE would not clear before a ring rebuild;
                                 * rebuild refused (fail closed): a0=ncr */
/* FIFO */
#define DT_FIFO_PUSH        1   /* a0=(src<<8|dst) a1=type a2=seq */
#define DT_FIFO_POP         2   /* a0=(dst<<8|src) a1=type a2=seq */
/* SCHED */
#define DT_SCHED_SWITCH     1   /* a0=prev_pid a1=next_pid */
#define DT_SCHED_PARK       2   /* a0=pid a1=last_seq */
#define DT_SCHED_WAKE       3   /* a0=pid a1=seq */
#define DT_SCHED_PREEMPT    4   /* a0=pid */
/* IRQ */
#define DT_IRQ_DISPATCH     1   /* a0=intid */
/* OTA / admin stream */
#define DT_OTA_BEGIN        1   /* a0=total */
#define DT_OTA_CHUNK        2   /* a0=received a1=total a2=drain_spins a3=readable */
#define DT_OTA_DRAIN_SPIN   3   /* a0=received a1=spins */
#define DT_OTA_WATCHDOG     4   /* a0=received a1=idle_ms */
#define DT_OTA_COMMIT       5   /* a0=total */
#define DT_OTA_RESET        6   /* a0=received */
#define DT_OTA_WADV         7   /* window re-adv sent; a0=received a1=total a2=wnd_free a3=wadv_count */
#define DT_OTA_REACTOR_GAP  8   /* gap since last drain pass; a0=gap_ticks a1=received a2=total */
#define DT_OTA_RX_RECOVER   9   /* recovery fired inside the tight drain loop;
                                   a0=received a1=total a2=drain_spins a3=rx_recover_count */
#define DT_OTA_SD_BLOCK_DUR 10  /* whole-loop core0-blocking duration of the synchronous
                                   multi-block SD flush done by http_write_kernel_slot_range()
                                   (OTA commit / uartflash commit both funnel through it).
                                   Since CORE_DISK == CORE_NET == 0, this entire duration is
                                   time net_poll()/TCP/UART do NOT run -- a direct measurement
                                   of network-reactor starvation caused by disk I/O sharing
                                   core 0. a0=bytes_written a1=dur_ticks a2=slot_offset a3=0 */
/* REACTOR (core 0 service loop) */
#define DT_RX_PHASE_NET     1   /* a0=dur_ticks */
#define DT_RX_PHASE_TCP     2   /* a0=dur_ticks */
#define DT_RX_PHASE_ADMIN   3   /* a0=dur_ticks */
#define DT_RX_PHASE_ECHO    4   /* a0=dur_ticks */
#define DT_RX_PHASE_HTTP    5   /* a0=dur_ticks (uhttp_bridge_poll — the :80 handler) */
#define DT_RX_WFE           5   /* a0=idle_ticks — keep old ID in case code refs it */
#define DT_RX_WAKE          6   /* a0=flags */
#define DT_RX_IRQ_QUENCH    7   /* IRQ-driven RX quench outcome; a0=core0_eth_irq_count
                                   a1=quench_passes a2=clear(0/1) a3=rx_recv_count.
                                   Diagnoses receive-livelock risk: core0_eth_irq_
                                   drain_and_quench() re-arms the IRQ unconditionally
                                   even when clear==0 (still not caught up after its
                                   8-pass budget); if this fires repeatedly with
                                   clear==0 and irq_count climbing fast relative to
                                   rx_recv_count, the IRQ is re-triggering faster than
                                   the drain can keep up -- a livelock, not a hardware
                                   fault. */

struct dtrace_rec {
    u64 ts;          /* cntpct_el0 at emit */
    u8  core;
    u8  cat_log2;    /* category bit index (0..31) */
    u16 ev;
    u32 _pad;
    u64 a0, a1, a2, a3;
    u64 _pad2[2];    /* pad to 64 bytes */
} ALIGNED(64);

/* Global active mask: 0 == disabled. When enabled it equals the category mask,
 * so the emit gate is a single load + and. Written only by core 0 control. */
extern volatile u32 g_dtrace_active_mask;

void dtrace_init(void);
void dtrace_set_enabled(bool on);
bool dtrace_enabled(void);
void dtrace_set_mask(u32 mask);
u32  dtrace_mask(void);
void dtrace_clear(void);

/* The real worker (out of line). Callers use the DTRACE() macro. */
void dtrace_emit(u32 cat, u16 ev, u64 a0, u64 a1, u64 a2, u64 a3);

/* Format up to max_records most-recent records (timestamp-merged across cores)
 * into out[0..max). Returns bytes written. */
u32  dtrace_dump(char *out, u32 max, u32 max_records);
/* Dump straight to UART (for the serial console). */
void dtrace_dump_uart(u32 max_records);
/* One-line status (enabled, mask, per-core counts). */
u32  dtrace_status(char *out, u32 max);

/* Cheap emit gate: load + and + (rare) call. cat must be a single DTRACE_CAT_*. */
#define DTRACE(cat, ev, a0, a1, a2, a3) \
    do { \
        if (unlikely(g_dtrace_active_mask & (cat))) \
            dtrace_emit((cat), (ev), (u64)(a0), (u64)(a1), (u64)(a2), (u64)(a3)); \
    } while (0)
