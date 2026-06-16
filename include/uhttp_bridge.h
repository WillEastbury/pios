/*
 * uhttp_bridge.h - shared kernel<->userland HTTP bridge.
 *
 * The kernel (core 0, network) terminates TCP on benchmark ports and hands each request
 * to a userland HTTP process via a shared-memory ring + a cross-core software
 * wake (proc_post_remote_wake + SEV). The userland process sleeps (BLOCKED)
 * between requests and is woken only when a request arrives — no syscall spin,
 * zero-copy request/response buffers. The bridge lives at IPC_SHM_BASE, which
 * the kernel sees via the identity map and the userland process sees via its
 * mapped IPC_SHM window.
 *
 * Cross-core coherency: the bridge RAM is mapped Normal cacheable inner-shareable
 * on both cores, yet on this bring-up a single low-frequency write by an
 * otherwise-idle core is NOT reliably propagated to another core's cache via
 * snoop alone — the writer's value sits in its L1 and is never evicted
 * (high-throughput FIFOs work only because continuous writes force eviction).
 * NOTE: this is NOT the A76 "SMPEN" bit (a no-op on A76 — the cluster is always
 * coherent for inner-shareable accesses) and it is NOT an attribute mismatch
 * (every core shares shared_ttbr0/shared_mair/shared_tcr). Root cause is
 * unresolved and under investigation. So every cross-core control field is
 * explicitly cleaned to PoC by its writer and invalidated before each read by the
 * reader (uhttp_clean / uhttp_inval below). To make that safe against dc-op
 * write-back clobber, the two traffic directions are split onto SEPARATE 64-byte
 * cache lines so each line has exactly one writer:
 *   Zone B (offset 0,  one line): userland(core2) -> core0  [httpd writes]
 *   Zone A (offset 64, one line): core0 -> userland(core2)  [core0 writes]
 * The req[]/resp[] buffers are line-aligned and multiples of 64 bytes, so a line
 * is never shared across directions. Both the kernel and the userland httpd run
 * at EL1 (processes are entered via ctx_switch's `ret`, not `eret`), so dc cvac /
 * dc ivac are legal on both sides.
 */
#pragma once
#include "types.h"
#include "platform.h"

#define UHTTP_BRIDGE_MAGIC   0x48545450U   /* 'HTTP' */
#define UHTTP_PORT           82U
#define UHTTP_NATIVE_PORT    83U
#define UHTTP_BRIDGE_COUNT   2U
#define UHTTP_BRIDGE_STRIDE  0x00010000UL
#define UHTTP_REQ_MAX        8192U
#define UHTTP_RESP_MAX       24576U
#define UHTTP_PICO_MAX       4096U
#define UHTTP_PICOWEB_CARD   410U
#define UHTTP_PICO_PROGRAM_RECORD 0U
#define UHTTP_PICO_STATIC_RECORD  1U
#define UHTTP_PICO_API_RECORD     2U
#define UHTTP_PICO_DYNAMIC_RECORD 3U
/* Must equal IPC_SHM_BASE (core_env.h); verified by _Static_assert in the .c. */
#ifdef PIOS_USER_EL0
#ifndef UHTTP_BRIDGE_INDEX
#define UHTTP_BRIDGE_INDEX   0U
#endif
#define UHTTP_BRIDGE_ALIAS_BASE 0x2003000000UL
#define UHTTP_BRIDGE_ADDR    (UHTTP_BRIDGE_ALIAS_BASE + (UHTTP_BRIDGE_INDEX * UHTTP_BRIDGE_STRIDE))
#else
#define UHTTP_BRIDGE_ADDR    PIOS_IPC_SHM_BASE
#endif

#define UHTTP_LINE           64U

struct uhttp_bridge {
    /* ── Zone B: userland(core2) -> core0. httpd writes, core0 reads. One line. */
    volatile u32 magic;       /* userland sets UHTTP_BRIDGE_MAGIC when attached */
    volatile u32 httpd_pid;   /* userland writes its pid for cross-core wake    */
    volatile u32 resp_seq;    /* userland sets == the req_seq it answered        */
    volatile u32 resp_len;    /* bytes in resp[]                                 */
    volatile u32 dbg_loops;   /* userland increments each handler-loop pass      */
    volatile u32 dbg_phase;   /* userland publishes its current loop step (debug) */
    u8 _padB[UHTTP_LINE - 6U * 4U];
    /* ── Zone A: core0 -> userland(core2). core0 writes, httpd reads. One line. */
    volatile u32 req_seq;     /* core 0 increments when a request is ready       */
    volatile u32 req_len;     /* bytes in req[]                                  */
    volatile u32 reqs_total;  /* lifetime request count (diagnostics)            */
    u8 _padA[UHTTP_LINE - 3U * 4U];
    /* ── Bulk buffers: each line-aligned, length a multiple of 64 bytes. */
    u8 req[UHTTP_REQ_MAX];    /* Zone A data: core0 -> httpd */
    /* Optional PicoScript web assets, preloaded by core0 from WALFS/picowal
     * before waking the user process. This keeps the live request path non-
     * blocking in userland while still serving bytecode/static/API data from
     * cards. Metadata is core0->userland and sits on its own line. */
    volatile u32 pico_prog_len;
    volatile u32 pico_static_len;
    volatile u32 pico_api_len;
    volatile u32 pico_flags;
    u8 _padP[UHTTP_LINE - 4U * 4U];
    u8 pico_prog[UHTTP_PICO_MAX];
    u8 pico_static[UHTTP_PICO_MAX];
    u8 pico_api[UHTTP_PICO_MAX];
    u8 resp[UHTTP_RESP_MAX];  /* Zone B data: httpd -> core0 */
};

/* Clean a range to the Point of Coherency (writer side): make these stores
 * visible to other cores even if this core stays idle and never evicts. */
static inline void uhttp_clean(const volatile void *p, u32 n)
{
#ifdef PIOS_USER_EL0
    (void)p; (void)n;
    __asm__ volatile("" ::: "memory");
#else
    u64 a = (u64)(usize)p & ~(u64)(UHTTP_LINE - 1U);
    u64 e = (u64)(usize)p + n;
    for (; a < e; a += UHTTP_LINE)
        __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
#endif
}

/* Invalidate a range to PoC (reader side): drop this core's possibly-stale copy
 * so the next read refills from PoC where the writer cleaned. ONLY call on a line
 * this core never writes — dc ivac discards dirty data without write-back. */
static inline void uhttp_inval(volatile void *p, u32 n)
{
#ifdef PIOS_USER_EL0
    (void)p; (void)n;
    __asm__ volatile("" ::: "memory");
#else
    u64 a = (u64)(usize)p & ~(u64)(UHTTP_LINE - 1U);
    u64 e = (u64)(usize)p + n;
    for (; a < e; a += UHTTP_LINE)
        __asm__ volatile("dc ivac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
#endif
}

static inline struct uhttp_bridge *uhttp_bridge(void)
{
    return (struct uhttp_bridge *)(usize)UHTTP_BRIDGE_ADDR;
}

static inline struct uhttp_bridge *uhttp_bridge_at(u32 idx)
{
#ifdef PIOS_USER_EL0
    return (struct uhttp_bridge *)(usize)(UHTTP_BRIDGE_ALIAS_BASE + (idx * UHTTP_BRIDGE_STRIDE));
#else
    return (struct uhttp_bridge *)(usize)(0x04D00000UL + (idx * UHTTP_BRIDGE_STRIDE));
#endif
}

/* Core 0 side. */
void uhttp_bridge_init(void);   /* tcp_listen(81) + reset bridge header */
void uhttp_bridge_poll(void);   /* per-connection state machine; call from core 0 */
bool uhttp_bridge_ready(void);  /* true once the userland httpd has attached */
u32  uhttp_bridge_requests(void);
/* Diagnostics: snapshot the core0 state machine + shared bridge header. */
void uhttp_bridge_state(i32 *listen_conn, u32 *state, u32 *req_seq, u32 *resp_seq,
                        u32 *reqs, u32 *magic, u32 *httpd_pid);
void uhttp_bridge_state_idx(u32 idx, i32 *listen_conn, u32 *state, u32 *req_seq, u32 *resp_seq,
                            u32 *reqs, u32 *magic, u32 *httpd_pid);
