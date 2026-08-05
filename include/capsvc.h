/*
 * capsvc.h - generic capsule-service dispatcher.
 *
 * Purpose: let TCP-terminated services (admin console, web server, future
 * TCP services) run as separate EL0 capsule processes instead of in-kernel
 * code, WITHOUT one bespoke kernel bridge source file per service, and
 * WITHOUT syscall/EL0<->EL1 transitions on the per-message hot path (that
 * cost is the whole point of moving work out of the kernel in the first
 * place -- an architecture that saves a context switch by adding a syscall
 * per message would be a net loss). There is exactly one dispatcher
 * (src/capsvc.c) driven by a small registration table (capsvc_register());
 * adding a new capsule-backed service is a table entry, not new kernel C
 * code, and the actual per-service behaviour is PicoScript bytecode loaded
 * from a WALFS/picowal card, not hand-written C (see capsule_store.h,
 * user/httpd.c's uhttp_load_capsule_process() for the existing card-loading
 * pattern this reuses).
 *
 * Transport: a shared Normal-NC memory arena, same technique uhttp_bridge
 * already uses and for the same reason -- NC memory needs NO cache
 * maintenance instructions to stay coherent, so EL0 can read a request and
 * write a reply with plain loads/stores and zero syscalls. The two
 * mechanisms that DO need a privileged op are both kernel-initiated, never
 * capsule-initiated:
 *   - kernel -> capsule wake: proc_post_remote_wake() (SGI/SEV), called by
 *     the kernel after writing a request -- this is the "make sure it wakes"
 *     half of "push a message, then make sure it wakes".
 *   - capsule -> kernel reply: no wake at all. The capsule just writes the
 *     reply into the arena slot (plain store); the kernel discovers it by
 *     polling/invalidating that slot on its next reactor pass, which runs
 *     far more often than any real latency this could add (core 0 is never
 *     asleep for long once real traffic is flowing).
 *   - capsule's ONLY syscall is park()/wake-resume itself when genuinely
 *     idle -- unavoidable, since blocking requires the scheduler.
 * Raw fifo.h (the inter-core SPSC ring) is deliberately NOT used here: its
 * push path ends in a GIC SGI send and its pop path calls `dc ivac`
 * internally, both EL1-privileged, so it can never be safely called by an
 * EL0 process in either direction -- it is only ever used core-to-core
 * between kernel-resident (EL1) code today.
 *
 * Every slot carries a generation counter, bumped on release, so a
 * stale/replayed tag can never be mistaken for a live request (the
 * "generation field required for reusable objects" invariant).
 */
#pragma once
#include "types.h"
#include "platform.h"

#define CAPSVC_MAX_SERVICES   8U
#define CAPSVC_SLOT_COUNT     8U
#define CAPSVC_SLOT_DATA_MAX  2048U
#define CAPSVC_LINE           64U

/* Arena lives in the existing IPC shared-memory window, clear of the two
 * uhttp_bridge instances (which occupy [0, UHTTP_BRIDGE_STRIDE*2) = [0,0x20000)). */
#define CAPSVC_ARENA_OFFSET   0x00040000UL
#define CAPSVC_ARENA_BASE     (PIOS_IPC_SHM_BASE + CAPSVC_ARENA_OFFSET)
#ifdef PIOS_USER_EL0
#define CAPSVC_ARENA_ALIAS_BASE 0x2003000000UL   /* == UHTTP_BRIDGE_ALIAS_BASE, same PA window;
                                                   * mapped Normal-NC (mmu.c map_user_ipc_el0_alias) */
#define CAPSVC_ARENA_ADDR       (CAPSVC_ARENA_ALIAS_BASE + CAPSVC_ARENA_OFFSET)
#else
#define CAPSVC_ARENA_ADDR       CAPSVC_ARENA_BASE
#endif

enum {
    CAPSVC_SLOT_FREE = 0,     /* available for a new request */
    CAPSVC_SLOT_REQUEST = 1,  /* kernel has written a request, capsule owns it */
    CAPSVC_SLOT_REPLY = 2,    /* capsule has written a reply, kernel owns it */
};

/* One arena slot: control header on its own cache line, kept separate from
 * the payload area (control and payload must never share a cache line).
 * request/reply reuse the SAME slot sequentially (never concurrently -- the
 * state field enforces that only one side ever writes at a time), so one
 * data buffer suffices for both directions. */
struct capsvc_slot_hdr {
    volatile u32 generation;   /* bumped on every release; ties a reply to its request */
    volatile u32 state;        /* CAPSVC_SLOT_* */
    volatile u32 svc_idx;      /* which capsvc_registration this belongs to */
    volatile u32 conn_slot;    /* index into the kernel's own connection table */
    volatile u32 req_len;      /* kernel writes: request byte count */
    volatile u32 resp_len;     /* capsule writes: response byte count */
    volatile u64 tag;          /* capsvc_tag_pack(port, arena_slot, generation), opaque to capsule */
    u8 _pad[CAPSVC_LINE - (6U * 4U + 8U)];
};

#define CAPSVC_ATTACH_MAGIC 0x43415053U   /* 'CAPS' */
#define CAPSVC_DESC_MAX     CAPSVC_LINE   /* one dedicated line, write-once at attach time */
#define CAPSVC_HOST_MAX     64U

/* One attach record per registered service. Two dedicated cache lines
 * (capsule writes; kernel only reads): the first is the hot magic/pid
 * handshake line (mirrors uhttp_bridge's magic/httpd_pid handshake -- the
 * capsule publishes who it is once at startup, the kernel discovers it by
 * polling and calls capsvc_attach()); the second holds a write-once
 * human-readable description string, compiled into whichever capsvc_host.c
 * (or future capsule host) binary owns this service, so it never shares a
 * line with the live magic/pid fields (kept off that hot path, even though
 * both are written exactly once here at startup). */
struct capsvc_attach_block {
    volatile u32 magic;    /* CAPSVC_ATTACH_MAGIC once the capsule has attached */
    volatile u32 svc_idx;
    volatile u32 pid;
    u8 _pad[CAPSVC_LINE - 3U * 4U];
    char description[CAPSVC_DESC_MAX];   /* NUL-terminated; empty until the capsule attaches */
};

/* Per-service PicoScript bytecode program, preloaded ONCE by the kernel at
 * capsvc_init() from a WALFS/picowal capsule manifest (capsule_store.h,
 * capsule_manifest_parse -- the same frozen manifest format and card-pairing
 * convention uhttp_bridge.c already uses for the port-82/83 workers), matched
 * to this service's port via the manifest's `io = tcp/<port>` binding. This
 * is kernel(EL1)->capsule(EL0) one-way: only the kernel ever calls
 * capsule_store_read (a privileged WALFS path); the capsule only ever reads
 * this NC-mapped buffer with plain loads, same zero-syscall discipline as
 * the request/reply slots. Preloaded once at init (not per-request) to keep
 * the request hot path free of WALFS I/O; a live "reload" console command is
 * a documented follow-up, not implemented yet. len==0 means no manifest
 * entry matched this port -- the capsule falls back to its own compiled-in
 * default program (mirrors picoweb_default_program's fallback in httpd.c). */
#define CAPSVC_PROG_MAX 4096U
struct capsvc_program {
    volatile u32 len;
    u8 data[CAPSVC_PROG_MAX];
};

struct capsvc_slot {
    struct capsvc_slot_hdr hdr;
    u8 data[CAPSVC_SLOT_DATA_MAX];   /* request bytes in place; capsule overwrites with reply */
};

struct capsvc_arena {
    struct capsvc_attach_block attach[CAPSVC_MAX_SERVICES];
    struct capsvc_program programs[CAPSVC_MAX_SERVICES];
    struct capsvc_slot slots[CAPSVC_SLOT_COUNT];
};

_Static_assert(sizeof(struct capsvc_attach_block) == 2U * CAPSVC_LINE, "capsvc attach block must be two lines (magic/pid + description)");
_Static_assert(sizeof(struct capsvc_slot_hdr) == CAPSVC_LINE, "capsvc slot header must be one line");
_Static_assert((CAPSVC_SLOT_DATA_MAX % CAPSVC_LINE) == 0U, "capsvc slot data must be line-sized");
_Static_assert(CAPSVC_ARENA_OFFSET + sizeof(struct capsvc_arena) <= PIOS_IPC_SHM_SIZE,
               "capsvc arena must fit inside the IPC shared-memory window "
               "alongside the two existing uhttp_bridge instances");

static inline struct capsvc_arena *capsvc_arena(void)
{
    return (struct capsvc_arena *)(usize)CAPSVC_ARENA_ADDR;
}

/* tag = port(16) | arena_slot(16) | generation(32) -- capsule treats this
 * opaquely and echoes it back verbatim in hdr.tag; only the kernel ever
 * decodes it, and only the kernel ever sees a real tcp_conn_t. */
static inline u64 capsvc_tag_pack(u16 port, u16 arena_slot, u32 generation)
{
    return ((u64)port << 48) | ((u64)arena_slot << 32) | (u64)generation;
}
static inline u16 capsvc_tag_port(u64 tag)       { return (u16)(tag >> 48); }
static inline u16 capsvc_tag_slot(u64 tag)       { return (u16)(tag >> 32); }
static inline u32 capsvc_tag_generation(u64 tag) { return (u32)tag; }

/* Cache maintenance, same split as uhttp_bridge.h: EL1 (kernel) does real
 * dc cvac/ivac; EL0 (capsule) compiles these to a bare compiler barrier
 * because its alias is mapped Normal-NC -- no cache op is needed there at
 * all, which is exactly what makes the capsule's read/write path syscall-free. */
static inline void capsvc_clean(const volatile void *p, u32 n)
{
#ifdef PIOS_USER_EL0
    (void)p; (void)n;
    __asm__ volatile("" ::: "memory");
#else
    u64 a = (u64)(usize)p & ~(u64)(CAPSVC_LINE - 1U);
    u64 e = (u64)(usize)p + n;
    for (; a < e; a += CAPSVC_LINE)
        __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
#endif
}

static inline void capsvc_inval(volatile void *p, u32 n)
{
#ifdef PIOS_USER_EL0
    (void)p; (void)n;
    __asm__ volatile("" ::: "memory");
#else
    u64 a = (u64)(usize)p & ~(u64)(CAPSVC_LINE - 1U);
    u64 e = (u64)(usize)p + n;
    for (; a < e; a += CAPSVC_LINE)
        __asm__ volatile("dc ivac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
#endif
}

#ifndef PIOS_USER_EL0
/* ---- Kernel (core 0) side API ---- */

/* Register a capsule-backed TCP service. Call once per service at init,
 * before capsvc_init(). Returns the service index, or -1 if the table is
 * full. This is the ONLY thing a new service needs from kernel-resident
 * code -- no new .c file, no new state machine. `target_core` is where the
 * PicoScript-hosting capsule for this service will run (CORE_USER0/CORE_USER1). */
i32 capsvc_register(u16 port, u32 target_core);
bool capsvc_bind_host(u32 svc_idx, const char *host);
i32 capsvc_find_host(const char *host);

/* Bind the capsule pid that owns a registered service, once it attaches
 * (mirrors uhttp_bridge's magic/httpd_pid handshake -- the capsule publishes
 * its own pid into a well-known slot on first run; capsvc_poll() picks it up). */
void capsvc_attach(u32 svc_idx, u32 capsule_pid);

void capsvc_init(void);   /* opens tcp_listen() for every registered service */
void capsvc_poll(void);   /* per-connection state machine; call from core 0 reactor */

/* Kernel-side diagnostics: dump attach state for every registered service.
 * Generic (works for any capsvc.h-registered service, not admin-specific). */
void capsvc_debug_status(char *out, u32 *len, u32 max);

/* Generic service enumeration, for dashboard/workbench/admin-console UIs that
 * want to show "which capsule is hosting what" -- capsvc.c stays
 * service-agnostic (it never names a service), but callers like kernel.c are
 * allowed to be service-aware and may map a returned port to a friendly label
 * themselves. desc_out receives the capsule's own self-published description
 * (empty string until it attaches; pass NULL/0 to skip). capsvc_service_info()
 * returns false for an out-of-range idx. */
u32 capsvc_service_count(void);
bool capsvc_service_info(u32 idx, u16 *port_out, u32 *target_core_out,
                          u32 *capsule_pid_out, bool *attached_out,
                          char *desc_out, u32 desc_max);
bool capsvc_service_host(u32 idx, char *host_out, u32 host_max);

/* TLS/router transport: publish one already-terminated request into the same
 * generation-backed capsule arena without creating another listener. */
i32 capsvc_external_begin(u32 svc_idx, const u8 *request, u32 request_len,
                          u64 *token_out);
i32 capsvc_external_poll(u64 token, u8 *response, u32 response_max,
                         u32 *response_len_out);
void capsvc_external_cancel(u64 token);
#endif
