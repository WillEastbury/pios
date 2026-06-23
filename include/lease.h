#pragma once
#include "types.h"

/*
 * lease.h - leased-span / descriptor-ownership foundation.
 *
 * Core model (see the FIFO/DMA/Lease refactor design):
 *   - Everything is a Card  -> a semantic handle that resolves to lease(s).
 *   - Everything is a Lease  -> temporary access to physical bytes (a span).
 *   - Move ownership, not data -> descriptors transfer ownership; copy only
 *     across trust boundaries or for transformations.
 *
 * MMU ENFORCEMENT (not advisory):
 *   Isolation is enforced by the page tables, not by descriptor checks alone.
 *   - Each CAPSULE arena's PA is mapped into that capsule's EL2 stage-2 plan
 *     (el2_stage2_plan_set / activate) and/or its EL0/EL1 user table. Other
 *     capsules' tables do not contain that PA, so cross-capsule access faults.
 *   - KERNEL arenas (DMA / FIFO / NIC-RX / device buffers) are NEVER mapped
 *     into any capsule address space. Only EL1 (the kernel) can touch them.
 *     lease_grant() refuses to grant a kernel arena to a capsule.
 *   - The kernel retains EL1 access to all PAs. That is deliberate: it lets a
 *     trusted processing stage write a *result* straight into a capsule lease.
 *   The descriptor generation/bounds checks here are the software half; the
 *   real enforcement is the map/unmap performed by the installed MMU ops.
 *
 * MEMORY MODEL = MESSAGE PASSING + FIFO / ZERO-COPY DESCRIPTOR PASSING:
 *   The primary IPC path is posting a compact lease HANDLE on a FIFO; the
 *   receiver resolves it back to a descriptor (lease_resolve) and reads the
 *   span in place. Nothing is copied to move ownership intra-capsule.
 *     producer: acquire -> write -> publish -> post handle on FIFO
 *     consumer: pop handle -> resolve -> begin_read -> ... -> end_read -> release
 *
 * TRUST-BOUNDARY DATA (the one allowed exception to "no copy"):
 *   Crossing a trust boundary or transforming data produces a NEW span. Two
 *   equivalent shapes:
 *     (a) copy-once: lease_copy_into(src, dst_capsule_arena, ...).
 *     (b) trusted direct-write (e.g. TLS offload / decompress / decrypt): the
 *         kernel acquires a RW lease in the DESTINATION capsule arena, writes
 *         the transformed bytes straight into lease_ptr(dst) (it has EL1
 *         access), publishes (clean so the capsule's WBC view is coherent),
 *         then posts a completion handle on the capsule's FIFO. No staging
 *         buffer, no second copy.
 *
 * An ARENA is a contiguous physical region with an explicit cache-visibility
 * policy. Kernel arenas default to WBC-OFF (deterministic visibility, used for
 * DMA/FIFO/device buffers). Capsule arenas default to WBC-ON (fast path, the
 * capsule owns its own intra-capsule coherency).
 *
 * A LEASE_DESCRIPTOR is a reusable slot from a global pool that names a span
 * inside an arena plus its owner, source, flags and lifecycle state. Slots
 * carry a generation that is bumped on release (poison-on-release) so a stale
 * descriptor copy can never alias a recycled slot.
 *
 * Lifecycle (release/acquire ordered):
 *   FREE -> WRITING -> READY -> READING -> EMPTY -> RELEASE -> FREE
 *
 * Cache-visibility is applied at the lifecycle edges for WBC-ON arenas:
 *   - publish (WRITING->READY):   clean      (make producer writes visible)
 *   - acquire (READY->READING):   invalidate (see fresh bytes, e.g. post-DMA)
 */

#define LEASE_ARENA_MAX            16U
#define LEASE_POOL_MAX             256U

/* Arena kind. */
#define LEASE_ARENA_KIND_KERNEL    1U
#define LEASE_ARENA_KIND_CAPSULE   2U

/* Arena cache-visibility policy. */
#define LEASE_CACHE_WBC_OFF        1U  /* Non-cacheable: deterministic visibility. */
#define LEASE_CACHE_WBC_ON         2U  /* Write-back cacheable: explicit clean/invalidate. */

/* Lease flags. */
#define LEASE_F_READONLY           0x0001U
#define LEASE_F_READWRITE          0x0002U
#define LEASE_F_DEVICE             0x0004U
#define LEASE_F_DMA                0x0008U
#define LEASE_F_KERNEL             0x0010U
#define LEASE_F_CAPSULE            0x0020U
#define LEASE_F_SHARED             0x0040U

/* Source type (consumers must not assume origin). */
#define LEASE_SRC_KERNEL_MEM       1U
#define LEASE_SRC_CAPSULE_MEM      2U
#define LEASE_SRC_DISK_CACHE       3U
#define LEASE_SRC_DMA_ARENA        4U
#define LEASE_SRC_NIC_ARENA        5U
#define LEASE_SRC_TENSOR_CACHE     6U
#define LEASE_SRC_GPU_BUFFER       7U

/* Lifecycle state. */
#define LEASE_STATE_FREE           0U
#define LEASE_STATE_WRITING        1U
#define LEASE_STATE_READY          2U
#define LEASE_STATE_READING        3U
#define LEASE_STATE_EMPTY          4U
#define LEASE_STATE_RELEASE        5U

/* Return codes. */
#define LEASE_OK                   0
#define LEASE_ERR_INVAL           -1
#define LEASE_ERR_NOSPC           -2
#define LEASE_ERR_STALE           -3   /* generation mismatch (poisoned). */
#define LEASE_ERR_STATE           -4   /* illegal lifecycle transition. */
#define LEASE_ERR_NOENT           -5

#define LEASE_ARENA_NONE           0xFFFFFFFFU
#define LEASE_OWNER_KERNEL         0U   /* capsule 0 == kernel. */

struct lease_arena {
    u32 arena_id;
    u32 kind;            /* LEASE_ARENA_KIND_* */
    u32 cache_policy;    /* LEASE_CACHE_WBC_* */
    u32 owner_capsule;   /* 0 for kernel arenas */
    u64 base;            /* physical base */
    u64 size;            /* bytes */
    u64 cursor;          /* bump-allocation cursor (offset within arena) */
    u32 generation;      /* bumped on arena reset */
    u32 in_use;          /* registry slot occupied */
};

/* Reusable descriptor slot. PACKED so it can travel over a fifo_span. */
struct lease_descriptor {
    u32 slot_id;         /* index into the descriptor pool */
    u32 generation;      /* slot generation (poison-on-release) */
    u64 lease_id;        /* monotonic, globally unique while live */
    u32 arena_id;
    u32 source_type;     /* LEASE_SRC_* */
    u64 offset;          /* byte offset within the arena */
    u64 length;          /* byte length of the span */
    u32 owner_capsule;
    u32 flags;           /* LEASE_F_* */
    u32 state;           /* LEASE_STATE_* */
    u32 _pad;
} PACKED;

struct lease_stats {
    u32 arenas;
    u32 slots_total;
    u32 slots_live;       /* state != FREE */
    u32 acquires;
    u32 releases;
    u32 transfers;
    u32 copies;           /* cross-boundary copy-once count */
    u32 grants;           /* spans mapped into a capsule (MMU) */
    u32 revokes;          /* spans unmapped from a capsule (MMU) */
    u32 mmu_rejects;      /* refused grants (e.g. kernel arena -> capsule) */
    u32 stale_rejects;
    u32 state_rejects;
    u64 next_lease_id;
};

/*
 * Compact handle that travels over a FIFO (fits a fifo_span_msg). The receiver
 * calls lease_resolve() to validate the generation and recover the descriptor.
 * This is the zero-copy, message-passing transport unit.
 */
struct lease_handle {
    u64 lease_id;
    u32 slot_id;
    u32 generation;
};

/*
 * MMU-enforcement seam. The MMU layer installs these so lease_grant/revoke map
 * and unmap a span's PA into a capsule's stage-2 / user table. Kept as a hook
 * so this module compiles and self-tests standalone; real enforcement plugs in.
 *   access is LEASE_F_READONLY or LEASE_F_READWRITE.
 *   return 0 on success, negative on failure.
 */
struct lease_mmu_ops {
    i32 (*map)(u32 capsule, u64 pa, u64 length, u32 access);
    i32 (*unmap)(u32 capsule, u64 pa, u64 length);
};
void lease_set_mmu_ops(const struct lease_mmu_ops *ops);

void lease_init(void);

/* Arena registry. Returns arena_id or LEASE_ARENA_NONE on failure. */
u32  lease_arena_register(u32 kind, u32 cache_policy, u32 owner_capsule,
                          u64 base, u64 size);
const struct lease_arena *lease_arena_get(u32 arena_id);
void lease_arena_reset(u32 arena_id);   /* bump cursor=0, generation++ */

/*
 * Acquire a fresh lease: bump-allocate `length` bytes in `arena_id`, grab a
 * FREE descriptor slot, set state=WRITING, assign owner/flags/source.
 * On success fills *out and returns LEASE_OK.
 */
i32  lease_acquire(u32 arena_id, u64 length, u32 flags, u32 source_type,
                   u32 owner_capsule, struct lease_descriptor *out);

/* Lifecycle transitions. Each re-validates slot+generation. */
i32  lease_publish(const struct lease_descriptor *d);   /* WRITING -> READY  (clean if WBC_ON) */
i32  lease_begin_read(const struct lease_descriptor *d); /* READY  -> READING (invalidate if WBC_ON) */
i32  lease_end_read(const struct lease_descriptor *d);   /* READING-> EMPTY */
i32  lease_release(const struct lease_descriptor *d);    /* any    -> RELEASE -> FREE (poison) */

/* Ownership transfer (intra-capsule move). Re-binds owner_capsule in place. */
i32  lease_transfer(const struct lease_descriptor *d, u32 new_owner_capsule);

/*
 * MMU-enforced grant/revoke. lease_grant maps the span's PA into `capsule`'s
 * address space with `access` (LEASE_F_READONLY|READWRITE) via the installed
 * MMU ops; it REFUSES kernel arenas (those stay EL1-only). lease_revoke
 * unmaps it. These are the hardware half of isolation.
 */
i32  lease_grant(const struct lease_descriptor *d, u32 capsule, u32 access);
i32  lease_revoke(const struct lease_descriptor *d, u32 capsule);

/* Compact-handle (FIFO message) transport for zero-copy descriptor passing. */
void lease_handle_of(const struct lease_descriptor *d, struct lease_handle *out);
i32  lease_resolve(const struct lease_handle *h, struct lease_descriptor *out);

/*
 * Cross-boundary copy-once: copy a live source lease's bytes into a fresh
 * lease in `dst_arena` owned by `dst_capsule`, then publish the destination.
 * Establishes a trust boundary (no shared mutable memory). Fills *out_dst.
 */
i32  lease_copy_into(const struct lease_descriptor *src, u32 dst_arena,
                     u32 dst_capsule, u32 dst_flags, struct lease_descriptor *out_dst);

/* Kernel gatekeeper validation: bounds + slot + generation + (optional) owner. */
i32  lease_validate(const struct lease_descriptor *d, u32 expect_owner /* or LEASE_ARENA_NONE */);

/* Resolve a live lease to a kernel-side pointer (NULL if invalid). */
void *lease_ptr(const struct lease_descriptor *d);

void lease_get_stats(struct lease_stats *out);
bool lease_selftest(void);
