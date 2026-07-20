/*
 * fifo.c - Lock-free SPSC inter-core FIFOs in shared memory
 * 16 channels (4×4 grid). fifo[i][i] unused but allocated.
 * Lives at SHARED_FIFO_BASE to keep per-core RAM truly private.
 */

#include "fifo.h"
#include "simd.h"
#include "core_env.h"
#include "fb.h"
#include "dtrace.h"
#include "gic.h"
#include "mmu.h"

#define FIFO_GRID_ENTRIES       16U
#define FIFO_MSG_POOL_BYTES     (FIFO_GRID_ENTRIES * sizeof(struct fifo))
#define FIFO_SPAN_POOL_OFFSET   FIFO_MSG_POOL_BYTES
#define FIFO_SPAN_POOL_BYTES    (FIFO_GRID_ENTRIES * sizeof(struct fifo_span))
#define FIFO_IRQ_TARGET_OFFSET  (FIFO_SPAN_POOL_OFFSET + FIFO_SPAN_POOL_BYTES)

struct fifo_core_seq {
    volatile u32 seq;
    u32 _pad[15];
} ALIGNED(64);
static struct fifo_core_seq fifo_seq[4];
_Static_assert(sizeof(struct fifo_core_seq) == 64,
               "FIFO sequence records must be one cache line");

struct fifo_irq_target {
    volatile u32 ready;
    u32 _pad[15];
} ALIGNED(64);

struct fifo_irq_counter {
    volatile u32 sent;
    u32 _pad[15];
} ALIGNED(64);

#define FIFO_IRQ_TARGET_BYTES   (4U * sizeof(struct fifo_irq_target))
#define FIFO_IRQ_COUNTER_OFFSET (FIFO_IRQ_TARGET_OFFSET + FIFO_IRQ_TARGET_BYTES)
#define FIFO_IRQ_COUNTER_BYTES  (FIFO_GRID_ENTRIES * sizeof(struct fifo_irq_counter))
#define FIFO_SHARED_BYTES       (FIFO_IRQ_COUNTER_OFFSET + FIFO_IRQ_COUNTER_BYTES)

_Static_assert(sizeof(struct fifo_irq_target) == 64,
               "FIFO IRQ target records must be one cache line");
_Static_assert(sizeof(struct fifo_irq_counter) == 64,
               "FIFO IRQ counters must be one cache line");
_Static_assert(FIFO_SHARED_BYTES <= SHARED_FIFO_SIZE,
               "FIFO rings and IRQ metadata exceed SHARED_FIFO_SIZE");

static inline void fifo_clean(const volatile void *p, u64 len) {
    dcache_clean_range((u64)(usize)p, len);
}

static inline void fifo_invalidate(const volatile void *p, u64 len) {
    dcache_invalidate_range((u64)(usize)p, len);
}

static inline u32 fifo_load_acquire(volatile u32 *p) {
    fifo_invalidate(p, sizeof(*p));
    u32 value = *p;
    dmb_ishld();
    return value;
}

static inline void fifo_store_release(volatile u32 *p, u32 value) {
    /* Full release: producer head stores order payload writes; consumer tail
     * stores must also order payload reads before freeing the slot. */
    dmb_ish();
    *p = value;
    fifo_clean(p, sizeof(*p));
    dsb_ishst();
}

static inline u32 fifo_atomic_load_acquire(volatile u32 *p) {
    fifo_invalidate(p, sizeof(*p));
    return atomic_load32(p);
}

static inline void fifo_atomic_store_release(volatile u32 *p, u32 value) {
    atomic_store32(p, value);
    fifo_clean(p, sizeof(*p));
    dsb_ishst();
}

static inline void fifo_note_activity(u32 count)
{
    if (count == 0)
        return;
    u32 c = core_id() & 3U;
    fifo_seq[c].seq += count;
    fifo_clean(&fifo_seq[c], sizeof(fifo_seq[c]));
    dsb_ishst();
}

u32 fifo_last_sequence(u32 core)
{
    if (core >= 4U)
        return 0;
    fifo_invalidate(&fifo_seq[core], sizeof(fifo_seq[core]));
    dmb_ishld();
    return fifo_seq[core].seq;
}

/* FIFO pool in shared memory: 4×4 = 16 fifo structs */
static inline struct fifo *get_fifo(u32 src, u32 dst) {
    return (struct fifo *)(SHARED_FIFO_BASE +
                           ((usize)src * 4 + dst) * sizeof(struct fifo));
}

static inline struct fifo_span *get_span_fifo(u32 src, u32 dst) {
    return (struct fifo_span *)(SHARED_FIFO_BASE +
                                FIFO_SPAN_POOL_OFFSET +
                                ((usize)src * 4 + dst) * sizeof(struct fifo_span));
}

static inline struct fifo_irq_target *get_irq_target(u32 core) {
    return (struct fifo_irq_target *)(SHARED_FIFO_BASE +
                                      FIFO_IRQ_TARGET_OFFSET +
                                      (usize)core * sizeof(struct fifo_irq_target));
}

static inline struct fifo_irq_counter *get_irq_counter(u32 src, u32 dst) {
    return (struct fifo_irq_counter *)(SHARED_FIFO_BASE +
                                       FIFO_IRQ_COUNTER_OFFSET +
                                       ((usize)src * 4 + dst) * sizeof(struct fifo_irq_counter));
}

void fifo_irq_enable(u32 core) {
    if (core >= 4U)
        return;
    fifo_store_release(&get_irq_target(core)->ready, 1U);
}

bool fifo_irq_ready(u32 core) {
    if (core >= 4U)
        return false;
    return fifo_load_acquire(&get_irq_target(core)->ready) != 0U;
}

u32 fifo_irq_sent(u32 core) {
    if (core >= 4U)
        return 0;
    u32 total = 0;
    for (u32 src = 0; src < 4U; src++)
        total += fifo_load_acquire(&get_irq_counter(src, core)->sent);
    return total;
}

static inline void fifo_notify(u32 src, u32 dst) {
    if (fifo_irq_ready(dst)) {
        struct fifo_irq_counter *counter = get_irq_counter(src, dst);
        fifo_store_release(&counter->sent, counter->sent + 1U);
        gic_send_sgi((u8)(1U << dst), GIC_SGI_WAKE);
    }
    /* SEV remains the sticky correctness backstop if the platform's SGI
     * security/group routing is not yet delivering to that secondary. */
    sev();
}

void fifo_init_all(void) {
    /* Zero the entire shared FIFO region with NEON */
    simd_zero((void *)SHARED_FIFO_BASE, SHARED_FIFO_SIZE);
    simd_zero(fifo_seq, sizeof(fifo_seq));
    dsb();
}

bool fifo_push(u32 src, u32 dst, const struct fifo_msg *msg) {
    if (src >= 4 || dst >= 4) return false;
    struct fifo *f = get_fifo(src, dst);
    u32 head = f->head;
    u32 tail = fifo_load_acquire(&f->tail);
    u32 next = (head + 1) & (FIFO_CAPACITY - 1); /* power-of-2 mask */

    if (unlikely(next == tail))
        return false;   /* full */

    /* Copy message with NEON (64 bytes = one cache line) */
    simd_memcpy(&f->msgs[head], msg, sizeof(struct fifo_msg));
    fifo_clean(&f->msgs[head], sizeof(struct fifo_msg));
    fifo_store_release(&f->head, next);
    fifo_note_activity(1);
    DTRACE(DTRACE_CAT_FIFO, DT_FIFO_PUSH, (src << 8) | dst, msg->type, next, msg->tag);
    fifo_notify(src, dst);
    return true;
}

u32 fifo_push_batch(u32 src, u32 dst, const struct fifo_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo *f = get_fifo(src, dst);
    u32 head = f->head;
    u32 tail = fifo_load_acquire(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        simd_memcpy(&f->msgs[head], &msgs[pushed], sizeof(struct fifo_msg));
        fifo_clean(&f->msgs[head], sizeof(struct fifo_msg));
        head = next;
        pushed++;
    }
    if (pushed) {
        fifo_store_release(&f->head, head);
        fifo_note_activity(pushed);
        fifo_notify(src, dst);
    }
    return pushed;
}

bool fifo_pop(u32 dst, u32 src, struct fifo_msg *msg) {
    if (src >= 4 || dst >= 4) return false;
    struct fifo *f = get_fifo(src, dst);
    u32 tail = f->tail;

    u32 head = fifo_load_acquire(&f->head);
    if (unlikely(tail == head))
        return false;   /* empty */

    fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_msg));
    simd_memcpy(msg, &f->msgs[tail], sizeof(struct fifo_msg));
    fifo_store_release(&f->tail, (tail + 1) & (FIFO_CAPACITY - 1));
    fifo_note_activity(1);
    DTRACE(DTRACE_CAT_FIFO, DT_FIFO_POP, (dst << 8) | src, msg->type, f->tail, msg->tag);
    return true;
}

u32 fifo_pop_batch(u32 dst, u32 src, struct fifo_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo *f = get_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = fifo_load_acquire(&f->head);
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_msg));
        simd_memcpy(&msgs[popped], &f->msgs[tail], sizeof(struct fifo_msg));
        tail = (tail + 1) & (FIFO_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        fifo_store_release(&f->tail, tail);
        fifo_note_activity(popped);
    }
    return popped;
}

bool fifo_peek(u32 dst, u32 src, struct fifo_msg *msg) {
    if (src >= 4 || dst >= 4 || !msg) return false;
    struct fifo *f = get_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = fifo_load_acquire(&f->head);
    if (unlikely(tail == head))
        return false;
    fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_msg));
    simd_memcpy(msg, &f->msgs[tail], sizeof(struct fifo_msg));
    return true;
}

bool fifo_empty(u32 dst, u32 src) {
    struct fifo *f = get_fifo(src, dst);
    u32 head = fifo_load_acquire(&f->head);
    u32 tail = fifo_load_acquire(&f->tail);
    return tail == head;
}

u32 fifo_count(u32 dst, u32 src) {
    struct fifo *f = get_fifo(src, dst);
    u32 h = fifo_load_acquire(&f->head);
    u32 t = fifo_load_acquire(&f->tail);
    return (h - t) & (FIFO_CAPACITY - 1);
}

u32 fifo_span_push_batch(u32 src, u32 dst, const struct fifo_span_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 head = f->head;
    u32 tail = fifo_load_acquire(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        f->msgs[head] = msgs[pushed];
        fifo_clean(&f->msgs[head], sizeof(struct fifo_span_msg));
        head = next;
        pushed++;
    }
    if (pushed) {
        fifo_store_release(&f->head, head);
        fifo_note_activity(pushed);
        fifo_notify(src, dst);
    }
    return pushed;
}

u32 fifo_span_pop_batch(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = fifo_load_acquire(&f->head);
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_span_msg));
        msgs[popped] = f->msgs[tail];
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        fifo_store_release(&f->tail, tail);
        fifo_note_activity(popped);
    }
    return popped;
}

/* ------------------------------------------------------------------ */
/*  A/B span-ring variants: acquire/release + hand-asm descriptor copy */
/* ------------------------------------------------------------------ */

/* Hand-rolled 32-byte (one fifo_span_msg) copy via a single NEON ldp/stp
 * q-register pair — 2 instructions, no loop, no memcpy call. */
static inline void span_copy32_asm(struct fifo_span_msg *d, const struct fifo_span_msg *s) {
    __asm__ volatile(
        "ldp q0, q1, [%1]\n\t"
        "stp q0, q1, [%0]\n\t"
        :
        : "r"(d), "r"(s)
        : "v0", "v1", "memory"
    );
}

/* Idiomatic C: acquire/release ordering, compiler-generated 32B struct copy.
 * Correct SPSC ordering on the 4 inner-shareable A76 cores without DMB-SY:
 *   producer publishes head with STLR (release) after the data stores;
 *   producer reads tail with LDAR (acquire) to see consumer-freed slots. */
u32 fifo_span_push_batch_acqrel(u32 src, u32 dst, const struct fifo_span_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 head = f->head;
    u32 tail = fifo_atomic_load_acquire(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        f->msgs[head] = msgs[pushed];
        fifo_clean(&f->msgs[head], sizeof(struct fifo_span_msg));
        head = next;
        pushed++;
    }
    if (pushed) {
        fifo_atomic_store_release(&f->head, head);
        fifo_note_activity(pushed);
        fifo_notify(src, dst);
    }
    return pushed;
}

u32 fifo_span_pop_batch_acqrel(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = fifo_atomic_load_acquire(&f->head);
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_span_msg));
        msgs[popped] = f->msgs[tail];
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        fifo_atomic_store_release(&f->tail, tail);
        fifo_note_activity(popped);
    }
    return popped;
}

/* Hand-tuned: identical ordering to _acqrel, but the descriptor copy is the
 * explicit ldp/stp q NEON pair instead of compiler struct assignment. */
u32 fifo_span_push_batch_asm(u32 src, u32 dst, const struct fifo_span_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 head = f->head;
    u32 tail = fifo_atomic_load_acquire(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        span_copy32_asm(&f->msgs[head], &msgs[pushed]);
        fifo_clean(&f->msgs[head], sizeof(struct fifo_span_msg));
        head = next;
        pushed++;
    }
    if (pushed) {
        fifo_atomic_store_release(&f->head, head);
        fifo_note_activity(pushed);
        fifo_notify(src, dst);
    }
    return pushed;
}

u32 fifo_span_pop_batch_asm(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = fifo_atomic_load_acquire(&f->head);
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_span_msg));
        span_copy32_asm(&msgs[popped], &f->msgs[tail]);
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        fifo_atomic_store_release(&f->tail, tail);
        fifo_note_activity(popped);
    }
    return popped;
}

/* Inner-shareable DMB scope — same structure as the DMB-SY baseline but with
 * the minimal correct barrier scope for inner-shareable cores:
 *   producer DMB ISHST (data before head) + DSB ISHST (head before doorbell)
 *   consumer DMB ISHLD (head before data reads) + DMB ISH (reads before tail) */
u32 fifo_span_push_batch_ish(u32 src, u32 dst, const struct fifo_span_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 head = f->head;
    u32 tail = fifo_load_acquire(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        f->msgs[head] = msgs[pushed];
        fifo_clean(&f->msgs[head], sizeof(struct fifo_span_msg));
        head = next;
        pushed++;
    }
    if (pushed) {
        fifo_store_release(&f->head, head);
        fifo_note_activity(pushed);
        fifo_notify(src, dst);
    }
    return pushed;
}

u32 fifo_span_pop_batch_ish(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = fifo_load_acquire(&f->head);
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        fifo_invalidate(&f->msgs[tail], sizeof(struct fifo_span_msg));
        msgs[popped] = f->msgs[tail];
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        fifo_store_release(&f->tail, tail);
        fifo_note_activity(popped);
    }
    return popped;
}
