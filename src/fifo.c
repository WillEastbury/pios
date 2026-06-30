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

struct fifo_core_seq {
    volatile u32 seq;
    u32 _pad[15];
} ALIGNED(64);
static struct fifo_core_seq fifo_seq[4];
_Static_assert(sizeof(struct fifo_core_seq) == 64,
               "FIFO sequence records must be one cache line");

static inline void fifo_note_activity(u32 count)
{
    if (count == 0)
        return;
    u32 c = core_id() & 3U;
    fifo_seq[c].seq += count;
    dmb_ishst();
}

u32 fifo_last_sequence(u32 core)
{
    if (core >= 4U)
        return 0;
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
                                (16U * sizeof(struct fifo)) +
                                ((usize)src * 4 + dst) * sizeof(struct fifo_span));
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
    u32 next = (head + 1) & (FIFO_CAPACITY - 1); /* power-of-2 mask */

    if (unlikely(next == f->tail))
        return false;   /* full */

    /* Copy message with NEON (64 bytes = one cache line) */
    simd_memcpy(&f->msgs[head], msg, sizeof(struct fifo_msg));
    dmb();              /* msg visible before head update */
    f->head = next;
    dsb();              /* head visible before event signal */
    fifo_note_activity(1);
    DTRACE(DTRACE_CAT_FIFO, DT_FIFO_PUSH, (src << 8) | dst, msg->type, next, msg->tag);
    sev();              /* wake sleeping cores */
    return true;
}

u32 fifo_push_batch(u32 src, u32 dst, const struct fifo_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo *f = get_fifo(src, dst);
    u32 head = f->head;
    u32 tail = f->tail;
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        simd_memcpy(&f->msgs[head], &msgs[pushed], sizeof(struct fifo_msg));
        head = next;
        pushed++;
    }
    if (pushed) {
        dmb();
        f->head = head;
        dsb();
        fifo_note_activity(pushed);
        sev();
    }
    return pushed;
}

bool fifo_pop(u32 dst, u32 src, struct fifo_msg *msg) {
    if (src >= 4 || dst >= 4) return false;
    struct fifo *f = get_fifo(src, dst);
    u32 tail = f->tail;

    if (unlikely(tail == f->head))
        return false;   /* empty */

    dmb();              /* ensure we read current head before msg */
    simd_memcpy(msg, &f->msgs[tail], sizeof(struct fifo_msg));
    dmb();              /* msg consumed before tail advance */
    f->tail = (tail + 1) & (FIFO_CAPACITY - 1);
    fifo_note_activity(1);
    DTRACE(DTRACE_CAT_FIFO, DT_FIFO_POP, (dst << 8) | src, msg->type, f->tail, msg->tag);
    return true;
}

u32 fifo_pop_batch(u32 dst, u32 src, struct fifo_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo *f = get_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = f->head;
    u32 popped = 0;

    dmb();
    while (popped < max_count && tail != head) {
        simd_memcpy(&msgs[popped], &f->msgs[tail], sizeof(struct fifo_msg));
        tail = (tail + 1) & (FIFO_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        dmb();
        f->tail = tail;
        fifo_note_activity(popped);
    }
    return popped;
}

bool fifo_peek(u32 dst, u32 src, struct fifo_msg *msg) {
    if (src >= 4 || dst >= 4 || !msg) return false;
    struct fifo *f = get_fifo(src, dst);
    u32 tail = f->tail;
    if (unlikely(tail == f->head))
        return false;
    dmb();
    simd_memcpy(msg, &f->msgs[tail], sizeof(struct fifo_msg));
    return true;
}

bool fifo_empty(u32 dst, u32 src) {
    struct fifo *f = get_fifo(src, dst);
    return (f->tail == f->head);
}

u32 fifo_count(u32 dst, u32 src) {
    struct fifo *f = get_fifo(src, dst);
    u32 h = f->head;
    u32 t = f->tail;
    return (h - t) & (FIFO_CAPACITY - 1);
}

u32 fifo_span_push_batch(u32 src, u32 dst, const struct fifo_span_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 head = f->head;
    u32 tail = f->tail;
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        f->msgs[head] = msgs[pushed];
        head = next;
        pushed++;
    }
    if (pushed) {
        dmb_ishst();        /* data stores before head (inner-shareable scope) */
        f->head = head;
        dsb_ishst();        /* head visible before SEV */
        fifo_note_activity(pushed);
        sev();
    }
    return pushed;
}

u32 fifo_span_pop_batch(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = f->head;
    u32 popped = 0;

    dmb_ishld();            /* head read before data reads (inner-shareable) */
    while (popped < max_count && tail != head) {
        msgs[popped] = f->msgs[tail];
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        dmb_ish();          /* data reads before tail store */
        f->tail = tail;
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
    u32 tail = atomic_load32(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        f->msgs[head] = msgs[pushed];
        head = next;
        pushed++;
    }
    if (pushed) {
        atomic_store32(&f->head, head);              /* release: data before head */
        __asm__ volatile("dsb ishst" ::: "memory");  /* head visible before SEV */
        fifo_note_activity(pushed);
        sev();
    }
    return pushed;
}

u32 fifo_span_pop_batch_acqrel(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = atomic_load32(&f->head);              /* acquire: see published data */
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        msgs[popped] = f->msgs[tail];
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        atomic_store32(&f->tail, tail);              /* release: reads before tail */
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
    u32 tail = atomic_load32(&f->tail);
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        span_copy32_asm(&f->msgs[head], &msgs[pushed]);
        head = next;
        pushed++;
    }
    if (pushed) {
        atomic_store32(&f->head, head);
        __asm__ volatile("dsb ishst" ::: "memory");
        fifo_note_activity(pushed);
        sev();
    }
    return pushed;
}

u32 fifo_span_pop_batch_asm(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = atomic_load32(&f->head);
    u32 popped = 0;

    while (popped < max_count && tail != head) {
        span_copy32_asm(&msgs[popped], &f->msgs[tail]);
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        atomic_store32(&f->tail, tail);
        fifo_note_activity(popped);
    }
    return popped;
}

/* Inner-shareable DMB scope — same structure as the DMB-SY baseline but with
 * the minimal correct barrier scope for inner-shareable cores:
 *   producer DMB ISHST (data before head) + DSB ISHST (head before SEV)
 *   consumer DMB ISHLD (head before data reads) + DMB ISH (reads before tail) */
u32 fifo_span_push_batch_ish(u32 src, u32 dst, const struct fifo_span_msg *msgs, u32 count) {
    if (src >= 4 || dst >= 4 || !msgs || count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 head = f->head;
    u32 tail = f->tail;
    u32 pushed = 0;

    while (pushed < count) {
        u32 next = (head + 1) & (FIFO_SPAN_CAPACITY - 1);
        if (unlikely(next == tail))
            break;
        f->msgs[head] = msgs[pushed];
        head = next;
        pushed++;
    }
    if (pushed) {
        dmb_ishst();
        f->head = head;
        dsb_ishst();
        fifo_note_activity(pushed);
        sev();
    }
    return pushed;
}

u32 fifo_span_pop_batch_ish(u32 dst, u32 src, struct fifo_span_msg *msgs, u32 max_count) {
    if (src >= 4 || dst >= 4 || !msgs || max_count == 0) return 0;
    struct fifo_span *f = get_span_fifo(src, dst);
    u32 tail = f->tail;
    u32 head = f->head;
    u32 popped = 0;

    dmb_ishld();
    while (popped < max_count && tail != head) {
        msgs[popped] = f->msgs[tail];
        tail = (tail + 1) & (FIFO_SPAN_CAPACITY - 1);
        popped++;
    }
    if (popped) {
        dmb_ish();
        f->tail = tail;
        fifo_note_activity(popped);
    }
    return popped;
}
