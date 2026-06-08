/*
 * fifo.c - Lock-free SPSC inter-core FIFOs in shared memory
 * 16 channels (4×4 grid). fifo[i][i] unused but allocated.
 * Lives at SHARED_FIFO_BASE to keep per-core RAM truly private.
 */

#include "fifo.h"
#include "simd.h"
#include "core_env.h"
#include "fb.h"

/* FIFO pool in shared memory: 4×4 = 16 fifo structs */
static inline struct fifo *get_fifo(u32 src, u32 dst) {
    return (struct fifo *)(SHARED_FIFO_BASE +
                           ((usize)src * 4 + dst) * sizeof(struct fifo));
}

void fifo_init_all(void) {
    /* Zero the entire shared FIFO region with NEON */
    simd_zero((void *)SHARED_FIFO_BASE, SHARED_FIFO_SIZE);
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
