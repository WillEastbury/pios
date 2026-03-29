#include "workq.h"

struct work_item {
    work_fn_t fn;
    void *ctx;
};

struct work_ring {
    volatile u32 head;
    volatile u32 tail;
    struct work_item items[WORKQ_DEPTH];
};

static struct work_ring rings[NUM_CORES];

static inline u64 irq_save(void)
{
    u64 daif;
    __asm__ volatile("mrs %0, daif" : "=r"(daif));
    __asm__ volatile("msr daifset, #2" ::: "memory");
    return daif;
}

static inline void irq_restore(u64 daif)
{
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
}

void workq_init_core(void)
{
    u32 c = core_id() & 3U;
    rings[c].head = 0;
    rings[c].tail = 0;
    dmb();
}

bool workq_enqueue(u32 target_core, work_fn_t fn, void *ctx)
{
    if (!fn || target_core >= NUM_CORES)
        return false;

    u32 self = core_id() & 3U;
    if (target_core != self)
        return false; /* cross-core enqueue must be mediated by owner core loop */

    struct work_ring *q = &rings[self];
    u64 daif = irq_save();
    u32 head = q->head;
    u32 next = (head + 1U) & (WORKQ_DEPTH - 1U);
    if (next == q->tail) {
        irq_restore(daif);
        return false;
    }

    q->items[head].fn = fn;
    q->items[head].ctx = ctx;
    dmb();
    q->head = next;
    irq_restore(daif);
    sev();
    return true;
}

u32 workq_drain(u32 budget)
{
    u32 self = core_id() & 3U;
    struct work_ring *q = &rings[self];
    u32 done = 0;

    if (budget == 0)
        budget = WORKQ_DEPTH - 1U;

    while (done < budget) {
        u64 daif = irq_save();
        u32 tail = q->tail;
        if (tail == q->head) {
            irq_restore(daif);
            break;
        }

        struct work_item item = q->items[tail];
        q->tail = (tail + 1U) & (WORKQ_DEPTH - 1U);
        irq_restore(daif);

        if (item.fn)
            item.fn(item.ctx);
        done++;
    }

    return done;
}

u32 workq_pending(u32 core)
{
    if (core >= NUM_CORES)
        return 0;

    struct work_ring *q = &rings[core];
    u32 h = q->head;
    u32 t = q->tail;
    return (h - t) & (WORKQ_DEPTH - 1U);
}
