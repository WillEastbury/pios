#pragma once

#include "types.h"
#include "platform.h"

/*
 * EL0 scheduler ABI.
 *
 * The process slot is mapped through the existing Normal-NC IPC alias. EL0
 * owns the command-ring head and entries; the kernel owns the tail and the
 * metadata line. A generation in every command makes slot reuse fail closed.
 */
#define EL0_SCHED_ALIAS_BASE   0x2003000000ULL
#define EL0_SCHED_OFFSET       0x00080000UL
#define EL0_SCHED_SLOT_COUNT   6U
#define EL0_SCHED_RING_DEPTH   8U
#define EL0_SCHED_SLOT_STRIDE  704U

enum {
    EL0_SCHED_OP_PARK = 1U,
    EL0_SCHED_OP_YIELD = 2U,
    EL0_SCHED_OP_EXIT = 3U,
    EL0_SCHED_OP_REPORT = 4U
};

struct el0_sched_meta {
    volatile u32 pid;
    volatile u32 generation;
    volatile u32 state;
    volatile u32 priority_class;
    volatile u32 preemptions;
    volatile u32 _reserved;
    volatile u64 inbound_seq;
    volatile u64 runtime_ticks;
    u8 _pad[24];
} ALIGNED(64);

struct el0_sched_index {
    volatile u32 value;
    u32 _pad[15];
} ALIGNED(64);

struct el0_sched_cmd {
    u32 op;
    u32 generation;
    u64 observed_seq;
    u64 arg0;
    u64 arg1;
    u64 publish_seq;
    u64 _reserved[3];
} ALIGNED(64);

struct el0_sched_ring {
    struct el0_sched_index head;
    struct el0_sched_index tail;
    struct el0_sched_cmd cmds[EL0_SCHED_RING_DEPTH];
} ALIGNED(64);

struct el0_sched_slot {
    struct el0_sched_meta meta;
    struct el0_sched_ring ring;
} ALIGNED(64);

_Static_assert(sizeof(struct el0_sched_meta) == 64U,
               "EL0 scheduler metadata must own one cache line");
_Static_assert(sizeof(struct el0_sched_index) == 64U,
               "EL0 scheduler indexes must own one cache line");
_Static_assert(sizeof(struct el0_sched_cmd) == 64U,
               "EL0 scheduler commands must own one cache line");
_Static_assert(sizeof(struct el0_sched_ring) == 640U,
               "EL0 scheduler ring layout changed");
_Static_assert(sizeof(struct el0_sched_slot) == EL0_SCHED_SLOT_STRIDE,
               "EL0 scheduler slot stride changed");
_Static_assert(EL0_SCHED_OFFSET +
               EL0_SCHED_SLOT_COUNT * sizeof(struct el0_sched_slot) <=
               PIOS_IPC_SHM_SIZE,
               "EL0 scheduler arena exceeds IPC shared memory");

#define EL0_SCHED_GETPID 1U
#define EL0_SCHED_REPORT 2U
#define EL0_SCHED_EXIT 3U
#define EL0_SCHED_PARK 4U

static inline struct el0_sched_slot *el0_sched_slot(void)
{
    u64 address;
    __asm__ volatile("mov %0, x21" : "=r"(address) :: "memory");
    return (struct el0_sched_slot *)(usize)address;
}

static inline bool el0_sched_publish(u32 op, u64 observed_seq,
                                     u64 arg0, u64 arg1)
{
    struct el0_sched_slot *slot = el0_sched_slot();
    u32 head = slot->ring.head.value;
    u32 tail = slot->ring.tail.value;
    if (head >= EL0_SCHED_RING_DEPTH || tail >= EL0_SCHED_RING_DEPTH)
        return false;
    u32 next = (head + 1U) & (EL0_SCHED_RING_DEPTH - 1U);
    if (next == tail)
        return false;
    struct el0_sched_cmd *cmd = &slot->ring.cmds[head];
    cmd->op = op;
    cmd->generation = slot->meta.generation;
    cmd->observed_seq = observed_seq;
    cmd->arg0 = arg0;
    cmd->arg1 = arg1;
    cmd->publish_seq++;
    dmb_ishst();
    slot->ring.head.value = next;
    dmb_ishst();
    return true;
}

static inline u32 el0_sched_getpid(void)
{
    return el0_sched_slot()->meta.pid;
}

static inline void el0_sched_report(u64 value)
{
    while (!el0_sched_publish(EL0_SCHED_OP_REPORT, 0, value, 0))
        __asm__ volatile("wfe" ::: "memory");
}

static inline NORETURN void el0_sched_exit(u64 code)
{
    while (!el0_sched_publish(EL0_SCHED_OP_EXIT, 0, code, 0))
        __asm__ volatile("wfe" ::: "memory");
    for (;;)
        __asm__ volatile("wfe" ::: "memory");
}

static inline void el0_sched_park(void)
{
    struct el0_sched_slot *slot = el0_sched_slot();
    while (!el0_sched_publish(EL0_SCHED_OP_PARK,
                              slot->meta.inbound_seq, 0, 0))
        __asm__ volatile("wfe" ::: "memory");
    __asm__ volatile("wfe" ::: "memory");
}
