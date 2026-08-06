#pragma once
#include "types.h"

/*
 * Lock-free single-producer single-consumer (SPSC) inter-core FIFO.
 * One FIFO per ordered (src, dst) core pair = 12 active FIFOs.
 * FIFOs live in shared memory at SHARED_FIFO_BASE.
 * No locks, no atomics - just memory barriers + SEV/WFE.
 */

#define FIFO_CAPACITY   512
#define FIFO_MSG_SIZE   64
#define FIFO_SPAN_CAPACITY 256

/* ---- Message types ---- */

/* Disk I/O (user cores → core 1, replies core 1 → user) */
#define MSG_DISK_READ       1   /* param=LBA, buffer=dst ptr, length=block count */
#define MSG_DISK_WRITE      2   /* param=LBA, buffer=src ptr, length=block count */
#define MSG_DISK_DONE       3   /* status=0 ok, buffer=data ptr */
#define MSG_DISK_ERROR      4   /* status=error code */

/* Network I/O (user cores → core 0, replies core 0 → user) */
#define MSG_NET_UDP_SEND    10  /* param=dst_ip, buffer=payload, length=payload_len,
                                   tag = (src_port << 16) | dst_port */
#define MSG_NET_UDP_RECV    11  /* param=src_ip, buffer=payload, length=payload_len,
                                   tag = (src_port << 16) | dst_port */
#define MSG_NET_UDP_DONE    12  /* status=0 ok, status=1 no neighbor */
#define MSG_NET_STATS       13  /* buffer=pointer to net_stats_t */
#define MSG_NET_LINK_UP     14
#define MSG_NET_LINK_DOWN   15
#define MSG_DNS_RESOLVE      16  /* buffer=hostname cstring ptr (sender core RAM),
                                    length=strlen+1, tag=request id (echoed back).
                                    Core 0 owns dns.c state; never call dns_resolve()
                                    or net_poll() directly from a user core. */
#define MSG_DNS_RESOLVE_DONE 17  /* status=0 ok/1 fail/2 busy(retry), param=host-order
                                    IPv4 result, tag=echoed request id */

/* Generic */
#define MSG_BENCH_BATCH     252
#define MSG_BENCH_ECHO      253
#define MSG_PING            254
#define MSG_ACK             255

struct fifo_msg {
    u32 type;
    u32 param;
    u64 buffer;
    u32 length;
    u32 status;
    u64 tag;
    u64 timestamp;      /* cntvct_el0 at send time */
    u64 _reserved;      /* pad to 64 bytes */
} ALIGNED(64);

struct fifo {
    volatile u32 head ALIGNED(64);  /* producer writes */
    volatile u32 tail ALIGNED(64);  /* consumer writes */
    u8 _pad[128 - 2*64];           /* avoid false sharing */
    struct fifo_msg msgs[FIFO_CAPACITY] ALIGNED(64);
};

struct fifo_span_msg {
    u64 addr;
    u64 tag;
    u32 len;
    u32 flags;
    u32 aux;
    u32 _pad;
    u64 _reserved[4];
} ALIGNED(64);

_Static_assert(sizeof(struct fifo_span_msg) == 64,
               "FIFO span descriptors must be one cache line");

struct fifo_span {
    volatile u32 head ALIGNED(64);
    volatile u32 tail ALIGNED(64);
    u8 _pad[128 - 2*64];
    struct fifo_span_msg msgs[FIFO_SPAN_CAPACITY] ALIGNED(64);
};

void  fifo_init_all(void);
bool  fifo_push(u32 src_core, u32 dst_core, const struct fifo_msg *msg);
u32   fifo_push_batch(u32 src_core, u32 dst_core, const struct fifo_msg *msgs, u32 count);
bool  fifo_peek(u32 dst_core, u32 src_core, struct fifo_msg *msg);
bool  fifo_pop(u32 dst_core, u32 src_core, struct fifo_msg *msg);
u32   fifo_pop_batch(u32 dst_core, u32 src_core, struct fifo_msg *msgs, u32 max_count);
bool  fifo_empty(u32 dst_core, u32 src_core);
u32   fifo_count(u32 dst_core, u32 src_core);
u32   fifo_span_push_batch(u32 src_core, u32 dst_core, const struct fifo_span_msg *msgs, u32 count);
u32   fifo_span_pop_batch(u32 dst_core, u32 src_core, struct fifo_span_msg *msgs, u32 max_count);

/* A/B variants of the span ring (same ABI). Used by the IPC benchmark to
 * compare memory-ordering + descriptor-copy strategies head to head:
 *   (baseline above) DMB-SY full barriers + compiler 64B struct copy
 *   _acqrel          LDAR/STLR acquire-release + DSB ISHST, compiler copy
 *   _asm             same ordering, hand-rolled ldp/stp q (NEON) 64B copy */
u32   fifo_span_push_batch_acqrel(u32 src_core, u32 dst_core, const struct fifo_span_msg *msgs, u32 count);
u32   fifo_span_pop_batch_acqrel(u32 dst_core, u32 src_core, struct fifo_span_msg *msgs, u32 max_count);
u32   fifo_span_push_batch_asm(u32 src_core, u32 dst_core, const struct fifo_span_msg *msgs, u32 count);
u32   fifo_span_pop_batch_asm(u32 dst_core, u32 src_core, struct fifo_span_msg *msgs, u32 max_count);
u32   fifo_span_push_batch_ish(u32 src_core, u32 dst_core, const struct fifo_span_msg *msgs, u32 count);
u32   fifo_span_pop_batch_ish(u32 dst_core, u32 src_core, struct fifo_span_msg *msgs, u32 max_count);
u32   fifo_last_sequence(u32 core);
/* Mark `core` ready to receive FIFO publication doorbells via GIC SGI.
 * Called by the target core only after its SGI handler and banked enable are live. */
void  fifo_irq_enable(u32 core);
bool  fifo_irq_ready(u32 core);
u32   fifo_irq_sent(u32 core);
