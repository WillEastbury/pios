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

/* Generic */
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

void  fifo_init_all(void);
bool  fifo_push(u32 src_core, u32 dst_core, const struct fifo_msg *msg);
bool  fifo_pop(u32 dst_core, u32 src_core, struct fifo_msg *msg);
bool  fifo_empty(u32 dst_core, u32 src_core);
u32   fifo_count(u32 dst_core, u32 src_core);
