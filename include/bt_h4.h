/*
 * bt_h4.h - offline HCI H4 receive framing and bounded packet ownership.
 *
 * This module knows only the H4 byte framing. It has no transport, controller,
 * firmware, board, or kernel-lifecycle dependency.
 */
#ifndef PIOS_BT_H4_H
#define PIOS_BT_H4_H

#include "types.h"

/* Controller-to-host H4 packet indicators accepted by this receive parser. */
#define BT_H4_TYPE_ACL    0x02U
#define BT_H4_TYPE_SCO    0x03U
#define BT_H4_TYPE_EVENT  0x04U
#define BT_H4_TYPE_ISO    0x05U

/*
 * Packet bytes include the H4 indicator followed by its fixed HCI header and
 * payload. A type's maximum accepted payload is consequently
 * BT_H4_PACKET_CAPACITY - 1 - its fixed header length.
 */
#define BT_H4_PACKET_CAPACITY  1024U
#define BT_H4_QUEUE_CAPACITY   8U

#define BT_H4_EVENT_HEADER_LEN 2U
#define BT_H4_ACL_HEADER_LEN   4U
#define BT_H4_SCO_HEADER_LEN   3U
#define BT_H4_ISO_HEADER_LEN   4U

/*
 * A copied immutable packet. `bytes[0]` is `type`; `byte_count` includes that
 * byte. A consumer obtains a generation-checked handle, copies into storage it
 * owns with bt_h4_packet_copy(), then releases the handle. No queue pointer is
 * handed across that ownership boundary.
 */
struct bt_h4_packet {
    u64 generation;
    u32 byte_count;
    u16 payload_len;
    u8 type;
    u8 header_len;
    u8 bytes[BT_H4_PACKET_CAPACITY];
} ALIGNED(64);

_Static_assert((sizeof(struct bt_h4_packet) % 64U) == 0U,
               "bt_h4 packets must have a cache-line stride");

struct bt_h4_packet_handle {
    u64 generation;
    u32 slot;
    u32 _reserved;
};

/*
 * Producer and consumer control records are deliberately separate cache lines.
 * Exactly one producer may call bt_h4_feed()/bt_h4_abort(); exactly one
 * consumer may call dequeue/copy/release. reset requires both sides quiesced.
 */
struct bt_h4_producer_state {
    u32 head;                 /* next sequence to publish; producer-owned */
    u32 assembly_bytes;
    u32 assembly_payload_len;
    u32 assembly_header_len;
    u64 next_generation;
    u32 assembly_pending;     /* complete frame retained while queue is full */
    u8 _pad[64U - 28U];
} ALIGNED(64);

struct bt_h4_consumer_state {
    u32 tail;                 /* next sequence to release; consumer-owned */
    u32 leased_slot;
    u64 leased_generation;    /* zero means no dequeued record */
    u8 _pad[64U - 16U];
} ALIGNED(64);

_Static_assert(sizeof(struct bt_h4_producer_state) == 64U,
               "bt_h4 producer control must own one cache line");
_Static_assert(sizeof(struct bt_h4_consumer_state) == 64U,
               "bt_h4 consumer control must own one cache line");

struct bt_h4_rx {
    struct bt_h4_producer_state producer;
    struct bt_h4_consumer_state consumer;
    struct bt_h4_packet assembly; /* producer-only; never externally visible */
    struct bt_h4_packet queue[BT_H4_QUEUE_CAPACITY];
} ALIGNED(64);

/*
 * Result of bt_h4_feed().
 *
 * On BT_H4_FEED_OK, every supplied byte was accepted and no completed frame is
 * waiting for queue credit. On BT_H4_FEED_BACKPRESSURE, `*consumed_out` is the
 * exact accepted prefix. That prefix can equal `len`: the final accepted byte
 * may complete a frame retained in `assembly`. Do not submit any remaining
 * input until a consumer releases a record and feed is called again (a zero-
 * length feed is sufficient to publish retained input).
 *
 * INVALID_TYPE consumes only the bad indicator. OVERSIZE consumes the
 * indicator and its complete fixed header, rejects the advertised payload
 * before copying any payload byte, discards that frame, and leaves any later
 * source bytes unconsumed. Both rejection results reset only the partial
 * assembly; already queued packets stay intact.
 */
enum bt_h4_feed_result {
    BT_H4_FEED_OK = 0,
    BT_H4_FEED_BACKPRESSURE,
    BT_H4_FEED_INVALID_TYPE,
    BT_H4_FEED_OVERSIZE,
    BT_H4_FEED_ARGUMENT
};

void bt_h4_init(struct bt_h4_rx *rx);

/*
 * Feed an arbitrary fragment or coalesced sequence of controller receive
 * bytes. `data` may be NULL only when `len` is zero. `consumed_out` is required
 * and is written on every call. This function neither allocates nor discards
 * valid input because queue credit is exhausted.
 */
enum bt_h4_feed_result bt_h4_feed(struct bt_h4_rx *rx, const u8 *data, u32 len,
                                  u32 *consumed_out);

/* Returns one FIFO-ordered record handle. Only one handle may be leased. */
bool bt_h4_dequeue(struct bt_h4_rx *rx, struct bt_h4_packet_handle *out);

/*
 * Copy the immutable leased record to caller-owned storage. Both the slot and
 * generation must match the currently leased record.
 */
bool bt_h4_packet_copy(const struct bt_h4_rx *rx,
                       const struct bt_h4_packet_handle *handle,
                       struct bt_h4_packet *out);

/*
 * Release the currently leased FIFO head. A stale, forged, or out-of-order
 * handle fails closed and leaves queue state unchanged.
 */
bool bt_h4_release(struct bt_h4_rx *rx,
                   const struct bt_h4_packet_handle *handle);

/* Discard only the producer's incomplete or queue-credit-pending assembly. */
void bt_h4_abort(struct bt_h4_rx *rx);

/*
 * Discard assembly and every queued/leased record, invalidating all handles.
 * The producer and consumer must be quiesced before this administrative reset.
 */
void bt_h4_reset(struct bt_h4_rx *rx);

/* Includes a leased head: it continues to occupy bounded queue credit. */
u32 bt_h4_queue_depth(const struct bt_h4_rx *rx);

#endif /* PIOS_BT_H4_H */
