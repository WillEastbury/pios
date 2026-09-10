/*
 * bt_h4.c - offline HCI H4 receive framing. See include/bt_h4.h.
 */
#include "types.h"
#include "bt_h4.h"

_Static_assert((BT_H4_QUEUE_CAPACITY & (BT_H4_QUEUE_CAPACITY - 1U)) == 0U,
               "bt_h4 queue capacity must be a power of two");

static u32 h4_header_len(u8 type)
{
    if (type == BT_H4_TYPE_EVENT)
        return BT_H4_EVENT_HEADER_LEN;
    if (type == BT_H4_TYPE_ACL)
        return BT_H4_ACL_HEADER_LEN;
    if (type == BT_H4_TYPE_SCO)
        return BT_H4_SCO_HEADER_LEN;
    if (type == BT_H4_TYPE_ISO)
        return BT_H4_ISO_HEADER_LEN;
    return 0U;
}

static u32 h4_payload_len(const struct bt_h4_packet *packet)
{
    const u8 *header = packet->bytes + 1U;

    if (packet->type == BT_H4_TYPE_EVENT || packet->type == BT_H4_TYPE_SCO)
        return header[packet->header_len - 1U];
    if (packet->type == BT_H4_TYPE_ACL)
        return (u32)header[2] | ((u32)header[3] << 8);

    /* ISO length uses its low fourteen bits; the upper bits are PB/TS flags. */
    return ((u32)header[2] | ((u32)header[3] << 8)) & 0x3FFFU;
}

static void h4_clear_assembly(struct bt_h4_rx *rx)
{
    rx->producer.assembly_bytes = 0U;
    rx->producer.assembly_payload_len = 0U;
    rx->producer.assembly_header_len = 0U;
    rx->producer.assembly_pending = false;
    rx->assembly.byte_count = 0U;
    rx->assembly.payload_len = 0U;
    rx->assembly.type = 0U;
    rx->assembly.header_len = 0U;
}

static bool h4_queue_full(const struct bt_h4_rx *rx)
{
    dmb_ishld();
    return rx->producer.head - rx->consumer.tail >= BT_H4_QUEUE_CAPACITY;
}

static void h4_copy_packet(struct bt_h4_packet *dst,
                           const struct bt_h4_packet *src)
{
    u32 i;

    dst->generation = src->generation;
    dst->byte_count = src->byte_count;
    dst->payload_len = src->payload_len;
    dst->type = src->type;
    dst->header_len = src->header_len;
    for (i = 0U; i < src->byte_count; i++)
        dst->bytes[i] = src->bytes[i];
}

static bool h4_publish_assembly(struct bt_h4_rx *rx)
{
    struct bt_h4_packet *slot;
    u32 index;

    if (!rx->producer.assembly_pending)
        return true;
    if (h4_queue_full(rx))
        return false;

    index = rx->producer.head & (BT_H4_QUEUE_CAPACITY - 1U);
    slot = &rx->queue[index];
    rx->assembly.generation = rx->producer.next_generation++;
    if (rx->producer.next_generation == 0U)
        rx->producer.next_generation = 1U;
    h4_copy_packet(slot, &rx->assembly);

    /* Immutable packet bytes before the producer publication sequence. */
    dmb_ishst();
    rx->producer.head++;
    dmb_ishst();
    h4_clear_assembly(rx);
    return true;
}

void bt_h4_init(struct bt_h4_rx *rx)
{
    u32 i;

    if (!rx)
        return;
    rx->producer.head = 0U;
    rx->producer.next_generation = 1U;
    h4_clear_assembly(rx);
    rx->consumer.tail = 0U;
    rx->consumer.leased_slot = 0U;
    rx->consumer.leased_generation = 0U;
    for (i = 0U; i < BT_H4_QUEUE_CAPACITY; i++)
        rx->queue[i].generation = 0U;
    dmb_ishst();
}

enum bt_h4_feed_result bt_h4_feed(struct bt_h4_rx *rx, const u8 *data, u32 len,
                                  u32 *consumed_out)
{
    u32 consumed = 0U;

    if (!consumed_out)
        return BT_H4_FEED_ARGUMENT;
    *consumed_out = 0U;
    if (!rx || (!data && len != 0U))
        return BT_H4_FEED_ARGUMENT;

    if (!h4_publish_assembly(rx))
        return BT_H4_FEED_BACKPRESSURE;

    while (consumed < len) {
        struct bt_h4_packet *packet = &rx->assembly;
        u32 header_len;

        if (rx->producer.assembly_bytes == 0U) {
            header_len = h4_header_len(data[consumed]);
            if (header_len == 0U) {
                consumed++;
                *consumed_out = consumed;
                return BT_H4_FEED_INVALID_TYPE;
            }
            packet->type = data[consumed];
            packet->header_len = (u8)header_len;
            packet->bytes[0] = data[consumed++];
            packet->byte_count = 1U;
            rx->producer.assembly_bytes = 1U;
            rx->producer.assembly_header_len = header_len;
            continue;
        }

        packet->bytes[rx->producer.assembly_bytes++] = data[consumed++];
        packet->byte_count = rx->producer.assembly_bytes;

        if (rx->producer.assembly_bytes == 1U + rx->producer.assembly_header_len) {
            u32 payload_len = h4_payload_len(packet);
            u32 maximum = BT_H4_PACKET_CAPACITY - 1U -
                          rx->producer.assembly_header_len;

            if (payload_len > maximum) {
                h4_clear_assembly(rx);
                *consumed_out = consumed;
                return BT_H4_FEED_OVERSIZE;
            }
            packet->payload_len = (u16)payload_len;
            rx->producer.assembly_payload_len = payload_len;
            if (payload_len == 0U)
                rx->producer.assembly_pending = true;
        }

        if (rx->producer.assembly_bytes ==
            1U + rx->producer.assembly_header_len +
            rx->producer.assembly_payload_len) {
            rx->producer.assembly_pending = true;
            if (!h4_publish_assembly(rx)) {
                *consumed_out = consumed;
                return BT_H4_FEED_BACKPRESSURE;
            }
        }
    }

    *consumed_out = consumed;
    return BT_H4_FEED_OK;
}

bool bt_h4_dequeue(struct bt_h4_rx *rx, struct bt_h4_packet_handle *out)
{
    u32 slot;
    const struct bt_h4_packet *packet;

    if (!rx || !out || rx->consumer.leased_generation != 0U)
        return false;
    dmb_ishld();
    if (rx->consumer.tail == rx->producer.head)
        return false;

    slot = rx->consumer.tail & (BT_H4_QUEUE_CAPACITY - 1U);
    packet = &rx->queue[slot];
    if (packet->generation == 0U)
        return false;
    rx->consumer.leased_slot = slot;
    rx->consumer.leased_generation = packet->generation;
    out->slot = slot;
    out->generation = packet->generation;
    out->_reserved = 0U;
    return true;
}

bool bt_h4_packet_copy(const struct bt_h4_rx *rx,
                       const struct bt_h4_packet_handle *handle,
                       struct bt_h4_packet *out)
{
    const struct bt_h4_packet *packet;

    if (!rx || !handle || !out || handle->slot >= BT_H4_QUEUE_CAPACITY)
        return false;
    if (handle->slot != rx->consumer.leased_slot ||
        handle->generation == 0U ||
        handle->generation != rx->consumer.leased_generation)
        return false;
    packet = &rx->queue[handle->slot];
    if (packet->generation != handle->generation ||
        packet->byte_count > BT_H4_PACKET_CAPACITY)
        return false;
    h4_copy_packet(out, packet);
    return true;
}

bool bt_h4_release(struct bt_h4_rx *rx,
                   const struct bt_h4_packet_handle *handle)
{
    if (!rx || !handle || handle->slot >= BT_H4_QUEUE_CAPACITY)
        return false;
    if (handle->slot != rx->consumer.leased_slot ||
        handle->generation == 0U ||
        handle->generation != rx->consumer.leased_generation ||
        rx->queue[handle->slot].generation != handle->generation)
        return false;

    /* Poison before returning producer credit; the next use gets a new ID. */
    rx->queue[handle->slot].generation = 0U;
    rx->consumer.leased_generation = 0U;
    dmb_ishst();
    rx->consumer.tail++;
    dmb_ishst();
    return true;
}

void bt_h4_abort(struct bt_h4_rx *rx)
{
    if (!rx)
        return;
    h4_clear_assembly(rx);
    dmb_ishst();
}

void bt_h4_reset(struct bt_h4_rx *rx)
{
    u64 next_generation;
    u32 i;

    if (!rx)
        return;
    next_generation = rx->producer.next_generation + 1U;
    if (next_generation == 0U)
        next_generation = 1U;
    rx->producer.head = 0U;
    rx->producer.next_generation = next_generation;
    h4_clear_assembly(rx);
    rx->consumer.tail = 0U;
    rx->consumer.leased_slot = 0U;
    rx->consumer.leased_generation = 0U;
    for (i = 0U; i < BT_H4_QUEUE_CAPACITY; i++)
        rx->queue[i].generation = 0U;
    dmb_ishst();
}

u32 bt_h4_queue_depth(const struct bt_h4_rx *rx)
{
    if (!rx)
        return 0U;
    dmb_ishld();
    return rx->producer.head - rx->consumer.tail;
}
