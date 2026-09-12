#include <stdio.h>
#include <string.h>

#include "types.h"
#include "bt_h4.h"

static int failures;
static int checks;

static void check(int condition, const char *what)
{
    checks++;
    if (!condition) {
        failures++;
        printf("  [FAIL] %s\n", what);
    }
}

static void reset_rx(struct bt_h4_rx *rx)
{
    memset(rx, 0, sizeof(*rx));
    bt_h4_init(rx);
}

static void expect_packet(struct bt_h4_rx *rx, const u8 *expected, u32 bytes,
                          u16 payload_len, const char *what)
{
    struct bt_h4_packet_handle handle;
    struct bt_h4_packet packet;

    check(bt_h4_dequeue(rx, &handle), what);
    if (!bt_h4_packet_copy(rx, &handle, &packet)) {
        check(0, "packet copy succeeds");
        return;
    }
    check(packet.byte_count == bytes, "packet byte count");
    check(packet.payload_len == payload_len, "packet payload length");
    check(memcmp(packet.bytes, expected, bytes) == 0, "packet bytes preserved");
    check(bt_h4_release(rx, &handle), "packet release succeeds");
}

static void test_bytewise_all_types(void)
{
    static const u8 event[] = {0x04, 0x0E, 0x02, 0xAA, 0xBB};
    static const u8 acl[] = {0x02, 0x01, 0x20, 0x03, 0x00, 1, 2, 3};
    static const u8 sco[] = {0x03, 0x01, 0x00, 0x02, 0xCC, 0xDD};
    static const u8 iso[] = {0x05, 0x01, 0x20, 0x02, 0x40, 0xEE, 0xFF};
    const u8 *frames[] = {event, acl, sco, iso};
    const u32 sizes[] = {sizeof(event), sizeof(acl), sizeof(sco), sizeof(iso)};
    const u16 payloads[] = {2U, 3U, 2U, 2U};
    struct bt_h4_rx rx;
    u32 i, j, consumed;

    printf("bytewise framing: event, ACL, SCO, ISO\n");
    for (i = 0U; i < 4U; i++) {
        reset_rx(&rx);
        for (j = 0U; j < sizes[i]; j++) {
            check(bt_h4_feed(&rx, frames[i] + j, 1U, &consumed) == BT_H4_FEED_OK,
                  "bytewise feed succeeds");
            check(consumed == 1U, "bytewise byte consumed");
        }
        check(bt_h4_queue_depth(&rx) == 1U, "complete frame queued");
        expect_packet(&rx, frames[i], sizes[i], payloads[i], "dequeue frame");
    }
}

static void test_coalesced_and_lengths(void)
{
    static const u8 frames[] = {
        0x04, 0x0F, 0x00,
        0x02, 0x01, 0x20, 0x02, 0x00, 0xA1, 0xA2,
        0x03, 0x01, 0x00, 0x01, 0xA3
    };
    static const u8 event[] = {0x04, 0x0F, 0x00};
    static const u8 acl[] = {0x02, 0x01, 0x20, 0x02, 0x00, 0xA1, 0xA2};
    static const u8 sco[] = {0x03, 0x01, 0x00, 0x01, 0xA3};
    struct bt_h4_rx rx;
    u32 consumed;

    printf("coalesced frames and declared lengths\n");
    reset_rx(&rx);
    check(bt_h4_feed(&rx, frames, sizeof(frames), &consumed) == BT_H4_FEED_OK,
          "coalesced feed succeeds");
    check(consumed == sizeof(frames), "coalesced feed consumes all");
    check(bt_h4_queue_depth(&rx) == 3U, "three frames queued");
    expect_packet(&rx, event, sizeof(event), 0U, "event order");
    expect_packet(&rx, acl, sizeof(acl), 2U, "ACL order");
    expect_packet(&rx, sco, sizeof(sco), 1U, "SCO order");
}

static void test_rejections(void)
{
    static const u8 invalid[] = {0x01, 0x04, 0x0E, 0x00};
    static const u8 acl_oversize[] = {0x02, 0x01, 0x20, 0xFC, 0x03, 0xAA};
    static const u8 iso_oversize[] = {0x05, 0x01, 0x00, 0xFF, 0x43};
    static const u8 event[] = {0x04, 0x0E, 0x00};
    struct bt_h4_rx rx;
    u32 consumed;

    printf("malformed indicator/header rejection\n");
    reset_rx(&rx);
    check(bt_h4_feed(&rx, invalid, sizeof(invalid), &consumed) ==
          BT_H4_FEED_INVALID_TYPE, "command indicator is rejected on receive");
    check(consumed == 1U, "only bad indicator consumed");
    check(bt_h4_feed(&rx, invalid + consumed, sizeof(invalid) - consumed, &consumed) ==
          BT_H4_FEED_OK, "input after bad indicator remains usable");
    check(consumed == 3U, "later valid frame consumed");
    expect_packet(&rx, event, sizeof(event), 0U, "valid frame after bad type");

    reset_rx(&rx);
    check(bt_h4_feed(&rx, acl_oversize, sizeof(acl_oversize), &consumed) ==
          BT_H4_FEED_OVERSIZE, "oversize ACL header rejected");
    check(consumed == 5U, "oversize rejects after exact ACL header");
    check(bt_h4_queue_depth(&rx) == 0U, "oversize does not queue packet");

    reset_rx(&rx);
    check(bt_h4_feed(&rx, iso_oversize, sizeof(iso_oversize), &consumed) ==
          BT_H4_FEED_OVERSIZE, "oversize ISO header rejected");
    check(consumed == 5U, "ISO header length uses low fourteen bits");
    check(bt_h4_feed(&rx, NULL, 1U, &consumed) == BT_H4_FEED_ARGUMENT,
          "null nonempty source rejected");
}

static void test_backpressure_order_and_retention(void)
{
    u8 frames[(BT_H4_QUEUE_CAPACITY + 1U) * 3U];
    struct bt_h4_rx rx;
    struct bt_h4_packet_handle handle;
    struct bt_h4_packet packet;
    u32 i, consumed;

    printf("full queue applies explicit backpressure without loss\n");
    for (i = 0U; i < BT_H4_QUEUE_CAPACITY + 1U; i++) {
        frames[i * 3U] = BT_H4_TYPE_EVENT;
        frames[i * 3U + 1U] = (u8)i;
        frames[i * 3U + 2U] = 0U;
    }
    reset_rx(&rx);
    check(bt_h4_feed(&rx, frames, sizeof(frames), &consumed) ==
          BT_H4_FEED_BACKPRESSURE, "full queue reports backpressure");
    check(consumed == sizeof(frames), "completed retained frame is accepted");
    check(bt_h4_queue_depth(&rx) == BT_H4_QUEUE_CAPACITY, "queue reaches fixed capacity");

    for (i = 0U; i < BT_H4_QUEUE_CAPACITY; i++) {
        check(bt_h4_dequeue(&rx, &handle), "queued handle available in order");
        check(bt_h4_packet_copy(&rx, &handle, &packet), "queued packet copy");
        check(packet.bytes[1] == (u8)i, "queue preserves FIFO order");
        check(bt_h4_release(&rx, &handle), "release makes one credit");
    }
    check(bt_h4_feed(&rx, NULL, 0U, &consumed) == BT_H4_FEED_OK,
          "zero-length feed publishes retained packet");
    check(consumed == 0U, "zero-length feed consumes zero");
    check(bt_h4_dequeue(&rx, &handle), "retained packet becomes available");
    check(bt_h4_packet_copy(&rx, &handle, &packet), "retained packet copy");
    check(packet.bytes[1] == BT_H4_QUEUE_CAPACITY, "retained packet unchanged");
    check(bt_h4_release(&rx, &handle), "retained release");
}

static void test_abort_and_reset(void)
{
    static const u8 partial[] = {0x04, 0x0E};
    static const u8 complete[] = {0x04, 0x0F, 0x00};
    struct bt_h4_rx rx;
    struct bt_h4_packet_handle stale, handle;
    u32 consumed;

    printf("abort discards assembly; reset invalidates queued handles\n");
    reset_rx(&rx);
    check(bt_h4_feed(&rx, partial, sizeof(partial), &consumed) == BT_H4_FEED_OK,
          "partial frame accepted");
    bt_h4_abort(&rx);
    check(bt_h4_feed(&rx, complete, sizeof(complete), &consumed) == BT_H4_FEED_OK,
          "feed after abort starts a new frame");
    check(bt_h4_dequeue(&rx, &handle), "post-abort packet queued");
    check(bt_h4_release(&rx, &handle), "post-abort packet released");

    check(bt_h4_feed(&rx, complete, sizeof(complete), &consumed) == BT_H4_FEED_OK,
          "packet before reset queued");
    check(bt_h4_dequeue(&rx, &stale), "handle before reset");
    bt_h4_reset(&rx);
    check(bt_h4_queue_depth(&rx) == 0U, "reset empties queue");
    check(!bt_h4_packet_copy(&rx, &stale, &(struct bt_h4_packet){0}),
          "reset invalidates stale handle");
    check(!bt_h4_release(&rx, &stale), "stale release fails closed");
}

static void test_offline_source_gate(void)
{
    static const char *files[] = {"src/bt_h4.c", "include/bt_h4.h"};
    static const char *forbidden[] = {"platform", "uart", "mmio", "mailbox", "hcd"};
    char buffer[32768];
    u32 i, j;

    printf("offline source dependency gate\n");
    for (i = 0U; i < 2U; i++) {
        FILE *f = fopen(files[i], "rb");
        size_t n;
        check(f != NULL, "source gate file opens");
        if (!f)
            continue;
        n = fread(buffer, 1U, sizeof(buffer) - 1U, f);
        fclose(f);
        buffer[n] = '\0';
        for (j = 0U; j < 5U; j++)
            check(strstr(buffer, forbidden[j]) == NULL,
                  "offline implementation has no forbidden hardware reference");
    }
}

int main(void)
{
    test_bytewise_all_types();
    test_coalesced_and_lengths();
    test_rejections();
    test_backpressure_order_and_retention();
    test_abort_and_reset();
    test_offline_source_gate();

    if (failures) {
        printf("bt_h4: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("bt_h4: all %d checks passed\n", checks);
    return 0;
}
