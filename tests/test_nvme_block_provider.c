/*
 * Host tests for the offline callback-backed namespace provider.
 *
 * The persistence helpers below are a test-only fixed superblock/record
 * model. They deliberately do not implement or exercise a PIOS filesystem.
 */
#include "types.h"
#include <stdio.h>
#include <string.h>

#include "nvme_block_provider.h"

#define FAKE_BLOCKS 64U
#define TEST_BLOCK_BYTES 512U
#define TEST_PERSISTENCE_MAGIC 0x54504231U
#define TEST_RECORD_MAGIC 0x54505231U
#define TEST_RECORD_LIMIT 16U

struct fake_media {
    u8 blocks[FAKE_BLOCKS][TEST_BLOCK_BYTES];
    u32 read_calls;
    u32 write_calls;
    bool present;
    bool short_transfer;
    bool false_transfer;
    bool remove_after_partial_write;
};

struct test_provider_persistence_superblock {
    u32 magic;
    u32 record_count;
    u32 generation;
    u32 checksum;
    u8 reserved[TEST_BLOCK_BYTES - 16U];
};

struct test_provider_persistence_record {
    u32 magic;
    u32 id;
    u32 generation;
    u32 checksum;
    u8 payload[TEST_BLOCK_BYTES - 16U];
};

struct test_provider_persistence_state {
    u32 record_count;
    u32 generation;
    u32 ids[TEST_RECORD_LIMIT];
};

_Static_assert(sizeof(struct test_provider_persistence_superblock) ==
               TEST_BLOCK_BYTES, "test superblock must occupy one block");
_Static_assert(sizeof(struct test_provider_persistence_record) ==
               TEST_BLOCK_BYTES, "test record must occupy one block");

static u32 checks;
static u32 failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static bool fake_transfer(struct fake_media *media, bool writing, u64 lba,
                          u32 block_count,
                          struct nvme_block_provider_span *span)
{
    u64 bytes = (u64)block_count * TEST_BLOCK_BYTES;

    if (!media || !span || !span->bytes || span->capacity != bytes ||
        lba >= FAKE_BLOCKS || (u64)block_count > FAKE_BLOCKS - lba ||
        !media->present)
        return false;
    if (writing)
        media->write_calls++;
    else
        media->read_calls++;
    if (media->remove_after_partial_write && writing) {
        u64 partial = bytes / 2U;

        memcpy(&media->blocks[lba][0], span->bytes, (usize)partial);
        span->used = partial;
        media->present = false;
        return false;
    }
    if (writing)
        memcpy(&media->blocks[lba][0], span->bytes, (usize)bytes);
    else
        memcpy(span->bytes, &media->blocks[lba][0], (usize)bytes);
    span->used = media->short_transfer ? bytes - 1U : bytes;
    return !media->false_transfer;
}

static bool fake_read(void *context, u64 lba, u32 block_count,
                      struct nvme_block_provider_span *span)
{
    return fake_transfer((struct fake_media *)context, false, lba,
                         block_count, span);
}

static bool fake_write(void *context, u64 lba, u32 block_count,
                       struct nvme_block_provider_span *span)
{
    return fake_transfer((struct fake_media *)context, true, lba,
                         block_count, span);
}

static struct nvme_block_provider_geometry geometry_make(u64 epoch,
                                                          bool read_only)
{
    return (struct nvme_block_provider_geometry) {
        .logical_blocks = FAKE_BLOCKS,
        .capacity_bytes = (u64)FAKE_BLOCKS * TEST_BLOCK_BYTES,
        .namespace_generation = 7U,
        .instance_epoch = epoch,
        .namespace_id = 1U,
        .block_bytes = TEST_BLOCK_BYTES,
        .max_transfer_blocks = FAKE_BLOCKS,
        .read_only = read_only ? 1U : 0U,
    };
}

static struct nvme_block_provider_callbacks callbacks_make(
    struct fake_media *media)
{
    return (struct nvme_block_provider_callbacks) {
        .read = fake_read,
        .write = fake_write,
        .context = media,
    };
}

static enum nvme_block_provider_result provider_make(
    struct nvme_block_provider *provider,
    struct nvme_block_provider_handle *handle, struct fake_media *media,
    u64 epoch, bool read_only)
{
    struct nvme_block_provider_geometry geometry = geometry_make(epoch,
                                                                  read_only);
    struct nvme_block_provider_callbacks callbacks = callbacks_make(media);

    memset(provider, 0, sizeof(*provider));
    return nvme_block_provider_init(provider, &geometry, &callbacks, 0U,
                                    handle);
}

static u32 test_superblock_checksum(
    const struct test_provider_persistence_superblock *super)
{
    return super->magic ^ super->record_count ^ super->generation ^
           0x53554231U;
}

static u32 test_record_checksum(
    const struct test_provider_persistence_record *record)
{
    u32 value = record->magic ^ record->id ^ record->generation ^
                0x52454331U;
    u32 i;

    for (i = 0U; i < sizeof(record->payload); i++)
        value = (value << 5) ^ (value >> 2) ^ record->payload[i];
    return value;
}

static bool test_persistence_read_block(struct nvme_block_provider *provider,
                                        const struct nvme_block_provider_handle *handle,
                                        u64 lba, void *out)
{
    struct nvme_block_provider_span span = {
        .bytes = out,
        .capacity = TEST_BLOCK_BYTES,
    };
    return nvme_block_provider_read(provider, handle, 0U, lba, 1U, &span) ==
               NVME_BLOCK_PROVIDER_OK &&
           span.used == TEST_BLOCK_BYTES;
}

static bool test_persistence_write_block(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u64 lba,
    const void *input)
{
    struct nvme_block_provider_span span = {
        .bytes = (u8 *)input,
        .capacity = TEST_BLOCK_BYTES,
    };
    return nvme_block_provider_write(provider, handle, 0U, lba, 1U, &span) ==
               NVME_BLOCK_PROVIDER_OK &&
           span.used == TEST_BLOCK_BYTES;
}

static bool test_provider_persistence_format(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle)
{
    struct test_provider_persistence_superblock super = {
        .magic = TEST_PERSISTENCE_MAGIC,
        .record_count = 0U,
        .generation = 1U,
    };

    super.checksum = test_superblock_checksum(&super);
    return test_persistence_write_block(provider, handle, 0U, &super);
}

static bool test_provider_persistence_mount(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle,
    struct test_provider_persistence_state *state)
{
    struct test_provider_persistence_superblock super;
    struct test_provider_persistence_record record;
    u32 i;

    if (!state)
        return false;
    memset(state, 0, sizeof(*state));
    if (!test_persistence_read_block(provider, handle, 0U, &super) ||
        super.magic != TEST_PERSISTENCE_MAGIC ||
        super.record_count > TEST_RECORD_LIMIT ||
        super.record_count > FAKE_BLOCKS - 1U ||
        super.checksum != test_superblock_checksum(&super))
        return false;
    state->record_count = super.record_count;
    state->generation = super.generation;
    for (i = 0U; i < super.record_count; i++) {
        if (!test_persistence_read_block(provider, handle, (u64)i + 1U,
                                         &record) ||
            record.magic != TEST_RECORD_MAGIC ||
            record.generation != super.generation ||
            record.checksum != test_record_checksum(&record))
            return false;
        state->ids[i] = record.id;
    }
    return true;
}

static bool test_provider_persistence_put(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle,
    struct test_provider_persistence_state *state, u32 id,
    const u8 *payload, u32 payload_bytes)
{
    struct test_provider_persistence_superblock super;
    struct test_provider_persistence_record record;

    if (!state || !payload || payload_bytes > sizeof(record.payload) - 1U ||
        state->record_count >= TEST_RECORD_LIMIT ||
        state->record_count >= FAKE_BLOCKS - 1U)
        return false;
    memset(&record, 0, sizeof(record));
    record.magic = TEST_RECORD_MAGIC;
    record.id = id;
    record.generation = state->generation;
    record.payload[0] = (u8)payload_bytes;
    memcpy(record.payload + 1U, payload, payload_bytes);
    record.checksum = test_record_checksum(&record);
    if (!test_persistence_write_block(provider, handle,
                                      (u64)state->record_count + 1U,
                                      &record))
        return false;

    super.magic = TEST_PERSISTENCE_MAGIC;
    super.record_count = state->record_count + 1U;
    super.generation = state->generation;
    super.checksum = test_superblock_checksum(&super);
    memset(super.reserved, 0, sizeof(super.reserved));
    if (!test_persistence_write_block(provider, handle, 0U, &super))
        return false;
    state->ids[state->record_count++] = id;
    return true;
}

static void test_initialization_and_geometry(void)
{
    struct fake_media media = { .present = true };
    struct nvme_block_provider provider;
    struct nvme_block_provider_handle handle;
    struct nvme_block_provider_geometry geometry = geometry_make(11U, false);
    struct nvme_block_provider_geometry copied;
    struct nvme_block_provider_status status;
    struct nvme_block_provider_callbacks callbacks = callbacks_make(&media);

    memset(&provider, 0, sizeof(provider));
    CHECK(nvme_block_provider_init(&provider, &geometry, &callbacks, 1U,
                                   &handle) == NVME_BLOCK_PROVIDER_INVALID);
    CHECK(handle.magic == 0U);
    geometry.capacity_bytes--;
    CHECK(nvme_block_provider_init(&provider, &geometry, &callbacks, 0U,
                                   &handle) == NVME_BLOCK_PROVIDER_INVALID);
    geometry = geometry_make(0U, false);
    CHECK(nvme_block_provider_init(&provider, &geometry, &callbacks, 0U,
                                   &handle) == NVME_BLOCK_PROVIDER_INVALID);
    geometry = geometry_make(11U, false);
    callbacks.read = 0;
    CHECK(nvme_block_provider_init(&provider, &geometry, &callbacks, 0U,
                                   &handle) == NVME_BLOCK_PROVIDER_INVALID);
    callbacks = callbacks_make(&media);
    callbacks.write = 0;
    CHECK(nvme_block_provider_init(&provider, &geometry, &callbacks, 0U,
                                   &handle) == NVME_BLOCK_PROVIDER_INVALID);
    callbacks = callbacks_make(&media);
    provider.control.magic = 1U;
    CHECK(nvme_block_provider_init(&provider, &geometry, &callbacks, 0U,
                                   &handle) == NVME_BLOCK_PROVIDER_INVALID);
    CHECK(provider_make(&provider, &handle, &media, 11U, false) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(handle.magic == NVME_BLOCK_PROVIDER_HANDLE_MAGIC);
    CHECK(handle.instance_epoch == 11U && handle.namespace_generation == 7U);
    CHECK(nvme_block_provider_geometry_get(&provider, 0U, &copied) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(copied.capacity_bytes == (u64)FAKE_BLOCKS * TEST_BLOCK_BYTES);
    CHECK(nvme_block_provider_geometry_get(&provider, 1U, &copied) ==
          NVME_BLOCK_PROVIDER_OWNER);
    CHECK(nvme_block_provider_status_get(&provider, 0U, &status) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(status.state == NVME_BLOCK_PROVIDER_ACTIVE &&
          status.owner_core == 0U);
    CHECK(nvme_block_provider_open(&provider, 0U, &handle) ==
          NVME_BLOCK_PROVIDER_OK);
}

static void test_persistence_and_ranges(void)
{
    struct fake_media media = { .present = true };
    struct nvme_block_provider provider;
    struct nvme_block_provider reload;
    struct nvme_block_provider limited;
    struct nvme_block_provider_handle handle;
    struct nvme_block_provider_handle reload_handle;
    struct nvme_block_provider_handle limited_handle;
    struct nvme_block_provider_geometry limited_geometry;
    struct nvme_block_provider_callbacks callbacks;
    struct test_provider_persistence_state state;
    struct test_provider_persistence_state reloaded;
    u8 buffer[TEST_BLOCK_BYTES];
    static u8 full_range[FAKE_BLOCKS][TEST_BLOCK_BYTES];
    u8 data_a[] = { 'a', 'l', 'p', 'h', 'a' };
    u8 data_b[] = { 'b', 'e', 't', 'a' };
    u32 i;
    struct nvme_block_provider_span span = {
        .bytes = buffer,
        .capacity = sizeof(buffer),
        .used = 99U,
    };

    CHECK(provider_make(&provider, &handle, &media, 21U, false) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(test_provider_persistence_format(&provider, &handle));
    CHECK(test_provider_persistence_mount(&provider, &handle, &state));
    CHECK(state.record_count == 0U && state.generation == 1U);
    CHECK(test_provider_persistence_put(&provider, &handle, &state, 41U,
                                        data_a, sizeof(data_a)));
    CHECK(test_provider_persistence_put(&provider, &handle, &state, 42U,
                                        data_b, sizeof(data_b)));
    CHECK(state.record_count == 2U && state.ids[0] == 41U &&
          state.ids[1] == 42U);
    CHECK(provider_make(&reload, &reload_handle, &media, 22U, false) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(test_provider_persistence_mount(&reload, &reload_handle, &reloaded));
    CHECK(reloaded.record_count == 2U && reloaded.ids[0] == 41U &&
          reloaded.ids[1] == 42U);

    media.blocks[2U][33U] ^= 1U;
    CHECK(!test_provider_persistence_mount(&reload, &reload_handle, &reloaded));
    CHECK(test_provider_persistence_format(&reload, &reload_handle));
    CHECK(test_provider_persistence_mount(&reload, &reload_handle, &reloaded));
    CHECK(reloaded.record_count == 0U && reloaded.generation == 1U);

    span.bytes = &full_range[0][0];
    span.capacity = sizeof(full_range);
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U, 0U,
                                   FAKE_BLOCKS, &span) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(span.used == sizeof(full_range));
    CHECK(full_range[0][0] == media.blocks[0][0] &&
          full_range[FAKE_BLOCKS - 1U][0] == media.blocks[FAKE_BLOCKS - 1U][0]);
    span.bytes = buffer;
    span.capacity = sizeof(buffer);
    for (i = 0U; i < FAKE_BLOCKS; i++) {
        memset(buffer, 0, sizeof(buffer));
        span.used = 99U;
        CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U, i, 1U,
                                       &span) == NVME_BLOCK_PROVIDER_OK);
        CHECK(span.used == TEST_BLOCK_BYTES);
        CHECK(buffer[0] == media.blocks[i][0]);
    }
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U,
                                   FAKE_BLOCKS - 1U, 2U, &span) ==
          NVME_BLOCK_PROVIDER_RANGE);
    CHECK(span.used == 0U);
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U,
                                   FAKE_BLOCKS, 1U, &span) ==
          NVME_BLOCK_PROVIDER_RANGE);
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U, 0U, 0U,
                                   &span) == NVME_BLOCK_PROVIDER_RANGE);
    limited_geometry = geometry_make(23U, false);
    limited_geometry.max_transfer_blocks = 1U;
    callbacks = callbacks_make(&media);
    memset(&limited, 0, sizeof(limited));
    CHECK(nvme_block_provider_init(&limited, &limited_geometry, &callbacks,
                                   0U, &limited_handle) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_read(&limited, &limited_handle, 0U, 0U, 2U,
                                   &span) == NVME_BLOCK_PROVIDER_RANGE);
    span.capacity = TEST_BLOCK_BYTES - 1U;
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U, 0U, 1U,
                                   &span) == NVME_BLOCK_PROVIDER_RANGE);
    span.capacity = sizeof(buffer);
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 1U, 0U, 1U,
                                   &span) == NVME_BLOCK_PROVIDER_OWNER);
    reload.control.callback_active = 1U;
    CHECK(nvme_block_provider_read(&reload, &reload_handle, 0U, 0U, 1U,
                                   &span) == NVME_BLOCK_PROVIDER_BUSY);
    reload.control.callback_active = 0U;
}

static void test_failures_read_only_and_revocation(void)
{
    struct fake_media media = { .present = true };
    struct nvme_block_provider provider;
    struct nvme_block_provider short_provider;
    struct nvme_block_provider false_provider;
    struct nvme_block_provider removed_provider;
    struct nvme_block_provider readonly_provider;
    struct nvme_block_provider retired_provider;
    struct nvme_block_provider_handle handle;
    struct nvme_block_provider_handle short_handle;
    struct nvme_block_provider_handle false_handle;
    struct nvme_block_provider_handle removed_handle;
    struct nvme_block_provider_handle readonly_handle;
    struct nvme_block_provider_handle retired_handle;
    struct nvme_block_provider_status status;
    struct nvme_block_provider_geometry geometry;
    struct nvme_block_provider_callbacks callbacks;
    u8 buffer[TEST_BLOCK_BYTES] = {0};
    struct nvme_block_provider_span span = {
        .bytes = buffer,
        .capacity = sizeof(buffer),
    };

    CHECK(provider_make(&readonly_provider, &readonly_handle, &media, 31U,
                        true) == NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_write(&readonly_provider, &readonly_handle, 0U,
                                    0U, 1U, &span) ==
          NVME_BLOCK_PROVIDER_READ_ONLY);
    CHECK(media.write_calls == 0U);

    media.short_transfer = true;
    CHECK(provider_make(&short_provider, &short_handle, &media, 32U, false) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_read(&short_provider, &short_handle, 0U, 0U,
                                   1U, &span) ==
          NVME_BLOCK_PROVIDER_PARTIAL);
    CHECK(nvme_block_provider_status_get(&short_provider, 0U, &status) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(status.state == NVME_BLOCK_PROVIDER_QUARANTINED &&
          status.fault == NVME_BLOCK_PROVIDER_FAULT_PARTIAL);
    CHECK(nvme_block_provider_read(&short_provider, &short_handle, 0U, 0U,
                                   1U, &span) == NVME_BLOCK_PROVIDER_STALE);
    CHECK(nvme_block_provider_open(&short_provider, 0U, &handle) ==
          NVME_BLOCK_PROVIDER_INACTIVE);
    media.short_transfer = false;

    media.false_transfer = true;
    CHECK(provider_make(&false_provider, &false_handle, &media, 321U, false) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_read(&false_provider, &false_handle, 0U, 0U,
                                   1U, &span) == NVME_BLOCK_PROVIDER_IO);
    CHECK(nvme_block_provider_status_get(&false_provider, 0U, &status) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(status.state == NVME_BLOCK_PROVIDER_QUARANTINED &&
          status.fault == NVME_BLOCK_PROVIDER_FAULT_CALLBACK);
    CHECK(nvme_block_provider_read(&false_provider, &false_handle, 0U, 0U,
                                   1U, &span) == NVME_BLOCK_PROVIDER_STALE);
    media.false_transfer = false;

    media.remove_after_partial_write = true;
    media.present = true;
    CHECK(provider_make(&removed_provider, &removed_handle, &media, 33U,
                        false) == NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_write(&removed_provider, &removed_handle, 0U,
                                    3U, 1U, &span) == NVME_BLOCK_PROVIDER_IO);
    CHECK(nvme_block_provider_status_get(&removed_provider, 0U, &status) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(status.state == NVME_BLOCK_PROVIDER_QUARANTINED &&
          status.fault == NVME_BLOCK_PROVIDER_FAULT_CALLBACK);
    CHECK(nvme_block_provider_read(&removed_provider, &removed_handle, 0U,
                                   3U, 1U, &span) == NVME_BLOCK_PROVIDER_STALE);
    media.remove_after_partial_write = false;
    media.present = true;

    CHECK(provider_make(&provider, &handle, &media, 34U, false) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_revoke(&provider, 0U,
                                     NVME_BLOCK_PROVIDER_REVOKE_REMOVAL) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_status_get(&provider, 0U, &status) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(status.state == NVME_BLOCK_PROVIDER_REMOVED &&
          status.fault == NVME_BLOCK_PROVIDER_FAULT_REMOVED);
    CHECK(nvme_block_provider_read(&provider, &handle, 0U, 0U, 1U, &span) ==
          NVME_BLOCK_PROVIDER_STALE);
    CHECK(nvme_block_provider_revoke(&provider, 0U,
                                     NVME_BLOCK_PROVIDER_REVOKE_AER) ==
          NVME_BLOCK_PROVIDER_INACTIVE);

    geometry = geometry_make(35U, false);
    geometry.namespace_generation = ~0ULL;
    callbacks = callbacks_make(&media);
    memset(&retired_provider, 0, sizeof(retired_provider));
    CHECK(nvme_block_provider_init(&retired_provider, &geometry, &callbacks,
                                   0U, &retired_handle) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_revoke(&retired_provider, 0U,
                                     NVME_BLOCK_PROVIDER_REVOKE_AER) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(nvme_block_provider_status_get(&retired_provider, 0U, &status) ==
          NVME_BLOCK_PROVIDER_OK);
    CHECK(status.state == NVME_BLOCK_PROVIDER_RETIRED);
    CHECK(nvme_block_provider_read(&retired_provider, &retired_handle, 0U,
                                   0U, 1U, &span) ==
          NVME_BLOCK_PROVIDER_RETIRED_RESULT);
}

int main(void)
{
    test_initialization_and_geometry();
    test_persistence_and_ranges();
    test_failures_read_only_and_revocation();
    if (failures) {
        printf("nvme block provider: %u/%u checks failed\n", failures, checks);
        return 1;
    }
    printf("nvme block provider: %u checks PASS\n", checks);
    return checks >= 200U ? 0 : 1;
}
