/* Host tests for #193's offline dedicated FAT32 exchange policy. */
#include "types.h"
#include <stdio.h>
#include <string.h>

#include "exchange_volume_policy.h"

static u32 failures;
static u32 checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static const u8 basic_data_guid[] = {
    0xA2U, 0xA0U, 0xD0U, 0xEBU, 0xE5U, 0xB9U, 0x33U, 0x44U,
    0x87U, 0xC0U, 0x68U, 0xB6U, 0xB7U, 0x26U, 0x99U, 0xC7U
};

static struct exchange_volume_partition_fact mbr_fact(
    u64 identity, u64 first, u64 count)
{
    struct exchange_volume_partition_fact fact = {0};

    fact.identity = identity;
    fact.first_lba = first;
    fact.block_count = count;
    fact.table_scheme = EXCHANGE_VOLUME_TABLE_MBR;
    fact.filesystem = EXCHANGE_VOLUME_FILESYSTEM_FAT32;
    fact.mbr_type = 0x0CU;
    fact.filesystem_label_bytes = 8U;
    memcpy(fact.filesystem_label, "PIOSXFER", 8U);
    return fact;
}

static struct exchange_volume_partition_fact gpt_fact(
    u64 identity, u64 first, u64 count)
{
    struct exchange_volume_partition_fact fact = mbr_fact(identity, first,
                                                           count);

    fact.table_scheme = EXCHANGE_VOLUME_TABLE_GPT;
    fact.table_index = 1U;
    fact.mbr_type = 0U;
    memcpy(fact.gpt_type_guid, basic_data_guid, sizeof(basic_data_guid));
    return fact;
}

static struct exchange_volume_policy_input input_make(
    const struct exchange_volume_partition_fact *facts, u32 count, u32 scheme)
{
    return (struct exchange_volume_policy_input) {
        .facts = facts,
        .total_blocks = 10000U,
        .enumeration_generation = 7U,
        .fact_count = count,
        .table_scheme = scheme,
    };
}

static enum exchange_volume_policy_result attach(
    struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_input *input, u64 identity,
    struct exchange_volume_policy_handle *handle)
{
    struct exchange_volume_policy_request request = {
        .requested_identity = identity,
    };

    return exchange_volume_policy_attach(policy, input, &request, 0U, handle);
}

static void test_mbr_selection_and_generation_binding(void)
{
    struct exchange_volume_policy policy = {0};
    struct exchange_volume_partition_fact facts[2] = {
        mbr_fact(0x111U, 2048U, 4096U),
        {0},
    };
    struct exchange_volume_policy_input input = input_make(
        facts, 2U, EXCHANGE_VOLUME_TABLE_MBR);
    struct exchange_volume_policy_handle handle;
    struct exchange_volume_policy_handle forged;
    struct exchange_volume_partition_fact out;

    facts[1].identity = 0x222U;
    facts[1].first_lba = 7000U;
    facts[1].block_count = 1000U;
    facts[1].table_scheme = EXCHANGE_VOLUME_TABLE_MBR;
    facts[1].filesystem = EXCHANGE_VOLUME_FILESYSTEM_UNKNOWN;
    facts[1].mbr_type = 0x83U;
    facts[1].table_index = 1U;

    CHECK(exchange_volume_policy_init(&policy, 0xABC1U, 0U) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(exchange_volume_policy_init(&policy, 0xABC2U, 0U) ==
          EXCHANGE_VOLUME_POLICY_BUSY);
    CHECK(attach(&policy, &input, 0U, &handle) ==
          EXCHANGE_VOLUME_POLICY_INVALID && handle.magic == 0U);
    CHECK(attach(&policy, &input, 0x222U, &handle) ==
          EXCHANGE_VOLUME_POLICY_REJECTED);
    CHECK(attach(&policy, &input, 0x111U, &handle) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(handle.identity == 0x111U && handle.instance_epoch == 0xABC1U &&
          handle.enumeration_generation == 7U &&
          handle.attachment_generation == 1U);
    CHECK(exchange_volume_policy_attachment_get(&policy, &handle, 0U, &out) ==
          EXCHANGE_VOLUME_POLICY_OK && out.identity == 0x111U &&
          out.first_lba == 2048U && out.block_count == 4096U);
    forged = handle;
    forged.identity++;
    CHECK(exchange_volume_policy_attachment_get(&policy, &forged, 0U, &out) ==
          EXCHANGE_VOLUME_POLICY_STALE);
    forged = handle;
    forged.enumeration_generation++;
    CHECK(exchange_volume_policy_attachment_get(&policy, &forged, 0U, &out) ==
          EXCHANGE_VOLUME_POLICY_STALE);
    CHECK(attach(&policy, &input, 0x111U, &handle) ==
          EXCHANGE_VOLUME_POLICY_BUSY);
}

static void test_candidate_rejections(void)
{
    struct exchange_volume_policy policy;
    struct exchange_volume_partition_fact facts[2];
    struct exchange_volume_policy_input input;
    struct exchange_volume_policy_handle handle;

#define REJECT(what, mutate, expected) do { \
    policy = (struct exchange_volume_policy){0}; \
    facts[0] = mbr_fact(0x100U, 2048U, 2048U); \
    input = input_make(facts, 1U, EXCHANGE_VOLUME_TABLE_MBR); \
    mutate; \
    CHECK(exchange_volume_policy_init(&policy, 1U, 0U) == \
          EXCHANGE_VOLUME_POLICY_OK); \
    CHECK(attach(&policy, &input, 0x100U, &handle) == (expected)); \
} while (0)

    REJECT("boot fact excluded",
           facts[0].flags = EXCHANGE_VOLUME_FACT_BOOT,
           EXCHANGE_VOLUME_POLICY_REJECTED);
    REJECT("system fact excluded",
           facts[0].flags = EXCHANGE_VOLUME_FACT_PIOS_SYSTEM,
           EXCHANGE_VOLUME_POLICY_REJECTED);
    REJECT("wrong MBR type excluded", facts[0].mbr_type = 0x07U,
           EXCHANGE_VOLUME_POLICY_REJECTED);
    REJECT("label absent excluded", facts[0].filesystem_label_bytes = 0U;
           memset(facts[0].filesystem_label, 0,
                  sizeof(facts[0].filesystem_label)),
           EXCHANGE_VOLUME_POLICY_REJECTED);
    REJECT("label mismatch excluded", facts[0].filesystem_label[7U] = 0x53U,
           EXCHANGE_VOLUME_POLICY_REJECTED);
    REJECT("non-FAT32 excluded",
           facts[0].filesystem = EXCHANGE_VOLUME_FILESYSTEM_UNKNOWN,
           EXCHANGE_VOLUME_POLICY_REJECTED);
    REJECT("unknown scheme rejected",
           facts[0].table_scheme = EXCHANGE_VOLUME_TABLE_INVALID;
           input.table_scheme = EXCHANGE_VOLUME_TABLE_INVALID,
           EXCHANGE_VOLUME_POLICY_INVALID);
    REJECT("range overflow rejected",
           facts[0].first_lba = ~0ULL; facts[0].block_count = 1U,
           EXCHANGE_VOLUME_POLICY_INVALID);
    REJECT("unknown flags rejected", facts[0].flags = 4U,
           EXCHANGE_VOLUME_POLICY_INVALID);

#undef REJECT

    policy = (struct exchange_volume_policy){0};
    facts[0] = mbr_fact(0x100U, 2048U, 2048U);
    facts[1] = mbr_fact(0x200U, 5000U, 2048U);
    facts[1].table_index = 1U;
    input = input_make(facts, 2U, EXCHANGE_VOLUME_TABLE_MBR);
    CHECK(exchange_volume_policy_init(&policy, 2U, 0U) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(attach(&policy, &input, 0x100U, &handle) ==
          EXCHANGE_VOLUME_POLICY_DUPLICATE);

    policy = (struct exchange_volume_policy){0};
    facts[0] = mbr_fact(0x100U, 2048U, 2048U);
    facts[1] = mbr_fact(0x200U, 3000U, 2048U);
    facts[1].table_index = 1U;
    input = input_make(facts, 2U, EXCHANGE_VOLUME_TABLE_MBR);
    CHECK(exchange_volume_policy_init(&policy, 3U, 0U) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(attach(&policy, &input, 0x100U, &handle) ==
          EXCHANGE_VOLUME_POLICY_INVALID);
}

static void test_gpt_and_input_canonicality(void)
{
    struct exchange_volume_policy policy = {0};
    struct exchange_volume_partition_fact facts[2] = {
        gpt_fact(0xA1U, 2048U, 2048U),
        {0},
    };
    struct exchange_volume_policy_input input = input_make(
        facts, 1U, EXCHANGE_VOLUME_TABLE_GPT);
    struct exchange_volume_policy_handle handle;

    CHECK(exchange_volume_policy_init(&policy, 8U, 0U) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(attach(&policy, &input, 0xA1U, &handle) ==
          EXCHANGE_VOLUME_POLICY_OK);

    policy = (struct exchange_volume_policy){0};
    facts[0] = gpt_fact(0xA1U, 2048U, 2048U);
    facts[0].gpt_type_guid[0U] ^= 1U;
    CHECK(exchange_volume_policy_init(&policy, 9U, 0U) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(attach(&policy, &input, 0xA1U, &handle) ==
          EXCHANGE_VOLUME_POLICY_REJECTED);

    policy = (struct exchange_volume_policy){0};
    facts[0] = mbr_fact(0xA1U, 2048U, 2048U);
    facts[0].filesystem_label[8U] = 1U;
    input = input_make(facts, 1U, EXCHANGE_VOLUME_TABLE_MBR);
    CHECK(exchange_volume_policy_init(&policy, 10U, 0U) ==
          EXCHANGE_VOLUME_POLICY_OK);
    CHECK(attach(&policy, &input, 0xA1U, &handle) ==
          EXCHANGE_VOLUME_POLICY_INVALID);
}

static void test_owner_rejection(void)
{
    struct exchange_volume_policy policy = {0};
    struct exchange_volume_partition_fact fact = mbr_fact(1U, 2048U, 2048U);
    struct exchange_volume_policy_input input = input_make(
        &fact, 1U, EXCHANGE_VOLUME_TABLE_MBR);
    struct exchange_volume_policy_handle handle;

    CHECK(exchange_volume_policy_init(&policy, 1U, 1U) ==
          EXCHANGE_VOLUME_POLICY_OWNER);
    CHECK(exchange_volume_policy_attach(&policy, &input, NULL, 1U, &handle) ==
          EXCHANGE_VOLUME_POLICY_OWNER);
}

int main(void)
{
    test_mbr_selection_and_generation_binding();
    test_candidate_rejections();
    test_gpt_and_input_canonicality();
    test_owner_rejection();
    if (failures != 0U) {
        printf("%u/%u exchange volume policy checks failed\n", failures,
               checks);
        return 1;
    }
    printf("exchange volume policy tests passed (%u checks)\n", checks);
    return 0;
}
