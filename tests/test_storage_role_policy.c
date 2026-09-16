#include <stdio.h>
#include <string.h>

#include "storage_role_policy.h"

static u32 checks;
static u32 failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    } \
} while (0)

static struct storage_role_policy_input valid_input(
    struct storage_role_fact facts[STORAGE_ROLE_POLICY_MAX_FACTS])
{
    memset(facts, 0, sizeof(struct storage_role_fact) *
           STORAGE_ROLE_POLICY_MAX_FACTS);
    facts[0].identity = 1U;
    facts[0].first_lba = 2048U;
    facts[0].block_count = 1000U;
    facts[0].table_index = 0U;
    facts[0].role = STORAGE_ROLE_BOOT;
    facts[0].marker = STORAGE_ROLE_MARKER_FAT32_BPB;
    facts[0].marker_version = 1U;
    facts[1].identity = 2U;
    facts[1].first_lba = 3048U;
    facts[1].block_count = 2000U;
    facts[1].table_index = 1U;
    facts[1].role = STORAGE_ROLE_WALFS;
    facts[1].marker = STORAGE_ROLE_MARKER_WALFS_SUPERBLOCK;
    facts[1].marker_version = 1U;
    facts[2].identity = 3U;
    facts[2].first_lba = 5048U;
    facts[2].block_count = 1000U;
    facts[2].table_index = 2U;
    facts[2].role = STORAGE_ROLE_EXCHANGE;
    facts[2].marker = STORAGE_ROLE_MARKER_FAT32_BPB;
    facts[2].marker_version = 1U;
    return (struct storage_role_policy_input) {
        .facts = facts,
        .total_blocks = 6048U,
        .enumeration_generation = 9U,
        .fact_count = 3U,
    };
}

int main(void)
{
    struct storage_role_fact facts[STORAGE_ROLE_POLICY_MAX_FACTS];
    struct storage_role_fact out[STORAGE_ROLE_POLICY_MAX_FACTS];
    struct storage_role_policy_input input = valid_input(facts);
    struct storage_role_policy policy = {0};
    struct storage_role_policy_handle handle;
    u32 used;

    CHECK(storage_role_policy_init(&policy, 1U, 0U) ==
          STORAGE_ROLE_POLICY_OK);
    CHECK(storage_role_policy_validate(&policy, &input, 0U, &handle) ==
          STORAGE_ROLE_POLICY_OK);
    CHECK(storage_role_policy_get(&policy, &handle, 0U, out,
                                  STORAGE_ROLE_POLICY_MAX_FACTS, &used) ==
          STORAGE_ROLE_POLICY_OK);
    CHECK(used == 3U && out[1].role == STORAGE_ROLE_WALFS &&
          out[1].marker == STORAGE_ROLE_MARKER_WALFS_SUPERBLOCK);

    policy = (struct storage_role_policy){0};
    facts[1].marker = STORAGE_ROLE_MARKER_NONE;
    CHECK(storage_role_policy_init(&policy, 2U, 0U) ==
          STORAGE_ROLE_POLICY_OK);
    CHECK(storage_role_policy_validate(&policy, &input, 0U, &handle) ==
          STORAGE_ROLE_POLICY_REJECTED);

    policy = (struct storage_role_policy){0};
    input = valid_input(facts);
    facts[2].role = STORAGE_ROLE_WALFS;
    facts[2].marker = STORAGE_ROLE_MARKER_WALFS_SUPERBLOCK;
    CHECK(storage_role_policy_init(&policy, 3U, 0U) ==
          STORAGE_ROLE_POLICY_OK);
    CHECK(storage_role_policy_validate(&policy, &input, 0U, &handle) ==
          STORAGE_ROLE_POLICY_REJECTED);

    policy = (struct storage_role_policy){0};
    input = valid_input(facts);
    facts[2].first_lba = facts[1].first_lba + 1U;
    CHECK(storage_role_policy_init(&policy, 4U, 0U) ==
          STORAGE_ROLE_POLICY_OK);
    CHECK(storage_role_policy_validate(&policy, &input, 0U, &handle) ==
          STORAGE_ROLE_POLICY_DUPLICATE);

    CHECK(checks == 11U && failures == 0U);
    printf("storage_role_policy: %u checks passed\n", checks);
    return failures != 0U;
}
