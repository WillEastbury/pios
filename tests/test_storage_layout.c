#include <stdio.h>
#include <string.h>

#include "storage_layout.h"

#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; \
} } while (0)

static void put32(u8 *p, u32 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
    p[2] = (u8)(value >> 16);
    p[3] = (u8)(value >> 24);
}

static void entry(u8 mbr[512], u32 index, u8 status, u8 type,
                  u32 first, u32 count)
{
    u8 *p = mbr + 446U + index * 16U;

    p[0] = status;
    p[4] = type;
    put32(p + 8U, first);
    put32(p + 12U, count);
}

static void valid_three(u8 mbr[512])
{
    memset(mbr, 0, 512U);
    put32(mbr + 440U, 0x12345678U);
    mbr[510U] = 0x55U;
    mbr[511U] = 0xAAU;
    entry(mbr, 0U, 0x80U, 0x0CU, 2048U, 1000U);
    entry(mbr, 1U, 0U, 0xDAU, 3048U, 2000U);
    entry(mbr, 2U, 0U, 0x0BU, 5048U, 1000U);
}

int main(void)
{
    u8 mbr[512];
    struct storage_layout layout;

    valid_three(mbr);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK);
    CHECK(layout.kind == STORAGE_LAYOUT_THREE_PARTITION);
    CHECK(layout.facts[0].role == STORAGE_LAYOUT_ROLE_BOOT &&
          layout.facts[0].mbr_status == 0x80U);
    CHECK(layout.facts[1].role == STORAGE_LAYOUT_ROLE_SYSTEM &&
          layout.facts[1].mbr_type == 0xDAU);
    CHECK(layout.facts[2].role == STORAGE_LAYOUT_ROLE_EXCHANGE &&
          layout.facts[2].first_lba == 5048U &&
          layout.p3_present && layout.p3_result == STORAGE_LAYOUT_OK);
    valid_three(mbr);
    entry(mbr, 2U, 0U, STORAGE_LAYOUT_MBR_TYPE_PIOS_RAW, 5048U, 1000U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION &&
          layout.p3_result == STORAGE_LAYOUT_ROLE_MISMATCH);
    CHECK(storage_layout_validate_exchange(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_ROLE_MISMATCH);
    valid_three(mbr);
    mbr[446U] = 0U;
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_THREE_PARTITION);

    valid_three(mbr);
    mbr[510U] = 0U;
    CHECK(storage_layout_validate(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_MBR_SIGNATURE);
    valid_three(mbr);
    memset(mbr + 446U + 2U * 16U, 0, 16U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION &&
          !layout.p3_present && layout.p3_result == STORAGE_LAYOUT_P3_ABSENT);
    mbr[446U + 16U + 4U] = 0x07U;
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION);
    valid_three(mbr);
    entry(mbr, 2U, 0U, 0x83U, 5048U, 1000U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION &&
          layout.p3_present &&
          layout.p3_result == STORAGE_LAYOUT_ROLE_MISMATCH &&
          layout.facts[1].first_lba == 3048U &&
          layout.facts[2].block_count == 0U);
    CHECK(storage_layout_validate_exchange(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_ROLE_MISMATCH);
    valid_three(mbr);
    entry(mbr, 2U, 0U, 0x0CU, 5000U, 1000U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION &&
          layout.p3_present && layout.p3_result == STORAGE_LAYOUT_MALFORMED &&
          layout.facts[1].first_lba == 3048U);
    CHECK(storage_layout_validate_exchange(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_MALFORMED);
    valid_three(mbr);
    entry(mbr, 2U, 0U, 0x0CU, 5048U, 1001U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION &&
          layout.p3_present && layout.p3_result == STORAGE_LAYOUT_MALFORMED &&
          layout.facts[1].first_lba == 3048U);
    CHECK(storage_layout_validate_exchange(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_MALFORMED);
    valid_three(mbr);
    entry(mbr, 3U, 0U, 0x0CU, 1U, 1U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_MALFORMED);
    valid_three(mbr);
    entry(mbr, 0U, 0U, 0x83U, 2048U, 1000U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_ROLE_MISMATCH);
    valid_three(mbr);
    entry(mbr, 1U, 0U, 0x07U, 3048U, 2000U);
    CHECK(storage_layout_validate(mbr, 6048U, &layout) == STORAGE_LAYOUT_OK &&
          layout.kind == STORAGE_LAYOUT_LEGACY_TWO_PARTITION &&
          layout.p3_present &&
          layout.p3_result == STORAGE_LAYOUT_ROLE_MISMATCH);
    CHECK(storage_layout_validate_exchange(mbr, 6048U, &layout) ==
          STORAGE_LAYOUT_ROLE_MISMATCH);
    puts("storage layout: PASS");
    return 0;
}
