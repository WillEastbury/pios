#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "intel_guc_boot.h"

static u8 ads[0x00B00000U];
static u8 log_buffer[0x00200000U];

static u32 get32(const u8 *data, u32 offset)
{
    return (u32)data[offset] | ((u32)data[offset + 1U] << 8) |
           ((u32)data[offset + 2U] << 16) |
           ((u32)data[offset + 3U] << 24);
}

int main(void)
{
    struct intel_guc_boot_layout layout;
    memset(ads, 0, sizeof(ads));
    memset(log_buffer, 0, sizeof(log_buffer));
    assert(intel_guc_min_boot_build(
        ads, sizeof(ads), log_buffer, sizeof(log_buffer),
        0x02000000U, 0x03000000U, 9441280U,
        0xE212U, 0U, &layout));
    assert(layout.ads_bytes == 0x0090B000U);
    assert(layout.private_offset == 0xA000U);
    assert(layout.private_bytes == 0x901000U);
    assert(layout.log_bytes == 0x115000U);
    assert(layout.params[0] == 0x030003F7U);
    assert(layout.params[3] == 0x40U);
    assert(layout.params[4] == 0x4000U);
    assert(layout.params[5] == 0xE2120000U);
    assert(get32(ads, 4100U) == 0x020011DCU);
    assert(get32(ads, 4104U) == 0x0200123CU);
    assert(get32(ads, 4132U) == 0x02006000U);
    assert(get32(ads, 4196U) == 0x00001E80U);
    assert(get32(ads, 4244U) == 0x0200A000U);
    for (u32 i = 4668U; i < 4668U + 512U; i++) {
        u32 mapping_offset = i - 4668U;
        assert(ads[i] == (mapping_offset == 4U * 32U ? 0U : 32U));
    }
    assert(get32(ads, 5196U) == 1U);
    assert(get32(ads, 1024U) == 0x0200553CU);
    assert(get32(ads, 1028U) == 2U);
    assert(get32(ads, 21820U) == 0x0001A080U);
    assert(get32(ads, 21824U) == 0U);
    assert(get32(ads, 21836U) == 0x0001A0A8U);
    assert(!intel_guc_min_boot_build(
        ads, 0x900000U, log_buffer, sizeof(log_buffer),
        0x02000000U, 0x03000000U, 9441280U,
        0xE212U, 0U, &layout));
    assert(!intel_guc_min_boot_build(
        ads, sizeof(ads), log_buffer, 0x100000U,
        0x02000000U, 0x03000000U, 9441280U,
        0xE212U, 0U, &layout));
    puts("intel GuC minimal ADS/log/params: PASS");
    return 0;
}
