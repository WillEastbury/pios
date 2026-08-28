#include "sdhost.h"

bool sdhost_clock_divider(u32 core_hz, u32 target_hz,
                          u32 *divider, u32 *actual_hz)
{
    if (!divider || !actual_hz || core_hz == 0 || target_hz == 0)
        return false;

    u32 divisor = core_hz / target_hz;
    if (divisor < 2U)
        divisor = 2U;
    if (core_hz / divisor > target_hz) {
        if (divisor == 0xFFFFFFFFU)
            return false;
        divisor++;
    }

    u32 cdiv = divisor - 2U;
    if (cdiv > 0x7FFU)
        cdiv = 0x7FFU;
    u32 actual = core_hz / (cdiv + 2U);
    if (actual == 0)
        return false;

    *divider = cdiv;
    *actual_hz = actual;
    return true;
}

static u32 csd_bits(const u32 response[4], u32 start, u32 width)
{
    u32 word = 3U - (start / 32U);
    u32 shift = start & 31U;
    u32 value = response[word] >> shift;
    if (width + shift > 32U && word > 0U)
        value |= response[word - 1U] << (32U - shift);
    if (width < 32U)
        value &= (1U << width) - 1U;
    return value;
}

bool sdhost_csd_capacity(const u32 response[4], u64 *capacity)
{
    if (!response || !capacity)
        return false;

    u32 version = csd_bits(response, 126U, 2U);
    if (version == 1U) {
        u32 c_size = csd_bits(response, 48U, 22U);
        *capacity = (u64)(c_size + 1U) * 512ULL * 1024ULL;
        return true;
    }
    if (version == 0U) {
        u32 read_bl_len = csd_bits(response, 80U, 4U);
        u32 c_size = csd_bits(response, 62U, 12U);
        u32 c_size_mult = csd_bits(response, 47U, 3U);
        if (read_bl_len > 31U)
            return false;
        *capacity = (u64)(c_size + 1U) *
                    (1ULL << (c_size_mult + 2U)) *
                    (1ULL << read_bl_len);
        return true;
    }
    return false;
}
