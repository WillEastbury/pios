#include "rp1_adc.h"

static i64 div_round_closest_i64(i64 value, i64 divisor)
{
    if (value >= 0)
        return (value + divisor / 2) / divisor;
    return (value - divisor / 2) / divisor;
}

u32 rp1_adc_raw_to_mv(u32 raw)
{
    if (raw > 0xFFFU)
        raw = 0xFFFU;
    return (RP1_ADC_VREF_MV * raw + 0x7FFU) / 0xFFFU;
}

i32 rp1_adc_raw_to_millidegrees(u32 raw)
{
    i64 mv = (i64)rp1_adc_raw_to_mv(raw);
    i64 delta = div_round_closest_i64((mv - 706) * 1000000LL, 1721);
    return (i32)(27000LL - delta);
}
