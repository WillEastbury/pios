#include "rp1_adc.h"

#include <stdio.h>

static int check_u32(const char *name, u32 got, u32 expected)
{
    if (got == expected)
        return 0;
    printf("FAIL %s got=%u expected=%u\n", name, got, expected);
    return 1;
}

static int check_i32(const char *name, i32 got, i32 expected)
{
    if (got == expected)
        return 0;
    printf("FAIL %s got=%d expected=%d\n", name, got, expected);
    return 1;
}

int main(void)
{
    int failures = 0;
    failures += check_u32("zero_mv", rp1_adc_raw_to_mv(0), 0);
    failures += check_u32("full_mv", rp1_adc_raw_to_mv(4095), 3300);

    u32 raw_706mv = (706U * 4095U + 1650U) / 3300U;
    i32 at_reference = rp1_adc_raw_to_millidegrees(raw_706mv);
    if (at_reference < 26700 || at_reference > 27300) {
        printf("FAIL temp_reference got=%d\n", at_reference);
        failures++;
    }

    failures += check_i32("clamp_high",
                          rp1_adc_raw_to_millidegrees(5000),
                          rp1_adc_raw_to_millidegrees(4095));
    if (failures)
        return 1;
    puts("test_rp1_adc: ALL PASS");
    return 0;
}
