#include <stdio.h>
#include <string.h>

#include "sdhost.h"

static int failures;

#define CHECK(expr) do { \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void set_bits(u32 response[4], u32 start, u32 width, u32 value)
{
    for (u32 bit = 0; bit < width; bit++) {
        u32 absolute = start + bit;
        u32 word = 3U - absolute / 32U;
        u32 shift = absolute & 31U;
        if (value & (1U << bit))
            response[word] |= 1U << shift;
    }
}

int main(void)
{
    u32 divider;
    u32 actual;
    CHECK(sdhost_clock_divider(250000000U, 400000U,
                               &divider, &actual));
    CHECK(divider == 623U);
    CHECK(actual == 400000U);
    CHECK(sdhost_clock_divider(250000000U, 25000000U,
                               &divider, &actual));
    CHECK(divider == 8U);
    CHECK(actual == 25000000U);
    CHECK(!sdhost_clock_divider(0U, 400000U, &divider, &actual));

    u32 csd[4];
    memset(csd, 0, sizeof(csd));
    set_bits(csd, 126U, 2U, 1U);
    set_bits(csd, 48U, 22U, 0x1DFFFU);
    u64 capacity;
    CHECK(sdhost_csd_capacity(csd, &capacity));
    CHECK(capacity == (u64)(0x1E000U) * 512ULL * 1024ULL);

    memset(csd, 0, sizeof(csd));
    set_bits(csd, 126U, 2U, 3U);
    CHECK(!sdhost_csd_capacity(csd, &capacity));

    if (failures) {
        printf("sdhost: %d failure(s)\n", failures);
        return 1;
    }
    printf("sdhost: all checks passed\n");
    return 0;
}
