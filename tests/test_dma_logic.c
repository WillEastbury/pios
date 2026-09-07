#include <stdio.h>
#include "types.h"
#include "dma.h"

static u32 failures;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #c); failures++; \
} } while (0)

int main(void)
{
    struct dma_cb cb;
    CHECK(sizeof(cb) == 32);
    CHECK(DMA_CS_ERROR == (1U << 8));
    CHECK(DMA_CS_RESET == (1U << 31));
    CHECK(dma_cb_encode(&cb, 0x1200000040ULL, 0x3400000080ULL, 16384, true, 0x200000));
    CHECK(cb.ti == 0x118 && cb.src_addr == 0x40 && cb.dst_addr == 0x80);
    CHECK(cb.stride == 0x3412 && cb.next_cb == 0x10000 && cb.xfer_len == 16384);
    CHECK(cb._pad[0] == 0 && cb._pad[1] == 0);
    CHECK(dma_cb_encode(&cb, 0x1000, 0x2000, DMA_MAX_CB_BYTES, false, 0));
    CHECK(cb.ti == 0x18 && cb.stride == 0 && cb.next_cb == 0);
    struct dma_cb before = cb;
    CHECK(!dma_cb_encode(&cb, 0x1000, 0x2000, 65536, true, 0));
    CHECK(memcmp(&cb, &before, sizeof(cb)) == 0);
    CHECK(!dma_cb_encode(&cb, (1ULL << 40) - 8, 0x2000, 16, true, 0));
    CHECK(!dma_cb_encode(&cb, 0x1000, (1ULL << 40) - 8, 16, true, 0));
    CHECK(!dma_cb_encode(&cb, 0x1000, 0x2000, 16, true, 33));
    CHECK(!dma_cb_encode(&cb, 0x1000, 0x2000, 16, true, 1ULL << 37));
    CHECK(!dma_cb_encode(&cb, 0x1000, 0x2000, 0, true, 0));
    CHECK(memcmp(&cb, &before, sizeof(cb)) == 0);
    printf("DMA logic: %u failures\n", failures);
    return failures ? 1 : 0;
}
