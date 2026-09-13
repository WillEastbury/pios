#include <stdio.h>
#include <string.h>

#include "qemu_xfer_partition.h"

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

static void entry(u8 *mbr, u32 index, u8 type, u32 first, u32 count)
{
    u8 *p = mbr + 0x1BEU + index * 16U;

    p[4] = type;
    put32(p + 8U, first);
    put32(p + 12U, count);
}

static void valid_mbr(u8 mbr[512])
{
    memset(mbr, 0, 512U);
    put32(mbr + 440U, 0x1255AA77U);
    mbr[510U] = 0x55U;
    mbr[511U] = 0xAAU;
    entry(mbr, 0U, 0x0CU, 2048U, 131072U);
    entry(mbr, 1U, 0xDAU, 133120U, 196608U);
    entry(mbr, 2U, 0x0CU, 329728U, 131072U);
}

int main(void)
{
    u8 mbr[512];
    struct qemu_xfer_partition out;

    valid_mbr(mbr);
    CHECK(qemu_xfer_partition_select(mbr, 460800U, &out) ==
          QEMU_XFER_PARTITION_OK);
    CHECK(out.disk_id == 0x1255AA77U && out.first_lba == 329728U &&
          out.block_count == 131072U);
    CHECK(out.identity == 0x1255AA7700050800ULL);
    CHECK(memcmp(out.expected_label, "PIOSXFER   ", 11U) == 0);

    mbr[0x1DEU + 4U] = 0xDAU;
    CHECK(qemu_xfer_partition_select(mbr, 460800U, &out) ==
          QEMU_XFER_PARTITION_LAYOUT);
    valid_mbr(mbr);
    put32(mbr + 0x1DEU + 8U, 2048U);
    CHECK(qemu_xfer_partition_select(mbr, 460800U, &out) ==
          QEMU_XFER_PARTITION_LAYOUT);
    valid_mbr(mbr);
    put32(mbr + 0x1DEU + 12U, 0U);
    CHECK(qemu_xfer_partition_select(mbr, 460800U, &out) ==
          QEMU_XFER_PARTITION_LAYOUT);
    valid_mbr(mbr);
    mbr[510U] = 0U;
    CHECK(qemu_xfer_partition_select(mbr, 460800U, &out) ==
          QEMU_XFER_PARTITION_MBR);
    valid_mbr(mbr);
    put32(mbr + 440U, 0U);
    CHECK(qemu_xfer_partition_select(mbr, 460800U, &out) ==
          QEMU_XFER_PARTITION_LAYOUT);
    puts("qemu xfer partition: PASS");
    return 0;
}
