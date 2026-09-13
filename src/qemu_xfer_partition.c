/* QEMU adapter over the shared, production MBR layout validation. */
#include "types.h"
#include "qemu_xfer_partition.h"
#include "storage_layout.h"

static void clear_partition(struct qemu_xfer_partition *out)
{
    u8 *p = (u8 *)out;

    for (u32 i = 0U; i < sizeof(*out); i++)
        p[i] = 0U;
}

enum qemu_xfer_partition_result qemu_xfer_partition_select(
    const u8 mbr[QEMU_XFER_MBR_BYTES], u64 total_blocks,
    struct qemu_xfer_partition *out)
{
    struct storage_layout layout;
    const struct storage_layout_fact *p3;
    enum storage_layout_result result;
    static const u8 label[QEMU_XFER_LABEL_BYTES] = {
        'P', 'I', 'O', 'S', 'X', 'F', 'E', 'R', ' ', ' ', ' '
    };

    if (!out)
        return QEMU_XFER_PARTITION_INVALID;
    clear_partition(out);
    result = storage_layout_validate(mbr, total_blocks, &layout);
    if (result == STORAGE_LAYOUT_INVALID_ARGUMENT)
        return QEMU_XFER_PARTITION_INVALID;
    if (result == STORAGE_LAYOUT_MBR_SIGNATURE)
        return QEMU_XFER_PARTITION_MBR;
    if (result != STORAGE_LAYOUT_OK ||
        layout.kind != STORAGE_LAYOUT_THREE_PARTITION ||
        layout.disk_id == 0U)
        return QEMU_XFER_PARTITION_LAYOUT;

    p3 = &layout.facts[QEMU_XFER_PARTITION_INDEX - 1U];
    out->disk_id = layout.disk_id;
    out->first_lba = (u32)p3->first_lba;
    out->block_count = (u32)p3->block_count;
    out->identity = ((u64)layout.disk_id << 32) | p3->first_lba;
    for (u32 i = 0U; i < QEMU_XFER_LABEL_BYTES; i++)
        out->expected_label[i] = label[i];
    return QEMU_XFER_PARTITION_OK;
}
