#pragma once
#include "types.h"

/*
 * Raw SD block I/O via SDHCI (BCM2712 EMMC2 controller).
 * No filesystem. No partitions. Just LBA -> 512-byte blocks.
 */

#define SD_BLOCK_SIZE   512

bool sd_init(void);
bool sd_read_block(u32 lba, u8 *buf);
bool sd_write_block(u32 lba, const u8 *buf);
bool sd_read_blocks(u32 lba, u32 count, u8 *buf);
bool sd_write_blocks(u32 lba, u32 count, const u8 *buf);

/* Card info (populated after sd_init) */
typedef struct {
    u32 type;       /* 1=SDSC, 2=SDHC/SDXC */
    u32 rca;        /* Relative card address */
    u64 capacity;   /* Total bytes */
} sd_card_t;

const sd_card_t *sd_get_card_info(void);
