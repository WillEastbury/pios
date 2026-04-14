#pragma once
#include "types.h"

/*
 * Raw SD block I/O via SDHCI (BCM2712 EMMC2 controller).
 * No filesystem. No partitions. Just LBA -> 512-byte blocks.
 *
 * Buffer contract:
 *   - All buffers passed to read/write must be at least count * SD_BLOCK_SIZE bytes.
 *   - 4-byte aligned buffers use the fast 32-bit PIO path.
 *   - Unaligned buffers are handled safely via a byte-copy fallback.
 *
 * Cache coherency:
 *   - The EMMC2 controller is accessed via PIO (CPU loads/stores to REG_DATA).
 *   - No DMA is used, so no explicit cache maintenance is required.
 *   - Callers sharing buffers across cores must ensure their own coherency.
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

/* Lightweight I/O statistics (counters since sd_init) */
typedef struct {
    u64 reads;          /* successful block reads */
    u64 writes;         /* successful block writes */
    u64 errors;         /* total error events */
    u64 retries;        /* transient-error retries */
    u64 cmd_timeouts;   /* command timeouts */
    u64 data_timeouts;  /* data transfer timeouts */
} sd_stats_t;

const sd_stats_t *sd_get_stats(void);
