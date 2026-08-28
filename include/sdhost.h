#pragma once

#include "types.h"

/*
 * BCM2835 SDHOST controller used by the removable microSD slot on Pi 3
 * B/B+ and Pi Zero 2 W. This is not the Arasan SDHCI controller used by
 * onboard Wi-Fi on those boards.
 */

struct sdhost_card_info {
    u32 type;       /* 1=SDSC, 2=SDHC/SDXC */
    u32 rca;
    u64 capacity;
};

bool sdhost_clock_divider(u32 core_hz, u32 target_hz,
                          u32 *divider, u32 *actual_hz);
bool sdhost_csd_capacity(const u32 response[4], u64 *capacity);

bool sdhost_init(u64 base, u32 core_clock_hz,
                 struct sdhost_card_info *info);
bool sdhost_read_block(u32 lba, u8 *buf);
bool sdhost_write_block(u32 lba, const u8 *buf);
bool sdhost_read_blocks(u32 lba, u32 count, u8 *buf);
bool sdhost_write_blocks(u32 lba, u32 count, const u8 *buf);
