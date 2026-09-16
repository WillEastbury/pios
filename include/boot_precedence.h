/*
 * Host-testable boot-control helpers. The kernel shares the package-id parser;
 * stage0 mirrors the byte-level checksum rules because it is linked minimally
 * and production selection also owns media I/O transitions.
 */
#pragma once

#include "types.h"
#include "walfs.h"

enum boot_precedence_choice {
    BOOT_PRECEDENCE_NONE = 0,
    BOOT_PRECEDENCE_OVERRIDE_O,
    BOOT_PRECEDENCE_PENDING,
    BOOT_PRECEDENCE_ACTIVE,
    BOOT_PRECEDENCE_FAT_RECOVERY,
};

struct boot_precedence_input {
    bool override_armed;
    bool override_valid;
    bool override_consumed;
    bool pending_bootable;
    bool active_bootable;
    bool fat_bootable;
};

u32 bootctrl_checksum_versioned(const u8 *sector, u32 version);
bool bootctrl_validate_versioned(const u8 *sector);
bool bootctrl_migrate_v1_to_v2(u8 *sector);
bool bootctrl_parse_package_id(const char *text, u64 *package_id_out);
enum boot_precedence_choice
boot_precedence_choose(const struct boot_precedence_input *input);
