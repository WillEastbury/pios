#pragma once
#include "types.h"

#define PIOS_STAGE2_MANIFEST_MAGIC   0x32534750  /* 'PGS2' little-endian */
#define PIOS_STAGE2_MANIFEST_VERSION 1
#define PIOS_STAGE2_MANIFEST_FLAG_PACKAGED (1U << 0)

#define PIOS_STAGE2_PLATFORM_PI5        1U
#define PIOS_STAGE2_PLATFORM_QEMU_VIRT  2U
#define PIOS_STAGE2_PLATFORM_UEFI       3U

#define PIOS_STAGE2_FEAT_AARCH64        (1ULL << 0)
#define PIOS_STAGE2_FEAT_GENERIC_TIMER  (1ULL << 1)
#define PIOS_STAGE2_FEAT_GICV2          (1ULL << 2)
#define PIOS_STAGE2_FEAT_PL011          (1ULL << 3)
#define PIOS_STAGE2_FEAT_PI_FIRMWARE    (1ULL << 4)
#define PIOS_STAGE2_FEAT_RP1            (1ULL << 5)
#define PIOS_STAGE2_FEAT_PCIE           (1ULL << 6)
#define PIOS_STAGE2_FEAT_SD             (1ULL << 7)
#define PIOS_STAGE2_FEAT_GENET          (1ULL << 8)
#define PIOS_STAGE2_FEAT_MAILBOX_FB     (1ULL << 9)
#define PIOS_STAGE2_FEAT_RAM_WALFS      (1ULL << 10)
#define PIOS_STAGE2_FEAT_UEFI           (1ULL << 11)

struct pios_stage2_manifest_header {
    u32 magic;
    u16 version;
    u16 header_bytes;
    u16 entry_count;
    u16 entry_bytes;
    u32 flags;
    u64 image_base_hint;
    u64 image_size;
} PACKED;

struct pios_stage2_manifest_entry {
    u32 platform_id;
    u32 config_id;
    u64 entry_offset;
    u64 required_features;
    u64 optional_features;
    char name[32];
} PACKED;

#define PIOS_STAGE2_PACKAGED_ENTRY_BYTES 96U

struct pios_stage2_packaged_entry {
    struct pios_stage2_manifest_entry base;
    u64 payload_offset;
    u64 payload_size;
    u64 load_addr;
    u64 memory_size;
} PACKED;

extern const u8 pios_stage2_manifest_start[];
extern const u8 pios_stage2_manifest_end[];
