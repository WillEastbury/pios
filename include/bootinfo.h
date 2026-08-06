#pragma once
#include "types.h"

#define PIOS_BOOTINFO_MAGIC   0x5042494FU  /* 'PBIO' */
#define PIOS_BOOTINFO_VERSION 1U
/* QEMU RAM starts at 0x40000000 and the kernel is linked at 0x40080000.
 * Keep bootinfo in its own loader-reserved page immediately below the kernel,
 * outside the stage2 image/BSS/heap and all per-core private arenas. */
#define PIOS_BOOTINFO_ADDR    0x40070000UL

struct pios_bootinfo {
    u32 magic;
    u32 version;
    u64 flags;
    u64 framebuffer_base;
    u32 framebuffer_width;
    u32 framebuffer_height;
    u32 framebuffer_pitch;
    u32 framebuffer_format;
} PACKED;

#define PIOS_BOOTINFO_FLAG_FRAMEBUFFER (1ULL << 0)
