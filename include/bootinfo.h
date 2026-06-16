#pragma once
#include "types.h"

#define PIOS_BOOTINFO_MAGIC   0x5042494FU  /* 'PBIO' */
#define PIOS_BOOTINFO_VERSION 1U
#define PIOS_BOOTINFO_ADDR    0x41F00000UL

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
