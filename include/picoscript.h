#pragma once
#include "types.h"

#define PICOSCRIPT_MAGIC       0x31434250U /* 'PBC1' little-endian */
#define PICOSCRIPT_VERSION     1U
#define PICOSCRIPT_MAX_IMAGE   4096U
#define PICOSCRIPT_MAX_LINE    255U

struct picoscript_header {
    u32 magic;
    u16 version;
    u16 command_count;
    u32 image_bytes;
    u32 reserved;
} PACKED;

struct picoscript_record {
    u16 length;
    u8 flags;
    u8 reserved;
    /* command bytes follow, no terminator */
} PACKED;
