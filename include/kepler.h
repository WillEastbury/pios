#pragma once
#include "types.h"

#define KEPLER_ROM_LIMIT 0x00100000U
#define KEPLER_ROM_CHUNK 128U
#define KEPLER_VRAM_BASE 0x00100000U
#define KEPLER_VRAM_BYTES 0x00400000U
#define KEPLER_VRAM_CHUNK 128U

enum kepler_result {
    KEPLER_OK = 0,
    KEPLER_UNSUPPORTED,
    KEPLER_WRONG_OWNER,
    KEPLER_NOT_READY,
    KEPLER_IDENTITY,
    KEPLER_MAPPING,
    KEPLER_DMA_ENABLED,
    KEPLER_ROM_SHADOWED,
    KEPLER_RANGE,
    KEPLER_AER,
    KEPLER_I2C_TIMEOUT,
    KEPLER_I2C_NACK,
    KEPLER_VRAM_VERIFY,
};

/* Core 0 owns the snapshot; no IRQ or other core mutates it. */
struct kepler_status {
    u32 generation;
    u32 result;
    u32 bdf;
    u32 boot0;
    u32 post;
    u32 engines;
    u32 fuse;
    u32 bios_shadow;
    u32 rom_shadow;
    u32 pci_command;
    u32 probe_ok;
    u32 posted;
    u32 rom_available;
    u32 compute_ready;
    u32 reserved[2];
} ALIGNED(64);
_Static_assert(sizeof(struct kepler_status) == 64U, "Kepler status owns a cache line");

/* Explicit diagnostics only. These do not initialize engines or enable DMA. */
enum kepler_result kepler_probe(void);
void kepler_snapshot(struct kepler_status *out);
enum kepler_result kepler_rom_read(u32 offset, u8 *out, u32 length);
/* Current board's VBIOS uses unshared I2C drive 2, address 0x4c.
 * One byte transaction, 2 ms limit; explicit diagnostics, not a general bus API. */
enum kepler_result kepler_i2c_byte(bool write, u8 reg, u8 *value);
/* Operator-owned bring-up arena only; stale probe generations are refused.
 * mode: 0 read, 1 write, 2 zero. No channel or DMA is launched here. */
enum kepler_result kepler_vram(u32 generation, u32 mode, u32 offset,
                               u8 *bytes, u32 length);
const char *kepler_result_name(enum kepler_result result);
