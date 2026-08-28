#pragma once

#include "platform.h"
#include "types.h"

#define STAGE0_DIAG_MAGIC   0x53304447U
#define STAGE0_DIAG_VERSION 1U
#define STAGE0_DIAG_ADDR    (PIOS_IPC_SHM_BASE + PIOS_IPC_SHM_SIZE - 192U)
#define STAGE0_DIAG_ATTEMPTS 2U
#define STAGE2_BOOT_DIAG_MAGIC 0x53324244U
#define STAGE2_BOOT_DIAG_ADDR  (STAGE0_DIAG_ADDR - 64U)
#define STAGE0_FB_HANDOFF_MAGIC   0x53304642U
#define STAGE0_FB_HANDOFF_VERSION 1U
#define STAGE0_FB_HANDOFF_ADDR    (STAGE2_BOOT_DIAG_ADDR - 64U)

struct stage0_fb_handoff {
    u32 magic;
    u32 version;
    u64 base;
    u32 width;
    u32 height;
    u32 pitch;
    u32 size;
    u32 checksum;
    u32 reserved[7];
} ALIGNED(64);

struct stage2_boot_diag {
    u32 magic;
    u32 stage;
    u32 stage_inverse;
    u32 reserved[13];
} ALIGNED(64);

struct stage0_fb_attempt {
    u64 sctlr_el1;
    u32 current_el;
    u32 status;
    u32 message;
    u32 response;
    u32 allocation_addr;
    u32 allocation_size;
    u32 pitch;
    u32 reserved[5];
} ALIGNED(64);

struct stage0_diag {
    u32 magic;
    u32 version;
    u32 attempt_count;
    u32 framebuffer_ready;
    u32 hello_attempted;
    u32 hello_status;
    u32 hello_response;
    u32 hello_allocation_addr;
    u32 hello_allocation_size;
    u32 reserved[7];
    struct stage0_fb_attempt attempt[STAGE0_DIAG_ATTEMPTS];
} ALIGNED(64);

_Static_assert(sizeof(struct stage0_fb_attempt) == 64U,
               "stage0 framebuffer attempt must own one cache line");
_Static_assert(sizeof(struct stage0_diag) == 192U,
               "stage0 diagnostics must have a fixed cache-line stride");
_Static_assert((STAGE0_DIAG_ADDR & 63UL) == 0UL,
               "stage0 diagnostics must be cache-line aligned");
_Static_assert(sizeof(struct stage2_boot_diag) == 64U,
               "stage2 boot diagnostics must own one cache line");
_Static_assert((STAGE2_BOOT_DIAG_ADDR & 63UL) == 0UL,
               "stage2 boot diagnostics must be cache-line aligned");
_Static_assert(sizeof(struct stage0_fb_handoff) == 64U,
               "stage0 framebuffer handoff must own one cache line");
_Static_assert((STAGE0_FB_HANDOFF_ADDR & 63UL) == 0UL,
               "stage0 framebuffer handoff must be cache-line aligned");

static inline u32 stage0_fb_handoff_checksum(u64 base, u32 width, u32 height,
                                             u32 pitch, u32 size)
{
    return STAGE0_FB_HANDOFF_VERSION ^ (u32)base ^ (u32)(base >> 32) ^
           width ^ height ^ pitch ^ size ^ 0xA55AF00DU;
}

static inline void stage0_fb_handoff_publish(u64 base, u32 width, u32 height,
                                              u32 pitch, u32 size)
{
    volatile struct stage0_fb_handoff *handoff =
        (volatile struct stage0_fb_handoff *)(usize)STAGE0_FB_HANDOFF_ADDR;
    handoff->magic = 0U;
    handoff->version = STAGE0_FB_HANDOFF_VERSION;
    handoff->base = base;
    handoff->width = width;
    handoff->height = height;
    handoff->pitch = pitch;
    handoff->size = size;
    handoff->checksum = stage0_fb_handoff_checksum(base, width, height,
                                                   pitch, size);
    dsb();
    handoff->magic = STAGE0_FB_HANDOFF_MAGIC;
    dsb();
}

static inline bool stage0_fb_handoff_read(struct stage0_fb_handoff *out)
{
    if (!out)
        return false;
    volatile const struct stage0_fb_handoff *handoff =
        (volatile const struct stage0_fb_handoff *)(usize)STAGE0_FB_HANDOFF_ADDR;
    dsb();
    struct stage0_fb_handoff copy = *handoff;
    if (copy.magic != STAGE0_FB_HANDOFF_MAGIC ||
        copy.version != STAGE0_FB_HANDOFF_VERSION ||
        copy.checksum != stage0_fb_handoff_checksum(copy.base, copy.width,
                                                     copy.height, copy.pitch,
                                                     copy.size))
        return false;
    *out = copy;
    return true;
}

static inline void stage2_boot_diag_mark(u32 stage)
{
    volatile struct stage2_boot_diag *diag =
        (volatile struct stage2_boot_diag *)(usize)STAGE2_BOOT_DIAG_ADDR;
    diag->magic = STAGE2_BOOT_DIAG_MAGIC;
    diag->stage = stage;
    diag->stage_inverse = ~stage;
    dsb();
}
