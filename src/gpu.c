/*
 * gpu.c - VideoCore VII GPU interface
 *
 * GPU memory management, VPU code execution, and QPU compute dispatch.
 * All via VideoCore mailbox property tags — no Linux, no V3D DRM.
 */

#include "gpu.h"
#include "mailbox.h"
#include "mmio.h"
#include "uart.h"

/* Mailbox property tags for GPU operations */
#define TAG_ALLOCATE_MEMORY     0x0003000C
#define TAG_LOCK_MEMORY         0x0003000D
#define TAG_UNLOCK_MEMORY       0x0003000E
#define TAG_RELEASE_MEMORY      0x0003000F
#define TAG_EXECUTE_CODE        0x00030010
#define TAG_QPU_ENABLE          0x00030012
#define TAG_QPU_EXECUTE         0x00030011

/* Shared mailbox buffer — 16-byte aligned */
static volatile u32 __attribute__((aligned(16))) gpu_mbox[32];

void gpu_init(void) {
    uart_puts("[gpu] VideoCore VII interface ready\n");
}

/* ---- GPU Memory Management ---- */

u32 gpu_mem_alloc(u32 size, u32 align, u32 flags) {
    gpu_mbox[0] = 9 * 4;       /* total size */
    gpu_mbox[1] = 0;            /* request */
    gpu_mbox[2] = TAG_ALLOCATE_MEMORY;
    gpu_mbox[3] = 12;           /* value buffer size */
    gpu_mbox[4] = 12;           /* request size */
    gpu_mbox[5] = size;
    gpu_mbox[6] = align;
    gpu_mbox[7] = flags;
    gpu_mbox[8] = 0;            /* end tag */

    if (!mbox_call(MBOX_CH_PROP, gpu_mbox))
        return 0;
    return gpu_mbox[5];         /* handle */
}

u32 gpu_mem_lock(u32 handle) {
    gpu_mbox[0] = 7 * 4;
    gpu_mbox[1] = 0;
    gpu_mbox[2] = TAG_LOCK_MEMORY;
    gpu_mbox[3] = 4;
    gpu_mbox[4] = 4;
    gpu_mbox[5] = handle;
    gpu_mbox[6] = 0;

    if (!mbox_call(MBOX_CH_PROP, gpu_mbox))
        return 0;
    return gpu_mbox[5];         /* bus address */
}

void gpu_mem_unlock(u32 handle) {
    gpu_mbox[0] = 7 * 4;
    gpu_mbox[1] = 0;
    gpu_mbox[2] = TAG_UNLOCK_MEMORY;
    gpu_mbox[3] = 4;
    gpu_mbox[4] = 4;
    gpu_mbox[5] = handle;
    gpu_mbox[6] = 0;

    mbox_call(MBOX_CH_PROP, gpu_mbox);
}

void gpu_mem_free(u32 handle) {
    gpu_mbox[0] = 7 * 4;
    gpu_mbox[1] = 0;
    gpu_mbox[2] = TAG_RELEASE_MEMORY;
    gpu_mbox[3] = 4;
    gpu_mbox[4] = 4;
    gpu_mbox[5] = handle;
    gpu_mbox[6] = 0;

    mbox_call(MBOX_CH_PROP, gpu_mbox);
}

/* ---- GPU Execute Code ---- */

u32 gpu_execute(u32 func_ptr, u32 r0, u32 r1, u32 r2,
                u32 r3, u32 r4, u32 r5) {
    gpu_mbox[0]  = 13 * 4;
    gpu_mbox[1]  = 0;
    gpu_mbox[2]  = TAG_EXECUTE_CODE;
    gpu_mbox[3]  = 28;         /* 7 × 4 bytes value buffer */
    gpu_mbox[4]  = 28;
    gpu_mbox[5]  = func_ptr;
    gpu_mbox[6]  = r0;
    gpu_mbox[7]  = r1;
    gpu_mbox[8]  = r2;
    gpu_mbox[9]  = r3;
    gpu_mbox[10] = r4;
    gpu_mbox[11] = r5;
    gpu_mbox[12] = 0;

    if (!mbox_call(MBOX_CH_PROP, gpu_mbox))
        return 0xFFFFFFFF;
    return gpu_mbox[5];         /* return value from VC */
}

/* ---- QPU Enable/Execute ---- */

bool qpu_enable(bool enable) {
    gpu_mbox[0] = 7 * 4;
    gpu_mbox[1] = 0;
    gpu_mbox[2] = TAG_QPU_ENABLE;
    gpu_mbox[3] = 4;
    gpu_mbox[4] = 4;
    gpu_mbox[5] = enable ? 1 : 0;
    gpu_mbox[6] = 0;

    return mbox_call(MBOX_CH_PROP, gpu_mbox);
}

bool qpu_execute(u32 num_qpus, u32 control, bool noflush) {
    gpu_mbox[0]  = 10 * 4;
    gpu_mbox[1]  = 0;
    gpu_mbox[2]  = TAG_QPU_EXECUTE;
    gpu_mbox[3]  = 16;         /* 4 × 4 bytes */
    gpu_mbox[4]  = 16;
    gpu_mbox[5]  = num_qpus;
    gpu_mbox[6]  = control;    /* bus addr of (uniform, code) pairs */
    gpu_mbox[7]  = noflush ? 1 : 0;
    gpu_mbox[8]  = 0;          /* timeout (0 = default) */
    gpu_mbox[9]  = 0;

    return mbox_call(MBOX_CH_PROP, gpu_mbox);
}

/* ---- GPU DMA Copy ---- */

bool gpu_dma_copy(u32 dst_bus, u32 src_bus, u32 len) {
    /*
     * There's no direct "DMA copy" mailbox tag. Instead we use
     * the VPU execute path with a tiny VC program that does:
     *   memcpy(r0, r1, r2); return 0;
     *
     * The VPU firmware has a built-in memcpy at well-known addresses.
     * For now, we fall back to reporting this as unsupported and
     * the caller should use BCM2712 DMA instead.
     *
     * This stub exists so the API is ready when we have the VC
     * firmware entry point mapped.
     */
    (void)dst_bus;
    (void)src_bus;
    (void)len;
    return false;
}
