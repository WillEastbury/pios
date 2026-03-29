/*
 * gpu.h - VideoCore VII GPU offload interface
 *
 * Three tiers:
 *   1. GPU DMA — mailbox-triggered bulk memory moves
 *   2. QPU compute — submit SIMD jobs to 12 QPUs
 *   3. Mailbox utilities — memory alloc/lock on GPU side
 */

#pragma once
#include "types.h"

/* ---- GPU Memory Management (via mailbox) ---- */

/* Allocate GPU memory. Returns handle (0 = fail). */
u32 gpu_mem_alloc(u32 size, u32 align, u32 flags);

/* Lock GPU memory handle → bus address. */
u32 gpu_mem_lock(u32 handle);

/* Unlock GPU memory handle. */
void gpu_mem_unlock(u32 handle);

/* Free GPU memory handle. */
void gpu_mem_free(u32 handle);

/* GPU memory flags */
#define GPU_MEM_FLAG_DISCARDABLE    (1 << 0)
#define GPU_MEM_FLAG_NORMAL         (0 << 2)
#define GPU_MEM_FLAG_DIRECT         (1 << 2)  /* uncached */
#define GPU_MEM_FLAG_COHERENT       (2 << 2)  /* coherent with ARM */
#define GPU_MEM_FLAG_L1_NONALLOC    (1 << 3)
#define GPU_MEM_FLAG_ZERO           (1 << 4)
#define GPU_MEM_FLAG_NO_INIT        (1 << 5)
#define GPU_MEM_FLAG_HINT_PERMALOCK (1 << 6)

/* ---- GPU Execute Code ---- */

/* Execute arbitrary code on the VideoCore VPU.
 * func_ptr = bus address of VC code.
 * r0-r5 = arguments passed in VC registers. */
u32 gpu_execute(u32 func_ptr, u32 r0, u32 r1, u32 r2,
                u32 r3, u32 r4, u32 r5);

/* ---- QPU Compute ---- */

/*
 * Submit a job to the V3D QPUs.
 * num_qpus: how many QPUs to launch (1-12)
 * control: bus address of control list (uniform stream pointers)
 * noflush: if true, don't flush caches (caller manages coherency)
 *
 * The control list is an array of (uniform_ptr, shader_ptr) pairs,
 * one per QPU instance. The shader code must be QPU assembly
 * already loaded into GPU-accessible memory.
 *
 * Returns true if submission succeeded.
 */
bool qpu_execute(u32 num_qpus, u32 control, bool noflush);

/* Submit with an explicit firmware timeout (milliseconds). */
bool qpu_execute_timeout(u32 num_qpus, u32 control, bool noflush, u32 timeout_ms);

/* Enable/disable QPU access (must call before qpu_execute) */
bool qpu_enable(bool enable);

/* ---- GPU DMA (mailbox-based) ---- */

/* GPU-side DMA: copy `len` bytes from `src` to `dst` (bus addresses).
 * This uses the VideoCore's own DMA engine, separate from BCM2712 DMA.
 * Useful for GPU↔ARM memory transfers when data is in GPU-allocated buffers. */
bool gpu_dma_copy(u32 dst_bus, u32 src_bus, u32 len);

/* ---- Initialization ---- */

void gpu_init(void);
