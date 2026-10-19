#pragma once
/*
 * Core-0 placement scheduler for PicoScript tensor work.
 *
 * The CPU agent owns the math. Register it with the same function shape as
 * tensor_service's ts_execute. This module only decides where a request
 * runs, whether an already-loaded Arc tensor can be reused, and when a
 * fabric DMA must complete before the FIFO reply is visible.
 *
 * It does not map BARs, program the GuC, or copy tensor bytes.
 */
#include "types.h"

#define PIOS_EXEC_DEVICES 8U
#define PIOS_EXEC_JOBS 16U
#define PIOS_EXEC_RESIDENT 32U

#define PIOS_EXEC_OK 0
#define PIOS_EXEC_INVALID (-1)
#define PIOS_EXEC_FULL (-2)
#define PIOS_EXEC_STALE (-3)
#define PIOS_EXEC_UNAVAILABLE (-4)
#define PIOS_EXEC_BUSY (-5)

#define PIOS_EXEC_FREE 0U
#define PIOS_EXEC_QUEUED 1U
#define PIOS_EXEC_WAIT_DMA 2U
#define PIOS_EXEC_COMPLETE 3U
#define PIOS_EXEC_FAILED 4U

#define PIOS_EXEC_BACKEND_NONE 0U
#define PIOS_EXEC_BACKEND_CPU 1U
#define PIOS_EXEC_BACKEND_B50 2U
#define PIOS_EXEC_BACKEND_B60 3U

struct pcie_fabric_snapshot;

/* Same call shape as ts_execute. Null operands mean "place only". */
typedef bool (*pios_exec_cpu_fn)(const i8 *matrix, const i8 *vector,
                                 u32 rows, u32 cols, i32 *out);

struct pios_exec_device {
    u16 vendor;
    u16 device_id;
    u8 bus, dev, func;
    u8 backend;
    u32 generation;
} ALIGNED(16);

struct pios_exec_resident {
    u64 model_id;
    u32 layer, expert, shard;
    u8 bus, dev, func;
    u8 live;
    u32 bytes;
} ALIGNED(32);

struct pios_exec_job {
    u32 generation, state, backend, device_index, src_device;
    u32 dma, reused, layer, expert, shard, bytes, rows, cols;
    i32 status;
    u64 model_id;
    const i8 *matrix;
    const i8 *vector;
    i32 *out;
} ALIGNED(64);

struct pios_exec_sched {
    struct pios_exec_device devices[PIOS_EXEC_DEVICES];
    struct pios_exec_resident resident[PIOS_EXEC_RESIDENT];
    struct pios_exec_job jobs[PIOS_EXEC_JOBS];
    u32 ring[PIOS_EXEC_JOBS];
    u32 head, count, device_count, model_generation;
    u64 model_id;
    pios_exec_cpu_fn cpu;
    u32 last_backend, last_device, last_dma, last_reused;
    i32 last_status;
};

void pios_exec_init(struct pios_exec_sched *s);
void pios_exec_bind_cpu(struct pios_exec_sched *s, pios_exec_cpu_fn fn);
void pios_exec_set_model(struct pios_exec_sched *s, u64 model_id);

/* Quiescent. Fails while a job is still queued or waiting on DMA.
 * Keeps only reachable B50 (8086:E212) and B60 (8086:E211) functions. */
bool pios_exec_observe(struct pios_exec_sched *s,
                       const struct pcie_fabric_snapshot *fabric);

bool pios_exec_note_resident(struct pios_exec_sched *s, u64 model_id,
                             u32 layer, u32 expert, u32 shard,
                             u8 bus, u8 dev, u8 func, u32 bytes);

/* target 0 chooses a resident online Arc, else the CPU backend.
 * target is a 1-based device index from observe order. */
i32 pios_exec_submit(struct pios_exec_sched *s, u32 layer, u32 expert,
                     u32 shard, u32 target, u32 *handle);
i32 pios_exec_submit_cpu(struct pios_exec_sched *s, u32 layer, u32 expert,
                         u32 shard, u32 target,
                         const i8 *matrix, const i8 *vector,
                         u32 rows, u32 cols, i32 *out, u32 *handle);

/* Place the oldest queued request. One request per call. */
bool pios_exec_step(struct pios_exec_sched *s);
/* Completion for a fabric DMA. The reply stays hidden until this. */
i32 pios_exec_dma_done(struct pios_exec_sched *s, u32 handle);

/* FIFO reply: pops the oldest terminal job. BUSY while it is still open. */
i32 pios_exec_reply(struct pios_exec_sched *s, u32 *packed);
i32 pios_exec_where(const struct pios_exec_sched *s, u32 layer, u32 expert);

/* PicoScript host hook. Returns 1 when the hook id is ours. */
struct pv_ctx;
int pios_exec_sched_vm_hook(struct pv_ctx *ctx, int hook,
                            int rd, int rs1, int rs2);
void pios_exec_sched_bind(struct pios_exec_sched *s);
void pios_exec_sched_install(void);
