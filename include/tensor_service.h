#pragma once
#include "types.h"
#include "picovm.h"

#define TS_DEPTH 4U
#define TS_ROWS 16U
#define TS_COLS 256U
#define TS_AUTO 0U
#define TS_CPU 1U
#define TS_QPU 2U
#define TS_I8_I32 1U
#define TS_OK 0
#define TS_INVALID (-1)
#define TS_FULL (-2)
#define TS_STALE (-3)
#define TS_UNAVAILABLE (-4)
#define TS_EXECUTION (-5)
#define TS_OWNER (-6)

/* Trusted same-core service only. No EL0 or cross-core pointer transport.
 * Admission copies inputs; completion owns output until explicit release.
 * All entry points run on the owning core in scheduled context, never IRQ.
 * Callers must externally enforce that execution-context contract. */
typedef bool (*ts_execute)(const i8 *, const i8 *, u32, u32, i32 *);
struct ts_backend {
    ts_execute execute;
    u32 generation, precision, max_rows, max_cols, col_multiple;
    bool live;
} ALIGNED(64);
struct ts_job {
    u32 generation, backend, backend_generation, rows, cols;
    i32 status;
    u32 state;
} ALIGNED(64);
struct ts_payload {
    i8 matrix[TS_ROWS * TS_COLS];
    i8 vector[TS_COLS];
    i32 output[TS_ROWS];
} ALIGNED(64);
struct tensor_service {
    struct ts_backend backends[2];
    struct ts_job jobs[TS_DEPTH];
    struct ts_payload payload[TS_DEPTH];
    u32 queue[TS_DEPTH], head, count;
} ALIGNED(64);

i32 ts_register(struct tensor_service *, u32, ts_execute, u32, u32, u32, u32);
i32 ts_retire(struct tensor_service *, u32);
i32 ts_submit(struct tensor_service *, u32, u32, const i8 *, u32,
              const i8 *, u32, u32, u32, u32 *);
bool ts_step(struct tensor_service *);
i32 ts_result(struct tensor_service *, u32, i32 *, u32);
i32 ts_release(struct tensor_service *, u32);
/* Resolves LOCAL VM span bounds and copies them before queue publication.
 * This is not an OS lease grant. It must not be exposed as an EL0 bridge. */
i32 ts_vm_matvec(struct tensor_service *, pv_ctx *, u32, i32, i32);
/* Trusted adapter for canonical i8 MatVec with request-scoped placement.
 * Not the production OS broker, DMA grant, or EL0 arena lifecycle. */
i32 ts_vm_pinned(struct tensor_service *, pv_ctx *, i32, u32);
i32 ts_vm_selected(pv_ctx *, i32);
