/*
 * tensor.h - QPU tensor compute offload
 *
 * Uses VideoCore VII's 12 QPUs for parallel vector/matrix ops.
 * QPUs are 4-wide SIMD, dual-issue, 800MHz = 76.8 GFLOPS peak.
 *
 * We provide high-level tensor primitives that:
 *   1. Allocate GPU-side buffers
 *   2. Upload data + QPU shader code
 *   3. Dispatch across QPUs
 *   4. Read back results
 *
 * Data format: 32-bit float or 16-bit half (QPU native).
 */

#pragma once
#include "types.h"

#define QPU_MAX_INSTANCES   12

/* Tensor descriptor — lives in GPU-accessible memory */
typedef struct {
    u32 handle;         /* GPU memory handle */
    u32 bus_addr;       /* Bus address (for QPU access) */
    void *arm_ptr;      /* ARM-side pointer (bus & 0x3FFFFFFF) */
    u32 rows;
    u32 cols;
    u32 elem_size;      /* 4 = float32, 2 = float16 */
    u32 total_bytes;
} tensor_t;

/* QPU program handle */
typedef struct {
    u32 handle;
    u32 bus_addr;
    u32 code_size;
} qpu_program_t;

/* QPU job control — one per QPU instance */
struct qpu_job {
    u32 uniform_addr;   /* bus addr of uniforms for this QPU */
    u32 shader_addr;    /* bus addr of shader code */
};

/* ---- Tensor lifecycle ---- */

/* Allocate a tensor in GPU memory. Returns false if allocation fails. */
bool tensor_alloc(tensor_t *t, u32 rows, u32 cols, u32 elem_size);

/* Free a tensor's GPU memory. */
void tensor_free(tensor_t *t);

/* Upload data from ARM memory to tensor. */
void tensor_upload(tensor_t *t, const void *data);

/* Download tensor data to ARM memory. */
void tensor_download(const tensor_t *t, void *data);

/* Zero a tensor. */
void tensor_zero(tensor_t *t);

/* ---- Built-in QPU operations ---- */

/* Element-wise add: C = A + B (all same shape) */
bool tensor_add(tensor_t *c, const tensor_t *a, const tensor_t *b);

/* Element-wise multiply: C = A * B */
bool tensor_mul(tensor_t *c, const tensor_t *a, const tensor_t *b);

/* Scalar multiply: B = A * scalar */
bool tensor_scale(tensor_t *b, const tensor_t *a, float scalar);

/* Dot product: result = sum(A[i] * B[i]) — vectors only */
bool tensor_dot(float *result, const tensor_t *a, const tensor_t *b);

/* Matrix multiply: C[m×p] = A[m×n] × B[n×p]
 * Distributes across QPUs by row blocks. */
bool tensor_matmul(tensor_t *c, const tensor_t *a, const tensor_t *b);

/* ReLU activation: B[i] = max(0, A[i]) */
bool tensor_relu(tensor_t *b, const tensor_t *a);

/* Softmax: B = softmax(A) along cols — per-row softmax */
bool tensor_softmax(tensor_t *b, const tensor_t *a);

/* ---- Low-level QPU dispatch ---- */

/* Load a QPU shader program into GPU memory.
 * `code` is QPU machine code (assembled offline or built-in).
 * Returns false if allocation fails. */
bool qpu_load_program(qpu_program_t *prog, const u32 *code, u32 code_words);

/* Free a QPU program. */
void qpu_free_program(qpu_program_t *prog);

/* Dispatch a QPU program across `num_qpus` instances.
 * `jobs` array has one entry per QPU with uniform + shader pointers.
 * Blocks until all QPUs complete. */
bool qpu_dispatch(const qpu_program_t *prog, struct qpu_job *jobs,
                  u32 num_qpus);

/* ---- Init ---- */
void tensor_init(void);
