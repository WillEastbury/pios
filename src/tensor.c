/*
 * tensor.c - QPU tensor compute offload
 *
 * Allocates GPU memory for tensors, uploads QPU shader microcode,
 * dispatches across up to 12 QPUs via mailbox, reads back results.
 *
 * QPU programs are pre-assembled 32-bit instruction words.
 * VideoCore VII QPU: 4-wide float SIMD, dual-issue ALU + SFU.
 */

#include "tensor.h"
#include "gpu.h"
#include "simd.h"
#include "uart.h"

/* ---- Built-in QPU microcode ---- */

/*
 * QPU instruction encoding for VideoCore VII:
 *   Each QPU instruction is 64 bits (2 x u32).
 *   These are minimal kernels that:
 *     - Read uniforms (input addresses, counts)
 *     - Load data via TMU (Texture Memory Unit)
 *     - Process with ALU
 *     - Write results via TLB/VPM
 *
 * NOTE: VideoCore VII QPU ISA is not fully public.
 * These are placeholder shaders that demonstrate the dispatch
 * framework. Real QPU code would be assembled from the
 * VC VII instruction set once fully reverse-engineered.
 *
 * For now, tensor ops fall back to NEON on the ARM side
 * while the QPU dispatch infrastructure is ready and waiting.
 */

/* Placeholder: the actual QPU shader dispatch works but we
 * run the math on ARM/NEON until VC VII ISA is fully mapped. */
static bool use_qpu_fallback = true;

/* ---- Tensor lifecycle ---- */

bool tensor_alloc(tensor_t *t, u32 rows, u32 cols, u32 elem_size) {
    t->rows       = rows;
    t->cols       = cols;
    t->elem_size  = elem_size;
    t->total_bytes = rows * cols * elem_size;

    /* Align to 4KB for DMA/QPU access */
    u32 alloc_size = (t->total_bytes + 4095) & ~4095U;

    t->handle = gpu_mem_alloc(alloc_size, 4096,
                              GPU_MEM_FLAG_COHERENT | GPU_MEM_FLAG_ZERO);
    if (!t->handle) return false;

    t->bus_addr = gpu_mem_lock(t->handle);
    if (!t->bus_addr) {
        gpu_mem_free(t->handle);
        t->handle = 0;
        return false;
    }

    /* Convert bus address to ARM physical address */
    t->arm_ptr = (void *)(usize)(t->bus_addr & 0x3FFFFFFF);
    return true;
}

void tensor_free(tensor_t *t) {
    if (t->handle) {
        gpu_mem_unlock(t->handle);
        gpu_mem_free(t->handle);
        t->handle   = 0;
        t->bus_addr = 0;
        t->arm_ptr  = NULL;
    }
}

void tensor_upload(tensor_t *t, const void *data) {
    simd_memcpy(t->arm_ptr, data, t->total_bytes);
    dsb();
}

void tensor_download(const tensor_t *t, void *data) {
    dsb();
    simd_memcpy(data, t->arm_ptr, t->total_bytes);
}

void tensor_zero(tensor_t *t) {
    simd_zero(t->arm_ptr, t->total_bytes);
    dsb();
}

/* ---- NEON fallback implementations ---- */

/* These run on the ARM A76 with NEON when QPU shaders aren't available.
 * Still fast: 128-bit SIMD, dual-issue, 2.4GHz. */

static void neon_vec_add_f32(float *c, const float *a, const float *b, u32 n) {
    u32 i = 0;
    /* 4 floats per NEON register */
    for (; i + 4 <= n; i += 4) {
        __asm__ volatile(
            "ld1 {v0.4s}, [%1], #16  \n"
            "ld1 {v1.4s}, [%2], #16  \n"
            "fadd v2.4s, v0.4s, v1.4s \n"
            "st1 {v2.4s}, [%0], #16  \n"
            : "+r"(c), "+r"(a), "+r"(b)
            :: "v0","v1","v2","memory"
        );
    }
    for (; i < n; i++)
        c[i] = a[i] + b[i];
}

static void neon_vec_mul_f32(float *c, const float *a, const float *b, u32 n) {
    u32 i = 0;
    for (; i + 4 <= n; i += 4) {
        __asm__ volatile(
            "ld1 {v0.4s}, [%1], #16  \n"
            "ld1 {v1.4s}, [%2], #16  \n"
            "fmul v2.4s, v0.4s, v1.4s \n"
            "st1 {v2.4s}, [%0], #16  \n"
            : "+r"(c), "+r"(a), "+r"(b)
            :: "v0","v1","v2","memory"
        );
    }
    for (; i < n; i++)
        c[i] = a[i] * b[i];
}

static void neon_vec_scale_f32(float *b, const float *a, float s, u32 n) {
    u32 i = 0;
    for (; i + 4 <= n; i += 4) {
        __asm__ volatile(
            "ld1  {v0.4s}, [%1], #16 \n"
            "dup  v1.4s, %w3         \n"
            "fmul v2.4s, v0.4s, v1.4s \n"
            "st1  {v2.4s}, [%0], #16 \n"
            : "+r"(b), "+r"(a)
            : "r"(b), "r"(*(u32 *)&s)
            : "v0","v1","v2","memory"
        );
    }
    for (; i < n; i++)
        b[i] = a[i] * s;
}

static float neon_vec_dot_f32(const float *a, const float *b, u32 n) {
    float result = 0.0f;
    u32 i = 0;

    if (n >= 16) {
        /* Accumulate in 4 NEON lanes */
        __asm__ volatile("movi v4.4s, #0" ::: "v4"); /* accumulator */
        for (; i + 4 <= n; i += 4) {
            __asm__ volatile(
                "ld1  {v0.4s}, [%0], #16  \n"
                "ld1  {v1.4s}, [%1], #16  \n"
                "fmla v4.4s, v0.4s, v1.4s \n"
                : "+r"(a), "+r"(b)
                :: "v0","v1","v4","memory"
            );
        }
        /* Horizontal reduction: v4 → scalar */
        __asm__ volatile(
            "faddp v4.4s, v4.4s, v4.4s \n"
            "faddp s4, v4.2s           \n"
            "fmov  %w0, s4             \n"
            : "=r"(*(u32 *)&result)
            :: "v4"
        );
    }
    for (; i < n; i++)
        result += a[i] * b[i];
    return result;
}

static void neon_vec_relu_f32(float *b, const float *a, u32 n) {
    u32 i = 0;
    __asm__ volatile("movi v1.4s, #0" ::: "v1");
    for (; i + 4 <= n; i += 4) {
        __asm__ volatile(
            "ld1  {v0.4s}, [%1], #16  \n"
            "fmax v0.4s, v0.4s, v1.4s \n"
            "st1  {v0.4s}, [%0], #16  \n"
            : "+r"(b), "+r"(a)
            :: "v0","v1","memory"
        );
    }
    for (; i < n; i++)
        b[i] = (a[i] > 0.0f) ? a[i] : 0.0f;
}

/* Naive NEON matmul: C[m×p] = A[m×n] × B[n×p]
 * Uses FMLA (fused multiply-accumulate) for inner loop. */
static void neon_matmul_f32(float *c, const float *a, const float *b,
                            u32 m, u32 n, u32 p) {
    /* Transpose B into a scratch area so inner-loop columns become rows.
     * This makes the inner product operate on contiguous memory → NEON-friendly. */
    static float bt[4096] ALIGNED(64); /* max 4096 floats = 64×64 */
    bool transposed = (n * p <= 4096);

    if (transposed) {
        for (u32 j = 0; j < p; j++)
            for (u32 k = 0; k < n; k++)
                bt[j * n + k] = b[k * p + j];
    }

    for (u32 i = 0; i < m; i++) {
        const float *a_row = a + i * n;
        for (u32 j = 0; j < p; j++) {
            const float *b_row = transposed ? (bt + j * n) : NULL;
            u32 k = 0;

            if (transposed && n >= 4) {
                /* NEON FMLA: 4 floats per cycle */
                __asm__ volatile("movi v4.4s, #0" ::: "v4");
                const float *ap = a_row;
                const float *bp = b_row;
                for (; k + 4 <= n; k += 4) {
                    __asm__ volatile(
                        "ld1  {v0.4s}, [%0], #16  \n"
                        "ld1  {v1.4s}, [%1], #16  \n"
                        "fmla v4.4s, v0.4s, v1.4s \n"
                        : "+r"(ap), "+r"(bp)
                        :: "v0","v1","v4","memory"
                    );
                }
                float sum;
                __asm__ volatile(
                    "faddp v4.4s, v4.4s, v4.4s \n"
                    "faddp s4, v4.2s           \n"
                    "fmov  %w0, s4             \n"
                    : "=r"(*(u32 *)&sum) :: "v4"
                );
                for (; k < n; k++)
                    sum += a_row[k] * b_row[k];
                c[i * p + j] = sum;
            } else {
                /* Scalar fallback for strided B or small n */
                float sum = 0.0f;
                const float *b_col = b + j;
                for (k = 0; k < n; k++)
                    sum += a_row[k] * b_col[k * p];
                c[i * p + j] = sum;
            }
        }
    }
}

/* ---- Public tensor operations ---- */

bool tensor_add(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n || c->rows * c->cols != n) return false;

    if (use_qpu_fallback) {
        neon_vec_add_f32((float *)c->arm_ptr,
                         (const float *)a->arm_ptr,
                         (const float *)b->arm_ptr, n);
        dsb();
        return true;
    }

    /* QPU path would go here when shaders are ready */
    return false;
}

bool tensor_mul(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n || c->rows * c->cols != n) return false;

    if (use_qpu_fallback) {
        neon_vec_mul_f32((float *)c->arm_ptr,
                         (const float *)a->arm_ptr,
                         (const float *)b->arm_ptr, n);
        dsb();
        return true;
    }
    return false;
}

bool tensor_scale(tensor_t *b, const tensor_t *a, float scalar) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n) return false;

    if (use_qpu_fallback) {
        neon_vec_scale_f32((float *)b->arm_ptr,
                           (const float *)a->arm_ptr, scalar, n);
        dsb();
        return true;
    }
    return false;
}

bool tensor_dot(float *result, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n) return false;

    if (use_qpu_fallback) {
        *result = neon_vec_dot_f32((const float *)a->arm_ptr,
                                   (const float *)b->arm_ptr, n);
        return true;
    }
    return false;
}

bool tensor_matmul(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    if (a->cols != b->rows) return false;
    if (c->rows != a->rows || c->cols != b->cols) return false;

    if (use_qpu_fallback) {
        neon_matmul_f32((float *)c->arm_ptr,
                        (const float *)a->arm_ptr,
                        (const float *)b->arm_ptr,
                        a->rows, a->cols, b->cols);
        dsb();
        return true;
    }
    return false;
}

bool tensor_relu(tensor_t *b, const tensor_t *a) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n) return false;

    if (use_qpu_fallback) {
        neon_vec_relu_f32((float *)b->arm_ptr,
                          (const float *)a->arm_ptr, n);
        dsb();
        return true;
    }
    return false;
}

bool tensor_softmax(tensor_t *b, const tensor_t *a) {
    if (b->rows != a->rows || b->cols != a->cols) return false;

    float *src = (float *)a->arm_ptr;
    float *dst = (float *)b->arm_ptr;
    u32 rows = a->rows;
    u32 cols = a->cols;

    for (u32 r = 0; r < rows; r++) {
        float *row_s = src + r * cols;
        float *row_d = dst + r * cols;
        u32 c;

        /* NEON max-finding: 4 lanes parallel compare */
        float max_val = row_s[0];
        if (cols >= 8) {
            __asm__ volatile(
                "ld1  {v0.4s}, [%1]    \n"
                :: "r"(&max_val), "r"(row_s) : "v0"
            );
            c = 4;
            for (; c + 4 <= cols; c += 4) {
                __asm__ volatile(
                    "ld1  {v1.4s}, [%0], #16  \n"
                    "fmax v0.4s, v0.4s, v1.4s \n"
                    : "+r"(row_s) :: "v0","v1","memory"
                );
            }
            row_s = src + r * cols; /* reset pointer */
            /* Horizontal max reduction */
            __asm__ volatile(
                "fmaxp v0.4s, v0.4s, v0.4s \n"
                "fmaxp s0, v0.2s           \n"
                "fmov  %w0, s0             \n"
                : "=r"(*(u32 *)&max_val) :: "v0"
            );
            for (; c < cols; c++)
                if (row_s[c] > max_val) max_val = row_s[c];
        } else {
            for (c = 1; c < cols; c++)
                if (row_s[c] > max_val) max_val = row_s[c];
        }

        /* exp(x - max) via Schraudolph's approximation and sum */
        float sum = 0.0f;
        for (c = 0; c < cols; c++) {
            float x = row_s[c] - max_val;
            if (x < -87.0f) x = -87.0f;
            union { float f; u32 i; } v;
            v.i = (u32)(12102203.2f * x + 1065353216.0f);
            row_d[c] = v.f;
            sum += row_d[c];
        }

        /* NEON normalize: multiply all by 1/sum */
        if (sum > 0.0f) {
            float inv_sum = 1.0f / sum;
            c = 0;
            if (cols >= 4) {
                __asm__ volatile("dup v2.4s, %w0" :: "r"(*(u32 *)&inv_sum) : "v2");
                for (; c + 4 <= cols; c += 4) {
                    __asm__ volatile(
                        "ld1  {v0.4s}, [%0]       \n"
                        "fmul v0.4s, v0.4s, v2.4s \n"
                        "st1  {v0.4s}, [%0], #16  \n"
                        : "+r"(row_d) :: "v0","v2","memory"
                    );
                }
                row_d = dst + r * cols + c; /* adjust for tail */
            }
            for (; c < cols; c++)
                row_d[c - c] = (dst + r * cols)[c] * inv_sum; /* tail scalar */
        }
    }
    dsb();
    return true;
}

/* ---- QPU program management ---- */

bool qpu_load_program(qpu_program_t *prog, const u32 *code, u32 code_words) {
    u32 size = code_words * 4;
    u32 alloc = (size + 4095) & ~4095U;

    prog->handle = gpu_mem_alloc(alloc, 4096, GPU_MEM_FLAG_COHERENT);
    if (!prog->handle) return false;

    prog->bus_addr = gpu_mem_lock(prog->handle);
    if (!prog->bus_addr) {
        gpu_mem_free(prog->handle);
        return false;
    }

    prog->code_size = size;

    /* Copy code to GPU memory */
    void *arm_ptr = (void *)(usize)(prog->bus_addr & 0x3FFFFFFF);
    simd_memcpy(arm_ptr, code, size);
    dsb();

    return true;
}

void qpu_free_program(qpu_program_t *prog) {
    if (prog->handle) {
        gpu_mem_unlock(prog->handle);
        gpu_mem_free(prog->handle);
        prog->handle   = 0;
        prog->bus_addr = 0;
    }
}

bool qpu_dispatch(const qpu_program_t *prog, struct qpu_job *jobs,
                  u32 num_qpus) {
    if (num_qpus == 0 || num_qpus > QPU_MAX_INSTANCES) return false;

    /* The control list is the jobs array itself — must be in GPU-visible mem.
     * For now we require the caller to have allocated it appropriately. */
    (void)prog;

    /* Build control list: array of (uniform_addr, shader_addr) pairs */
    u32 control_handle = gpu_mem_alloc(num_qpus * 8, 16, GPU_MEM_FLAG_COHERENT);
    if (!control_handle) return false;

    u32 control_bus = gpu_mem_lock(control_handle);
    u32 *ctrl = (u32 *)(usize)(control_bus & 0x3FFFFFFF);

    for (u32 i = 0; i < num_qpus; i++) {
        ctrl[i * 2 + 0] = jobs[i].uniform_addr;
        ctrl[i * 2 + 1] = jobs[i].shader_addr;
    }
    dsb();

    bool ok = qpu_execute(num_qpus, control_bus, false);

    gpu_mem_unlock(control_handle);
    gpu_mem_free(control_handle);

    return ok;
}

void tensor_init(void) {
    gpu_init();
    if (qpu_enable(true)) {
        uart_puts("[tensor] QPU enabled (12 QPUs)\n");
        /* In future: set use_qpu_fallback = false when shaders ready */
    } else {
        uart_puts("[tensor] QPU enable failed, using NEON fallback\n");
    }
    uart_puts("[tensor] NEON float: add/mul/scale/dot/matmul/relu/softmax\n");
}
