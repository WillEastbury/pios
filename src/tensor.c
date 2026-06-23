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
#include "v3d.h"
#include "simd.h"
#include "mmu.h"
#include "uart.h"
#include "platform.h"

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
 * VideoCore VII QPU kernels are provided via externally bound
 * shader/uniform blobs (`v3d_kernel_bind_blob`), with NEON as
 * deterministic fallback when no validated kernel is bound.
 */

/* Kernel dispatch is enabled only when V3D kernel descriptors are bound.
 * Until then, operations run on ARM/NEON. */
static bool use_qpu_fallback = true;
static bool v3d_kernel_disabled[V3D_KERNEL_MAX];
static bool v3d_direct_grid_disabled[V3D_KERNEL_MAX];
static bool v3d_kernel_warned[V3D_KERNEL_MAX];
static i32 tiny_last_stage;
static i32 tiny_last_status;
static u32 tiny_last_output_bits;
static u32 tiny_last_expected_bits;
static float tiny_a_buf ALIGNED(64);
static float tiny_b_buf ALIGNED(64);
static float tiny_c_buf ALIGNED(64);
static float tiny_r_buf ALIGNED(64);

#define TENSOR_NATIVE_HANDLE_FLAG 0x80000000U
#define TENSOR_NATIVE_POOL_SLOTS  16U
#define TENSOR_NATIVE_POOL_BYTES  4096U
static u8 tensor_native_pool[TENSOR_NATIVE_POOL_SLOTS][TENSOR_NATIVE_POOL_BYTES] ALIGNED(4096);
static bool tensor_native_pool_used[TENSOR_NATIVE_POOL_SLOTS];

static bool tensor_alloc_native_pool(tensor_t *t, u32 alloc_size)
{
    if (alloc_size > TENSOR_NATIVE_POOL_BYTES)
        return false;
    for (u32 i = 0; i < TENSOR_NATIVE_POOL_SLOTS; i++) {
        if (tensor_native_pool_used[i])
            continue;
        tensor_native_pool_used[i] = true;
        simd_zero(tensor_native_pool[i], TENSOR_NATIVE_POOL_BYTES);
        dcache_clean_range((u64)(usize)tensor_native_pool[i], TENSOR_NATIVE_POOL_BYTES);
        t->handle = TENSOR_NATIVE_HANDLE_FLAG | i;
        t->bus_addr = (u32)(usize)tensor_native_pool[i];
        t->arm_ptr = tensor_native_pool[i];
        return true;
    }
    return false;
}

#define TINY_PROOF_A    (&tiny_a_buf)
#define TINY_PROOF_B    (&tiny_b_buf)
#define TINY_PROOF_C    (&tiny_c_buf)

#define TENSOR_V3D_VEC_MIN_ELEMS   128U
#define TENSOR_V3D_VEC_MAX_ELEMS   1024U
#define TENSOR_V3D_DOT_MIN_ELEMS   256U
#define TENSOR_V3D_MATMUL_MIN_MACS 4096U
/* Cap on total synchronous GPU workgroup-dispatches per fused matmul call so a
 * single matmul cannot starve the core-0 network loop / trip the watchdog or
 * wedge the V3D CSD queue. 64 keeps matvec (1x64) and small matmul (4x16) on the
 * GPU while routing larger matmuls to fast, non-blocking NEON. */
#define TENSOR_V3D_MATMUL_MAX_DISPATCH 64ULL
#define TENSOR_V3D_MATMUL_DOT_MAX_MACS 4096U
#define TENSOR_V3D_SOFTMAX_MIN_ELEMS 256U

static inline u32 f32_bits(float f)
{
    union { float f; u32 u; } v;
    v.f = f;
    return v.u;
}

static inline float bits_f32(u32 u)
{
    union { float f; u32 u; } v;
    v.u = u;
    return v.f;
}

static bool tensor_v3d_work_eligible(v3d_kernel_id_t id, u64 work_hint)
{
    switch (id) {
    case V3D_KERNEL_ADD:
    case V3D_KERNEL_RELU:
#if PIOS_ENABLE_TINY_QPU_KERNELS
        if (work_hint > 0U && work_hint <= TENSOR_V3D_VEC_MAX_ELEMS)
            return true;
#endif
        return false;
    case V3D_KERNEL_MUL:
#if PIOS_ENABLE_TINY_QPU_KERNELS
        if (work_hint > 0U && work_hint <= TENSOR_V3D_VEC_MAX_ELEMS)
            return true;
#endif
        return false;
    case V3D_KERNEL_SCALE:
        return false;
    case V3D_KERNEL_DOT:
        /*
         * DOT returns a scalar to ARM-visible memory; keep this on NEON
         * until V3D scalar writeback/result-plumbing is implemented.
         */
        return false;
    case V3D_KERNEL_MATMUL:
        return work_hint >= TENSOR_V3D_MATMUL_MIN_MACS;
    case V3D_KERNEL_SOFTMAX:
        return work_hint >= TENSOR_V3D_SOFTMAX_MIN_ELEMS;
    default:
        return false;
    }
}

static bool tensor_try_v3d(v3d_kernel_id_t id, u64 work_hint)
{
    if (!v3d_dispatch_supported())
        return false;
    if (id >= V3D_KERNEL_MAX || v3d_kernel_disabled[id])
        return false;
    if (!tensor_v3d_work_eligible(id, work_hint))
        return false;
    const struct v3d_kernel_desc *k = v3d_kernel_desc_get(id);
    if (!k || !k->ready || (!k->native_csd && k->control_list_bus == 0) || k->qpu_count == 0)
        return false;
    v3d_status_t r = v3d_dispatch_kernel(id, 25);
    if (r == V3D_STATUS_OK)
        return true;

    /*
     * Quarantine kernels that fail dispatch so we preserve deterministic
     * NEON fallback behavior and avoid timeout-heavy retry loops.
     */
    v3d_kernel_disabled[id] = true;
    if (!v3d_kernel_warned[id]) {
        uart_puts("[ten] dis V3D: ");
        uart_puts(k->name);
        uart_puts(" st=");
        uart_hex((u32)r);
        uart_puts("\n");
        v3d_kernel_warned[id] = true;
    }
    return false;
}

static bool tensor_any_bound_v3d_kernel(void)
{
    for (u32 i = 0; i < V3D_KERNEL_MAX; i++) {
        const struct v3d_kernel_desc *k = v3d_kernel_desc_get((v3d_kernel_id_t)i);
        if (k && k->ready && !v3d_kernel_disabled[i] &&
            (k->native_csd || k->control_list_bus != 0) && k->qpu_count != 0)
            return true;
    }

    return false;
}

static bool tensor_verify_mul_sample(const float *c, const float *a, const float *b, u32 n)
{
    for (u32 i = 0; i < n; i++) {
        float expected = a[i] * b[i];
        float diff = c[i] - expected;
        if (diff < 0.0f)
            diff = -diff;
        if (diff > 0.0001f)
            return false;
    }
    return true;
}

static bool tensor_verify_relu_sample(const float *b, const float *a, u32 n)
{
    for (u32 i = 0; i < n; i++) {
        float expected = (a[i] > 0.0f) ? a[i] : 0.0f;
        float diff = b[i] - expected;
        if (diff < 0.0f)
            diff = -diff;
        if (diff > 0.0001f)
            return false;
    }
    return true;
}

void tensor_status(struct tensor_status *out)
{
    if (!out) return;
    const struct v3d_caps *c = v3d_caps_get();
    out->v3d_available = v3d_available();
    out->v3d_dispatch_supported = v3d_dispatch_supported();
    out->v3d_native_probe_ok = c ? c->native_probe_ok : false;
    out->v3d_native_selftest_ok = c ? c->native_selftest_ok : false;
    out->v3d_native_compute_enabled = c ? c->native_compute_enabled : false;
    out->v3d_native_mmu_ready = c ? c->native_mmu_ready : false;
    out->v3d_native_tiny_kernels_ready = c ? c->native_tiny_kernels_ready : false;
    out->qpu_fallback = use_qpu_fallback;
    out->any_kernel_bound = tensor_any_bound_v3d_kernel();
    out->ready_mask = 0;
    out->disabled_mask = 0;
    out->ident0 = c ? c->ident0 : 0;
    out->ident1 = c ? c->ident1 : 0;
    out->ident2 = c ? c->ident2 : 0;
    out->v3d_tech_version = c ? c->tech_version : 0;
    out->v3d_core_count = c ? c->core_count : 0;
    out->v3d_qpus_per_slice = c ? c->qpus_per_slice : 0;
    out->v3d_slice_count = c ? c->slice_count : 0;
    out->v3d_csd_status = c ? c->csd_status : 0;
    out->v3d_mmu_ctl = c ? c->mmu_ctl : 0;
    out->v3d_mmuc_control = c ? c->mmuc_control : 0;
    out->v3d_tiny_ready_mask = c ? c->tiny_ready_mask : 0;
    out->v3d_tiny_verified_mask = c ? c->tiny_verified_mask : 0;
    out->v3d_native_selftest_status = c ? (i32)c->native_selftest_status : (i32)V3D_STATUS_UNSUPPORTED;
    for (u32 i = 0; i < V3D_KERNEL_MAX; i++) {
        const struct v3d_kernel_desc *k = v3d_kernel_desc_get((v3d_kernel_id_t)i);
        if (k && k->ready)
            out->ready_mask |= (1U << i);
        if (v3d_kernel_disabled[i])
            out->disabled_mask |= (1U << i);
    }
}

static bool tensor_verify_add_sample(const float *c, const float *a,
                                     const float *b, u32 n)
{
    for (u32 i = 0; i < n; i++) {
        float expected = a[i] + b[i];
        float diff = c[i] - expected;
        if (diff < 0.0f)
            diff = -diff;
        if (diff > 0.0001f)
            return false;
    }
    return true;
}

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
    if (!t->handle)
        return tensor_alloc_native_pool(t, alloc_size);

    t->bus_addr = gpu_mem_lock(t->handle);
    if (!t->bus_addr) {
        gpu_mem_free(t->handle);
        t->handle = 0;
        return tensor_alloc_native_pool(t, alloc_size);
    }

    /* Convert bus address to ARM physical address */
    t->arm_ptr = (void *)(usize)(t->bus_addr & 0x3FFFFFFF);
    return true;
}

void tensor_free(tensor_t *t) {
    if ((t->handle & TENSOR_NATIVE_HANDLE_FLAG) != 0U) {
        u32 slot = t->handle & ~TENSOR_NATIVE_HANDLE_FLAG;
        if (slot < TENSOR_NATIVE_POOL_SLOTS)
            tensor_native_pool_used[slot] = false;
        t->handle = 0;
        t->bus_addr = 0;
        t->arm_ptr = NULL;
        return;
    }
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
            : "r"(b), "r"(f32_bits(s))
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
        u32 result_bits = 0;
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
            : "=r"(result_bits)
            :: "v4"
        );
        result = bits_f32(result_bits);
    }
    for (; i < n; i++)
        result += a[i] * b[i];
    return result;
}

static float neon_sum_f32(const float *a, u32 n)
{
    float result = 0.0f;
    u32 i = 0;

    if (n >= 4) {
        u32 result_bits = 0;
        __asm__ volatile("movi v4.4s, #0" ::: "v4");
        for (; i + 4 <= n; i += 4) {
            __asm__ volatile(
                "ld1  {v0.4s}, [%0], #16  \n"
                "fadd v4.4s, v4.4s, v0.4s \n"
                : "+r"(a)
                :: "v0","v4","memory"
            );
        }
        __asm__ volatile(
            "faddp v4.4s, v4.4s, v4.4s \n"
            "faddp s4, v4.2s           \n"
            "fmov  %w0, s4             \n"
            : "=r"(result_bits)
            :: "v4"
        );
        result = bits_f32(result_bits);
    }

    for (; i < n; i++)
        result += a[i];
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
                u32 sum_bits = 0;
                __asm__ volatile(
                    "faddp v4.4s, v4.4s, v4.4s \n"
                    "faddp s4, v4.2s           \n"
                    "fmov  %w0, s4             \n"
                    : "=r"(sum_bits) :: "v4"
                );
                sum = bits_f32(sum_bits);
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

static bool f32_close(float a, float b)
{
    float d = a - b;
    if (d < 0.0f) d = -d;
    return d < 0.001f;
}

bool tensor_selftest(void)
{
    float a[16];
    float b[16];
    float c[16];
    float r[16];
    for (u32 i = 0; i < 16; i++) {
        a[i] = (float)i - 4.0f;
        b[i] = (float)(i + 1) * 0.5f;
        c[i] = 0.0f;
        r[i] = 0.0f;
    }

    neon_vec_add_f32(c, a, b, 16);
    for (u32 i = 0; i < 16; i++)
        if (!f32_close(c[i], a[i] + b[i])) return false;

    neon_vec_mul_f32(c, a, b, 16);
    for (u32 i = 0; i < 16; i++)
        if (!f32_close(c[i], a[i] * b[i])) return false;

    neon_vec_scale_f32(c, a, 2.0f, 16);
    for (u32 i = 0; i < 16; i++)
        if (!f32_close(c[i], a[i] * 2.0f)) return false;

    neon_vec_relu_f32(c, a, 16);
    for (u32 i = 0; i < 16; i++)
        if (!f32_close(c[i], a[i] > 0.0f ? a[i] : 0.0f)) return false;

    float dot = neon_vec_dot_f32(a, b, 16);
    float expect = 0.0f;
    for (u32 i = 0; i < 16; i++) expect += a[i] * b[i];
    if (!f32_close(dot, expect)) return false;

    float ma[4] = { 1, 2, 3, 4 };
    float mb[4] = { 5, 6, 7, 8 };
    float mc[4] = { 0, 0, 0, 0 };
    neon_matmul_f32(mc, ma, mb, 2, 2, 2);
    if (!f32_close(mc[0], 19.0f) || !f32_close(mc[1], 22.0f) ||
        !f32_close(mc[2], 43.0f) || !f32_close(mc[3], 50.0f))
        return false;

    neon_vec_scale_f32(r, b, 1.0f, 16);
    return true;
}

bool tensor_tiny_selftest(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    tiny_last_stage = 1;
    tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
    return false;
#else
    float av = 1.25f;
    float bv = 2.50f;
    float neg = -3.0f;
    float out = 0.0f;
    bool ok = false;
    v3d_status_t st;

    tiny_last_stage = 0;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = 0;

    if (!v3d_dispatch_supported()) {
        tiny_last_stage = 2;
        tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
        return false;
    }

    tiny_a_buf = av;
    tiny_b_buf = bv;
    tiny_c_buf = 0.0f;
    tiny_r_buf = 0.0f;
    dcache_clean_range((u64)(usize)&tiny_a_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_b_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_c_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_r_buf, 64U);
    {
        u32 uniforms[4] = {
            0x1AU,
            (u32)(usize)&tiny_a_buf,
            (u32)(usize)&tiny_b_buf,
            (u32)(usize)&tiny_c_buf
        };
        tiny_last_stage = 10;
        st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_ADD, uniforms, sizeof(uniforms));
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK)
            goto out;
        tiny_last_stage = 11;
        st = v3d_dispatch_kernel(V3D_KERNEL_ADD, 25);
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK) {
            v3d_kernel_disabled[V3D_KERNEL_ADD] = true;
            goto out;
        }
    }
    tiny_last_stage = 12;
    v3d_kernel_mark_verified(V3D_KERNEL_ADD, true);
    dcache_invalidate_range((u64)(usize)&tiny_c_buf, 64U);
    out = tiny_c_buf;
    tiny_last_output_bits = f32_bits(out);
    tiny_last_expected_bits = f32_bits(av + bv);
    if (!f32_close(out, av + bv)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        v3d_kernel_mark_verified(V3D_KERNEL_ADD, false);
        v3d_kernel_disabled[V3D_KERNEL_ADD] = true;
        goto out;
    }

    tiny_c_buf = 0.0f;
    dcache_clean_range((u64)(usize)&tiny_c_buf, 64U);
    {
        u32 uniforms[4] = {
            0x1AU,
            (u32)(usize)&tiny_a_buf,
            (u32)(usize)&tiny_b_buf,
            (u32)(usize)&tiny_c_buf
        };
        tiny_last_stage = 15;
        st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_MUL, uniforms, sizeof(uniforms));
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK)
            goto out;
        tiny_last_stage = 16;
        st = v3d_dispatch_kernel(V3D_KERNEL_MUL, 25);
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK) {
            v3d_kernel_disabled[V3D_KERNEL_MUL] = true;
            goto out;
        }
    }
    tiny_last_stage = 17;
    v3d_kernel_mark_verified(V3D_KERNEL_MUL, true);
    dcache_invalidate_range((u64)(usize)&tiny_c_buf, 64U);
    out = tiny_c_buf;
    tiny_last_output_bits = f32_bits(out);
    tiny_last_expected_bits = f32_bits(av * bv);
    if (!f32_close(out, av * bv)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        v3d_kernel_mark_verified(V3D_KERNEL_MUL, false);
        v3d_kernel_disabled[V3D_KERNEL_MUL] = true;
        goto out;
    }

    tiny_a_buf = neg;
    tiny_r_buf = 0.0f;
    dcache_clean_range((u64)(usize)&tiny_a_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_r_buf, 64U);
    {
        u32 uniforms[3] = {
            0x1AU,
            (u32)(usize)&tiny_a_buf,
            (u32)(usize)&tiny_r_buf
        };
        tiny_last_stage = 20;
        st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_RELU, uniforms, sizeof(uniforms));
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK)
            goto out;
        tiny_last_stage = 21;
        st = v3d_dispatch_kernel(V3D_KERNEL_RELU, 25);
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK) {
            v3d_kernel_disabled[V3D_KERNEL_RELU] = true;
            goto out;
        }
    }
    tiny_last_stage = 22;
    v3d_kernel_mark_verified(V3D_KERNEL_RELU, true);
    dcache_invalidate_range((u64)(usize)&tiny_r_buf, 64U);
    out = tiny_r_buf;
    tiny_last_output_bits = f32_bits(out);
    tiny_last_expected_bits = f32_bits(0.0f);
    if (!f32_close(out, 0.0f)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        v3d_kernel_mark_verified(V3D_KERNEL_RELU, false);
        v3d_kernel_disabled[V3D_KERNEL_RELU] = true;
        goto out;
    }

    ok = true;
    tiny_last_stage = 99;
    tiny_last_status = 0;

out:
    return ok;
#endif
}

bool tensor_tiny_noop_proof(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    tiny_last_stage = 21;
    tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
    return false;
#else
    v3d_status_t st;
    tiny_last_stage = 20;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = 0;

    if (!v3d_dispatch_supported()) {
        tiny_last_stage = 21;
        tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
        return false;
    }

    tiny_last_stage = 22;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_NOOP, NULL, 0);
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 23;
    st = v3d_dispatch_kernel(V3D_KERNEL_NOOP, 25);
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 29;
    tiny_last_status = 0;
    return true;
#endif
}

bool tensor_tiny_store_proof(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    tiny_last_stage = 31;
    tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
    return false;
#else
    const float expect = 3.75f;
    float out;
    v3d_status_t st;
    u32 uniforms[4];
    void *dst_ptr;
    u32 dst_addr;

    tiny_last_stage = 30;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = f32_bits(expect);

    if (!v3d_dispatch_supported()) {
        tiny_last_stage = 31;
        tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
        return false;
    }

    dst_ptr = TINY_PROOF_C;
    dst_addr = (u32)(usize)TINY_PROOF_C;
    simd_zero(dst_ptr, 64U);
    dcache_clean_range((u64)(usize)dst_ptr, 64U);
    uniforms[0] = 0x1AU;
    uniforms[1] = f32_bits(expect);
    uniforms[2] = dst_addr;

    tiny_last_stage = 32;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_STORE_SSBO, uniforms, sizeof(uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 33;
    st = v3d_dispatch_kernel(V3D_KERNEL_STORE_SSBO, 25);
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 34;
    dcache_invalidate_range((u64)(usize)dst_ptr, 64U);
    out = *(float *)dst_ptr;
    tiny_last_output_bits = f32_bits(out);
    if (!f32_close(out, expect)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        return false;
    }

    tiny_last_stage = 39;
    tiny_last_status = 0;
    return true;
#endif
}

bool tensor_tiny_load_store_proof(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    tiny_last_stage = 41;
    tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
    return false;
#else
    const float expect = 1.25f;
    float out;
    v3d_status_t st;
    u32 uniforms[2];
    void *src_ptr;
    void *dst_ptr;
    u32 src_addr;
    u32 dst_addr;

    tiny_last_stage = 40;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = f32_bits(expect);

    if (!v3d_dispatch_supported()) {
        tiny_last_stage = 41;
        tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
        return false;
    }

    src_ptr = TINY_PROOF_A;
    dst_ptr = TINY_PROOF_C;
    src_addr = (u32)(usize)TINY_PROOF_A;
    dst_addr = (u32)(usize)TINY_PROOF_C;
    *(float *)src_ptr = expect;
    *(float *)dst_ptr = 0.0f;
    dcache_clean_range((u64)(usize)src_ptr, 64U);
    dcache_clean_range((u64)(usize)dst_ptr, 64U);
    uniforms[0] = src_addr;
    uniforms[1] = dst_addr;

    tiny_last_stage = 42;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_LOAD_STORE, uniforms, sizeof(uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 43;
    st = v3d_dispatch_kernel(V3D_KERNEL_LOAD_STORE, 25);
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 44;
    dcache_invalidate_range((u64)(usize)dst_ptr, 64U);
    out = *(float *)dst_ptr;
    tiny_last_output_bits = f32_bits(out);
    if (!f32_close(out, expect)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        return false;
    }

    tiny_last_stage = 49;
    tiny_last_status = 0;
    return true;
#endif
}

bool tensor_tiny_memory_proof(void)
{
    if (!tensor_tiny_store_proof())
        return false;
    if (!tensor_tiny_load_store_proof())
        return false;

#if !PIOS_ENABLE_TINY_QPU_KERNELS
    return false;
#else
    const float av = 1.25f;
    const float bv = 2.50f;
    const float expect = av + bv;
    float out;
    v3d_status_t st;
    u32 uniforms[4];

    tiny_a_buf = av;
    tiny_b_buf = bv;
    tiny_c_buf = 0.0f;
    dcache_clean_range((u64)(usize)&tiny_a_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_b_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_c_buf, 64U);
    uniforms[0] = 0x1AU;
    uniforms[1] = (u32)(usize)&tiny_a_buf;
    uniforms[2] = (u32)(usize)&tiny_b_buf;
    uniforms[3] = (u32)(usize)&tiny_c_buf;

    tiny_last_stage = 52;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = f32_bits(expect);
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_ADD, uniforms, sizeof(uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 53;
    st = v3d_dispatch_kernel(V3D_KERNEL_ADD, 25);
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 54;
    dcache_invalidate_range((u64)(usize)&tiny_c_buf, 64U);
    out = tiny_c_buf;
    tiny_last_output_bits = f32_bits(out);
    if (!f32_close(out, expect)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        return false;
    }

    tiny_last_stage = 59;
    tiny_last_status = 0;
    return true;
#endif
}

i32 tensor_tiny_last_stage(void)
{
    return tiny_last_stage;
}

i32 tensor_tiny_last_status(void)
{
    return tiny_last_status;
}

u32 tensor_tiny_last_output_bits(void)
{
    return tiny_last_output_bits;
}

u32 tensor_tiny_last_expected_bits(void)
{
    return tiny_last_expected_bits;
}

/* ---- Public tensor operations ---- */

static bool tensor_v3d_binary_chunks(v3d_kernel_id_t id, tensor_t *c,
                                     const tensor_t *a, const tensor_t *b,
                                     u32 n)
{
    if (n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS || (n & 15U) != 0U)
        return false;

    for (u32 off = 0; off < n; off += 16U) {
        u32 byte_off = off * sizeof(float);
        u32 uniforms[4] = {
            0x1AU,
            (a->bus_addr + byte_off) & 0x3FFFFFFFU,
            (b->bus_addr + byte_off) & 0x3FFFFFFFU,
            (c->bus_addr + byte_off) & 0x3FFFFFFFU
        };
        if (v3d_kernel_bind_builtin_qpu_grid(id, uniforms, sizeof(uniforms), 1U) != V3D_STATUS_OK)
            return false;
        if (!tensor_try_v3d(id, 16U))
            return false;
    }
    return true;
}

static bool tensor_v3d_binary_direct(v3d_kernel_id_t id, tensor_t *c,
                                     const tensor_t *a, const tensor_t *b,
                                     u32 n)
{
    if (n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS || id >= V3D_KERNEL_MAX ||
        v3d_direct_grid_disabled[id])
        return false;

    u32 uniforms[7] = {
        0x0000FFFFU,
        0x1AU,
        n,
        0x0000000CU,
        a->bus_addr & 0x3FFFFFFFU,
        b->bus_addr & 0x3FFFFFFFU,
        c->bus_addr & 0x3FFFFFFFU
    };
    u32 workgroups = (n + 15U) / 16U;
    if (v3d_kernel_bind_builtin_qpu_grid(id, uniforms, sizeof(uniforms), workgroups) != V3D_STATUS_OK)
        return false;

    v3d_status_t st = v3d_dispatch_kernel(id, 25);
    if (st == V3D_STATUS_OK)
        return true;

    v3d_direct_grid_disabled[id] = true;
    return false;
}

static bool tensor_v3d_relu_chunks(tensor_t *b, const tensor_t *a, u32 n)
{
    if (n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS || (n & 15U) != 0U)
        return false;

    for (u32 off = 0; off < n; off += 16U) {
        u32 byte_off = off * sizeof(float);
        u32 uniforms[3] = {
            0x1AU,
            (a->bus_addr + byte_off) & 0x3FFFFFFFU,
            (b->bus_addr + byte_off) & 0x3FFFFFFFU
        };
        if (v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_RELU, uniforms, sizeof(uniforms), 1U) != V3D_STATUS_OK)
            return false;
        if (!tensor_try_v3d(V3D_KERNEL_RELU, 16U))
            return false;
    }
    return true;
}

static bool tensor_v3d_relu_direct(tensor_t *b, const tensor_t *a, u32 n)
{
    if (n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS ||
        v3d_direct_grid_disabled[V3D_KERNEL_RELU])
        return false;

    u32 uniforms[6] = {
        0x0000FFFFU,
        0x1AU,
        n,
        0x00000008U,
        a->bus_addr & 0x3FFFFFFFU,
        b->bus_addr & 0x3FFFFFFFU
    };
    u32 workgroups = (n + 15U) / 16U;
    if (v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_RELU, uniforms, sizeof(uniforms), workgroups) != V3D_STATUS_OK)
        return false;

    v3d_status_t st = v3d_dispatch_kernel(V3D_KERNEL_RELU, 25);
    if (st == V3D_STATUS_OK)
        return true;

    v3d_direct_grid_disabled[V3D_KERNEL_RELU] = true;
    return false;
}

static bool tensor_verify_matmul_sample(const float *c, const float *a,
                                        const float *b, u32 m, u32 k, u32 p)
    __attribute__((unused));
static bool tensor_verify_matmul_sample(const float *c, const float *a,
                                        const float *b, u32 m, u32 k, u32 p)
{
    u32 total = m * p;
    u32 sample = (total < 16U) ? total : 16U;
    for (u32 s = 0; s < sample; s++) {
        u32 i = s / p;
        u32 j = s % p;
        float expected = 0.0f;
        for (u32 kk = 0; kk < k; kk++)
            expected += a[i * k + kk] * b[kk * p + j];
        float diff = c[i * p + j] - expected;
        if (diff < 0.0f)
            diff = -diff;
        if (diff > 0.001f)
            return false;
    }
    return true;
}


bool tensor_add(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n || c->rows * c->cols != n) return false;

    bool v3d_bound = false;
#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        v3d_status_t st;
        dcache_clean_range((u64)(usize)a->arm_ptr, a->total_bytes);
        dcache_clean_range((u64)(usize)b->arm_ptr, b->total_bytes);
        dcache_clean_range((u64)(usize)c->arm_ptr, c->total_bytes);
        if (n <= 16U) {
            u32 uniforms[4] = {
                0x1AU,
                a->bus_addr & 0x3FFFFFFFU,
                b->bus_addr & 0x3FFFFFFFU,
                c->bus_addr & 0x3FFFFFFFU
            };
            st = v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_ADD, uniforms,
                                                  sizeof(uniforms), 1U);
            v3d_bound = (st == V3D_STATUS_OK);
        } else {
            if (tensor_v3d_binary_direct(V3D_KERNEL_ADD, c, a, b, n)) {
                dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
                if (tensor_verify_add_sample((const float *)c->arm_ptr,
                                             (const float *)a->arm_ptr,
                                             (const float *)b->arm_ptr, n)) {
                    v3d_kernel_mark_verified(V3D_KERNEL_ADD, true);
                    return true;
                }
                v3d_kernel_mark_verified(V3D_KERNEL_ADD, false);
                v3d_direct_grid_disabled[V3D_KERNEL_ADD] = true;
            }
            if (tensor_v3d_binary_chunks(V3D_KERNEL_ADD, c, a, b, n)) {
                dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
                if (tensor_verify_add_sample((const float *)c->arm_ptr,
                                             (const float *)a->arm_ptr,
                                             (const float *)b->arm_ptr, n)) {
                    v3d_kernel_mark_verified(V3D_KERNEL_ADD, true);
                    return true;
                }
                v3d_kernel_mark_verified(V3D_KERNEL_ADD, false);
                v3d_kernel_disabled[V3D_KERNEL_ADD] = true;
            }
        }
    }
#endif
    if (!use_qpu_fallback && v3d_bound && tensor_try_v3d(V3D_KERNEL_ADD, n)) {
        dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
        if (tensor_verify_add_sample((const float *)c->arm_ptr,
                                     (const float *)a->arm_ptr,
                                     (const float *)b->arm_ptr, n)) {
            v3d_kernel_mark_verified(V3D_KERNEL_ADD, true);
            return true;
        }

        v3d_kernel_mark_verified(V3D_KERNEL_ADD, false);
        v3d_kernel_disabled[V3D_KERNEL_ADD] = true;
        if (!v3d_kernel_warned[V3D_KERNEL_ADD]) {
            uart_puts("[ten] dis V3D add: verify fail\n");
            v3d_kernel_warned[V3D_KERNEL_ADD] = true;
        }
    }
    neon_vec_add_f32((float *)c->arm_ptr,
                     (const float *)a->arm_ptr,
                     (const float *)b->arm_ptr, n);
    dsb();
    return true;
}

bool tensor_mul(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n || c->rows * c->cols != n) return false;

    bool v3d_bound = false;
#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        v3d_status_t st;
        dcache_clean_range((u64)(usize)a->arm_ptr, a->total_bytes);
        dcache_clean_range((u64)(usize)b->arm_ptr, b->total_bytes);
        dcache_clean_range((u64)(usize)c->arm_ptr, c->total_bytes);
        if (n <= 16U) {
            u32 uniforms[4] = {
                0x1AU,
                a->bus_addr & 0x3FFFFFFFU,
                b->bus_addr & 0x3FFFFFFFU,
                c->bus_addr & 0x3FFFFFFFU
            };
            st = v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_MUL, uniforms,
                                                  sizeof(uniforms), 1U);
            v3d_bound = (st == V3D_STATUS_OK);
        } else {
            if (tensor_v3d_binary_direct(V3D_KERNEL_MUL, c, a, b, n)) {
                dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
                if (tensor_verify_mul_sample((const float *)c->arm_ptr,
                                             (const float *)a->arm_ptr,
                                             (const float *)b->arm_ptr, n)) {
                    v3d_kernel_mark_verified(V3D_KERNEL_MUL, true);
                    return true;
                }
                v3d_kernel_mark_verified(V3D_KERNEL_MUL, false);
                v3d_direct_grid_disabled[V3D_KERNEL_MUL] = true;
            }
            if (tensor_v3d_binary_chunks(V3D_KERNEL_MUL, c, a, b, n)) {
                dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
                if (tensor_verify_mul_sample((const float *)c->arm_ptr,
                                             (const float *)a->arm_ptr,
                                             (const float *)b->arm_ptr, n)) {
                    v3d_kernel_mark_verified(V3D_KERNEL_MUL, true);
                    return true;
                }
                v3d_kernel_mark_verified(V3D_KERNEL_MUL, false);
                v3d_kernel_disabled[V3D_KERNEL_MUL] = true;
            }
        }
    }
#endif
    if (!use_qpu_fallback && v3d_bound && tensor_try_v3d(V3D_KERNEL_MUL, n)) {
        dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
        if (tensor_verify_mul_sample((const float *)c->arm_ptr,
                                     (const float *)a->arm_ptr,
                                     (const float *)b->arm_ptr, n)) {
            v3d_kernel_mark_verified(V3D_KERNEL_MUL, true);
            return true;
        }
        v3d_kernel_mark_verified(V3D_KERNEL_MUL, false);
        v3d_kernel_disabled[V3D_KERNEL_MUL] = true;
    }
    neon_vec_mul_f32((float *)c->arm_ptr,
                     (const float *)a->arm_ptr,
                     (const float *)b->arm_ptr, n);
    dsb();
    return true;
}

bool tensor_scale(tensor_t *b, const tensor_t *a, float scalar) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n) return false;

    if (!use_qpu_fallback && tensor_try_v3d(V3D_KERNEL_SCALE, n))
        return true;
    neon_vec_scale_f32((float *)b->arm_ptr,
                       (const float *)a->arm_ptr, scalar, n);
    dsb();
    return true;
}

bool tensor_dot(float *result, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n) return false;

#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        tensor_t tmp = {0};
        if (tensor_alloc(&tmp, 1, n, sizeof(float))) {
            if (tensor_mul(&tmp, a, b)) {
                dcache_invalidate_range((u64)(usize)tmp.arm_ptr, tmp.total_bytes);
                const float *p = (const float *)tmp.arm_ptr;
                float sum = 0.0f;
                for (u32 i = 0; i < n; i++)
                    sum += p[i];
                *result = sum;
                tensor_free(&tmp);
                return true;
            }
            tensor_free(&tmp);
        }
    }
#endif

    *result = neon_vec_dot_f32((const float *)a->arm_ptr,
                               (const float *)b->arm_ptr, n);
    return true;
}

bool tensor_matmul(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    if (a->cols != b->rows) return false;
    if (c->rows != a->rows || c->cols != b->cols) return false;

    u64 macs = (u64)a->rows * (u64)a->cols * (u64)b->cols;
    if (!use_qpu_fallback && tensor_try_v3d(V3D_KERNEL_MATMUL, macs))
        return true;

#if PIOS_ENABLE_TINY_QPU_KERNELS
    /*
     * Fused GPU matmul: each output column j is one matrix-vector product. We
     * broadcast B[:,j] across all rows, run ONE loop-free vector_mulN over the
     * whole m*K matrix, then reduce each row on ARM. p columns => p GPU
     * dispatches (vs m*p for the per-element dot path). Scratch buffers are
     * shared across columns. Requires m*K <= 1024 (vector_mulN element cap).
     *
     * Total synchronous GPU work is bounded: this runs in the caller's context
     * (e.g. the core-0 HTTP handler), so an unbounded number of CSD dispatches
     * would starve the network loop and trip the watchdog. We cap the total
     * workgroup-dispatches (p * ceil(m*K/16)); larger matmuls use NEON, which is
     * fast and non-blocking on the A76.
     */
    if (!use_qpu_fallback && a->rows > 0U && a->cols > 0U && b->cols > 0U &&
        (a->rows * a->cols) <= TENSOR_V3D_VEC_MAX_ELEMS &&
        (u64)b->cols * (((u64)a->rows * a->cols + 15U) / 16U) <= TENSOR_V3D_MATMUL_MAX_DISPATCH) {
        u32 m = a->rows;
        u32 k = a->cols;
        u32 p = b->cols;
        tensor_t xb = {0};
        tensor_t prod = {0};
        if (tensor_alloc(&xb, m, k, sizeof(float)) &&
            tensor_alloc(&prod, m, k, sizeof(float))) {
            const float *bp = (const float *)b->arm_ptr;
            float *xbp = (float *)xb.arm_ptr;
            float *cp = (float *)c->arm_ptr;
            bool ok = true;
            for (u32 j = 0; j < p && ok; j++) {
                for (u32 i = 0; i < m; i++)
                    for (u32 kk = 0; kk < k; kk++)
                        xbp[i * k + kk] = bp[kk * p + j];
                dcache_clean_range((u64)(usize)xb.arm_ptr, xb.total_bytes);
                if (!tensor_mul(&prod, a, &xb)) {
                    ok = false;
                    break;
                }
                dcache_invalidate_range((u64)(usize)prod.arm_ptr, prod.total_bytes);
                const float *pp = (const float *)prod.arm_ptr;
                for (u32 i = 0; i < m; i++) {
                    float s = 0.0f;
                    for (u32 kk = 0; kk < k; kk++)
                        s += pp[i * k + kk];
                    cp[i * c->cols + j] = s;
                }
            }
            tensor_free(&xb);
            tensor_free(&prod);
            if (ok) {
                dcache_clean_range((u64)(usize)c->arm_ptr, c->total_bytes);
                if (tensor_verify_matmul_sample((const float *)c->arm_ptr,
                                                (const float *)a->arm_ptr,
                                                (const float *)b->arm_ptr, m, k, p))
                    return true;
                /* Verification failed: fall through to the dot path / NEON. */
            }
        } else {
            tensor_free(&xb);
            tensor_free(&prod);
        }
    }

    if (!use_qpu_fallback && a->cols > 0U &&
        a->cols <= TENSOR_V3D_VEC_MAX_ELEMS &&
        macs <= TENSOR_V3D_MATMUL_DOT_MAX_MACS) {
        tensor_t row = {0};
        tensor_t col = {0};
        if (tensor_alloc(&row, 1, a->cols, sizeof(float)) &&
            tensor_alloc(&col, 1, a->cols, sizeof(float))) {
            const float *ap = (const float *)a->arm_ptr;
            const float *bp = (const float *)b->arm_ptr;
            float *cp = (float *)c->arm_ptr;
            float *rp = (float *)row.arm_ptr;
            float *kp = (float *)col.arm_ptr;
            bool ok = true;
            for (u32 i = 0; i < a->rows && ok; i++) {
                for (u32 j = 0; j < b->cols && ok; j++) {
                    for (u32 k = 0; k < a->cols; k++) {
                        rp[k] = ap[i * a->cols + k];
                        kp[k] = bp[k * b->cols + j];
                    }
                    dcache_clean_range((u64)(usize)row.arm_ptr, row.total_bytes);
                    dcache_clean_range((u64)(usize)col.arm_ptr, col.total_bytes);
                    float v = 0.0f;
                    ok = tensor_dot(&v, &row, &col);
                    cp[i * c->cols + j] = v;
                }
            }
            dcache_clean_range((u64)(usize)c->arm_ptr, c->total_bytes);
            tensor_free(&row);
            tensor_free(&col);
            if (ok)
                return true;
        } else {
            tensor_free(&row);
            tensor_free(&col);
        }
    }
#endif

    neon_matmul_f32((float *)c->arm_ptr,
                    (const float *)a->arm_ptr,
                    (const float *)b->arm_ptr,
                    a->rows, a->cols, b->cols);
    dsb();
    return true;
}

bool tensor_relu(tensor_t *b, const tensor_t *a) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n) return false;

    bool v3d_bound = false;
#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        v3d_status_t st;
        dcache_clean_range((u64)(usize)a->arm_ptr, a->total_bytes);
        dcache_clean_range((u64)(usize)b->arm_ptr, b->total_bytes);
        if (n <= 16U) {
            u32 uniforms[3] = {
                0x1AU,
                a->bus_addr & 0x3FFFFFFFU,
                b->bus_addr & 0x3FFFFFFFU
            };
            st = v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_RELU, uniforms,
                                                  sizeof(uniforms), 1U);
            v3d_bound = (st == V3D_STATUS_OK);
        } else {
            if (tensor_v3d_relu_direct(b, a, n)) {
                dcache_invalidate_range((u64)(usize)b->arm_ptr, b->total_bytes);
                if (tensor_verify_relu_sample((const float *)b->arm_ptr,
                                             (const float *)a->arm_ptr, n)) {
                    v3d_kernel_mark_verified(V3D_KERNEL_RELU, true);
                    return true;
                }
                v3d_kernel_mark_verified(V3D_KERNEL_RELU, false);
                v3d_direct_grid_disabled[V3D_KERNEL_RELU] = true;
            }
            if (tensor_v3d_relu_chunks(b, a, n)) {
                dcache_invalidate_range((u64)(usize)b->arm_ptr, b->total_bytes);
                if (tensor_verify_relu_sample((const float *)b->arm_ptr,
                                             (const float *)a->arm_ptr, n)) {
                    v3d_kernel_mark_verified(V3D_KERNEL_RELU, true);
                    return true;
                }
                v3d_kernel_mark_verified(V3D_KERNEL_RELU, false);
                v3d_kernel_disabled[V3D_KERNEL_RELU] = true;
            }
        }
    }
#endif
    if (!use_qpu_fallback && v3d_bound && tensor_try_v3d(V3D_KERNEL_RELU, n)) {
        dcache_invalidate_range((u64)(usize)b->arm_ptr, b->total_bytes);
        if (tensor_verify_relu_sample((const float *)b->arm_ptr,
                                      (const float *)a->arm_ptr, n)) {
            v3d_kernel_mark_verified(V3D_KERNEL_RELU, true);
            return true;
        }

        v3d_kernel_mark_verified(V3D_KERNEL_RELU, false);
        v3d_kernel_disabled[V3D_KERNEL_RELU] = true;
        if (!v3d_kernel_warned[V3D_KERNEL_RELU]) {
            uart_puts("[ten] dis V3D relu: verify fail\n");
            v3d_kernel_warned[V3D_KERNEL_RELU] = true;
        }
    }
    neon_vec_relu_f32((float *)b->arm_ptr,
                      (const float *)a->arm_ptr, n);
    dsb();
    return true;
}

bool tensor_vector16_selftest(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    tiny_last_stage = 70;
    tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
    return false;
#else
    tensor_t a = {0};
    tensor_t b = {0};
    tensor_t c = {0};
    tensor_t r = {0};
    tiny_last_stage = 70;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = 0;

    if (!tensor_alloc(&a, 1, 16, sizeof(float)) ||
        !tensor_alloc(&b, 1, 16, sizeof(float)) ||
        !tensor_alloc(&c, 1, 16, sizeof(float)) ||
        !tensor_alloc(&r, 1, 16, sizeof(float))) {
        tiny_last_stage = 71;
        tiny_last_status = (i32)V3D_STATUS_NOT_READY;
        tensor_free(&a); tensor_free(&b); tensor_free(&c); tensor_free(&r);
        return false;
    }

    float *ap = (float *)a.arm_ptr;
    float *bp = (float *)b.arm_ptr;
    float *cp = (float *)c.arm_ptr;
    float *rp = (float *)r.arm_ptr;
    for (u32 i = 0; i < 16U; i++) {
        ap[i] = (float)i - 4.0f;
        bp[i] = (float)(i + 1U) * 0.5f;
        cp[i] = 0.0f;
        rp[i] = 0.0f;
    }
    dcache_clean_range((u64)(usize)a.arm_ptr, a.total_bytes);
    dcache_clean_range((u64)(usize)b.arm_ptr, b.total_bytes);
    dcache_clean_range((u64)(usize)c.arm_ptr, c.total_bytes);
    dcache_clean_range((u64)(usize)r.arm_ptr, r.total_bytes);

    tiny_last_stage = 72;
    if (!tensor_add(&c, &a, &b) ||
        !tensor_verify_add_sample((const float *)c.arm_ptr,
                                  (const float *)a.arm_ptr,
                                  (const float *)b.arm_ptr, 16U)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    for (u32 i = 0; i < 16U; i++)
        cp[i] = 0.0f;
    dcache_clean_range((u64)(usize)c.arm_ptr, c.total_bytes);
    tiny_last_stage = 73;
    if (!tensor_mul(&c, &a, &b) ||
        !tensor_verify_mul_sample((const float *)c.arm_ptr,
                                  (const float *)a.arm_ptr,
                                  (const float *)b.arm_ptr, 16U)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    tiny_last_stage = 74;
    if (!tensor_relu(&r, &a) ||
        !tensor_verify_relu_sample((const float *)r.arm_ptr,
                                   (const float *)a.arm_ptr, 16U)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    tiny_last_stage = 75;
    float dot = 0.0f;
    float expect = 0.0f;
    for (u32 i = 0; i < 16U; i++)
        expect += ap[i] * bp[i];
    if (!tensor_dot(&dot, &a, &b) || !f32_close(dot, expect)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        tiny_last_output_bits = f32_bits(dot);
        tiny_last_expected_bits = f32_bits(expect);
        goto fail;
    }

    tensor_t ma = {0};
    tensor_t mb = {0};
    tensor_t mc = {0};
    tiny_last_stage = 76;
    if (!tensor_alloc(&ma, 2, 2, sizeof(float)) ||
        !tensor_alloc(&mb, 2, 2, sizeof(float)) ||
        !tensor_alloc(&mc, 2, 2, sizeof(float))) {
        tiny_last_status = (i32)V3D_STATUS_NOT_READY;
        tensor_free(&ma); tensor_free(&mb); tensor_free(&mc);
        goto fail;
    }
    float *map = (float *)ma.arm_ptr;
    float *mbp = (float *)mb.arm_ptr;
    float *mcp = (float *)mc.arm_ptr;
    map[0] = 1.0f; map[1] = 2.0f; map[2] = 3.0f; map[3] = 4.0f;
    mbp[0] = 5.0f; mbp[1] = 6.0f; mbp[2] = 7.0f; mbp[3] = 8.0f;
    mcp[0] = 0.0f; mcp[1] = 0.0f; mcp[2] = 0.0f; mcp[3] = 0.0f;
    dcache_clean_range((u64)(usize)ma.arm_ptr, ma.total_bytes);
    dcache_clean_range((u64)(usize)mb.arm_ptr, mb.total_bytes);
    dcache_clean_range((u64)(usize)mc.arm_ptr, mc.total_bytes);
    if (!tensor_matmul(&mc, &ma, &mb) ||
        !f32_close(mcp[0], 19.0f) || !f32_close(mcp[1], 22.0f) ||
        !f32_close(mcp[2], 43.0f) || !f32_close(mcp[3], 50.0f)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        tiny_last_output_bits = f32_bits(mcp[0]);
        tiny_last_expected_bits = f32_bits(19.0f);
        tensor_free(&ma); tensor_free(&mb); tensor_free(&mc);
        goto fail;
    }
    tensor_free(&ma); tensor_free(&mb); tensor_free(&mc);

    tiny_last_stage = 79;
    tiny_last_status = 0;
    tensor_free(&a); tensor_free(&b); tensor_free(&c); tensor_free(&r);
    return true;

fail:
    tiny_last_output_bits = f32_bits(cp[0]);
    tiny_last_expected_bits = f32_bits(ap[0] + bp[0]);
    tensor_free(&a); tensor_free(&b); tensor_free(&c); tensor_free(&r);
    return false;
#endif
}

bool tensor_vectorn_selftest(u32 n)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    tiny_last_stage = 80;
    tiny_last_status = (i32)V3D_STATUS_UNSUPPORTED;
    return false;
#else
    if (n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS) {
        tiny_last_stage = 80;
        tiny_last_status = (i32)V3D_STATUS_INVALID;
        return false;
    }

    tensor_t a = {0};
    tensor_t b = {0};
    tensor_t c = {0};
    tensor_t r = {0};
    tiny_last_stage = 80;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = 0;

    if (!tensor_alloc(&a, 1, n, sizeof(float)) ||
        !tensor_alloc(&b, 1, n, sizeof(float)) ||
        !tensor_alloc(&c, 1, n, sizeof(float)) ||
        !tensor_alloc(&r, 1, n, sizeof(float))) {
        tiny_last_stage = 81;
        tiny_last_status = (i32)V3D_STATUS_NOT_READY;
        tensor_free(&a); tensor_free(&b); tensor_free(&c); tensor_free(&r);
        return false;
    }

    float *ap = (float *)a.arm_ptr;
    float *bp = (float *)b.arm_ptr;
    float *cp = (float *)c.arm_ptr;
    float *rp = (float *)r.arm_ptr;
    for (u32 i = 0; i < n; i++) {
        ap[i] = ((float)(i % 29U) - 11.0f) * 0.25f;
        bp[i] = ((float)(i % 17U) + 1.0f) * 0.125f;
        cp[i] = 0.0f;
        rp[i] = 0.0f;
    }
    dcache_clean_range((u64)(usize)a.arm_ptr, a.total_bytes);
    dcache_clean_range((u64)(usize)b.arm_ptr, b.total_bytes);
    dcache_clean_range((u64)(usize)c.arm_ptr, c.total_bytes);
    dcache_clean_range((u64)(usize)r.arm_ptr, r.total_bytes);

    tiny_last_stage = 82;
    if (!tensor_add(&c, &a, &b) ||
        !tensor_verify_add_sample((const float *)c.arm_ptr,
                                  (const float *)a.arm_ptr,
                                  (const float *)b.arm_ptr, n)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    for (u32 i = 0; i < n; i++)
        cp[i] = 0.0f;
    dcache_clean_range((u64)(usize)c.arm_ptr, c.total_bytes);
    tiny_last_stage = 83;
    if (!tensor_mul(&c, &a, &b) ||
        !tensor_verify_mul_sample((const float *)c.arm_ptr,
                                  (const float *)a.arm_ptr,
                                  (const float *)b.arm_ptr, n)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    tiny_last_stage = 84;
    if (!tensor_relu(&r, &a) ||
        !tensor_verify_relu_sample((const float *)r.arm_ptr,
                                   (const float *)a.arm_ptr, n)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    tiny_last_stage = 85;
    float dot = 0.0f;
    float expect = 0.0f;
    for (u32 i = 0; i < n; i++)
        expect += ap[i] * bp[i];
    if (!tensor_dot(&dot, &a, &b) || !f32_close(dot, expect)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        tiny_last_output_bits = f32_bits(dot);
        tiny_last_expected_bits = f32_bits(expect);
        goto fail;
    }

    tiny_last_stage = 89;
    tiny_last_status = 0;
    tensor_free(&a); tensor_free(&b); tensor_free(&c); tensor_free(&r);
    return true;

fail:
    tiny_last_output_bits = f32_bits(cp[0]);
    tiny_last_expected_bits = f32_bits(ap[0] + bp[0]);
    tensor_free(&a); tensor_free(&b); tensor_free(&c); tensor_free(&r);
    return false;
#endif
}

bool tensor_matvec128_selftest(void)
{
    tensor_t a = {0};
    tensor_t x = {0};
    tensor_t y = {0};
    const u32 rows = 8U;
    const u32 cols = 128U;
    tiny_last_stage = 90;
    tiny_last_status = 0;

    if (!tensor_alloc(&a, rows, cols, sizeof(float)) ||
        !tensor_alloc(&x, cols, 1, sizeof(float)) ||
        !tensor_alloc(&y, rows, 1, sizeof(float))) {
        tiny_last_stage = 91;
        tiny_last_status = (i32)V3D_STATUS_NOT_READY;
        tensor_free(&a); tensor_free(&x); tensor_free(&y);
        return false;
    }

    float *ap = (float *)a.arm_ptr;
    float *xp = (float *)x.arm_ptr;
    float *yp = (float *)y.arm_ptr;
    for (u32 r = 0; r < rows; r++)
        for (u32 c = 0; c < cols; c++)
            ap[r * cols + c] = ((float)((r + c) % 13U) - 6.0f) * 0.0625f;
    for (u32 c = 0; c < cols; c++)
        xp[c] = ((float)(c % 11U) - 5.0f) * 0.125f;
    for (u32 r = 0; r < rows; r++)
        yp[r] = 0.0f;

    dcache_clean_range((u64)(usize)a.arm_ptr, a.total_bytes);
    dcache_clean_range((u64)(usize)x.arm_ptr, x.total_bytes);
    dcache_clean_range((u64)(usize)y.arm_ptr, y.total_bytes);

    tiny_last_stage = 92;
    if (!tensor_matmul(&y, &a, &x)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    dcache_invalidate_range((u64)(usize)y.arm_ptr, y.total_bytes);
    tiny_last_stage = 93;
    for (u32 r = 0; r < rows; r++) {
        float expect = 0.0f;
        for (u32 c = 0; c < cols; c++)
            expect += ap[r * cols + c] * xp[c];
        if (!f32_close(yp[r], expect)) {
            tiny_last_status = (i32)V3D_STATUS_FAILED;
            tiny_last_output_bits = f32_bits(yp[r]);
            tiny_last_expected_bits = f32_bits(expect);
            goto fail;
        }
    }

    tiny_last_stage = 99;
    tiny_last_status = 0;
    tensor_free(&a); tensor_free(&x); tensor_free(&y);
    return true;

fail:
    tensor_free(&a); tensor_free(&x); tensor_free(&y);
    return false;
}

bool tensor_matmul64_selftest(void)
{
    tensor_t a = {0};
    tensor_t b = {0};
    tensor_t c = {0};
    const u32 m = 4U;
    const u32 k = 64U;
    const u32 p = 4U;
    tiny_last_stage = 100;
    tiny_last_status = 0;

    if (!tensor_alloc(&a, m, k, sizeof(float)) ||
        !tensor_alloc(&b, k, p, sizeof(float)) ||
        !tensor_alloc(&c, m, p, sizeof(float))) {
        tiny_last_stage = 101;
        tiny_last_status = (i32)V3D_STATUS_NOT_READY;
        tensor_free(&a); tensor_free(&b); tensor_free(&c);
        return false;
    }

    float *ap = (float *)a.arm_ptr;
    float *bp = (float *)b.arm_ptr;
    float *cp = (float *)c.arm_ptr;
    for (u32 i = 0; i < m * k; i++)
        ap[i] = ((float)(i % 19U) - 9.0f) * 0.03125f;
    for (u32 i = 0; i < k * p; i++)
        bp[i] = ((float)(i % 23U) - 11.0f) * 0.03125f;
    for (u32 i = 0; i < m * p; i++)
        cp[i] = 0.0f;

    dcache_clean_range((u64)(usize)a.arm_ptr, a.total_bytes);
    dcache_clean_range((u64)(usize)b.arm_ptr, b.total_bytes);
    dcache_clean_range((u64)(usize)c.arm_ptr, c.total_bytes);

    tiny_last_stage = 102;
    if (!tensor_matmul(&c, &a, &b)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    dcache_invalidate_range((u64)(usize)c.arm_ptr, c.total_bytes);
    tiny_last_stage = 103;
    for (u32 r = 0; r < m; r++) {
        for (u32 col = 0; col < p; col++) {
            float expect = 0.0f;
            for (u32 kk = 0; kk < k; kk++)
                expect += ap[r * k + kk] * bp[kk * p + col];
            if (!f32_close(cp[r * p + col], expect)) {
                tiny_last_status = (i32)V3D_STATUS_FAILED;
                tiny_last_output_bits = f32_bits(cp[r * p + col]);
                tiny_last_expected_bits = f32_bits(expect);
                goto fail;
            }
        }
    }

    tiny_last_stage = 109;
    tiny_last_status = 0;
    tensor_free(&a); tensor_free(&b); tensor_free(&c);
    return true;

fail:
    tensor_free(&a); tensor_free(&b); tensor_free(&c);
    return false;
}

/* ---- Benchmark harness ---- */

/*
 * Scalar baselines. Forced no-vectorize so -O2 cannot silently lower these to
 * NEON; this gives an honest single-lane reference to compare NEON/V3D against.
 */
__attribute__((optimize("no-tree-vectorize")))
static void scalar_add_f32(float *c, const float *a, const float *b, u32 n)
{
    for (u32 i = 0; i < n; i++)
        c[i] = a[i] + b[i];
}

__attribute__((optimize("no-tree-vectorize")))
static void scalar_mul_f32(float *c, const float *a, const float *b, u32 n)
{
    for (u32 i = 0; i < n; i++)
        c[i] = a[i] * b[i];
}

__attribute__((optimize("no-tree-vectorize")))
static void scalar_relu_f32(float *b, const float *a, u32 n)
{
    for (u32 i = 0; i < n; i++)
        b[i] = (a[i] > 0.0f) ? a[i] : 0.0f;
}

__attribute__((optimize("no-tree-vectorize")))
static float scalar_dot_f32(const float *a, const float *b, u32 n)
{
    float s = 0.0f;
    for (u32 i = 0; i < n; i++)
        s += a[i] * b[i];
    return s;
}

__attribute__((optimize("no-tree-vectorize")))
static void scalar_matmul_f32(float *c, const float *a, const float *b,
                              u32 m, u32 n, u32 p)
{
    for (u32 i = 0; i < m; i++)
        for (u32 j = 0; j < p; j++) {
            float s = 0.0f;
            for (u32 k = 0; k < n; k++)
                s += a[i * n + k] * b[k * p + j];
            c[i * p + j] = s;
        }
}

static inline u64 bench_now(void)
{
    u64 v;
    __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v) :: "memory");
    return v;
}

static inline u64 bench_freq(void)
{
    u64 v;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}

/*
 * Time one (op, shape, backend) and return per-iteration nanoseconds (0 if the
 * backend/op is unsupported or the V3D dispatch did not run). Backends:
 *   TENSOR_BENCH_SCALAR / _NEON call the in-file kernels directly (never touch
 *   V3D, so safe even when the GPU CSD queue is wedged). _V3D binds an N-grid
 *   kernel once then times reps raw dispatches (no per-iter verify); only run
 *   it via `tensor bench v3d` after a clean cold boot.
 */
u64 tensor_bench(u32 op, u32 m, u32 k, u32 p, u32 backend, u32 reps)
{
    if (reps == 0U)
        return 0U;

    bool elementwise = (op == TENSOR_BENCH_ADD || op == TENSOR_BENCH_MUL ||
                        op == TENSOR_BENCH_RELU);
    u32 n = m * k;

    tensor_t a = {0};
    tensor_t b = {0};
    tensor_t c = {0};
    u64 per_ns = 0U;

    if (elementwise || op == TENSOR_BENCH_DOT) {
        if (n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS)
            return 0U;
        if (!tensor_alloc(&a, 1, n, sizeof(float)) ||
            !tensor_alloc(&b, 1, n, sizeof(float)) ||
            !tensor_alloc(&c, 1, n, sizeof(float)))
            goto done;
    } else if (op == TENSOR_BENCH_MATMUL) {
        if (m == 0U || k == 0U || p == 0U ||
            (m * k) > TENSOR_V3D_VEC_MAX_ELEMS ||
            (k * p) > TENSOR_V3D_VEC_MAX_ELEMS ||
            (m * p) > TENSOR_V3D_VEC_MAX_ELEMS)
            return 0U;
        if (!tensor_alloc(&a, m, k, sizeof(float)) ||
            !tensor_alloc(&b, k, p, sizeof(float)) ||
            !tensor_alloc(&c, m, p, sizeof(float)))
            goto done;
    } else {
        return 0U;
    }

    float *ap = (float *)a.arm_ptr;
    float *bp = (float *)b.arm_ptr;
    float *cp = (float *)c.arm_ptr;
    u32 acount = a.rows * a.cols;
    u32 bcount = b.rows * b.cols;
    for (u32 i = 0; i < acount; i++)
        ap[i] = ((float)(i % 17U) - 8.0f) * 0.125f;
    for (u32 i = 0; i < bcount; i++)
        bp[i] = ((float)(i % 13U) - 6.0f) * 0.0625f;

    if (backend == TENSOR_BENCH_V3D) {
        if (!elementwise || use_qpu_fallback || !v3d_dispatch_supported())
            goto done;
        v3d_kernel_id_t id = (op == TENSOR_BENCH_ADD) ? V3D_KERNEL_ADD :
                             (op == TENSOR_BENCH_MUL) ? V3D_KERNEL_MUL :
                                                        V3D_KERNEL_RELU;
        if (v3d_kernel_disabled[id])
            goto done;
        dcache_clean_range((u64)(usize)a.arm_ptr, a.total_bytes);
        dcache_clean_range((u64)(usize)b.arm_ptr, b.total_bytes);
        dcache_clean_range((u64)(usize)c.arm_ptr, c.total_bytes);
        u32 wgs = (n + 15U) / 16U;
        v3d_status_t st;
        if (op == TENSOR_BENCH_RELU) {
            u32 u[6] = { 0x0000FFFFU, 0x1AU, n, 0x00000008U,
                         a.bus_addr & 0x3FFFFFFFU, c.bus_addr & 0x3FFFFFFFU };
            st = v3d_kernel_bind_builtin_qpu_grid(id, u, sizeof(u), wgs);
        } else {
            u32 u[7] = { 0x0000FFFFU, 0x1AU, n, 0x0000000CU,
                         a.bus_addr & 0x3FFFFFFFU, b.bus_addr & 0x3FFFFFFFU,
                         c.bus_addr & 0x3FFFFFFFU };
            st = v3d_kernel_bind_builtin_qpu_grid(id, u, sizeof(u), wgs);
        }
        if (st != V3D_STATUS_OK)
            goto done;
        if (v3d_dispatch_kernel(id, 25) != V3D_STATUS_OK)
            goto done;
        u64 t0 = bench_now();
        for (u32 r = 0; r < reps; r++) {
            if (v3d_dispatch_kernel(id, 25) != V3D_STATUS_OK) {
                per_ns = 0U;
                goto done;
            }
        }
        u64 dt = bench_now() - t0;
        per_ns = (dt * 1000000000ULL) / bench_freq() / reps;
        goto done;
    }

    /* Scalar / NEON: call the in-file kernels directly (no V3D touch). */
    {
        volatile float sink = 0.0f;
        u64 t0 = bench_now();
        for (u32 r = 0; r < reps; r++) {
            switch (op) {
            case TENSOR_BENCH_ADD:
                if (backend == TENSOR_BENCH_NEON)
                    neon_vec_add_f32(cp, ap, bp, n);
                else
                    scalar_add_f32(cp, ap, bp, n);
                break;
            case TENSOR_BENCH_MUL:
                if (backend == TENSOR_BENCH_NEON)
                    neon_vec_mul_f32(cp, ap, bp, n);
                else
                    scalar_mul_f32(cp, ap, bp, n);
                break;
            case TENSOR_BENCH_RELU:
                if (backend == TENSOR_BENCH_NEON)
                    neon_vec_relu_f32(cp, ap, n);
                else
                    scalar_relu_f32(cp, ap, n);
                break;
            case TENSOR_BENCH_DOT:
                sink += (backend == TENSOR_BENCH_NEON)
                            ? neon_vec_dot_f32(ap, bp, n)
                            : scalar_dot_f32(ap, bp, n);
                break;
            case TENSOR_BENCH_MATMUL:
                if (backend == TENSOR_BENCH_NEON)
                    neon_matmul_f32(cp, ap, bp, m, k, p);
                else
                    scalar_matmul_f32(cp, ap, bp, m, k, p);
                break;
            default:
                break;
            }
        }
        u64 dt = bench_now() - t0;
        (void)sink;
        per_ns = (dt * 1000000000ULL) / bench_freq() / reps;
    }

done:
    tensor_free(&a);
    tensor_free(&b);
    tensor_free(&c);
    return per_ns;
}

bool tensor_softmax(tensor_t *b, const tensor_t *a) {
    if (b->rows != a->rows || b->cols != a->cols) return false;

    float *src = (float *)a->arm_ptr;
    float *dst = (float *)b->arm_ptr;
    u32 rows = a->rows;
    u32 cols = a->cols;
    u64 elems = (u64)rows * (u64)cols;

    if (!use_qpu_fallback && tensor_try_v3d(V3D_KERNEL_SOFTMAX, elems))
        return true;

    for (u32 r = 0; r < rows; r++) {
        float *row_s = src + r * cols;
        float *row_d = dst + r * cols;
        u32 c;

        /* NEON max-finding: 4 lanes parallel compare */
        float max_val = row_s[0];
        if (cols >= 8) {
            u32 max_bits = 0;
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
                : "=r"(max_bits) :: "v0"
            );
            max_val = bits_f32(max_bits);
            for (; c < cols; c++)
                if (row_s[c] > max_val) max_val = row_s[c];
        } else {
            for (c = 1; c < cols; c++)
                if (row_s[c] > max_val) max_val = row_s[c];
        }

        /* exp(x - max) via Schraudolph's approximation and sum */
        for (c = 0; c < cols; c++) {
            float x = row_s[c] - max_val;
            if (x < -87.0f) x = -87.0f;
            union { float f; u32 i; } v;
            v.i = (u32)(12102203.2f * x + 1065353216.0f);
            row_d[c] = v.f;
        }
        float sum = neon_sum_f32(row_d, cols);

        /* NEON normalize: multiply all by 1/sum */
        if (sum > 0.0f) {
            float inv_sum = 1.0f / sum;
            float *row_n = row_d;
            c = 0;
            if (cols >= 4) {
                __asm__ volatile("dup v2.4s, %w0" :: "r"(f32_bits(inv_sum)) : "v2");
                for (; c + 4 <= cols; c += 4) {
                    __asm__ volatile(
                        "ld1  {v0.4s}, [%0]       \n"
                        "fmul v0.4s, v0.4s, v2.4s \n"
                        "st1  {v0.4s}, [%0], #16  \n"
                        : "+r"(row_n) :: "v0","v2","memory"
                    );
                }
            }
            for (; c < cols; c++)
                row_d[c] *= inv_sum;
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
    if (!prog || !jobs) return false;
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

    bool ok = false;
    if (v3d_dispatch_supported()) {
        struct v3d_dispatch_cfg cfg;
        cfg.qpu_count = num_qpus;
        cfg.control_list_bus = control_bus;
        for (u32 i = 0; i < 7U; i++)
            cfg.csd_cfg[i] = 0;
        cfg.csd_cfg_valid = false;
        cfg.noflush = false;
        cfg.timeout_ms = 25;
        cfg.backend = V3D_BACKEND_AUTO;
        ok = (v3d_dispatch_compute(&cfg) == V3D_STATUS_OK);
    } else {
        ok = qpu_execute(num_qpus, control_bus, false);
    }

    gpu_mem_unlock(control_handle);
    gpu_mem_free(control_handle);

    return ok;
}

void tensor_init(void) {
#if !PIOS_HAS_MAILBOX_FB
    for (u32 i = 0; i < V3D_KERNEL_MAX; i++) {
        v3d_kernel_disabled[i] = true;
        v3d_direct_grid_disabled[i] = false;
        v3d_kernel_warned[i] = false;
    }
    use_qpu_fallback = true;
    uart_puts("[gpu] skipped on this platform; tensor NEON fallback ready\n");
    uart_puts("[ten] NEON: add/mul/scale/dot/mm/relu/sm\n");
    return;
#else
    gpu_init();
    v3d_init();
    for (u32 i = 0; i < V3D_KERNEL_MAX; i++) {
        v3d_kernel_disabled[i] = false;
        v3d_direct_grid_disabled[i] = false;
        v3d_kernel_warned[i] = false;
    }
    use_qpu_fallback = !v3d_dispatch_supported();
    if (qpu_enable(true)) {
        uart_puts("[ten] QPU on (12)\n");
    } else {
        uart_puts("[ten] QPU fail, NEON fallback\n");
        use_qpu_fallback = true;
    }
    if (!tensor_any_bound_v3d_kernel())
#if PIOS_ENABLE_TINY_QPU_KERNELS
        use_qpu_fallback = !v3d_dispatch_supported();
#else
        use_qpu_fallback = true;
#endif
    uart_puts("[ten] NEON: add/mul/scale/dot/mm/relu/sm\n");
#endif
}
