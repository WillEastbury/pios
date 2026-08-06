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
#include "bitnet_pi5_fixture.h"
#include "bitnet_kernel.h"
#include "gpu.h"
#include "v3d.h"
#include "v3d_matvec64_qpu.h"
#include "v3d_matmul64x16_qpu.h"
#include "simd.h"
#include "mmu.h"
#include "pico_hooks.h"
#include "picovm.h"
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
static bool picovm_qpu_ready;
static u32 picovm_qpu_ops;
static bool picovm_prefer_qpu_int8;
static bool picovm_force_qpu_int8;
static bool prefer_qpu_ternary;
static u32 picovm_bitlinear_qpu_ops;
static bool prefer_qpu_fp32_tiles;
static bool picovm_media_qpu_ready;
static bool picovm_h264_qpu_ready;
/* Core-0-only guarded diagnostic context; never touched from scheduler cores. */
static pv_ctx picovm_qpu_test_ctx ALIGNED(64);
static u8 picovm_qpu_test_mem[65536] ALIGNED(64);
static i32 picovm_alu_input[16] ALIGNED(64);
static i32 picovm_alu_output[16] ALIGNED(64);
static inline u64 bench_now(void);
static inline u64 bench_freq(void);
static float neon_vec_dot_f32(const float *a, const float *b, u32 n);
static i32 tensor_neon_dot_i8(const i8 *a, const i8 *b, u32 n);
static bool tensor_qpu_bitnet_bitmap64x16(const u8 *matrix,
                                          const i8 *vectors,
                                          i32 *output,
                                          u32 repeats,
                                          bool full_verify,
                                          u64 *ns_out);
static u64 matvec64_last_dispatch_ns;
static float tiny_a_buf ALIGNED(64);
static float tiny_b_buf ALIGNED(64);
static float tiny_c_buf ALIGNED(64);
static float tiny_r_buf ALIGNED(64);

#define TENSOR_NATIVE_HANDLE_FLAG 0x80000000U
#define TENSOR_NATIVE_POOL_SLOTS  16U
#define TENSOR_NATIVE_POOL_BYTES  4096U
static u8 tensor_native_pool[TENSOR_NATIVE_POOL_SLOTS][TENSOR_NATIVE_POOL_BYTES] ALIGNED(4096);
static bool tensor_native_pool_used[TENSOR_NATIVE_POOL_SLOTS];
/* Core-0-owned fused MatVec staging. Not shared with scheduler/user cores. */
static float matvec64_tile_buf[64U * 64U] ALIGNED(4096);
static float matvec64_vec_buf[64U] ALIGNED(64);
static float matvec64_partial_buf[64U] ALIGNED(64);
static float matmul64_vectors_buf[16U * 64U] ALIGNED(4096);
static float matmul64_output_buf[16U * 64U] ALIGNED(4096);
static i8 profile_i8_vectors[16U * 64U] ALIGNED(64);
static i32 profile_i32_output[16U * 64U] ALIGNED(64);
static u8 profile_bitmap_matrix[64U * 16U] ALIGNED(64);

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
    tiny_last_stage = 599;
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

static bool tensor_picovm_span(pv_ctx *ctx, int handle, u32 *ptr, i32 *len)
{
    if (!ctx || !ptr || !len || !ctx->mem ||
        handle <= 0 || handle >= ctx->span_count)
        return false;
    u32 p = ctx->span_ptr[handle];
    i32 n = ctx->span_len[handle];
    if (n < 0 || p > (u32)ctx->mem_size ||
        (u32)n > (u32)ctx->mem_size - p)
        return false;
    *ptr = p;
    *len = n;
    return true;
}

static void tensor_picovm_put_i32be(u8 *p, i32 value)
{
    u32 v = (u32)value;
    p[0] = (u8)(v >> 24);
    p[1] = (u8)(v >> 16);
    p[2] = (u8)(v >> 8);
    p[3] = (u8)v;
}

static i32 tensor_picovm_get_i32be(const u8 *p)
{
    return (i32)(((u32)p[0] << 24) |
                 ((u32)p[1] << 16) |
                 ((u32)p[2] << 8) |
                 (u32)p[3]);
}

static bool tensor_qpu_dot_i8(const u8 *x, const u8 *y, u32 n, i32 *result)
{
    tensor_t a = {0};
    tensor_t b = {0};
    tensor_t product = {0};
    bool ok = false;

    if (!x || !y || !result || n == 0U || n > TENSOR_V3D_VEC_MAX_ELEMS ||
        v3d_kernel_disabled[V3D_KERNEL_MUL])
        return false;
    if (n > 32U) {
        u32 acc = 0U;
        for (u32 off = 0; off < n; off += 32U) {
            u32 chunk = n - off;
            if (chunk > 32U)
                chunk = 32U;
            i32 part = 0;
            if (!tensor_qpu_dot_i8(x + off, y + off, chunk, &part))
                return false;
            acc += (u32)part;
        }
        *result = (i32)acc;
        return true;
    }
    if (!tensor_alloc(&a, 1U, n, sizeof(float)) ||
        !tensor_alloc(&b, 1U, n, sizeof(float)) ||
        !tensor_alloc(&product, 1U, n, sizeof(float)))
        goto out;

    float *af = (float *)a.arm_ptr;
    float *bf = (float *)b.arm_ptr;
    for (u32 i = 0; i < n; i++) {
        af[i] = (float)(signed char)x[i];
        bf[i] = (float)(signed char)y[i];
    }
    dcache_clean_range((u64)(usize)a.arm_ptr, a.total_bytes);
    dcache_clean_range((u64)(usize)b.arm_ptr, b.total_bytes);
    if (!tensor_mul(&product, &a, &b))
        goto out;

    const struct v3d_kernel_desc *mul = v3d_kernel_desc_get(V3D_KERNEL_MUL);
    if (v3d_kernel_disabled[V3D_KERNEL_MUL] || !mul || !mul->verified)
        goto out;

    dcache_invalidate_range((u64)(usize)product.arm_ptr, product.total_bytes);
    const float *pf = (const float *)product.arm_ptr;
    u32 acc = 0U;
    for (u32 i = 0; i < n; i++)
        acc += (u32)(i32)pf[i];
    *result = (i32)acc;
    ok = true;

out:
    tensor_free(&a);
    tensor_free(&b);
    tensor_free(&product);
    return ok;
}

static bool tensor_qpu_matvec_i8_fused64(const u8 *matrix, const u8 *vector,
                                         u32 rows, u32 cols, i32 *out)
{
    bool ok = false;
    if (!matrix || !vector || !out || rows == 0U || rows > 64U ||
        cols == 0U || cols > TENSOR_V3D_VEC_MAX_ELEMS || (cols & 63U) != 0U)
        return false;
    for (u32 row = 0; row < rows; row++)
        out[row] = 0;

    for (u32 col0 = 0; col0 < cols; col0 += 64U) {
        float *tf = matvec64_tile_buf;
        float *vf = matvec64_vec_buf;
        float *pf = matvec64_partial_buf;
        for (u32 col = 0; col < 64U; col++)
            vf[col] = (float)(signed char)vector[col0 + col];
        for (u32 row = 0; row < rows; row++)
            for (u32 col = 0; col < 64U; col++)
                tf[row * 64U + col] =
                    (float)(signed char)matrix[row * cols + col0 + col];
        dcache_clean_range((u64)(usize)tf, rows * 64U * sizeof(float));
        dcache_clean_range((u64)(usize)vf, 64U * sizeof(float));

        u32 ma = (u32)(usize)tf;
        u32 va = (u32)(usize)vf;
        u32 oa = (u32)(usize)pf;
        u32 uniforms[V3D_TINY_MATVEC64_UNIFORMS];
        for (u32 i = 0; i < V3D_TINY_MATVEC64_UNIFORMS; i++) {
            u32 data = v3d_tiny_matvec64_uniform_data[i];
            if (v3d_tiny_matvec64_uniform_kind[i] == 53U)
                uniforms[i] = data == 0U ? ma : (data == 1U ? va : oa);
            else
                uniforms[i] = data;
        }
        if (v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_MATVEC64,
                                             uniforms, sizeof(uniforms),
                                             rows) != V3D_STATUS_OK)
            goto done;
        bool tile_ok = false;
        for (u32 attempt = 0; attempt < 8U; attempt++) {
            simd_zero(pf, sizeof(matvec64_partial_buf));
            for (u32 row = 0; row < rows; row++)
                pf[row] = -123.5f;
            dcache_clean_range((u64)(usize)pf, sizeof(matvec64_partial_buf));
            u64 dispatch_start = bench_now();
            v3d_status_t dispatch_status =
                v3d_dispatch_kernel(V3D_KERNEL_MATVEC64, 25U);
            u64 dispatch_ticks = bench_now() - dispatch_start;
            if (dispatch_status != V3D_STATUS_OK)
                continue;
            matvec64_last_dispatch_ns =
                (dispatch_ticks * 1000000000ULL) / bench_freq();
            dcache_invalidate_range((u64)(usize)pf, sizeof(matvec64_partial_buf));
            tile_ok = true;
            for (u32 row = 0; row < rows; row++) {
                i32 expected = 0;
                for (u32 col = 0; col < 64U; col++)
                    expected += (i32)(signed char)matrix[row * cols + col0 + col] *
                                (i32)(signed char)vector[col0 + col];
                if ((i32)pf[row] != expected) {
                    tile_ok = false;
                    break;
                }
            }
            if (tile_ok)
                break;
        }
        if (!tile_ok)
            goto done;
        for (u32 row = 0; row < rows; row++)
            out[row] = (i32)((u32)out[row] + (u32)(i32)pf[row]);
    }
    v3d_kernel_mark_verified(V3D_KERNEL_MATVEC64, true);
    ok = true;

done:
    if (!ok)
        v3d_kernel_mark_verified(V3D_KERNEL_MATVEC64, false);
    return ok;
}

static bool tensor_qpu_matvec_i8_batch(const u8 *matrix, const u8 *vector,
                                       u32 rows, u32 cols, i32 *out)
{
    tensor_t tile = {0};
    tensor_t vec = {0};
    tensor_t partial = {0};
    bool ok = false;

    if (!matrix || !vector || !out || rows == 0U || rows > 64U ||
        cols == 0U || cols > TENSOR_V3D_VEC_MAX_ELEMS || (cols & 15U) != 0U)
        return false;
    if ((cols & 63U) == 0U &&
        tensor_qpu_matvec_i8_fused64(matrix, vector, rows, cols, out))
        return true;
    if (!tensor_alloc(&tile, rows, 16U, sizeof(float)) ||
        !tensor_alloc(&vec, 1U, 16U, sizeof(float)) ||
        !tensor_alloc(&partial, rows, 1U, sizeof(float)))
        goto done;

    for (u32 row = 0; row < rows; row++)
        out[row] = 0;
    for (u32 col0 = 0; col0 < cols; col0 += 16U) {
        float *tf = (float *)tile.arm_ptr;
        float *vf = (float *)vec.arm_ptr;
        float *pf = (float *)partial.arm_ptr;
        for (u32 col = 0; col < 16U; col++)
            vf[col] = (float)(signed char)vector[col0 + col];
        for (u32 row = 0; row < rows; row++)
            for (u32 col = 0; col < 16U; col++)
                tf[row * 16U + col] =
                    (float)(signed char)matrix[row * cols + col0 + col];
        dcache_clean_range((u64)(usize)tile.arm_ptr, tile.total_bytes);
        dcache_clean_range((u64)(usize)vec.arm_ptr, vec.total_bytes);

        u32 ma = tile.bus_addr & 0x3FFFFFFFU;
        u32 va = vec.bus_addr & 0x3FFFFFFFU;
        u32 oa = partial.bus_addr & 0x3FFFFFFFU;
        u32 uniforms[23] = {
            0x0000FFFFU, 0x00000010U, ma, 0xFFFFFFFCU,
            va, 0xFFFFFFFCU, 0xFFFFFFFCU, 0x00000020U,
            va, 0x00000010U, 0xFFFFFFFCU, ma,
            0xFFFFFFFCU, va, 0x00000020U, 0xFFFFFFFCU,
            0x00000030U, ma, 0xFFFFFFFCU, va,
            0x00000030U, 0xFFFFFFFCU, oa
        };
        if (v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_MATVEC16,
                                             uniforms, sizeof(uniforms),
                                             rows) != V3D_STATUS_OK) {
            tiny_last_stage = 300;
            goto done;
        }
        bool tile_ok = false;
        bool dispatched = false;
        for (u32 attempt = 0; attempt < 8U; attempt++) {
            simd_zero(pf, partial.total_bytes);
            for (u32 row = 0; row < rows; row++)
                pf[row] = -123.5f;
            dcache_clean_range((u64)(usize)partial.arm_ptr, partial.total_bytes);
            if (v3d_dispatch_kernel(V3D_KERNEL_MATVEC16, 25U) != V3D_STATUS_OK)
                continue;
            dispatched = true;
            dcache_invalidate_range((u64)(usize)partial.arm_ptr, partial.total_bytes);
            tile_ok = true;
            for (u32 row = 0; row < rows; row++) {
                i32 expected = 0;
                for (u32 col = 0; col < 16U; col++)
                    expected += (i32)(signed char)matrix[row * cols + col0 + col] *
                                (i32)(signed char)vector[col0 + col];
                i32 got = (i32)pf[row];
                if (got != expected) {
                    tile_ok = false;
                    tiny_last_stage = (i32)(320U + row);
                    tiny_last_output_bits = f32_bits(pf[row]);
                    tiny_last_expected_bits = f32_bits((float)expected);
                    break;
                }
            }
            if (tile_ok)
                break;
        }
        if (!tile_ok) {
            if (!dispatched)
                tiny_last_stage = 301;
            goto done;
        }
        for (u32 row = 0; row < rows; row++) {
            i32 got = (i32)pf[row];
            out[row] = (i32)((u32)out[row] + (u32)got);
        }
    }

    for (u32 row = 0; row < rows; row++) {
        u32 expected = 0U;
        for (u32 col = 0; col < cols; col++)
            expected += (u32)((i32)(signed char)matrix[row * cols + col] *
                              (i32)(signed char)vector[col]);
        if (out[row] != (i32)expected)
            goto done;
    }
    v3d_kernel_mark_verified(V3D_KERNEL_MATVEC16, true);
    tiny_last_stage = 399;
    tiny_last_output_bits = f32_bits((float)out[0]);
    tiny_last_expected_bits = tiny_last_output_bits;
    ok = true;

done:
    if (!ok)
        v3d_kernel_mark_verified(V3D_KERNEL_MATVEC16, false);
    tensor_free(&tile);
    tensor_free(&vec);
    tensor_free(&partial);
    return ok;
}

static bool tensor_qpu_gray_xor(const u8 *src, const u8 *pred, u8 *dst, u32 bytes)
{
    tensor_t s = {0};
    tensor_t p = {0};
    tensor_t d = {0};
    bool ok = false;
    if (!src || !pred || !dst || bytes == 0U || bytes > 4096U ||
        (bytes & 63U) != 0U)
        return false;
    if (!tensor_alloc(&s, 1U, bytes, 1U) ||
        !tensor_alloc(&p, 1U, bytes, 1U) ||
        !tensor_alloc(&d, 1U, bytes, 1U))
        goto done;
    simd_memcpy(s.arm_ptr, src, bytes);
    simd_memcpy(p.arm_ptr, pred, bytes);
    dcache_clean_range((u64)(usize)s.arm_ptr, bytes);
    dcache_clean_range((u64)(usize)p.arm_ptr, bytes);

    for (u32 off = 0; off < bytes; off += 64U) {
        u32 sa = (s.bus_addr + off) & 0x3FFFFFFFU;
        u32 pa = (p.bus_addr + off) & 0x3FFFFFFFU;
        u32 da = (d.bus_addr + off) & 0x3FFFFFFFU;
        u32 uniforms[15] = {
            sa, pa, da, 0x10U, 0x14U, 0x18U, 0x1CU, 0x20U,
            0x24U, 0x28U, 0x2CU, 0x30U, 0x34U, 0x38U, 0x3CU
        };
        bool tile_ok = false;
        if (v3d_kernel_bind_builtin_qpu(V3D_KERNEL_GRAY_XOR64,
                                        uniforms, sizeof(uniforms)) != V3D_STATUS_OK)
            goto done;
        for (u32 attempt = 0; attempt < 3U; attempt++) {
            simd_memset((u8 *)d.arm_ptr + off, 0xA5U, 64U);
            dcache_clean_range((u64)(usize)d.arm_ptr + off, 64U);
            if (v3d_dispatch_kernel(V3D_KERNEL_GRAY_XOR64, 25U) != V3D_STATUS_OK)
                continue;
            dcache_invalidate_range((u64)(usize)d.arm_ptr + off, 64U);
            tile_ok = true;
            for (u32 i = 0; i < 64U; i++) {
                if (((u8 *)d.arm_ptr)[off + i] !=
                    (u8)(src[off + i] ^ pred[off + i])) {
                    tiny_last_output_bits = ((u8 *)d.arm_ptr)[off + i];
                    tiny_last_expected_bits = (u8)(src[off + i] ^ pred[off + i]);
                    tile_ok = false;
                    break;
                }
            }
            if (tile_ok)
                break;
        }
        if (!tile_ok)
            goto done;
    }
    simd_memcpy(dst, d.arm_ptr, bytes);
    v3d_kernel_mark_verified(V3D_KERNEL_GRAY_XOR64, true);
    ok = true;

done:
    if (!ok)
        v3d_kernel_mark_verified(V3D_KERNEL_GRAY_XOR64, false);
    tensor_free(&s);
    tensor_free(&p);
    tensor_free(&d);
    return ok;
}

static bool tensor_qpu_gray_residual(const u8 *src, const u8 *pred,
                                     u8 *dst, u32 bytes, bool restore)
{
    tensor_t s = {0};
    tensor_t p = {0};
    tensor_t d = {0};
    bool ok = false;
    if (!src || !pred || !dst || bytes == 0U || bytes > 4096U ||
        (bytes & 15U) != 0U)
        return false;
    if (!tensor_alloc(&s, 1U, 64U, 1U) ||
        !tensor_alloc(&p, 1U, 64U, 1U) ||
        !tensor_alloc(&d, 1U, 64U, 1U))
        goto done;
    for (u32 off = 0; off < bytes; off += 16U) {
        simd_zero(s.arm_ptr, 64U);
        simd_zero(p.arm_ptr, 64U);
        simd_zero(d.arm_ptr, 64U);
        simd_memcpy(s.arm_ptr, src + off, 16U);
        if (restore) {
            u8 *neg = (u8 *)p.arm_ptr;
            for (u32 i = 0; i < 16U; i++)
                neg[i] = (u8)(0U - pred[off + i]);
        } else {
            simd_memcpy(p.arm_ptr, pred + off, 16U);
        }
        dcache_clean_range((u64)(usize)s.arm_ptr, 64U);
        dcache_clean_range((u64)(usize)p.arm_ptr, 64U);
        u32 sa = s.bus_addr & 0x3FFFFFFFU;
        u32 pa = p.bus_addr & 0x3FFFFFFFU;
        u32 da = d.bus_addr & 0x3FFFFFFFU;
        u32 uniforms[6] = {
            sa, pa, 0x7F7F7F7FU, 0x01010101U, 0x80808080U, da
        };
        bool tile_ok = false;
        if (v3d_kernel_bind_builtin_qpu(V3D_KERNEL_GRAY_RESIDUAL64,
                                        uniforms, sizeof(uniforms)) != V3D_STATUS_OK)
            goto done;
        for (u32 attempt = 0; attempt < 3U; attempt++) {
            simd_memset(d.arm_ptr, 0xA5U, 64U);
            dcache_clean_range((u64)(usize)d.arm_ptr, 64U);
            if (v3d_dispatch_kernel(V3D_KERNEL_GRAY_RESIDUAL64, 25U) != V3D_STATUS_OK)
                continue;
            dcache_invalidate_range((u64)(usize)d.arm_ptr, 64U);
            tile_ok = true;
            for (u32 i = 0; i < 16U; i++) {
                u8 expected = restore
                    ? (u8)(src[off + i] + pred[off + i])
                    : (u8)(src[off + i] - pred[off + i]);
                if (((u8 *)d.arm_ptr)[i] != expected) {
                    tiny_last_output_bits = ((u8 *)d.arm_ptr)[i];
                    tiny_last_expected_bits = expected;
                    tile_ok = false;
                    break;
                }
            }
            if (tile_ok)
                break;
        }
        if (!tile_ok)
            goto done;
        simd_memcpy(dst + off, d.arm_ptr, 16U);
    }
    v3d_kernel_mark_verified(V3D_KERNEL_GRAY_RESIDUAL64, true);
    ok = true;

done:
    if (!ok)
        v3d_kernel_mark_verified(V3D_KERNEL_GRAY_RESIDUAL64, false);
    tensor_free(&s);
    tensor_free(&p);
    tensor_free(&d);
    return ok;
}

bool tensor_matvec_batch_selftest(u32 rows)
{
    u8 matrix[16U * 16U];
    u8 vector[16U];
    i32 out[16U];
    if (rows == 0U || rows > 16U)
        return false;
    for (u32 col = 0; col < 16U; col++)
        vector[col] = (u8)(col + 1U);
    for (u32 row = 0; row < rows; row++)
        for (u32 col = 0; col < 16U; col++)
            matrix[row * 16U + col] = (u8)(row + 1U);
    if (!tensor_qpu_matvec_i8_batch(matrix, vector, rows, 16U, out))
        return false;
    for (u32 row = 0; row < rows; row++)
        if (out[row] != (i32)((row + 1U) * 136U))
            return false;
    return true;
}

static int tensor_picovm_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (!ctx || rd < 0 || rd >= PV_NUM_REGS ||
        rs1 < 0 || rs1 >= PV_NUM_REGS || rs2 < 0 || rs2 >= PV_NUM_REGS)
        return 0;
    if (hook == PV_HOOK_TENSOR_HASACCEL) {
        ctx->regs[rd] = picovm_qpu_ready ? 1 : 0;
        return 1;
    }
    if (!picovm_qpu_ready ||
        (hook != PV_HOOK_TENSOR_DOTI8 && hook != PV_HOOK_TENSOR_MATVECI8))
        return 0;

    u32 xp = 0U;
    u32 yp = 0U;
    i32 xn = 0;
    i32 yn = 0;
    if (!tensor_picovm_span(ctx, ctx->regs[rs1], &xp, &xn) ||
        !tensor_picovm_span(ctx, ctx->regs[rs2], &yp, &yn))
        return 0;

    if (hook == PV_HOOK_TENSOR_DOTI8) {
        i32 n = ctx->tensor_cols > 0 ? ctx->tensor_cols : (xn < yn ? xn : yn);
        if (n <= 0 || n > xn || n > yn)
            return 0;
        if (!picovm_force_qpu_int8 && !picovm_prefer_qpu_int8) {
            ctx->regs[rd] = tensor_neon_dot_i8((const i8 *)(ctx->mem + xp),
                                               (const i8 *)(ctx->mem + yp),
                                               (u32)n);
            return 1;
        }
        i32 result = 0;
        if (!tensor_qpu_dot_i8(ctx->mem + xp, ctx->mem + yp, (u32)n, &result))
            return 0;
        ctx->regs[rd] = result;
        picovm_qpu_ops++;
        return 1;
    }

    u32 rows = ctx->tensor_rows > 0 ? (u32)ctx->tensor_rows : 0U;
    u32 cols = ctx->tensor_cols > 0 ? (u32)ctx->tensor_cols : (u32)yn;
    u64 matrix_bytes = (u64)rows * cols;
    u64 output_bytes = (u64)rows * 4U;
    if (rows == 0U || cols == 0U || cols > TENSOR_V3D_VEC_MAX_ELEMS ||
        matrix_bytes > (u32)xn || cols > (u32)yn ||
        ctx->no_alloc || ctx->span_count >= PV_MAX_SPANS ||
        output_bytes > 0xffffffffULL ||
        ctx->arena_top > (u32)ctx->mem_size ||
        (u32)output_bytes > (u32)ctx->mem_size - ctx->arena_top)
        return 0;

    u32 base = ctx->arena_top;
    if (!picovm_force_qpu_int8 && !picovm_prefer_qpu_int8) {
        for (u32 row = 0; row < rows; row++) {
            i32 result = tensor_neon_dot_i8(
                (const i8 *)(ctx->mem + xp + row * cols),
                (const i8 *)(ctx->mem + yp), cols);
            tensor_picovm_put_i32be(ctx->mem + base + row * 4U, result);
        }
        int handle = ctx->span_count++;
        ctx->span_ptr[handle] = base;
        ctx->span_len[handle] = (i32)output_bytes;
        ctx->arena_top += (u32)output_bytes;
        ctx->regs[rd] = handle;
        return 1;
    }
    i32 batched[64];
    const struct v3d_kernel_desc *matvec =
        v3d_kernel_desc_get(V3D_KERNEL_MATVEC16);
    if (matvec && matvec->verified && rows <= 64U && (cols & 15U) == 0U) {
        if (tensor_qpu_matvec_i8_batch(ctx->mem + xp, ctx->mem + yp,
                                      rows, cols, batched)) {
            for (u32 row = 0; row < rows; row++)
                tensor_picovm_put_i32be(ctx->mem + base + row * 4U, batched[row]);
            int handle = ctx->span_count++;
            ctx->span_ptr[handle] = base;
            ctx->span_len[handle] = (i32)output_bytes;
            ctx->arena_top += (u32)output_bytes;
            ctx->regs[rd] = handle;
            picovm_qpu_ops++;
            return 1;
        }
        return 0;
    }

    for (u32 row = 0; row < rows; row++) {
        i32 result = 0;
        if (!tensor_qpu_dot_i8(ctx->mem + xp + row * cols,
                               ctx->mem + yp, cols, &result))
            return 0;
        tensor_picovm_put_i32be(ctx->mem + base + row * 4U, result);
    }
    int handle = ctx->span_count++;
    ctx->span_ptr[handle] = base;
    ctx->span_len[handle] = (i32)output_bytes;
    ctx->arena_top += (u32)output_bytes;
    ctx->regs[rd] = handle;
    picovm_qpu_ops++;
    return 1;
}

static int tensor_picovm_media_hook(pv_ctx *ctx, int hook,
                                    int rd, int rs1, int rs2)
{
    if (!ctx || rd < 0 || rd >= PV_NUM_REGS ||
        rs1 < 0 || rs1 >= PV_NUM_REGS || rs2 < 0 || rs2 >= PV_NUM_REGS)
        return 0;
    if (hook == PV_HOOK_MEDIA_HASACCEL) {
        ctx->regs[rd] = picovm_media_qpu_ready ? 1 : 0;
        return 1;
    }
    bool xor_op = hook == PV_HOOK_MEDIA_GRAYXORRESIDUAL ||
                  hook == PV_HOOK_MEDIA_GRAYXORRESTORE;
    bool h264_op = hook == PV_HOOK_MEDIA_H264RESIDUAL ||
                   hook == PV_HOOK_MEDIA_H264RESTORE;
    if ((!xor_op && !h264_op) ||
        (xor_op && !picovm_media_qpu_ready) ||
        (h264_op && !picovm_h264_qpu_ready))
        return 0;

    u32 xp = 0U, yp = 0U;
    i32 xn = 0, yn = 0;
    if (!tensor_picovm_span(ctx, ctx->regs[rs1], &xp, &xn) ||
        !tensor_picovm_span(ctx, ctx->regs[rs2], &yp, &yn) ||
        xn <= 0 || xn != yn || (xn & 63) != 0 ||
        ctx->no_alloc || ctx->span_count >= PV_MAX_SPANS ||
        ctx->arena_top > (u32)ctx->mem_size ||
        (u32)xn > (u32)ctx->mem_size - ctx->arena_top)
        return 0;
    u32 base = ctx->arena_top;
    bool ok = xor_op
        ? tensor_qpu_gray_xor(ctx->mem + xp, ctx->mem + yp,
                              ctx->mem + base, (u32)xn)
        : tensor_qpu_gray_residual(ctx->mem + xp, ctx->mem + yp,
                                   ctx->mem + base, (u32)xn,
                                   hook == PV_HOOK_MEDIA_H264RESTORE);
    if (!ok)
        return 0;
    int handle = ctx->span_count++;
    ctx->span_ptr[handle] = base;
    ctx->span_len[handle] = xn;
    ctx->arena_top += (u32)xn;
    ctx->regs[rd] = handle;
    return 1;
}

static int tensor_picovm_bitlinear_hook(pv_ctx *ctx, int hook,
                                        int rd, int rs1, int rs2)
{
    if (!ctx || hook != PV_HOOK_BITLINEAR_MATMULBITMAPBATCH ||
        !prefer_qpu_ternary ||
        ctx->bitlinear_rows != 64 || ctx->bitlinear_cols != 64)
        return 0;
    u32 wp = 0U, vp = 0U;
    i32 wn = 0, vn = 0;
    if (!tensor_picovm_span(ctx, ctx->regs[rs1], &wp, &wn) ||
        !tensor_picovm_span(ctx, ctx->regs[rs2], &vp, &vn) ||
        wn < 64 * 16 || vn < 16 * 64 ||
        ctx->no_alloc || ctx->span_count >= PV_MAX_SPANS ||
        ctx->arena_top > (u32)ctx->mem_size ||
        16U * 64U * 4U > (u32)ctx->mem_size - ctx->arena_top)
        return 0;
    if (!tensor_qpu_bitnet_bitmap64x16(ctx->mem + wp,
                                       (const i8 *)(ctx->mem + vp),
                                       profile_i32_output, 1U, false, NULL))
        return 0;
    u32 base = ctx->arena_top;
    for (u32 i = 0; i < 16U * 64U; i++)
        tensor_picovm_put_i32be(ctx->mem + base + i * 4U,
                                profile_i32_output[i]);
    int handle = ctx->span_count++;
    ctx->span_ptr[handle] = base;
    ctx->span_len[handle] = 16 * 64 * 4;
    ctx->arena_top += 16U * 64U * 4U;
    ctx->regs[rd] = handle;
    picovm_bitlinear_qpu_ops++;
    return 1;
}

bool tensor_picovm_selftest(void)
{
    u8 x[32];
    u8 y[32];
    i32 expected = 0;
    picovm_qpu_ready = false;
    tiny_last_stage = 500;
    if (!tensor_tiny_noop_proof() ||
        !tensor_tiny_store_proof() ||
        !tensor_tiny_load_store_proof()) {
        tiny_last_stage = 501;
        return false;
    }
    for (u32 i = 0; i < 32U; i++) {
        x[i] = (u8)(i32)((i32)(i % 13U) - 6);
        y[i] = (u8)(i32)((i32)(i % 9U) - 4);
        expected += (i32)(signed char)x[i] * (i32)(signed char)y[i];
    }
    picovm_qpu_ready = true;
    pv_init(&picovm_qpu_test_ctx);
    picovm_qpu_test_ctx.mem = picovm_qpu_test_mem;
    picovm_qpu_test_ctx.mem_size = sizeof(picovm_qpu_test_mem);
    for (u32 i = 0; i < 32U; i++) {
        picovm_qpu_test_mem[0x1000U + i] = x[i];
        picovm_qpu_test_mem[0x1100U + i] = y[i];
    }
    int xh = picovm_qpu_test_ctx.span_count++;
    int yh = picovm_qpu_test_ctx.span_count++;
    picovm_qpu_test_ctx.span_ptr[xh] = 0x1000U;
    picovm_qpu_test_ctx.span_len[xh] = 32;
    picovm_qpu_test_ctx.span_ptr[yh] = 0x1100U;
    picovm_qpu_test_ctx.span_len[yh] = 32;
    picovm_qpu_test_ctx.tensor_cols = 32;
    picovm_qpu_test_ctx.regs[1] = xh;
    picovm_qpu_test_ctx.regs[2] = yh;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_TENSOR_HASACCEL, 0, 0, 0, 0);
    if (picovm_qpu_test_ctx.regs[0] != 1) {
        tiny_last_stage = 511;
        picovm_qpu_ready = false;
        return false;
    }
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_TENSOR_DOTI8, 0, 1, 2, 0);
    if (picovm_qpu_test_ctx.regs[0] != expected) {
        tiny_last_stage = 512;
        picovm_qpu_ready = false;
        return false;
    }
    i32 row0 = 0;
    i32 row1 = 0;
    for (u32 i = 0; i < 16U; i++) {
        row0 += (i32)(signed char)x[i] * (i32)(signed char)y[i];
        row1 += (i32)(signed char)x[16U + i] * (i32)(signed char)y[i];
    }
    picovm_qpu_test_ctx.tensor_rows = 2;
    picovm_qpu_test_ctx.tensor_cols = 16;
    picovm_qpu_test_ctx.span_len[yh] = 16;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_TENSOR_MATVECI8, 0, 1, 2, 0);
    int output = picovm_qpu_test_ctx.regs[0];
    if (output <= 0 || output >= picovm_qpu_test_ctx.span_count ||
        picovm_qpu_test_ctx.span_len[output] != 8 ||
        tensor_picovm_get_i32be(picovm_qpu_test_mem +
                                picovm_qpu_test_ctx.span_ptr[output]) != row0 ||
        tensor_picovm_get_i32be(picovm_qpu_test_mem +
                                picovm_qpu_test_ctx.span_ptr[output] + 4U) != row1) {
        tiny_last_stage = 513;
        picovm_qpu_ready = false;
        return false;
    }
    if (!tensor_matvec_batch_selftest(2U)) {
        tiny_last_stage = 520;
        picovm_qpu_ready = false;
        return false;
    }
    for (u32 i = 0; i < 16U * 64U; i++)
        profile_i8_vectors[i] = (i8)((i % 17U) - 8);
    if (!tensor_qpu_bitnet_bitmap64x16(bitnet_wq_bitmap,
                                       profile_i8_vectors,
                                       profile_i32_output, 2U, true, NULL)) {
        tiny_last_stage = 540;
        picovm_qpu_ready = false;
        return false;
    }
    {
        u8 matrix[2U * 64U];
        u8 vector[64U];
        i32 result[2U];
        for (u32 i = 0; i < 64U; i++)
            vector[i] = (u8)(i + 1U);
        for (u32 row = 0; row < 2U; row++)
            for (u32 col = 0; col < 64U; col++)
                matrix[row * 64U + col] = (u8)(row + 1U);
        if (!tensor_qpu_matvec_i8_fused64(matrix, vector, 2U, 64U, result) ||
            result[0] != 2080 || result[1] != 4160) {
            tiny_last_stage = 530;
            picovm_qpu_ready = false;
            return false;
        }
    }
    return true;
}

bool tensor_picovm_accel_ready(void)
{
    return picovm_qpu_ready;
}

bool tensor_picovm_bitlinear_selftest(u64 *elapsed_ns)
{
    if (!elapsed_ns || !picovm_qpu_ready || !prefer_qpu_ternary)
        return false;
    pv_init(&picovm_qpu_test_ctx);
    simd_zero(picovm_qpu_test_mem, sizeof(picovm_qpu_test_mem));
    picovm_qpu_test_ctx.mem = picovm_qpu_test_mem;
    picovm_qpu_test_ctx.mem_size = sizeof(picovm_qpu_test_mem);
    simd_memcpy(picovm_qpu_test_mem + 0x1000U, bitnet_wq_bitmap,
                sizeof(bitnet_wq_bitmap));
    for (u32 i = 0; i < 16U * 64U; i++)
        picovm_qpu_test_mem[0x2000U + i] = (u8)(i8)((i % 17U) - 8);
    int wh = picovm_qpu_test_ctx.span_count++;
    int vh = picovm_qpu_test_ctx.span_count++;
    picovm_qpu_test_ctx.span_ptr[wh] = 0x1000U;
    picovm_qpu_test_ctx.span_len[wh] = sizeof(bitnet_wq_bitmap);
    picovm_qpu_test_ctx.span_ptr[vh] = 0x2000U;
    picovm_qpu_test_ctx.span_len[vh] = 16 * 64;
    picovm_qpu_test_ctx.regs[1] = 64;
    picovm_qpu_test_ctx.regs[2] = 64;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_BITLINEAR_SETSHAPE,
                    0, 1, 2, 0);
    picovm_qpu_test_ctx.regs[1] = wh;
    picovm_qpu_test_ctx.regs[2] = vh;
    u32 ops = picovm_bitlinear_qpu_ops;
    u64 start = bench_now();
    pv_default_host(&picovm_qpu_test_ctx,
                    PV_HOOK_BITLINEAR_MATMULBITMAPBATCH,
                    0, 1, 2, 0);
    u64 ticks = bench_now() - start;
    int output = picovm_qpu_test_ctx.regs[0];
    if (picovm_bitlinear_qpu_ops != ops + 1U ||
        output <= 0 || output >= picovm_qpu_test_ctx.span_count ||
        picovm_qpu_test_ctx.span_len[output] != 16 * 64 * 4)
        return false;
    u32 base = picovm_qpu_test_ctx.span_ptr[output];
    for (u32 vec = 0; vec < 16U; vec++)
        for (u32 row = 0; row < 64U; row++) {
            i32 expected = bitnet_bitmap_row_dot_scalar(
                bitnet_wq_bitmap + row * 16U, 64U,
                (const i8 *)(picovm_qpu_test_mem + 0x2000U + vec * 64U));
            if (tensor_picovm_get_i32be(
                    picovm_qpu_test_mem + base + (vec * 64U + row) * 4U) !=
                expected)
                return false;
        }
    *elapsed_ns = (ticks * 1000000000ULL) / bench_freq();
    return true;
}

bool tensor_picovm_kernel_selftest(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    return false;
#else
    if (!v3d_dispatch_supported())
        return false;

    picovm_alu_input[0] = 7;
    picovm_alu_input[1] = -3;
    picovm_alu_input[2] = 11;
    picovm_alu_output[0] = -1;
    dcache_clean_range((u64)(usize)picovm_alu_input, 64U);
    dcache_clean_range((u64)(usize)picovm_alu_output, 64U);
    u32 uniforms[2] = {
        (u32)(usize)picovm_alu_input,
        (u32)(usize)picovm_alu_output
    };
    if (v3d_kernel_bind_builtin_qpu(V3D_KERNEL_PICOVM_ALU,
                                    uniforms, sizeof(uniforms)) != V3D_STATUS_OK)
        return false;

    for (u32 attempt = 0; attempt < 2U; attempt++) {
        picovm_alu_output[0] = -1;
        dcache_clean_range((u64)(usize)picovm_alu_output, 64U);
        if (v3d_dispatch_kernel(V3D_KERNEL_PICOVM_ALU, 25U) != V3D_STATUS_OK)
            continue;
        dcache_invalidate_range((u64)(usize)picovm_alu_output, 64U);
        if (picovm_alu_output[0] == 44) {
            v3d_kernel_mark_verified(V3D_KERNEL_PICOVM_ALU, true);
            return true;
        }
    }
    v3d_kernel_mark_verified(V3D_KERNEL_PICOVM_ALU, false);
    return false;
#endif
}

bool tensor_picovm_kernel_bench(u32 reps, u64 *cpu_ns, u64 *qpu_ns)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    (void)reps;
    (void)cpu_ns;
    (void)qpu_ns;
    return false;
#else
    if (!cpu_ns || !qpu_ns || reps == 0U || reps > 256U)
        return false;
    u32 uniforms[2] = {
        (u32)(usize)picovm_alu_input,
        (u32)(usize)picovm_alu_output
    };
    picovm_alu_input[0] = 7;
    picovm_alu_input[1] = -3;
    picovm_alu_input[2] = 11;
    dcache_clean_range((u64)(usize)picovm_alu_input, 64U);
    if (v3d_kernel_bind_builtin_qpu(V3D_KERNEL_PICOVM_ALU,
                                    uniforms, sizeof(uniforms)) != V3D_STATUS_OK)
        return false;
    if (v3d_dispatch_kernel(V3D_KERNEL_PICOVM_ALU, 25U) != V3D_STATUS_OK)
        return false;

    u64 start = bench_now();
    for (u32 i = 0; i < reps; i++) {
        if (v3d_dispatch_kernel(V3D_KERNEL_PICOVM_ALU, 25U) != V3D_STATUS_OK)
            return false;
    }
    u64 qticks = bench_now() - start;

    volatile i32 a = 7;
    volatile i32 b = -3;
    volatile i32 c = 11;
    volatile i32 result = 0;
    start = bench_now();
    for (u32 i = 0; i < reps; i++)
        result = (a + b) * c;
    u64 cticks = bench_now() - start;
    if (result != 44)
        return false;

    u64 freq = bench_freq();
    *cpu_ns = (cticks * 1000000000ULL) / freq / reps;
    *qpu_ns = (qticks * 1000000000ULL) / freq / reps;
    return true;
#endif
}

__attribute__((optimize("no-tree-vectorize")))
static i32 tensor_scalar_dot_i8(const i8 *a, const i8 *b, u32 n)
{
    i32 acc = 0;
    for (u32 i = 0; i < n; i++)
        acc += (i32)a[i] * (i32)b[i];
    return acc;
}

static i32 tensor_neon_dot_i8(const i8 *a, const i8 *b, u32 n)
{
    i32 acc = 0;
    u32 i = 0;
    for (; i + 16U <= n; i += 16U) {
        i32 part;
        __asm__ volatile(
            "ld1 {v0.16b}, [%1]\n"
            "ld1 {v1.16b}, [%2]\n"
            "smull v2.8h, v0.8b, v1.8b\n"
            "smull2 v3.8h, v0.16b, v1.16b\n"
            "saddlp v2.4s, v2.8h\n"
            "saddlp v3.4s, v3.8h\n"
            "add v2.4s, v2.4s, v3.4s\n"
            "addv s2, v2.4s\n"
            "fmov %w0, s2\n"
            : "=r"(part)
            : "r"(a + i), "r"(b + i)
            : "v0", "v1", "v2", "v3", "memory"
        );
        acc += part;
    }
    for (; i < n; i++)
        acc += (i32)a[i] * (i32)b[i];
    return acc;
}

static void tensor_project_i8(const i8 *matrix, const i8 *vector, i32 *out,
                              bool neon)
{
    for (u32 row = 0; row < BITNET_PI5_DIM; row++)
        out[row] = neon
            ? tensor_neon_dot_i8(matrix + row * BITNET_PI5_DIM,
                                 vector, BITNET_PI5_DIM)
            : tensor_scalar_dot_i8(matrix + row * BITNET_PI5_DIM,
                                   vector, BITNET_PI5_DIM);
}

static bool tensor_projection_signature(const i32 *out, i32 *argmax, i32 *checksum)
{
    i32 best = 0;
    i32 sum = 0;
    for (u32 i = 0; i < BITNET_PI5_DIM; i++) {
        if (out[i] > out[best])
            best = (i32)i;
        sum += (i32)(i + 1U) * out[i];
    }
    *argmax = best;
    *checksum = sum;
    return best == BITNET_PI5_EXPECTED_ARGMAX &&
           sum == BITNET_PI5_EXPECTED_CHECKSUM;
}

bool tensor_picovm_bitnet_selftest(i32 *argmax_out, i32 *checksum_out,
                                   u64 *scalar_ns_out, u64 *neon_ns_out,
                                   u64 *qpu_kernel_ns_out, u64 *qpu_total_ns_out)
{
    if (!argmax_out || !checksum_out || !scalar_ns_out || !neon_ns_out ||
        !qpu_kernel_ns_out || !qpu_total_ns_out ||
        !picovm_qpu_ready)
        return false;

    pv_init(&picovm_qpu_test_ctx);
    simd_zero(picovm_qpu_test_mem, sizeof(picovm_qpu_test_mem));
    picovm_qpu_test_ctx.mem = picovm_qpu_test_mem;
    picovm_qpu_test_ctx.mem_size = sizeof(picovm_qpu_test_mem);

    for (u32 row = 0; row < BITNET_PI5_DIM; row++) {
        const u8 *zero = bitnet_wq_bitmap + row * 16U;
        const u8 *minus = zero + 8U;
        for (u32 col = 0; col < BITNET_PI5_DIM; col++) {
            u8 bit = (u8)(1U << (col & 7U));
            u8 value = (zero[col >> 3] & bit) ? 0U :
                       ((minus[col >> 3] & bit) ? 0xFFU : 1U);
            picovm_qpu_test_mem[4096U + row * BITNET_PI5_DIM + col] = value;
        }
    }
    for (u32 i = 0; i < BITNET_PI5_DIM; i++)
        picovm_qpu_test_mem[12288U + i] = (u8)bitnet_act_i8[i];

    u32 ops_before = picovm_qpu_ops;
    picovm_force_qpu_int8 = true;
    u64 start = bench_now();
    (void)pv_vm_run(&picovm_qpu_test_ctx, bitnet_pi5_program,
                    (int)(sizeof(bitnet_pi5_program) / sizeof(bitnet_pi5_program[0])));
    u64 ticks = bench_now() - start;
    picovm_force_qpu_int8 = false;
    if (picovm_qpu_test_ctx.fault != PV_FAULT_NONE ||
        picovm_qpu_test_ctx.out_len != 4 ||
        picovm_qpu_ops != ops_before + 1U)
        return false;

    i32 argmax = tensor_picovm_get_i32be(picovm_qpu_test_ctx.out);
    int output = picovm_qpu_test_ctx.span_count - 1;
    if (output <= 0 || picovm_qpu_test_ctx.span_len[output] != 64 * 4)
        return false;
    u32 base = picovm_qpu_test_ctx.span_ptr[output];
    i32 checksum = 0;
    for (u32 i = 0; i < BITNET_PI5_DIM; i++)
        checksum += (i32)(i + 1U) *
                    tensor_picovm_get_i32be(picovm_qpu_test_mem + base + i * 4U);

    u64 qpu_ns = (ticks * 1000000000ULL) / bench_freq();
    if (argmax != BITNET_PI5_EXPECTED_ARGMAX ||
        checksum != BITNET_PI5_EXPECTED_CHECKSUM)
        return false;

    const i8 *matrix = (const i8 *)(picovm_qpu_test_mem + 4096U);
    const i8 *vector = (const i8 *)(picovm_qpu_test_mem + 12288U);
    i32 reference[BITNET_PI5_DIM];
    volatile i32 sink = 0;
    const u32 reps = 128U;

    start = bench_now();
    for (u32 rep = 0; rep < reps; rep++) {
        tensor_project_i8(matrix, vector, reference, false);
        sink ^= reference[rep & (BITNET_PI5_DIM - 1U)];
    }
    ticks = bench_now() - start;
    u64 scalar_ns = (ticks * 1000000000ULL) / bench_freq() / reps;
    i32 scalar_argmax = 0;
    i32 scalar_checksum = 0;
    if (!tensor_projection_signature(reference, &scalar_argmax, &scalar_checksum))
        return false;

    start = bench_now();
    for (u32 rep = 0; rep < reps; rep++) {
        tensor_project_i8(matrix, vector, reference, true);
        sink ^= reference[rep & (BITNET_PI5_DIM - 1U)];
    }
    ticks = bench_now() - start;
    u64 neon_ns = (ticks * 1000000000ULL) / bench_freq() / reps;
    i32 neon_argmax = 0;
    i32 neon_checksum = 0;
    if (!tensor_projection_signature(reference, &neon_argmax, &neon_checksum) ||
        neon_argmax != scalar_argmax || neon_checksum != scalar_checksum)
        return false;
    (void)sink;

    *argmax_out = argmax;
    *checksum_out = checksum;
    *scalar_ns_out = scalar_ns;
    *neon_ns_out = neon_ns;
    *qpu_kernel_ns_out = matvec64_last_dispatch_ns;
    *qpu_total_ns_out = qpu_ns;
    return true;
}

bool tensor_picovm_media_selftest(void)
{
    static const u8 gray[8] = {10, 12, 15, 20, 100, 90, 80, 70};
    static const u8 pred[8] = {8, 10, 14, 18, 95, 95, 75, 75};
    pv_init(&picovm_qpu_test_ctx);
    picovm_qpu_test_ctx.mem = picovm_qpu_test_mem;
    picovm_qpu_test_ctx.mem_size = sizeof(picovm_qpu_test_mem);
    for (u32 i = 0; i < 8U; i++) {
        picovm_qpu_test_mem[0x1000U + i] = gray[i];
        picovm_qpu_test_mem[0x1100U + i] = pred[i];
    }
    picovm_qpu_test_ctx.regs[1] = 4;
    picovm_qpu_test_ctx.regs[2] = 2;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_MEDIA_SETSHAPE, 0, 1, 2, 0);
    int gh = picovm_qpu_test_ctx.span_count++;
    int ph = picovm_qpu_test_ctx.span_count++;
    picovm_qpu_test_ctx.span_ptr[gh] = 0x1000U;
    picovm_qpu_test_ctx.span_len[gh] = 8;
    picovm_qpu_test_ctx.span_ptr[ph] = 0x1100U;
    picovm_qpu_test_ctx.span_len[ph] = 8;

    picovm_qpu_test_ctx.regs[1] = gh;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_MEDIA_GRAYDELTAENCODE, 0, 1, 0, 0);
    picovm_qpu_test_ctx.regs[1] = picovm_qpu_test_ctx.regs[0];
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_MEDIA_GRAYDELTADECODE, 0, 1, 0, 0);
    int decoded = picovm_qpu_test_ctx.regs[0];
    if (decoded <= 0 || picovm_qpu_test_ctx.span_len[decoded] != 8)
        return false;
    for (u32 i = 0; i < 8U; i++)
        if (picovm_qpu_test_mem[picovm_qpu_test_ctx.span_ptr[decoded] + i] != gray[i])
            return false;

    picovm_qpu_test_ctx.regs[1] = gh;
    picovm_qpu_test_ctx.regs[2] = ph;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_MEDIA_H264RESIDUAL, 0, 1, 2, 0);
    picovm_qpu_test_ctx.regs[1] = picovm_qpu_test_ctx.regs[0];
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_MEDIA_H264RESTORE, 0, 1, 2, 0);
    int restored = picovm_qpu_test_ctx.regs[0];
    if (restored <= 0 || picovm_qpu_test_ctx.span_len[restored] != 8)
        return false;
    for (u32 i = 0; i < 8U; i++)
        if (picovm_qpu_test_mem[picovm_qpu_test_ctx.span_ptr[restored] + i] != gray[i])
            return false;
    pv_default_host(&picovm_qpu_test_ctx, PV_HOOK_MEDIA_HASHEVC, 0, 0, 0, 0);
    if (picovm_qpu_test_ctx.regs[0] != 0)
        return false;
    u8 a[64], b[64], residual[64], restored_qpu[64];
    for (u32 i = 0; i < 64U; i++) {
        a[i] = (u8)(i * 17U + 3U);
        b[i] = (u8)(i * 9U + 11U);
    }
    picovm_media_qpu_ready = false;
    picovm_h264_qpu_ready = false;
    tiny_last_stage = 400;
    if (!tensor_qpu_gray_xor(a, b, residual, sizeof(residual)) ||
        !tensor_qpu_gray_xor(residual, b, restored_qpu, sizeof(restored_qpu))) {
        tiny_last_stage = 401;
        return false;
    }
    for (u32 i = 0; i < 64U; i++)
        if (restored_qpu[i] != a[i]) {
            tiny_last_stage = 402;
            return false;
        }
    picovm_media_qpu_ready = true;
    tiny_last_stage = 410;
    if (tensor_qpu_gray_residual(a, b, residual, sizeof(residual), false) &&
        tensor_qpu_gray_residual(residual, b, restored_qpu,
                                 sizeof(restored_qpu), true)) {
        bool h264_ok = true;
        for (u32 i = 0; i < 64U; i++) {
            if (restored_qpu[i] != a[i]) {
                h264_ok = false;
                break;
            }
        }
        picovm_h264_qpu_ready = h264_ok;
    }
    tiny_last_stage = 499;
    tiny_last_output_bits = 0U;
    tiny_last_expected_bits = 0U;
    return true;
}

bool tensor_picovm_media_accel_ready(void)
{
    return picovm_media_qpu_ready;
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
    const struct v3d_caps *caps = v3d_caps_get();

    /*
     * Native CSD uses PIOS's identity-mapped V3D MMU. Mailbox allocations
     * return VideoCore bus aliases whose legacy 0x3fffffff conversion is not a
     * physical-address contract on Pi5, so keep native tensors in known ARM
     * pages with one unambiguous PA for both processors.
     */
    if (caps && caps->native_mmu_ready && caps->mmio_csd)
        return tensor_alloc_native_pool(t, alloc_size);

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

static bool tiny_dispatch_verify_f32(v3d_kernel_id_t id, float *dst, float expected)
{
    for (u32 attempt = 0; attempt < 2U; attempt++) {
        simd_zero(dst, 64U);
        *dst = -123.5f;
        dcache_clean_range((u64)(usize)dst, 64U);

        v3d_status_t st = v3d_dispatch_kernel(id, 25);
        tiny_last_status = (i32)st;
        if (st != V3D_STATUS_OK)
            continue;

        dcache_invalidate_range((u64)(usize)dst, 64U);
        float out = *dst;
        tiny_last_output_bits = f32_bits(out);
        tiny_last_expected_bits = f32_bits(expected);
        if (f32_close(out, expected))
            return true;
    }

    tiny_last_status = (i32)V3D_STATUS_FAILED;
    return false;
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
    const float av = 1.25f;
    const float bv = 2.50f;
    const float relu_in = 2.25f;
    v3d_status_t st;
    u32 add_uniforms[3];
    u32 mul_uniforms[3];
    u32 relu_uniforms[2];

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

    add_uniforms[0] = (u32)(usize)&tiny_a_buf;
    add_uniforms[1] = (u32)(usize)&tiny_b_buf;
    add_uniforms[2] = (u32)(usize)&tiny_c_buf;
    tiny_last_stage = 10;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_ADD, add_uniforms, sizeof(add_uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;
    tiny_last_stage = 11;
    if (!tiny_dispatch_verify_f32(V3D_KERNEL_ADD, &tiny_c_buf, av + bv)) {
        v3d_kernel_disabled[V3D_KERNEL_ADD] = true;
        return false;
    }
    tiny_last_stage = 12;
    v3d_kernel_mark_verified(V3D_KERNEL_ADD, true);

    mul_uniforms[0] = (u32)(usize)&tiny_a_buf;
    mul_uniforms[1] = (u32)(usize)&tiny_b_buf;
    mul_uniforms[2] = (u32)(usize)&tiny_c_buf;
    tiny_last_stage = 15;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_MUL, mul_uniforms, sizeof(mul_uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;
    tiny_last_stage = 16;
    if (!tiny_dispatch_verify_f32(V3D_KERNEL_MUL, &tiny_c_buf, av * bv)) {
        v3d_kernel_disabled[V3D_KERNEL_MUL] = true;
        return false;
    }
    tiny_last_stage = 17;
    v3d_kernel_mark_verified(V3D_KERNEL_MUL, true);

    tiny_a_buf = relu_in;
    dcache_clean_range((u64)(usize)&tiny_a_buf, 64U);
    relu_uniforms[0] = (u32)(usize)&tiny_a_buf;
    relu_uniforms[1] = (u32)(usize)&tiny_r_buf;
    tiny_last_stage = 20;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_RELU, relu_uniforms, sizeof(relu_uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;
    tiny_last_stage = 21;
    if (!tiny_dispatch_verify_f32(V3D_KERNEL_RELU, &tiny_r_buf, relu_in)) {
        v3d_kernel_disabled[V3D_KERNEL_RELU] = true;
        return false;
    }
    tiny_last_stage = 22;
    v3d_kernel_mark_verified(V3D_KERNEL_RELU, true);

    tiny_last_stage = 99;
    tiny_last_status = 0;
    return true;
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
    tensor_t slots[5] = {0};
    bool ok = false;
    v3d_status_t st;
    u32 uniforms[2];
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

    for (u32 i = 0; i < 5U; i++) {
        if (!tensor_alloc(&slots[i], 1U, 16U, sizeof(float))) {
            tiny_last_stage = 31;
            tiny_last_status = (i32)V3D_STATUS_NOT_READY;
            goto out;
        }
    }
    dst_ptr = slots[4].arm_ptr;
    dst_addr = slots[4].bus_addr & 0x3FFFFFFFU;
    simd_zero(dst_ptr, 64U);
    dcache_clean_range((u64)(usize)dst_ptr, 64U);
    uniforms[0] = dst_addr;
    uniforms[1] = f32_bits(expect);

    tiny_last_stage = 32;
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_STORE_CONST, uniforms, sizeof(uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        goto out;

    tiny_last_stage = 33;
    if (!tiny_dispatch_verify_f32(V3D_KERNEL_STORE_CONST, (float *)dst_ptr, expect)) {
        tiny_last_stage = 34;
        goto out;
    }

    tiny_last_stage = 39;
    tiny_last_status = 0;
    ok = true;
out:
    for (u32 i = 0; i < 5U; i++)
        tensor_free(&slots[i]);
    return ok;
#endif
}

bool tensor_tiny_store_ssbo_proof(void)
{
#if !PIOS_ENABLE_TINY_QPU_KERNELS
    return false;
#else
    const float expect = 3.75f;
    tensor_t dst = {0};
    if (!tensor_alloc(&dst, 1U, 16U, sizeof(float)))
        return false;
    simd_zero(dst.arm_ptr, 64U);
    dcache_clean_range((u64)(usize)dst.arm_ptr, 64U);
    u32 uniforms[3] = {
        0x1AU,
        f32_bits(expect),
        dst.bus_addr & 0x3FFFFFFFU
    };
    bool ok = false;
    if (v3d_kernel_bind_builtin_qpu(V3D_KERNEL_STORE_SSBO,
                                    uniforms, sizeof(uniforms)) == V3D_STATUS_OK) {
        for (u32 attempt = 0; attempt < 2U; attempt++) {
            if (v3d_dispatch_kernel(V3D_KERNEL_STORE_SSBO, 25U) != V3D_STATUS_OK)
                continue;
            dcache_invalidate_range((u64)(usize)dst.arm_ptr, 64U);
            float out = ((float *)dst.arm_ptr)[0];
            tiny_last_output_bits = f32_bits(out);
            tiny_last_expected_bits = f32_bits(expect);
            if (f32_close(out, expect)) {
                ok = true;
                break;
            }
        }
    }
    tensor_free(&dst);
    return ok;
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
    tensor_t src = {0};
    tensor_t dst = {0};
    bool ok = false;
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

    if (!tensor_alloc(&src, 1U, 16U, sizeof(float)) ||
        !tensor_alloc(&dst, 1U, 16U, sizeof(float))) {
        tiny_last_stage = 41;
        tiny_last_status = (i32)V3D_STATUS_NOT_READY;
        tensor_free(&src);
        tensor_free(&dst);
        return false;
    }
    src_ptr = src.arm_ptr;
    dst_ptr = dst.arm_ptr;
    src_addr = src.bus_addr & 0x3FFFFFFFU;
    dst_addr = dst.bus_addr & 0x3FFFFFFFU;
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
        goto out;

    tiny_last_stage = 43;
    if (!tiny_dispatch_verify_f32(V3D_KERNEL_LOAD_STORE, (float *)dst_ptr, expect)) {
        tiny_last_stage = 44;
        goto out;
    }

    tiny_last_stage = 49;
    tiny_last_status = 0;
    ok = true;
out:
    tensor_free(&src);
    tensor_free(&dst);
    return ok;
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
    v3d_status_t st;
    u32 uniforms[3];

    tiny_a_buf = av;
    tiny_b_buf = bv;
    tiny_c_buf = 0.0f;
    dcache_clean_range((u64)(usize)&tiny_a_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_b_buf, 64U);
    dcache_clean_range((u64)(usize)&tiny_c_buf, 64U);
    uniforms[0] = (u32)(usize)&tiny_a_buf;
    uniforms[1] = (u32)(usize)&tiny_b_buf;
    uniforms[2] = (u32)(usize)&tiny_c_buf;

    tiny_last_stage = 52;
    tiny_last_status = 0;
    tiny_last_output_bits = 0;
    tiny_last_expected_bits = f32_bits(expect);
    st = v3d_kernel_bind_builtin_qpu(V3D_KERNEL_ADD, uniforms, sizeof(uniforms));
    tiny_last_status = (i32)st;
    if (st != V3D_STATUS_OK)
        return false;

    tiny_last_stage = 53;
    if (!tiny_dispatch_verify_f32(V3D_KERNEL_ADD, &tiny_c_buf, expect)) {
        tiny_last_stage = 54;
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

#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        dcache_clean_range((u64)(usize)a->arm_ptr, a->total_bytes);
        dcache_clean_range((u64)(usize)b->arm_ptr, b->total_bytes);
        for (u32 attempt = 0; attempt < 2U; attempt++) {
            simd_zero(c->arm_ptr, c->total_bytes);
            dcache_clean_range((u64)(usize)c->arm_ptr, c->total_bytes);
            if (!tensor_v3d_binary_direct(V3D_KERNEL_ADD, c, a, b, n))
                break;
            dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
            if (tensor_verify_add_sample((const float *)c->arm_ptr,
                                         (const float *)a->arm_ptr,
                                         (const float *)b->arm_ptr, n)) {
                v3d_kernel_mark_verified(V3D_KERNEL_ADD, true);
                return true;
            }
        }
        v3d_kernel_mark_verified(V3D_KERNEL_ADD, false);
        v3d_direct_grid_disabled[V3D_KERNEL_ADD] = true;
        v3d_kernel_disabled[V3D_KERNEL_ADD] = true;
        tiny_last_output_bits = f32_bits(((const float *)c->arm_ptr)[0]);
        tiny_last_expected_bits =
            f32_bits(((const float *)a->arm_ptr)[0] + ((const float *)b->arm_ptr)[0]);
        if (!v3d_kernel_warned[V3D_KERNEL_ADD]) {
            uart_puts("[ten] dis V3D add: verify fail\n");
            v3d_kernel_warned[V3D_KERNEL_ADD] = true;
        }
    }
#endif
    neon_vec_add_f32((float *)c->arm_ptr,
                     (const float *)a->arm_ptr,
                     (const float *)b->arm_ptr, n);
    dsb();
    return true;
}

bool tensor_mul(tensor_t *c, const tensor_t *a, const tensor_t *b) {
    u32 n = a->rows * a->cols;
    if (b->rows * b->cols != n || c->rows * c->cols != n) return false;

#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        dcache_clean_range((u64)(usize)a->arm_ptr, a->total_bytes);
        dcache_clean_range((u64)(usize)b->arm_ptr, b->total_bytes);
        for (u32 attempt = 0; attempt < 2U; attempt++) {
            simd_zero(c->arm_ptr, c->total_bytes);
            dcache_clean_range((u64)(usize)c->arm_ptr, c->total_bytes);
            if (!tensor_v3d_binary_direct(V3D_KERNEL_MUL, c, a, b, n))
                break;
            dcache_invalidate_range((u64)(usize)c->arm_ptr, c->total_bytes);
            if (tensor_verify_mul_sample((const float *)c->arm_ptr,
                                         (const float *)a->arm_ptr,
                                         (const float *)b->arm_ptr, n)) {
                v3d_kernel_mark_verified(V3D_KERNEL_MUL, true);
                return true;
            }
        }
        v3d_kernel_mark_verified(V3D_KERNEL_MUL, false);
        v3d_direct_grid_disabled[V3D_KERNEL_MUL] = true;
        v3d_kernel_disabled[V3D_KERNEL_MUL] = true;
        tiny_last_output_bits = f32_bits(((const float *)c->arm_ptr)[0]);
        tiny_last_expected_bits =
            f32_bits(((const float *)a->arm_ptr)[0] * ((const float *)b->arm_ptr)[0]);
    }
#endif
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

#if PIOS_ENABLE_TINY_QPU_KERNELS
    if (!use_qpu_fallback && n > 0U && n <= TENSOR_V3D_VEC_MAX_ELEMS) {
        dcache_clean_range((u64)(usize)a->arm_ptr, a->total_bytes);
        for (u32 attempt = 0; attempt < 2U; attempt++) {
            simd_zero(b->arm_ptr, b->total_bytes);
            dcache_clean_range((u64)(usize)b->arm_ptr, b->total_bytes);
            if (!tensor_v3d_relu_direct(b, a, n))
                break;
            dcache_invalidate_range((u64)(usize)b->arm_ptr, b->total_bytes);
            if (tensor_verify_relu_sample((const float *)b->arm_ptr,
                                          (const float *)a->arm_ptr, n)) {
                v3d_kernel_mark_verified(V3D_KERNEL_RELU, true);
                return true;
            }
        }
        v3d_kernel_mark_verified(V3D_KERNEL_RELU, false);
        v3d_direct_grid_disabled[V3D_KERNEL_RELU] = true;
        v3d_kernel_disabled[V3D_KERNEL_RELU] = true;
        tiny_last_output_bits = f32_bits(((const float *)b->arm_ptr)[0]);
        tiny_last_expected_bits =
            f32_bits(((const float *)a->arm_ptr)[0] > 0.0f ?
                     ((const float *)a->arm_ptr)[0] : 0.0f);
        if (!v3d_kernel_warned[V3D_KERNEL_RELU]) {
            uart_puts("[ten] dis V3D relu: verify fail\n");
            v3d_kernel_warned[V3D_KERNEL_RELU] = true;
        }
    }
#endif
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
        v3d_kernel_disabled[V3D_KERNEL_ADD] ||
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
        v3d_kernel_disabled[V3D_KERNEL_MUL] ||
        !tensor_verify_mul_sample((const float *)c.arm_ptr,
                                  (const float *)a.arm_ptr,
                                  (const float *)b.arm_ptr, 16U)) {
        tiny_last_status = (i32)V3D_STATUS_FAILED;
        goto fail;
    }

    tiny_last_stage = 74;
    if (!tensor_relu(&r, &a) ||
        v3d_kernel_disabled[V3D_KERNEL_RELU] ||
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
    if (tiny_last_expected_bits == 0U) {
        tiny_last_output_bits = f32_bits(cp[0]);
        tiny_last_expected_bits = f32_bits(ap[0] + bp[0]);
    }
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

static void profile_int8_tile(bool neon)
{
    for (u32 vec = 0; vec < 16U; vec++)
        bitnet_bitmap_matvec(bitnet_wq_bitmap, 64U, 64U,
                             profile_i8_vectors + vec * 64U,
                             profile_i32_output + vec * 64U, neon);
}

static void profile_fp32_tile(bool neon)
{
    for (u32 vec = 0; vec < 16U; vec++)
        for (u32 row = 0; row < 64U; row++)
            matmul64_output_buf[vec * 64U + row] = neon
                ? neon_vec_dot_f32(matvec64_tile_buf + row * 64U,
                                   matmul64_vectors_buf + vec * 64U, 64U)
                : scalar_dot_f32(matvec64_tile_buf + row * 64U,
                                 matmul64_vectors_buf + vec * 64U, 64U);
}

static bool profile_qpu_matmul64x16(u64 *ns_out)
{
    u32 ma = (u32)(usize)matvec64_tile_buf;
    u32 va = (u32)(usize)matmul64_vectors_buf;
    u32 oa = (u32)(usize)matmul64_output_buf;
    u32 uniforms[V3D_TINY_MATMUL64X16_UNIFORMS];
    for (u32 i = 0; i < V3D_TINY_MATMUL64X16_UNIFORMS; i++) {
        u32 data = v3d_tiny_matmul64x16_uniform_data[i];
        if (v3d_tiny_matmul64x16_uniform_kind[i] == 53U)
            uniforms[i] = data == 0U ? ma : (data == 1U ? va : oa);
        else
            uniforms[i] = data;
    }
    dcache_clean_range((u64)(usize)matvec64_tile_buf, sizeof(matvec64_tile_buf));
    dcache_clean_range((u64)(usize)matmul64_vectors_buf, sizeof(matmul64_vectors_buf));
    if (v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_MATMUL64X16,
                                         uniforms, sizeof(uniforms),
                                         64U * 16U) != V3D_STATUS_OK)
        return false;
    bool warm = false;
    for (u32 attempt = 0; attempt < 8U; attempt++) {
        simd_zero(matmul64_output_buf, sizeof(matmul64_output_buf));
        dcache_clean_range((u64)(usize)matmul64_output_buf,
                           sizeof(matmul64_output_buf));
        if (v3d_dispatch_kernel(V3D_KERNEL_MATMUL64X16, 25U) == V3D_STATUS_OK) {
            warm = true;
            break;
        }
    }
    if (!warm)
        return false;
    u64 start = bench_now();
    for (u32 tile = 0; tile < 64U; tile++)
        if (v3d_dispatch_kernel(V3D_KERNEL_MATMUL64X16, 25U) != V3D_STATUS_OK)
            return false;
    u64 ticks = bench_now() - start;
    dcache_invalidate_range((u64)(usize)matmul64_output_buf,
                            sizeof(matmul64_output_buf));
    *ns_out = (ticks * 1000000000ULL) / bench_freq();
    return true;
}

static bool tensor_qpu_bitnet_bitmap64x16(const u8 *matrix,
                                          const i8 *vectors,
                                          i32 *output,
                                          u32 repeats,
                                          bool full_verify,
                                          u64 *ns_out)
{
    if (!matrix || !vectors || !output || repeats == 0U)
        return false;
    simd_memcpy(profile_bitmap_matrix, matrix,
                sizeof(profile_bitmap_matrix));
    simd_memcpy(profile_i8_vectors, vectors, sizeof(profile_i8_vectors));
    simd_zero(profile_i32_output, sizeof(profile_i32_output));
    dcache_clean_range((u64)(usize)profile_bitmap_matrix,
                       sizeof(profile_bitmap_matrix));
    dcache_clean_range((u64)(usize)profile_i8_vectors,
                       sizeof(profile_i8_vectors));
    dcache_clean_range((u64)(usize)profile_i32_output,
                       sizeof(profile_i32_output));
    u32 buffers[3] = {
        (u32)(usize)profile_bitmap_matrix,
        (u32)(usize)profile_i8_vectors,
        (u32)(usize)profile_i32_output
    };
    if (v3d_kernel_bind_builtin_qpu_grid(V3D_KERNEL_BITNET_BITMAP64X16,
                                         buffers, sizeof(buffers),
                                         64U * 16U) != V3D_STATUS_OK)
        return false;
    bool warm = false;
    for (u32 attempt = 0; attempt < 3U; attempt++) {
        simd_zero(profile_i32_output, sizeof(profile_i32_output));
        dcache_clean_range((u64)(usize)profile_i32_output,
                           sizeof(profile_i32_output));
        if (v3d_dispatch_kernel(V3D_KERNEL_BITNET_BITMAP64X16, 25U) ==
            V3D_STATUS_OK) {
            dcache_invalidate_range((u64)(usize)profile_i32_output,
                                    sizeof(profile_i32_output));
            warm = true;
            u32 vec_start = full_verify ? 0U : 0U;
            u32 vec_end = full_verify ? 16U : 1U;
            u32 row_start = full_verify ? 0U : 0U;
            u32 row_end = full_verify ? 64U : 4U;
            for (u32 vec = vec_start; vec < vec_end && warm; vec++)
                for (u32 row = row_start; row < row_end; row++) {
                    i32 expected = bitnet_bitmap_row_dot_scalar(
                        matrix + row * 16U, 64U,
                        profile_i8_vectors + vec * 64U);
                    if (profile_i32_output[vec * 64U + row] != expected) {
                        warm = false;
                        break;
                    }
                }
            if (warm)
                break;
        }
        timer_delay_ms(1U);
    }
    if (!warm)
        return false;
    u64 start = bench_now();
    for (u32 tile = 0; tile < repeats; tile++)
        if (v3d_dispatch_kernel(V3D_KERNEL_BITNET_BITMAP64X16, 25U) !=
            V3D_STATUS_OK)
            return false;
    u64 ticks = bench_now() - start;
    dcache_invalidate_range((u64)(usize)profile_i32_output,
                            sizeof(profile_i32_output));
    u32 vec_end = full_verify ? 16U : 1U;
    u32 row_end = full_verify ? 64U : 4U;
    for (u32 vec = 0; vec < vec_end; vec++)
        for (u32 row = 0; row < row_end; row++) {
            i32 expected = bitnet_bitmap_row_dot_scalar(
                matrix + row * 16U, 64U,
                profile_i8_vectors + vec * 64U);
            if (profile_i32_output[vec * 64U + row] != expected)
                return false;
        }
    simd_memcpy(output, profile_i32_output, sizeof(profile_i32_output));
    if (ns_out)
        *ns_out = (ticks * 1000000000ULL) / bench_freq();
    v3d_kernel_mark_verified(V3D_KERNEL_BITNET_BITMAP64X16, true);
    return true;
}

bool tensor_accel_profile_run(struct tensor_accel_profile *out)
{
    if (!out || !v3d_dispatch_supported())
        return false;
    simd_zero(out, sizeof(*out));
    for (u32 i = 0; i < 16U * 64U; i++)
        profile_i8_vectors[i] = (i8)((i % 17U) - 8);

    volatile i32 sink = 0;
    u64 start = bench_now();
    for (u32 tile = 0; tile < 64U; tile++) {
        profile_int8_tile(false);
        sink ^= profile_i32_output[tile & 1023U];
    }
    out->int8_scalar_ns =
        ((bench_now() - start) * 1000000000ULL) / bench_freq();

    start = bench_now();
    for (u32 tile = 0; tile < 64U; tile++) {
        profile_int8_tile(true);
        sink ^= profile_i32_output[tile & 1023U];
    }
    out->int8_neon_ns =
        ((bench_now() - start) * 1000000000ULL) / bench_freq();

    out->int8_qpu_verified = tensor_qpu_bitnet_bitmap64x16(
        bitnet_wq_bitmap, profile_i8_vectors, profile_i32_output,
        64U, true, &out->int8_qpu_ns);

    for (u32 i = 0; i < 64U * 64U; i++)
        matvec64_tile_buf[i] = ((float)((i % 19U) - 9)) * 0.03125f;
    for (u32 i = 0; i < 16U * 64U; i++)
        matmul64_vectors_buf[i] = ((float)((i % 23U) - 11)) * 0.03125f;
    start = bench_now();
    for (u32 tile = 0; tile < 64U; tile++)
        profile_fp32_tile(false);
    out->fp32_scalar_ns =
        ((bench_now() - start) * 1000000000ULL) / bench_freq();
    start = bench_now();
    for (u32 tile = 0; tile < 64U; tile++)
        profile_fp32_tile(true);
    out->fp32_neon_ns =
        ((bench_now() - start) * 1000000000ULL) / bench_freq();
    out->fp32_qpu_verified = profile_qpu_matmul64x16(&out->fp32_qpu_ns);
    if (out->fp32_qpu_verified) {
        for (u32 i = 0; i < 16U * 64U; i++) {
            float diff = matmul64_output_buf[i] -
                         scalar_dot_f32(matvec64_tile_buf + (i & 63U) * 64U,
                                        matmul64_vectors_buf + (i >> 6) * 64U,
                                        64U);
            if (diff < 0.0f) diff = -diff;
            if (diff > 0.001f)
                out->fp32_qpu_verified = false;
        }
    }
    prefer_qpu_ternary =
        out->int8_qpu_verified && out->int8_qpu_ns < out->int8_neon_ns;
    prefer_qpu_fp32_tiles =
        out->fp32_qpu_verified && out->fp32_qpu_ns < out->fp32_neon_ns;
    (void)sink;
    return out->int8_qpu_verified && out->fp32_qpu_verified;
}

bool tensor_prefer_qpu_int8(void)
{
    return picovm_prefer_qpu_int8;
}

bool tensor_prefer_qpu_fp32(void)
{
    return prefer_qpu_fp32_tiles;
}

bool tensor_prefer_qpu_ternary(void)
{
    return prefer_qpu_ternary;
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
    pv_tensor_hook = tensor_picovm_hook;
    pv_media_hook = tensor_picovm_media_hook;
    pv_bitlinear_hook = tensor_picovm_bitlinear_hook;
    picovm_qpu_ready = false;
    picovm_prefer_qpu_int8 = false;
    picovm_force_qpu_int8 = false;
    prefer_qpu_ternary = true;
    picovm_bitlinear_qpu_ops = 0U;
    prefer_qpu_fp32_tiles = v3d_dispatch_supported();
    picovm_media_qpu_ready = false;
    picovm_h264_qpu_ready = false;
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
