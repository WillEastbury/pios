#include "bitnet_kernel.h"

u32 bitnet_bitmap_row_stride(u32 cols)
{
    return ((cols + 7U) / 8U) * 2U;
}

static u32 ctz32(u32 value)
{
#if defined(__aarch64__)
    u32 out;
    __asm__ volatile("rbit %w0, %w1\nclz %w0, %w0"
                     : "=&r"(out) : "r"(value));
    return out;
#else
    return (u32)__builtin_ctz(value);
#endif
}

i32 bitnet_bitmap_row_dot_scalar(const u8 *row, u32 cols, const i8 *act)
{
    u32 mask_bytes = (cols + 7U) / 8U;
    const u8 *zero = row;
    const u8 *minus = row + mask_bytes;
    i32 acc = 0;
    for (u32 block = 0; block < mask_bytes; block++) {
        u32 base = block * 8U;
        u32 valid = cols - base;
        if (valid > 8U) valid = 8U;
        u32 mask = valid == 8U ? 0xFFU : ((1U << valid) - 1U);
        u32 plus = ((u32)~zero[block] & (u32)~minus[block]) & mask;
        u32 neg = ((u32)~zero[block] & (u32)minus[block]) & mask;
        while (plus) {
            u32 lane = ctz32(plus);
            acc += act[base + lane];
            plus &= plus - 1U;
        }
        while (neg) {
            u32 lane = ctz32(neg);
            acc -= act[base + lane];
            neg &= neg - 1U;
        }
    }
    return acc;
}

static i32 dot16_i8(const i8 *a, const i8 *b)
{
#if defined(__aarch64__)
    i32 out;
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
        : "=r"(out) : "r"(a), "r"(b)
        : "v0", "v1", "v2", "v3", "memory");
    return out;
#else
    i32 out = 0;
    for (u32 i = 0; i < 16U; i++) out += (i32)a[i] * b[i];
    return out;
#endif
}

static void dot4_16_i8(const i8 *s0, const i8 *s1,
                       const i8 *s2, const i8 *s3, const i8 *act,
                       i32 *o0, i32 *o1, i32 *o2, i32 *o3)
{
#if defined(__aarch64__)
    i32 r0, r1, r2, r3;
    __asm__ volatile(
        "ld1 {v0.16b}, [%4]\n"
        "ld1 {v1.16b}, [%5]\n"
        "ld1 {v2.16b}, [%6]\n"
        "ld1 {v3.16b}, [%7]\n"
        "ld1 {v4.16b}, [%8]\n"
        "smull v5.8h, v0.8b, v1.8b\n"
        "smull2 v6.8h, v0.16b, v1.16b\n"
        "saddlp v5.4s, v5.8h\nsaddlp v6.4s, v6.8h\n"
        "add v5.4s, v5.4s, v6.4s\naddv s5, v5.4s\nfmov %w0, s5\n"
        "smull v5.8h, v0.8b, v2.8b\n"
        "smull2 v6.8h, v0.16b, v2.16b\n"
        "saddlp v5.4s, v5.8h\nsaddlp v6.4s, v6.8h\n"
        "add v5.4s, v5.4s, v6.4s\naddv s5, v5.4s\nfmov %w1, s5\n"
        "smull v5.8h, v0.8b, v3.8b\n"
        "smull2 v6.8h, v0.16b, v3.16b\n"
        "saddlp v5.4s, v5.8h\nsaddlp v6.4s, v6.8h\n"
        "add v5.4s, v5.4s, v6.4s\naddv s5, v5.4s\nfmov %w2, s5\n"
        "smull v5.8h, v0.8b, v4.8b\n"
        "smull2 v6.8h, v0.16b, v4.16b\n"
        "saddlp v5.4s, v5.8h\nsaddlp v6.4s, v6.8h\n"
        "add v5.4s, v5.4s, v6.4s\naddv s5, v5.4s\nfmov %w3, s5\n"
        : "=r"(r0), "=r"(r1), "=r"(r2), "=r"(r3)
        : "r"(act), "r"(s0), "r"(s1), "r"(s2), "r"(s3)
        : "v0", "v1", "v2", "v3", "v4", "v5", "v6", "memory");
    *o0 = r0; *o1 = r1; *o2 = r2; *o3 = r3;
#else
    *o0 = dot16_i8(s0, act); *o1 = dot16_i8(s1, act);
    *o2 = dot16_i8(s2, act); *o3 = dot16_i8(s3, act);
#endif
}

i32 bitnet_bitmap_row_dot_neon(const u8 *row, u32 cols, const i8 *act)
{
    u32 mask_bytes = (cols + 7U) / 8U;
    const u8 *zero = row;
    const u8 *minus = row + mask_bytes;
    i32 acc = 0;
    u32 col = 0;
    for (; col + 16U <= cols; col += 16U) {
        u32 byte = col >> 3;
        u16 z = (u16)zero[byte] | ((u16)zero[byte + 1U] << 8);
        u16 m = (u16)minus[byte] | ((u16)minus[byte + 1U] << 8);
        u16 plus = (u16)(~z & ~m);
        u16 neg = (u16)(~z & m);
        i8 sign[16] ALIGNED(16);
        for (u32 lane = 0; lane < 16U; lane++)
            sign[lane] = (plus & (1U << lane)) ? 1 :
                         ((neg & (1U << lane)) ? -1 : 0);
        acc += dot16_i8(sign, act + col);
    }
    if (col < cols) {
        u32 byte = col >> 3;
        u32 remaining = cols - col;
        u32 offset = col & 7U;
        for (u32 lane = 0; lane < remaining; lane++) {
            u32 bit_index = offset + lane;
            u32 bi = byte + (bit_index >> 3);
            u8 bit = (u8)(1U << (bit_index & 7U));
            if ((zero[bi] & bit) == 0U)
                acc += (minus[bi] & bit) ? -act[col + lane] : act[col + lane];
        }
    }
    return acc;
}

void bitnet_bitmap_matvec(const u8 *matrix, u32 rows, u32 cols,
                          const i8 *act, i32 *out, bool neon)
{
    u32 stride = bitnet_bitmap_row_stride(cols);
    if (neon && (cols & 15U) == 0U) {
        u32 row = 0;
        for (; row + 4U <= rows; row += 4U) {
            i32 sums[4] = {0, 0, 0, 0};
            for (u32 col = 0; col < cols; col += 16U) {
                i8 signs[4][16] ALIGNED(16);
                for (u32 r = 0; r < 4U; r++) {
                    const u8 *packed = matrix + (row + r) * stride;
                    const u8 *zero = packed;
                    const u8 *minus = packed + stride / 2U;
                    u32 byte = col >> 3;
                    u16 z = (u16)zero[byte] | ((u16)zero[byte + 1U] << 8);
                    u16 m = (u16)minus[byte] | ((u16)minus[byte + 1U] << 8);
                    u16 plus = (u16)(~z & ~m);
                    u16 neg = (u16)(~z & m);
                    for (u32 lane = 0; lane < 16U; lane++)
                        signs[r][lane] = (plus & (1U << lane)) ? 1 :
                                         ((neg & (1U << lane)) ? -1 : 0);
                }
                i32 part[4];
                dot4_16_i8(signs[0], signs[1], signs[2], signs[3],
                           act + col, &part[0], &part[1], &part[2], &part[3]);
                for (u32 r = 0; r < 4U; r++) sums[r] += part[r];
            }
            for (u32 r = 0; r < 4U; r++) out[row + r] = sums[r];
        }
        for (; row < rows; row++)
            out[row] = bitnet_bitmap_row_dot_neon(matrix + row * stride,
                                                  cols, act);
        return;
    }
    for (u32 row = 0; row < rows; row++)
        out[row] = neon
            ? bitnet_bitmap_row_dot_neon(matrix + row * stride, cols, act)
            : bitnet_bitmap_row_dot_scalar(matrix + row * stride, cols, act);
}

bool bitnet_i2s_packed_bytes(u32 rows, u32 cols, u32 *bytes_out)
{
    u64 elements;
    if (!bytes_out || rows == 0U || cols == 0U ||
        (rows % BITNET_I2S_TILE_ROWS) != 0U ||
        (cols % BITNET_I2S_TILE_COLS) != 0U)
        return false;
    elements = (u64)rows * cols;
    if (elements > 0xFFFFFFFFULL || (elements & 3ULL) != 0ULL)
        return false;
    *bytes_out = (u32)(elements / 4ULL);
    return true;
}

static u32 load_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void store_le32(u8 *p, u32 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
    p[2] = (u8)(value >> 16);
    p[3] = (u8)(value >> 24);
}

static u32 bitnet_i2s_interleave_word(u32 value)
{
    u32 out = 0U;
    for (u32 offset = 0U; offset < 16U; offset++) {
        u32 code = (value >> (2U * offset)) & 3U;
        u32 shift = (offset % 4U) * 8U + (offset / 4U) * 2U;
        out |= code << shift;
    }
    return out;
}

static i32 bitnet_i2s_decode(const u8 *tile, u32 perm_row, u32 perm_col)
{
    u32 word = load_le32(tile + perm_row * 8U + (perm_col / 16U) * 4U);
    u32 offset = perm_col & 15U;
    u32 shift = (offset % 4U) * 8U + (offset / 4U) * 2U;
    return (i32)((word >> shift) & 3U) - 2;
}

static void bitnet_i2s_source_position(u32 perm_row, u32 perm_col,
                                       u32 *row_out, u32 *col_out)
{
    u32 thread = perm_row * 2U + perm_col / 16U;
    *row_out = (thread / 16U) * 8U + (thread % 8U);
    *col_out = (perm_col % 16U) + 16U * ((thread % 16U) / 8U);
}

bool bitnet_i2s_pack_microsoft(const i8 *weights, u32 weight_count,
                               u32 rows, u32 cols,
                               u8 *packed, u32 packed_capacity)
{
    u32 required;
    if (!weights || !packed ||
        !bitnet_i2s_packed_bytes(rows, cols, &required) ||
        weight_count < rows * cols || packed_capacity < required)
        return false;
    u32 row_tiles = rows / BITNET_I2S_TILE_ROWS;
    u32 col_tiles = cols / BITNET_I2S_TILE_COLS;
    for (u32 rt = 0U; rt < row_tiles; rt++) {
        for (u32 ct = 0U; ct < col_tiles; ct++) {
            u8 *tile = packed +
                (rt * col_tiles + ct) *
                (BITNET_I2S_TILE_ROWS * BITNET_I2S_TILE_COLS / 4U);
            for (u32 pr = 0U; pr < BITNET_I2S_TILE_ROWS; pr++) {
                u8 compressed[8] = {0};
                for (u32 pc = 0U; pc < BITNET_I2S_TILE_COLS; pc++) {
                    u32 sr, sc;
                    bitnet_i2s_source_position(pr, pc, &sr, &sc);
                    i32 weight = weights[(rt * 16U + sr) * cols +
                                         ct * 32U + sc];
                    if (weight < -1 || weight > 1)
                        return false;
                    compressed[pc / 4U] |=
                        (u8)((u32)(weight + 2) << (2U * (pc & 3U)));
                }
                for (u32 word = 0U; word < 2U; word++)
                    store_le32(tile + pr * 8U + word * 4U,
                               bitnet_i2s_interleave_word(
                                   load_le32(compressed + word * 4U)));
            }
        }
    }
    return true;
}

bool bitnet_i2s_matvec_i32(const u8 *packed, u32 packed_bytes,
                           u32 rows, u32 cols,
                           const i8 *activation, u32 activation_count,
                           i32 *output, u32 output_count)
{
    u32 required;
    if (!packed || !activation || !output ||
        !bitnet_i2s_packed_bytes(rows, cols, &required) ||
        packed_bytes < required || activation_count < cols ||
        output_count < rows)
        return false;
    for (u32 row = 0U; row < rows; row++)
        output[row] = 0;
    u32 row_tiles = rows / BITNET_I2S_TILE_ROWS;
    u32 col_tiles = cols / BITNET_I2S_TILE_COLS;
    for (u32 rt = 0U; rt < row_tiles; rt++) {
        for (u32 ct = 0U; ct < col_tiles; ct++) {
            const u8 *tile = packed +
                (rt * col_tiles + ct) *
                (BITNET_I2S_TILE_ROWS * BITNET_I2S_TILE_COLS / 4U);
            for (u32 pr = 0U; pr < BITNET_I2S_TILE_ROWS; pr++) {
                for (u32 pc = 0U; pc < BITNET_I2S_TILE_COLS; pc++) {
                    u32 sr, sc;
                    bitnet_i2s_source_position(pr, pc, &sr, &sc);
                    output[rt * 16U + sr] +=
                        bitnet_i2s_decode(tile, pr, pc) *
                        activation[ct * 32U + sc];
                }
            }
        }
    }
    return true;
}

bool bitnet_i2s_quantize_activation(const float *input, u32 count,
                                    i8 *output, u32 output_count,
                                    float *scale_out)
{
    float max_abs = 0.0f;
    if (!input || !output || !scale_out || count == 0U ||
        output_count < count)
        return false;
    for (u32 i = 0U; i < count; i++) {
        float value = input[i] < 0.0f ? -input[i] : input[i];
        if (value > max_abs)
            max_abs = value;
    }
    if (max_abs < 0.00001f)
        max_abs = 0.00001f;
    float scale = 127.0f / max_abs;
    for (u32 i = 0U; i < count; i++) {
        float scaled = input[i] * scale;
        i32 quantized = scaled >= 0.0f ?
            (i32)(scaled + 0.5f) : (i32)(scaled - 0.5f);
        if (quantized < -128)
            quantized = -128;
        if (quantized > 127)
            quantized = 127;
        output[i] = (i8)quantized;
    }
    *scale_out = scale;
    return true;
}

bool bitnet_i2s_matvec_f32(const u8 *packed, u32 packed_bytes,
                           u32 rows, u32 cols,
                           const i8 *activation, u32 activation_count,
                           float activation_scale,
                           const float *weight_scales, u32 scale_count,
                           float *output, u32 output_count)
{
    i32 sums[BITNET_I2S_TILE_ROWS];
    if (!packed || !activation || !weight_scales || !output ||
        activation_scale <= 0.0f || scale_count == 0U ||
        rows > BITNET_I2S_TILE_ROWS || output_count < rows ||
        rows % scale_count != 0U)
        return false;
    if (!bitnet_i2s_matvec_i32(packed, packed_bytes, rows, cols,
                               activation, activation_count,
                               sums, BITNET_I2S_TILE_ROWS))
        return false;
    u32 rows_per_scale = rows / scale_count;
    for (u32 row = 0U; row < rows; row++)
        output[row] = ((float)sums[row] / activation_scale) *
                      weight_scales[row / rows_per_scale];
    return true;
}
