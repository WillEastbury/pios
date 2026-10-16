#pragma once
#include "types.h"

/* Storage formats, not accelerator capability or model-execution selectors. */
enum tensor_format {
    TENSOR_FMT_F32, TENSOR_FMT_F16, TENSOR_FMT_I16, TENSOR_FMT_I8,
    TENSOR_FMT_I4, TENSOR_FMT_GGUF_Q4_0, TENSOR_FMT_GGUF_Q8_0
};
enum tensor_format_result {
    TENSOR_FORMAT_OK, TENSOR_FORMAT_INVALID, TENSOR_FORMAT_BOUNDS,
    TENSOR_FORMAT_DOMAIN
};
#define TENSOR_FORMAT_MAX_ELEMENTS 1048576U
#define TENSOR_FORMAT_MAX_RANK 8U

u16 tensor_f32_to_f16(float value);
float tensor_f16_to_f32(u16 bits);
enum tensor_format_result tensor_format_shape_bytes(
    enum tensor_format format, const u32 *dims, u32 rank,
    u32 *elements_out, u32 *bytes_out);
/* Integer real value = signed code * scale (finite, strictly positive).
 * Buffers must not overlap. All errors leave destination untouched.
 * Decode supports every listed format; encode only the five plain formats. */
enum tensor_format_result tensor_format_decode(
    enum tensor_format format, const u8 *src, u32 src_bytes,
    u32 count, float scale, float *dst, u32 dst_count);
enum tensor_format_result tensor_format_encode(
    enum tensor_format format, const float *src, u32 src_count,
    u32 count, float scale, u8 *dst, u32 dst_bytes);
