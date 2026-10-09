#pragma once

#include "types.h"

u32 bitnet_bitmap_row_stride(u32 cols);
i32 bitnet_bitmap_row_dot_scalar(const u8 *row, u32 cols, const i8 *act);
i32 bitnet_bitmap_row_dot_neon(const u8 *row, u32 cols, const i8 *act);
void bitnet_bitmap_matvec(const u8 *matrix, u32 rows, u32 cols,
                          const i8 *act, i32 *out, bool neon);

/*
 * Microsoft BitNet GPU W2A8 layout: 16x32 weight tiles, ternary values encoded
 * as int2 code = weight + 2, then packed/interleaved for four-lane decode.
 * Contract pinned to microsoft/BitNet commit 0b341e582afbf9e1011f24744b554c96a3477eb5.
 */
#define BITNET_I2S_TILE_ROWS 16U
#define BITNET_I2S_TILE_COLS 32U

bool bitnet_i2s_packed_bytes(u32 rows, u32 cols, u32 *bytes_out);
bool bitnet_i2s_pack_microsoft(const i8 *weights, u32 weight_count,
                               u32 rows, u32 cols,
                               u8 *packed, u32 packed_capacity);
bool bitnet_i2s_matvec_i32(const u8 *packed, u32 packed_bytes,
                           u32 rows, u32 cols,
                           const i8 *activation, u32 activation_count,
                           i32 *output, u32 output_count);
bool bitnet_i2s_quantize_activation(const float *input, u32 count,
                                    i8 *output, u32 output_count,
                                    float *scale_out);
bool bitnet_i2s_matvec_f32(const u8 *packed, u32 packed_bytes,
                           u32 rows, u32 cols,
                           const i8 *activation, u32 activation_count,
                           float activation_scale,
                           const float *weight_scales, u32 scale_count,
                           float *output, u32 output_count);
