#pragma once

#include "types.h"

u32 bitnet_bitmap_row_stride(u32 cols);
i32 bitnet_bitmap_row_dot_scalar(const u8 *row, u32 cols, const i8 *act);
i32 bitnet_bitmap_row_dot_neon(const u8 *row, u32 cols, const i8 *act);
void bitnet_bitmap_matvec(const u8 *matrix, u32 rows, u32 cols,
                          const i8 *act, i32 *out, bool neon);
