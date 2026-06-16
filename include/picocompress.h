#pragma once
#include "types.h"

#ifndef PC_BLOCK_SIZE
#define PC_BLOCK_SIZE 508U
#endif

#define PC_BLOCK_MAX_COMPRESSED (PC_BLOCK_SIZE + 16U)

typedef enum pc_result {
    PC_OK = 0,
    PC_ERR_WRITE = -1,
    PC_ERR_INPUT = -2,
    PC_ERR_CORRUPT = -3,
    PC_ERR_OUTPUT_TOO_SMALL = -4
} pc_result;

typedef int (*pc_write_fn)(void *user, const u8 *data, usize len);

typedef struct pc_encoder_stats {
    u32 bytes_in;
    u32 bytes_out;
    u32 blocks;
    u32 literal_bytes;
    u32 match_count;
    u32 repeat_hits;
    u32 dict_hits;
    u32 lz_short_hits;
    u32 lz_long_hits;
    u32 good_enough_hits;
    u32 lazy_improvements;
} pc_encoder_stats;

typedef struct pc_encoder {
    u8 block[PC_BLOCK_SIZE];
    u16 block_len;
    pc_encoder_stats stats;
} pc_encoder;

typedef struct pc_decoder {
    u8 header[4];
    u8 header_len;
    u16 raw_len;
    u16 comp_len;
    u16 payload_len;
    bool raw_frame;
    u8 payload[PC_BLOCK_MAX_COMPRESSED];
} pc_decoder;

void pc_encoder_init(pc_encoder *enc);
pc_result pc_encoder_sink(pc_encoder *enc, const u8 *data, usize len,
                          pc_write_fn write_fn, void *user);
pc_result pc_encoder_finish(pc_encoder *enc, pc_write_fn write_fn, void *user);
void pc_encoder_get_stats(const pc_encoder *enc, pc_encoder_stats *out);

void pc_decoder_init(pc_decoder *dec);
pc_result pc_decoder_sink(pc_decoder *dec, const u8 *data, usize len,
                          pc_write_fn write_fn, void *user);
pc_result pc_decoder_finish(pc_decoder *dec);

usize pc_compress_bound(usize input_len);
pc_result pc_compress_buffer(const u8 *input, usize input_len,
                             u8 *output, usize output_cap,
                             usize *output_len);
pc_result pc_decompress_buffer(const u8 *input, usize input_len,
                               u8 *output, usize output_cap,
                               usize *output_len);
bool pc_selftest(void);
