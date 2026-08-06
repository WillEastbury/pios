#include "picocompress.h"

#define PC_TOKEN_LITERAL_MAX 128U
#define PC_TOKEN_MATCH_MIN   3U
#define PC_TOKEN_MATCH_MAX   10U
#define PC_TOKEN_OFFSET_MAX  4095U

struct pc_buf_writer {
    u8 *data;
    usize cap;
    usize len;
};

static void pc_wr16(u8 *p, u16 v)
{
    p[0] = (u8)v;
    p[1] = (u8)(v >> 8);
}

static u16 pc_rd16(const u8 *p)
{
    return (u16)((u16)p[0] | ((u16)p[1] << 8));
}

static int pc_write_buffer(void *user, const u8 *data, usize len)
{
    struct pc_buf_writer *w = (struct pc_buf_writer *)user;
    if (!w || !data || w->len > w->cap || len > w->cap - w->len)
        return 1;
    memcpy(w->data + w->len, data, len);
    w->len += len;
    return 0;
}

static void pc_stat_block(pc_encoder *enc, u32 in, u32 out)
{
    enc->stats.bytes_in += in;
    enc->stats.bytes_out += out;
    enc->stats.blocks++;
}

void pc_encoder_init(pc_encoder *enc)
{
    if (!enc) return;
    memset(enc, 0, sizeof(*enc));
}

void pc_decoder_init(pc_decoder *dec)
{
    if (!dec) return;
    memset(dec, 0, sizeof(*dec));
}

void pc_encoder_get_stats(const pc_encoder *enc, pc_encoder_stats *out)
{
    if (!out) return;
    if (!enc) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = enc->stats;
}

usize pc_compress_bound(usize input_len)
{
    usize blocks = (input_len + PC_BLOCK_SIZE - 1U) / PC_BLOCK_SIZE;
    return input_len + blocks * 4U + input_len / 16U + 32U;
}

static u16 pc_find_match(const u8 *in, u16 pos, u16 len, u16 *out_off)
{
    u16 best_len = 0;
    u16 best_off = 0;
    u16 start = pos > PC_TOKEN_OFFSET_MAX ? (u16)(pos - PC_TOKEN_OFFSET_MAX) : 0;
    if (pos + PC_TOKEN_MATCH_MIN > len)
        return 0;
    for (u16 prev = start; prev + PC_TOKEN_MATCH_MIN <= pos; prev++) {
        if (in[prev] != in[pos] || in[prev + 1U] != in[pos + 1U] ||
            in[prev + 2U] != in[pos + 2U])
            continue;
        u16 m = PC_TOKEN_MATCH_MIN;
        while (m < PC_TOKEN_MATCH_MAX && pos + m < len && in[prev + m] == in[pos + m])
            m++;
        if (m > best_len) {
            best_len = m;
            best_off = (u16)(pos - prev);
            if (m == PC_TOKEN_MATCH_MAX)
                break;
        }
    }
    *out_off = best_off;
    return best_len;
}

static bool pc_emit_literal(const u8 *in, u16 start, u16 len, u8 *out, u16 out_cap, u16 *op)
{
    while (len) {
        u16 chunk = len > PC_TOKEN_LITERAL_MAX ? PC_TOKEN_LITERAL_MAX : len;
        if ((u32)*op + 1U + chunk > out_cap)
            return false;
        out[(*op)++] = (u8)(chunk - 1U);
        memcpy(out + *op, in + start, chunk);
        *op = (u16)(*op + chunk);
        start = (u16)(start + chunk);
        len = (u16)(len - chunk);
    }
    return true;
}

static pc_result pc_flush_block(pc_encoder *enc, pc_write_fn write_fn, void *user)
{
    u8 comp[PC_BLOCK_MAX_COMPRESSED];
    u8 frame[4 + PC_BLOCK_MAX_COMPRESSED];
    u16 ip = 0, op = 0, lit_start = 0, lit_len = 0;
    u16 raw_len;
    if (!enc || !write_fn)
        return PC_ERR_INPUT;
    raw_len = enc->block_len;
    if (raw_len == 0)
        return PC_OK;

    while (ip < raw_len) {
        u16 off = 0;
        u16 m = pc_find_match(enc->block, ip, raw_len, &off);
        if (m >= PC_TOKEN_MATCH_MIN && off > 0) {
            if (lit_len) {
                if (!pc_emit_literal(enc->block, lit_start, lit_len, comp, sizeof(comp), &op))
                    goto raw;
                enc->stats.literal_bytes += lit_len;
                lit_len = 0;
            }
            if ((u32)op + 2U > sizeof(comp))
                goto raw;
            comp[op++] = (u8)(0x80U | ((m - PC_TOKEN_MATCH_MIN) << 4U) | ((off >> 8) & 0x0FU));
            comp[op++] = (u8)off;
            enc->stats.match_count++;
            if (off <= 255U) enc->stats.lz_short_hits++;
            else enc->stats.lz_long_hits++;
            ip = (u16)(ip + m);
            lit_start = ip;
        } else {
            if (lit_len == 0)
                lit_start = ip;
            lit_len++;
            ip++;
            if (lit_len == PC_TOKEN_LITERAL_MAX) {
                if (!pc_emit_literal(enc->block, lit_start, lit_len, comp, sizeof(comp), &op))
                    goto raw;
                enc->stats.literal_bytes += lit_len;
                lit_len = 0;
                lit_start = ip;
            }
        }
    }
    if (lit_len) {
        if (!pc_emit_literal(enc->block, lit_start, lit_len, comp, sizeof(comp), &op))
            goto raw;
        enc->stats.literal_bytes += lit_len;
    }

    if (op < raw_len) {
        pc_wr16(frame, raw_len);
        pc_wr16(frame + 2, op);
        memcpy(frame + 4, comp, op);
        if (write_fn(user, frame, (usize)op + 4U) != 0)
            return PC_ERR_WRITE;
        pc_stat_block(enc, raw_len, (u32)op + 4U);
        enc->block_len = 0;
        return PC_OK;
    }

raw:
    pc_wr16(frame, raw_len);
    pc_wr16(frame + 2, 0);
    memcpy(frame + 4, enc->block, raw_len);
    if (write_fn(user, frame, (usize)raw_len + 4U) != 0)
        return PC_ERR_WRITE;
    pc_stat_block(enc, raw_len, (u32)raw_len + 4U);
    enc->stats.literal_bytes += raw_len;
    enc->block_len = 0;
    return PC_OK;
}

pc_result pc_encoder_sink(pc_encoder *enc, const u8 *data, usize len,
                          pc_write_fn write_fn, void *user)
{
    if (!enc || (!data && len) || !write_fn)
        return PC_ERR_INPUT;
    while (len) {
        u16 space = (u16)(PC_BLOCK_SIZE - enc->block_len);
        u16 n = len > space ? space : (u16)len;
        memcpy(enc->block + enc->block_len, data, n);
        enc->block_len = (u16)(enc->block_len + n);
        data += n;
        len -= n;
        if (enc->block_len == PC_BLOCK_SIZE) {
            pc_result r = pc_flush_block(enc, write_fn, user);
            if (r != PC_OK)
                return r;
        }
    }
    return PC_OK;
}

pc_result pc_encoder_finish(pc_encoder *enc, pc_write_fn write_fn, void *user)
{
    return pc_flush_block(enc, write_fn, user);
}

static pc_result pc_decode_frame(pc_decoder *dec, pc_write_fn write_fn, void *user)
{
    u8 raw[PC_BLOCK_SIZE];
    u16 ip = 0, op = 0;
    if (!dec || !write_fn || dec->raw_len > PC_BLOCK_SIZE)
        return PC_ERR_CORRUPT;
    if (dec->raw_frame) {
        if (write_fn(user, dec->payload, dec->raw_len) != 0)
            return PC_ERR_WRITE;
        return PC_OK;
    }
    while (ip < dec->comp_len) {
        u8 token = dec->payload[ip++];
        if ((token & 0x80U) == 0) {
            u16 n = (u16)((token & 0x7FU) + 1U);
            if (ip + n > dec->comp_len || op + n > dec->raw_len)
                return PC_ERR_CORRUPT;
            memcpy(raw + op, dec->payload + ip, n);
            ip = (u16)(ip + n);
            op = (u16)(op + n);
        } else {
            if (ip >= dec->comp_len)
                return PC_ERR_CORRUPT;
            u16 len = (u16)(((token >> 4U) & 0x07U) + PC_TOKEN_MATCH_MIN);
            u16 off = (u16)(((u16)(token & 0x0FU) << 8U) | dec->payload[ip++]);
            if (off == 0 || off > op || op + len > dec->raw_len)
                return PC_ERR_CORRUPT;
            for (u16 i = 0; i < len; i++) {
                raw[op] = raw[op - off];
                op++;
            }
        }
    }
    if (op != dec->raw_len)
        return PC_ERR_CORRUPT;
    if (write_fn(user, raw, op) != 0)
        return PC_ERR_WRITE;
    return PC_OK;
}

pc_result pc_decoder_sink(pc_decoder *dec, const u8 *data, usize len,
                          pc_write_fn write_fn, void *user)
{
    if (!dec || (!data && len) || !write_fn)
        return PC_ERR_INPUT;
    while (len) {
        if (dec->header_len < 4U) {
            dec->header[dec->header_len++] = *data++;
            len--;
            if (dec->header_len == 4U) {
                dec->raw_len = pc_rd16(dec->header);
                dec->comp_len = pc_rd16(dec->header + 2);
                dec->payload_len = 0;
                dec->raw_frame = dec->comp_len == 0;
                if (dec->raw_len > PC_BLOCK_SIZE || dec->comp_len > PC_BLOCK_MAX_COMPRESSED)
                    return PC_ERR_CORRUPT;
                if (dec->raw_frame)
                    dec->comp_len = dec->raw_len;
            }
            continue;
        }
        u16 need = (u16)(dec->comp_len - dec->payload_len);
        u16 n = len > need ? need : (u16)len;
        memcpy(dec->payload + dec->payload_len, data, n);
        dec->payload_len = (u16)(dec->payload_len + n);
        data += n;
        len -= n;
        if (dec->payload_len == dec->comp_len) {
            pc_result r = pc_decode_frame(dec, write_fn, user);
            if (r != PC_OK)
                return r;
            dec->header_len = 0;
            dec->raw_len = dec->comp_len = dec->payload_len = 0;
            dec->raw_frame = false;
        }
    }
    return PC_OK;
}

pc_result pc_decoder_finish(pc_decoder *dec)
{
    if (!dec)
        return PC_ERR_INPUT;
    return dec->header_len == 0 ? PC_OK : PC_ERR_INPUT;
}

pc_result pc_compress_buffer(const u8 *input, usize input_len,
                             u8 *output, usize output_cap,
                             usize *output_len)
{
    pc_encoder enc;
    struct pc_buf_writer w = { output, output_cap, 0 };
    pc_result r;
    if ((!input && input_len) || !output || !output_len)
        return PC_ERR_INPUT;
    pc_encoder_init(&enc);
    r = pc_encoder_sink(&enc, input, input_len, pc_write_buffer, &w);
    if (r == PC_OK)
        r = pc_encoder_finish(&enc, pc_write_buffer, &w);
    if (r == PC_OK)
        *output_len = w.len;
    return r;
}

pc_result pc_decompress_buffer(const u8 *input, usize input_len,
                               u8 *output, usize output_cap,
                               usize *output_len)
{
    pc_decoder dec;
    struct pc_buf_writer w = { output, output_cap, 0 };
    pc_result r;
    if ((!input && input_len) || !output || !output_len)
        return PC_ERR_INPUT;
    pc_decoder_init(&dec);
    r = pc_decoder_sink(&dec, input, input_len, pc_write_buffer, &w);
    if (r == PC_OK)
        r = pc_decoder_finish(&dec);
    if (r == PC_OK)
        *output_len = w.len;
    return r;
}

bool pc_selftest(void)
{
    static const u8 pat[] = "AACCBBDDAACCBBDD1122334455667788";
    u8 input[1024];
    u8 compressed[1400];
    u8 restored[sizeof(input)];
    usize clen = 0, rlen = 0;
    pc_encoder enc;
    pc_encoder_stats st;
    struct pc_buf_writer w = { compressed, sizeof(compressed), 0 };
    for (u32 i = 0; i < sizeof(input); i++)
        input[i] = pat[i % (sizeof(pat) - 1U)];
    pc_encoder_init(&enc);
    if (pc_encoder_sink(&enc, input, sizeof(input), pc_write_buffer, &w) != PC_OK)
        return false;
    if (pc_encoder_finish(&enc, pc_write_buffer, &w) != PC_OK)
        return false;
    pc_encoder_get_stats(&enc, &st);
    clen = w.len;
    if (st.bytes_in != sizeof(input) || st.blocks == 0 || st.match_count == 0)
        return false;
    if (pc_decompress_buffer(compressed, clen, restored, sizeof(restored), &rlen) != PC_OK)
        return false;
    return rlen == sizeof(input) && memcmp(input, restored, sizeof(input)) == 0;
}
