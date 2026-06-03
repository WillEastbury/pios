#include "brotli.h"
#include "simd.h"

struct br_len_code { u32 base; int extra; };

static const u8 kCLOrder[18] = {
    1, 2, 3, 4, 0, 5, 17, 6, 16, 7, 8, 9, 10, 11, 12, 13, 14, 15
};
static const u8 kCLCL_val[6] = {0, 7, 3, 2, 1, 15};
static const u8 kCLCL_len[6] = {2, 4, 3, 2, 2, 4};
static const struct br_len_code kInsLen[24] = {
    {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,1},{8,1},
    {10,2},{14,2},{18,3},{26,3},{34,4},{50,4},{66,5},{98,5},
    {130,6},{194,7},{322,8},{578,9},{1090,10},{2114,12},
    {6210,14},{22594,24}
};
static const struct br_len_code kCopyLen[24] = {
    {2,0},{3,0},{4,0},{5,0},{6,0},{7,0},{8,0},{9,0},
    {10,1},{12,1},{14,2},{18,2},{22,3},{30,3},{38,4},{54,4},
    {70,5},{102,5},{134,6},{198,7},{326,8},{582,9},
    {1094,10},{2118,24}
};

typedef struct {
    u8 *buf;
    usize cap;
    usize pos;
    u64 accum;
    int nbits;
} bitw_t;

typedef struct {
    const u8 *p;
    usize len;
    usize bit;
} bitr_t;

typedef struct {
    u8 len[704];
    u16 code[704];
    int nsym;
    int single_symbol;
} hdec_t;

static void bw_init(bitw_t *w, u8 *buf, usize cap)
{
    w->buf = buf; w->cap = cap; w->pos = 0; w->accum = 0; w->nbits = 0;
}

static void bw_flush(bitw_t *w)
{
    while (w->nbits >= 8 && w->pos < w->cap) {
        w->buf[w->pos++] = (u8)(w->accum & 0xFF);
        w->accum >>= 8;
        w->nbits -= 8;
    }
}

static void bw_put(bitw_t *w, u64 val, int nbits)
{
    w->accum |= val << w->nbits;
    w->nbits += nbits;
    if (w->nbits >= 48) bw_flush(w);
}

static bool bw_finish(bitw_t *w)
{
    bw_flush(w);
    if (w->nbits > 0) {
        if (w->pos >= w->cap) return false;
        w->buf[w->pos++] = (u8)(w->accum & 0xFF);
        w->accum = 0;
        w->nbits = 0;
    }
    return true;
}

usize brotli_bound(usize input_len)
{
    return input_len + input_len / 64 + 64;
}

i32 brotli_encode(const u8 *input, usize input_len, u8 *output, usize output_cap)
{
    bitw_t w;
    if (!output || (!input && input_len)) return -1;
    if (input_len == 0) {
        if (output_cap < 1) return -1;
        output[0] = 0x06;
        return 1;
    }
    if (input_len > 0xFFFFFFu) return -1;
    bw_init(&w, output, output_cap);
    bw_put(&w, 0, 1); /* WBITS=16 */
    bw_put(&w, 0, 1); /* ISLAST=0, because uncompressed blocks need a final empty block */
    u32 mlen = (u32)(input_len - 1);
    int mn = (mlen < (1u << 16)) ? 4 : (mlen < (1u << 20)) ? 5 : 6;
    bw_put(&w, (u32)(mn - 4), 2);
    bw_put(&w, mlen, mn * 4);
    bw_put(&w, 1, 1); /* ISUNCOMPRESSED */
    if (w.nbits > 0) {
        int pad = 8 - (w.nbits % 8);
        if (pad < 8) bw_put(&w, 0, pad);
    }
    if (!bw_finish(&w)) return -1;
    if (w.pos + input_len + 1 > output_cap) return -1;
    simd_memcpy(output + w.pos, input, input_len);
    w.pos += input_len;
    bw_put(&w, 1, 1); /* ISLAST */
    bw_put(&w, 1, 1); /* ISLASTEMPTY */
    if (!bw_finish(&w)) return -1;
    return (i32)w.pos;
}

static int br_read(bitr_t *r, int n, u32 *out)
{
    u32 v = 0;
    if (n < 0 || n > 24 || r->bit + (usize)n > r->len * 8) return -1;
    for (int i = 0; i < n; i++) {
        usize bi = r->bit++;
        v |= (u32)(((r->p[bi >> 3] >> (bi & 7)) & 1U) << i);
    }
    *out = v;
    return 0;
}

static void br_align_byte(bitr_t *r)
{
    r->bit = (r->bit + 7u) & ~(usize)7u;
}

static u16 bit_reverse(u16 v, int n)
{
    u16 r = 0;
    for (int i = 0; i < n; i++) {
        r = (u16)((r << 1) | (v & 1U));
        v >>= 1;
    }
    return r;
}

static int hdec_build(hdec_t *h, const u8 *lens, int nsym)
{
    int bl_count[16];
    u16 next[16];
    simd_zero(h, sizeof(*h));
    h->single_symbol = -1;
    h->nsym = nsym;
    simd_zero(bl_count, sizeof(bl_count));
    for (int i = 0; i < nsym; i++) {
        if (lens[i] > 15) return -1;
        h->len[i] = lens[i];
        if (lens[i]) bl_count[lens[i]]++;
    }
    u16 c = 0;
    next[0] = 0;
    for (int b = 1; b <= 15; b++) {
        c = (u16)((c + bl_count[b - 1]) << 1);
        next[b] = c;
    }
    for (int i = 0; i < nsym; i++) {
        if (h->len[i]) {
            u16 canon = next[h->len[i]]++;
            h->code[i] = bit_reverse(canon, h->len[i]);
        }
    }
    return 0;
}

static int hdec_symbol(bitr_t *r, const hdec_t *h)
{
    u32 bit;
    u16 code = 0;
    if (h->single_symbol >= 0) return h->single_symbol;
    for (int len = 1; len <= 15; len++) {
        if (br_read(r, 1, &bit) != 0) return -1;
        code |= (u16)(bit << (len - 1));
        for (int s = 0; s < h->nsym; s++) {
            if (h->len[s] == len && h->code[s] == code) return s;
        }
    }
    return -1;
}

static int read_clcl_symbol(bitr_t *r)
{
    u32 bit;
    u16 code = 0;
    for (int len = 1; len <= 4; len++) {
        if (br_read(r, 1, &bit) != 0) return -1;
        code |= (u16)(bit << (len - 1));
        for (int v = 0; v <= 5; v++) {
            if (kCLCL_len[v] == len && kCLCL_val[v] == code) return v;
        }
    }
    return -1;
}

static int read_prefix_code(bitr_t *r, hdec_t *h, int nsym, int alpha_bits)
{
    u32 hskip;
    u8 lens[704];
    simd_zero(lens, sizeof(lens));
    if (nsym > (int)sizeof(lens)) return -1;
    if (br_read(r, 2, &hskip) != 0) return -1;
    if (hskip == 1) {
        u32 nsym_m1;
        int used[4] = {0,0,0,0};
        if (br_read(r, 2, &nsym_m1) != 0) return -1;
        int n = (int)nsym_m1 + 1;
        for (int i = 0; i < n; i++) {
            u32 sym;
            if (br_read(r, alpha_bits, &sym) != 0 || sym >= (u32)nsym) return -1;
            used[i] = (int)sym;
        }
        if (n == 1) {
            if (hdec_build(h, lens, nsym) != 0) return -1;
            h->single_symbol = used[0];
            return 0;
        } else if (n == 2) {
            lens[used[0]] = 1; lens[used[1]] = 1;
        } else if (n == 3) {
            lens[used[0]] = 1; lens[used[1]] = 2; lens[used[2]] = 2;
        } else {
            u32 tree_sel;
            if (br_read(r, 1, &tree_sel) != 0) return -1;
            if (tree_sel) {
                lens[used[0]] = 1; lens[used[1]] = 2; lens[used[2]] = 3; lens[used[3]] = 3;
            } else {
                lens[used[0]] = 2; lens[used[1]] = 2; lens[used[2]] = 2; lens[used[3]] = 2;
            }
        }
        return hdec_build(h, lens, nsym);
    }
    if (hskip > 3) return -1;
    u8 cl_lens[18];
    simd_zero(cl_lens, sizeof(cl_lens));
    int space = 0;
    for (int i = (int)hskip; i < 18; i++) {
        int v = read_clcl_symbol(r);
        if (v < 0) return -1;
        cl_lens[kCLOrder[i]] = (u8)v;
        if (v) space += 1 << (5 - v);
        if (i + 1 >= 4 && space == 32) break;
        if (space > 32) return -1;
    }
    hdec_t clh;
    if (hdec_build(&clh, cl_lens, 18) != 0) return -1;
    int pos = 0;
    int code_space = 0;
    while (pos < nsym && code_space < (1 << 15)) {
        int sym = hdec_symbol(r, &clh);
        if (sym < 0) return -1;
        if (sym == 17) {
            u32 extra;
            if (br_read(r, 3, &extra) != 0) return -1;
            int run = 3 + (int)extra;
            if (pos + run > nsym) return -1;
            pos += run;
        } else if (sym <= 15) {
            lens[pos++] = (u8)sym;
            if (sym) {
                code_space += 1 << (15 - sym);
                if (code_space > (1 << 15)) return -1;
            }
        } else return -1;
    }
    if (code_space != (1 << 15)) return -1;
    return hdec_build(h, lens, nsym);
}

static int decode_ic_symbol(int sym, int *ic, int *cc, bool *explicit_dist)
{
    int base;
    *explicit_dist = true;
    if (sym < 0 || sym >= 704) return -1;
    if (sym < 64) { *explicit_dist = false; base = sym; *ic = base >> 3; *cc = base & 7; return 0; }
    if (sym < 128) { *explicit_dist = false; base = sym - 64; *ic = base >> 3; *cc = 8 + (base & 7); return 0; }
    if (sym < 192) { base = sym - 128; *ic = base >> 3; *cc = base & 7; return 0; }
    if (sym < 256) { base = sym - 192; *ic = base >> 3; *cc = 8 + (base & 7); return 0; }
    if (sym < 320) { base = sym - 256; *ic = 8 + (base >> 3); *cc = base & 7; return 0; }
    if (sym < 384) { base = sym - 320; *ic = 8 + (base >> 3); *cc = 8 + (base & 7); return 0; }
    if (sym < 448) { base = sym - 384; *ic = base >> 3; *cc = 16 + (base & 7); return 0; }
    if (sym < 512) { base = sym - 448; *ic = 16 + (base >> 3); *cc = base & 7; return 0; }
    if (sym < 576) { base = sym - 512; *ic = 8 + (base >> 3); *cc = 16 + (base & 7); return 0; }
    if (sym < 640) { base = sym - 576; *ic = 16 + (base >> 3); *cc = 8 + (base & 7); return 0; }
    base = sym - 640; *ic = 16 + (base >> 3); *cc = 16 + (base & 7); return 0;
}

static int decode_distance(bitr_t *r, int dc, u32 *dist)
{
    if (dc < 16 || dc >= 64) return -1;
    int hcode = dc - 16;
    int nb = 1 + (hcode >> 1);
    u32 extra;
    u32 off = ((u32)(2 + (hcode & 1)) << nb) - 4;
    if (br_read(r, nb, &extra) != 0) return -1;
    *dist = off + extra + 1;
    return 0;
}

static int copy_match(u8 *out, usize out_cap, usize *pos, u32 dist, u32 len)
{
    if (dist == 0 || dist > *pos || *pos + len > out_cap) return -1;
    usize src = *pos - dist;
    if (dist >= len) {
        simd_memcpy(out + *pos, out + src, len);
        *pos += len;
        return 0;
    }
    for (u32 i = 0; i < len; i++) out[(*pos)++] = out[src + i];
    return 0;
}

static int decode_compressed_meta(bitr_t *r, u32 mlen, u8 *out, usize out_cap, usize *out_pos)
{
    u32 v;
    hdec_t lit_h, ic_h, dist_h;
    if (br_read(r, 1, &v) != 0 || v != 0) return -1;
    if (br_read(r, 1, &v) != 0 || v != 0) return -1;
    if (br_read(r, 1, &v) != 0 || v != 0) return -1;
    if (br_read(r, 2, &v) != 0 || v != 0) return -1;
    if (br_read(r, 4, &v) != 0 || v != 0) return -1;
    if (br_read(r, 2, &v) != 0 || v != 0) return -1;
    if (br_read(r, 1, &v) != 0 || v != 0) return -1;
    if (br_read(r, 1, &v) != 0 || v != 0) return -1;
    if (read_prefix_code(r, &lit_h, 256, 8) != 0) return -1;
    if (read_prefix_code(r, &ic_h, 704, 10) != 0) return -1;
    if (read_prefix_code(r, &dist_h, 64, 6) != 0) return -1;
    usize end = *out_pos + mlen;
    if (end > out_cap || end < *out_pos) return -1;
    while (*out_pos < end) {
        int sym = hdec_symbol(r, &ic_h);
        int ic, cc;
        bool explicit_dist;
        u32 extra, ins_len, copy_len = 0, dist = 0;
        if (decode_ic_symbol(sym, &ic, &cc, &explicit_dist) != 0) return -1;
        if (br_read(r, kInsLen[ic].extra, &extra) != 0) return -1;
        ins_len = kInsLen[ic].base + extra;
        if (explicit_dist) {
            if (br_read(r, kCopyLen[cc].extra, &extra) != 0) return -1;
            copy_len = kCopyLen[cc].base + extra;
        }
        if (*out_pos + ins_len > end) return -1;
        for (u32 i = 0; i < ins_len; i++) {
            int lit = hdec_symbol(r, &lit_h);
            if (lit < 0 || lit > 255) return -1;
            out[(*out_pos)++] = (u8)lit;
        }
        if (explicit_dist) {
            int dc = hdec_symbol(r, &dist_h);
            if (decode_distance(r, dc, &dist) != 0) return -1;
            if (copy_match(out, out_cap, out_pos, dist, copy_len) != 0) return -1;
            if (*out_pos > end) return -1;
        }
    }
    return *out_pos == end ? 0 : -1;
}

i32 brotli_decode(const u8 *input, usize input_len, u8 *output, usize output_cap)
{
    bitr_t r = { input, input_len, 0 };
    usize out_pos = 0;
    u32 v;
    if (!input || (!output && output_cap)) return -1;
    if (br_read(&r, 1, &v) != 0 || v != 0) return -1;
    for (;;) {
        u32 islast, islastempty = 0, mn, mlen_m1;
        if (br_read(&r, 1, &islast) != 0) return -1;
        if (islast) {
            if (br_read(&r, 1, &islastempty) != 0) return -1;
            if (islastempty) return (i32)out_pos;
        }
        if (br_read(&r, 2, &mn) != 0 || mn > 2) return -1;
        int nibbles = 4 + (int)mn;
        if (br_read(&r, nibbles * 4, &mlen_m1) != 0) return -1;
        u32 mlen = mlen_m1 + 1;
        if (!islast) {
            u32 isuncompressed;
            if (br_read(&r, 1, &isuncompressed) != 0) return -1;
            if (isuncompressed) {
                br_align_byte(&r);
                if ((r.bit >> 3) + mlen > input_len || out_pos + mlen > output_cap) return -1;
                simd_memcpy(output + out_pos, input + (r.bit >> 3), mlen);
                r.bit += (usize)mlen * 8u;
                out_pos += mlen;
                continue;
            }
        }
        if (decode_compressed_meta(&r, mlen, output, output_cap, &out_pos) != 0) return -1;
        if (islast) return (i32)out_pos;
    }
}

bool brotli_selftest(void)
{
    static const u8 sample[] = "PIOS-brotli-stored-selftest";
    static const u8 fixture[38] = {
        0xE2,0x3F,0x00,0x00,0xE8,0x0E,0x7F,0xBF,0xDF,0xCF,0x7E,0xBF,0xAB,0xAB,0xE8,0x77,
        0x44,0x11,0x11,0x29,0xAA,0xC1,0x26,0x0C,0xAA,0x78,0x42,0xAC,0xB0,0xE9,0xAD,0x44,
        0xE4,0xEF,0x8D,0xE3,0x48,0x07
    };
    u8 enc[128];
    u8 dec[512];
    i32 n = brotli_encode(sample, sizeof(sample) - 1, enc, sizeof(enc));
    if (n <= 0) return false;
    i32 d = brotli_decode(enc, (usize)n, dec, sizeof(dec));
    if (d != (i32)(sizeof(sample) - 1) || memcmp(dec, sample, sizeof(sample) - 1) != 0) return false;
    d = brotli_decode(fixture, sizeof(fixture), dec, sizeof(dec));
    if (d != 512) return false;
    for (u32 i = 0; i < 512; i++) {
        static const u8 pat[] = "PIOS-brotli-fixture-";
        if (dec[i] != pat[i % (sizeof(pat) - 1)]) return false;
    }
    return true;
}
