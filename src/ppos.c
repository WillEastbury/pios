#include "ppos.h"

#define PPOS_READ_MAX_VARINT_BYTES 5U

struct ppos_cursor {
    const u8 *p;
    const u8 *end;
};

struct ppos_term {
    const u8 *ptr;
    u32 len;
};

static u32 ppos_get_le16(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8);
}

static u32 ppos_get_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void ppos_put_le16(u8 *p, u32 v)
{
    p[0] = (u8)v;
    p[1] = (u8)(v >> 8);
}

static void ppos_put_le32(u8 *p, u32 v)
{
    p[0] = (u8)v;
    p[1] = (u8)(v >> 8);
    p[2] = (u8)(v >> 16);
    p[3] = (u8)(v >> 24);
}

u32 ppos_crc32(const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    u32 crc = 0xFFFFFFFFU;
    while (len--) {
        crc ^= *p++;
        for (u32 bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320U & (u32)-(i32)(crc & 1U));
    }
    return ~crc;
}

static bool ppos_utf8(const u8 *s, u32 n)
{
    u32 i = 0;
    while (i < n) {
        u8 c = s[i++];
        if (c < 0x80U) continue;
        if (c < 0xC2U) return false;
        if (c < 0xE0U) {
            if (i >= n || (s[i] & 0xC0U) != 0x80U) return false;
            i++;
        } else if (c < 0xF0U) {
            if (i + 1U >= n || (s[i] & 0xC0U) != 0x80U ||
                (s[i + 1U] & 0xC0U) != 0x80U) return false;
            if (c == 0xE0U && s[i] < 0xA0U) return false;
            if (c == 0xEDU && s[i] >= 0xA0U) return false;
            i += 2U;
        } else if (c < 0xF5U) {
            if (i + 2U >= n || (s[i] & 0xC0U) != 0x80U ||
                (s[i + 1U] & 0xC0U) != 0x80U ||
                (s[i + 2U] & 0xC0U) != 0x80U) return false;
            if (c == 0xF0U && s[i] < 0x90U) return false;
            if (c == 0xF4U && s[i] >= 0x90U) return false;
            i += 3U;
        } else {
            return false;
        }
    }
    return true;
}

static int ppos_cmp(const u8 *a, u32 an, const u8 *b, u32 bn)
{
    u32 n = an < bn ? an : bn;
    for (u32 i = 0; i < n; i++) {
        if (a[i] < b[i]) return -1;
        if (a[i] > b[i]) return 1;
    }
    return an < bn ? -1 : an > bn ? 1 : 0;
}

static i32 ppos_read_varint(struct ppos_cursor *c, u32 *out)
{
    u32 value = 0;
    for (u32 i = 0; i < PPOS_READ_MAX_VARINT_BYTES; i++) {
        if (c->p >= c->end) return PPOS_CORRUPT;
        u8 b = *c->p++;
        if (i == 4U && b > 0x0FU) return PPOS_CORRUPT;
        value |= (u32)(b & 0x7FU) << (i * 7U);
        if (!(b & 0x80U)) {
            *out = value;
            return PPOS_OK;
        }
    }
    return PPOS_CORRUPT;
}

static i32 ppos_write_byte(u8 *body, u32 *used, u32 cap, u8 b)
{
    if (*used >= cap) return PPOS_FULL;
    body[(*used)++] = b;
    return PPOS_OK;
}

static i32 ppos_write_bytes(u8 *body, u32 *used, u32 cap,
                            const u8 *src, u32 len)
{
    if (len > cap - *used) return PPOS_FULL;
    memcpy(body + *used, src, len);
    *used += len;
    return PPOS_OK;
}

static i32 ppos_write_varint(u8 *body, u32 *used, u32 cap, u32 value)
{
    do {
        u8 b = (u8)(value & 0x7FU);
        value >>= 7;
        if (value) b |= 0x80U;
        i32 rc = ppos_write_byte(body, used, cap, b);
        if (rc != PPOS_OK) return rc;
    } while (value);
    return PPOS_OK;
}

/*
 * Tokenization mirrors the reference's lower().replace("_", " ").split()
 * for the UTF-8 byte domain supported by freestanding PIOS: ASCII letters
 * are folded, non-ASCII UTF-8 is preserved and validated.
 */
static i32 ppos_next_token(const u8 *doc, u32 len, u32 *offset,
                            u8 *out, u32 *out_len)
{
    u32 i = *offset;
    while (i < len && (doc[i] <= 0x20U || doc[i] == '_')) i++;
    if (i == len) {
        *offset = i;
        return PPOS_NOT_FOUND;
    }
    u32 n = 0;
    while (i < len && doc[i] > 0x20U && doc[i] != '_') {
        if (n >= PPOS_MAX_TERM_BYTES) return PPOS_INVALID;
        u8 c = doc[i++];
        if (c >= 'A' && c <= 'Z') c = (u8)(c + ('a' - 'A'));
        out[n++] = c;
    }
    if (!ppos_utf8(out, n)) return PPOS_INVALID;
    *offset = i;
    *out_len = n;
    return PPOS_OK;
}

static i32 ppos_load_doc(const struct ppos_source *source, u32 id,
                         u8 *buf, u32 *len)
{
    i32 n = source->read(source->ctx, id, buf, source->document_buf_cap);
    if (n == PPOS_NOT_FOUND) return PPOS_NOT_FOUND;
    if (n < 0) return n;
    if ((u32)n > source->document_buf_cap) return PPOS_BUFFER_TOO_SMALL;
    *len = (u32)n;
    return PPOS_OK;
}

static i32 ppos_find_next_term(const struct ppos_source *source,
                               const u8 *last, u32 last_len,
                               bool have_last, u8 *candidate,
                               u32 *candidate_len)
{
    bool found = false;
    u8 token[PPOS_MAX_TERM_BYTES];
    for (u32 id = 0; id < source->document_limit; id++) {
        u32 len = 0;
        i32 rc = ppos_load_doc(source, id, source->document_buf, &len);
        if (rc == PPOS_NOT_FOUND) continue;
        if (rc != PPOS_OK) return rc;
        u32 off = 0;
        while (off < len) {
            u32 n = 0;
            rc = ppos_next_token(source->document_buf, len, &off, token, &n);
            if (rc == PPOS_NOT_FOUND) break;
            if (rc != PPOS_OK) return rc;
            if (have_last && ppos_cmp(token, n, last, last_len) <= 0)
                continue;
            if (!found || ppos_cmp(token, n, candidate, *candidate_len) < 0) {
                memcpy(candidate, token, n);
                *candidate_len = n;
                found = true;
            }
        }
    }
    return found ? PPOS_OK : PPOS_NOT_FOUND;
}

static i32 ppos_count_docs_for_term(const struct ppos_source *source,
                                    const u8 *term, u32 term_len,
                                    u32 *out_count)
{
    u8 token[PPOS_MAX_TERM_BYTES];
    u32 count = 0;
    for (u32 id = 0; id < source->document_limit; id++) {
        u32 len = 0;
        i32 rc = ppos_load_doc(source, id, source->document_buf, &len);
        if (rc == PPOS_NOT_FOUND) continue;
        if (rc != PPOS_OK) return rc;
        u32 off = 0;
        bool hit = false;
        while (off < len) {
            u32 n = 0;
            rc = ppos_next_token(source->document_buf, len, &off, token, &n);
            if (rc == PPOS_NOT_FOUND) break;
            if (rc != PPOS_OK) return rc;
            if (ppos_cmp(token, n, term, term_len) == 0) {
                hit = true;
                break;
            }
        }
        if (hit) count++;
    }
    *out_count = count;
    return PPOS_OK;
}

static i32 ppos_write_postings(const struct ppos_source *source,
                               const u8 *term, u32 term_len,
                               u8 *body, u32 *used, u32 cap)
{
    u8 token[PPOS_MAX_TERM_BYTES];
    u32 count = 0;
    i32 rc = ppos_count_docs_for_term(source, term, term_len, &count);
    if (rc != PPOS_OK) return rc;
    rc = ppos_write_varint(body, used, cap, count);
    if (rc != PPOS_OK) return rc;

    u32 last_doc = 0;
    for (u32 id = 0; id < source->document_limit; id++) {
        u32 len = 0;
        rc = ppos_load_doc(source, id, source->document_buf, &len);
        if (rc == PPOS_NOT_FOUND) continue;
        if (rc != PPOS_OK) return rc;
        u32 off = 0;
        u32 position_count = 0;
        bool hit = false;
        while (off < len) {
            u32 n = 0;
            rc = ppos_next_token(source->document_buf, len, &off, token, &n);
            if (rc == PPOS_NOT_FOUND) break;
            if (rc != PPOS_OK) return rc;
            if (ppos_cmp(token, n, term, term_len) == 0) {
                /*
                 * We only need the count for the first pass here.  Positions
                 * are emitted in a second token walk below, avoiding a
                 * per-document posting array.
                 */
                hit = true;
                position_count++;
            }
        }
        if (!hit) continue;
        rc = ppos_write_varint(body, used, cap, id - last_doc);
        if (rc != PPOS_OK) return rc;
        last_doc = id;
        rc = ppos_write_varint(body, used, cap, position_count);
        if (rc != PPOS_OK) return rc;
        off = 0;
        u32 last_position = 0;
        u32 position = 0;
        while (off < len) {
            u32 n = 0;
            rc = ppos_next_token(source->document_buf, len, &off, token, &n);
            if (rc == PPOS_NOT_FOUND) break;
            if (rc != PPOS_OK) return rc;
            if (ppos_cmp(token, n, term, term_len) != 0) {
                position++;
                continue;
            }
            rc = ppos_write_varint(body, used, cap, position - last_position);
            if (rc != PPOS_OK) return rc;
            last_position = position;
            position++;
        }
    }
    return PPOS_OK;
}

i32 ppos_build(u8 *out, u32 out_cap, u32 generation,
               const struct ppos_source *source, u32 *out_len)
{
    if (out_len) *out_len = 0;
    if (!out || !out_len || !source || !source->read ||
        !source->document_buf || !source->document_buf_cap ||
        !source->document_limit || source->document_limit > PPOS_MAX_DOCUMENTS ||
        out_cap < PPOS_HEADER_BYTES) return PPOS_INVALID;

    u8 *body = out + PPOS_HEADER_BYTES;
    u32 body_cap = out_cap - PPOS_HEADER_BYTES;
    if (body_cap > PPOS_BODY_MAX_BYTES) body_cap = PPOS_BODY_MAX_BYTES;
    u32 used = 0;
    u32 terms = 0;
    u8 last[PPOS_MAX_TERM_BYTES];
    u32 last_len = 0;
    bool have_last = false;

    for (;;) {
        u8 term[PPOS_MAX_TERM_BYTES];
        u32 term_len = 0;
        i32 rc = ppos_find_next_term(source, last, last_len, have_last,
                                      term, &term_len);
        if (rc == PPOS_NOT_FOUND) break;
        if (rc != PPOS_OK) return rc;
        if (terms >= PPOS_MAX_TERMS) return PPOS_FULL;
        rc = ppos_write_byte(body, &used, body_cap, (u8)term_len);
        if (rc != PPOS_OK) return rc;
        rc = ppos_write_bytes(body, &used, body_cap, term, term_len);
        if (rc != PPOS_OK) return rc;
        rc = ppos_write_postings(source, term, term_len, body, &used, body_cap);
        if (rc != PPOS_OK) return rc;
        memcpy(last, term, term_len);
        last_len = term_len;
        have_last = true;
        terms++;
    }

    ppos_put_le32(out, 0x534F5050U); /* little-endian "PPOS" */
    out[4] = (u8)PPOS_VERSION;
    out[5] = 0;
    ppos_put_le16(out + 6, 0);
    ppos_put_le32(out + 8, terms);
    ppos_put_le32(out + 12, ppos_crc32(body, used));
    ppos_put_le32(out + 16, generation);
    *out_len = PPOS_HEADER_BYTES + used;
    return PPOS_OK;
}

static i32 ppos_skip_postings(struct ppos_cursor *c, u32 doc_count)
{
    for (u32 d = 0; d < doc_count; d++) {
        u32 delta = 0, positions = 0;
        i32 rc = ppos_read_varint(c, &delta);
        if (rc != PPOS_OK) return rc;
        rc = ppos_read_varint(c, &positions);
        if (rc != PPOS_OK) return rc;
        for (u32 p = 0; p < positions; p++) {
            rc = ppos_read_varint(c, &delta);
            if (rc != PPOS_OK) return rc;
        }
    }
    return PPOS_OK;
}

static i32 ppos_skip_positions(struct ppos_cursor *c, u32 count)
{
    for (u32 i = 0; i < count; i++) {
        u32 value = 0;
        i32 rc = ppos_read_varint(c, &value);
        if (rc != PPOS_OK) return rc;
    }
    return PPOS_OK;
}

i32 ppos_open(const void *encoded, u32 encoded_len, struct ppos_page *out)
{
    if (out) memset(out, 0, sizeof(*out));
    if (!encoded || !out || encoded_len < PPOS_HEADER_BYTES ||
        encoded_len > PPOS_PAGE_MAX_BYTES) return PPOS_INVALID;
    const u8 *raw = (const u8 *)encoded;
    if (ppos_get_le32(raw) != 0x534F5050U || raw[4] != PPOS_VERSION ||
        raw[5] != 0 || ppos_get_le16(raw + 6) != 0)
        return PPOS_CORRUPT;
    u32 terms = ppos_get_le32(raw + 8);
    u32 checksum = ppos_get_le32(raw + 12);
    if (terms > PPOS_MAX_TERMS ||
        ppos_crc32(raw + PPOS_HEADER_BYTES, encoded_len - PPOS_HEADER_BYTES) != checksum)
        return PPOS_CORRUPT;

    struct ppos_cursor c = { raw + PPOS_HEADER_BYTES, raw + encoded_len };
    u8 previous[PPOS_MAX_TERM_BYTES];
    u32 previous_len = 0;
    bool have_previous = false;
    u32 decoded_terms = 0;
    while (c.p < c.end) {
        u8 term_len = *c.p++;
        if (!term_len ||
            (u32)(c.end - c.p) < term_len ||
            !ppos_utf8(c.p, term_len) ||
            (have_previous && ppos_cmp(previous, previous_len, c.p, term_len) >= 0))
            return PPOS_CORRUPT;
        memcpy(previous, c.p, term_len);
        previous_len = term_len;
        have_previous = true;
        c.p += term_len;
        u32 docs = 0;
        i32 rc = ppos_read_varint(&c, &docs);
        if (rc != PPOS_OK) return rc;
        u32 last_doc = 0;
        for (u32 d = 0; d < docs; d++) {
            u32 doc_delta = 0, positions = 0;
            rc = ppos_read_varint(&c, &doc_delta);
            if (rc != PPOS_OK || (d == 0U ? doc_delta : 0U) > PPOS_MAX_DOCUMENTS)
                return PPOS_CORRUPT;
            if (doc_delta > PPOS_MAX_DOCUMENTS - last_doc) return PPOS_CORRUPT;
            last_doc += doc_delta;
            if (last_doc >= PPOS_MAX_DOCUMENTS) return PPOS_CORRUPT;
            rc = ppos_read_varint(&c, &positions);
            if (rc != PPOS_OK) return rc;
            u32 last_pos = 0;
            for (u32 p = 0; p < positions; p++) {
                u32 delta = 0;
                rc = ppos_read_varint(&c, &delta);
                if (rc != PPOS_OK || delta > 0xFFFFFFFFU - last_pos)
                    return PPOS_CORRUPT;
                last_pos += delta;
            }
        }
        decoded_terms++;
        if (decoded_terms > terms) return PPOS_CORRUPT;
    }
    if (decoded_terms != terms) return PPOS_CORRUPT;
    out->encoded = raw;
    out->encoded_len = encoded_len;
    out->generation = ppos_get_le32(raw + 16);
    out->term_count = terms;
    return PPOS_OK;
}

static i32 ppos_find_term(const struct ppos_page *page,
                          const u8 *term, u32 term_len,
                          struct ppos_cursor *postings, u32 *doc_count)
{
    struct ppos_cursor c = {
        page->encoded + PPOS_HEADER_BYTES,
        page->encoded + page->encoded_len
    };
    for (u32 i = 0; i < page->term_count; i++) {
        if (c.p >= c.end) return PPOS_CORRUPT;
        u32 n = *c.p++;
        if ((u32)(c.end - c.p) < n) return PPOS_CORRUPT;
        int cmp = ppos_cmp(c.p, n, term, term_len);
        c.p += n;
        u32 docs = 0;
        i32 rc = ppos_read_varint(&c, &docs);
        if (rc != PPOS_OK) return rc;
        if (cmp == 0) {
            postings->p = c.p;
            postings->end = c.end;
            *doc_count = docs;
            return PPOS_OK;
        }
        rc = ppos_skip_postings(&c, docs);
        if (rc != PPOS_OK) return rc;
        if (cmp > 0) break;
    }
    return PPOS_NOT_FOUND;
}

/*
 * The query helpers below deliberately rescan a term's compact postings
 * instead of materialising an unbounded posting table.  This keeps the EL0
 * provider allocation-free and makes every read bounded by page size.
 */
static bool ppos_doc_has_term(const struct ppos_page *page,
                              const struct ppos_term *term, u32 doc)
{
    struct ppos_cursor c;
    u32 docs = 0;
    if (ppos_find_term(page, term->ptr, term->len, &c, &docs) != PPOS_OK)
        return false;
    u32 current = 0;
    for (u32 d = 0; d < docs; d++) {
        u32 dd = 0, count = 0;
        if (ppos_read_varint(&c, &dd) != PPOS_OK ||
            dd > PPOS_MAX_DOCUMENTS - current ||
            ppos_read_varint(&c, &count) != PPOS_OK) return false;
        current += dd;
        if (current == doc) return true;
        if (current > doc) return false;
        if (ppos_skip_positions(&c, count) != PPOS_OK) return false;
    }
    return false;
}

static bool ppos_doc_has_position(const struct ppos_page *page,
                                  const struct ppos_term *term, u32 doc,
                                  u32 wanted)
{
    struct ppos_cursor c;
    u32 docs = 0;
    if (ppos_find_term(page, term->ptr, term->len, &c, &docs) != PPOS_OK)
        return false;
    u32 current = 0;
    for (u32 d = 0; d < docs; d++) {
        u32 dd = 0, count = 0;
        if (ppos_read_varint(&c, &dd) != PPOS_OK ||
            dd > PPOS_MAX_DOCUMENTS - current ||
            ppos_read_varint(&c, &count) != PPOS_OK) return false;
        current += dd;
        u32 pos = 0;
        for (u32 p = 0; p < count; p++) {
            u32 delta = 0;
            if (ppos_read_varint(&c, &delta) != PPOS_OK ||
                delta > 0xFFFFFFFFU - pos) return false;
            pos += delta;
            if (current == doc && pos == wanted) return true;
        }
        if (current > doc) break;
    }
    return false;
}

static bool ppos_phrase_doc(const struct ppos_page *page,
                            const struct ppos_term *terms, u32 term_count,
                            u32 doc)
{
    struct ppos_cursor c;
    u32 docs = 0;
    if (!term_count || ppos_find_term(page, terms[0].ptr, terms[0].len,
                                      &c, &docs) != PPOS_OK) return false;
    u32 current = 0;
    for (u32 d = 0; d < docs; d++) {
        u32 dd = 0, count = 0;
        if (ppos_read_varint(&c, &dd) != PPOS_OK ||
            dd > PPOS_MAX_DOCUMENTS - current ||
            ppos_read_varint(&c, &count) != PPOS_OK) return false;
        current += dd;
        u32 pos = 0;
        for (u32 p = 0; p < count; p++) {
            u32 delta = 0;
            if (ppos_read_varint(&c, &delta) != PPOS_OK ||
                delta > 0xFFFFFFFFU - pos) return false;
            pos += delta;
            if (current != doc) continue;
            bool match = true;
            for (u32 t = 1; t < term_count; t++) {
                if (!ppos_doc_has_position(page, &terms[t], doc, pos + t)) {
                    match = false;
                    break;
                }
            }
            if (match) return true;
        }
        if (current > doc) break;
    }
    return false;
}

static bool ppos_near_doc(const struct ppos_page *page,
                          const struct ppos_term *terms, u32 term_count,
                          u32 doc, u32 distance)
{
    if (!term_count) return false;
    struct ppos_cursor c;
    u32 docs = 0;
    if (ppos_find_term(page, terms[0].ptr, terms[0].len, &c, &docs) != PPOS_OK)
        return false;
    u32 current = 0;
    for (u32 d = 0; d < docs; d++) {
        u32 dd = 0, count = 0;
        if (ppos_read_varint(&c, &dd) != PPOS_OK ||
            dd > PPOS_MAX_DOCUMENTS - current ||
            ppos_read_varint(&c, &count) != PPOS_OK) return false;
        current += dd;
        u32 pos = 0;
        for (u32 p = 0; p < count; p++) {
            u32 delta = 0;
            if (ppos_read_varint(&c, &delta) != PPOS_OK ||
                delta > 0xFFFFFFFFU - pos) return false;
            pos += delta;
            if (current != doc) continue;
            bool match = true;
            for (u32 t = 1; t < term_count; t++) {
                struct ppos_cursor tc;
                u32 tdocs = 0;
                if (ppos_find_term(page, terms[t].ptr, terms[t].len,
                                   &tc, &tdocs) != PPOS_OK) {
                    match = false;
                    break;
                }
                u32 tdoc = 0;
                bool close = false;
                for (u32 td = 0; td < tdocs; td++) {
                    u32 tdelta = 0, tcount = 0;
                    if (ppos_read_varint(&tc, &tdelta) != PPOS_OK ||
                        tdelta > PPOS_MAX_DOCUMENTS - tdoc ||
                        ppos_read_varint(&tc, &tcount) != PPOS_OK) {
                        match = false;
                        break;
                    }
                    tdoc += tdelta;
                    u32 tpos = 0;
                    for (u32 tp = 0; tp < tcount; tp++) {
                        u32 pd = 0;
                        if (ppos_read_varint(&tc, &pd) != PPOS_OK ||
                            pd > 0xFFFFFFFFU - tpos) {
                            match = false;
                            break;
                        }
                        tpos += pd;
                        if (tdoc == doc) {
                            u32 lo = pos > distance ? pos - distance : 0;
                            u32 hi = pos > 0xFFFFFFFFU - distance ?
                                     0xFFFFFFFFU : pos + distance;
                            if (tpos >= lo && tpos <= hi) close = true;
                        }
                    }
                    if (tdoc > doc) break;
                }
                if (!close) match = false;
                if (!match) break;
            }
            if (match) return true;
        }
        if (current > doc) break;
    }
    return false;
}

static bool ppos_add_result(u32 *out, u32 *count, u32 cap, u32 id)
{
    u32 at = 0;
    while (at < *count && out[at] < id) at++;
    if (at < *count && out[at] == id) return true;
    if (*count < cap) {
        for (u32 i = *count; i > at; i--) out[i] = out[i - 1U];
        out[at] = id;
        (*count)++;
        return true;
    }
    if (cap && at < cap) {
        for (u32 i = cap - 1U; i > at; i--) out[i] = out[i - 1U];
        out[at] = id;
    }
    return false;
}

i32 ppos_query(const struct ppos_page *page, const void *query,
               u32 query_len, u32 mode, u32 near_distance,
               u32 *out_ids, u32 out_cap, u32 *out_count)
{
    if (out_count) *out_count = 0;
    if (!page || !page->encoded || !query || !out_count ||
        (!out_ids && out_cap)) return PPOS_INVALID;
    if (mode > PPOS_QUERY_NEAR) return PPOS_UNSUPPORTED;
    if (out_cap > PPOS_MAX_RESULTS) out_cap = PPOS_MAX_RESULTS;
    struct ppos_page checked;
    i32 rc = ppos_open(page->encoded, page->encoded_len, &checked);
    if (rc != PPOS_OK) return rc;

    u8 term_storage[PPOS_MAX_QUERY_TERMS][PPOS_MAX_TERM_BYTES];
    struct ppos_term terms[PPOS_MAX_QUERY_TERMS];
    u32 term_count = 0;
    u32 off = 0;
    while (off < query_len) {
        u32 n = 0;
        if (term_count >= PPOS_MAX_QUERY_TERMS) return PPOS_FULL;
        rc = ppos_next_token((const u8 *)query, query_len, &off,
                             term_storage[term_count], &n);
        if (rc == PPOS_NOT_FOUND) break;
        if (rc != PPOS_OK) return rc;
        terms[term_count].ptr = term_storage[term_count];
        terms[term_count].len = n;
        term_count++;
    }
    if (!term_count) return PPOS_OK;

    u32 count = 0;
    bool partial = false;
    if (mode == PPOS_QUERY_ANY) {
        for (u32 t = 0; t < term_count; t++) {
            struct ppos_cursor c;
            u32 docs = 0;
            rc = ppos_find_term(&checked, terms[t].ptr, terms[t].len,
                                &c, &docs);
            if (rc == PPOS_NOT_FOUND) continue;
            if (rc != PPOS_OK) return rc;
            u32 doc = 0;
            for (u32 d = 0; d < docs; d++) {
                u32 delta = 0, positions = 0;
                if (ppos_read_varint(&c, &delta) != PPOS_OK ||
                    delta > PPOS_MAX_DOCUMENTS - doc ||
                    ppos_read_varint(&c, &positions) != PPOS_OK) return PPOS_CORRUPT;
                doc += delta;
                if (!ppos_add_result(out_ids, &count, out_cap, doc))
                    partial = true;
                rc = ppos_skip_positions(&c, positions);
                if (rc != PPOS_OK) return rc;
            }
        }
    } else {
        struct ppos_cursor c;
        u32 docs = 0;
        rc = ppos_find_term(&checked, terms[0].ptr, terms[0].len, &c, &docs);
        if (rc == PPOS_NOT_FOUND) return PPOS_OK;
        if (rc != PPOS_OK) return rc;
        u32 doc = 0;
        for (u32 d = 0; d < docs; d++) {
            u32 delta = 0, positions = 0;
            if (ppos_read_varint(&c, &delta) != PPOS_OK ||
                delta > PPOS_MAX_DOCUMENTS - doc ||
                ppos_read_varint(&c, &positions) != PPOS_OK) return PPOS_CORRUPT;
            doc += delta;
            bool match = true;
            if (mode == PPOS_QUERY_AND) {
                for (u32 t = 1; t < term_count; t++)
                    if (!ppos_doc_has_term(&checked, &terms[t], doc)) {
                        match = false;
                        break;
                    }
            } else if (mode == PPOS_QUERY_PHRASE) {
                match = ppos_phrase_doc(&checked, terms, term_count, doc);
            } else {
                u32 distance = near_distance;
                if (distance == 0) distance = 1;
                if (distance > PPOS_MAX_NEAR) distance = PPOS_MAX_NEAR;
                match = ppos_near_doc(&checked, terms, term_count, doc, distance);
            }
            if (match) {
                if (!ppos_add_result(out_ids, &count, out_cap, doc))
                    partial = true;
            }
            rc = ppos_skip_positions(&c, positions);
            if (rc != PPOS_OK) return rc;
        }
    }
    *out_count = count;
    return partial ? PPOS_PARTIAL : PPOS_OK;
}
