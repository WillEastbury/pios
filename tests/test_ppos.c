#include "ppos.h"
#include <stdio.h>

struct doc_store {
    const char *text[8];
};

static i32 read_doc(void *opaque, u32 id, u8 *out, u32 cap)
{
    struct doc_store *s = (struct doc_store *)opaque;
    if (id >= 8 || !s->text[id]) return PPOS_NOT_FOUND;
    u32 n = 0;
    while (s->text[id][n]) n++;
    if (n > cap) return PPOS_BUFFER_TOO_SMALL;
    for (u32 i = 0; i < n; i++) out[i] = (u8)s->text[id][i];
    return (i32)n;
}

static int check(bool condition, const char *what)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", what);
        return 1;
    }
    return 0;
}

int main(void)
{
    struct doc_store store = {{
        "The quick brown fox",
        "quick fox jumps",
        "brown dog",
        "quick_brown fox",
        0, 0, 0, 0
    }};
    u8 doc_buf[128];
    u8 encoded[PPOS_PAGE_MAX_BYTES];
    struct ppos_source source = {
        read_doc, &store, doc_buf, sizeof(doc_buf), 8
    };
    u32 encoded_len = 0;
    if (check(ppos_build(encoded, sizeof(encoded), 7, &source, &encoded_len)
              == PPOS_OK, "build")) return 1;
    if (check(encoded_len > PPOS_HEADER_BYTES, "non-empty page")) return 1;
    if (check(encoded[0] == 'P' && encoded[1] == 'P' &&
              encoded[2] == 'O' && encoded[3] == 'S', "header magic")) return 1;

    struct ppos_page page;
    if (check(ppos_open(encoded, encoded_len, &page) == PPOS_OK, "open")) return 1;
    if (check(page.generation == 7 && page.term_count == 6, "header metadata"))
        return 1;

    u32 ids[8], count = 0;
    if (check(ppos_query(&page, "quick", 5, PPOS_QUERY_ANY, 0,
                         ids, 8, &count) == PPOS_OK &&
              count == 3 && ids[0] == 0 && ids[1] == 1 && ids[2] == 3,
              "ANY")) return 1;
    if (check(ppos_query(&page, "quick fox", 9, PPOS_QUERY_AND, 0,
                         ids, 8, &count) == PPOS_OK &&
              count == 3 && ids[0] == 0 && ids[1] == 1 && ids[2] == 3,
              "AND")) return 1;
    if (check(ppos_query(&page, "quick brown", 11, PPOS_QUERY_PHRASE, 0,
                         ids, 8, &count) == PPOS_OK &&
              count == 2 && ids[0] == 0 && ids[1] == 3, "PHRASE")) return 1;
    if (check(ppos_query(&page, "brown fox", 9, PPOS_QUERY_NEAR, 1,
                         ids, 8, &count) == PPOS_OK &&
              count == 2 && ids[0] == 0 && ids[1] == 3, "NEAR")) return 1;
    if (check(ppos_query(&page, "quick fox", 9, PPOS_QUERY_ANY, 0,
                         ids, 1, &count) == PPOS_PARTIAL &&
              count == 1, "partial result status")) return 1;

    encoded[PPOS_HEADER_BYTES] ^= 1;
    if (check(ppos_open(encoded, encoded_len, &page) == PPOS_CORRUPT,
              "CRC rejection")) return 1;

    struct doc_store fixture_store = {{"a b", "a", 0, 0, 0, 0, 0, 0}};
    struct ppos_source fixture_source = {
        read_doc, &fixture_store, doc_buf, sizeof(doc_buf), 2
    };
    u32 fixture_len = 0;
    static const u8 fixture_expected[] = {
        0x50,0x50,0x4F,0x53,0x01,0x00,0x00,0x00,
        0x02,0x00,0x00,0x00,0x3C,0x59,0xF7,0xFB,
        0x02,0x00,0x00,0x00,0x01,0x61,0x02,0x00,
        0x01,0x00,0x01,0x01,0x00,0x01,0x62,0x01,
        0x00,0x01,0x01
    };
    i32 fixture_rc = ppos_build(encoded, sizeof(encoded), 2, &fixture_source,
                                &fixture_len);
    bool fixture_equal = fixture_len == sizeof(fixture_expected);
    for (u32 i = 0; fixture_equal && i < fixture_len; i++)
        if (encoded[i] != fixture_expected[i]) fixture_equal = false;
    if (check(fixture_rc == PPOS_OK &&
              fixture_len == 35 &&
              fixture_equal &&
              ppos_crc32(encoded + PPOS_HEADER_BYTES, fixture_len -
                         PPOS_HEADER_BYTES) == 0xFBF7593CU,
              "reference fixture bytes")) return 1;
    printf("PPOS: format, CRC, ANY/AND/PHRASE/NEAR PASS\n");
    return 0;
}
