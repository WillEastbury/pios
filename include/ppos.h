#pragma once

#include "types.h"

/*
 * PPOS - bounded positional full-text overlay.
 *
 * The encoded page is intentionally independent of WALFS.  A provider may
 * store the immutable byte string in any append-only record store, while
 * ppos_open() remains the single authority for format, bounds, and CRC
 * validation.
 */
#define PPOS_MAGIC                 "PPOS"
#define PPOS_VERSION               1U
#define PPOS_HEADER_BYTES          20U
#define PPOS_BODY_MAX_BYTES        (16U * 1024U)
#define PPOS_PAGE_MAX_BYTES        (PPOS_HEADER_BYTES + PPOS_BODY_MAX_BYTES)
#define PPOS_MAX_TERMS             16384U
#define PPOS_MAX_DOCUMENTS         65536U
#define PPOS_MAX_RESULTS            4096U
#define PPOS_MAX_NEAR              64U
#define PPOS_MAX_TERM_BYTES        255U
#define PPOS_MAX_QUERY_TERMS       32U

enum ppos_status {
    PPOS_OK = 0,
    PPOS_INVALID = -1,
    PPOS_CORRUPT = -2,
    PPOS_PARTIAL = -3,
    PPOS_FULL = -4,
    PPOS_NOT_FOUND = -5,
    PPOS_STALE = -6,
    PPOS_IO_ERROR = -7,
    PPOS_BUFFER_TOO_SMALL = -8,
    PPOS_UNSUPPORTED = -9
};

enum ppos_query_mode {
    PPOS_QUERY_ANY = 0,
    PPOS_QUERY_AND = 1,
    PPOS_QUERY_PHRASE = 2,
    PPOS_QUERY_NEAR = 3
};

struct ppos_page {
    const u8 *encoded;
    u32 encoded_len;
    u32 generation;
    u32 term_count;
};

/*
 * read() returns the document byte length, PPOS_NOT_FOUND for a hole, or a
 * negative ppos_status.  The callback must not write beyond document_buf_cap.
 */
typedef i32 (*ppos_read_doc_fn)(void *ctx, u32 document_id,
                                u8 *document_buf, u32 document_buf_cap);

struct ppos_source {
    ppos_read_doc_fn read;
    void *ctx;
    u8 *document_buf;
    u32 document_buf_cap;
    u32 document_limit; /* number of IDs in [0, document_limit) */
};

/* IEEE CRC-32 used by the reference Python implementation (not CRC32C). */
u32 ppos_crc32(const void *data, u32 len);

/*
 * Build an immutable page from an authoritative bounded document source.
 * No heap is used.  The output is byte-for-byte compatible with the Python
 * reference for ASCII/UTF-8 token streams.
 */
i32 ppos_build(u8 *out, u32 out_cap, u32 generation,
               const struct ppos_source *source, u32 *out_len);

/* Validate header, decoded postings, term ordering, bounds, and CRC. */
i32 ppos_open(const void *encoded, u32 encoded_len, struct ppos_page *out);

/*
 * Execute one bounded query.  *out_count is always the number of returned
 * IDs.  PPOS_PARTIAL means the sorted result set exceeded out_cap; the first
 * out_cap IDs were retained.  Invalid modes/input and corrupt pages fail
 * explicitly.
 */
i32 ppos_query(const struct ppos_page *page, const void *query,
               u32 query_len, u32 mode, u32 near_distance,
               u32 *out_ids, u32 out_cap, u32 *out_count);
