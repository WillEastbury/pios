#ifndef PICOBROTLI_H
#define PICOBROTLI_H

/*
 * picobrotli: minimal RFC 7932 (Brotli) encoder + decoder.
 *
 * Vendored into picoscript from the picoweb codec
 * (C:/source/www.wavefunctionlabs.com/picoweb/src/brotli.c). Unmodified
 * algorithm; only the header guard / include name changed.
 *
 * Produces valid Brotli streams decodable by any browser (or zlib/Node's
 * brotli decoder). LZ77 + canonical Huffman, single meta-block, WBITS=16,
 * uncompressed meta-block fallback for incompressible data. The decoder
 * reads the subset this encoder emits (plus uncompressed meta-blocks).
 * Zero external dependencies; fully deterministic so the Python / JS / C
 * ports in picoscript are byte-identical.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Encode `input` into a valid Brotli stream in `output`.
 * Returns bytes written, or -1 on error. */
int brotli_encode(const uint8_t* input, size_t input_len,
                  uint8_t* output, size_t output_cap);

/* Decode streams produced by brotli_encode().
 * Returns bytes written, or -1 on unsupported/corrupt input or too-small output. */
int brotli_decode(const uint8_t* input, size_t input_len,
                  uint8_t* output, size_t output_cap);

/* Worst-case output size. */
size_t brotli_bound(size_t input_len);

/* True if Accept-Encoding value contains the "br" token with q > 0. */
bool brotli_accepted(const char* accept_encoding, size_t len);

#endif
