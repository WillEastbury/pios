/*
 * Picoweb scatter-gather descriptor (`pw_iov_t`).
 *
 * This is the canonical "byte plan" type used to thread response
 * fragments through the layered userspace stack without copying the
 * underlying bytes. It is intentionally identical in shape to POSIX
 * `struct iovec` so it maps 1:1 onto:
 *
 *   - writev(2) / sendmsg(2) / send_msg(MSG_ZEROCOPY)
 *   - io_uring SQE iov entries
 *   - DPDK rte_mbuf chained payloads (next pointer + segment length)
 *   - AF_XDP TX descriptor batches
 *
 * The webserver builds an array of these pointing at static-arena
 * bytes (header / chrome / page body / footer / status line) and
 * hands the array to the TLS layer. TLS computes the total length
 * up front, then walks the chain feeding ChaCha20 XOR + Poly1305
 * incrementally. No copy of the payload ever happens on the way out.
 *
 * Memory rules:
 *   - `base` is a borrowed pointer; the caller owns the storage.
 *   - The chain is read-only from TLS's perspective (TLS produces
 *     ciphertext into a separate, single, contiguous buffer rented
 *     from the per-worker AEAD output pool).
 *   - No descriptor in a chain may be NULL; zero-length entries are
 *     legal but should be avoided (they cost iteration overhead).
 */
#ifndef PICOWEB_USERSPACE_IOV_H
#define PICOWEB_USERSPACE_IOV_H

#include <stddef.h>
#include <stdint.h>

#define PW_IOV_MAX_FRAGS  8u

typedef struct pw_iov {
    const uint8_t* base;
    size_t         len;
} pw_iov_t;

/* Total payload length of a chain. */
static inline size_t pw_iov_total(const pw_iov_t* iov, unsigned n) {
    size_t t = 0;
    for (unsigned i = 0; i < n; i++) t += iov[i].len;
    return t;
}

#endif
