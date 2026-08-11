#ifndef PICOSCRIPT_ABI_H
#define PICOSCRIPT_ABI_H

#include <stdint.h>
#include <stddef.h>

typedef struct { uint64_t offset; uint32_t length; uint32_t flags; } pico_span;
enum { PICO_SPAN_BORROWED=1u, PICO_SPAN_OWNED=2u, PICO_SPAN_READONLY=4u, PICO_SPAN_INVALID=8u };
typedef enum { PICO_OK=0, PICO_INVALID_ARGUMENT=1, PICO_INVALID_HANDLE=2,
    PICO_MALFORMED_RECORD=3, PICO_CRC_MISMATCH=4, PICO_SCHEMA_MISMATCH=5,
    PICO_CHECKPOINT_MISMATCH=6, PICO_RESOURCE_EXHAUSTED=7, PICO_CANCELLED=8,
    PICO_PROVIDER_UNAVAILABLE=9, PICO_UNSUPPORTED=10, PICO_OVERFLOW=11 } pico_status;

static inline int pico_span_valid(pico_span s) {
    return !(s.flags & PICO_SPAN_INVALID);
}
static inline pico_status pico_span_bounds(pico_span s, uint32_t off, uint32_t width) {
    return pico_span_valid(s) && off <= s.length && width <= s.length - off ? PICO_OK : PICO_INVALID_ARGUMENT;
}
static inline uint16_t pico_le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t pico_le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint64_t pico_le64(const uint8_t *p) { return (uint64_t)pico_le32(p) | ((uint64_t)pico_le32(p+4) << 32); }

#endif
