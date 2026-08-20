#ifndef PICOSCRIPT_RNG_H
#define PICOSCRIPT_RNG_H
#include <stdint.h>
static inline uint64_t pico_rng_mix(uint64_t x) {
    x += UINT64_C(0x9E3779B97F4A7C15); x = (x ^ (x >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94D049BB133111EB); return x ^ (x >> 31);
}
typedef struct { uint64_t seed, stream_id, position; } pico_rng_stream;
static inline uint64_t pico_rng_next(pico_rng_stream *r) {
    uint64_t v = pico_rng_mix(r->seed ^ pico_rng_mix(r->stream_id) ^ r->position++); return v;
}
static inline int pico_rng_below(pico_rng_stream *r, uint64_t upper, uint64_t *out) {
    if (!r || !out || !upper) return 0;
    uint64_t limit = UINT64_MAX - (UINT64_MAX % upper);
    for (;;) { uint64_t v = pico_rng_next(r); if (v < limit) { *out = v % upper; return 1; } }
}
#endif
