/*
 * Host-build shim for PIOS types.h.
 *
 * Put tests/stubinc on the include path BEFORE include/ so that pure-logic
 * kernel modules (which do `#include "types.h"`) pick up these host-safe
 * typedefs instead of the real bare-metal header (which contains AArch64
 * inline asm that will not compile on the host). Only modules that touch no
 * MMIO/asm can be unit-tested this way (e.g. picocompress, dhcp_options).
 */
#pragma once
#define PIOS_HOST_TYPES_SHIM  1   /* suppress bare-metal defs in include/types.h */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>   /* memset/memcpy/memcmp: declared by the real types.h */

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t   i8;
typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;
typedef size_t   usize;

#ifndef ALIGNED
#define ALIGNED(n)  __attribute__((aligned(n)))
#endif
#ifndef PACKED
#define PACKED      __attribute__((packed))
#endif
#ifndef NORETURN
#define NORETURN    __attribute__((noreturn))
#endif

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

/* Barrier/sev/wfe stubs — no-ops on the host (pure-logic modules must not
 * depend on their ordering semantics for correctness of the tested logic). */
static inline void dmb(void) {}
static inline void dsb(void) {}
static inline void isb(void) {}
/* Inner-shareable scoped variants used by publication/consumption contracts
 * (e.g. src/adrv.c's call stamp). No-ops on the host. */
static inline void dmb_ish(void) {}
static inline void dmb_ishst(void) {}
static inline void dmb_ishld(void) {}
static inline void dsb_ish(void) {}
static inline void dsb_ishst(void) {}
static inline void sev(void) {}
static inline void wfe(void) {}
static inline u32  core_id(void) { return 0; }
