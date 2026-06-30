/*
 * Host stubs for dhcp.c unit-test build.
 * Only the symbols actually referenced by parse_dhcp_options and the
 * dhcp_parse_options_test wrapper are provided; everything else is omitted.
 * Put tests/stubinc on the include path BEFORE include/ so the real headers
 * pick up the host types shim; these stubs satisfy the linker.
 */
#pragma once

/* simd_zero: used by parse_dhcp_options to clear the dhcp_parsed struct. */
#include <string.h>
static inline void simd_zero(void *p, unsigned long n) { memset(p, 0, n); }
