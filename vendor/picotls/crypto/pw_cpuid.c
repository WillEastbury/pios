/*
 * Runtime CPU feature detection — implementation.
 */

#include "pw_cpuid.h"

#include <string.h>

#if defined(__x86_64__) || defined(__i386__)
#  include <cpuid.h>
#endif

#if defined(__aarch64__) || defined(__arm__)
#  include <sys/auxv.h>
#  if !defined(HWCAP_SHA2)
#    define HWCAP_SHA2 (1u << 6)   /* aarch64 hwcap bit per kernel docs */
#  endif
#  if !defined(HWCAP_NEON)
#    define HWCAP_NEON (1u << 12)
#  endif
#endif

static cpu_features_t g_cpu;
static int g_cpu_initialised = 0;

const cpu_features_t* cpu_features_init(void) {
    if (g_cpu_initialised) return &g_cpu;
    memset(&g_cpu, 0, sizeof(g_cpu));

#if defined(__x86_64__) || defined(__i386__)
    /* Leaf 1: SSE2, SSSE3, SSE4.1 */
    unsigned eax, ebx, ecx, edx;
    if (__get_cpuid(1, &eax, &ebx, &ecx, &edx)) {
        g_cpu.x86_sse2  = (edx & (1u << 26)) ? 1 : 0;
        g_cpu.x86_ssse3 = (ecx & (1u <<  9)) ? 1 : 0;
        g_cpu.x86_sse41 = (ecx & (1u << 19)) ? 1 : 0;
    }
    /* Leaf 7, sub-leaf 0: SHA-NI is EBX bit 29. */
    unsigned max_basic = __get_cpuid_max(0, NULL);
    if (max_basic >= 7) {
        unsigned a, b, c, d;
        __cpuid_count(7, 0, a, b, c, d);
        g_cpu.x86_sha = (b & (1u << 29)) ? 1 : 0;
    }
#endif

#if defined(__aarch64__) || defined(__arm__)
    unsigned long hwcap = getauxval(AT_HWCAP);
    g_cpu.arm_neon = (hwcap & HWCAP_NEON) ? 1 : 0;
    g_cpu.arm_sha2 = (hwcap & HWCAP_SHA2) ? 1 : 0;
#endif

    g_cpu_initialised = 1;
    return &g_cpu;
}

const cpu_features_t* cpu_features(void) {
    if (!g_cpu_initialised) return cpu_features_init();
    return &g_cpu;
}
