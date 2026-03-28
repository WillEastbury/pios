#pragma once

typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned int        u32;
typedef unsigned long       u64;
typedef signed char         i8;
typedef signed short        i16;
typedef signed int          i32;
typedef signed long         i64;
typedef unsigned long       usize;
typedef int                 bool;

#define true  1
#define false 0
#define NULL  ((void *)0)

#define PACKED      __attribute__((packed))
#define ALIGNED(n)  __attribute__((aligned(n)))
#define UNUSED      __attribute__((unused))
#define NORETURN    __attribute__((noreturn))
#define SECTION(s)  __attribute__((section(s)))

/* Memory barriers (Cortex-A76) */
static inline void dmb(void)  { __asm__ volatile("dmb sy" ::: "memory"); }
static inline void dsb(void)  { __asm__ volatile("dsb sy" ::: "memory"); }
static inline void isb(void)  { __asm__ volatile("isb"    ::: "memory"); }
static inline void wfe(void)  { __asm__ volatile("wfe"); }
static inline void wfi(void)  { __asm__ volatile("wfi"); }
static inline void sev(void)  { __asm__ volatile("sev"); }

static inline u32 core_id(void) {
    u64 mpidr;
    __asm__ volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    return (u32)(mpidr & 0x3);
}

static inline void delay_cycles(u32 n) {
    while (n--) __asm__ volatile("nop");
}

/* Branch prediction hints */
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

/* Compiler barriers */
#define barrier()   __asm__ volatile("" ::: "memory")

/* Minimal libc replacements */
void *memset(void *dst, int c, usize n);
void *memcpy(void *dst, const void *src, usize n);
int   memcmp(const void *a, const void *b, usize n);
u32   pios_strlen(const char *s);
