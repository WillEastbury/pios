/*
 * mmu.c - AArch64 MMU with identity-mapped page tables
 *
 * 4KB granule, 2-level (L1 1GB blocks + L2 2MB blocks for fine control).
 * Identity map: VA == PA. Attribute control only.
 *
 * Memory map:
 *   0x00000000 - 0x3FFFFFFF  RAM (Normal, cacheable, inner-shareable)
 *   0x40000000 - 0x7FFFFFFF  RAM continued
 *   0x107C000000+            Device (nGnRnE, non-cacheable)
 */

#include "mmu.h"
#include "mmio.h"
#include "uart.h"
#include "simd.h"

/* Page tables — 4KB aligned.
 * L1: 512 entries × 1GB blocks = 512GB coverage (enough for Pi 5 36-bit PA).
 * L2: one table for finer 2MB granularity where needed. */
static u64 l1_table[512] ALIGNED(4096);
static u64 l2_table_low[512] ALIGNED(4096);  /* covers 0x00000000-0x3FFFFFFF */

/* MAIR: define the 4 attribute sets we use */
#define MAIR_VALUE ( \
    (0x00UL <<  0) | /* Index 0: Device-nGnRnE */ \
    (0x44UL <<  8) | /* Index 1: Normal Non-Cacheable */ \
    (0xFFUL << 16) | /* Index 2: Normal WB RW-Alloc */ \
    (0xBBUL << 24)   /* Index 3: Normal WT RW-Alloc */ \
)

/* TCR_EL1: 4KB granule, 36-bit PA, 48-bit VA space, inner-shareable */
#define TCR_VALUE ( \
    (16UL << 0)  |   /* T0SZ = 16 → 48-bit VA space */ \
    (0UL  << 6)  |   /* !EPD0 = walks enabled for TTBR0 */ \
    (1UL  << 8)  |   /* IRGN0 = WB WA */ \
    (1UL  << 10) |   /* ORGN0 = WB WA */ \
    (3UL  << 12) |   /* SH0 = Inner Shareable */ \
    (0UL  << 14) |   /* TG0 = 4KB granule */ \
    (2UL  << 32)     /* IPS = 36-bit PA (4GB addressable) */ \
)

void mmu_init(void) {
    /* Zero tables */
    simd_zero(l1_table, sizeof(l1_table));
    simd_zero(l2_table_low, sizeof(l2_table_low));

    /*
     * L2 table: covers 0x00000000 - 0x3FFFFFFF in 2MB blocks.
     * All normal cacheable RAM.
     */
    for (u32 i = 0; i < 512; i++) {
        u64 addr = (u64)i * L2_BLOCK_SIZE;
        l2_table_low[i] = addr | PTE_VALID | PTE_BLOCK | PTE_AF |
                          PTE_SH_INNER | PTE_ATTR(MT_NORMAL) |
                          PTE_AP_RW_EL1 | PTE_UXN;
    }

    /* L1[0] → L2 table (first 1GB, fine-grained control) */
    l1_table[0] = (u64)(usize)l2_table_low | PTE_VALID | PTE_TABLE;

    /* L1[1]: 0x40000000 - 0x7FFFFFFF — RAM, 1GB block, cacheable */
    l1_table[1] = 0x40000000UL | PTE_VALID | PTE_BLOCK | PTE_AF |
                  PTE_SH_INNER | PTE_ATTR(MT_NORMAL) |
                  PTE_AP_RW_EL1 | PTE_UXN;

    /* L1[2]: 0x80000000 - 0xBFFFFFFF — more RAM if present */
    l1_table[2] = 0x80000000UL | PTE_VALID | PTE_BLOCK | PTE_AF |
                  PTE_SH_INNER | PTE_ATTR(MT_NORMAL) |
                  PTE_AP_RW_EL1 | PTE_UXN;

    /* L1[3]: 0xC0000000 - 0xFFFFFFFF — more RAM */
    l1_table[3] = 0xC0000000UL | PTE_VALID | PTE_BLOCK | PTE_AF |
                  PTE_SH_INNER | PTE_ATTR(MT_NORMAL) |
                  PTE_AP_RW_EL1 | PTE_UXN;

    /* Peripheral regions: device memory.
     * BCM2712 peripherals at 0x107C000000 → L1 index = 0x107C000000 >> 30 = 65 */
    for (u32 idx = 64; idx < 72; idx++) {
        u64 addr = (u64)idx * L1_BLOCK_SIZE;
        l1_table[idx] = addr | PTE_VALID | PTE_BLOCK | PTE_AF |
                        PTE_ATTR(MT_DEVICE_nGnRnE) |
                        PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
    }

    /* RP1 at 0x1F00000000 → L1 index = 124 */
    for (u32 idx = 124; idx < 128; idx++) {
        u64 addr = (u64)idx * L1_BLOCK_SIZE;
        l1_table[idx] = addr | PTE_VALID | PTE_BLOCK | PTE_AF |
                        PTE_ATTR(MT_DEVICE_nGnRnE) |
                        PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
    }

    dsb();
    isb();

    /* Set MAIR */
    __asm__ volatile("msr mair_el1, %0" :: "r"(MAIR_VALUE));

    /* Set TCR */
    __asm__ volatile("msr tcr_el1, %0" :: "r"(TCR_VALUE));

    /* Set TTBR0 */
    u64 ttbr = (u64)(usize)l1_table;
    __asm__ volatile("msr ttbr0_el1, %0" :: "r"(ttbr));

    /* TTBR1 unused — disable walks */
    __asm__ volatile("msr ttbr1_el1, %0" :: "r"(0UL));

    /* Invalidate TLB */
    __asm__ volatile("tlbi vmalle1" ::: "memory");
    dsb();
    isb();

    /* Enable MMU: SCTLR_EL1 bits M=1, C=1, I=1 */
    u64 sctlr;
    __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
    sctlr |= (1 << 0);     /* M — MMU enable */
    sctlr |= (1 << 2);     /* C — Data cache enable */
    sctlr |= (1 << 12);    /* I — Instruction cache enable */
    __asm__ volatile("msr sctlr_el1, %0" :: "r"(sctlr));
    isb();

    uart_puts("[mmu] Enabled: identity map, 4KB granule, WB cacheable\n");
}

void mmu_invalidate_tlb(void) {
    __asm__ volatile("tlbi vmalle1" ::: "memory");
    dsb();
    isb();
}

void dcache_clean_range(u64 start, u64 size) {
    u64 line = 64;  /* Cortex-A76 cache line = 64 bytes */
    u64 addr = start & ~(line - 1);
    u64 end = start + size;
    while (addr < end) {
        __asm__ volatile("dc cvac, %0" :: "r"(addr) : "memory");
        addr += line;
    }
    dsb();
}

void dcache_invalidate_range(u64 start, u64 size) {
    u64 line = 64;
    u64 addr = start & ~(line - 1);
    u64 end = start + size;
    while (addr < end) {
        __asm__ volatile("dc ivac, %0" :: "r"(addr) : "memory");
        addr += line;
    }
    dsb();
}

void dcache_clean_invalidate_range(u64 start, u64 size) {
    u64 line = 64;
    u64 addr = start & ~(line - 1);
    u64 end = start + size;
    while (addr < end) {
        __asm__ volatile("dc civac, %0" :: "r"(addr) : "memory");
        addr += line;
    }
    dsb();
}
