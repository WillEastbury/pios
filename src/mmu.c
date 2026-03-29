/*
 * mmu.c - AArch64 MMU with identity-mapped page tables
 *
 * Identity map: VA == PA.
 *
 * Three regions:
 *   1. LOW MEMORY  (RAM)          — Normal WB cacheable, inner-shareable
 *   2. PERIPHERALS (BCM2712+RP1)  — Device-nGnRnE, non-cacheable
 *   3. CACHES ENABLED             — L1 I-cache + D-cache + TLB
 *
 * 4KB granule. L1 = 1GB blocks. L2 = 2MB blocks for first 1GB.
 *
 * Memory layout (BCM2712 / Pi 5 with 36-bit PA):
 *   0x0_00000000 - 0x0_3FFFFFFF  1GB  RAM (L2 fine-grained)
 *   0x0_40000000 - 0x0_FFFFFFFF  3GB  RAM (L1 1GB blocks)
 *   0x1_00000000 - 0x1_0FFFFFFF  ~256MB  More peripherals / PCIe
 *   0x1_07C00000 - 0x1_07FFFFFF  4MB  BCM2712 ARM peripherals
 *   0x1_08000000 - 0x1_09FFFFFF  32MB GIC-400
 *   0x1_F0000000 - 0x1_F0FFFFFF  16MB RP1 BAR0 (via PCIe)
 */

#include "mmu.h"
#include "mmio.h"
#include "uart.h"
#include "simd.h"
#include "core_env.h"
#include "proc.h"

/* Page tables — 4KB aligned */
static u64 l1_table[512] ALIGNED(4096);
static u64 l2_table_low[512] ALIGNED(4096);  /* first 1GB in 2MB blocks */

/* Per-process user tables for user cores (core2/core3). */
static u64 user_l1[2][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l2_low[2][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static bool user_table_valid[2][MAX_PROCS_PER_CORE];

/* Exported for secondary cores (read by start.S) */
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;

/*
 * MAIR_EL1: Memory Attribute Indirection Register
 *   Index 0: Device-nGnRnE (0x00) — strongly-ordered MMIO
 *   Index 1: Normal Non-Cacheable (0x44)
 *   Index 2: Normal Write-Back R/W Allocate (0xFF) — main RAM
 *   Index 3: Normal Write-Through R/W Allocate (0xBB)
 */
#define MAIR_VALUE ( \
    (0x00UL <<  0) |  \
    (0x44UL <<  8) |  \
    (0xFFUL << 16) |  \
    (0xBBUL << 24)    \
)

/*
 * TCR_EL1: Translation Control Register
 *   T0SZ  = 16  → 48-bit VA (256TB, way more than we need)
 *   IRGN0 = WB WA  (inner cache for page walks)
 *   ORGN0 = WB WA  (outer cache for page walks)
 *   SH0   = Inner Shareable (multi-core coherency)
 *   TG0   = 4KB granule
 *   IPS   = 36-bit PA (64GB physical, Pi 5 max)
 *   EPD1  = 1 → disable TTBR1 walks (no kernel/user split)
 */
#define TCR_VALUE ( \
    (16UL << 0)  |    \
    (0UL  << 6)  |    \
    (1UL  << 8)  |    \
    (1UL  << 10) |    \
    (3UL  << 12) |    \
    (0UL  << 14) |    \
    (1UL  << 23) |    \
    (2UL  << 32)      \
)

/* Helper: create a 1GB L1 block entry for normal cacheable RAM */
static inline u64 ram_block_1g(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;
}

/* Helper: create a 1GB L1 block entry for device MMIO */
static inline u64 dev_block_1g(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_ATTR(MT_DEVICE_nGnRnE) |
           PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
}

/* Helper: create a 2MB L2 block entry for normal cacheable RAM */
static inline u64 ram_block_2m(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;
}

/* Helper: create a 2MB L2 block entry for device MMIO */
static inline u64 dev_block_2m(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_ATTR(MT_DEVICE_nGnRnE) |
           PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
}

static inline bool is_user_core(u32 core) {
    return core == 2 || core == 3;
}

static inline u32 user_core_index(u32 core) {
    return core - 2;
}

static inline void map_user_low_2m(u64 *l2, u32 idx, u64 pa, u64 attrs)
{
    if (idx < 512)
        l2[idx] = (pa & ~(L2_BLOCK_SIZE - 1)) | attrs;
}

void mmu_init(void) {
    /* Zero all tables */
    simd_zero(l1_table, sizeof(l1_table));
    simd_zero(l2_table_low, sizeof(l2_table_low));
    simd_zero(user_l1, sizeof(user_l1));
    simd_zero(user_l2_low, sizeof(user_l2_low));
    simd_zero(user_table_valid, sizeof(user_table_valid));

    /* ============================================================
     * L2 TABLE: First 1GB (0x00000000 - 0x3FFFFFFF) in 2MB blocks
     *   All normal cacheable RAM.
     * ============================================================ */
    for (u32 i = 0; i < 512; i++) {
        l2_table_low[i] = ram_block_2m((u64)i * L2_BLOCK_SIZE);
    }

    /* L1[0] → L2 table descriptor (first 1GB) */
    l1_table[0] = (u64)(usize)l2_table_low | PTE_VALID | PTE_TABLE;

    /* ============================================================
     * L1 BLOCKS: RAM at 1GB granularity
     *   0x40000000 - 0xFFFFFFFF (indices 1-3)
     * ============================================================ */
    l1_table[1] = ram_block_1g(0x40000000UL);
    l1_table[2] = ram_block_1g(0x80000000UL);
    l1_table[3] = ram_block_1g(0xC0000000UL);

    /* ============================================================
     * L1 BLOCKS: PERIPHERALS at 1GB granularity
     *
     * BCM2712 peripherals: 0x1_00000000 - 0x1_0FFFFFFF
     *   L1 index = 0x100000000 >> 30 = 4
     *   Covers: UART, EMMC2, Mailbox, GENET, DMA, GIC, etc.
     *
     * More peripheral space: indices 4-7 (4GB peripheral window)
     * ============================================================ */
    for (u32 idx = 4; idx < 8; idx++) {
        l1_table[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
    }

    /* ============================================================
     * RP1 southbridge: 0x1_F0000000 (via PCIe BAR0)
     *   L1 index = 0x1F0000000 >> 30 = 7 (already covered above)
     *   But the actual mapping may be at higher addresses:
     *   0x1F_00000000 → L1 index = 124
     * ============================================================ */
    for (u32 idx = 124; idx < 128; idx++) {
        l1_table[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
    }

    dsb();
    isb();

    /* ============================================================
     * PROGRAM SYSTEM REGISTERS
     * ============================================================ */

    /* Store values for secondary cores to replicate */
    shared_ttbr0 = (u64)(usize)l1_table;
    shared_mair  = MAIR_VALUE;
    shared_tcr   = TCR_VALUE;

    /* MAIR_EL1: memory attribute encodings */
    __asm__ volatile("msr mair_el1, %0" :: "r"((u64)MAIR_VALUE));

    /* TCR_EL1: translation control */
    __asm__ volatile("msr tcr_el1, %0" :: "r"((u64)TCR_VALUE));

    /* TTBR0_EL1: base of page table hierarchy */
    __asm__ volatile("msr ttbr0_el1, %0" :: "r"(shared_ttbr0));

    /* TTBR1_EL1: unused, EPD1 disables walks but zero it anyway */
    __asm__ volatile("msr ttbr1_el1, xzr");

    /* Full TLB invalidate before enabling */
    __asm__ volatile(
        "tlbi vmalle1    \n"
        "dsb  sy         \n"
        "isb             \n"
        ::: "memory"
    );

    /* ============================================================
     * SCTLR_EL1: Enable MMU + Caches
     *   M  (bit 0)  = 1: MMU ON
     *   A  (bit 1)  = 1: Alignment check ON
     *   C  (bit 2)  = 1: Data cache ON
     *   SA (bit 3)  = 1: Stack alignment check ON
     *   I  (bit 12) = 1: Instruction cache ON
     *   WXN(bit 19) = 0: No write-implies-XN
     *   EE (bit 25) = 0: Little-endian
     * ============================================================ */
    u64 sctlr;
    __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
    sctlr |=  (1UL << 0);      /* M  — MMU enable */
    sctlr |=  (1UL << 1);      /* A  — Alignment check */
    sctlr |=  (1UL << 2);      /* C  — Data cache enable */
    sctlr |=  (1UL << 3);      /* SA — Stack alignment check */
    sctlr |=  (1UL << 12);     /* I  — Instruction cache enable */
    sctlr &= ~(1UL << 19);     /* WXN — clear */
    sctlr &= ~(1UL << 25);     /* EE — little-endian */
    __asm__ volatile(
        "msr sctlr_el1, %0  \n"
        "isb                 \n"
        :: "r"(sctlr)
    );

    uart_puts("[mmu] SCTLR_EL1=");
    uart_hex(sctlr);
    uart_puts(" TTBR0_EL1=");
    uart_hex(shared_ttbr0);
    uart_puts("\n");
    uart_puts("[mmu] Identity map: low mem + periph, caches ON\n");
}

void mmu_invalidate_tlb(void) {
    __asm__ volatile("tlbi vmalle1" ::: "memory");
    dsb();
    isb();
}

u64 mmu_kernel_ttbr0(void) {
    return shared_ttbr0;
}

bool mmu_user_table_build(u32 core, u32 slot, u64 slot_base, u64 slot_size)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    if ((slot_base & (L2_BLOCK_SIZE - 1)) != 0 || slot_size != PROC_SLOT_SIZE)
        return false;

    u32 uc = user_core_index(core);
    u64 *l1 = user_l1[uc][slot];
    u64 *l2 = user_l2_low[uc][slot];
    simd_zero(l1, 512 * sizeof(u64));
    simd_zero(l2, 512 * sizeof(u64));

    l1[0] = (u64)(usize)l2 | PTE_VALID | PTE_TABLE;

    /* Keep only minimum low RAM: kernel image/BSS/stacks (first 2MB). */
    const u64 ram_attrs = PTE_VALID | PTE_BLOCK | PTE_AF |
                          PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;
    map_user_low_2m(l2, 0, 0, ram_attrs);

    /* Map this process slot and shared FIFO/DMA windows. */
    map_user_low_2m(l2, (u32)(slot_base / L2_BLOCK_SIZE), slot_base, ram_attrs);
    for (u64 pa = SHARED_FIFO_BASE; pa < DMA_DISK_BASE + DMA_DISK_SIZE; pa += L2_BLOCK_SIZE)
        map_user_low_2m(l2, (u32)(pa / L2_BLOCK_SIZE), pa, ram_attrs);

    /* Keep peripheral MMIO mapped for current direct-call syscall ABI. */
    for (u32 idx = 4; idx < 8; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
    for (u32 idx = 124; idx < 128; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);

    user_table_valid[uc][slot] = true;
    return true;
}

bool mmu_switch_to_user(u32 core, u32 slot)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    u32 uc = user_core_index(core);
    if (!user_table_valid[uc][slot])
        return false;

    u64 ttbr = (u64)(usize)user_l1[uc][slot];
    __asm__ volatile("msr ttbr0_el1, %0" :: "r"(ttbr));
    mmu_invalidate_tlb();
    return true;
}

void mmu_switch_to_kernel(void)
{
    __asm__ volatile("msr ttbr0_el1, %0" :: "r"(shared_ttbr0));
    mmu_invalidate_tlb();
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
