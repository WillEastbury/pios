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
#include "fb.h"

/* Page tables — 4KB aligned (l1_table used by start.S for early MMU) */
u64 l1_table[512] ALIGNED(4096);
static u64 l2_table_low[512] ALIGNED(4096);  /* first 1GB in 2MB blocks */

/* Per-process user tables for user cores. Process slots are 1MiB-misaligned,
 * so each 2MiB slot can straddle two L2 entries and needs two L3 tables. */
static u64 user_l1[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l2_low[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l3_proc[3][MAX_PROCS_PER_CORE][2][512] ALIGNED(4096);
static bool user_table_valid[3][MAX_PROCS_PER_CORE];

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
 *   T0SZ  = 25  → 39-bit VA (512GB — matches 512-entry L1)
 *   IRGN0 = WB WA  (inner cache for page walks)
 *   ORGN0 = WB WA  (outer cache for page walks)
 *   SH0   = Inner Shareable (multi-core coherency)
 *   TG0   = 4KB granule
 *   IPS   = 36-bit PA (64GB physical, Pi 5 max)
 *   EPD1  = 1 → disable TTBR1 walks (no kernel/user split)
 */
#define TCR_VALUE ( \
    (25UL << 0)  |    \
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
    return core >= 1 && core <= 3;
}

static inline u32 user_core_index(u32 core) {
    return core - 1;
}

static inline void map_user_low_2m(u64 *l2, u32 idx, u64 pa, u64 attrs)
{
    if (idx < 512)
        l2[idx] = (pa & ~(L2_BLOCK_SIZE - 1)) | attrs;
}

static inline u64 user_ram_attrs(void)
{
    return PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;
}

static inline u64 user_page_rwx_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;
}

static inline u64 user_page_rx_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RO_EL1 | PTE_UXN;
}

static inline u64 user_page_rw_xn_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
}

static bool mmu_user_slot_pages(u32 core, u32 slot, u64 slot_base, u64 slot_size,
                                u32 code_bytes, bool split)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE || slot_size != PROC_SLOT_SIZE)
        return false;
    if ((slot_base & (L3_PAGE_SIZE - 1)) != 0)
        return false;

    u32 uc = user_core_index(core);
    u64 *l2 = user_l2_low[uc][slot];
    u64 first_l2 = slot_base / L2_BLOCK_SIZE;
    u64 end = slot_base + slot_size;
    u64 code_end = slot_base + code_bytes;
    u64 data_start = split ? ((code_end + L3_PAGE_SIZE - 1) & ~(L3_PAGE_SIZE - 1)) : slot_base;
    if (end < slot_base || first_l2 >= 512 || (end - 1) / L2_BLOCK_SIZE >= 512)
        return false;
    if (split && (code_bytes == 0 || code_end > end || data_start >= end))
        return false;

    simd_zero(user_l3_proc[uc][slot][0], 512 * sizeof(u64));
    simd_zero(user_l3_proc[uc][slot][1], 512 * sizeof(u64));

    u32 l3_count = 0;
    for (u64 a = slot_base; a < end; a += L3_PAGE_SIZE) {
        u64 l2idx = a / L2_BLOCK_SIZE;
        u32 table = (u32)(l2idx - first_l2);
        if (table >= 2)
            return false;
        if (l2[l2idx] == 0) {
            l2[l2idx] = (u64)(usize)user_l3_proc[uc][slot][table] | PTE_VALID | PTE_TABLE;
            l3_count++;
        }
        u64 attrs = user_page_rwx_attrs();
        if (split)
            attrs = (a < data_start) ? user_page_rx_attrs() : user_page_rw_xn_attrs();
        user_l3_proc[uc][slot][table][(a / L3_PAGE_SIZE) & 511U] =
            (a & ~(L3_PAGE_SIZE - 1)) | attrs;
    }

    return l3_count > 0;
}

void mmu_init(void) {
    /* Write the L1 table entirely in inline asm to avoid compiler
     * generating NEON/FP instructions that might fault. */
    volatile u64 *l1 = l1_table;

    /* Zero the table (512 entries) */
    for (u32 i = 0; i < 512; i++)
        l1[i] = 0;

    /* RAM: L1[0] split into 2MB blocks via l2_table_low, L1[1-3] WB cacheable */
    u64 ram_attr = PTE_VALID | PTE_BLOCK | PTE_AF |
                   PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;

    /* L1[0] = table descriptor → l2_table_low (first 1GB in 2MB blocks) */
    volatile u64 *l2 = l2_table_low;
    for (u32 i = 0; i < 512; i++) {
        u64 addr = (u64)i * L2_BLOCK_SIZE;
        l2[i] = addr | ram_attr;  /* Normal WB Cacheable */
    }
    l1[0] = (u64)(usize)l2_table_low | PTE_VALID | PTE_TABLE;

    l1[1] = 0x40000000UL | ram_attr;
    l1[2] = 0x80000000UL | ram_attr;
    l1[3] = 0xC0000000UL | ram_attr;

    /* Peripherals: indices 64-67 (BCM2712 at 0x107C000000 = 65GB) */
    u64 dev_attr = PTE_VALID | PTE_BLOCK | PTE_AF |
                   PTE_ATTR(MT_DEVICE_nGnRnE) |
                   PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
    for (u32 idx = 64; idx < 68; idx++)
        l1[idx] = ((u64)idx * L1_BLOCK_SIZE) | dev_attr;

    /* RP1: indices 124-127 */
    for (u32 idx = 124; idx < 128; idx++)
        l1[idx] = ((u64)idx * L1_BLOCK_SIZE) | dev_attr;

    __asm__ volatile("dsb sy" ::: "memory");
    __asm__ volatile("isb" ::: "memory");

    u64 ttbr = (u64)(usize)l1_table;
    shared_ttbr0 = ttbr;
    shared_mair  = MAIR_VALUE;
    shared_tcr   = TCR_VALUE;

    /* Program system registers one at a time with barriers */
    __asm__ volatile("msr mair_el1, %0; isb" :: "r"((u64)MAIR_VALUE));
    __asm__ volatile("msr tcr_el1, %0; isb" :: "r"((u64)TCR_VALUE));
    __asm__ volatile("msr ttbr0_el1, %0; isb" :: "r"(ttbr));
    __asm__ volatile("msr ttbr1_el1, xzr; isb");
    __asm__ volatile("tlbi vmalle1; dsb sy; isb" ::: "memory");

    /* Enable MMU + caches */
    u64 sctlr;
    __asm__ volatile("mrs %0, sctlr_el1" : "=r"(sctlr));
    sctlr |=  (1UL << 0);      /* M  — MMU ON */
    sctlr |=  (1UL << 2);      /* C  — Data cache */
    sctlr |=  (1UL << 12);     /* I  — Instruction cache */
    sctlr &= ~(1UL << 1);      /* A  — no alignment check */
    sctlr &= ~(1UL << 3);      /* SA — no stack align check */
    sctlr &= ~(1UL << 19);     /* WXN */
    sctlr &= ~(1UL << 25);     /* EE */
    sctlr &= ~(1UL << 3);      /* SA — no stack align check */
    sctlr &= ~(1UL << 19);     /* WXN */
    sctlr &= ~(1UL << 25);     /* EE */
    __asm__ volatile(
        "msr sctlr_el1, %0  \n"
        "isb                 \n"
        :: "r"(sctlr)
    );

    fb_puts("  [mmu] MMU + caches ON!\n");
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
    if ((slot_base & (L3_PAGE_SIZE - 1)) != 0 || slot_size != PROC_SLOT_SIZE)
        return false;

    u32 uc = user_core_index(core);
    u64 *l1 = user_l1[uc][slot];
    u64 *l2 = user_l2_low[uc][slot];
    simd_zero(l1, 512 * sizeof(u64));
    simd_zero(l2, 512 * sizeof(u64));

    l1[0] = (u64)(usize)l2 | PTE_VALID | PTE_TABLE;

    /* Keep only minimum low RAM: kernel image/BSS/stacks (first 2MB). */
    const u64 ram_attrs = user_ram_attrs();
    map_user_low_2m(l2, 0, 0, ram_attrs);

    /* Map this process slot and shared FIFO window (kernel ABI FIFO path). */
    if (!mmu_user_slot_pages(core, slot, slot_base, slot_size, 0, false))
        return false;
    map_user_low_2m(l2, (u32)(SHARED_FIFO_BASE / L2_BLOCK_SIZE), SHARED_FIFO_BASE, ram_attrs);

    /* Keep peripheral MMIO mapped for current direct-call kernel API ABI. */
    for (u32 idx = 4; idx < 8; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
    for (u32 idx = 124; idx < 128; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);

    user_table_valid[uc][slot] = true;
    return true;
}

bool mmu_user_table_build_split(u32 core, u32 slot, u64 slot_base, u64 slot_size, u32 code_bytes)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    if ((slot_base & (L3_PAGE_SIZE - 1)) != 0 || slot_size != PROC_SLOT_SIZE)
        return false;

    u32 uc = user_core_index(core);
    u64 *l1 = user_l1[uc][slot];
    u64 *l2 = user_l2_low[uc][slot];
    simd_zero(l1, 512 * sizeof(u64));
    simd_zero(l2, 512 * sizeof(u64));

    l1[0] = (u64)(usize)l2 | PTE_VALID | PTE_TABLE;

    const u64 ram_attrs = user_ram_attrs();
    map_user_low_2m(l2, 0, 0, ram_attrs);
    if (!mmu_user_slot_pages(core, slot, slot_base, slot_size, code_bytes, true))
        return false;
    map_user_low_2m(l2, (u32)(SHARED_FIFO_BASE / L2_BLOCK_SIZE), SHARED_FIFO_BASE, ram_attrs);

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

bool mmu_user_ipc_shm_window(u32 core, u32 slot, bool enable)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    u32 uc = user_core_index(core);
    if (!user_table_valid[uc][slot])
        return false;

    u64 *l2 = user_l2_low[uc][slot];
    u32 idx = (u32)(IPC_SHM_BASE / L2_BLOCK_SIZE);
    if (enable) {
        map_user_low_2m(l2, idx, IPC_SHM_BASE, user_ram_attrs());
    } else if (idx < 512) {
        l2[idx] = 0;
    }

    if (core == core_id())
        mmu_invalidate_tlb();
    return true;
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

void icache_invalidate_range(u64 start, u64 size) {
    u64 line = 64;
    u64 addr = start & ~(line - 1);
    u64 end = start + size;
    while (addr < end) {
        __asm__ volatile("ic ivau, %0" :: "r"(addr) : "memory");
        addr += line;
    }
    dsb();
    isb();
}
