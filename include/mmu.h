/*
 * mmu.h - AArch64 MMU with identity-mapped page tables
 *
 * Flat identity mapping (VA == PA) with attribute control:
 *   - Normal cacheable for RAM
 *   - Device-nGnRnE for MMIO peripherals
 *   - Per-core memory marked RW, other cores' RAM marked RO
 *   - Executable only where needed
 */

#pragma once
#include "types.h"

/* Memory attribute indices in MAIR_EL1 */
#define MT_DEVICE_nGnRnE    0   /* Strongly-ordered device memory */
#define MT_NORMAL_NC        1   /* Normal non-cacheable */
#define MT_NORMAL           2   /* Normal write-back cacheable */
#define MT_NORMAL_WT        3   /* Normal write-through cacheable */

/* Page table entry flags */
#define PTE_VALID           (1UL << 0)
#define PTE_TABLE           (1UL << 1)   /* L0-L2: points to next table */
#define PTE_BLOCK           (0UL << 1)   /* L1-L2: block mapping */
#define PTE_PAGE            (1UL << 1)   /* L3: page mapping */
#define PTE_AF              (1UL << 10)  /* Access flag */
#define PTE_SH_INNER        (3UL << 8)   /* Inner shareable */
#define PTE_SH_OUTER        (2UL << 8)   /* Outer shareable */
#define PTE_AP_RW_EL1       (0UL << 6)   /* EL1 read/write */
#define PTE_AP_RO_EL1       (2UL << 6)   /* EL1 read-only */
#define PTE_UXN             (1UL << 54)  /* Unprivileged Execute Never */
#define PTE_PXN             (1UL << 53)  /* Privileged Execute Never */
#define PTE_ATTR(idx)       ((u64)(idx) << 2)

/* Block sizes at each translation level (4KB granule) */
#define L1_BLOCK_SIZE       (1UL << 30)  /* 1GB */
#define L2_BLOCK_SIZE       (1UL << 21)  /* 2MB */
#define L3_PAGE_SIZE        (1UL << 12)  /* 4KB */

/* Enable the MMU with identity mapping.
 * Call from each core after setting up page tables. */
void mmu_init(void);

/* Invalidate all TLB entries */
void mmu_invalidate_tlb(void);

/* Data cache operations */
void dcache_clean_range(u64 start, u64 size);
void dcache_invalidate_range(u64 start, u64 size);
void dcache_clean_invalidate_range(u64 start, u64 size);
