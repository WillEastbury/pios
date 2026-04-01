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

/* L1 page table (populated by start.S for early boot) */
extern u64 l1_table[512];

/* Shared values for secondary cores */
extern u64 shared_ttbr0;
extern u64 shared_mair;
extern u64 shared_tcr;

/* Invalidate all TLB entries */
void mmu_invalidate_tlb(void);

/* Kernel TTBR0 base used by service cores and scheduler context */
u64 mmu_kernel_ttbr0(void);

/* Build an isolated user table for a process slot on core 2 or 3 */
bool mmu_user_table_build(u32 core, u32 slot, u64 slot_base, u64 slot_size);

/* Switch active TTBR0 to a process table on core 2/3 */
bool mmu_switch_to_user(u32 core, u32 slot);

/* Toggle user-process visibility of the IPC SHM window (coarse 2MB block). */
bool mmu_user_ipc_shm_window(u32 core, u32 slot, bool enable);

/* Switch active TTBR0 back to the global kernel table */
void mmu_switch_to_kernel(void);

/* Data cache operations */
void dcache_clean_range(u64 start, u64 size);
void dcache_invalidate_range(u64 start, u64 size);
void dcache_clean_invalidate_range(u64 start, u64 size);
