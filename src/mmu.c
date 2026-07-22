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
u64 l2_table_boot[512] ALIGNED(4096);  /* start.S low-1GB split; never edited live */
static u64 l1_table_cached[512] ALIGNED(4096); /* BBM-safe target: fresh root for cache enable */
static u64 l2_table_low[512] ALIGNED(4096);  /* first 1GB in 2MB blocks */
static u64 l3_block0[512] ALIGNED(4096);     /* block 0 (0-2MB) split to 4KB pages */

/* Per-process user tables for user cores. Process slots are 1MiB-misaligned,
 * so each 2MiB slot can straddle two L2 entries and needs two L3 tables.
 *
 * A user table can touch up to three distinct 1 GiB L1 regions, and each region
 * gets its OWN L2 table so a mapping installed for one region can never alias
 * into another. (The previous single user_l2_low was shared by L1[0], the
 * process VA and the IPC alias at once, which manufactured high-VA aliases and
 * — because QEMU's slots/FIFO/IPC sit above 1 GiB — indexed the 512-entry table
 * out of bounds so QEMU user tables never built.)
 *   user_l2_low  -> L1[0]                          low identity 0-1 GiB
 *   user_l2_phys -> L1[CORE0_RAM_BASE / 1 GiB]     kernel/core/FIFO/IPC RAM
 *                                                  (== L1[0] on Pi, L1[1] QEMU)
 *   user_l2_high -> L1[0x2001000000 / 1 GiB] (128) EL0 linked image + IPC alias
 * On Pi the phys region IS L1[0], so user_l2_low backs it and user_l2_phys stays
 * unused (see user_l2_install / user_l2_lookup), preserving the exact Pi L1[0]
 * table used today. */
static u64 user_l1[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l2_low[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l2_phys[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l2_high[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);
static u64 user_l3_proc[3][MAX_PROCS_PER_CORE][2][512] ALIGNED(4096);
static u64 user_l3_ipc[3][MAX_PROCS_PER_CORE][512] ALIGNED(4096);

/* 1 GiB L1 region indices backed by a process's own fine-grained L2 tables.
 * All are compile-time constants folded from the platform memory map. */
#define USER_L1_LOW_INDEX   0u
#define USER_L1_PHYS_INDEX  ((u32)(CORE0_RAM_BASE / L1_BLOCK_SIZE))
#define USER_HIGH_LINK_BASE 0x2001000000ULL
#define USER_HIGH_IPC_ALIAS 0x2003000000ULL
#define USER_L1_HIGH_INDEX  ((u32)(USER_HIGH_LINK_BASE / L1_BLOCK_SIZE))
_Static_assert((USER_HIGH_IPC_ALIAS / L1_BLOCK_SIZE) == (USER_HIGH_LINK_BASE / L1_BLOCK_SIZE),
               "EL0 linked base and IPC alias must share one high L1 region");
struct mmu_valid_slot {
    volatile u32 v;
    u32 _pad[15];
} ALIGNED(64);
static struct mmu_valid_slot user_table_valid[3][MAX_PROCS_PER_CORE];
_Static_assert(sizeof(struct mmu_valid_slot) == 64,
               "user table validity slots must be one cache line");

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

/* Helper: create a 1GB L1 block entry for normal cacheable RAM.
 * PXN|UXN: the kernel never executes instructions through this coarse
 * block-level mapping (kernel .text lives only in the fine-grained block-0
 * code window built by kimg_page_attrs()); everywhere else covered by this
 * helper is process-slot storage, scratch RAM, or framebuffer memory that
 * the kernel only ever reads/writes, never fetches instructions from under
 * its own TTBR. Real W^X: never both writable and executable. */
static inline u64 ram_block_1g(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1 | PTE_PXN | PTE_UXN;
}

/* Helper: create a 1GB L1 block entry for device MMIO */
static inline u64 dev_block_1g(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_ATTR(MT_DEVICE_nGnRnE) |
           PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
}

/* Helper: create a 2MB L2 block entry for normal cacheable RAM (see
 * ram_block_1g comment: PXN|UXN because the kernel never executes through
 * this coarse mapping). */
static inline u64 ram_block_2m(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1 | PTE_PXN | PTE_UXN;
}

/* Helper: 2MB WB L2 block that stays EL1-EXECUTABLE (PXN clear). The QEMU
 * kernel image/scheduler window must remain fetchable at EL1 while a user
 * TTBR0 is active, so the scheduler can execute its dispatch tail, ctx_switch()
 * and proc_el0_trampoline() (all in kernel .text at 0x40080000) after
 * mmu_switch_to_user(). This mirrors the kernel's own coarse L1[1] block
 * (start.S installs 0x709 = WB, AP_RW_EL1, PXN=0; mmu_enable_caching() copies
 * L1 entries 1..511 verbatim into l1_table_cached, so the live kernel view of
 * this PA range is WB + EL1-executable). AP_RW_EL1 + UXN keep EL0 out entirely
 * (no access, no execute); cacheability is WB so the same PA carries identical
 * memory attributes under both the kernel and user TTBRs. */
static inline u64 ram_block_2m_exec(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1 | PTE_UXN;
}

/* Helper: create a 2MB L2 block entry for Normal Non-Cacheable RAM.
 * Bit pattern (addr | 0x705) is identical to the NC block start.S installs
 * at l1_table[0], so NC regions keep their exact current attributes (the
 * added PXN|UXN bits sit above bit 52 and don't disturb that low-bit
 * invariant). */
static inline u64 ram_block_2m_nc(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL_NC) | PTE_AP_RW_EL1 | PTE_PXN | PTE_UXN;
}

/* Helper: create a 2MB L2 block entry for device MMIO */
static inline u64 dev_block_2m(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_ATTR(MT_DEVICE_nGnRnE) |
           PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN;
}

/* Kernel image page attributes (4KB granule), real W^X enforcement for
 * block 0 (0x0-0x1FFFFF): a 3-way split matching the R E / RW ELF PHDRS
 * split already applied to the link scripts.
 *   [code_lo, rodata_hi)  .text.boot+.text+.stage2_manifest+.rodata
 *                          -> read-only, executable (no PXN/UXN)
 *   [rodata_hi, bss_lo)   .data (writable initialised globals)
 *                          -> read-write, execute-never (PXN|UXN)
 *   everything else       low RAM below the image + .bss head in block 0
 *                          -> read-write, execute-never (PXN|UXN)
 * No page is ever both writable and executable. Cacheability is unchanged
 * from before (WB only inside [code_lo, bss_lo), NC elsewhere) so this is
 * a pure permission tightening, not a behavior change to caching. */
static inline u64 kimg_page_attrs(u64 page, u64 code_lo, u64 rodata_hi, u64 bss_lo)
{
    u64 memattr = (page >= code_lo && page < bss_lo)
                      ? PTE_ATTR(MT_NORMAL) : PTE_ATTR(MT_NORMAL_NC);
    if (page >= code_lo && page < rodata_hi)
        return page | PTE_VALID | PTE_PAGE | PTE_AF | PTE_SH_INNER |
               memattr | PTE_AP_RO_EL1;
    return page | PTE_VALID | PTE_PAGE | PTE_AF | PTE_SH_INNER |
           memattr | PTE_AP_RW_EL1 | PTE_PXN | PTE_UXN;
}

static void map_user_device_windows(u64 *l1)
{
    for (u32 idx = 4; idx < 8; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
    for (u32 idx = 64; idx < 68; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
    for (u32 idx = 124; idx < 128; idx++)
        l1[idx] = dev_block_1g((u64)idx * L1_BLOCK_SIZE);
}

static inline bool is_user_core(u32 core) {
    return core >= 1 && core <= 3;
}

static inline u32 user_core_index(u32 core) {
    return core - 1;
}

/* Return the per-slot L2 table that backs 1 GiB L1 region `l1idx`, or NULL if
 * that region is not one a user table fine-grains (device windows / unused RAM).
 * Install form: also writes the L1 table descriptor pointing at the L2 table.
 * On Pi USER_L1_PHYS_INDEX == USER_L1_LOW_INDEX, so the low branch catches it
 * and the phys region resolves to user_l2_low, exactly as before. */
static u64 *user_l2_install(u32 uc, u32 slot, u64 *l1, u32 l1idx)
{
    u64 *l2;
    if (l1idx == USER_L1_LOW_INDEX)
        l2 = user_l2_low[uc][slot];
    else if (l1idx == USER_L1_PHYS_INDEX)
        l2 = user_l2_phys[uc][slot];
    else if (l1idx == USER_L1_HIGH_INDEX)
        l2 = user_l2_high[uc][slot];
    else
        return 0;
    if (l1idx < 512)
        l1[l1idx] = (u64)(usize)l2 | PTE_VALID | PTE_TABLE;
    return l2;
}

/* Lookup form (no L1 write): pick the L2 table a queried VA's L1 region uses. */
static u64 *user_l2_lookup(u32 uc, u32 slot, u32 l1idx)
{
    if (l1idx == USER_L1_LOW_INDEX)
        return user_l2_low[uc][slot];
    if (l1idx == USER_L1_PHYS_INDEX)
        return user_l2_phys[uc][slot];
    if (l1idx == USER_L1_HIGH_INDEX)
        return user_l2_high[uc][slot];
    return 0;
}

/* Map a 2 MiB identity block (VA == PA) for `pa` into whichever per-slot L2
 * table backs pa's 1 GiB L1 region, installing the L1 descriptor. Fails closed
 * if pa's region is not user-fine-grained. Indexing is relative to the region
 * base, so it is correct whether the block sits below or above 1 GiB. */
static bool map_user_identity_2m(u32 uc, u32 slot, u64 *l1, u64 pa, u64 attrs)
{
    u32 l1idx = (u32)(pa / L1_BLOCK_SIZE);
    u64 *l2 = user_l2_install(uc, slot, l1, l1idx);
    if (!l2)
        return false;
    u32 idx = (u32)((pa % L1_BLOCK_SIZE) / L2_BLOCK_SIZE);
    l2[idx] = (pa & ~(L2_BLOCK_SIZE - 1)) | attrs;
    return true;
}

static inline u64 user_ram_attrs(void)
{
    return PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1;
}

static inline u64 user_ram_nc_attrs(void)
{
    return PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL_NC) | PTE_AP_RW_EL1;
}

static void map_user_kernel_low(u64 *l2)
{
    /* Mirror every privileged kernel/private-RAM block before SHARED_FIFO.
     * On Pi this includes the linker-reserved secondary scheduler stacks at
     * ~9.8-10.7 MiB. Mapping only [0, CORE0_RAM_BASE) (the old bound, 8 MiB)
     * made the first stack access after mmu_switch_to_user fault at scheduler
     * stage 50, before ctx_switch/EL0 entry. Copy l2_table_low verbatim so each
     * PA keeps its exact NC/WB and PXN/UXN attributes; EL0 still has no access.
     *
     * On QEMU these physical ranges are in L1[1], so the 512-entry L1[0] table
     * remains clamped here and map_user_kernel_phys() handles the actual kernel
     * window separately. */
    u32 count = (u32)(SHARED_FIFO_BASE / L2_BLOCK_SIZE);
    if (count > 512U)
        count = 512U;
    for (u32 b = 0; b < count; b++) {
        u64 pa = (u64)b * L2_BLOCK_SIZE;
        /* Mirror the live kernel low-RAM attributes EXACTLY by copying
         * l2_table_low. After cache enable, l2_table_low[0] points at the
         * WB-code/NC-.bss L3 table and blocks 1..3 (.bss/stacks/heap) are NC;
         * before cache enable, fall back to the original all-NC low-RAM mapping.
         * Copying verbatim guarantees the user TTBR and kernel TTBR always agree
         * on the attribute for every physical address, so SVC/ctx-switch code
         * running under the user TTBR can never write process/scheduler state
         * through one cacheability that the kernel TTBR later reads through the
         * other (the WB/NC alias that previously broke the multicore wake ring). */
        l2[b] = l2_table_low[b] ? l2_table_low[b] : ram_block_2m_nc(pa);
    }
}

/* QEMU: the kernel image, BSS, per-core scheduler stacks and every global
 * scheduler object live in [region_base, CORE0_RAM_BASE) inside L1[1], not L1[0]
 * (the QEMU kernel links at 0x40080000). Map that whole window privileged WB
 * and EL1-EXECUTABLE (EL1 RW+X, EL0 no access, UXN) so the scheduler can fetch
 * and run its dispatch tail + ctx_switch + proc_el0_trampoline on its own kernel
 * stack while TTBR0 points at this user table -- the same WB, EL1-executable
 * attributes as the kernel's coarse L1[1] block (start.S 0x709, PXN clear), just
 * at 2 MiB granularity. Using a PXN block here instead faults the very first
 * instruction fetch after mmu_switch_to_user(), so the process is never entered
 * (el0_enter_count stays 0) and the core spins in an EL1 instruction-abort loop.
 * On Pi the phys region IS L1[0] and is already covered verbatim by
 * map_user_kernel_low(), so USER_L1_PHYS_INDEX == USER_L1_LOW_INDEX and this is
 * a no-op there (Pi L1[0] table unchanged). */
static void map_user_kernel_phys(u32 uc, u32 slot, u64 *l1)
{
    u32 l1idx = USER_L1_PHYS_INDEX;
    if (l1idx == USER_L1_LOW_INDEX)
        return;
    u64 region_base = (u64)l1idx * L1_BLOCK_SIZE;
    u64 *l2 = user_l2_install(uc, slot, l1, l1idx);
    if (!l2)
        return;
    for (u64 pa = region_base; pa < CORE0_RAM_BASE; pa += L2_BLOCK_SIZE) {
        u32 idx = (u32)((pa - region_base) / L2_BLOCK_SIZE);
        if (idx < 512)
            l2[idx] = ram_block_2m_exec(pa);
    }
}

/* PTE_NG (not global) on all three of these: they map per-process, per-slot
 * private code/data pages (user_l3_proc[uc][slot], reachable only via this
 * process's own TTBR0). Without NG these entries are TLB-global, so a stale
 * translation could in principle outlive a slot's reuse by ASID alone --
 * a rubber-duck review caught that the EL1-side variants here lacked PTE_NG
 * while the EL0-side ones (user_page_el0_*_attrs below) already had it.
 * Correctness never depended on this: mmu_switch_to_user/_kernel always do a
 * full, all-ASID TLB invalidate on every switch regardless of tagging (see
 * the ASID comment on mmu_switch_to_user), so no stale entry has ever
 * actually survived a switch. This closes the gap so the ASID tagging is
 * also real defense-in-depth here, not just on the EL0 mappings. */
static inline u64 user_page_rwx_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1 | PTE_NG;
}

static inline u64 user_page_rx_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RO_EL1 | PTE_UXN | PTE_NG;
}

static inline u64 user_page_rw_xn_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL1 | PTE_UXN | PTE_PXN | PTE_NG;
}

static inline u64 user_page_el0_rx_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL0 | PTE_PXN | PTE_NG;
}

static inline u64 user_page_el0_rw_xn_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL) | PTE_AP_RW_EL0 | PTE_UXN | PTE_PXN | PTE_NG;
}

static inline u64 user_page_el0_nc_xn_attrs(void)
{
    return PTE_VALID | PTE_PAGE | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL_NC) | PTE_AP_RW_EL0 | PTE_UXN | PTE_PXN | PTE_NG;
}

/* Map the shared IPC_SHM window EL0-readable at `alias` (a high VA) into the
 * high region's OWN L2 table (user_l2_high), never the low/phys L2 -- so the
 * IPC alias can never leak into the identity map. Fails closed if the alias is
 * not inside the single supported high L1 region (USER_L1_HIGH_INDEX). */
static bool map_user_ipc_el0_alias(u32 uc, u32 slot, u64 *l1, u64 alias)
{
    u32 l1idx = (u32)(alias / L1_BLOCK_SIZE);
    if (l1idx != USER_L1_HIGH_INDEX)
        return false;
    u64 *l2 = user_l2_install(uc, slot, l1, l1idx);
    if (!l2)
        return false;
    u32 idx = (u32)((alias % L1_BLOCK_SIZE) / L2_BLOCK_SIZE);
    simd_zero(user_l3_ipc[uc][slot], sizeof(user_l3_ipc[uc][slot]));
    l2[idx] = (u64)(usize)user_l3_ipc[uc][slot] | PTE_VALID | PTE_TABLE;
    for (u32 i = 0; i < 512; i++) {
        if ((u64)i * L3_PAGE_SIZE >= IPC_SHM_SIZE) {
            user_l3_ipc[uc][slot][i] = 0;
            continue;
        }
        u64 pa = IPC_SHM_BASE + (u64)i * L3_PAGE_SIZE;
        user_l3_ipc[uc][slot][i] = pa | user_page_el0_nc_xn_attrs();
    }
    return true;
}

static bool mmu_user_slot_pages(u32 core, u32 slot, u64 *l1, u64 slot_base,
                                u64 slot_size, u32 code_bytes, bool split,
                                bool el0_access)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE || slot_size != PROC_SLOT_SIZE)
        return false;
    if ((slot_base & (L3_PAGE_SIZE - 1)) != 0)
        return false;

    u32 uc = user_core_index(core);
    u64 end = slot_base + slot_size;
    if (end < slot_base)
        return false;

    /* The slot must sit entirely inside ONE 1 GiB L1 region so it maps through a
     * single per-slot L2 table; index that L2 RELATIVE to the region base, never
     * by absolute PA (QEMU slots at 0x44xxxxxx would otherwise index a 512-entry
     * L2 out of bounds). */
    u32 l1idx = (u32)(slot_base / L1_BLOCK_SIZE);
    if ((u32)((end - 1) / L1_BLOCK_SIZE) != l1idx)
        return false;
    u64 *l2 = user_l2_install(uc, slot, l1, l1idx);
    if (!l2)
        return false;
    u64 region_base = (u64)l1idx * L1_BLOCK_SIZE;

    u64 first_l2 = (slot_base - region_base) / L2_BLOCK_SIZE;
    u64 last_l2 = (end - 1 - region_base) / L2_BLOCK_SIZE;
    u64 code_end = slot_base + code_bytes;
    u64 data_start = split ? ((code_end + L3_PAGE_SIZE - 1) & ~(L3_PAGE_SIZE - 1)) : slot_base;
    if (first_l2 >= 512 || last_l2 >= 512)
        return false;
    if (split && (code_bytes == 0 || code_end > end || data_start >= end))
        return false;

    simd_zero(user_l3_proc[uc][slot][0], 512 * sizeof(u64));
    simd_zero(user_l3_proc[uc][slot][1], 512 * sizeof(u64));

    u32 l3_count = 0;
    for (u64 a = slot_base; a < end; a += L3_PAGE_SIZE) {
        u64 l2idx = (a - region_base) / L2_BLOCK_SIZE;
        u32 table = (u32)(l2idx - first_l2);
        if (table >= 2)
            return false;
        if (l2[l2idx] == 0) {
            l2[l2idx] = (u64)(usize)user_l3_proc[uc][slot][table] | PTE_VALID | PTE_TABLE;
            l3_count++;
        }
        u64 attrs = user_page_rwx_attrs();
        if (split) {
            if (el0_access)
                attrs = (a < data_start) ? user_page_el0_rx_attrs() : user_page_el0_rw_xn_attrs();
            else
                attrs = (a < data_start) ? user_page_rx_attrs() : user_page_rw_xn_attrs();
        }
        user_l3_proc[uc][slot][table][(a / L3_PAGE_SIZE) & 511U] =
            (a & ~(L3_PAGE_SIZE - 1)) | attrs;
    }

    return l3_count > 0;
}

static void mmu_user_table_publish(u32 uc, u32 slot)
{
    dcache_clean_invalidate_range((u64)(usize)user_l1[uc][slot], sizeof(user_l1[uc][slot]));
    dcache_clean_invalidate_range((u64)(usize)user_l2_low[uc][slot], sizeof(user_l2_low[uc][slot]));
    dcache_clean_invalidate_range((u64)(usize)user_l2_phys[uc][slot], sizeof(user_l2_phys[uc][slot]));
    dcache_clean_invalidate_range((u64)(usize)user_l2_high[uc][slot], sizeof(user_l2_high[uc][slot]));
    dcache_clean_invalidate_range((u64)(usize)user_l3_proc[uc][slot], sizeof(user_l3_proc[uc][slot]));
    dcache_clean_invalidate_range((u64)(usize)user_l3_ipc[uc][slot], sizeof(user_l3_ipc[uc][slot]));
    dsb();
}

/* Fresh per-slot table skeleton shared by every builder: zero the L1 and all
 * three per-slot L2 tables, then install the always-present kernel-side regions
 * -- the low identity map (L1[0], kernel low RAM copied verbatim from
 * l2_table_low) and, on QEMU only, the kernel image/stack/globals window in
 * L1[1]. Per-builder code then adds the process slot, FIFO, IPC window and (for
 * the EL0-at builder) the high linked image window. */
static void mmu_user_table_reset(u32 uc, u32 slot, u64 *l1)
{
    simd_zero(l1, 512 * sizeof(u64));
    simd_zero(user_l2_low[uc][slot], 512 * sizeof(u64));
    simd_zero(user_l2_phys[uc][slot], 512 * sizeof(u64));
    simd_zero(user_l2_high[uc][slot], 512 * sizeof(u64));
    l1[USER_L1_LOW_INDEX] = (u64)(usize)user_l2_low[uc][slot] | PTE_VALID | PTE_TABLE;
    map_user_kernel_low(user_l2_low[uc][slot]);
    map_user_kernel_phys(uc, slot, l1);
}

void mmu_init(void) {
    /* Write the L1 table entirely in inline asm to avoid compiler
     * generating NEON/FP instructions that might fault. */
    volatile u64 *l1 = l1_table;

    /* Zero the table (512 entries) */
    for (u32 i = 0; i < 512; i++)
        l1[i] = 0;

    /* L1[0] = table descriptor -> l2_table_low. Preserve the same low-RAM
     * attribute split used by mmu_enable_caching(): kernel code/data WB,
     * kernel BSS/page-tables/scheduler metadata NC, core-private RAM WB,
     * shared FIFO/DMA/IPC NC, framebuffer back buffer WB. */
    volatile u64 *l2 = l2_table_low;
    extern char __text_start[];
    extern char __rodata_end[];
    extern char __bss_start[];
    const u64 code_lo    = (u64)(usize)__text_start  & ~(L3_PAGE_SIZE - 1);
    const u64 rodata_hi  = (u64)(usize)__rodata_end & ~(L3_PAGE_SIZE - 1);
    const u64 code_hi    = (u64)(usize)__bss_start   & ~(L3_PAGE_SIZE - 1);
    for (u32 i = 0; i < 512; i++) {
        u64 addr = (u64)i * L2_BLOCK_SIZE;
        if (i == 0) {
            for (u32 p = 0; p < 512; p++) {
                u64 page = (u64)p * L3_PAGE_SIZE;
                l3_block0[p] = kimg_page_attrs(page, code_lo, rodata_hi, code_hi);
            }
            l2[i] = (u64)(usize)l3_block0 | PTE_VALID | PTE_TABLE;
            continue;
        }
        bool cache = (addr >= CORE0_RAM_BASE && addr < SHARED_FIFO_BASE) ||
                     (addr >= FB_BACK_BASE && addr < FB_BACK_BASE + FB_BACK_SIZE);
        l2[i] = cache ? ram_block_2m(addr) : ram_block_2m_nc(addr);
    }
    l1[0] = (u64)(usize)l2_table_low | PTE_VALID | PTE_TABLE;

    l1[1] = ram_block_1g(0x40000000UL);
    l1[2] = ram_block_1g(0x80000000UL);
    l1[3] = ram_block_1g(0xC0000000UL);

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

/*
 * mmu_enable_caching() — Phase 1 cache-coherency cleanup (BBM-safe).
 *
 * start.S brings the MMU up with l1_table[0] as a single 1GB Normal
 * Non-Cacheable block (0x705), so the entire first 1GB (kernel, all RAM,
 * FIFO, DMA, IPC, framebuffers) runs uncached: ~118ns/write and fragile
 * cross-core sharing. This converts the first 1GB into a 2MB-granular L2
 * table and makes the single-owner / DSU-coherent RAM regions Write-Back
 * cacheable + Inner Shareable (0x709), while keeping Non-Cacheable every
 * region a non-coherent DMA master or the VideoCore scanout can touch:
 *
 *   block 0       0x00000000-0x001FFFFF  split to 4KB pages (L3):
 *                  [__text_start,__bss_start) kernel code/.rodata/.data -> WB IS
 *                  (cached so instruction fetch + font/const reads are fast)
 *                  rest of block 0 (low 512KB + .bss head)              -> NC
 *   blocks 1-3    0x00200000-0x007FFFFF  .bss/stacks/heap/page-tables -> NC
 *                  (active NIC=MACB, USB/xHCI, DMA engine all keep their DMA
 *                   rings/buffers here but already do dcache clean/invalidate;
 *                   caching these is Phase 2b, gated on per-driver review)
 *   blocks 4-35   0x00800000-0x047FFFFF  per-core private RAM        -> WB IS
 *   blocks 36-39  0x04800000-0x04FFFFFF  FIFO/DMA_NET/DMA_DISK/IPC   -> NC
 *   blocks 40-47  0x05000000-0x05FFFFFF  HDMI back buffer           -> WB IS
 *   blocks 48-511 0x06000000-0x3FFFFFFF  high RAM + VideoCore scanout-> NC
 *
 * Break-before-make is honoured at the TRANSLATION ROOT rather than by an
 * in-place edit of the live l1_table[0] descriptor (block->table in place lets
 * the walker hold the old 1GB block and a freshly-walked table for the same VA
 * at once -> TLB conflict abort / PiSOD). The fine-grained hierarchy is built
 * off to the side in l1_table_cached (entry 0 -> L2 split; entries 1..511
 * copied verbatim) and installed with a single atomic TTBR0_EL1 swap + full
 * TLB flush; the new table is VA==PA identical for all executing kernel code
 * (attributes only tighten NC->WB), so there is no unmapped/conflicting window.
 *
 * Safe because: the system is fully NC until this runs (no dirty D-cache RAM
 * lines), dma.c already cleans/invalidates its DMA targets, the HDMI blit is
 * NEON (CPU), SD is PIO, and the process loader cleans D-cache to PoC +
 * invalidates I-cache after copying images. Call once on core 0 after the
 * start.S MMU handoff and before secondaries launch — it republishes
 * shared_ttbr0 to l1_table_cached so they inherit the cached mapping.
 */
void mmu_enable_caching(void) {
    const u64 cache_lo_base = CORE0_RAM_BASE;                /* 0x00800000 */
    const u64 cache_lo_end  = SHARED_FIFO_BASE;              /* 0x04800000 */
    const u64 cache_fb_base = FB_BACK_BASE;                  /* 0x05000000 */
    const u64 cache_fb_end  = FB_BACK_BASE + FB_BACK_SIZE;   /* 0x06000000 */

    /* Split block 0 (0x0-0x1FFFFF) into 4KB pages so ONLY the kernel code +
     * read-only/initialised data window [__text_start, __bss_start) is Write-Back
     * cacheable + Inner Shareable; the low 512KB below the kernel and the head of
     * .bss stay Non-Cacheable. Blocks 1-3 (0x200000-0x7FFFFF: rest of .bss, the
     * per-core stacks, the heap) stay Non-Cacheable too (cache_lo_base starts at
     * CORE0_RAM_BASE). This keeps every driver DMA buffer/descriptor that lives in
     * .bss (MACB rings/bufs, etc.) coherent with its DMA master: with the caches
     * now actually allocating (WB-from-boot fix), mapping that .bss WB caused the
     * NIC to wedge under load. The dedicated per-core DMA arena (follow-up) will
     * let us reclaim .bss WB while keeping DMA memory NC.
     *
     * Real W^X on top of that same cacheability split (kimg_page_attrs, see its
     * comment above): [__text_start, __rodata_end) is read-only + executable;
     * [__rodata_end, __bss_start) (.data) and everything else in block 0 is
     * read-write + execute-never. rodata_hi rounds DOWN like code_hi/code_lo --
     * a page straddling the rodata/data boundary must come out writable, not
     * read-only, or the first write to an early .data global page-sharing with
     * the .rodata tail would fault.
     *
     * IMPORTANT (corrected after rubber-duck review): this whole mechanism only
     * covers block 0, physical [0x0, 0x1FFFFF] via l2_table_low/l3_block0 -- it
     * assumes __text_start falls inside that first 2MB, which is true for real
     * Pi5 hardware (link.ld links at 0x80000) but NOT for QEMU (link_qemu_full.ld
     * links at 0x40080000, inside L1[1]'s separate 1GB block, untouched by this
     * function). So even when force-enabled on QEMU for a one-off test
     * (PIOS_ENABLE_CACHE_REMAP=1), this loop still runs and correctly enforces
     * RO+X/RW+XN over the *unused* low-2MB region, but it does NOT touch the
     * QEMU kernel's own .text/.rodata/.data at 0x40080000+, which stays whatever
     * start.S's coarse 1GB block left it as (WB, RWX, no PXN/UXN). A QEMU-forced
     * run of this path therefore does not prove W^X actually protects the
     * running kernel's own code -- only a real Pi5 hardware boot (where
     * PIOS_ENABLE_CACHE_REMAP defaults to 1 and __text_start genuinely sits in
     * block 0) exercises that. Verify via readelf -l on real_kernel.elf (PT_LOAD
     * R E vs RW segments) and, ideally, an on-device page-table dump rather than
     * a QEMU run. */
    extern char __text_start[];
    extern char __rodata_end[];
    extern char __bss_start[];
    const u64 code_lo   = (u64)(usize)__text_start  & ~(L3_PAGE_SIZE - 1);
    const u64 rodata_hi = (u64)(usize)__rodata_end  & ~(L3_PAGE_SIZE - 1);
    const u64 code_hi   = (u64)(usize)__bss_start   & ~(L3_PAGE_SIZE - 1);
    for (u32 i = 0; i < 512; i++) {
        u64 addr = (u64)i * L3_PAGE_SIZE;
        l3_block0[i] = kimg_page_attrs(addr, code_lo, rodata_hi, code_hi);
    }

    for (u32 i = 0; i < 512; i++) {
        if (i == 0) {
            /* block 0 is now fine-grained via l3_block0 (cached code window). */
            l2_table_low[0] = (u64)(usize)l3_block0 | PTE_VALID | PTE_TABLE;
            continue;
        }
        u64 addr = (u64)i * L2_BLOCK_SIZE;
        bool cache = (addr >= cache_lo_base && addr < cache_lo_end) ||
                     (addr >= cache_fb_base && addr < cache_fb_end);
        l2_table_low[i] = cache ? ram_block_2m(addr) : ram_block_2m_nc(addr);
    }

    /* l2_table_low and l3_block0 were written through the still-active NC
     * mapping, so the stores already sit in DRAM. Build the fresh top-level
     * table beside the live one: entry 0 -> the fine-grained L2 above; entries
     * 1..511 copied verbatim from l1_table so every other VA (high RAM WB
     * blocks, BCM2712 + RP1 device windows) keeps its exact mapping. */
    l1_table_cached[0] = (u64)(usize)l2_table_low | PTE_VALID | PTE_TABLE;
    for (u32 i = 1; i < 512; i++)
        l1_table_cached[i] = l1_table[i];

    /* Push all three tables to PoC and drop any stray cached lines so the
     * cacheable table walker (TCR IRGN0/ORGN0=WB) reads the fresh descriptors. */
    dsb();
    dcache_clean_invalidate_range((u64)(usize)l3_block0,       sizeof(l3_block0));
    dcache_clean_invalidate_range((u64)(usize)l2_table_low,    sizeof(l2_table_low));
    dcache_clean_invalidate_range((u64)(usize)l1_table_cached, sizeof(l1_table_cached));

    /* Publish the cached root so secondaries (launched later by core_start_all)
     * pick it up via shared_ttbr0 when they program their own TTBR0_EL1. */
    shared_ttbr0 = (u64)(usize)l1_table_cached;
    dsb();
    dcache_clean_invalidate_range((u64)(usize)&shared_ttbr0, sizeof(shared_ttbr0));

    /* Break-before-make at the TRANSLATION ROOT, not by an in-place l1_table[0]
     * block->table edit (which lets the walker hold the old 1GB block and a new
     * table for the same VA at once -> TLB conflict abort / PiSOD). Switch TTBR0
     * to the fresh table and flush the whole stage-1 TLB in one shot (no ASIDs,
     * so vmalle1 drops every stale block/page; kernel mappings are global, so an
     * ASID bump would not help anyway). This mirrors the proven
     * mmu_switch_to_user/_kernel pattern (msr ttbr0; tlbi vmalle1; dsb; isb) that
     * already does live same-ASID block<->table swaps every scheduler tick on
     * this A76. No `isb` between msr and tlbi: that would make the new context
     * observable while the stale global 1GB NC entry still lingers, widening the
     * alias window; instead the old entry stays a TLB hit for the identity-mapped
     * code (no conflicting walk, and the swap asm does no data accesses here)
     * until tlbi drops it. The new table is VA==PA identical for all executing
     * kernel code (attributes only tighten NC->WB); the system was fully NC until
     * now so the D-cache holds no dirty RAM lines; ic iallu drops any NC
     * instruction fetches so they refetch from the now-WB code window. */
    u64 ttbr = (u64)(usize)l1_table_cached;
    __asm__ volatile(
        "dsb    sy            \n"
        "msr    ttbr0_el1, %0 \n"
        "tlbi   vmalle1       \n"
        "dsb    sy            \n"
        "ic     iallu         \n"
        "dsb    sy            \n"
        "isb                  \n"
        :: "r"(ttbr) : "memory"
    );
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

    /* Fresh skeleton: L1[0] low identity (kernel low RAM copied verbatim from
     * l2_table_low, privileged/EL0-inaccessible) + the QEMU kernel image window
     * in L1[1]. The scheduler keeps executing kernel code with TTBR0 pointing at
     * this user table between mmu_switch_to_user() and ctx_switch(), so it must
     * reach its own stack, procs[], scheduler_ctx and preempt state through these
     * privileged mappings; isolation is preserved because they are EL0-no-access. */
    mmu_user_table_reset(uc, slot, l1);

    /* Map this process slot (identity VA == PA) and shared FIFO window. */
    if (!mmu_user_slot_pages(core, slot, l1, slot_base, slot_size, 0, false, false))
        return false;
    if (!map_user_identity_2m(uc, slot, l1, SHARED_FIFO_BASE, user_ram_nc_attrs()))
        return false;
    if (!map_user_ipc_el0_alias(uc, slot, l1, USER_HIGH_IPC_ALIAS))
        return false;

    /* Keep peripheral MMIO mapped for current direct-call kernel API ABI. */
    map_user_device_windows(l1);

    mmu_user_table_publish(uc, slot);
    user_table_valid[uc][slot].v = true;
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

    /* Kernel-side regions privileged (see mmu_user_table_build): required so the
     * scheduler can run its dispatch tail + ctx_switch on its own kernel stack
     * while TTBR0 points at this user table. EL0-inaccessible. */
    mmu_user_table_reset(uc, slot, l1);
    if (!mmu_user_slot_pages(core, slot, l1, slot_base, slot_size, code_bytes, true, false))
        return false;
    if (!map_user_identity_2m(uc, slot, l1, SHARED_FIFO_BASE, user_ram_nc_attrs()))
        return false;

    map_user_device_windows(l1);

    mmu_user_table_publish(uc, slot);
    user_table_valid[uc][slot].v = true;
    return true;
}

bool mmu_user_table_build_split_el0(u32 core, u32 slot, u64 slot_base, u64 slot_size, u32 code_bytes)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    if ((slot_base & (L3_PAGE_SIZE - 1)) != 0 || slot_size != PROC_SLOT_SIZE)
        return false;

    u32 uc = user_core_index(core);
    u64 *l1 = user_l1[uc][slot];

    mmu_user_table_reset(uc, slot, l1);
    if (!mmu_user_slot_pages(core, slot, l1, slot_base, slot_size, code_bytes, true, true))
        return false;
    if (!map_user_identity_2m(uc, slot, l1, SHARED_FIFO_BASE, user_ram_nc_attrs()))
        return false;
    if (!map_user_ipc_el0_alias(uc, slot, l1, USER_HIGH_IPC_ALIAS))
        return false;

    map_user_device_windows(l1);

    mmu_user_table_publish(uc, slot);
    user_table_valid[uc][slot].v = true;
    return true;
}

bool mmu_user_table_build_split_el0_at(u32 core, u32 slot, u64 va_base, u64 pa_base,
                                       u64 slot_size, u32 code_bytes)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    if ((va_base & (L3_PAGE_SIZE - 1)) != 0 || (pa_base & (L3_PAGE_SIZE - 1)) != 0 ||
        slot_size != PROC_SLOT_SIZE)
        return false;
    if (code_bytes == 0 || code_bytes > slot_size)
        return false;

    u32 uc = user_core_index(core);
    u64 *l1 = user_l1[uc][slot];

    mmu_user_table_reset(uc, slot, l1);

    /* Map the linked EL0 image window (va_base -> pa_base) into the high L1
     * region's OWN L2 table (user_l2_high), never the low/phys L2 -- so the slot
     * is reachable ONLY at its linked VA and is never aliased into the identity
     * regions. Fail closed if va_base is outside the supported high L1 region. */
    u32 l1idx = (u32)(va_base / L1_BLOCK_SIZE);
    if (l1idx != USER_L1_HIGH_INDEX)
        return false;
    u64 end = va_base + slot_size;
    if (end < va_base || (u32)((end - 1) / L1_BLOCK_SIZE) != l1idx)
        return false;
    u64 *l2 = user_l2_install(uc, slot, l1, l1idx);
    if (!l2)
        return false;
    u64 region_base = (u64)l1idx * L1_BLOCK_SIZE;

    simd_zero(user_l3_proc[uc][slot][0], 512 * sizeof(u64));
    simd_zero(user_l3_proc[uc][slot][1], 512 * sizeof(u64));
    u64 code_end = va_base + code_bytes;
    u64 data_start = (code_end + L3_PAGE_SIZE - 1) & ~(u64)(L3_PAGE_SIZE - 1);
    u64 first_l2 = (va_base - region_base) / L2_BLOCK_SIZE;
    for (u64 off = 0; off < slot_size; off += L3_PAGE_SIZE) {
        u64 va = va_base + off;
        u64 pa = pa_base + off;
        u64 idx = (va - region_base) / L2_BLOCK_SIZE;
        u32 table = (u32)(idx - first_l2);
        if (table >= 2)
            return false;
        if (l2[idx] == 0)
            l2[idx] = (u64)(usize)user_l3_proc[uc][slot][table] | PTE_VALID | PTE_TABLE;
        u64 attrs = (va < data_start) ? user_page_el0_rx_attrs() : user_page_el0_rw_xn_attrs();
        user_l3_proc[uc][slot][table][(va / L3_PAGE_SIZE) & 511U] =
            (pa & ~(L3_PAGE_SIZE - 1)) | attrs;
    }

    if (!map_user_identity_2m(uc, slot, l1, SHARED_FIFO_BASE, user_ram_nc_attrs()))
        return false;
    if (!map_user_ipc_el0_alias(uc, slot, l1, USER_HIGH_IPC_ALIAS))
        return false;
    map_user_device_windows(l1);
    mmu_user_table_publish(uc, slot);
    user_table_valid[uc][slot].v = true;
    return true;
}

bool mmu_switch_to_user(u32 core, u32 slot)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    u32 uc = user_core_index(core);
    if (!user_table_valid[uc][slot].v)
        return false;

    /* Tag this process's table with a distinct 8-bit ASID (TCR_EL1.AS=0,
     * the default here -- see TCR_VALUE -- selects 8-bit ASID in
     * TTBR0_EL1[55:48]). ASID 0 is reserved for the shared kernel table
     * (mmu_switch_to_kernel). Real physical addresses on this hardware
     * never reach bit 48+, so OR-ing the ASID into that field cannot
     * collide with the table's physical base.
     *
     * This is additive defense-in-depth, not a change to the isolation
     * boundary itself: mmu_invalidate_tlb() below still does a full
     * (all-ASID) invalidate on every switch, exactly as before, so
     * correctness never depended on ASID tagging. But an untagged switch
     * left every process's translations sharing ASID 0 -- if a future
     * change ever dropped or narrowed that flush (e.g. for a legitimate
     * perf optimization), stale cross-process TLB entries would have had
     * no ASID to distinguish them. Tagging now means that failure mode
     * would show up as one process's table walking into a completely
     * different physical mapping (an immediate, loud fault or wrong-data
     * bug), not a silent, exploitable stale-permission read/write. */
    u64 asid = (u64)(1U + uc * MAX_PROCS_PER_CORE + slot);
    u64 ttbr = (asid << 48) | (u64)(usize)user_l1[uc][slot];
    __asm__ volatile("msr ttbr0_el1, %0" :: "r"(ttbr));
    mmu_invalidate_tlb();
    return true;
}

void mmu_switch_to_kernel(void)
{
    /* ASID 0, reserved for the shared kernel table (see mmu_switch_to_user). */
    __asm__ volatile("msr ttbr0_el1, %0" :: "r"(shared_ttbr0));
    mmu_invalidate_tlb();
}

bool mmu_user_ipc_shm_window(u32 core, u32 slot, bool enable)
{
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    u32 uc = user_core_index(core);
    if (!user_table_valid[uc][slot].v)
        return false;

    /* IPC_SHM_BASE is identity-mapped (VA == PA) as a coarse 2 MiB NC block in
     * whichever L2 table backs its 1 GiB L1 region (L1[0] on Pi, L1[1] on QEMU),
     * not always user_l2_low. */
    u32 l1idx = (u32)(IPC_SHM_BASE / L1_BLOCK_SIZE);
    u32 idx = (u32)((IPC_SHM_BASE % L1_BLOCK_SIZE) / L2_BLOCK_SIZE);
    if (enable) {
        if (!map_user_identity_2m(uc, slot, user_l1[uc][slot], IPC_SHM_BASE,
                                  user_ram_nc_attrs()))
            return false;
    } else {
        u64 *l2 = user_l2_lookup(uc, slot, l1idx);
        if (!l2)
            return false;
        l2[idx] = 0;
    }

    if (core == core_id())
        mmu_invalidate_tlb();
    return true;
}

bool mmu_user_pte_snapshot(u32 core, u32 slot, u64 va, u64 *l1e, u64 *l2e, u64 *l3e)
{
    if (l1e) *l1e = 0;
    if (l2e) *l2e = 0;
    if (l3e) *l3e = 0;
    if (!is_user_core(core) || slot >= MAX_PROCS_PER_CORE)
        return false;
    u32 uc = user_core_index(core);
    if (!user_table_valid[uc][slot].v)
        return false;
    u32 l1idx = (u32)(va / L1_BLOCK_SIZE);
    if (l1idx >= 512)
        return false;
    if (l1e) *l1e = user_l1[uc][slot][l1idx];

    /* Walk through the L2 table this VA's L1 region actually references, not
     * always user_l2_low. Regions with no per-slot L2 (device windows) stop
     * here with l2e/l3e = 0. */
    u64 *l2 = user_l2_lookup(uc, slot, l1idx);
    if (!l2)
        return true;
    u32 l2idx = (u32)((va % L1_BLOCK_SIZE) / L2_BLOCK_SIZE);
    u64 e2 = l2[l2idx];
    if (l2e) *l2e = e2;
    if ((e2 & (PTE_VALID | PTE_TABLE)) != (PTE_VALID | PTE_TABLE))
        return true;

    /* Resolve which L3 table the L2 entry points at by matching its table base,
     * so slot pages, the linked EL0 image and the IPC alias are all reported
     * correctly regardless of which region/index they landed in. */
    u64 l3base = e2 & 0x0000FFFFFFFFF000ULL;
    u32 pidx = (u32)((va / L3_PAGE_SIZE) & 511U);
    if (l3base == (u64)(usize)user_l3_ipc[uc][slot]) {
        if (l3e) *l3e = user_l3_ipc[uc][slot][pidx];
    } else if (l3base == (u64)(usize)user_l3_proc[uc][slot][0]) {
        if (l3e) *l3e = user_l3_proc[uc][slot][0][pidx];
    } else if (l3base == (u64)(usize)user_l3_proc[uc][slot][1]) {
        if (l3e) *l3e = user_l3_proc[uc][slot][1][pidx];
    }
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
    const u64 line = 64;
    if (size == 0)
        return;
    u64 end = start + size;
    if (end < start)
        return;

    u64 first = start & ~(line - 1);
    u64 last = (end - 1) & ~(line - 1);
    if (first == last) {
        if (start == first && end == first + line) {
            __asm__ volatile("dc ivac, %0" :: "r"(first) : "memory");
        } else {
            /* Preserve dirty bytes belonging to adjacent objects that share
             * this partial line. Plain ivac would silently discard them. */
            __asm__ volatile("dc civac, %0" :: "r"(first) : "memory");
        }
        dsb();
        return;
    }

    u64 addr = first;
    if (start != first) {
        __asm__ volatile("dc civac, %0" :: "r"(addr) : "memory");
        addr += line;
    }

    u64 interior_end = (end & (line - 1)) ? last : end;
    while (addr < interior_end) {
        __asm__ volatile("dc ivac, %0" :: "r"(addr) : "memory");
        addr += line;
    }
    if (end & (line - 1))
        __asm__ volatile("dc civac, %0" :: "r"(last) : "memory");
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
