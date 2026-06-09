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
static u64 l3_block0[512] ALIGNED(4096);     /* block 0 (0-2MB) split to 4KB pages */

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

/* Helper: create a 2MB L2 block entry for Normal Non-Cacheable RAM.
 * Bit pattern (addr | 0x705) is identical to the NC block start.S installs
 * at l1_table[0], so NC regions keep their exact current attributes. */
static inline u64 ram_block_2m_nc(u64 addr) {
    return addr | PTE_VALID | PTE_BLOCK | PTE_AF |
           PTE_SH_INNER | PTE_ATTR(MT_NORMAL_NC) | PTE_AP_RW_EL1;
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

/*
 * mmu_enable_caching() — Phase 1 cache-coherency cleanup.
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
 * Safe because: dma.c already cleans/invalidates its DMA targets, the HDMI
 * blit is NEON (CPU), SD is PIO, and the process loader cleans D-cache to
 * PoC + invalidates I-cache after copying images. Call once on core 0 after
 * the start.S MMU handoff and before secondaries launch (they pick up the
 * rebuilt l1_table via shared_ttbr0).
 */
void mmu_enable_caching(void) {
    const u64 cache_lo_base = CORE0_RAM_BASE;                /* 0x00800000 */
    const u64 cache_lo_end  = SHARED_FIFO_BASE;              /* 0x04800000 */
    const u64 cache_fb_base = FB_BACK_BASE;                  /* 0x05000000 */
    const u64 cache_fb_end  = FB_BACK_BASE + FB_BACK_SIZE;   /* 0x06000000 */

    /* Phase 2a: split block 0 (0x0-0x1FFFFF) into 4KB pages so the kernel's
     * code + read-only data + initialised data window [__text_start,__bss_start)
     * becomes Write-Back cacheable + Inner Shareable. Instruction fetch and
     * .rodata reads (font glyph bitmaps, const tables) then hit the L1/L2 caches
     * instead of stalling ~250ns/line on NC DRAM — this is what makes the HDMI
     * dashboard render (instruction-fetch bound) fast. Everything else in block
     * 0 — the low 512KB below the kernel and all of .bss (DMA descriptor
     * rings/buffers, page tables, stacks) — stays Non-Cacheable, so no DMA
     * master loses coherency and the page-table walker keeps reading NC tables. */
    extern char __text_start[];
    extern char __bss_start[];
    const u64 code_lo = (u64)(usize)__text_start & ~(L3_PAGE_SIZE - 1);
    const u64 code_hi = (u64)(usize)__bss_start  & ~(L3_PAGE_SIZE - 1);
    for (u32 i = 0; i < 512; i++) {
        u64 addr = (u64)i * L3_PAGE_SIZE;
        u64 attr = (addr >= code_lo && addr < code_hi)
                       ? PTE_ATTR(MT_NORMAL) : PTE_ATTR(MT_NORMAL_NC);
        l3_block0[i] = addr | PTE_VALID | PTE_PAGE | PTE_AF |
                       PTE_SH_INNER | attr | PTE_AP_RW_EL1;
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

    /* l2_table_low and l3_block0 live in NC-mapped .bss, so the stores above
     * land directly in DRAM. Push to PoC and drop any stray cached lines so the
     * cacheable table walker (TCR IRGN0/ORGN0=WB) cannot read a stale copy. */
    dsb();
    dcache_clean_invalidate_range((u64)(usize)l3_block0, sizeof(l3_block0));
    dcache_clean_invalidate_range((u64)(usize)l2_table_low, sizeof(l2_table_low));

    /* Atomically replace the 1GB NC block with a table descriptor. The old
     * block translation stays live in the TLB until the tlbi below, so there
     * is no unmapped window for the kernel executing this code. */
    l1_table[0] = (u64)(usize)l2_table_low | PTE_VALID | PTE_TABLE;
    dsb();

    /* The walker had cached the old l1_table[0] block descriptor. Invalidate
     * (never clean) that line so the walk re-fetches the new table descriptor
     * from DRAM; the line is clean, so invalidate-only cannot write back and
     * clobber the value just stored via the NC mapping. */
    dcache_invalidate_range((u64)(usize)&l1_table[0], sizeof(u64));

    __asm__ volatile("tlbi vmalle1; dsb sy; isb" ::: "memory");
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

    /* Map ALL kernel RAM [0, CORE0_RAM_BASE) privileged (AP_RW_EL1, EL0 no
     * access). The scheduler keeps executing kernel code with TTBR0 pointing at
     * this user table between mmu_switch_to_user() and ctx_switch(), so it must
     * be able to reach its own stack, procs[], scheduler_ctx and preempt state
     * (all live well above the old 2MB window, up to ~5.6MB). These blocks are
     * EL0-inaccessible, so the user process gains nothing: isolation preserved. */
    const u64 ram_attrs = user_ram_attrs();
    for (u32 b = 0; b < (u32)(CORE0_RAM_BASE / L2_BLOCK_SIZE); b++)
        map_user_low_2m(l2, b, (u64)b * L2_BLOCK_SIZE, ram_attrs);

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

    /* Map ALL kernel RAM [0, CORE0_RAM_BASE) privileged (see mmu_user_table_build).
     * Required so the scheduler can run its dispatch tail + ctx_switch on its own
     * (high) kernel stack while TTBR0 points at this user table. EL0-inaccessible. */
    const u64 ram_attrs = user_ram_attrs();
    for (u32 b = 0; b < (u32)(CORE0_RAM_BASE / L2_BLOCK_SIZE); b++)
        map_user_low_2m(l2, b, (u64)b * L2_BLOCK_SIZE, ram_attrs);
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
