/*
 * pcie1.c - BCM2712 PCIe1 (FFC / HAT) root complex
 *
 * Brings up the x1 FFC root port. Must never touch pcie2/RP1 registers,
 * the RP1 outbound window, or RP1 inbound BAR1 (MIP0 MSI).
 *
 * BCM2712 init includes the Raspberry Pi Linux 54 MHz PHY sequence:
 *   - RC base 0x1000110000 (not 0x1000120000)
 *   - reset id 43 (pcie2 is 44)
 *   - 32 MiB Device ATU at CPU 0x1B80000000 -> PCIe 0x80000000
 *   - 2 MiB inbound NC arena only (#141); not 64 GiB -> PA 0
 *   - no BAR1 MIP0
 *   - MSI INTID 255/256 left masked (no handler yet)
 *
 * Firmware must have dtparam=pciex1. A powered riser is required; the
 * FFC is 5 V / 1 A and cannot feed a 70 W GPU.
 */
#include "pcie1.h"
#include "platform.h"
#include "mmio.h"
#include "mmu.h"
#include "uart.h"
#include "timer.h"

#if PIOS_HAS_PCIE1

_Static_assert(PIOS_PCIE1_CPU_WIN_BASE != 0x1F00000000UL,
               "pcie1 outbound window must not be the RP1 ATU");
_Static_assert((PIOS_PCIE1_CPU_WIN_BASE + PIOS_PCIE1_CPU_WIN_SIZE)
                   <= 0x1F00000000UL ||
               PIOS_PCIE1_CPU_WIN_BASE >= (0x1F00000000UL + 0x00800000UL),
               "pcie1 8 MiB ATU must not intersect RP1 8 MiB ATU");
_Static_assert(PIOS_PCIE1_CPU_WIN_SIZE == 0x02000000UL,
               "pcie1 ATU is 32 MiB for BAR0 MMIO; LMEM stays unmapped");
_Static_assert(PIOS_PCIE1_RESET_ID != 44U,
               "pcie1 must not assert pcie2/RP1 reset id 44");
_Static_assert(PIOS_DMA_PCIE1_SIZE == 0x00200000UL,
               "pcie1 inbound arena is 2 MiB");
_Static_assert(PIOS_DMA_PCIE1_BASE + PIOS_DMA_PCIE1_SIZE == PIOS_FB_BACK_BASE,
               "pcie1 DMA arena sits in the NC hole before FB_BACK");
_Static_assert(PIOS_IPC_SHM_BASE + PIOS_IPC_SHM_SIZE == PIOS_DMA_PCIE1_BASE,
               "pcie1 DMA arena follows IPC");
_Static_assert((PIOS_DMA_PCIE1_BASE + PIOS_DMA_PCIE1_SIZE) <= PIOS_DMA_NET_BASE ||
               PIOS_DMA_PCIE1_BASE >= (PIOS_DMA_NET_BASE + PIOS_DMA_NET_SIZE),
               "pcie1 DMA arena must not overlap DMA_NET");
_Static_assert((PIOS_DMA_PCIE1_BASE + PIOS_DMA_PCIE1_SIZE) <= PIOS_DMA_DISK_BASE ||
               PIOS_DMA_PCIE1_BASE >= (PIOS_DMA_DISK_BASE + PIOS_DMA_DISK_SIZE),
               "pcie1 DMA arena must not overlap DMA_DISK");
_Static_assert((PIOS_DMA_PCIE1_BASE + PIOS_DMA_PCIE1_SIZE) <= PIOS_PROC_ARENA_BASE ||
               PIOS_DMA_PCIE1_BASE >= (PIOS_PROC_ARENA_BASE + PIOS_PROC_ARENA_SIZE),
               "pcie1 DMA arena must not overlap process arena");

#define PCIE1_RC_BASE               PIOS_PCIE1_RC_BASE

#define RC_DL_MDIO_ADDR             0x1100
#define RC_DL_MDIO_WR_DATA          0x1104
#define RC_DL_MDIO_RD_DATA          0x1108
#define RC_PL_PHY_CTL_15            0x184C
#define MDIO_READ                   (1U << 20)
#define MDIO_DONE                   (1U << 31)
#define MDIO_BLOCK_SELECT           0x1FU
#define MDIO_POLL_US                10U
#define MDIO_TIMEOUT_US             100U
#define PHY_PM_CLK_PERIOD_MASK      0xFFU
#define PHY_PM_CLK_PERIOD_54MHZ     0x12U
#define MISC_MISC_CTRL              0x4008
#define MISC_CPU_2_PCIE_WIN0_LO     0x400C
#define MISC_CPU_2_PCIE_WIN0_HI     0x4010
#define MISC_PCIE_CTRL              0x4064
#define MISC_PCIE_STATUS            0x4068
#define MISC_CPU_2_PCIE_WIN0_BL     0x4070
#define MISC_CPU_2_PCIE_WIN0_BH     0x4080
#define MISC_CPU_2_PCIE_WIN0_LH     0x4084
#define MISC_UBUS_CTRL              0x40A4
#define MISC_UBUS_TIMEOUT           0x40A8
#define MISC_RC_CFG_RETRY_TIMEOUT   0x405C
#define MISC_AXI_READ_ERROR_DATA    0x4170
#define HARD_DEBUG                  0x4304
#define MISC_RC_BAR2_CONFIG_LO      0x4034
#define MISC_RC_BAR2_CONFIG_HI      0x4038
#define MISC_RC_BAR3_CONFIG_LO      0x403C
#define MISC_RC_BAR3_CONFIG_HI      0x4040
#define MISC_UBUS_BAR2_CONFIG_REMAP    0x40B4
#define MISC_UBUS_BAR2_CONFIG_REMAP_HI 0x40B8
#define MISC_UBUS_BAR3_CONFIG_REMAP    0x40BC
#define MISC_UBUS_BAR3_CONFIG_REMAP_HI 0x40C0
#define RC_CFG_VENDOR_SPECIFIC_REG1 0x0188
#define  VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK 0xC
#define EXT_CFG_DATA                0x8000
#define EXT_CFG_INDEX               0x9000
#define RGR1_SW_INIT_1              0x9210
#define RC_CFG_PRIV1_ID_VAL3        0x043C
#define PCI_REG_CMD                 0x04
#define PCI_REG_BUS_NUM             0x18
#define PCI_REG_MEM_BASE_LIMIT      0x20
#define PCI_REG_CAP_PTR             0x34
#define PCIE1_AER_UNCORR_ERR       0x04
#define PCIE1_AER_UNCORR_MASK      0x08
#define PCIE1_AER_CORR_ERR         0x10
#define PCIE1_AER_CORR_MASK        0x14
#define PCIE1_AER_HDR_LOG0         0x1C
#define PCIE1_AER_HDR_LOG1         0x20
#define PCIE1_AER_HDR_LOG2         0x24
#define PCIE1_AER_HDR_LOG3         0x28

#define STATUS_DL_ACTIVE            (1U << 5)
#define STATUS_PHYLINKUP            (1U << 4)
#define MCTRL_RCB_64B               (1U << 7)
#define MCTRL_RCB_MPS               (1U << 10)
#define MCTRL_SCB_ACCESS_EN         (1U << 12)
#define MCTRL_CFG_READ_UR           (1U << 13)
#define MCTRL_MAX_BURST_MASK        (0x3U << 20)
#define SERDES_IDDQ                 (1U << 27)
#define CTRL_PERSTB                 (1U << 2)
#define UBUS_REPLY_ERR_DIS          (1U << 13)
#define UBUS_REPLY_DECERR_DIS       (1U << 19)
#define SW_INIT_MASK                (1U << 1)
#define PCI_CMD_MEM                 (1U << 1)
#define PCI_CMD_MASTER              (1U << 2)

#define BCM_RESET_BASE              0x1001504318UL
#define SW_INIT_SET                 0x00
#define SW_INIT_CLEAR               0x04
#define SW_INIT_BANK_SIZE           0x18

static inline void pw(u32 off, u32 val) { mmio_write(PCIE1_RC_BASE + off, val); }
static inline u32  pr(u32 off)          { return mmio_read(PCIE1_RC_BASE + off); }

static bool g_inited;
static bool g_link_up;
static const char *g_fail = "not inited";
static struct pcie1_status g_snap;
static u32 pcie1_aer_offset_rc;

static void bridge_reset_brcm(bool assert)
{
    u32 id = PIOS_PCIE1_RESET_ID;
    u32 bit = 1U << (id & 0x1FU);
    u32 bank_off = (id >> 5) * SW_INIT_BANK_SIZE;
    if (assert)
        mmio_write(BCM_RESET_BASE + bank_off + SW_INIT_SET, bit);
    else
        mmio_write(BCM_RESET_BASE + bank_off + SW_INIT_CLEAR, bit);
    delay_cycles(100000);
}

static void perst_set(bool assert)
{
    u32 tmp = pr(MISC_PCIE_CTRL);
    if (assert)
        tmp &= ~CTRL_PERSTB;
    else
        tmp |= CTRL_PERSTB;
    pw(MISC_PCIE_CTRL, tmp);
    dmb();
}

/* #144: nop-spin is not a deadline. Poll link with the generic timer. */
static bool wait_link_ms(u32 budget_ms)
{
    u64 t0 = timer_monotonic_ms();
    if (pcie1_link_up())
        return true;
    while (timer_monotonic_ms() - t0 < (u64)budget_ms) {
        timer_delay_ms(1);
        if (pcie1_link_up())
            return true;
    }
    return pcie1_link_up();
}

static bool rc_alive(void)
{
    u32 id = pr(0);
    return id != 0U && id != 0xFFFFFFFFU;
}

static bool mdio_wait(u32 offset, bool done_set, u32 *value)
{
    for (u32 elapsed = 0U; elapsed <= MDIO_TIMEOUT_US;
         elapsed += MDIO_POLL_US) {
        u32 data = pr(offset);
        if (data != 0xFFFFFFFFU && ((data & MDIO_DONE) != 0U) == done_set) {
            *value = data & ~MDIO_DONE;
            return true;
        }
        if (elapsed < MDIO_TIMEOUT_US)
            timer_delay_us(MDIO_POLL_US);
    }
    uart_puts("[pcie1] MDIO timeout\n");
    return false;
}

static bool mdio_read(u8 reg, u32 *value)
{
    pw(RC_DL_MDIO_ADDR, MDIO_READ | reg);
    (void)pr(RC_DL_MDIO_ADDR);
    return mdio_wait(RC_DL_MDIO_RD_DATA, true, value);
}

static bool mdio_write(u8 reg, u16 value)
{
    u32 result;
    pw(RC_DL_MDIO_ADDR, reg);
    (void)pr(RC_DL_MDIO_ADDR);
    pw(RC_DL_MDIO_WR_DATA, MDIO_DONE | value);
    return mdio_wait(RC_DL_MDIO_WR_DATA, false, &result);
}

static bool setup_refclk_54mhz(void)
{
    /* Raspberry Pi Linux brcm_pcie_munge_pll(): BCM2712's 54 MHz XOSC.
     * Program only this RC's PHY, with PERST held and shared RESCAL untouched. */
    static const u8 regs[] = {0x16, 0x17, 0x18, 0x19, 0x1B, 0x1C, 0x1E};
    static const u16 values[] = {
        0x50B9, 0xBDA1, 0x0094, 0x97B4, 0x5030, 0x5030, 0x0007
    };
    if (!mdio_write(MDIO_BLOCK_SELECT, 0x1600U))
        return false;
    for (u32 i = 0U; i < sizeof(regs); i++) {
        u32 before, after;
        if (!mdio_read(regs[i], &before) ||
            !mdio_write(regs[i], values[i]) ||
            !mdio_read(regs[i], &after))
            return false;
        uart_puts("[pcie1] PLL ");
        uart_hex(regs[i]);
        uart_puts(" ");
        uart_hex(before);
        uart_puts(" -> ");
        uart_hex(after);
        uart_puts("\n");
        if (after != values[i]) {
            uart_puts("[pcie1] PLL readback mismatch\n");
            return false;
        }
    }
    timer_delay_us(100U);
    u32 phy = pr(RC_PL_PHY_CTL_15);
    if (phy == 0xFFFFFFFFU)
        return false;
    phy = (phy & ~PHY_PM_CLK_PERIOD_MASK) | PHY_PM_CLK_PERIOD_54MHZ;
    pw(RC_PL_PHY_CTL_15, phy);
    dsb();
    if (pr(RC_PL_PHY_CTL_15) != phy) {
        uart_puts("[pcie1] PHY clock readback mismatch\n");
        return false;
    }
    return true;
}

static void program_inbound_arena(void)
{
    u32 tmp;
    /* 2 MiB at PCIe 0x10_00000000 -> CPU DMA_PCIE1. Not 64 GiB -> PA 0. */
    pw(MISC_RC_BAR2_CONFIG_LO, PCIE1_BAR2_SIZE_ENC);
    pw(MISC_RC_BAR2_CONFIG_HI, (u32)(PCIE1_DMA_PCIE_BASE >> 32));
    dmb();
    pw(MISC_UBUS_BAR2_CONFIG_REMAP, (u32)PIOS_DMA_PCIE1_BASE | 1U);
    pw(MISC_UBUS_BAR2_CONFIG_REMAP_HI, (u32)(PIOS_DMA_PCIE1_BASE >> 32));
    dmb();
    pw(MISC_RC_BAR3_CONFIG_LO, 0);
    pw(MISC_RC_BAR3_CONFIG_HI, 0);
    pw(MISC_UBUS_BAR3_CONFIG_REMAP, 0);
    pw(MISC_UBUS_BAR3_CONFIG_REMAP_HI, 0);
    dmb();
    tmp = pr(MISC_MISC_CTRL);
    tmp &= ~(0x1FU << 27);
    tmp |= (PCIE1_BAR2_SIZE_ENC << 27);
    pw(MISC_MISC_CTRL, tmp);
    dmb();
}

static void set_outbound_win(u64 cpu_addr, u64 pcie_addr, u64 size)
{
    u64 cpu_mb   = cpu_addr / (1024 * 1024);
    u64 limit_mb = (cpu_addr + size - 1) / (1024 * 1024);
    pw(MISC_CPU_2_PCIE_WIN0_LO, (u32)pcie_addr);
    pw(MISC_CPU_2_PCIE_WIN0_HI, (u32)(pcie_addr >> 32));
    pw(MISC_CPU_2_PCIE_WIN0_BL,
       ((u32)(limit_mb & 0xFFF) << 20) | ((u32)(cpu_mb & 0xFFF) << 4));
    pw(MISC_CPU_2_PCIE_WIN0_BH, (u32)(cpu_mb >> 12));
    pw(MISC_CPU_2_PCIE_WIN0_LH, (u32)(limit_mb >> 12));
    dmb();
}

bool pcie1_set_outbound_window(u64 size)
{
    u32 mem_window;
    if (!g_link_up || size < 0x00100000ULL ||
        size > PIOS_PCIE1_CPU_WIN_SIZE ||
        (size & (size - 1ULL)) != 0ULL ||
        !pcie1_bridge_mem_window(PIOS_PCIE1_PCI_WIN_BASE, size, &mem_window))
        return false;
    set_outbound_win(PIOS_PCIE1_CPU_WIN_BASE, PIOS_PCIE1_PCI_WIN_BASE, size);
    pw(PCI_REG_MEM_BASE_LIMIT, mem_window);
    dsb();
    return pr(PCI_REG_MEM_BASE_LIMIT) == mem_window;
}

bool pcie1_enable_memory_path(u32 target_bus)
{
    if (!g_link_up)
        return false;
    for (u32 i = 0; i < g_snap.ep_count; i++) {
        const struct pcie1_ep *e = &g_snap.eps[i];
        if (!pcie1_is_bridge(e->hdr_type) ||
            target_bus < e->sec_bus || target_bus > e->sub_bus)
            continue;
        u32 cmd = pcie1_cfg_read(e->bus, e->dev, e->func, PCI_REG_CMD);
        cmd = (cmd | PCI_CMD_MEM) & ~PCI_CMD_MASTER;
        pcie1_cfg_write(e->bus, e->dev, e->func, PCI_REG_CMD, cmd);
    }
    return true;
}

bool pcie1_dma_prepare_to_device(const void *ptr, u64 len, u64 *pci_addr)
{
    if (!pcie1_dma_addr(ptr, len, pci_addr))
        return false;
    dcache_clean_range((u64)(usize)ptr, len);
    dsb();
    return true;
}

bool pcie1_dma_prepare_from_device(void *ptr, u64 len, u64 *pci_addr)
{
    if (!pcie1_dma_addr(ptr, len, pci_addr))
        return false;
    dcache_invalidate_range((u64)(usize)ptr, len);
    dsb();
    return true;
}

bool pcie1_dma_complete_from_device(void *ptr, u64 len)
{
    u64 pci_addr;
    if (!pcie1_dma_addr(ptr, len, &pci_addr))
        return false;
    (void)pci_addr;
    dcache_invalidate_range((u64)(usize)ptr, len);
    dsb();
    return true;
}

static bool cap_set_gen2(void)
{
    u32 cap = pr(PCI_REG_CAP_PTR) & 0xFFU;
    for (u32 i = 0; i < 48U && cap >= 0x40U && cap <= 0xFCU &&
         (cap & 3U) == 0U; i++) {
        u32 hdr = pr(cap);
        if ((hdr & 0xFFU) == PCIE1_LINK_CAP_ID) {
            if (cap > 0xCCU)
                return false;
            /* Match Linux: limit both advertised and requested link speed. */
            u32 lcap = pr(cap + 0x0CU);
            lcap = (lcap & ~0xFU) | 2U;
            pw(cap + 0x0CU, lcap);
            u16 lc2 = mmio_read16(PCIE1_RC_BASE + cap + 0x30U);
            lc2 = (lc2 & ~0xFU) | 2U;
            mmio_write16(PCIE1_RC_BASE + cap + 0x30U, lc2);
            dsb();
            return (pr(cap + 0x0CU) & 0xFU) == 2U &&
                   (mmio_read16(PCIE1_RC_BASE + cap + 0x30U) & 0xFU) == 2U;
        }
        cap = (hdr >> 8) & 0xFFU;
        if (cap == 0U)
            break;
    }
    return false;
}

static u16 cap_link_status(void)
{
    u32 cap = pr(PCI_REG_CAP_PTR) & 0xFFU;
    for (u32 i = 0; i < 48U && cap >= 0x40U; i++) {
        u32 hdr = pr(cap);
        if ((hdr & 0xFFU) == PCIE1_LINK_CAP_ID)
            return (u16)(pr(cap + 0x10U) >> 16);
        cap = (hdr >> 8) & 0xFFU;
        if (cap == 0U)
            return 0;
    }
    return 0;
}

u32 pcie1_cfg_read(u32 bus, u32 dev, u32 func, u32 reg)
{
    if (bus == 0U && dev == 0U && func == 0U)
        return reg <= 0xFFCU && (reg & 3U) == 0U ? pr(reg) : 0xFFFFFFFFU;
    if (!g_link_up || !pcie1_cfg_addr_valid(bus, dev, func, reg))
        return 0xFFFFFFFFU;
    pw(EXT_CFG_INDEX, (bus << 20) | (dev << 15) | (func << 12));
    dmb();
    return pr(EXT_CFG_DATA + reg);
}

void pcie1_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val)
{
    if (bus == 0U && dev == 0U && func == 0U) {
        if (reg > 0xFFCU || (reg & 3U) != 0U)
            return;
        pw(reg, val);
        dmb();
        return;
    }
    if (!g_link_up || !pcie1_cfg_addr_valid(bus, dev, func, reg))
        return;
    pw(EXT_CFG_INDEX, (bus << 20) | (dev << 15) | (func << 12));
    dmb();
    pw(EXT_CFG_DATA + reg, val);
    dmb();
}

bool pcie1_link_up(void)
{
    u32 st = pr(MISC_PCIE_STATUS);
    return st != 0xFFFFFFFFU && (st & STATUS_DL_ACTIVE) &&
           (st & STATUS_PHYLINKUP);
}

static bool record_bridge_range(bool reachable[PCIE1_SCAN_BUS_HI + 1U],
                                u32 parent_bus, const struct pcie1_ep *e)
{
    if (!e || !pcie1_is_bridge(e->hdr_type))
        return true;
    if (!pcie1_bridge_range_valid(parent_bus, e->sec_bus, e->sub_bus))
        return false;
    for (u32 bus = e->sec_bus; bus <= e->sub_bus; bus++)
        reachable[bus] = true;
    return true;
}

static bool record_function(struct pcie1_status *s,
                            bool reachable[PCIE1_SCAN_BUS_HI + 1U],
                            u32 bus, u32 dev, u32 func)
{
    u32 cfg0, cfg8, cfgc, cfg18;
    struct pcie1_ep *e;
    if (s->ep_count >= PCIE1_SCAN_MAX) {
        s->scan_truncated = true;
        return false;
    }
    cfg0 = pcie1_cfg_read(bus, dev, func, 0x00);
    if (!pcie1_id_valid(cfg0))
        return true;
    cfg8 = pcie1_cfg_read(bus, dev, func, 0x08);
    cfgc = pcie1_cfg_read(bus, dev, func, 0x0C);
    cfg18 = pcie1_cfg_read(bus, dev, func, 0x18);
    e = &s->eps[s->ep_count];
    pcie1_fill_ep(e, (u8)bus, (u8)dev, (u8)func, cfg0, cfg8, cfgc, cfg18);
    if (!record_bridge_range(reachable, bus, e))
        s->malformed_topology = true;
    if (s->ep_count == 0) {
        s->first_vendor = e->vendor;
        s->first_device = e->device;
    }
    if (pcie1_is_b50(e->vendor, e->device)) {
        s->b50_found = true;
        s->b50_vendor = e->vendor;
        s->b50_device = e->device;
    }
    s->ep_count++;
    return true;
}

static void scan_endpoints(struct pcie1_status *s)
{
    bool reachable[PCIE1_SCAN_BUS_HI + 1U] = {0};
    s->ep_count = 0;
    s->first_vendor = 0;
    s->first_device = 0;
    s->b50_found = false;
    s->b50_vendor = 0;
    s->b50_device = 0;
    s->scan_truncated = false;
    s->malformed_topology = false;
    reachable[PCIE1_SCAN_BUS_LO] = true;
    for (u32 bus = PCIE1_SCAN_BUS_LO; bus <= PCIE1_SCAN_BUS_HI; bus++) {
        if (!reachable[bus])
            continue;
        for (u32 dev = 0; dev <= PCIE1_SCAN_DEV_HI; dev++) {
            u32 cfg0 = pcie1_cfg_read(bus, dev, 0, 0);
            u32 cfgc;
            u32 func;
            u32 nfunc;
            if (!pcie1_id_valid(cfg0))
                continue;
            cfgc = pcie1_cfg_read(bus, dev, 0, 0x0C);
            nfunc = pcie1_hdr_multifunction((u8)((cfgc >> 16) & 0xFFU))
                        ? 8U : 1U;
            for (func = 0; func < nfunc; func++) {
                if (s->ep_count >= PCIE1_SCAN_MAX) {
                    s->scan_truncated = true;
                    return;
                }
                (void)record_function(s, reachable, bus, dev, func);
            }
        }
    }
}

static void publish_link(const char *fail)
{
    g_fail = fail;
    g_snap.present = true;
    g_snap.inited = g_inited;
    g_snap.link_up = g_link_up;
    g_snap.rc_status = pr(MISC_PCIE_STATUS);
    g_snap.link_status = cap_link_status();
    g_snap.link_speed = pcie1_link_speed(g_snap.link_status);
    g_snap.link_width = pcie1_link_width(g_snap.link_status);
    g_snap.fail_reason = fail;
}

static void publish_snap(const char *fail)
{
    publish_link(fail);
    if (g_link_up)
        scan_endpoints(&g_snap);
    else {
        g_snap.ep_count = 0U;
        g_snap.first_vendor = 0U;
        g_snap.first_device = 0U;
        g_snap.b50_found = false;
        g_snap.b50_vendor = 0U;
        g_snap.b50_device = 0U;
        g_snap.scan_truncated = false;
        g_snap.malformed_topology = false;
    }
}

bool pcie1_init(void)
{
    u32 tmp;
    u32 i;

    g_inited = false;
    g_link_up = false;
    g_snap = (struct pcie1_status){0};
    pcie1_aer_offset_rc = 0U;
    g_fail = "init";

    /* RESCAL is shared with pcie2/RP1 and already ran in pcie_init().
     * Do not re-assert it while the RP1 link is live. */
    bridge_reset_brcm(true);
    timer_delay_ms(1);
    bridge_reset_brcm(false);

    /* #144: first RC MMIO. If firmware did not enable pciex1 the load may
     * still hang the fabric — skip long waits when the ID is already dead. */
    if (!rc_alive()) {
        g_inited = true;
        publish_link("rc absent (need dtparam=pciex1)");
        uart_puts("[pcie1] RC ID absent; skip (dtparam=pciex1)\n");
        return false;
    }

    /* Assert PERST before touching link configuration so a warm endpoint
     * cannot train from stale LTSSM state. */
    perst_set(true);
    dsb();
    timer_delay_ms(20);

    tmp = pr(HARD_DEBUG);
    tmp &= ~SERDES_IDDQ;
    pw(HARD_DEBUG, tmp);
    dmb();
    timer_delay_ms(1);

    if (!setup_refclk_54mhz()) {
        g_inited = true;
        publish_link("54MHz PHY setup failed");
        uart_puts("[pcie1] PHY setup failed; PERST held\n");
        return false;
    }
    g_snap.phy_ready = true;

    tmp = pr(MISC_MISC_CTRL);
    tmp |= MCTRL_SCB_ACCESS_EN;
    tmp |= MCTRL_CFG_READ_UR;
    tmp &= ~MCTRL_MAX_BURST_MASK;
    tmp |= (1U << 20);
    tmp |= MCTRL_RCB_MPS;
    tmp |= MCTRL_RCB_64B;
    pw(MISC_MISC_CTRL, tmp);
    dmb();

    program_inbound_arena();

    tmp = pr(MISC_UBUS_CTRL);
    tmp |= UBUS_REPLY_ERR_DIS | UBUS_REPLY_DECERR_DIS;
    pw(MISC_UBUS_CTRL, tmp);
    pw(MISC_AXI_READ_ERROR_DATA, 0xFFFFFFFF);
    pw(MISC_UBUS_TIMEOUT, 0x0B2D0000);
    pw(MISC_RC_CFG_RETRY_TIMEOUT, 0x0ABA0000);

    tmp = pr(RC_CFG_PRIV1_ID_VAL3);
    tmp &= ~0xFFFFFF;
    tmp |= 0x060400;
    pw(RC_CFG_PRIV1_ID_VAL3, tmp);
    dmb();

    /* Linux's pcie1 non-prefetchable range: CPU 0x1B80000000 ->
     * PCIe 0x80000000. BAR0 only; do not map the prefetch/LMEM range. */
    set_outbound_win(PIOS_PCIE1_CPU_WIN_BASE, PIOS_PCIE1_PCI_WIN_BASE,
                     PIOS_PCIE1_CPU_WIN_SIZE);
    {
        u32 mem_window;
        if (!pcie1_bridge_mem_window(PIOS_PCIE1_PCI_WIN_BASE,
                                     PIOS_PCIE1_CPU_WIN_SIZE, &mem_window)) {
            g_inited = true;
            publish_link("root PCI memory window invalid");
            return false;
        }
        pw(PCI_REG_MEM_BASE_LIMIT, mem_window);
    }

    tmp = pr(RC_CFG_VENDOR_SPECIFIC_REG1);
    tmp &= ~VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK;
    pw(RC_CFG_VENDOR_SPECIFIC_REG1, tmp);
    dmb();

    if (!cap_set_gen2()) {
        g_inited = true;
        publish_link("Gen2 capability setup failed");
        uart_puts("[pcie1] Gen2 setup failed; PERST held\n");
        return false;
    }
    dmb();

    dsb();
    perst_set(false);
    dsb();
    /* CEM requires 100 ms after PERST release before configuration access.
     * Keep the existing 200 ms total bound, without watchdog pets. */
    timer_delay_ms(100U);
    g_link_up = wait_link_ms(100U);
    if (!g_link_up) {
        g_inited = true;
        publish_snap("no link (dtparam=pciex1 + powered FFC riser?)");
        uart_puts("[pcie1] link down sts=");
        uart_hex(pr(MISC_PCIE_STATUS));
        uart_puts("\n");
        return false;
    }

    pw(PCI_REG_BUS_NUM, 0x00FF0100);  /* pri 0, sec 1, sub 255 for a riser switch */
    dmb();
    tmp = pr(PCI_REG_CMD);
    tmp |= PCI_CMD_MEM | PCI_CMD_MASTER;
    pw(PCI_REG_CMD, tmp);
    dmb();

    pcie1_aer_init();
    g_inited = true;
    publish_snap("ok");
    uart_puts("[pcie1] link up x");
    uart_hex(g_snap.link_width);
    uart_puts(" gen");
    uart_hex(g_snap.link_speed);
    uart_puts(" eps=");
    uart_hex(g_snap.ep_count);
    uart_puts(" id=");
    uart_hex(((u32)g_snap.first_device << 16) | g_snap.first_vendor);
    uart_puts(" b50=");
    uart_hex(g_snap.b50_found ? 1U : 0U);
    uart_puts("\n");
    for (i = 0; i < g_snap.ep_count; i++) {
        const struct pcie1_ep *e = &g_snap.eps[i];
        uart_puts("[pcie1] ");
        uart_hex(e->bus);
        uart_puts(":");
        uart_hex(e->dev);
        uart_puts(".");
        uart_hex(e->func);
        uart_puts(" ");
        uart_puts(pcie1_hdr_kind(e->hdr_type));
        uart_puts(" id=");
        uart_hex(((u32)e->device << 16) | e->vendor);
        uart_puts(" ");
        uart_puts(pcie1_class_label(e->base_class, e->subclass));
        uart_puts("\n");
    }
    uart_puts("[pcie1] MSI INTID 255/256 masked (no handler yet)\n");
    return true;
}

static u32 pcie1_find_aer_cap(u32 bus, u32 dev, u32 fn)
{
    u32 off = 0x100U;
    for (u32 i = 0; i < 48U && off >= 0x100U; i++) {
        u32 hdr = pcie1_cfg_read(bus, dev, fn, off);
        if ((hdr & 0xFFFFU) == 0x0001U)
            return off;
        off = (hdr >> 20) & 0xFFCU;
        if (off == 0U)
            break;
    }
    return 0U;
}

void pcie1_aer_init(void)
{
    pcie1_aer_offset_rc = pcie1_find_aer_cap(0, 0, 0);
    if (!pcie1_aer_offset_rc) {
        uart_puts("[pcie1] no AER\n");
        return;
    }
    uart_puts("[pcie1] AER @RC=");
    uart_hex(pcie1_aer_offset_rc);
    uart_puts("\n");
    pcie1_cfg_write(0, 0, 0, pcie1_aer_offset_rc + PCIE1_AER_UNCORR_ERR,
                    0xFFFFFFFFU);
    pcie1_cfg_write(0, 0, 0, pcie1_aer_offset_rc + PCIE1_AER_CORR_ERR,
                    0xFFFFFFFFU);
    pcie1_cfg_write(0, 0, 0, pcie1_aer_offset_rc + PCIE1_AER_UNCORR_MASK, 0U);
    pcie1_cfg_write(0, 0, 0, pcie1_aer_offset_rc + PCIE1_AER_CORR_MASK, 0U);
}

void pcie1_aer_dump(const char *tag)
{
    u32 uncorr, corr;
    if (!pcie1_aer_offset_rc)
        return;
    uncorr = pcie1_cfg_read(0, 0, 0,
                            pcie1_aer_offset_rc + PCIE1_AER_UNCORR_ERR);
    corr = pcie1_cfg_read(0, 0, 0,
                          pcie1_aer_offset_rc + PCIE1_AER_CORR_ERR);
    uart_puts("[pcie1-aer] ");
    uart_puts(tag ? tag : "snapshot");
    uart_puts(" uncorr=");
    uart_hex(uncorr);
    uart_puts(" corr=");
    uart_hex(corr);
    if (uncorr || corr) {
        uart_puts("\n[pcie1-aer] HDR: ");
        uart_hex(pcie1_cfg_read(0, 0, 0,
                                pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG0));
        uart_puts(" ");
        uart_hex(pcie1_cfg_read(0, 0, 0,
                                pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG1));
        uart_puts(" ");
        uart_hex(pcie1_cfg_read(0, 0, 0,
                                pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG2));
        uart_puts(" ");
        uart_hex(pcie1_cfg_read(0, 0, 0,
                                pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG3));
        pcie1_cfg_write(0, 0, 0,
                        pcie1_aer_offset_rc + PCIE1_AER_UNCORR_ERR,
                        0xFFFFFFFFU);
        pcie1_cfg_write(0, 0, 0,
                        pcie1_aer_offset_rc + PCIE1_AER_CORR_ERR,
                        0xFFFFFFFFU);
    }
    uart_puts("\n");
}

void pcie1_aer_snapshot(struct pcie1_aer_snapshot *out, bool clear)
{
    if (!out)
        return;
    out->aer_offset = pcie1_aer_offset_rc;
    out->uncorr = 0;
    out->corr = 0;
    out->hdr0 = 0;
    out->hdr1 = 0;
    out->hdr2 = 0;
    out->hdr3 = 0;
    if (!pcie1_aer_offset_rc)
        return;
    out->uncorr = pcie1_cfg_read(0, 0, 0,
                                 pcie1_aer_offset_rc + PCIE1_AER_UNCORR_ERR);
    out->corr = pcie1_cfg_read(0, 0, 0,
                               pcie1_aer_offset_rc + PCIE1_AER_CORR_ERR);
    out->hdr0 = pcie1_cfg_read(0, 0, 0,
                               pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG0);
    out->hdr1 = pcie1_cfg_read(0, 0, 0,
                               pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG1);
    out->hdr2 = pcie1_cfg_read(0, 0, 0,
                               pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG2);
    out->hdr3 = pcie1_cfg_read(0, 0, 0,
                               pcie1_aer_offset_rc + PCIE1_AER_HDR_LOG3);
    if (clear) {
        pcie1_cfg_write(0, 0, 0,
                        pcie1_aer_offset_rc + PCIE1_AER_UNCORR_ERR,
                        0xFFFFFFFFU);
        pcie1_cfg_write(0, 0, 0,
                        pcie1_aer_offset_rc + PCIE1_AER_CORR_ERR,
                        0xFFFFFFFFU);
    }
}

void pcie1_status(struct pcie1_status *out)
{
    if (!out)
        return;
    /* Cached snapshot only. Config enum is MMIO; do not rescan from the
     * dashboard refresh. `pcie1 scan` calls pcie1_rescan(). */
    if (g_inited) {
        u32 st = pr(MISC_PCIE_STATUS);
        g_snap.rc_status = st;
        g_link_up = (st & STATUS_DL_ACTIVE) && (st & STATUS_PHYLINKUP);
        g_snap.link_up = g_link_up;
        g_snap.inited = true;
        g_snap.present = true;
    }
    *out = g_snap;
    out->present = true;
    if (!out->fail_reason)
        out->fail_reason = g_fail ? g_fail : "not inited";
}

void pcie1_rescan(void)
{
    if (!g_inited)
        return;
    g_link_up = pcie1_link_up();
    publish_snap(g_link_up ? "ok" : "link lost");
}

#else /* !PIOS_HAS_PCIE1 */

bool pcie1_init(void) { return false; }
bool pcie1_link_up(void) { return false; }
u32  pcie1_cfg_read(u32 bus, u32 dev, u32 func, u32 reg)
{
    (void)bus; (void)dev; (void)func; (void)reg;
    return 0xFFFFFFFFU;
}
void pcie1_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val)
{
    (void)bus; (void)dev; (void)func; (void)reg; (void)val;
}
void pcie1_status(struct pcie1_status *out)
{
    if (!out)
        return;
    *out = (struct pcie1_status){0};
    out->fail_reason = "not on this platform";
}

void pcie1_rescan(void) {}
bool pcie1_set_outbound_window(u64 size)
{
    (void)size;
    return false;
}
bool pcie1_enable_memory_path(u32 target_bus)
{
    (void)target_bus;
    return false;
}
bool pcie1_dma_prepare_to_device(const void *ptr, u64 len, u64 *pci_addr)
{
    (void)ptr; (void)len; (void)pci_addr;
    return false;
}
bool pcie1_dma_prepare_from_device(void *ptr, u64 len, u64 *pci_addr)
{
    (void)ptr; (void)len; (void)pci_addr;
    return false;
}
bool pcie1_dma_complete_from_device(void *ptr, u64 len)
{
    (void)ptr; (void)len;
    return false;
}
void pcie1_aer_init(void) {}
void pcie1_aer_dump(const char *tag) { (void)tag; }
void pcie1_aer_snapshot(struct pcie1_aer_snapshot *out, bool clear)
{
    (void)clear;
    if (out)
        *out = (struct pcie1_aer_snapshot){0};
}

#endif
