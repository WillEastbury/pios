/*
 * pcie1.c - BCM2712 PCIe1 (FFC / HAT) root complex
 *
 * Brings up the x1 FFC root port. Must never touch pcie2/RP1 registers,
 * the RP1 outbound window, or RP1 inbound BAR1 (MIP0 MSI).
 *
 * Init mirrors src/pcie.c's BCM2712 sequence with:
 *   - RC base 0x1000110000 (not 0x1000120000)
 *   - reset id 43 (pcie2 is 44)
 *   - 32 MiB Device ATU at 0x1B00000000 (BAR0 MMIO; not LMEM, not RP1)
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
#define PCI_REG_CAP_PTR             0x34

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
}

static void cap_set_gen2(void)
{
    u32 cap = pr(PCI_REG_CAP_PTR) & 0xFFU;
    for (u32 i = 0; i < 48U && cap >= 0x40U; i++) {
        u32 hdr = pr(cap);
        if ((hdr & 0xFFU) == PCIE1_LINK_CAP_ID) {
            /* Link Control 2: target link speed = 2 (5.0 GT/s, official x1). */
            u32 lc2 = pr(cap + 0x30U);
            lc2 = (lc2 & ~0xFU) | 2U;
            pw(cap + 0x30U, lc2);
            return;
        }
        cap = (hdr >> 8) & 0xFFU;
        if (cap == 0U)
            return;
    }
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
    if (bus == 0 && dev == 0 && func == 0)
        return pr(reg & 0xFFC);
    if (!g_link_up)
        return 0xFFFFFFFFU;
    pw(EXT_CFG_INDEX, (bus << 20) | (dev << 15) | (func << 12));
    dmb();
    return pr(EXT_CFG_DATA + (reg & 0xFFC));
}

void pcie1_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val)
{
    if (bus == 0 && dev == 0 && func == 0) {
        pw(reg & 0xFFC, val);
        dmb();
        return;
    }
    if (!g_link_up)
        return;
    pw(EXT_CFG_INDEX, (bus << 20) | (dev << 15) | (func << 12));
    dmb();
    pw(EXT_CFG_DATA + (reg & 0xFFC), val);
    dmb();
}

bool pcie1_link_up(void)
{
    u32 st = pr(MISC_PCIE_STATUS);
    return (st & STATUS_DL_ACTIVE) && (st & STATUS_PHYLINKUP);
}

static void record_function(struct pcie1_status *s, u32 bus, u32 dev, u32 func)
{
    u32 cfg0, cfg8, cfgc, cfg18;
    struct pcie1_ep *e;
    if (s->ep_count >= PCIE1_SCAN_MAX) {
        s->scan_truncated = true;
        return;
    }
    cfg0 = pcie1_cfg_read(bus, dev, func, 0x00);
    if (!pcie1_id_valid(cfg0))
        return;
    cfg8 = pcie1_cfg_read(bus, dev, func, 0x08);
    cfgc = pcie1_cfg_read(bus, dev, func, 0x0C);
    cfg18 = pcie1_cfg_read(bus, dev, func, 0x18);
    e = &s->eps[s->ep_count];
    pcie1_fill_ep(e, (u8)bus, (u8)dev, (u8)func, cfg0, cfg8, cfgc, cfg18);
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
}

static void scan_endpoints(struct pcie1_status *s)
{
    s->ep_count = 0;
    s->first_vendor = 0;
    s->first_device = 0;
    s->b50_found = false;
    s->b50_vendor = 0;
    s->b50_device = 0;
    s->scan_truncated = false;
    for (u32 bus = PCIE1_SCAN_BUS_LO; bus <= PCIE1_SCAN_BUS_HI; bus++) {
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
                record_function(s, bus, dev, func);
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
}

bool pcie1_init(void)
{
    u32 tmp;
    u32 i;

    g_inited = false;
    g_link_up = false;
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

    tmp = pr(HARD_DEBUG);
    tmp &= ~SERDES_IDDQ;
    pw(HARD_DEBUG, tmp);
    dmb();
    timer_delay_ms(1);

    perst_set(true);
    timer_delay_ms(20);

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

    /* Device ATU: CPU 0x1B00000000, 32 MiB, PCIe 0. BAR0 only. Not RP1. */
    set_outbound_win(PIOS_PCIE1_CPU_WIN_BASE, 0x00000000UL,
                     PIOS_PCIE1_CPU_WIN_SIZE);

    tmp = pr(RC_CFG_VENDOR_SPECIFIC_REG1);
    tmp &= ~VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK;
    pw(RC_CFG_VENDOR_SPECIFIC_REG1, tmp);
    dmb();

    cap_set_gen2();
    dmb();

    perst_set(false);
    /* #144: 200 ms deadline, 1 ms polls. Do not pet the watchdog. */
    g_link_up = wait_link_ms(200);
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
    publish_snap(g_link_up ? "ok" : (g_fail ? g_fail : "no link"));
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

#endif
