/*
 * pcie.c - BCM2712 PCIe Root Complex driver
 *
 * Brings up PCIe2 (quad-lane) link to the RP1 southbridge.
 *
 * Init sequence (from Linux pcie-brcmstb.c, BCM2712 variant):
 *   1. Assert bridge + PERST# reset
 *   2. Deassert bridge, power up SerDes
 *   3. Configure controller: MISC_CTRL, class code, bus numbers
 *   4. Program outbound ATU window (CPU → PCIe address translation)
 *   5. Configure UBUS/AXI bridge and timeouts
 *   6. Deassert PERST#, wait for link training
 *   7. Enable Memory Space + Bus Master on root complex
 *
 * Reference: Linux drivers/pci/controller/pcie-brcmstb.c
 *            bcm2712.dtsi: pcie2@120000 { reg = <0x10 0x00120000 0x0 0x9310>; }
 *            Bare-metal ref: github.com/yuxiaolejs/rpi5-gpio/pcie.c
 */

#include "pcie.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"
#include "fb.h"

/* ---- PCIe RC register offsets (BCM2712 / BCM7712 family) ---- */

#define MISC_MISC_CTRL              0x4008
#define MISC_CPU_2_PCIE_WIN0_LO     0x400C
#define MISC_CPU_2_PCIE_WIN0_HI     0x4010
#define MISC_PCIE_CTRL              0x4064
#define MISC_PCIE_STATUS            0x4068
#define MISC_CPU_2_PCIE_WIN0_BL     0x4070  /* base/limit in MB */
#define MISC_CPU_2_PCIE_WIN0_BH     0x4080  /* base upper bits */
#define MISC_CPU_2_PCIE_WIN0_LH     0x4084  /* limit upper bits */
#define MISC_UBUS_CTRL              0x40A4
#define MISC_UBUS_TIMEOUT           0x40A8
#define MISC_RC_CFG_RETRY_TIMEOUT   0x40AC
#define MISC_AXI_READ_ERROR_DATA    0x4170
#define HARD_DEBUG                  0x4304
#define EXT_CFG_DATA                0x8000
#define EXT_CFG_INDEX               0x9000
#define RGR1_SW_INIT_1              0x9210
#define RC_CFG_PRIV1_ID_VAL3        0x043C

/* Standard PCI Type 1 header offsets (RC config space) */
#define PCI_REG_CMD                 0x04
#define PCI_REG_BUS_NUM             0x18

/* PCIE_STATUS bits */
#define STATUS_DL_ACTIVE            (1U << 5)
#define STATUS_PHYLINKUP            (1U << 4)

/* MISC_CTRL bits */
#define MCTRL_RCB_64B               (1U << 7)
#define MCTRL_RCB_MPS               (1U << 10)
#define MCTRL_SCB_ACCESS_EN         (1U << 12)
#define MCTRL_CFG_READ_UR           (1U << 13)
#define MCTRL_MAX_BURST_MASK        (0x3U << 20)
#define MCTRL_MAX_BURST_512         (0x2U << 20)

/* HARD_DEBUG bits */
#define SERDES_IDDQ                 (1U << 27)

/* PCIE_CTRL bits (PERST# is active-low) */
#define CTRL_PERSTB                 (1U << 2)

/* UBUS_CTRL bits */
#define UBUS_REPLY_ERR_DIS          (1U << 13)
#define UBUS_REPLY_DECERR_DIS       (1U << 19)

/* RGR1_SW_INIT_1 bits */
#define SW_INIT_MASK                (1U << 1)

/* PCI Command register bits */
#define PCI_CMD_MEM                 (1U << 1)
#define PCI_CMD_MASTER              (1U << 2)

/* ---- Register access helpers ---- */

static inline void pw(u32 off, u32 val) { mmio_write(PCIE_RC_BASE + off, val); }
static inline u32  pr(u32 off)          { return mmio_read(PCIE_RC_BASE + off); }

/* ---- Internal helpers ---- */

/* PERST# control — active low, so clear bit to assert, set to deassert */
static void perst_set(bool assert) {
    u32 tmp = pr(MISC_PCIE_CTRL);
    if (assert)
        tmp &= ~CTRL_PERSTB;
    else
        tmp |= CTRL_PERSTB;
    pw(MISC_PCIE_CTRL, tmp);
}

/* Bridge software reset control */
static void bridge_reset(bool assert) {
    u32 tmp = pr(RGR1_SW_INIT_1);
    if (assert)
        tmp |= SW_INIT_MASK;
    else
        tmp &= ~SW_INIT_MASK;
    pw(RGR1_SW_INIT_1, tmp);
}

/*
 * Program outbound ATU window 0.
 * Maps a CPU address range to a PCIe address range so that CPU loads/stores
 * to [cpu_addr, cpu_addr+size) become PCIe memory transactions to
 * [pcie_addr, pcie_addr+size).
 */
static void set_outbound_win(u64 cpu_addr, u64 pcie_addr, u64 size) {
    u64 cpu_mb   = cpu_addr / (1024 * 1024);
    u64 limit_mb = (cpu_addr + size - 1) / (1024 * 1024);

    /* PCIe target address (low 32 and high 32) */
    pw(MISC_CPU_2_PCIE_WIN0_LO, (u32)pcie_addr);
    pw(MISC_CPU_2_PCIE_WIN0_HI, (u32)(pcie_addr >> 32));

    /* BASE_LIMIT: [31:20]=limit_mb[11:0], [15:4]=base_mb[11:0] */
    pw(MISC_CPU_2_PCIE_WIN0_BL,
       ((u32)(limit_mb & 0xFFF) << 20) | ((u32)(cpu_mb & 0xFFF) << 4));

    /* Upper bits of base/limit MB (above the 12 bits in BASE_LIMIT) */
    pw(MISC_CPU_2_PCIE_WIN0_BH, (u32)(cpu_mb >> 12));
    pw(MISC_CPU_2_PCIE_WIN0_LH, (u32)(limit_mb >> 12));
}

/* ---- Public API ---- */

bool pcie_link_up(void) {
    u32 st = pr(MISC_PCIE_STATUS);
    return (st & STATUS_DL_ACTIVE) && (st & STATUS_PHYLINKUP);
}

u32 pcie_cfg_read(u32 bus, u32 dev, u32 func, u32 reg) {
    /* Bus 0 = root complex itself — direct register access */
    if (bus == 0 && dev == 0 && func == 0)
        return pr(reg & 0xFFC);

    if (!pcie_link_up())
        return 0xFFFFFFFF;

    /* ECAM-style index: bus[27:20] | dev[19:15] | func[14:12] */
    pw(EXT_CFG_INDEX, (bus << 20) | (dev << 15) | (func << 12));
    dmb();
    return pr(EXT_CFG_DATA + (reg & 0xFFC));
}

void pcie_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val) {
    if (bus == 0 && dev == 0 && func == 0) {
        pw(reg & 0xFFC, val);
        dmb();
        return;
    }

    if (!pcie_link_up())
        return;

    pw(EXT_CFG_INDEX, (bus << 20) | (dev << 15) | (func << 12));
    dmb();
    pw(EXT_CFG_DATA + (reg & 0xFFC), val);
    dmb();
}

bool pcie_init(void) {
    u32 tmp;

    uart_puts("[pcie] Init BCM2712 PCIe2 RC...\n");

    /* 1. Assert bridge + endpoint reset */
    bridge_reset(true);
    perst_set(true);
    timer_delay_us(200);

    /* 2. Deassert bridge reset (controller out of reset) */
    bridge_reset(false);

    /* 3. Power up SerDes (clear IDDQ power-down bit) */
    tmp = pr(HARD_DEBUG);
    tmp &= ~SERDES_IDDQ;
    pw(HARD_DEBUG, tmp);
    timer_delay_us(200);

    /* 4. Configure MISC_CTRL */
    tmp = pr(MISC_MISC_CTRL);
    tmp |= MCTRL_SCB_ACCESS_EN;    /* SCB access enable */
    tmp |= MCTRL_CFG_READ_UR;      /* return UR on bad config reads */
    tmp |= MCTRL_RCB_MPS;          /* MPS mode */
    tmp |= MCTRL_RCB_64B;          /* 64-byte read completion boundary */
    tmp = (tmp & ~MCTRL_MAX_BURST_MASK) | MCTRL_MAX_BURST_512;
    pw(MISC_MISC_CTRL, tmp);

    /* 5. Set RC class code to PCI-PCI Bridge (0x060400) */
    tmp = pr(RC_CFG_PRIV1_ID_VAL3);
    tmp = (tmp & ~0xFFFFFFU) | 0x060400U;
    pw(RC_CFG_PRIV1_ID_VAL3, tmp);

    /* 6. Bus numbers: primary=0, secondary=1, subordinate=1 */
    pw(PCI_REG_BUS_NUM, 0x00010100);
    dmb();

    /* 7. Outbound ATU window 0: CPU 0x1F00000000 → PCIe 0x80000000 */
    set_outbound_win(PCIE_CPU_WIN_BASE, PCIE_TARGET_ADDR, PCIE_CPU_WIN_SIZE);

    /* 8. UBUS/AXI bridge: suppress error responses, set timeouts */
    tmp = pr(MISC_UBUS_CTRL);
    tmp |= UBUS_REPLY_ERR_DIS;
    tmp |= UBUS_REPLY_DECERR_DIS;
    pw(MISC_UBUS_CTRL, tmp);
    pw(MISC_AXI_READ_ERROR_DATA, 0xFFFFFFFF);
    pw(MISC_UBUS_TIMEOUT, 0x0B2D0000);           /* ~250ms */
    pw(MISC_RC_CFG_RETRY_TIMEOUT, 0x0ABA0000);   /* ~240ms */

    /* 9. Deassert PERST# — endpoint begins link training */
    perst_set(false);
    timer_delay_ms(100);  /* PCIe spec: min 100ms after PERST# deassert */

    /* 10. Poll for link up (up to 100ms more) */
    for (u32 i = 0; i < 20; i++) {
        if (pcie_link_up())
            break;
        timer_delay_ms(5);
    }

    if (!pcie_link_up()) {
        uart_puts("[pcie] Link FAILED (status=");
        uart_hex(pr(MISC_PCIE_STATUS));
        uart_puts(")\n");
        return false;
    }

    /* 11. Enable Memory Space + Bus Master on RC */
    tmp = pr(PCI_REG_CMD);
    tmp |= PCI_CMD_MEM | PCI_CMD_MASTER;
    pw(PCI_REG_CMD, tmp);
    dmb();

    uart_puts("[pcie] Link UP\n");
    return true;
}
