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

/* Inbound (RC BAR) registers for RP1 DMA to reach system RAM */
#define MISC_RC_BAR2_CONFIG_LO      0x4034
#define MISC_RC_BAR2_CONFIG_HI      0x4038
#define MISC_RC_BAR3_CONFIG_LO      0x403C
#define MISC_UBUS_BAR2_CONFIG_REMAP    0x40B4
#define MISC_UBUS_BAR2_CONFIG_REMAP_HI 0x40B0

/* Vendor-specific config: BAR2 endian mode */
#define RC_CFG_VENDOR_SPECIFIC_REG1 0x0188
#define  VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK 0xC
#define  VENDOR_SPECIFIC_REG1_LITTLE_ENDIAN         0x0

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

/* RESCAL (PCIe resistor calibration) — BCM2712 SoC register */
#define RESCAL_BASE             0x1000119500UL
#define RESCAL_START            0x00
#define RESCAL_STATUS           0x08
#define RESCAL_START_BIT        (1U << 0)
#define RESCAL_STATUS_BIT       (1U << 0)

/* BCM2712 reset controller for bridge reset */
#define BCM_RESET_BASE          0x1001504318UL
#define SW_INIT_SET             0x00
#define SW_INIT_CLEAR           0x04
#define SW_INIT_STATUS          0x08
#define SW_INIT_BANK_SIZE       0x18

static void rescal_deassert(void) {
    u32 reg = mmio_read(RESCAL_BASE + RESCAL_START);
    mmio_write(RESCAL_BASE + RESCAL_START, reg | RESCAL_START_BIT);
    delay_cycles(100000);
    /* Wait for calibration complete */
    for (u32 i = 0; i < 10000; i++) {
        if (mmio_read(RESCAL_BASE + RESCAL_STATUS) & RESCAL_STATUS_BIT)
            break;
        delay_cycles(10000);
    }
    reg = mmio_read(RESCAL_BASE + RESCAL_START);
    mmio_write(RESCAL_BASE + RESCAL_START, reg & ~RESCAL_START_BIT);
}

/* BCM7712-style bridge reset via reset controller (not RGR1) */
static void bridge_reset_brcm(bool assert) {
    /* Reset ID 44 for pcie2 (RP1 bridge) */
    u32 bit = 1U << (44 & 0x1F);
    u32 bank_off = (44 >> 5) * SW_INIT_BANK_SIZE;
    if (assert)
        mmio_write(BCM_RESET_BASE + bank_off + SW_INIT_SET, bit);
    else
        mmio_write(BCM_RESET_BASE + bank_off + SW_INIT_CLEAR, bit);
    delay_cycles(100000);
}
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
    u32 i;

    /* ══════════════════════════════════════════════════════════════════
     * Full RC init — Circle/Linux sequence for BCM2712 Pi 5.
     * Includes rescal (missing from all previous attempts).
     * ══════════════════════════════════════════════════════════════════ */

    /* 0. RESCAL — PCIe SerDes resistor calibration (CRITICAL) */
    rescal_deassert();

    /* 1. Assert bridge reset (BCM7712 reset controller, ID 44) */
    bridge_reset_brcm(true);
    delay_cycles(100000);

    /* 2. Deassert bridge reset */
    bridge_reset_brcm(false);

    /* 3. Power up SerDes */
    tmp = pr(HARD_DEBUG);
    tmp &= ~SERDES_IDDQ;
    pw(HARD_DEBUG, tmp);
    delay_cycles(200000);

    /* 4. MISC_CTRL */
    tmp = pr(MISC_MISC_CTRL);
    tmp |= MCTRL_SCB_ACCESS_EN;
    tmp |= MCTRL_CFG_READ_UR;
    tmp &= ~MCTRL_MAX_BURST_MASK;
    tmp |= (1U << 20);  /* BURST_SIZE_256 */
    tmp |= MCTRL_RCB_MPS;
    tmp |= MCTRL_RCB_64B;
    pw(MISC_MISC_CTRL, tmp);
    dmb();

    /* 5. BAR2 inbound: 64GB at PCIe 0x10 → CPU 0x00 */
    pw(MISC_RC_BAR2_CONFIG_LO, 0x15);  /* size=21, offset_lo=0 */
    pw(MISC_RC_BAR2_CONFIG_HI, 0x10);
    dmb();

    /* 6. UBUS BAR2 remap */
    tmp = pr(MISC_UBUS_BAR2_CONFIG_REMAP);
    tmp |= 1;  /* ACCESS_ENABLE */
    pw(MISC_UBUS_BAR2_CONFIG_REMAP, tmp);
    pw(MISC_UBUS_BAR2_CONFIG_REMAP_HI, 0x00);
    dmb();

    /* 7. SCB0_SIZE = 21 */
    tmp = pr(MISC_MISC_CTRL);
    tmp &= ~(0x1FU << 27);
    tmp |= (21U << 27);
    pw(MISC_MISC_CTRL, tmp);
    dmb();

    /* 8. UBUS error suppression + timeouts */
    tmp = pr(MISC_UBUS_CTRL);
    tmp |= UBUS_REPLY_ERR_DIS | UBUS_REPLY_DECERR_DIS;
    pw(MISC_UBUS_CTRL, tmp);
    pw(MISC_AXI_READ_ERROR_DATA, 0xFFFFFFFF);
    pw(MISC_UBUS_TIMEOUT, 0x0B2D0000);
    pw(MISC_RC_CFG_RETRY_TIMEOUT, 0x0ABA0000);

    /* 9. Disable BAR1, BAR3 */
    pw(0x402C, 0);
    pw(MISC_RC_BAR3_CONFIG_LO, 0);

    /* 10. Class code */
    tmp = pr(RC_CFG_PRIV1_ID_VAL3);
    tmp &= ~0xFFFFFF;
    tmp |= 0x060400;
    pw(RC_CFG_PRIV1_ID_VAL3, tmp);

    /* 11. Outbound ATU */
    set_outbound_win(PCIE_CPU_WIN_BASE, PCIE_TARGET_ADDR, PCIE_CPU_WIN_SIZE);

    /* 12. Endian: little */
    tmp = pr(RC_CFG_VENDOR_SPECIFIC_REG1);
    tmp &= ~VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK;
    pw(RC_CFG_VENDOR_SPECIFIC_REG1, tmp);
    dmb();

    /* 13. PERST# deassert → link training */
    perst_set(false);
    delay_cycles(100000000);  /* ~200ms */

    /* 14. Wait for link */
    for (i = 0; i < 200; i++) {
        if (pcie_link_up()) break;
        delay_cycles(5000000);
    }
    if (!pcie_link_up()) {
        return false;
    }

    /* 15. Bus numbers */
    pw(PCI_REG_BUS_NUM, 0x00010100);
    dmb();

    /* 16. Bridge command */
    tmp = pr(PCI_REG_CMD);
    tmp |= PCI_CMD_MEM | PCI_CMD_MASTER;
    pw(PCI_REG_CMD, tmp);
    dmb();

    /* ── Verbose PCIe register dump for DMA debugging ── */
    uart_puts("[pci] RC regs:\n");
    uart_puts("[pci] MCTRL="); uart_hex(pr(MISC_MISC_CTRL)); uart_puts("\n");
    uart_puts("[pci] B2L="); uart_hex(pr(MISC_RC_BAR2_CONFIG_LO));
    uart_puts(" B2H="); uart_hex(pr(MISC_RC_BAR2_CONFIG_HI)); uart_puts("\n");
    uart_puts("[pci] UBRM="); uart_hex(pr(MISC_UBUS_BAR2_CONFIG_REMAP));
    uart_puts(" RMH="); uart_hex(pr(MISC_UBUS_BAR2_CONFIG_REMAP_HI)); uart_puts("\n");
    uart_puts("[pci] VREG="); uart_hex(pr(RC_CFG_VENDOR_SPECIFIC_REG1)); uart_puts("\n");
    uart_puts("[pci] UBUS="); uart_hex(pr(MISC_UBUS_CTRL)); uart_puts("\n");
    uart_puts("[pci] STS="); uart_hex(pr(MISC_PCIE_STATUS)); uart_puts("\n");
    uart_puts("[pci] CMD="); uart_hex(pr(PCI_REG_CMD)); uart_puts("\n");
    uart_puts("[pci] HDBG="); uart_hex(pr(HARD_DEBUG)); uart_puts("\n");
    uart_puts("[pci] OB0L="); uart_hex(pr(MISC_CPU_2_PCIE_WIN0_LO));
    uart_puts(" H="); uart_hex(pr(MISC_CPU_2_PCIE_WIN0_HI)); uart_puts("\n");
    uart_puts("[pci] OB0BL="); uart_hex(pr(MISC_CPU_2_PCIE_WIN0_BL)); uart_puts("\n");
    uart_puts("[pci] OB0BH="); uart_hex(pr(MISC_CPU_2_PCIE_WIN0_BH));
    uart_puts(" LH="); uart_hex(pr(MISC_CPU_2_PCIE_WIN0_LH)); uart_puts("\n");

    /* RP1 endpoint config */
    uart_puts("[pci] RP1 B1="); uart_hex(pcie_cfg_read(1,0,0,0x14)); uart_puts("\n");
    uart_puts("[pci] RP1 CMD="); uart_hex(pcie_cfg_read(1,0,0,0x04)); uart_puts("\n");
    uart_puts("[pci] RP1 ID="); uart_hex(pcie_cfg_read(1,0,0,0x00)); uart_puts("\n");

    /* Also dump to HDMI via fb so user can read it on screen */
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_printf("PCIe MISC_CTRL=%X\n", pr(MISC_MISC_CTRL));
    fb_printf("BAR2=%X/%X REMAP=%X\n", pr(MISC_RC_BAR2_CONFIG_LO),
              pr(MISC_RC_BAR2_CONFIG_HI), pr(MISC_UBUS_BAR2_CONFIG_REMAP));
    fb_printf("VENDOR=%X UBUS=%X\n", pr(RC_CFG_VENDOR_SPECIFIC_REG1), pr(MISC_UBUS_CTRL));
    fb_printf("RP1 BAR1=%X CMD=%X\n", pcie_cfg_read(1,0,0,0x14), pcie_cfg_read(1,0,0,0x04));

    uart_puts("[pci] end dump\n");

    /* Init AER early so we catch any DMA errors */
    pcie_aer_init();

    return true;
}

/* ---- PCIe Advanced Error Reporting (AER) ---- */

/*
 * AER is an Extended Capability (ID=0x0001) in PCIe config space.
 * Walk the extended capability list starting at offset 0x100 to find it.
 * Works on both the RC (bus 0) and the RP1 endpoint (bus 1).
 */

static u32 aer_offset_rc;   /* AER base in RC config space */

/* AER register offsets from AER base */
#define AER_UNCORR_ERR      0x04
#define AER_UNCORR_MASK     0x08
#define AER_UNCORR_SEV      0x0C
#define AER_CORR_ERR        0x10
#define AER_CORR_MASK       0x14
#define AER_HDR_LOG0        0x1C
#define AER_HDR_LOG1        0x20
#define AER_HDR_LOG2        0x24
#define AER_HDR_LOG3        0x28

static u32 find_aer_cap(u32 bus, u32 dev, u32 fn) {
    u32 off = 0x100;
    for (u32 i = 0; i < 48 && off >= 0x100; i++) {
        u32 hdr = pcie_cfg_read(bus, dev, fn, off);
        u16 cap_id = hdr & 0xFFFF;
        if (cap_id == 0x0001)
            return off;  /* found AER */
        off = (hdr >> 20) & 0xFFC;
        if (off == 0) break;
    }
    return 0;
}

void pcie_aer_init(void) {
    /* Find AER on the RC (bus 0) */
    aer_offset_rc = find_aer_cap(0, 0, 0);
    if (aer_offset_rc) {
        uart_puts("[pci] AER @RC=");
        uart_hex(aer_offset_rc);
        uart_puts("\n");
        /* Clear all errors */
        pcie_cfg_write(0, 0, 0, aer_offset_rc + AER_UNCORR_ERR, 0xFFFFFFFF);
        pcie_cfg_write(0, 0, 0, aer_offset_rc + AER_CORR_ERR, 0xFFFFFFFF);
        /* Unmask all errors */
        pcie_cfg_write(0, 0, 0, aer_offset_rc + AER_UNCORR_MASK, 0);
        pcie_cfg_write(0, 0, 0, aer_offset_rc + AER_CORR_MASK, 0);
    } else {
        uart_puts("[pci] no AER\n");
    }
}

void pcie_aer_dump(const char *tag) {
    if (!aer_offset_rc) return;
    u32 uncorr = pcie_cfg_read(0, 0, 0, aer_offset_rc + AER_UNCORR_ERR);
    u32 corr   = pcie_cfg_read(0, 0, 0, aer_offset_rc + AER_CORR_ERR);

    uart_puts("[aer] ");
    uart_puts(tag);
    uart_puts(" uncorr=");
    uart_hex(uncorr);
    uart_puts(" corr=");
    uart_hex(corr);

    if (uncorr || corr) {
        uart_puts("\n[aer] HDR: ");
        uart_hex(pcie_cfg_read(0, 0, 0, aer_offset_rc + AER_HDR_LOG0));
        uart_puts(" ");
        uart_hex(pcie_cfg_read(0, 0, 0, aer_offset_rc + AER_HDR_LOG1));
        uart_puts(" ");
        uart_hex(pcie_cfg_read(0, 0, 0, aer_offset_rc + AER_HDR_LOG2));
        uart_puts(" ");
        uart_hex(pcie_cfg_read(0, 0, 0, aer_offset_rc + AER_HDR_LOG3));

        /* Clear for next check */
        pcie_cfg_write(0, 0, 0, aer_offset_rc + AER_UNCORR_ERR, 0xFFFFFFFF);
        pcie_cfg_write(0, 0, 0, aer_offset_rc + AER_CORR_ERR, 0xFFFFFFFF);
    }
    uart_puts("\n");
}
