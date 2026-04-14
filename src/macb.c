/*
 * macb.c - Cadence GEM/MACB Ethernet driver for Pi 5 (RP1)
 *
 * Minimal polling driver: single TX/RX ring, no interrupts.
 * Based on Linux drivers/net/ethernet/cadence/macb.h register defs
 * and Circle's ARM_MACB_BASE = 0x1F00100000 for Pi 5.
 *
 * Reference: Cadence GEM Technical Reference Manual
 *            Linux macb.h / macb_main.c
 */

#include "macb.h"
#include "mmio.h"
#include "uart.h"
#include "mailbox.h"
#include "fb.h"
#include "timer.h"
#include "rp1_gpio.h"
#include "rp1_clk.h"
#include "mmu.h"
#include "pcie.h"
#include "rp1_clk.h"

/* PHY reset is on RP1 GPIO 32, funcsel 5, active LOW */
#define PHY_RESET_GPIO  32
#define PHY_RESET_FSEL  5

/* ── Register offsets ── */
#define NCR         0x0000  /* Network Control */
#define NCFGR       0x0004  /* Network Configuration */
#define NSR         0x0008  /* Network Status */
#define DMACFG      0x0010  /* DMA Configuration (GEM) */
#define TSR         0x0014  /* Transmit Status */
#define RBQP        0x0018  /* RX Buffer Queue Pointer */
#define TBQP        0x001C  /* TX Buffer Queue Pointer */
#define RSR         0x0020  /* Receive Status */
#define ISR         0x0024  /* Interrupt Status */
#define IER         0x0028  /* Interrupt Enable */
#define IDR         0x002C  /* Interrupt Disable */
#define IMR         0x0030  /* Interrupt Mask */
#define MAN         0x0034  /* PHY Maintenance (MDIO) */
#define SA1B        0x0088  /* Specific Address 1 Bottom (GEM) */
#define SA1T        0x008C  /* Specific Address 1 Top (GEM) */
#define USRIO       0x00C0  /* User I/O (GEM — not 0x0C which is MACB) */
#define GEM_AMP     0x0054  /* AXI Max Pipeline (RP1-specific) */
#define GEM_INTMOD  0x005C  /* Interrupt Moderation */
#define DCFG1       0x0280  /* Design Config 1 */
#define DCFG2       0x0284  /* Design Config 2 */
#define RBQPH       0x04D4  /* RX Queue Base Addr High */
#define TBQPH       0x04C8  /* TX Queue Base Addr High */

/* RP1 ETH_CFG block — separate from GEM registers */
#define ETH_CFG_BASE    0x1F00104000UL
#define ETH_CFG_CTRL    0x00  /* bit 3 = BUSERR_EN, bit 4 = MEM_PD */
#define ETH_CFG_STAT    0x04  /* bit 4 = ARLEN_ILLEGAL, bit 5 = AWLEN_ILLEGAL */
#define ETH_CFG_CLKGEN  0x14  /* bit 7 = ENABLE */

static inline u32 ecr(u32 off) { return mmio_read(ETH_CFG_BASE + off); }
static inline void ecw(u32 off, u32 val) { mmio_write(ETH_CFG_BASE + off, val); }

/* NCR bits */
#define NCR_LLB     (1 << 1)   /* Loopback local */
#define NCR_RE      (1 << 2)   /* Receive enable */
#define NCR_TE      (1 << 3)   /* Transmit enable */
#define NCR_MPE     (1 << 4)   /* Management port enable */
#define NCR_CLRSTAT (1 << 5)   /* Clear statistics */
#define NCR_TSTART  (1 << 9)   /* Start transmission */

/* NCFGR bits */
#define NCFGR_SPD       (1 << 0)   /* Speed (100Mbps) */
#define NCFGR_FD        (1 << 1)   /* Full duplex */
#define NCFGR_CAF       (1 << 4)   /* Copy all frames */
#define NCFGR_BIG       (1 << 8)   /* Receive 1536 byte frames */
#define NCFGR_GBE       (1 << 10)  /* Gigabit mode (GEM) */
#define NCFGR_CLK_MASK  (7 << 18)  /* MDC clock divider (GEM) */
#define NCFGR_CLK_DIV64 (3 << 18)
#define NCFGR_DBW_MASK  (3 << 21)  /* Data bus width */
#define NCFGR_RXCOEN    (1 << 24)  /* RX checksum offload */

/* NSR bits */
#define NSR_LINK    (1 << 0)   /* PHY link */
#define NSR_MDIO    (1 << 1)   /* MDIO input status */
#define NSR_IDLE    (1 << 2)   /* PHY management idle */

/* MAN (MDIO) bits */
#define MAN_SOF     (1 << 30)  /* Start of frame (must be 01 for Clause 22) */
#define MAN_READ    (2 << 28)  /* Read operation */
#define MAN_WRITE   (1 << 28)  /* Write operation */
#define MAN_CODE    (2 << 16)  /* Must be 10 */

/* ISR/IER/IDR bits */
#define INT_MFD     (1 << 0)   /* Management frame done */
#define INT_RCOMP   (1 << 1)   /* Receive complete */
#define INT_TCOMP   (1 << 7)   /* Transmit complete */

/* USRIO bits */
#define USRIO_RGMII (1 << 0)   /* RGMII mode */
#define USRIO_CLKEN (1 << 1)   /* Clock enable */

/* DMACFG bits */
#define DMACFG_RXBS_SHIFT   16
#define DMACFG_FBLDO_SHIFT  0
#define DMACFG_ADDR64       (1 << 30)

/* ── Buffer descriptor format ── */
/* RX descriptor: [addr] [status]
 *   addr bit 0 = ownership (0=MAC owns, 1=SW owns)
 *   addr bit 1 = wrap
 *   addr bits [31:2] = buffer address >> 2
 *   status bits [12:0] = frame length
 *   status bit 13 = SOF
 *   status bit 14 = EOF
 */
#define RX_ADDR_OWN     (1 << 0)
#define RX_ADDR_WRAP    (1 << 1)
#define RX_STAT_LEN_MASK  0x1FFF
#define RX_STAT_SOF     (1 << 14)
#define RX_STAT_EOF     (1 << 15)

/* TX descriptor: [addr] [status]
 *   status bit 15 = wrap
 *   status bit 31 = used (1=SW owns, 0=MAC owns)
 *   status bits [13:0] = frame length
 *   status bit 15 = last buffer
 */
#define TX_STAT_USED    (1U << 31)
#define TX_STAT_WRAP    (1 << 30)
#define TX_STAT_LAST    (1 << 15)
#define TX_STAT_LEN_MASK  0x3FFF

/* ── Ring sizes ── */
#define NUM_RX  32
#define NUM_TX  16
#define BUF_SIZE 2048

/* ── Descriptors and buffers (64-byte aligned for DMA coherency) ── */
/* Firmware BAR2 at PCIe 0x00 (64GB window covers 0x00-0x40).
 * But RP1 DT dma-ranges says child DMA is at 0x10. Use 0x10. */
#define MACB_DMA_HI       0x10
#define USE_8BYTE_DESC     0

#if USE_8BYTE_DESC
struct macb_desc {
    u32 addr;       /* buffer address + OWN/WRAP bits (RX) */
    u32 ctrl;       /* control/status */
} PACKED;
#else
struct macb_desc {
    u32 addr;       /* buffer address + OWN/WRAP bits (RX) */
    u32 ctrl;       /* control/status */
    u32 addr_hi;    /* buffer address high */
    u32 rsvd;       /* reserved */
} PACKED;
#endif

static struct macb_desc rx_ring[NUM_RX] ALIGNED(64);
static struct macb_desc tx_ring[NUM_TX] ALIGNED(64);
static u8 rx_bufs[NUM_RX][BUF_SIZE] ALIGNED(64);
static u8 tx_bufs[NUM_TX][BUF_SIZE] ALIGNED(64);
static u32 rx_idx;
static u32 tx_idx;
static u8 mac_addr[6];
static u8 phy_addr;

/* ── Register helpers ── */
static inline u32 mr(u32 off) { return mmio_read(MACB_BASE + off); }
static inline void mw(u32 off, u32 val) { mmio_write(MACB_BASE + off, val); }

static bool mac_is_zero(const u8 *mac)
{
    return mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
           mac[3] == 0 && mac[4] == 0 && mac[5] == 0;
}

static bool mac_is_broadcast(const u8 *mac)
{
    return mac[0] == 0xFF && mac[1] == 0xFF && mac[2] == 0xFF &&
           mac[3] == 0xFF && mac[4] == 0xFF && mac[5] == 0xFF;
}

static bool mac_is_placeholder(const u8 *mac)
{
    static const u8 placeholder[6] = { 0xDC, 0xA6, 0x32, 0x01, 0x02, 0x03 };
    for (u32 i = 0; i < 6; i++) {
        if (mac[i] != placeholder[i])
            return false;
    }
    return true;
}

static bool mac_is_valid(const u8 *mac)
{
    if (mac_is_zero(mac) || mac_is_broadcast(mac) || mac_is_placeholder(mac))
        return false;
    return (mac[0] & 0x01) == 0;
}

static void mac_copy(u8 *dst, const u8 *src)
{
    for (u32 i = 0; i < 6; i++)
        dst[i] = src[i];
}

static void mac_log(const char *prefix, const u8 *mac)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_puts(prefix);
    for (u32 i = 0; i < 6; i++) {
        uart_putc(hex[mac[i] >> 4]);
        uart_putc(hex[mac[i] & 0xF]);
        if (i < 5)
            uart_putc(':');
    }
    uart_puts("\n");
}

static void mac_log_invalid_reason(const char *prefix, const u8 *mac)
{
    uart_puts(prefix);
    if (mac_is_zero(mac)) {
        uart_puts("zero\n");
        return;
    }
    if (mac_is_broadcast(mac)) {
        uart_puts("broadcast\n");
        return;
    }
    if (mac_is_placeholder(mac)) {
        uart_puts("placeholder\n");
        return;
    }
    if (mac[0] & 0x01) {
        uart_puts("multicast\n");
        return;
    }
    uart_puts("unknown\n");
}

static void macb_dump_regs(const char *tag)
{
    uart_puts(tag);
    uart_puts(" NCR="); uart_hex(mr(NCR));
    uart_puts(" NCFGR="); uart_hex(mr(NCFGR));
    uart_puts(" NSR="); uart_hex(mr(NSR));
    uart_puts(" USRIO="); uart_hex(mr(USRIO));
    uart_puts(" DMACFG="); uart_hex(mr(DMACFG));
    uart_puts(" RBQP="); uart_hex(mr(RBQP));
    uart_puts(" TBQP="); uart_hex(mr(TBQP));
    uart_puts("\n");
}

static bool mac_load_from_sa1(u8 *out)
{
    u32 sa1b = mr(SA1B);
    u32 sa1t = mr(SA1T);
    u8 candidate[6];

    uart_puts("[macb] SA1B="); uart_hex(sa1b);
    uart_puts(" SA1T="); uart_hex(sa1t);
    uart_puts("\n");

    if ((sa1b | sa1t) == 0)
        return false;

    candidate[0] = (sa1b >> 0) & 0xFF;
    candidate[1] = (sa1b >> 8) & 0xFF;
    candidate[2] = (sa1b >> 16) & 0xFF;
    candidate[3] = (sa1b >> 24) & 0xFF;
    candidate[4] = (sa1t >> 0) & 0xFF;
    candidate[5] = (sa1t >> 8) & 0xFF;

    if (!mac_is_valid(candidate)) {
        mac_log("[macb] Rejecting invalid SA1 MAC ", candidate);
        mac_log_invalid_reason("[macb] SA1 reject reason: ", candidate);
        return false;
    }

    mac_copy(out, candidate);
    mac_log("[macb] MAC from SA1 regs ", out);
    return true;
}

static bool mac_load_from_mailbox(u8 *out)
{
    static volatile u32 __attribute__((aligned(16))) mb[8];
    u8 candidate[6];

    /* Try tag 0x00010003 — GET_MAC_ADDRESS */
    mb[0] = 8 * 4; mb[1] = 0;
    mb[2] = 0x00010003; mb[3] = 6; mb[4] = 6;
    mb[5] = 0; mb[6] = 0; mb[7] = 0;
    uart_puts("[macb] Mailbox GET_MAC_ADDRESS call...\n");
    bool ok = mbox_call(8, mb);
    uart_puts("[macb] Mailbox result=");
    uart_hex(ok ? 1 : 0);
    uart_puts(" mb[1]=");
    uart_hex(mb[1]);
    uart_puts(" mb[4]=");
    uart_hex(mb[4]);
    uart_puts("\n");

    if (ok) {
        const u8 *raw = (const u8 *)&mb[5];
        for (u32 i = 0; i < 6; i++)
            candidate[i] = raw[i];

        mac_log("[macb] Mailbox MAC raw ", candidate);

        if (mac_is_valid(candidate)) {
            mac_copy(out, candidate);
            mac_log("[macb] MAC from mailbox ", out);
            return true;
        }
        mac_log("[macb] Rejecting invalid mailbox MAC ", candidate);
        mac_log_invalid_reason("[macb] Mailbox reject reason: ", candidate);
    }

    /* Try tag 0x00010004 — GET_BOARD_SERIAL and derive MAC */
    uart_puts("[macb] Trying GET_BOARD_SERIAL for MAC...\n");
    mb[0] = 8 * 4; mb[1] = 0;
    mb[2] = 0x00010004; mb[3] = 8; mb[4] = 8;
    mb[5] = 0; mb[6] = 0; mb[7] = 0;
    ok = mbox_call(8, mb);
    uart_puts("[macb] Serial result=");
    uart_hex(ok ? 1 : 0);
    uart_puts(" mb[5]=");
    uart_hex(mb[5]);
    uart_puts(" mb[6]=");
    uart_hex(mb[6]);
    uart_puts("\n");

    if (ok && (mb[5] | mb[6])) {
        /* Derive a locally-administered MAC from the board serial */
        u64 serial = ((u64)mb[6] << 32) | mb[5];
        candidate[0] = 0xB8;  /* locally administered, unicast */
        candidate[1] = 0x27;
        candidate[2] = 0xEB;  /* Raspberry Pi OUI-ish prefix */
        candidate[3] = (u8)(serial >> 16);
        candidate[4] = (u8)(serial >> 8);
        candidate[5] = (u8)(serial);
        mac_log("[macb] MAC derived from serial ", candidate);
        mac_copy(out, candidate);
        return true;
    }

    uart_puts("[macb] All mailbox MAC sources exhausted\n");
    return false;
}

/* ── MDIO (Circle-style: toggle MPE per operation) ── */
static u16 mdio_read(u8 phy, u8 reg) {
    u32 ncr = mr(NCR);
    mw(NCR, ncr | NCR_MPE);
    mw(MAN, MAN_SOF | MAN_READ | MAN_CODE
       | ((u32)phy << 23) | ((u32)reg << 18));
    while (!(mr(NSR) & NSR_IDLE))
        delay_cycles(10);
    u16 val = (u16)(mr(MAN) & 0xFFFF);
    mw(NCR, mr(NCR) & ~NCR_MPE);
    return val;
}

static bool phy_select(void)
{
    for (u32 addr = 0; addr < 32; addr++) {
        u16 bmsr = mdio_read((u8)addr, 0x01);
        uart_puts("[macb] MDIO scan addr=");
        uart_hex(addr);
        uart_puts(" BMSR=");
        uart_hex(bmsr);
        uart_puts("\n");
        if (bmsr != 0x0000 && bmsr != 0xFFFF) {
            phy_addr = (u8)addr;
            uart_puts("[macb] PHY addr ");
            uart_hex(addr);
            uart_puts(" detected\n");
            return true;
        }
    }
    uart_puts("[macb] No PHY responded on MDIO\n");
    return false;
}

/* ── Init ── */
bool macb_init(void) {
    uart_puts("[macb] Init Cadence GEM\n");

    /* ── Step 0: Enable ETH clock on RP1 ── */
    rp1_clk_enable(RP1_CLK_ETH);

    /* ── Step 1: PHY hardware reset via GPIO 32 ── */
    rp1_gpio_set_function(PHY_RESET_GPIO, PHY_RESET_FSEL);
    rp1_gpio_set_dir_output(PHY_RESET_GPIO);
    rp1_gpio_write(PHY_RESET_GPIO, false);
    timer_delay_ms(10);
    rp1_gpio_write(PHY_RESET_GPIO, true);
    timer_delay_ms(10);
    timer_delay_ms(50);

    /* Read design config to verify it's alive */
    u32 dcfg1 = mr(DCFG1);
    u32 dcfg2 = mr(DCFG2);
    uart_puts("[macb] DCFG1=");
    uart_hex(dcfg1);
    uart_puts(" DCFG2=");
    uart_hex(dcfg2);
    uart_puts("\n");

    if (dcfg1 == 0 && dcfg2 == 0) {
        uart_puts("[macb] GEM not responding (all zeros)\n");
        return false;
    }

    /* Disable RX/TX, clear stats (Circle: macb_halt) */
    u32 ncr = mr(NCR);
    mw(NCR, ncr | (1 << 10));  /* THALT */
    while (mr(TSR) & (1 << 3)) /* wait for TGO clear */
        delay_cycles(100);
    mw(NCR, NCR_CLRSTAT);

    /* ── DMA config: set ADDR64 FIRST so MAC uses 16-byte descriptors ──
     * CRITICAL: firmware leaves ADDR64=0 (8-byte descriptors).
     * We MUST set ADDR64 before writing ring pointers, otherwise the MAC
     * will stride at 8 bytes through our 16-byte descriptor ring. */
    {
        u32 dmacfg = mr(DMACFG);
        uart_puts("[macb] DMACFG firmware="); uart_hex(dmacfg); uart_puts("\n");
        dmacfg &= ~(0xFF << DMACFG_RXBS_SHIFT);
        dmacfg &= ~(0x1F << DMACFG_FBLDO_SHIFT);
        dmacfg |= ((BUF_SIZE / 64) << DMACFG_RXBS_SHIFT);
        /* FBLDO=16 — RPi rp1-gem config uses 16 */
        dmacfg |= (16 << DMACFG_FBLDO_SHIFT);
        dmacfg |= (1 << 10);   /* TXPBMS */
        dmacfg |= (3 << 8);    /* RXBMS = 3 (max) */
        dmacfg &= ~(1 << 7);   /* no endian swap pkt */
        dmacfg &= ~(1 << 6);   /* no endian swap desc */
#if USE_8BYTE_DESC
        dmacfg &= ~DMACFG_ADDR64;  /* 8-byte descriptors */
#else
        dmacfg |= DMACFG_ADDR64;   /* 16-byte descriptors */
#endif
        mw(DMACFG, dmacfg);
        uart_puts("[macb] DMACFG final="); uart_hex(dmacfg); uart_puts("\n");
    }

    /* ── RP1 ETH_CFG: check status and enable bus errors ── */
    {
        u32 cfg_stat = ecr(ETH_CFG_STAT);
        u32 cfg_ctrl = ecr(ETH_CFG_CTRL);
        uart_puts("[macb] ETH_CFG CTRL="); uart_hex(cfg_ctrl);
        uart_puts(" STAT="); uart_hex(cfg_stat);
        uart_puts("\n");
        if (cfg_stat & (1 << 5)) uart_puts("[macb] WARNING: AWLEN_ILLEGAL set!\n");
        if (cfg_stat & (1 << 4)) uart_puts("[macb] WARNING: ARLEN_ILLEGAL set!\n");
        /* Enable bus error passthrough for debugging */
        ecw(ETH_CFG_CTRL, cfg_ctrl | (1 << 3));
    }

    /* ── GEM AXI pipeline config (RP1-specific, from Linux patch) ── */
    {
        u32 amp = mr(GEM_AMP);
        uart_puts("[macb] GEM_AMP before="); uart_hex(amp); uart_puts("\n");
        /* Set reasonable pipeline depths — RP1 Linux uses DT properties */
        amp &= ~0xFFFF;  /* clear AR2R and AW2W fields */
        amp |= (8 << 0);   /* AR2R_MAX_PIPE = 8 */
        amp |= (4 << 8);   /* AW2W_MAX_PIPE = 4 */
        amp |= (1 << 16);  /* AW2B_FILL = 1 (AW to B channel) */
        mw(GEM_AMP, amp);
        uart_puts("[macb] GEM_AMP after="); uart_hex(mr(GEM_AMP)); uart_puts("\n");
    }

    if (!mac_load_from_sa1(mac_addr) && !mac_load_from_mailbox(mac_addr)) {
        uart_puts("[macb] No valid MAC\n");
        return false;
    }

    /* Set MAC address (Circle uses MACB offsets 0x98/0x9C, we use GEM 0x88/0x8C) */
    mw(SA1B, (u32)mac_addr[0] | ((u32)mac_addr[1] << 8) |
             ((u32)mac_addr[2] << 16) | ((u32)mac_addr[3] << 24));
    mw(SA1T, (u32)mac_addr[4] | ((u32)mac_addr[5] << 8));

    /* Initial NCFGR: MDC clock + data bus width from DCFG1 + discard FCS
     * (Circle: minimal config before PHY, full config after negotiation) */
    {
        u32 dbwdef = (mr(DCFG1) >> 25) & 0x7;
        u32 dbw;
        if (dbwdef >= 4) dbw = 2;       /* 128-bit */
        else if (dbwdef >= 2) dbw = 1;   /* 64-bit */
        else dbw = 0;                     /* 32-bit */
        mw(NCFGR, NCFGR_CLK_DIV64 | (dbw << 21) | (1 << 17) /* DRFCS */);
    }

    /* Disable all interrupts (polling mode) */
    mw(IDR, 0xFFFFFFFF);
    (void)mr(ISR);

    /* ── Setup RX descriptors ── */
    for (u32 i = 0; i < NUM_RX; i++) {
        volatile struct macb_desc *d = &rx_ring[i];
#if !USE_8BYTE_DESC
        d->addr_hi = MACB_DMA_HI;
        d->rsvd = 0xCAFE0000;  /* canary w3 */
#endif
        d->ctrl = 0xDEAD0000;  /* canary w1 */
        __asm__ volatile("dsb sy" ::: "memory");
        u32 a = (u32)(usize)&rx_bufs[i][0];
        if (i == NUM_RX - 1) a |= RX_ADDR_WRAP;
        d->addr = a;
    }
    rx_idx = 0;

    /* ── Setup TX descriptors ── */
    for (u32 i = 0; i < NUM_TX; i++) {
        volatile struct macb_desc *d = &tx_ring[i];
#if !USE_8BYTE_DESC
        d->addr_hi = MACB_DMA_HI;
        d->rsvd = 0;
#endif
        d->addr = 0;
        d->ctrl = TX_STAT_USED;
        if (i == NUM_TX - 1)
            d->ctrl |= TX_STAT_WRAP;
    }
    tx_idx = 0;

    __asm__ volatile("dsb sy" ::: "memory");
    dcache_clean_range((u64)(usize)rx_ring, sizeof(rx_ring));
    dcache_clean_range((u64)(usize)rx_bufs, sizeof(rx_bufs));
    dcache_clean_range((u64)(usize)tx_ring, sizeof(tx_ring));
    __asm__ volatile("dsb sy" ::: "memory");

    uart_puts("[macb] RX verify: desc[0].addr=");
    uart_hex(rx_ring[0].addr);
    uart_puts(" expected=");
    uart_hex((u32)(usize)&rx_bufs[0][0]);
    uart_puts("\n");

    /* ── Ring base pointers ── */
    mw(RBQP, (u32)(usize)&rx_ring[0]);
    mw(TBQP, (u32)(usize)&tx_ring[0]);
#if !USE_8BYTE_DESC
    mw(RBQPH, MACB_DMA_HI);
    mw(TBQPH, MACB_DMA_HI);
#else
    mw(RBQPH, 0);
    mw(TBQPH, 0);
#endif

    /* (DMACFG already set above before ring pointers) */

    /* ── Multi-queue init (Circle: gmac_init_multi_queues) ── */
    {
        static struct macb_desc dummy_desc ALIGNED(64);
        dummy_desc.ctrl = TX_STAT_USED;
        dummy_desc.addr = 0;
#if !USE_8BYTE_DESC
        dummy_desc.addr_hi = MACB_DMA_HI;
        dummy_desc.rsvd = 0;
#endif
        __asm__ volatile("dsb sy" ::: "memory");
        dcache_clean_range((u64)(usize)&dummy_desc, sizeof(dummy_desc));

        u32 dcfg6 = mr(0x0294);
        u32 queue_mask = (dcfg6 & 0xFF) | 0x01;
        u32 dummy_lo = (u32)(usize)&dummy_desc;
        for (u32 q = 1; q < 8; q++) {
            if (queue_mask & (1 << q)) {
                mw(0x0440 + ((q-1) << 2), dummy_lo);
                mw(0x0480 + ((q-1) << 2), dummy_lo);
#if !USE_8BYTE_DESC
                mw(0x04C8, MACB_DMA_HI);
                mw(0x04D4, MACB_DMA_HI);
#endif
            }
        }
        uart_puts("[macb] Multi-queue init: mask=");
        uart_hex(queue_mask);
        uart_puts("\n");
    }

    /* USRIO: RGMII mode only (Circle: GEM_BIT(RGMII)) */
    mw(USRIO, USRIO_RGMII);

    /* Enable TX + RX only (NO MPE — Circle enables MPE per MDIO op) */
    mw(NCR, NCR_RE | NCR_TE);

    if (!phy_select())
        return false;

    /* PHY: inspect link partner on detected address */
    uart_puts("[macb] PHY BMSR=");
    u16 bmsr = mdio_read(phy_addr, 0x01);
    uart_hex(bmsr);
    uart_puts("\n");

    /* Wait for link (5 seconds) */
    uart_puts("[macb] Waiting for link (5s)...\n");
    for (u32 s = 0; s < 50; s++) {
        bmsr = mdio_read(phy_addr, 0x01);
        if (bmsr & (1 << 2)) {  /* BMSR link status */
            uart_puts("[macb] Link UP at poll ");
            uart_hex(s);
            uart_puts(" BMSR=");
            uart_hex(bmsr);
            uart_puts("\n");
            break;
        }
        timer_delay_ms(100);
        /* Spinner */
        static const char spin[] = "|/-\\";
        fb_set_cursor(126, 0);
        fb_set_color(0x00CCAA00, 0x00000000);
        fb_putc(spin[s & 3]);
    }

    mac_log("[macb] Active MAC ", mac_addr);

    /* Read negotiated speed from PHY and configure MAC to match */
    {
        u16 gbsr = mdio_read(phy_addr, 0x0A);  /* 1000BASE-T Status */
        u16 anlpar = mdio_read(phy_addr, 0x05); /* AN Link Partner Ability */
        u16 bmcr = mdio_read(phy_addr, 0x00);   /* Basic Mode Control */
        bool gig = false, fd = true, spd100 = false;

        uart_puts("[macb] PHY GBSR="); uart_hex(gbsr);
        uart_puts(" ANLPAR="); uart_hex(anlpar);
        uart_puts(" BMCR="); uart_hex(bmcr);
        uart_puts("\n");

        /* Check 1000BASE-T first */
        if ((gbsr & (1 << 11)) || (gbsr & (1 << 10))) {
            gig = true;
            fd = !!(gbsr & (1 << 11)); /* 1000BASE-T FD */
        } else if (anlpar & (1 << 8)) {
            spd100 = true; fd = true;  /* 100BASE-TX FD */
        } else if (anlpar & (1 << 7)) {
            spd100 = true; fd = false; /* 100BASE-TX HD */
        } else if (anlpar & (1 << 6)) {
            fd = true;  /* 10BASE-T FD */
        }

        /* Reconfigure NCFGR with correct speed — PRESERVE DBW! */
        u32 dbwdef2 = (mr(DCFG1) >> 25) & 0x7;
        u32 dbw2;
        if (dbwdef2 >= 4) dbw2 = 2;       /* 128-bit */
        else if (dbwdef2 >= 2) dbw2 = 1;   /* 64-bit */
        else dbw2 = 0;                     /* 32-bit */
        u32 ncfgr = NCFGR_BIG | NCFGR_CLK_DIV64 | NCFGR_CAF | NCFGR_RXCOEN
                   | (dbw2 << 21) | (1 << 17) /* DRFCS */;
        if (gig)    ncfgr |= NCFGR_GBE;
        if (spd100) ncfgr |= NCFGR_SPD;
        if (fd)     ncfgr |= NCFGR_FD;
        mw(NCFGR, ncfgr);

        uart_puts("[macb] Negotiated: ");
        uart_puts(gig ? "1000" : (spd100 ? "100" : "10"));
        uart_puts(fd ? "Mbps FD" : "Mbps HD");
        uart_puts(" NCFGR=");
        uart_hex(ncfgr);
        uart_puts("\n");

        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf("MACB %s%s NCFGR=%X\n",
                  gig ? "1000" : (spd100 ? "100" : "10"),
                  fd ? "FD" : "HD", ncfgr);
    }

    /* ── Verbose MACB DMA state dump ── */
    uart_puts("[macb] === MACB Register Dump ===\n");
    uart_puts("[macb] NCR="); uart_hex(mr(NCR)); uart_puts("\n");
    uart_puts("[macb] NCFGR="); uart_hex(mr(NCFGR)); uart_puts("\n");
    uart_puts("[macb] NSR="); uart_hex(mr(NSR)); uart_puts("\n");
    uart_puts("[macb] DMACFG="); uart_hex(mr(DMACFG)); uart_puts("\n");
    uart_puts("[macb] USRIO="); uart_hex(mr(USRIO)); uart_puts("\n");
    uart_puts("[macb] RBQP="); uart_hex(mr(RBQP));
    uart_puts(" RBQPH="); uart_hex(mr(RBQPH)); uart_puts("\n");
    uart_puts("[macb] TBQP="); uart_hex(mr(TBQP));
    uart_puts(" TBQPH="); uart_hex(mr(TBQPH)); uart_puts("\n");
    uart_puts("[macb] ISR="); uart_hex(mr(ISR)); uart_puts("\n");
    uart_puts("[macb] TSR="); uart_hex(mr(TSR)); uart_puts("\n");
    uart_puts("[macb] RSR="); uart_hex(mr(RSR)); uart_puts("\n");
    uart_puts("[macb] RX desc[0] addr="); uart_hex(rx_ring[0].addr);
    uart_puts(" ctrl="); uart_hex(rx_ring[0].ctrl); uart_puts("\n");
    uart_puts("[macb] TX desc[0] addr="); uart_hex(tx_ring[0].addr);
    uart_puts(" ctrl="); uart_hex(tx_ring[0].ctrl); uart_puts("\n");
    uart_puts("[macb] rx_bufs[0] phys="); uart_hex((u64)(usize)&rx_bufs[0][0]); uart_puts("\n");
    uart_puts("[macb] tx_bufs[0] phys="); uart_hex((u64)(usize)&tx_bufs[0][0]); uart_puts("\n");

    /* HDMI dump */
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_printf("MACB NCR=%X NCFGR=%X NSR=%X\n", mr(NCR), mr(NCFGR), mr(NSR));
    fb_printf("MACB RBQP=%X TBQP=%X\n", mr(RBQP), mr(TBQP));
    fb_printf("MACB ISR=%X TSR=%X RSR=%X\n", mr(ISR), mr(TSR), mr(RSR));
    fb_printf("MACB rxdesc0=%X txdesc0=%X\n", rx_ring[0].addr, tx_ring[0].addr);
    fb_printf("MACB rxbuf0=%X txbuf0=%X\n",
              (u32)(usize)&rx_bufs[0][0], (u32)(usize)&tx_bufs[0][0]);
    /* ── PCIe BAR2 state (visible here since UART is active) ── */
    uart_puts("[macb] PCIe BAR2_LO="); uart_hex(mmio_read(0x1000120000ULL + 0x4034));
    uart_puts(" BAR2_HI="); uart_hex(mmio_read(0x1000120000ULL + 0x4038));
    uart_puts("\n");
    uart_puts("[macb] UBUS_REMAP="); uart_hex(mmio_read(0x1000120000ULL + 0x40B4));
    uart_puts(" REMAP_HI="); uart_hex(mmio_read(0x1000120000ULL + 0x40B0));
    uart_puts("\n");
    uart_puts("[macb] MISC_CTRL="); uart_hex(mmio_read(0x1000120000ULL + 0x4008));
    uart_puts("\n");

    /* ── DMA write-back test: check if MAC can modify descriptor memory ──
     * Write a canary to rx_ring[NUM_RX-1].ctrl, clear it, wait, read back.
     * If MAC touched it (e.g. BNA sets descriptor state), we'll see it. */
    {
        volatile u32 *test_word = (volatile u32 *)(usize)&rx_ring[0].ctrl;
        u32 before = *test_word;
        /* Wait a moment for any pending DMA */
        __asm__ volatile("dsb sy; isb" ::: "memory");
        delay_cycles(1000000);
        __asm__ volatile("dsb sy; isb" ::: "memory");
        u32 after = *test_word;
        uart_puts("[macb] DMA test: desc[0].ctrl before="); uart_hex(before);
        uart_puts(" after="); uart_hex(after);
        uart_puts(before != after ? " CHANGED!\n" : " unchanged\n");
    }

    uart_puts("[macb] === End Dump ===\n");

    return true;
}

/* ── Send ── */
static u32 tx_send_count;

bool macb_send(const u8 *frame, u32 len) {
    if (len > BUF_SIZE || len == 0) return false;

    /* Invalidate TX descriptor to see MAC's latest USED bit */
    dcache_invalidate_range((u64)(usize)&tx_ring[tx_idx], sizeof(struct macb_desc));

    /* Wait for TX descriptor to be available */
    if (!(tx_ring[tx_idx].ctrl & TX_STAT_USED))
        return false;  /* busy */

    /* Copy frame to TX buffer */
    u8 *dst = tx_bufs[tx_idx];
    for (u32 i = 0; i < len; i++) dst[i] = frame[i];

    /* Setup descriptor (Circle: set addr during send, then barrier, then ctrl) */
#if !USE_8BYTE_DESC
    tx_ring[tx_idx].addr_hi = MACB_DMA_HI;
#endif
    __asm__ volatile("dsb sy" ::: "memory");
    tx_ring[tx_idx].addr = (u32)(usize)&tx_bufs[tx_idx][0];
    __asm__ volatile("dsb sy" ::: "memory");

    u32 ctrl = len & TX_STAT_LEN_MASK;
    ctrl |= TX_STAT_LAST;
    if (tx_idx == NUM_TX - 1) ctrl |= TX_STAT_WRAP;
    tx_ring[tx_idx].ctrl = ctrl;
    __asm__ volatile("dsb sy" ::: "memory");

    /* Trigger TX */
    mw(NCR, mr(NCR) | NCR_TSTART);

    /* Log first few TX attempts */
    if (tx_send_count < 5) {
        uart_puts("[macb] TX#");
        uart_hex(tx_send_count);
        uart_puts(" idx=");
        uart_hex(tx_idx);
        uart_puts(" len=");
        uart_hex(len);
        uart_puts(" ctrl=");
        uart_hex(ctrl);
        uart_puts(" addr=");
        uart_hex(tx_ring[tx_idx].addr);
        uart_puts("\n");
        /* Wait a moment then check if MAC consumed it + AER errors */
        delay_cycles(100000);
        dcache_invalidate_range((u64)(usize)&tx_ring[tx_idx], sizeof(struct macb_desc));
        uart_puts("[macb] TX post: ctrl=");
        uart_hex(tx_ring[tx_idx].ctrl);
        uart_puts(" TSR=");
        uart_hex(mr(TSR));
        uart_puts(" ISR=");
        uart_hex(mr(ISR));
        uart_puts(" NSR=");
        uart_hex(mr(NSR));
        uart_puts("\n");
        pcie_aer_dump("post-TX");
    }
    tx_send_count++;

    tx_idx = (tx_idx + 1) % NUM_TX;
    return true;
}

/* ── Receive ── */
static u32 rx_diag_count;
bool macb_recv(u8 *frame, u32 *len) {
    /* Force visibility of DMA writes */
    __asm__ volatile("dsb sy; isb" ::: "memory");

    /* Read descriptor via volatile pointer */
    volatile u32 *raw = (volatile u32 *)(usize)&rx_ring[rx_idx];
    u32 addr_val = raw[0];
    u32 ctrl_val = raw[1];

    /* Periodic diagnostic: dump first descriptor + RSR */
    if ((rx_diag_count & 0xFFFFF) == 0 && rx_diag_count > 0 && rx_diag_count < 0x500000) {
        uart_puts("[macb] RX diag idx="); uart_hex(rx_idx);
        uart_puts(" raw[0]="); uart_hex(addr_val);
        uart_puts(" raw[1]="); uart_hex(ctrl_val);
        uart_puts(" RSR="); uart_hex(mr(RSR));
        uart_puts(" RBQP="); uart_hex(mr(RBQP));
        uart_puts("\n");
        /* Clear RSR to see if new events occur */
        mw(RSR, mr(RSR));
    }
    rx_diag_count++;

    /* Check if current RX descriptor has been filled by MAC */
    if (!(addr_val & RX_ADDR_OWN))
        return false;

    u32 status = raw[1];
    u32 flen = status & RX_STAT_LEN_MASK;

    /* Dump first few OWN-set descriptors to see all words */
    static u32 own_dump_count;
    if (own_dump_count < 3) {
        uart_puts("[macb] RX OWN set! idx="); uart_hex(rx_idx);
        uart_puts(" w0="); uart_hex(addr_val);
        uart_puts(" w1="); uart_hex(status);
#if !USE_8BYTE_DESC
        uart_puts(" w2="); uart_hex(raw[2]);
        uart_puts(" w3="); uart_hex(raw[3]);
#endif
        uart_puts(" flen="); uart_hex(flen);
        uart_puts("\n");
        /* Scan 64 bytes around descriptor for misplaced writes */
        volatile u32 *scan = (volatile u32 *)((usize)raw - 16);
        uart_puts("[macb] memscan: ");
        for (u32 s = 0; s < 16; s++) {
            uart_hex(scan[s]);
            uart_puts(" ");
        }
        uart_puts("\n");
        /* Also dump first 32 bytes of RX buffer to check for frame data */
        volatile u32 *bufp = (volatile u32 *)(usize)rx_bufs[rx_idx];
        uart_puts("[macb] buf: ");
        for (u32 s = 0; s < 8; s++) {
            uart_hex(bufp[s]);
            uart_puts(" ");
        }
        uart_puts("\n");
        own_dump_count++;
    }

    if (flen == 0 || flen > BUF_SIZE) {
        /* Reclaim descriptor */
        rx_ring[rx_idx].addr &= ~RX_ADDR_OWN;
        dcache_clean_range((u64)(usize)&rx_ring[rx_idx], sizeof(struct macb_desc));
        rx_idx = (rx_idx + 1) % NUM_RX;
        return false;
    }

    /* Invalidate RX buffer to see data written by MAC via DMA */
    dcache_invalidate_range((u64)(usize)rx_bufs[rx_idx], BUF_SIZE);

    /* Copy frame out */
    u8 *src = rx_bufs[rx_idx];
    for (u32 i = 0; i < flen; i++) frame[i] = src[i];
    *len = flen;

    /* Reclaim: clear ownership bit and flush back to RAM */
    rx_ring[rx_idx].addr &= ~RX_ADDR_OWN;
    dcache_clean_range((u64)(usize)&rx_ring[rx_idx], sizeof(struct macb_desc));
    dsb();

    rx_idx = (rx_idx + 1) % NUM_RX;
    return true;
}

/* ── MAC address ── */
void macb_get_mac(u8 *mac) {
    for (u32 i = 0; i < 6; i++) mac[i] = mac_addr[i];
}

/* ── Link status ── */
bool macb_link_up(void) {
    /* BMSR link status (bit 2) is latching-low: read twice to get current state.
     * First read clears any latched-low condition, second gives live status. */
    (void)mdio_read(phy_addr, 0x01);
    return (mdio_read(phy_addr, 0x01) & (1 << 2)) != 0;
}
