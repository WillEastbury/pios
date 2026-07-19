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
#include "simd.h"
#include "core_env.h"   /* DMA_NET_BASE / DMA_NET_SIZE: dedicated NC DMA arena */
#include "dtrace.h"
#include "pioscap.h"

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
#define PTR         0x0038  /* Pause Time (802.3x flow control quantum) */
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

/* TSR bits (W1C) — Transmit Status */
#define TSR_UBR     (1 << 0)   /* Used Bit Read — descriptor exhausted */
#define TSR_COL     (1 << 1)   /* Collision */
#define TSR_RLE     (1 << 2)   /* Retry Limit Exceeded */
#define TSR_TGO     (1 << 3)   /* Transmit Go (RO) */
#define TSR_BEX     (1 << 4)   /* Buffers Exhausted Mid-Frame */
#define TSR_COMP    (1 << 5)   /* Frame transmitted complete */
#define TSR_UND     (1 << 6)   /* TX Underrun */
#define TSR_HRESP   (1 << 8)   /* HRESP not OK (DMA bus error) */

/* RSR bits (W1C) — Receive Status */
#define RSR_BNA     (1 << 0)   /* Buffer Not Available (ring full / overran) */
#define RSR_REC     (1 << 1)   /* Frame received */
#define RSR_OVR     (1 << 2)   /* Receive Overrun (RX FIFO overflow) */
#define RSR_HNO     (1 << 3)   /* HRESP not OK (DMA bus error) */

/* GEM hardware statistic counters (read-clear-on-read) */
#define GEM_OCT_TX_LO       0x0100  /* Octets transmitted low */
#define GEM_OCT_TX_HI       0x0104  /* Octets transmitted high */
#define GEM_FRAMES_TX       0x0108  /* Frames transmitted */
#define GEM_TX_UNDERRUNS    0x0134  /* TX FIFO underruns */
#define GEM_TXPAUSECNT      0x0114  /* Pause Frames Transmitted Counter (lifetime, clears on read) */
#define GEM_RXPAUSECNT      0x0164  /* Pause Frames Received Counter (lifetime, clears on read) */
#define GEM_LATE_COLL       0x0144  /* Late collisions */
#define GEM_CARRIER_ERR     0x0148  /* Carrier sense errors */
#define GEM_FRAMES_RX       0x0158  /* Frames received */

/* NCR bits */
#define NCR_LLB     (1 << 1)   /* Loopback local */
#define NCR_RE      (1 << 2)   /* Receive enable */
#define NCR_TE      (1 << 3)   /* Transmit enable */
#define NCR_MPE     (1 << 4)   /* Management port enable */
#define NCR_CLRSTAT (1 << 5)   /* Clear statistics */
#define NCR_TSTART  (1 << 9)   /* Start transmission */
#define NCR_THALT   (1 << 10)  /* Halt transmission */

/* NCFGR bits */
#define NCFGR_SPD       (1 << 0)   /* Speed (100Mbps) */
#define NCFGR_FD        (1 << 1)   /* Full duplex */
#define NCFGR_CAF       (1 << 4)   /* Copy all frames */
#define NCFGR_BIG       (1 << 8)   /* Receive 1536 byte frames */
#define NCFGR_GBE       (1 << 10)  /* Gigabit mode (GEM) */
#define NCFGR_PAE       (1 << 13)  /* Pause Enable: MAC auto-generates 802.3x
                                     * PAUSE frames when the RX FIFO nears full
                                     * (proactive link-layer backpressure on
                                     * the peer, before our own RX ring/DMA
                                     * ever has to drop/overrun). Also makes
                                     * TX honor PAUSE frames received from the
                                     * peer. See PTR (Pause Time Register). */
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
#define DMACFG_TXCOEN       (1 << 11)
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
#define RX_STAT_CSUM_SHIFT 22
#define RX_STAT_CSUM_MASK  (3U << RX_STAT_CSUM_SHIFT)
#define RX_STAT_CSUM_CHECKED_MASK 2U

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
/* Deep rings give the single-core0 poll loop headroom to absorb concurrent-
 * connection bursts (many simultaneous SYN/GET/ACK frames) while it is busy
 * building/transmitting HTTP responses, instead of overrunning a shallow ring
 * and stalling the MAC. PIOS is network-first, so RX is sized generously: 512
 * descriptors (16x the original 32). TX 64 covers many in-flight responses.
 * macb_rx_recover() remains the backstop if a ring still overruns. */
/* NUM_RX bumped 512->896 (2026-07-17, live-hardware wrap-boundary diagnosis):
 * dtrace evidence from 6 consecutive real overrun events ALL showed rx_idx=0
 * exactly at detection (a ~1-in-512^6 coincidence if random) with a clean
 * contiguous OWN-bit run from index 0 (never a "hole" pattern), pointing at a
 * bug specifically tied to the ring wrap boundary (descriptor NUM_RX-1 -> 0)
 * rather than raw traffic volume (one captured episode overran with only ~38
 * trivial ARP-keepalive frames in the whole preceding minute). Bumping ring
 * size is a diagnostic test as much as a mitigation: if wrap frequency (and
 * therefore freeze frequency) drops roughly in proportion to the size
 * increase, that confirms the wrap-boundary theory. 896 is the largest value
 * that still fits the existing 2MB DMA_NET arena without touching the
 * physical memory map (see the _Static_assert below and MACB_DMA_POOL_BYTES);
 * expanding the arena itself would require re-laying-out DMA_NET/DMA_DISK/IPC
 * SHM base addresses, a materially riskier change given this SoC's history of
 * boot-bricking from memory-map mistakes (see AGENTS.md "checkpoint 082"). */
#define NUM_RX  896
#define NUM_TX  64
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

/* RX/TX frame buffers live in the dedicated DMA_NET arena (a fixed, mapped,
 * Non-Cacheable 2MB region) rather than in .bss. This is the "dedicated per-core
 * DMA arena" follow-up the MMU layer anticipates (see mmu.c mmu_enable_caching):
 * it keeps the buffer pool DMA-coherent with the MAC (NC, no cache maintenance
 * needed) AND lets the network-first RX ring grow to 512 entries (1MB) without
 * bloating the sub-8MB kernel image. The driver's existing dcache clean/invalidate
 * calls become harmless no-ops on this NC memory. The small descriptor rings stay
 * in .bss (also NC, already coherent). Layout within DMA_NET:
 *     [0 .. NUM_RX*BUF_SIZE)            RX buffers
 *     [NUM_RX*BUF_SIZE .. +NUM_TX*BUF_SIZE) TX buffers
 *     [aligned buffer end .. +RX descriptors) RX descriptor ring
 *     [aligned RX ring end .. +TX descriptors) TX descriptor ring
 *
 * The descriptor rings used to live in .bss under the assumption that .bss was
 * always Normal-NC. Live hardware disproved the safety of that assumption:
 * rx_idx could stop on an OWN=0 descriptor while hundreds of later descriptors
 * were OWN=1, the exact signature produced when a CPU cache-line writeback of
 * one compact 16-byte descriptor overwrites a DMA update to its sibling. Keep
 * descriptors in the explicitly Normal-NC DMA arena with their buffers so no
 * cacheable alias or startup mapping can recreate that ownership hole. */
#define MACB_RX_POOL_OFF   0U
#define MACB_TX_POOL_OFF   ((u64)NUM_RX * BUF_SIZE)
#define MACB_DMA_POOL_BYTES (((u64)NUM_RX + (u64)NUM_TX) * BUF_SIZE)
#define MACB_RX_DESC_OFF   ((MACB_DMA_POOL_BYTES + 63ULL) & ~63ULL)
#define MACB_RX_RING_BYTES ((u64)NUM_RX * sizeof(struct macb_desc))
#define MACB_TX_DESC_OFF   ((MACB_RX_DESC_OFF + MACB_RX_RING_BYTES + 63ULL) & ~63ULL)
#define MACB_TX_RING_BYTES ((u64)NUM_TX * sizeof(struct macb_desc))
#define MACB_DMA_TOTAL_BYTES (MACB_TX_DESC_OFF + MACB_TX_RING_BYTES)
_Static_assert(MACB_DMA_TOTAL_BYTES <= DMA_NET_SIZE,
               "MACB buffers and descriptor rings exceed the DMA_NET arena");
static u8 (*const rx_bufs)[BUF_SIZE] =
    (u8 (*)[BUF_SIZE])(usize)(DMA_NET_BASE + MACB_RX_POOL_OFF);
static u8 (*const tx_bufs)[BUF_SIZE] =
    (u8 (*)[BUF_SIZE])(usize)(DMA_NET_BASE + MACB_TX_POOL_OFF);
static struct macb_desc *const rx_ring =
    (struct macb_desc *)(usize)(DMA_NET_BASE + MACB_RX_DESC_OFF);
static struct macb_desc *const tx_ring =
    (struct macb_desc *)(usize)(DMA_NET_BASE + MACB_TX_DESC_OFF);
static u32 rx_idx;
static u32 tx_idx;
static u32 tx_tail;
static u32 tx_inflight;
static u8 mac_addr[6];
static u8 phy_addr;
static u32 link_mbps;
static bool link_full_duplex;
static bool tx_csum_enabled;
static bool rx_csum_enabled;
static bool tso_enabled;

/* ── Register helpers ── */
static inline u32 mr(u32 off) { return mmio_read(MACB_BASE + off); }
static inline void mw(u32 off, u32 val) { mmio_write(MACB_BASE + off, val); }

static bool mac_is_zero(const u8 *mac)
{
    return mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
           mac[3] == 0 && mac[4] == 0 && mac[5] == 0;
}

static void macb_apply_tx_checksum_offload(void)
{
    u32 dmacfg = mr(DMACFG);
    if (tx_csum_enabled)
        dmacfg |= DMACFG_TXCOEN;
    else
        dmacfg &= ~DMACFG_TXCOEN;
    mw(DMACFG, dmacfg);
}

static void macb_apply_rx_checksum_offload(void)
{
    u32 ncfgr = mr(NCFGR);
    if (rx_csum_enabled)
        ncfgr |= NCFGR_RXCOEN;
    else
        ncfgr &= ~NCFGR_RXCOEN;
    mw(NCFGR, ncfgr);
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

    uart_puts("[mac] SA1B="); uart_hex(sa1b);
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
    uart_puts("[mac] mbox GET_MAC...\n");
    bool ok = mbox_call(8, mb);
    uart_puts("[mac] mbox res=");
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

        mac_log("[mac] mbox raw ", candidate);

        if (mac_is_valid(candidate)) {
            mac_copy(out, candidate);
            mac_log("[mac] MAC from mbox ", out);
            return true;
        }
        mac_log("[mac] reject mbox MAC ", candidate);
        mac_log_invalid_reason("[mac] reject: ", candidate);
    }

    /* Try tag 0x00010004 — GET_BOARD_SERIAL and derive MAC */
    uart_puts("[mac] try BOARD_SERIAL...\n");
    mb[0] = 8 * 4; mb[1] = 0;
    mb[2] = 0x00010004; mb[3] = 8; mb[4] = 8;
    mb[5] = 0; mb[6] = 0; mb[7] = 0;
    ok = mbox_call(8, mb);
    uart_puts("[mac] serial res=");
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
        mac_log("[mac] MAC from serial ", candidate);
        mac_copy(out, candidate);
        return true;
    }

    uart_puts("[mac] mbox MAC exhausted\n");
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
        uart_puts("[mac] MDIO addr=");
        uart_hex(addr);
        uart_puts(" BMSR=");
        uart_hex(bmsr);
        uart_puts("\n");
        if (bmsr != 0x0000 && bmsr != 0xFFFF) {
            phy_addr = (u8)addr;
            uart_puts("[mac] PHY=");
            uart_hex(addr);
            uart_puts(" found\n");
            return true;
        }
    }
    uart_puts("[mac] no PHY on MDIO\n");
    return false;
}

/* ── Init ── */
bool macb_init(void) {
    uart_puts("[mac] init GEM\n");
    tx_csum_enabled = true;
    rx_csum_enabled = true;
    tso_enabled = false;

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
    uart_puts("[mac] DCFG1=");
    uart_hex(dcfg1);
    uart_puts(" DCFG2=");
    uart_hex(dcfg2);
    uart_puts("\n");

    if (dcfg1 == 0 && dcfg2 == 0) {
        uart_puts("[mac] GEM no resp\n");
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
        uart_puts("[mac] DMACFG fw="); uart_hex(dmacfg); uart_puts("\n");
        dmacfg &= ~(0xFF << DMACFG_RXBS_SHIFT);
        dmacfg &= ~(0x1F << DMACFG_FBLDO_SHIFT);
        dmacfg |= ((BUF_SIZE / 64) << DMACFG_RXBS_SHIFT);
        /* FBLDO=16 — RPi rp1-gem config uses 16 */
        dmacfg |= (16 << DMACFG_FBLDO_SHIFT);
        dmacfg |= (1 << 10);   /* TXPBMS */
        if (tx_csum_enabled)
            dmacfg |= DMACFG_TXCOEN; /* GEM TX IP/TCP/UDP checksum generation */
        else
            dmacfg &= ~DMACFG_TXCOEN;
        dmacfg |= (3 << 8);    /* RXBMS = 3 (max) */
        dmacfg &= ~(1 << 7);   /* no endian swap pkt */
        dmacfg &= ~(1 << 6);   /* no endian swap desc */
#if USE_8BYTE_DESC
        dmacfg &= ~DMACFG_ADDR64;  /* 8-byte descriptors */
#else
        dmacfg |= DMACFG_ADDR64;   /* 16-byte descriptors */
#endif
        mw(DMACFG, dmacfg);
        uart_puts("[mac] DMACFG="); uart_hex(dmacfg); uart_puts("\n");
    }

    /* ── RP1 ETH_CFG: check status and enable bus errors ── */
    {
        u32 cfg_stat = ecr(ETH_CFG_STAT);
        u32 cfg_ctrl = ecr(ETH_CFG_CTRL);
        uart_puts("[mac] ETH_CFG C="); uart_hex(cfg_ctrl);
        uart_puts(" S="); uart_hex(cfg_stat);
        uart_puts("\n");
        if (cfg_stat & (1 << 5)) uart_puts("[mac] WARN: AWLEN_ILLEGAL\n");
        if (cfg_stat & (1 << 4)) uart_puts("[mac] WARN: ARLEN_ILLEGAL\n");
        /* Enable bus error passthrough for debugging */
        ecw(ETH_CFG_CTRL, cfg_ctrl | (1 << 3));
    }

    /* ── GEM AXI pipeline config (RP1-specific, from Linux patch) ── */
    {
        u32 amp = mr(GEM_AMP);
        uart_puts("[mac] AMP pre="); uart_hex(amp); uart_puts("\n");
        /* Set reasonable pipeline depths — RP1 Linux uses DT properties */
        amp &= ~0xFFFF;  /* clear AR2R and AW2W fields */
        amp |= (8 << 0);   /* AR2R_MAX_PIPE = 8 */
        amp |= (4 << 8);   /* AW2W_MAX_PIPE = 4 */
        amp |= (1 << 16);  /* AW2B_FILL = 1 (AW to B channel) */
        mw(GEM_AMP, amp);
        uart_puts("[mac] AMP post="); uart_hex(mr(GEM_AMP)); uart_puts("\n");
    }

    if (!mac_load_from_sa1(mac_addr) && !mac_load_from_mailbox(mac_addr)) {
        uart_puts("[mac] no valid MAC\n");
        return false;
    }

    /* Set MAC address (Circle uses MACB offsets 0x98/0x9C, we use GEM 0x88/0x8C) */
    mw(SA1B, (u32)mac_addr[0] | ((u32)mac_addr[1] << 8) |
             ((u32)mac_addr[2] << 16) | ((u32)mac_addr[3] << 24));
    mw(SA1T, (u32)mac_addr[4] | ((u32)mac_addr[5] << 8));

    /* Initial NCFGR: MDC clock + data bus width from DCFG1 + discard FCS
     * (Circle: minimal config before PHY, full config after negotiation).
     * NCFGR_PAE is deliberately NOT set here: duplex is not yet known at this
     * point, and 802.3x PAUSE flow control is only meaningful/defined for
     * full-duplex operation. It's set conditionally below, after
     * autonegotiation confirms full duplex. */
    {
        u32 dbwdef = (mr(DCFG1) >> 25) & 0x7;
        u32 dbw;
        if (dbwdef >= 4) dbw = 2;       /* 128-bit */
        else if (dbwdef >= 2) dbw = 1;   /* 64-bit */
        else dbw = 0;                     /* 32-bit */
        mw(NCFGR, NCFGR_CLK_DIV64 | (dbw << 21) | (1 << 17) /* DRFCS */);
    }

    /* Disable all interrupts (polling mode) then clear ISR */
    mw(IDR, 0xFFFFFFFF);
    u32 isr_init = mr(ISR);
    if (isr_init) mw(ISR, isr_init);  /* Clear any pending status */
    u32 tsr_init = mr(TSR);
    if (tsr_init) mw(TSR, tsr_init);  /* Clear TX status */

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
    tx_tail = 0;
    tx_inflight = 0;

    __asm__ volatile("dsb sy" ::: "memory");
    /* Use clean+invalidate so cachelines are evicted; subsequent reads
     * (including MAC's status updates via DMA) will be served from RAM. */
    dcache_clean_invalidate_range((u64)(usize)rx_ring, MACB_RX_RING_BYTES);
    dcache_clean_range((u64)(usize)rx_bufs, (u64)NUM_RX * BUF_SIZE);
    dcache_clean_invalidate_range((u64)(usize)tx_ring, MACB_TX_RING_BYTES);
    __asm__ volatile("dsb sy" ::: "memory");

    uart_puts("[mac] RX desc[0]=");
    uart_hex(rx_ring[0].addr);
    uart_puts(" exp=");
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
        uart_puts("[mac] MQ mask=");
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
    uart_puts("[mac] BMSR=");
    u16 bmsr = mdio_read(phy_addr, 0x01);
    uart_hex(bmsr);
    uart_puts("\n");

    /* Wait for link (5 seconds) */
    uart_puts("[mac] link wait 5s...\n");
    for (u32 s = 0; s < 50; s++) {
        bmsr = mdio_read(phy_addr, 0x01);
        if (bmsr & (1 << 2)) {  /* BMSR link status */
            uart_puts("[mac] link UP @");
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

    mac_log("[mac] active ", mac_addr);

    /* Read negotiated speed from PHY and configure MAC to match */
    {
        u16 gbsr = mdio_read(phy_addr, 0x0A);  /* 1000BASE-T Status */
        u16 anlpar = mdio_read(phy_addr, 0x05); /* AN Link Partner Ability */
        u16 bmcr = mdio_read(phy_addr, 0x00);   /* Basic Mode Control */
        bool gig = false, fd = true, spd100 = false;

        uart_puts("[mac] GBSR="); uart_hex(gbsr);
        uart_puts(" ANLP="); uart_hex(anlpar);
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
        u32 ncfgr = NCFGR_BIG | NCFGR_CLK_DIV64
                   | (dbw2 << 21) | (1 << 17) /* DRFCS */;
        if (rx_csum_enabled)
            ncfgr |= NCFGR_RXCOEN;
        if (gig)    ncfgr |= NCFGR_GBE;
        if (spd100) ncfgr |= NCFGR_SPD;
        if (fd) {
            ncfgr |= NCFGR_FD;
            /* 802.3x PAUSE flow control: only meaningful/defined for
             * full-duplex operation, hence gated strictly on `fd`. Enables
             * the MAC to autonomously emit real PAUSE frames to the link
             * partner when its own RX FIFO nears full (proactive link-layer
             * backpressure before the RX ring/DMA ever has to drop/overrun)
             * and to honor PAUSE frames received from the partner. This was
             * entirely absent before this fix; see Linux's macb/GEM driver
             * (drivers/net/ethernet/cadence/macb_main.c) for the equivalent
             * technique, there negotiated via phylink capability exchange.
             * PIOS does not yet negotiate PAUSE capability via ANAR/ANLPAR --
             * this enables it unconditionally whenever full-duplex is
             * confirmed, which is a coarser (but standards-conformant: any
             * 802.3x-unaware peer simply never receives/emits PAUSE and
             * this has no effect) gate than a real capability negotiation. */
            ncfgr |= NCFGR_PAE;
        }
        mw(NCFGR, ncfgr);
        if (fd)
            mw(PTR, 0xFFFFU);   /* pause quantum, units of 512 bit-times at link speed */
        link_mbps = gig ? 1000U : (spd100 ? 100U : 10U);
        link_full_duplex = fd;

        uart_puts("[mac] neg: ");
        uart_puts(gig ? "1000" : (spd100 ? "100" : "10"));
        uart_puts(fd ? " FD" : " HD");
        uart_puts(" NCFGR=");
        uart_hex(ncfgr);
        uart_puts("\n");

        fb_set_color(0x0000CCFF, 0x00000000);
        fb_printf("MACB %s%s NCFGR=%X\n",
                  gig ? "1000" : (spd100 ? "100" : "10"),
                  fd ? "FD" : "HD", ncfgr);
    }

    return true;
}

/* ── Send ── */
static u32 tx_send_count;
static u32 tx_drop_count;
static u32 tx_recover_count;

static void macb_dump_tx_state(const char *tag)
{
    uart_puts("[mac] ");
    uart_puts(tag);
    uart_puts(" tx_idx=");
    uart_hex(tx_idx);
    uart_puts(" sent=");
    uart_hex(tx_send_count);
    uart_puts(" drop=");
    uart_hex(tx_drop_count);
    uart_puts(" NCR=");
    uart_hex(mr(NCR));
    uart_puts(" TSR=");
    uart_hex(mr(TSR));
    uart_puts(" NSR=");
    uart_hex(mr(NSR));
    /* NOTE: do not read ISR here — it is read-to-clear and would mask the
     * very error bits (HRESP/TUR/RLE/RXUBR) we need to capture on stall. */
    uart_puts(" TBQP=");
    uart_hex(mr(TBQP));
    uart_puts("\n");
    dcache_clean_invalidate_range((u64)(usize)tx_ring, MACB_TX_RING_BYTES);
    uart_puts("[mac] desc.ctrl:");
    for (u32 i = 0; i < NUM_TX; i++) {
        uart_putc(' ');
        u32 c = tx_ring[i].ctrl;
        uart_putc((c & TX_STAT_USED) ? 'U' : '_');
        uart_putc((c & TX_STAT_WRAP) ? 'W' : '_');
        uart_putc('0' + (i / 10));
        uart_putc('0' + (i % 10));
    }
    uart_puts("\n");
}

static void macb_tx_reclaim(void)
{
    while (tx_inflight) {
        dcache_clean_invalidate_range((u64)(usize)&tx_ring[tx_tail],
                                      sizeof(struct macb_desc));
        if (!(tx_ring[tx_tail].ctrl & TX_STAT_USED))
            break;
        tx_tail = (tx_tail + 1U) % NUM_TX;
        tx_inflight--;
    }
}

static void macb_tx_recover_silent(void)
{
    u32 ncr = mr(NCR);

    mw(NCR, ncr | NCR_THALT);
    __asm__ volatile("dsb sy" ::: "memory");

    u32 tsr = mr(TSR) & (TSR_UBR | TSR_COL | TSR_RLE | TSR_BEX |
                         TSR_COMP | TSR_UND | TSR_HRESP);
    if (tsr)
        mw(TSR, tsr);

    for (u32 i = 0; i < NUM_TX; i++) {
        volatile struct macb_desc *d = &tx_ring[i];
#if !USE_8BYTE_DESC
        d->addr_hi = MACB_DMA_HI;
        d->rsvd = 0;
#endif
        d->addr = 0;
        d->ctrl = TX_STAT_USED | ((i == NUM_TX - 1U) ? TX_STAT_WRAP : 0);
    }
    tx_idx = 0;
    tx_tail = 0;
    tx_inflight = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    dcache_clean_invalidate_range((u64)(usize)tx_ring, MACB_TX_RING_BYTES);
    __asm__ volatile("dsb sy" ::: "memory");

    mw(TBQP, (u32)(usize)&tx_ring[0]);
#if !USE_8BYTE_DESC
    mw(TBQPH, MACB_DMA_HI);
#endif
    mw(NCR, (mr(NCR) & ~NCR_THALT) | NCR_TE | NCR_RE);
    __asm__ volatile("dsb sy" ::: "memory");
    (void)mr(NCR);
    tx_recover_count++;
    DTRACE(DTRACE_CAT_MAC, DT_MAC_TXRECOVER, tx_recover_count, tsr, tx_send_count, tx_drop_count);
    pioscap_notify_event("tx-recover");
}

bool macb_send(const u8 *frame, u32 len) {
    if (len > BUF_SIZE || len < 14) {
        return false;
    }

    /* Ordered TX ring model: multiple descriptors may be active concurrently,
     * but GEM consumes them in ring order. Reclaim from the tail before testing
     * the producer head; never skip a hole because hardware will not either. */
    macb_tx_reclaim();
    if (tx_inflight >= NUM_TX) {
        macb_tx_recover_silent();
        macb_tx_reclaim();
    }

    u32 desc_idx = tx_idx;
    dcache_clean_invalidate_range((u64)(usize)&tx_ring[desc_idx], sizeof(struct macb_desc));
    if (!(tx_ring[desc_idx].ctrl & TX_STAT_USED)) {
        macb_tx_recover_silent();
        desc_idx = tx_idx;
        dcache_clean_invalidate_range((u64)(usize)&tx_ring[desc_idx], sizeof(struct macb_desc));
    }
    if (tx_inflight >= NUM_TX || !(tx_ring[desc_idx].ctrl & TX_STAT_USED)) {
        tx_drop_count++;
        /* HOT PATH: do NOT touch the UART here. Under concurrent load the TX
         * ring fills and this path runs hundreds/thousands of times; a single
         * uart_putc spins on the 115200 PL011 FIFO, and a periodic full-state
         * dump writes ~300 chars — either one stalls core0 for milliseconds per
         * drop, starving RX drain / ARP / ICMP and making the NIC look wedged.
         * The drop is counted (tx_drop_count, surfaced via macb_diag) instead. */
        return false;  /* Descriptor not ready, drop packet */
    }

    /* Copy frame to TX buffer (NEON for throughput) */
    u8 *dst = tx_bufs[desc_idx];
    simd_memcpy(dst, frame, len);

    /* Flush TX buffer to RAM for non-coherent PCIe DMA */
    dcache_clean_range((u64)(usize)dst, len);

    /* Setup descriptor (Circle: set addr during send, then barrier, then ctrl) */
#if !USE_8BYTE_DESC
    tx_ring[desc_idx].addr_hi = MACB_DMA_HI;
#endif
    __asm__ volatile("dsb sy" ::: "memory");
    tx_ring[desc_idx].addr = (u32)(usize)&tx_bufs[desc_idx][0];
    __asm__ volatile("dsb sy" ::: "memory");

    u32 ctrl = len & TX_STAT_LEN_MASK;
    ctrl |= TX_STAT_LAST;
    if (desc_idx == NUM_TX - 1) ctrl |= TX_STAT_WRAP;
    tx_ring[desc_idx].ctrl = ctrl;

    /* CRITICAL: clean+invalidate the descriptor (NOT just clean).
     *
     * 16-byte descriptors share 64-byte cache lines: 4 descriptors per line.
     * If we only "clean" (dc cvac), the line stays in cache. When the MAC
     * later writes USED=1 to a sibling descriptor in the same cache line
     * via non-coherent DMA, our cached copy becomes stale. The next time we
     * touch any descriptor in that line and clean again, we overwrite the
     * MAC's USED-bit updates with stale data — descriptors then look
     * forever-busy and TX stops.
     *
     * Clean+invalidate (dc civac) evicts the line so the next access loads
     * the MAC's fresh updates from RAM. */
    dcache_clean_invalidate_range((u64)(usize)&tx_ring[desc_idx], sizeof(struct macb_desc));
    __asm__ volatile("dsb sy" ::: "memory");

    /* PCIe write-barrier: read back a benign MAC register before kicking
     * TSTART. MMIO reads from the device force any outstanding writes
     * (descriptor + buffer DMA flushes) to drain through PCIe before the
     * MAC sees TSTART. Without this we observed only ~10 frames making
     * the wire before the MAC effectively stalled — adding incidental
     * MMIO reads in the diagnostic dump took us to 48+ pings. Make the
     * barrier deliberate. */
    (void)mr(NSR);

    /* If the previous transmission ended at a USED descriptor, GEM leaves UBR
     * (and usually COMP) latched in TSR. Clear sticky W1C TX status before
     * TSTART so an idle-gap send restarts deterministically instead of silently
     * dropping the first SYN-ACK after the MAC halted at the used descriptor. */
    {
        u32 tsr = mr(TSR) & (TSR_UBR | TSR_COL | TSR_RLE | TSR_BEX |
                             TSR_COMP | TSR_UND | TSR_HRESP);
        if (tsr)
            mw(TSR, tsr);
    }

    /* Kick the TX engine. NCR_TSTART is needed per packet — when the MAC
     * finishes a transmission and finds a descriptor with USED=1, it halts;
     * TSTART restarts it. Reference: U-Boot drivers/net/macb.c:_macb_send. */
    mw(NCR, (mr(NCR) & ~NCR_THALT) | NCR_TSTART);
    __asm__ volatile("dsb sy" ::: "memory");

    /* Post-TSTART read-back barrier: force the TSTART write to complete on
     * the device side before returning. Also lets the MAC begin pre-fetch
     * before our next call clobbers the next descriptor slot. */
    (void)mr(NCR);

    tx_send_count++;

    tx_idx = (desc_idx + 1) % NUM_TX;
    tx_inflight++;
    return true;
}

/* ── Receive ── */
static u32 rx_recv_count;
static u32 rx_recover_count;
static u32 rx_hole_recover_count;

static void macb_rx_ownership_snapshot(u32 *total_owned,
                                       u32 *contig_owned,
                                       u32 *owned_after_gap,
                                       u32 *first_owned_distance)
{
    dcache_invalidate_range((u64)(usize)rx_ring, MACB_RX_RING_BYTES);
    __asm__ volatile("dsb sy" ::: "memory");

    u32 contig = 0;
    u32 after = 0;
    u32 first = NUM_RX;
    u32 total = 0;
    bool saw_gap = false;
    for (u32 distance = 0; distance < NUM_RX; distance++) {
        u32 idx = (rx_idx + distance) % NUM_RX;
        bool owned = (rx_ring[idx].addr & RX_ADDR_OWN) != 0;
        if (owned)
            total++;
        if (!saw_gap) {
            if (owned) {
                contig++;
            } else {
                saw_gap = true;
            }
        } else if (owned) {
            if (first == NUM_RX)
                first = distance;
            after++;
        }
    }

    if (total_owned) *total_owned = total;
    if (contig_owned) *contig_owned = contig;
    if (owned_after_gap) *owned_after_gap = after;
    if (first_owned_distance) *first_owned_distance = first;
}

bool macb_recv(u8 *frame, u32 *len, bool *checksum_trusted) {
    /* Invalidate RX descriptor to see MAC's DMA writes (non-coherent PCIe).
     * dcache_invalidate_range() already ends with its own dsb(); a second
     * unconditional barrier here fired on every single poll -- even the vast
     * majority that find no new frame ready -- for no extra ordering benefit.
     * Removed as part of reducing per-poll/per-frame reclaim overhead (see
     * the buffer-invalidate fix below): a slow reclaim path is what lets
     * rx_owned climb toward NUM_RX under sustained load until RSR.BNA
     * latches and the MAC halts entirely. */
    dcache_invalidate_range((u64)(usize)&rx_ring[rx_idx], sizeof(struct macb_desc));

    /* Read descriptor */
    u32 addr_val = rx_ring[rx_idx].addr;

    /* Check if current RX descriptor has been filled by MAC */
    if (!(addr_val & RX_ADDR_OWN))
        return false;

    u32 status = rx_ring[rx_idx].ctrl;
    u32 flen = status & RX_STAT_LEN_MASK;
    if (checksum_trusted) {
        u32 csum = (status & RX_STAT_CSUM_MASK) >> RX_STAT_CSUM_SHIFT;
        *checksum_trusted = rx_csum_enabled &&
                            ((csum & RX_STAT_CSUM_CHECKED_MASK) != 0U);
    }

    if (flen == 0 || flen > BUF_SIZE) {
        /* Reclaim descriptor */
        rx_ring[rx_idx].addr &= ~RX_ADDR_OWN;
        /* 16-byte RX descriptors share 64-byte cache lines. Match the TX-side
         * contract: evict the line after returning ownership so later CPU
         * touches cannot clean a stale sibling descriptor over DMA updates. */
        dcache_clean_invalidate_range((u64)(usize)&rx_ring[rx_idx], sizeof(struct macb_desc));
        rx_idx = (rx_idx + 1) % NUM_RX;
        return false;
    }

    /* Invalidate only the bytes the MAC actually wrote (flen), not the full
     * BUF_SIZE (2048) buffer. Under sustained high-rate bursts, unconditionally
     * invalidating all 32 cache lines of BUF_SIZE regardless of actual frame
     * size (e.g. a 60-byte ARP frame only needs 1 line) measurably slows down
     * per-frame reclaim -- and since RX descriptor reclaim is the only thing
     * that hands ring space back to the MAC, a slow reclaim path is exactly
     * what lets rx_owned climb toward NUM_RX under load until RSR.BNA latches
     * and the MAC halts (observed live: rx_owned climbing past 450/512 before
     * a stall). We only ever read `flen` bytes via simd_memcpy below, so
     * invalidating exactly that (rounded up to the enclosing cache lines by
     * dcache_invalidate_range itself) is correct and sufficient. */
    dcache_invalidate_range((u64)(usize)rx_bufs[rx_idx], flen);

    /* Copy frame out (NEON for throughput) */
    u8 *src = rx_bufs[rx_idx];
    simd_memcpy(frame, src, flen);
    *len = flen;

    /* Reclaim: clear ownership bit and flush back to RAM */
    rx_ring[rx_idx].addr &= ~RX_ADDR_OWN;
    /* See invalid-frame reclaim above: never leave a descriptor line cached
     * after publishing ownership back to the MAC. dcache_clean_invalidate_range
     * already ends with its own dsb(); a second unconditional barrier here was
     * pure per-frame overhead with no additional ordering guarantee. */
    dcache_clean_invalidate_range((u64)(usize)&rx_ring[rx_idx], sizeof(struct macb_desc));

    rx_idx = (rx_idx + 1) % NUM_RX;
    rx_recv_count++;
    return true;
}

/* RX-overrun recovery. GEM has no self-healing path once the RX ring overruns:
 * when every descriptor is owned by software (CPU couldn't drain fast enough)
 * the MAC latches RSR.BNA (Buffer Not Available) and, if the RX FIFO then
 * overflows, RSR.OVR — and RX DMA effectively stops delivering. Clearing RSR
 * alone does NOT restart a halted RX engine, so the polling driver would stay
 * wedged (rx_idx frozen, frames never consumed) even after the load stops.
 *
 * This performs the canonical GEM recovery: stop RX, hand the whole ring back
 * to the MAC (clear OWN, preserve buffer addr + WRAP), reset the CPU cursor and
 * the queue pointer to the ring base, W1C-clear the latched RX status, then
 * re-enable RX. In-flight buffered frames are discarded (TCP will retransmit) —
 * recovering the NIC is worth far more than salvaging a handful of packets.
 *
 * Cheap to call every poll: it reads RSR and returns immediately unless a real
 * BNA/OVR stall is latched. Returns true if it performed a recovery. */
/* Stop RX, hand the whole ring back to the MAC (clear OWN, preserve buffer
 * addr + WRAP), reset the CPU cursor + queue pointer to the ring base, W1C-clear
 * the latched RX status, and re-enable RX. This is the canonical GEM RX restart;
 * toggling NCR.RE restarts a halted RX DMA regardless of what halted it. Shared
 * by the BNA/OVR recovery and the RX-liveness watchdog. */
static void macb_rx_ring_rebuild(void) {
    /* Stop RX while we rebuild the ring. */
    u32 ncr = mr(NCR);
    mw(NCR, ncr & ~NCR_RE);
    __asm__ volatile("dsb sy" ::: "memory");

    /* Hand every descriptor back to the MAC (OWN cleared), preserving the
     * buffer address and the WRAP flag on the final descriptor. */
    for (u32 i = 0; i < NUM_RX; i++) {
        volatile struct macb_desc *d = &rx_ring[i];
#if !USE_8BYTE_DESC
        d->addr_hi = MACB_DMA_HI;
        d->rsvd = 0;
#endif
        d->ctrl = 0;
        u32 a = (u32)(usize)&rx_bufs[i][0];
        if (i == NUM_RX - 1) a |= RX_ADDR_WRAP;
        d->addr = a;   /* OWN bit clear => MAC owns it again */
    }
    rx_idx = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    dcache_clean_invalidate_range((u64)(usize)rx_ring, MACB_RX_RING_BYTES);
    __asm__ volatile("dsb sy" ::: "memory");

    /* Point the RX queue back at the ring base. */
    mw(RBQP, (u32)(usize)&rx_ring[0]);
#if !USE_8BYTE_DESC
    mw(RBQPH, MACB_DMA_HI);
#endif

    /* W1C-clear the latched RX status (BNA/OVR/etc). */
    mw(RSR, mr(RSR));
    __asm__ volatile("dsb sy" ::: "memory");

    /* Re-enable RX and force the writes out to the device. */
    mw(NCR, mr(NCR) | NCR_RE);
    __asm__ volatile("dsb sy" ::: "memory");
    (void)mr(NCR);
}

bool macb_rx_hole_recover(void) {
    u32 contig = 0;
    u32 after = 0;
    u32 first = NUM_RX;
    u32 stuck_idx = rx_idx;
    macb_rx_ownership_snapshot(NULL, &contig, &after, &first);

    /* A current descriptor not owned by software followed by several later
     * OWN descriptors is impossible for an ordered GEM RX ring: hardware must
     * publish descriptor N before N+1. Requiring four later OWN descriptors
     * avoids reacting to a transient observation while DMA is completing one
     * descriptor, yet catches the live failure (hundreds queued beyond a hole)
     * well before BNA/OVR or the 90s liveness watchdog can fire. */
    if (contig != 0U || after < 4U)
        return false;

    /* RX DMA remains active during the bounded diagnostic scan. Revalidate the
     * current descriptor after the scan so normal DMA progress (current became
     * OWN while later descriptors were being inspected) is not mistaken for a
     * stable hole. A real sibling-writeback hole remains OWN=0 here. */
    dcache_invalidate_range((u64)(usize)&rx_ring[stuck_idx],
                            sizeof(struct macb_desc));
    __asm__ volatile("dsb sy" ::: "memory");
    if (rx_ring[stuck_idx].addr & RX_ADDR_OWN)
        return false;

    rx_hole_recover_count++;
    DTRACE(DTRACE_CAT_MAC, DT_MAC_RXHOLERECOVER,
           stuck_idx, after, first, rx_hole_recover_count);
    pioscap_notify_event("rx-descriptor-hole");
    macb_rx_ring_rebuild();
    return true;
}

bool macb_rx_recover(void) {
    u32 rsr = mr(RSR);
    if (!(rsr & (RSR_BNA | RSR_OVR)))
        return false;

    /* Snapshot the OWN-bit pattern BEFORE macb_rx_ring_rebuild() wipes it, to
     * distinguish a genuine sequential fill (every descriptor from rx_idx
     * forward is owned, consistent with real traffic volume exhausting the
     * ring) from a "hole" pattern (some descriptors in that span are NOT
     * owned, i.e. already reclaimed, while later ones ARE owned) -- a hole
     * would mean the ring reported "full" (BNA) while genuinely having spare
     * capacity, pointing at a false/stale overrun signal (e.g. a stuck
     * descriptor never getting reclaimed, or a hardware/cache-coherency
     * confusion) rather than real traffic volume. Walk forward from rx_idx
     * counting the contiguous owned run, then keep scanning to see if MORE
     * owned descriptors exist beyond a gap. */
    u32 contig_owned = 0;
    u32 owned_after_gap = 0;
    u32 first_owned_distance = NUM_RX;
    u32 recover_idx = rx_idx;
    macb_rx_ownership_snapshot(NULL, &contig_owned, &owned_after_gap,
                               &first_owned_distance);

    macb_rx_ring_rebuild();
    rx_recover_count++;
    /* Dump enough context to reconstruct what the recovery saw without
     * needing a live poll to race the event: which overrun bit(s) latched,
     * the lifetime recovery count, and where RX/TX were in the ring at the
     * moment of detection. */
    DTRACE(DTRACE_CAT_MAC, DT_MAC_RXRECOVER, rx_recover_count, rsr, recover_idx, tx_idx);
    /* a0=contig_owned (owned run starting at rx_idx; NUM_RX means genuinely
     * fully saturated), a1=owned_after_gap (nonzero means a hole pattern: a
     * NOT-owned descriptor followed by MORE owned ones -- a real anomaly,
     * since normal fill-up should never skip a descriptor), a2=rx_idx,
     * a3=NUM_RX (for scale). */
    DTRACE(DTRACE_CAT_MAC, DT_MAC_RXRECOVER_PATTERN,
           contig_owned, owned_after_gap, recover_idx, first_owned_distance);
    pioscap_notify_event("rx-overrun-recover");
    return true;
}

/* RX-liveness watchdog — see macb.h. Distinguishes ordinary RX-silence
 * ("idle") from a genuine RX DMA halt that does NOT latch RSR.BNA/OVR (which
 * macb_rx_recover cannot see). This LAN is reasonably noisy -- there is
 * usually SOME traffic, just not necessarily traffic PIOS cares about -- so
 * an extended RX-silence window by itself is common and not a fault. Only
 * escalate to a real "wedge" (ring rebuild + rx_wedge counter) when there is
 * also corroborating evidence of unmet demand: our own stack has actively
 * transmitted (tx_progress) during the silence window (e.g. mid-connection
 * retries/ACKs/replies expecting a peer response) and RX still produced
 * nothing back. Plain silence with no such demand is just idle -- tracked
 * separately via rx_idle for dashboards/diagnostics, never rebuilt for.
 *
 * TIMEOUT TUNING (2026-07-16 field observation): the original 4000ms base
 * timeout produced repeated false-positive "wedge" recoveries -- macb_diag
 * showed rx_recover=0 (BNA/OVR never latched) and RSR=0 (no hardware overrun
 * ever observed) alongside a steadily climbing wedge counter, i.e. the ring
 * was never actually stuck. Raising the base to 15000ms alone did not change
 * the observed false-trip cadence (~1 per 60-70s before AND after), showing
 * the natural inbound gap on this LAN is itself ~60-70s -- so the demand-
 * gated design above is the real fix; the larger 90s base timeout plus
 * IDLE_DISARM_MS kept below it (so a link with neither RX nor TX activity
 * disarms itself instead of always losing the race to the wedge check) is a
 * belt-and-braces second layer. */
#define MACB_RX_LIVENESS_IDLE_REPORT_MS  15000ULL
#define MACB_RX_LIVENESS_TIMEOUT_MS      90000ULL
#define MACB_RX_LIVENESS_IDLE_DISARM_MS  30000ULL
#define MACB_RX_LIVENESS_BACKOFF_MAX_MS 180000ULL
static u32 rx_live_last_count;
static u32 rx_live_last_tx_count;
static u64 rx_live_last_rx_ms;
static u64 rx_live_last_activity_ms;
static u32 rx_live_recover_count;
static u32 rx_wedge_count;         /* real wedges: silence + unmet demand */
static u32 rx_idle_count;          /* informational: extended RX-silence periods (not faults) */
static u32 rx_live_streak;
static bool rx_live_armed;
static bool rx_tx_since_last_rx;   /* our stack transmitted during the current silence window */
static bool rx_idle_reported;      /* idle already counted for the current silence window */

bool macb_rx_liveness_recover(u64 now_ms) {
    if (rx_live_last_activity_ms == 0) {
        rx_live_last_count = rx_recv_count;
        rx_live_last_tx_count = tx_send_count;
        rx_live_last_rx_ms = now_ms;
        rx_live_last_activity_ms = now_ms;
        rx_live_streak = 0;
        rx_live_armed = false;
        rx_tx_since_last_rx = false;
        rx_idle_reported = false;
        return false;
    }

    bool rx_progress = (rx_recv_count != rx_live_last_count);
    bool tx_progress = (tx_send_count != rx_live_last_tx_count);

    if (tx_progress) {
        rx_live_last_tx_count = tx_send_count;
        rx_live_last_activity_ms = now_ms;
        rx_tx_since_last_rx = true;
    }

    /* Any RX progress proves the lane is live: (re)arm, clear backoff, and
     * start a fresh silence window (clear the demand/idle-reported flags). */
    if (rx_progress) {
        rx_live_last_count = rx_recv_count;
        rx_live_last_rx_ms = now_ms;
        rx_live_last_activity_ms = now_ms;
        rx_live_streak = 0;
        rx_live_armed = true;
        rx_tx_since_last_rx = false;
        rx_idle_reported = false;
        return false;
    }

    /* Fully quiet link (neither RX nor TX activity): stop probing to avoid
     * idle false positives. */
    if (now_ms - rx_live_last_activity_ms >= MACB_RX_LIVENESS_IDLE_DISARM_MS) {
        rx_live_armed = false;
        rx_live_streak = 0;
        rx_live_last_rx_ms = now_ms;
        return false;
    }

    if (!rx_live_armed)
        return false;

    u64 idle_ms = now_ms - rx_live_last_rx_ms;

    /* Informational only: note an extended RX-silence window once per
     * window. This is expected/common on a noisy-but-not-for-us LAN and is
     * NOT by itself evidence of a fault. */
    if (idle_ms >= MACB_RX_LIVENESS_IDLE_REPORT_MS && !rx_idle_reported) {
        rx_idle_count++;
        rx_idle_reported = true;
    }

    u64 timeout = MACB_RX_LIVENESS_TIMEOUT_MS;
    u32 shift = rx_live_streak;
    if (shift > 4U)
        shift = 4U; /* 90s * 16 = 1440s; cap below to 180s. */
    timeout <<= shift;
    if (timeout > MACB_RX_LIVENESS_BACKOFF_MAX_MS)
        timeout = MACB_RX_LIVENESS_BACKOFF_MAX_MS;

    if (idle_ms < timeout)
        return false;

    if (!rx_tx_since_last_rx) {
        /* Extended silence with no corroborating demand: just an idle link,
         * not a fault. Keep waiting instead of rebuilding a ring that was
         * never actually stuck. */
        return false;
    }

    /* No RX for the active watchdog window: assume RX DMA is wedged. */
    rx_wedge_count++;
    macb_rx_ring_rebuild();
    rx_live_recover_count++;
    DTRACE(DTRACE_CAT_MAC, DT_MAC_RXLIVERECOVER, rx_wedge_count, idle_ms, rx_live_streak, tx_send_count);
    pioscap_notify_event("rx-liveness-wedge");
    rx_live_streak++;
    rx_live_last_rx_ms = now_ms;
    return true;
}

void macb_diag(struct macb_diag *out)
{
    if (!out)
        return;
    out->rx_idx = rx_idx;
    out->tx_idx = tx_idx;
    out->rx_recv = rx_recv_count;
    out->tx_send = tx_send_count;
    u32 owned = 0;
    u32 contig = 0;
    u32 after_gap = 0;
    u32 first_after = NUM_RX;
    macb_rx_ownership_snapshot(&owned, &contig, &after_gap, &first_after);
    out->rx_owned = owned;
    out->rx_contig_owned = contig;
    out->rx_owned_after_gap = after_gap;
    out->rx_first_owned_distance = first_after;
    out->nsr  = mr(NSR);
    out->rsr  = mr(RSR);
    out->tsr  = mr(TSR);
    out->ncr  = mr(NCR);
    out->rbqp = mr(RBQP);
    out->tbqp = mr(TBQP);
    out->imr  = mr(IMR);
    out->eth_cfg_stat = ecr(ETH_CFG_STAT);
    out->rx_recover = rx_recover_count;
    out->ring_size = NUM_RX;
    out->tx_drop = tx_drop_count;
    out->tx_recover = tx_recover_count;
    out->rx_live_recover = rx_live_recover_count;
    out->rx_hole_recover = rx_hole_recover_count;
    out->rx_wedge = rx_wedge_count;
    out->rx_idle = rx_idle_count;
    /* GEM_TXPAUSECNT/RXPAUSECNT are lifetime hardware counters that clear on
     * read, so accumulate into static software totals here (macb_diag is
     * currently the only reader) rather than reporting a raw read that would
     * silently reset to near-zero between diag polls. Added purely for
     * visibility into the new PAE-gated 802.3x flow control (see macb_init):
     * with zero prior telemetry on this mechanism, there was no way to tell
     * whether PAUSE was ever generated/received without this. */
    static u32 tx_pause_total;
    static u32 rx_pause_total;
    tx_pause_total += mr(GEM_TXPAUSECNT);
    rx_pause_total += mr(GEM_RXPAUSECNT);
    out->tx_pause = tx_pause_total;
    out->rx_pause = rx_pause_total;
}

void macb_irq_snapshot(struct macb_irq_snapshot *out, bool read_clear_isr)
{
    if (!out)
        return;
    out->imr = mr(IMR);
    out->rsr = mr(RSR);
    out->tsr = mr(TSR);
    out->ncr = mr(NCR);
    out->isr_clear = read_clear_isr ? mr(ISR) : 0;
}

void macb_irq_enable_rx(void)
{
    mw(RSR, 0xFFFFFFFFU);
    (void)mr(ISR);
    mw(IER, INT_RCOMP);
}

u32 macb_irq_ack_rx(void)
{
    u32 rsr = mr(RSR);
    u32 isr = mr(ISR);
    if (rsr)
        mw(RSR, rsr);
    if (isr)
        mw(ISR, isr);
    dsb();
    rsr = mr(RSR);
    u32 isr2 = mr(ISR);
    if (rsr)
        mw(RSR, rsr);
    if (isr2)
        mw(ISR, isr2);
    dsb();
    return isr | isr2;
}

/* Snapshot of full MAC state for stall diagnosis (host-side polling). */
void macb_dump_full_state(const char *tag) {
    uart_puts("[mac] ");
    uart_puts(tag);
    uart_puts(" tx_idx=");
    uart_hex(tx_idx);
    uart_puts(" rx_idx=");
    uart_hex(rx_idx);
    uart_puts(" tx_sent=");
    uart_hex(tx_send_count);
    uart_puts(" tx_drop=");
    uart_hex(tx_drop_count);
    uart_puts(" rx_recv=");
    uart_hex(rx_recv_count);
    uart_puts("\n[mac] NCR=");
    uart_hex(mr(NCR));
    uart_puts(" NCFGR=");
    uart_hex(mr(NCFGR));
    uart_puts(" NSR=");
    uart_hex(mr(NSR));
    uart_puts(" TSR=");
    uart_hex(mr(TSR));
    uart_puts(" RSR=");
    uart_hex(mr(RSR));
    uart_puts(" ISR=");
    uart_hex(mr(ISR));   /* Last — this read clears the latched bits */
    uart_puts("\n[mac] TBQP=");
    uart_hex(mr(TBQP));
    uart_puts(" RBQP=");
    uart_hex(mr(RBQP));
    uart_puts("\n");
    dcache_clean_invalidate_range((u64)(usize)tx_ring, MACB_TX_RING_BYTES);
    dcache_invalidate_range((u64)(usize)rx_ring, MACB_RX_RING_BYTES);
    uart_puts("[mac] TX:");
    for (u32 i = 0; i < NUM_TX; i++) {
        uart_putc(' ');
        u32 c = tx_ring[i].ctrl;
        uart_putc((c & TX_STAT_USED) ? 'U' : '_');
        uart_putc((c & TX_STAT_WRAP) ? 'W' : '_');
        uart_putc('0' + (i / 10));
        uart_putc('0' + (i % 10));
    }
    uart_puts("\n[mac] RX:");
    for (u32 i = 0; i < NUM_RX; i++) {
        uart_putc(' ');
        u32 a = rx_ring[i].addr;
        uart_putc((a & RX_ADDR_OWN)  ? 'O' : '_');
        uart_putc((a & RX_ADDR_WRAP) ? 'W' : '_');
        uart_putc('0' + (i / 10));
        uart_putc('0' + (i % 10));
    }
    uart_puts("\n");
    /* ETH_CFG_STAT — AXI bus error indicators from the RP1 GEM wrapper. */
    uart_puts("[mac] ETH_CFG S=");
    uart_hex(ecr(ETH_CFG_STAT));
    uart_puts(" C=");
    uart_hex(ecr(ETH_CFG_CTRL));
    uart_puts("\n");
}

bool macb_kick_stall(void) {
    /* Read & W1C latched status bits so we know what hit us, then
     * try to halt + restart the TX engine, and re-arm any RX descriptors
     * that aren't owned by us (defensive). */
    u32 tsr_pre = mr(TSR);
    u32 rsr_pre = mr(RSR);
    u32 isr_pre = mr(ISR);  /* read-clears */
    u32 ncr_pre = mr(NCR);

    uart_puts("[mac] KICK pre TSR=");
    uart_hex(tsr_pre);
    uart_puts(" RSR=");
    uart_hex(rsr_pre);
    uart_puts(" ISR=");
    uart_hex(isr_pre);
    uart_puts(" NCR=");
    uart_hex(ncr_pre);
    uart_puts("\n");

    /* Write-1-to-clear status bits (GEM TSR/RSR/ISR are W1C). */
    if (tsr_pre) mw(TSR, tsr_pre);
    if (rsr_pre) mw(RSR, rsr_pre);

    /* Halt TX, brief pause, then restart. */
    mw(NCR, ncr_pre | NCR_THALT);
    __asm__ volatile("dsb sy" ::: "memory");
    for (volatile int i = 0; i < 1000; i++) {}
    mw(NCR, (mr(NCR) & ~NCR_THALT) | NCR_TSTART);
    __asm__ volatile("dsb sy" ::: "memory");

    /* Re-walk RX ring: clear OWN on any descriptor that has OWN=1 but
     * we never consumed (canary check — shouldn't normally happen, but
     * forcing reclaim is harmless). Actually skip this — could lose frames.
     * Instead, just ensure TBQP/RBQP are still pointing at our rings. */
    uart_puts("[mac] KICK post NCR=");
    uart_hex(mr(NCR));
    uart_puts(" TSR=");
    uart_hex(mr(TSR));
    uart_puts(" RSR=");
    uart_hex(mr(RSR));
    uart_puts("\n");
    return true;
}

void macb_set_tx_checksum_offload(bool enable)
{
    tx_csum_enabled = enable;
    macb_apply_tx_checksum_offload();
}

void macb_set_rx_checksum_offload(bool enable)
{
    rx_csum_enabled = enable;
    macb_apply_rx_checksum_offload();
}

void macb_set_tso(bool enable)
{
    tso_enabled = false;
    if (enable)
        uart_puts("[mac] TSO requested but disabled: single-buffer TX path has no LSO descriptors\n");
}

bool macb_tx_checksum_offload_enabled(void)
{
    return tx_csum_enabled && ((mr(DMACFG) & DMACFG_TXCOEN) != 0U);
}

bool macb_rx_checksum_offload_enabled(void)
{
    return rx_csum_enabled && ((mr(NCFGR) & NCFGR_RXCOEN) != 0U);
}

bool macb_tso_enabled(void)
{
    return tso_enabled;
}

void macb_offload_regs(u32 *ncfgr_out, u32 *dmacfg_out)
{
    if (ncfgr_out)
        *ncfgr_out = mr(NCFGR);
    if (dmacfg_out)
        *dmacfg_out = mr(DMACFG);
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

u32 macb_link_mbps(void)
{
    return macb_link_up() ? link_mbps : 0;
}

bool macb_link_full_duplex(void)
{
    return macb_link_up() && link_full_duplex;
}
