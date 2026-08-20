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
#include "macb_rx_engine.h"
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
#define DMACFG_RXEXT        (1U << 28)
#define DMACFG_TXEXT        (1U << 29)
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
 * it keeps the buffer pool and descriptor rings DMA-coherent with the MAC (NC,
 * no cache maintenance) AND lets the network-first RX ring grow without bloating
 * the sub-8MB kernel image. Layout within DMA_NET:
 *     [0 .. NUM_RX*BUF_SIZE)            RX buffers
 *     [NUM_RX*BUF_SIZE .. +NUM_TX*BUF_SIZE) TX buffers
 *     [aligned buffer end .. +RX descriptors) RX descriptor ring
 *     [aligned RX ring end .. +TX descriptors) TX descriptor ring
 *
 * The descriptor rings used to live in .bss. Live hardware later proved that
 * DMA_NET itself also had to be NC from the first MMU enable: a stopped RX
 * descriptor remained OWN=0 until dc ivac exposed GEM's OWN=1 publication.
 * Keep every MAC descriptor in the dedicated arena and preserve its continuous
 * NC mapping across boot/runtime tables. */
#define MACB_RX_POOL_OFF   0U
#define MACB_TX_POOL_OFF   ((u64)NUM_RX * BUF_SIZE)
#define MACB_DMA_POOL_BYTES (((u64)NUM_RX + (u64)NUM_TX) * BUF_SIZE)
#define MACB_RX_DESC_OFF   ((MACB_DMA_POOL_BYTES + 63ULL) & ~63ULL)
#define MACB_RX_RING_BYTES ((u64)NUM_RX * sizeof(struct macb_desc))
#define MACB_TX_DESC_OFF   ((MACB_RX_DESC_OFF + MACB_RX_RING_BYTES + 63ULL) & ~63ULL)
#define MACB_RX_TRAILING_BYTES (MACB_TX_DESC_OFF - (MACB_RX_DESC_OFF + MACB_RX_RING_BYTES))
#define MACB_TX_RING_BYTES ((u64)NUM_TX * sizeof(struct macb_desc))
#define MACB_DUMMY_DESC_OFF ((MACB_TX_DESC_OFF + MACB_TX_RING_BYTES + 63ULL) & ~63ULL)
#define MACB_DUMMY_DESC_BYTES 64ULL
#define MACB_DMA_TOTAL_BYTES (MACB_DUMMY_DESC_OFF + MACB_DUMMY_DESC_BYTES)
_Static_assert(MACB_DMA_TOTAL_BYTES <= DMA_NET_SIZE,
               "MACB buffers and descriptor rings exceed the DMA_NET arena");
static u8 (*const rx_bufs)[BUF_SIZE] =
    (u8 (*)[BUF_SIZE])(usize)(DMA_NET_BASE + MACB_RX_POOL_OFF);
static u8 (*const tx_bufs)[BUF_SIZE] =
    (u8 (*)[BUF_SIZE])(usize)(DMA_NET_BASE + MACB_TX_POOL_OFF);
static volatile struct macb_desc *const rx_ring =
    (volatile struct macb_desc *)(usize)(DMA_NET_BASE + MACB_RX_DESC_OFF);
static volatile struct macb_desc *const tx_ring =
    (volatile struct macb_desc *)(usize)(DMA_NET_BASE + MACB_TX_DESC_OFF);
static volatile struct macb_desc *const dummy_desc =
    (volatile struct macb_desc *)(usize)(DMA_NET_BASE + MACB_DUMMY_DESC_OFF);
static u64 rx_buffers_dma;
static u64 tx_buffers_dma;
static u64 rx_ring_dma;
static u64 tx_ring_dma;
static u64 dummy_desc_dma;

static bool macb_dma_layout_init(void)
{
    return rp1_pcie_dma_addr(&rx_bufs[0][0],
                              (u64)NUM_RX * BUF_SIZE, &rx_buffers_dma) &&
           rp1_pcie_dma_addr(&tx_bufs[0][0],
                              (u64)NUM_TX * BUF_SIZE, &tx_buffers_dma) &&
           rp1_pcie_dma_addr((const void *)rx_ring,
                              MACB_RX_RING_BYTES, &rx_ring_dma) &&
           rp1_pcie_dma_addr((const void *)tx_ring,
                              MACB_TX_RING_BYTES, &tx_ring_dma) &&
           rp1_pcie_dma_addr((const void *)dummy_desc,
                              MACB_DUMMY_DESC_BYTES, &dummy_desc_dma);
}
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

    if (!macb_dma_layout_init()) {
        uart_puts("[mac] DMA layout outside RP1 inbound window\n");
        return false;
    }

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
        /* We use the 64-bit, non-PTP four-word descriptor format. Firmware
         * state must not silently select six-word timestamp descriptors. */
        dmacfg &= ~(DMACFG_RXEXT | DMACFG_TXEXT);
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

    /* ── Setup RX descriptors ──
     * The RX ring is owned by the RX engine (src/macb_rx_engine.c). It
     * publishes the whole ring, programs RBQP/RBQPH, clears RSR and enables
     * NCR.RE at macb_rx_engine_init() time, once PHY/NCFGR are final. Nothing
     * to do here. */

    /* ── Setup TX descriptors ── */
    for (u32 i = 0; i < NUM_TX; i++) {
        volatile struct macb_desc *d = &tx_ring[i];
#if !USE_8BYTE_DESC
        d->addr_hi = (u32)(tx_buffers_dma >> 32);
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

    /* ── TX ring base pointer (the RX engine programs RBQP/RBQPH itself) ── */
    mw(TBQP, (u32)tx_ring_dma);
#if !USE_8BYTE_DESC
    mw(TBQPH, (u32)(tx_ring_dma >> 32));
#else
    mw(TBQPH, 0);
#endif

    /* (DMACFG already set above before ring pointers) */

    /* ── Multi-queue init (Circle: gmac_init_multi_queues) ── */
    {
        dummy_desc->ctrl = TX_STAT_USED;
        dummy_desc->addr = 0;
#if !USE_8BYTE_DESC
        dummy_desc->addr_hi = (u32)(dummy_desc_dma >> 32);
        dummy_desc->rsvd = 0;
#endif
        __asm__ volatile("dsb sy" ::: "memory");

        u32 dcfg6 = mr(0x0294);
        u32 queue_mask = (dcfg6 & 0xFF) | 0x01;
        u32 dummy_lo = (u32)dummy_desc_dma;
        for (u32 q = 1; q < 8; q++) {
            if (queue_mask & (1 << q)) {
                mw(0x0440 + ((q-1) << 2), dummy_lo);
                mw(0x0480 + ((q-1) << 2), dummy_lo);
#if !USE_8BYTE_DESC
                mw(0x04C8, (u32)(dummy_desc_dma >> 32));
                mw(0x04D4, (u32)(dummy_desc_dma >> 32));
#endif
            }
        }
        uart_puts("[mac] MQ mask=");
        uart_hex(queue_mask);
        uart_puts("\n");
    }

    /* USRIO: RGMII mode only (Circle: GEM_BIT(RGMII)) */
    mw(USRIO, USRIO_RGMII);

    /* Do PHY selection, link wait and the final NCFGR with RX and TX still
     * disabled. MDIO toggles NCR.MPE itself per op, so it works without RE/TE.
     * RX is enabled later by macb_rx_engine_init(); TX is enabled after it. */
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

    /* ── Bring up RX via the engine ──
     * PHY/NCFGR are final and RX/TX are still disabled. The engine publishes
     * the whole ring with reception off, programs RBQP/RBQPH, clears RSR, and
     * only then enables NCR.RE. */
    {
        struct macb_rx_config rxcfg = {
            .ring = (volatile void *)rx_ring,
            .buffers = &rx_bufs[0][0],
            .ring_count = NUM_RX,
            .buffer_size = BUF_SIZE,
            .ring_dma = rx_ring_dma,
            .buffers_dma = rx_buffers_dma,
            .trailing_bytes = (u32)MACB_RX_TRAILING_BYTES,
            .checksum_enabled = rx_csum_enabled,
        };
        if (!macb_rx_engine_init(&rxcfg)) {
            uart_puts("[mac] RX engine init failed\n");
            return false;
        }
    }

    /* Reception is now enabled by the engine; enable transmission while
     * preserving the NCR bits the engine set (RE). */
    mw(NCR, mr(NCR) | NCR_TE);
    __asm__ volatile("dsb sy" ::: "memory");
    (void)mr(NCR);

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
    __asm__ volatile("dsb sy" ::: "memory");
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
        __asm__ volatile("dsb sy" ::: "memory");
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
        d->addr_hi = (u32)(tx_buffers_dma >> 32);
        d->rsvd = 0;
#endif
        d->addr = 0;
        d->ctrl = TX_STAT_USED | ((i == NUM_TX - 1U) ? TX_STAT_WRAP : 0);
    }
    tx_idx = 0;
    tx_tail = 0;
    tx_inflight = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    __asm__ volatile("dsb sy" ::: "memory");

    mw(TBQP, (u32)tx_ring_dma);
#if !USE_8BYTE_DESC
    mw(TBQPH, (u32)(tx_ring_dma >> 32));
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
    __asm__ volatile("dsb sy" ::: "memory");
    if (!(tx_ring[desc_idx].ctrl & TX_STAT_USED)) {
        macb_tx_recover_silent();
        desc_idx = tx_idx;
        __asm__ volatile("dsb sy" ::: "memory");
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

    /* Setup descriptor (Circle: set addr during send, then barrier, then ctrl) */
#if !USE_8BYTE_DESC
    tx_ring[desc_idx].addr_hi = (u32)(tx_buffers_dma >> 32);
#endif
    __asm__ volatile("dsb sy" ::: "memory");
    tx_ring[desc_idx].addr =
        (u32)(tx_buffers_dma + (u64)desc_idx * BUF_SIZE);
    __asm__ volatile("dsb sy" ::: "memory");

    u32 ctrl = len & TX_STAT_LEN_MASK;
    ctrl |= TX_STAT_LAST;
    if (desc_idx == NUM_TX - 1) ctrl |= TX_STAT_WRAP;
    tx_ring[desc_idx].ctrl = ctrl;

    /* Descriptor and payload are Normal-NC; publish stores before TSTART. */
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

/* ── Receive ──
 * The RX ring, software cursor, OWN-bit publication/acquire ordering, ordered-
 * hole capture and every recovery path are owned by the RX engine
 * (src/macb_rx_engine.c). The wrappers below adapt the engine to the stable
 * public macb.h API without duplicating any ring state here. */

/* The public hole snapshot (macb.h) and the engine hole image capture the same
 * number of post-hole OWN descriptors; the converter below relies on it. */
_Static_assert(MACB_RX_CAPTURE_OWNED == MACB_HOLE_FOLLOW_COUNT,
               "engine/public hole follow-count mismatch");

bool macb_rx_hole_snapshot(struct macb_hole_snapshot *out)
{
    if (!out)
        return false;

    struct macb_rx_hole_image img;
    if (!macb_rx_engine_last_hole(&img) || !img.valid)
        return false;

    out->valid = img.valid ? 1U : 0U;
    out->sequence = img.sequence;
    out->stuck_idx = img.stuck_idx;
    out->rbqp = img.rbqp;
    out->rbqph = img.rbqph;
    out->rsr = img.rsr;
    out->ncr = img.ncr;
    out->expected_addr = img.expected_addr;
    out->rbqp_stopped = img.rbqp_stopped;
    out->dmacfg = img.dmacfg;
    out->dcfg10 = img.dcfg10;
    out->rxbdctrl = img.rxbdctrl;
    out->prefetch_descs = img.prefetch_descs;
    out->trailing_bytes = img.trailing_bytes;
    out->cache_line = img.cache_line;
    out->cache_probe_flags = img.cache_probe_flags;

    out->stuck.idx = img.stuck.index;
    out->stuck.addr = img.stuck.addr;
    out->stuck.ctrl = img.stuck.ctrl;
    out->stuck.addr_hi = img.stuck.addr_hi;
    out->stuck.word3 = img.stuck.word3;
    out->stopped.idx = img.stopped.index;
    out->stopped.addr = img.stopped.addr;
    out->stopped.ctrl = img.stopped.ctrl;
    out->stopped.addr_hi = img.stopped.addr_hi;
    out->stopped.word3 = img.stopped.word3;
    out->after_ivac.idx = img.after_ivac.index;
    out->after_ivac.addr = img.after_ivac.addr;
    out->after_ivac.ctrl = img.after_ivac.ctrl;
    out->after_ivac.addr_hi = img.after_ivac.addr_hi;
    out->after_ivac.word3 = img.after_ivac.word3;

    out->follow_count = img.later_owned_count;
    for (u32 i = 0; i < MACB_HOLE_FOLLOW_COUNT; i++) {
        out->follow[i].idx = img.later_owned[i].index;
        out->follow[i].addr = img.later_owned[i].addr;
        out->follow[i].ctrl = img.later_owned[i].ctrl;
        out->follow[i].addr_hi = img.later_owned[i].addr_hi;
        out->follow[i].word3 = img.later_owned[i].word3;
    }
    return true;
}

bool macb_recv(u8 *frame, u32 *len, bool *checksum_trusted) {
    return macb_rx_engine_recv(frame, BUF_SIZE, len, checksum_trusted);
}

bool macb_rx_hole_recover(void) {
    return macb_rx_engine_recover_ordering_hole();
}

bool macb_rx_recover(void) {
    return macb_rx_engine_recover_status();
}

/* RX-liveness watchdog — see macb.h. The engine owns the demand-gated
 * 90s silence detection, backoff and rebuild; pass the live TX progress
 * counter so it can tell an idle link from a genuine wedge. */
bool macb_rx_liveness_recover(u64 now_ms) {
    return macb_rx_engine_recover_liveness(now_ms, tx_send_count);
}

void macb_diag(struct macb_diag *out)
{
    if (!out)
        return;

    struct macb_rx_diag rd;
    macb_rx_engine_diag(&rd);
    out->rx_idx = rd.rx_idx;
    out->rx_recv = rd.rx_recv;
    out->rx_owned = rd.rx_owned;
    out->rx_contig_owned = rd.rx_contig_owned;
    out->rx_owned_after_gap = rd.rx_owned_after_gap;
    out->rx_first_owned_distance = rd.rx_first_owned_distance;
    out->rx_recover = rd.rx_recover;
    out->rx_live_recover = rd.rx_live_recover;
    out->rx_hole_recover = rd.rx_hole_recover;
    out->rx_wedge = rd.rx_wedge;
    out->rx_idle = rd.rx_idle;

    out->tx_idx = tx_idx;
    out->tx_send = tx_send_count;
    out->nsr  = mr(NSR);
    out->rsr  = mr(RSR);
    out->tsr  = mr(TSR);
    out->ncr  = mr(NCR);
    out->rbqp = mr(RBQP);
    out->tbqp = mr(TBQP);
    out->imr  = mr(IMR);
    out->eth_cfg_stat = ecr(ETH_CFG_STAT);
    out->ring_size = NUM_RX;
    out->tx_drop = tx_drop_count;
    out->tx_recover = tx_recover_count;
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
    /* The RX engine exclusively owns BNA/OVR/HRESP clearing + persistence.
     * Only W1C the frame-received bit here; leave fault bits latched for
     * macb_rx_engine_recover_status(). */
    mw(RSR, RSR_REC);
    (void)mr(ISR);
    mw(IER, INT_RCOMP);
}

u32 macb_irq_ack_rx(void)
{
    u32 rsr = mr(RSR);
    u32 isr = mr(ISR);
    /* W1C only RSR_REC — never BNA/OVR/HRESP (engine-owned). */
    if (rsr & RSR_REC)
        mw(RSR, RSR_REC);
    if (isr)
        mw(ISR, isr);
    dsb();
    rsr = mr(RSR);
    u32 isr2 = mr(ISR);
    if (rsr & RSR_REC)
        mw(RSR, RSR_REC);
    if (isr2)
        mw(ISR, isr2);
    dsb();
    return isr | isr2;
}

/* Snapshot of full MAC state for stall diagnosis (host-side polling). */
void macb_dump_full_state(const char *tag) {
    struct macb_rx_diag rd;
    macb_rx_engine_diag(&rd);
    uart_puts("[mac] ");
    uart_puts(tag);
    uart_puts(" tx_idx=");
    uart_hex(tx_idx);
    uart_puts(" rx_idx=");
    uart_hex(rd.rx_idx);
    uart_puts(" tx_sent=");
    uart_hex(tx_send_count);
    uart_puts(" tx_drop=");
    uart_hex(tx_drop_count);
    uart_puts(" rx_recv=");
    uart_hex(rd.rx_recv);
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
    __asm__ volatile("dsb sy" ::: "memory");
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

    /* Write-1-to-clear status bits (GEM TSR/RSR/ISR are W1C). The RX engine
     * exclusively owns BNA/OVR/HRESP fault clearing + persistence, so only
     * W1C the frame-received bit here and leave fault recovery to the engine. */
    if (tsr_pre) mw(TSR, tsr_pre);
    if (rsr_pre & RSR_REC) mw(RSR, RSR_REC);

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
    /* Keep the engine's recv() checksum-trust reporting consistent with the
     * runtime NCFGR.RXCOEN state it can no longer see directly. */
    macb_rx_engine_set_checksum_enabled(enable);
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
