/*
 * genet.c - Minimal GENET v5 Ethernet MAC driver for BCM2712 (Pi 5)
 * Single default queue (ring 16), polling mode, basic PHY autoneg.
 */

#include "genet.h"
#include "mmio.h"
#include "uart.h"
#include "simd.h"

/* ---- Register offsets from GENET_BASE ---- */

/* System */
#define SYS_REV_CTRL        0x0000
#define SYS_PORT_CTRL       0x0004
#define SYS_RBUF_FLUSH      0x0008
#define SYS_TBUF_FLUSH      0x000C

/* RBUF control */
#define RBUF_CTRL           0x0300
#define RBUF_64B_EN         0x0308
#define RBUF_CHK_CTRL       0x0314
#define RBUF_TBUF_SIZE_CTRL 0x03B4
#define RBUF_RXCHK_EN       (1 << 0)
#define RBUF_SKIP_FCS       (1 << 4)
#define RBUF_L3_PARSE_DIS   (1 << 5)

/* TBUF control */
#define TBUF_CTRL           0x0600
#define TBUF_64B_EN         (1 << 0)

/* UniMAC */
#define UMAC_CMD            0x0808
#define UMAC_MAC0           0x080C
#define UMAC_MAC1           0x0810
#define UMAC_MAX_FRAME      0x0814
#define UMAC_MDIO_CMD       0x0E14

/* UMAC_CMD bits */
#define CMD_TX_EN           (1 << 0)
#define CMD_RX_EN           (1 << 1)
#define CMD_SPEED_SHIFT     2
#define CMD_SPEED_10        (0 << CMD_SPEED_SHIFT)
#define CMD_SPEED_100       (1 << CMD_SPEED_SHIFT)
#define CMD_SPEED_1000      (2 << CMD_SPEED_SHIFT)
#define CMD_SW_RESET        (1 << 13)
#define CMD_CRC_FWD         (1 << 14)

/* DMA register bases */
#define RDMA_BASE           0x2000
#define TDMA_BASE           0x4000

/* Ring 16 (default queue) register base */
#define RDMA_RING16         0x2C00
#define TDMA_RING16         0x4C00

/* Per-ring register offsets */
#define DMA_WRITE_PTR       0x00
#define DMA_PROD_INDEX      0x04
#define DMA_CONS_INDEX      0x08
#define DMA_RING_BUF_SIZE   0x0C
#define DMA_START_ADDR      0x10
#define DMA_START_ADDR_HI   0x14
#define DMA_END_ADDR        0x18
#define DMA_END_ADDR_HI     0x1C
#define DMA_READ_PTR        0x2C

/* DMA global control */
#define RDMA_CTRL           (RDMA_BASE + 0xA00)
#define TDMA_CTRL           (TDMA_BASE + 0xA00)
#define DMA_EN              (1 << 0)

/* DMA ring enable bitmask registers */
#define RDMA_RING_CFG       (RDMA_BASE + 0xA04)
#define TDMA_RING_CFG       (TDMA_BASE + 0xA04)

/* Descriptor length_status bits */
#define DESC_OWN            (1 << 15)
#define DESC_EOP            (1 << 14)
#define DESC_SOP            (1 << 13)
#define DESC_WRAP           (1 << 12)
#define DESC_CRC            (1 << 6)
#define DESC_TX_DO_CSUM     (1 << 4)
#define DESC_LEN_SHIFT      16

/* MDIO */
#define MDIO_START_BUSY     (1 << 29)
#define MDIO_READ_OP        (2 << 26)
#define MDIO_WRITE_OP       (1 << 26)
#define MDIO_PMD_SHIFT      21
#define MDIO_REG_SHIFT      16

/* Standard MII PHY registers */
#define MII_BMCR            0x00
#define MII_BMSR            0x01
#define MII_ANAR            0x04
#define MII_GBCR            0x09

#define BMCR_RESET          (1 << 15)
#define BMCR_ANENABLE       (1 << 12)
#define BMCR_ANRESTART      (1 << 9)
#define BMSR_LSTATUS        (1 << 2)
#define BMSR_ANEGCOMPLETE   (1 << 5)

#define PHY_ADDR            1   /* Default PHY address on Pi 5 */

/* ---- Static data ---- */

#define NUM_DESC            64
#define BUF_SIZE            2048

/* v5 DMA descriptor: 12 bytes */
struct genet_desc {
    u32 length_status;
    u32 addr_lo;
    u32 addr_hi;
} PACKED;

static struct genet_desc rx_ring[NUM_DESC] ALIGNED(4096);
static struct genet_desc tx_ring[NUM_DESC] ALIGNED(4096);
static u8 rx_bufs[NUM_DESC][BUF_SIZE] ALIGNED(64);
static u8 tx_bufs[NUM_DESC][BUF_SIZE] ALIGNED(64);

static u32 rx_index;
static u32 tx_prod;
static u32 tx_cons;
static bool tx_csum_offload;
static bool rx_csum_offload;
static bool tso_enabled;
static bool tso_warned;

static u8 mac_addr[6] = { 0xDC, 0xA6, 0x32, 0x01, 0x02, 0x03 };

/* ---- Helpers ---- */

static inline void gw(u32 off, u32 val) { mmio_write(GENET_BASE + off, val); }
static inline u32  gr(u32 off)          { return mmio_read(GENET_BASE + off); }

static void genet_apply_offloads(void)
{
    u32 rchk = gr(RBUF_CHK_CTRL);
    if (rx_csum_offload) {
        rchk |= (RBUF_RXCHK_EN | RBUF_L3_PARSE_DIS);
        rchk &= ~RBUF_SKIP_FCS;
    } else {
        rchk &= ~(RBUF_RXCHK_EN | RBUF_L3_PARSE_DIS);
    }
    gw(RBUF_CHK_CTRL, rchk);

    /*
     * On GENET v3+, this register must be set for correct RX/TX status
     * sizing when checksum metadata is enabled.
     */
    gw(RBUF_TBUF_SIZE_CTRL, tx_csum_offload ? 1U : 0U);
}

/* ---- MDIO / PHY ---- */

static u16 mdio_read(u8 phy, u8 reg) {
    u32 cmd = MDIO_START_BUSY | MDIO_READ_OP |
              ((u32)phy << MDIO_PMD_SHIFT) |
              ((u32)reg << MDIO_REG_SHIFT);
    gw(UMAC_MDIO_CMD, cmd);
    u32 timeout = 100000;
    while ((gr(UMAC_MDIO_CMD) & MDIO_START_BUSY) && timeout--)
        delay_cycles(10);
    return (u16)(gr(UMAC_MDIO_CMD) & 0xFFFF);
}

static void mdio_write(u8 phy, u8 reg, u16 val) {
    u32 cmd = MDIO_START_BUSY | MDIO_WRITE_OP |
              ((u32)phy << MDIO_PMD_SHIFT) |
              ((u32)reg << MDIO_REG_SHIFT) |
              val;
    gw(UMAC_MDIO_CMD, cmd);
    u32 timeout = 100000;
    while ((gr(UMAC_MDIO_CMD) & MDIO_START_BUSY) && timeout--)
        delay_cycles(10);
}

static bool phy_init(void) {
    uart_puts("[genet] PHY reset...\n");

    /* Reset PHY */
    mdio_write(PHY_ADDR, MII_BMCR, BMCR_RESET);
    u32 timeout = 100000;
    while ((mdio_read(PHY_ADDR, MII_BMCR) & BMCR_RESET) && timeout--)
        delay_cycles(100);
    if (!timeout) return false;

    /* Advertise 10/100/1000 */
    mdio_write(PHY_ADDR, MII_ANAR, 0x01E1);   /* 10/100 FD+HD */
    mdio_write(PHY_ADDR, MII_GBCR, 0x0300);   /* 1000 FD+HD */

    /* Start auto-negotiation */
    mdio_write(PHY_ADDR, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

    /* Wait for link */
    uart_puts("[genet] Waiting for link...\n");
    timeout = 5000000;
    while (timeout--) {
        u16 bmsr = mdio_read(PHY_ADDR, MII_BMSR);
        if (bmsr & BMSR_LSTATUS) {
            uart_puts("[genet] Link UP\n");
            return true;
        }
        delay_cycles(1000);
    }

    uart_puts("[genet] Link timeout (continuing anyway)\n");
    return true;
}

/* ---- DMA ring setup ---- */

static void init_rx_ring(void) {
    for (u32 i = 0; i < NUM_DESC; i++) {
        rx_ring[i].addr_lo = (u32)(usize)&rx_bufs[i][0];
        rx_ring[i].addr_hi = (u32)((u64)(usize)&rx_bufs[i][0] >> 32);
        rx_ring[i].length_status = (BUF_SIZE << DESC_LEN_SHIFT) | DESC_OWN;
        if (i == NUM_DESC - 1)
            rx_ring[i].length_status |= DESC_WRAP;
    }
    rx_index = 0;

    u64 ring_addr = (u64)(usize)&rx_ring[0];
    gw(RDMA_RING16 + DMA_START_ADDR,    (u32)ring_addr);
    gw(RDMA_RING16 + DMA_START_ADDR_HI, (u32)(ring_addr >> 32));
    u64 ring_end = ring_addr + sizeof(rx_ring) - 1;
    gw(RDMA_RING16 + DMA_END_ADDR,      (u32)ring_end);
    gw(RDMA_RING16 + DMA_END_ADDR_HI,   (u32)(ring_end >> 32));
    gw(RDMA_RING16 + DMA_RING_BUF_SIZE, (NUM_DESC << 16) | BUF_SIZE);
    gw(RDMA_RING16 + DMA_WRITE_PTR,     0);
    gw(RDMA_RING16 + DMA_READ_PTR,      0);
    gw(RDMA_RING16 + DMA_PROD_INDEX,    0);
    gw(RDMA_RING16 + DMA_CONS_INDEX,    0);
}

static void init_tx_ring(void) {
    for (u32 i = 0; i < NUM_DESC; i++) {
        tx_ring[i].addr_lo = (u32)(usize)&tx_bufs[i][0];
        tx_ring[i].addr_hi = (u32)((u64)(usize)&tx_bufs[i][0] >> 32);
        tx_ring[i].length_status = 0;
        if (i == NUM_DESC - 1)
            tx_ring[i].length_status |= DESC_WRAP;
    }
    tx_prod = 0;
    tx_cons = 0;

    u64 ring_addr = (u64)(usize)&tx_ring[0];
    gw(TDMA_RING16 + DMA_START_ADDR,    (u32)ring_addr);
    gw(TDMA_RING16 + DMA_START_ADDR_HI, (u32)(ring_addr >> 32));
    u64 ring_end = ring_addr + sizeof(tx_ring) - 1;
    gw(TDMA_RING16 + DMA_END_ADDR,      (u32)ring_end);
    gw(TDMA_RING16 + DMA_END_ADDR_HI,   (u32)(ring_end >> 32));
    gw(TDMA_RING16 + DMA_RING_BUF_SIZE, (NUM_DESC << 16) | BUF_SIZE);
    gw(TDMA_RING16 + DMA_WRITE_PTR,     0);
    gw(TDMA_RING16 + DMA_READ_PTR,      0);
    gw(TDMA_RING16 + DMA_PROD_INDEX,    0);
    gw(TDMA_RING16 + DMA_CONS_INDEX,    0);
}

/* ---- Public API ---- */

bool genet_init(void) {
    uart_puts("[genet] Init GENET v5...\n");
    tx_csum_offload = false;
    rx_csum_offload = false;
    tso_enabled = false;
    tso_warned = false;

    /* Software reset */
    gw(SYS_RBUF_FLUSH, 1);
    delay_cycles(10000);
    gw(SYS_RBUF_FLUSH, 0);

    gw(UMAC_CMD, CMD_SW_RESET);
    delay_cycles(10000);
    gw(UMAC_CMD, 0);
    delay_cycles(10000);

    /* Set MAC address */
    gw(UMAC_MAC0, (mac_addr[0] << 24) | (mac_addr[1] << 16) |
                  (mac_addr[2] << 8)  |  mac_addr[3]);
    gw(UMAC_MAC1, (mac_addr[4] << 8)  |  mac_addr[5]);

    /* Max frame size */
    gw(UMAC_MAX_FRAME, ETH_FRAME_MAX);

    /* Disable 64-byte receive status block */
    gw(RBUF_CTRL, gr(RBUF_CTRL) & ~(1 << 0));
    gw(RBUF_64B_EN, 0);
    gw(TBUF_CTRL, gr(TBUF_CTRL) | TBUF_64B_EN);
    genet_apply_offloads();

    /* Init PHY */
    if (!phy_init())
        uart_puts("[genet] PHY init warning\n");

    /* Init DMA rings */
    init_rx_ring();
    init_tx_ring();

    /* Enable DMA */
    gw(RDMA_CTRL, DMA_EN);
    gw(TDMA_CTRL, DMA_EN);

    /* Enable ring 16 */
    gw(RDMA_RING_CFG, (1 << 16));
    gw(TDMA_RING_CFG, (1 << 16));

    /* Enable TX and RX, set speed to 1G */
    gw(UMAC_CMD, CMD_TX_EN | CMD_RX_EN | CMD_SPEED_1000);

    uart_puts("[genet] MAC ");
    for (int i = 0; i < 6; i++) {
        static const char hex[] = "0123456789ABCDEF";
        uart_putc(hex[mac_addr[i] >> 4]);
        uart_putc(hex[mac_addr[i] & 0xF]);
        if (i < 5) uart_putc(':');
    }
    uart_puts("\n");

    return true;
}

bool genet_send(const u8 *frame, u32 len) {
    if (len > ETH_FRAME_MAX || len < 14)
        return false;

    u32 idx = tx_prod % NUM_DESC;

    /* NEON copy frame to DMA buffer */
    prefetch_r(frame);
    prefetch_w(tx_bufs[idx]);
    simd_memcpy(tx_bufs[idx], frame, len);
    dsb();

    /* Set descriptor */
    tx_ring[idx].length_status =
        (len << DESC_LEN_SHIFT) | DESC_SOP | DESC_EOP | DESC_CRC;
    if (tx_csum_offload)
        tx_ring[idx].length_status |= DESC_TX_DO_CSUM;
    if (idx == NUM_DESC - 1)
        tx_ring[idx].length_status |= DESC_WRAP;
    dsb();

    /* Advance producer index to trigger DMA */
    tx_prod++;
    gw(TDMA_RING16 + DMA_PROD_INDEX, tx_prod & 0xFFFF);

    return true;
}

bool genet_recv(u8 *frame, u32 *len) {
    u32 idx = rx_index % NUM_DESC;
    u32 ls = rx_ring[idx].length_status;

    if (ls & DESC_OWN)
        return false;   /* DMA still owns this descriptor */

    u32 pkt_len = (ls >> DESC_LEN_SHIFT) & 0xFFFF;
    if (pkt_len > BUF_SIZE)
        pkt_len = BUF_SIZE;

    /* NEON copy from DMA buffer */
    prefetch_r(rx_bufs[idx]);
    simd_memcpy(frame, rx_bufs[idx], pkt_len);
    *len = pkt_len;

    /* Return descriptor to DMA */
    rx_ring[idx].length_status =
        (BUF_SIZE << DESC_LEN_SHIFT) | DESC_OWN;
    if (idx == NUM_DESC - 1)
        rx_ring[idx].length_status |= DESC_WRAP;
    dsb();

    rx_index++;
    gw(RDMA_RING16 + DMA_CONS_INDEX, rx_index & 0xFFFF);

    return true;
}

void genet_get_mac(u8 *mac) {
    memcpy(mac, mac_addr, 6);
}

bool genet_link_up(void) {
    return (mdio_read(PHY_ADDR, MII_BMSR) & BMSR_LSTATUS) != 0;
}

void genet_set_tx_checksum_offload(bool enable) {
    tx_csum_offload = enable;
    genet_apply_offloads();
}

void genet_set_rx_checksum_offload(bool enable) {
    rx_csum_offload = enable;
    genet_apply_offloads();
}

void genet_set_tso(bool enable) {
    /* Descriptor-level segmentation is not implemented yet in this driver. */
    tso_enabled = false;
    if (enable && !tso_warned) {
        uart_puts("[genet] TSO unsupported: keeping disabled\n");
        tso_warned = true;
    }
}

bool genet_tx_checksum_offload_enabled(void) {
    return tx_csum_offload;
}

bool genet_rx_checksum_offload_enabled(void) {
    return rx_csum_offload;
}

bool genet_tso_enabled(void) {
    return tso_enabled;
}
