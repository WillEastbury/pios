/*
 * xhci.c - xHCI Host Controller Interface for RP1 DWC3
 *
 * Pure HCI layer: DWC3 init, xHCI register management, ring operations,
 * port control, and transfer execution. No USB enumeration logic here.
 *
 * Reference:
 *   xHCI Spec 1.2 (Intel)
 *   Linux drivers/usb/dwc3/core.c, drivers/usb/host/xhci*.c
 *   rp1.dtsi: usb@200000 { compatible = "snps,dwc3"; dr_mode = "host"; }
 */

#include "xhci.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"
#include "mmu.h"
#include "rp1_gpio.h"
#include "pcie.h"
#include "fb.h"

/* USB VBUS power is controlled by RP1 GPIO 38 */
#define USB_VBUS_GPIO   38

/* ---- DWC3 Global Registers ---- */

#define DWC3_GLOBALS        0xC100
#define DWC3_GCTL           (DWC3_GLOBALS + 0x10)
#define DWC3_GSNPSID        (DWC3_GLOBALS + 0x20)
#define DWC3_GUSB2PHYCFG    (DWC3_GLOBALS + 0x100)

#define GCTL_PRTCAP_MASK    (3U << 12)
#define GCTL_PRTCAP_HOST    (1U << 12)
#define GCTL_DSBLCLKGTNG    (1U << 0)
#define GCTL_CORESOFTRESET  (1U << 11)
#define GUSB2_PHYSOFTRST    (1U << 31)

/* ---- xHCI Capability Registers ---- */

#define CAP_CAPLENGTH       0x00
#define CAP_HCSPARAMS1      0x04
#define CAP_HCSPARAMS2      0x08
#define CAP_HCCPARAMS1      0x10
#define CAP_DBOFF           0x14
#define CAP_RTSOFF          0x18

/* ---- xHCI Operational Registers (at op_base) ---- */

#define OP_USBCMD           0x00
#define OP_USBSTS           0x04
#define OP_CRCR_LO          0x18
#define OP_CRCR_HI          0x1C
#define OP_DCBAAP_LO        0x30
#define OP_DCBAAP_HI        0x34
#define OP_CONFIG            0x38

#define CMD_RUN             (1U << 0)
#define CMD_HCRST           (1U << 1)
#define CMD_INTE            (1U << 2)
#define STS_HCH             (1U << 0)
#define STS_CNR             (1U << 11)

/* ---- Port Registers (at op_base + 0x400 + port*0x10) ---- */

#define PORTSC_CCS          (1U << 0)
#define PORTSC_PED          (1U << 1)
#define PORTSC_PR           (1U << 4)
#define PORTSC_PP           (1U << 9)
#define PORTSC_SPEED_SHIFT  10
#define PORTSC_SPEED_MASK   (0xFU << 10)
#define PORTSC_CSC          (1U << 17)
#define PORTSC_PRC          (1U << 21)
#define PORTSC_RW1C_MASK    (PORTSC_CSC | PORTSC_PRC | (1U<<18) | (1U<<19) | \
                             (1U<<20) | (1U<<22) | (1U<<23))

/* ---- Runtime Registers (Interrupter 0) ---- */

#define IR0_IMAN            0x20
#define IR0_IMOD            0x24
#define IR0_ERSTSZ          0x28
#define IR0_ERSTBA_LO       0x30
#define IR0_ERSTBA_HI       0x34
#define IR0_ERDP_LO         0x38
#define IR0_ERDP_HI         0x3C

#define IMAN_IE             (1U << 1)
#define IMAN_IP             (1U << 0)

/* ---- TRB Definitions ---- */

#define RING_SIZE           64

#define TRB_TYPE(t)         ((u32)(t) << 10)
#define TRB_GET_TYPE(c)     (((c) >> 10) & 0x3F)

#define TRB_NORMAL          1
#define TRB_SETUP           2
#define TRB_DATA            3
#define TRB_STATUS          4
#define TRB_LINK            6
#define TRB_ENABLE_SLOT     9
#define TRB_ADDRESS_DEV     11
#define TRB_CONFIG_EP       12
#define TRB_STOP_EP         15
#define TRB_RESET_EP        14
#define TRB_CMD_COMPLETE    33
#define TRB_TRANSFER_EVT    32
#define TRB_PORT_STATUS     34

#define TRB_CYCLE           (1U << 0)
#define TRB_CHAIN           (1U << 4)
#define TRB_IOC             (1U << 5)
#define TRB_IDT             (1U << 6)
#define TRB_DIR_IN          (1U << 16)

#define TRB_COMP_CODE(s)    (((s) >> 24) & 0xFF)
#define CC_SUCCESS          1
#define CC_SHORT_PACKET     13
#define CC_STALL            6
#define CC_BABBLE           3
#define CC_USB_XACT_ERR     4
#define CC_TRB_ERR          5
#define CC_NO_SLOTS         9
#define CC_CMD_RING_STOP    24
#define CC_CMD_ABORTED      25
#define CC_STOPPED          26
#define CC_STOPPED_LEN      27

/* Max transfer length per TRB (xHCI spec: 17-bit field = 64KB-1) */
#define TRB_MAX_XFER_LEN   65536

/* ---- Data Structures ---- */

struct xhci_trb {
    u64 param;
    u32 status;
    u32 control;
} PACKED;

struct xhci_slot_ctx {
    u32 f0, f1, f2, f3;
    u32 rsvd[4];
} PACKED;

struct xhci_ep_ctx {
    u32 f0, f1;
    u64 deq;
    u32 f3;
    u32 rsvd[3];
} PACKED;

struct xhci_input_ctrl {
    u32 drop_flags, add_flags;
    u32 rsvd[6];
} PACKED;

struct xhci_erst_entry {
    u64 base;
    u32 size, rsvd;
} PACKED;

/* ---- Static Allocations ---- */

static u64 dcbaa[256] ALIGNED(64);
static struct xhci_trb cmd_ring[RING_SIZE] ALIGNED(64);
static struct xhci_trb evt_ring[RING_SIZE] ALIGNED(64);

#define NUM_EP_RINGS 16
static struct xhci_trb ep_rings[NUM_EP_RINGS][RING_SIZE] ALIGNED(64);
static u32 ep_enq[NUM_EP_RINGS];
static u32 ep_cyc[NUM_EP_RINGS];
static bool ep_used[NUM_EP_RINGS];

static struct xhci_erst_entry erst[1] ALIGNED(64);
static u8 dev_ctx_buf[2048] ALIGNED(64);
static u8 input_ctx_buf[2048] ALIGNED(64);
static u8 scratchpad_bufs[4][4096] ALIGNED(4096);
static u64 scratchpad_array[8] ALIGNED(64);

/* ---- Controller State ---- */

static u64 xhci_base, op_base, rt_base, db_base;
static u32 hci_max_ports, hci_max_slots, ctx_size;

static u32 cmd_enq, cmd_cycle;
static u32 evt_deq, evt_cycle;

/* ---- Instrumentation ---- */

static struct xhci_stats stats;

/*
 * RP1 DMA address translation: the xHCI controller lives inside RP1
 * which accesses system RAM via PCIe inbound translation.
 *
 * The firmware may have configured inbound windows. Try identity mapping
 * first (offset=0) since the firmware may map PCIe 0x00 → AXI 0x00.
 * If that fails, try 0x10_00000000 offset per rp1.dtsi dma-ranges.
 */
#define RP1_DMA_OFFSET  0x0ULL
static inline u64 dma_addr(const void *p) { return (u64)(usize)p + RP1_DMA_OFFSET; }

/* DCI → ep_rings index. 0xFF = unmapped. DCI 1 = EP0 always ring 0. */
static u8 dci_map[32];

/* ---- Register Access ---- */

static inline u32 xr(u64 off)              { return mmio_read(xhci_base + off); }
static inline void xw(u64 off, u32 v)      { mmio_write(xhci_base + off, v); }
static inline u32 opr(u32 off)             { return mmio_read(op_base + off); }
static inline void opw(u32 off, u32 v)     { mmio_write(op_base + off, v); }
static inline void rtw(u32 off, u32 v)     { mmio_write(rt_base + off, v); }
static inline void ring_db(u32 slot, u32 t) { mmio_write(db_base + slot * 4, t); }

/* ---- Ring Management ---- */

static void ring_init(struct xhci_trb *ring, u32 *enq, u32 *cycle) {
    for (u32 i = 0; i < RING_SIZE; i++) {
        ring[i].param = 0; ring[i].status = 0; ring[i].control = 0;
    }
    ring[RING_SIZE - 1].param = dma_addr(ring);
    ring[RING_SIZE - 1].control = TRB_TYPE(TRB_LINK) | (1U << 1) | TRB_CYCLE;
    *enq = 0; *cycle = 1;
}

static bool ring_enqueue(struct xhci_trb *ring, u32 *enq, u32 *cycle,
                         u64 param, u32 status, u32 control) {
    u32 idx = *enq;
    if (idx >= RING_SIZE - 1) {
        /* Should not happen: enqueue index reached the Link TRB slot */
        stats.ring_full++;
        uart_puts("[xhci] Ring full!\n");
        return false;
    }
    ring[idx].param = param;
    ring[idx].status = status;
    ring[idx].control = control | (*cycle ? TRB_CYCLE : 0);
    dmb();
    idx++;
    if (idx >= RING_SIZE - 1) {
        ring[RING_SIZE - 1].control ^= TRB_CYCLE;
        *cycle ^= 1;
        idx = 0;
    }
    *enq = idx;
    return true;
}

static i32 alloc_ep_ring(void) {
    for (u32 i = 0; i < NUM_EP_RINGS; i++) {
        if (!ep_used[i]) {
            ep_used[i] = true;
            ring_init(ep_rings[i], &ep_enq[i], &ep_cyc[i]);
            return (i32)i;
        }
    }
    return -1;
}

/* Release an EP ring back to the pool (used during device teardown). */
static void UNUSED free_ep_ring(u32 ri) {
    if (ri < NUM_EP_RINGS)
        ep_used[ri] = false;
}

/* ---- Event Ring ---- */

static bool evt_poll(struct xhci_trb *out, u32 timeout_ms) {
    for (u32 i = 0; i < timeout_ms * 100; i++) {
        dcache_invalidate_range((u64)(usize)&evt_ring[evt_deq], sizeof(struct xhci_trb));
        struct xhci_trb *trb = &evt_ring[evt_deq];
        dmb();
        if ((trb->control & TRB_CYCLE) == (evt_cycle ? 1U : 0U)) {
            *out = *trb;
            stats.evt_polled++;
            evt_deq++;
            if (evt_deq >= RING_SIZE) { evt_deq = 0; evt_cycle ^= 1; }
            u64 erdp = dma_addr(&evt_ring[evt_deq]);
            rtw(IR0_ERDP_LO, (u32)erdp | (1U << 3));
            rtw(IR0_ERDP_HI, (u32)(erdp >> 32));
            return true;
        }
        timer_delay_us(10);
    }
    return false;
}

/* Drain any stale events from the event ring (e.g. after controller start) */
static void evt_drain(void) {
    struct xhci_trb tmp;
    u32 drained = 0;
    while (evt_poll(&tmp, 1)) {
        drained++;
        stats.evt_stale_drained++;
        if (drained >= RING_SIZE) break;
    }
    if (drained)
        uart_puts("[xhci] Drained stale events\n");
}

/* ---- Command Submission ---- */

static bool cmd_submit(u64 param, u32 status, u32 control, struct xhci_trb *evt) {
    if (!ring_enqueue(cmd_ring, &cmd_enq, &cmd_cycle, param, status, control))
        return false;
    dcache_clean_range((u64)(usize)cmd_ring, sizeof(cmd_ring));
    dmb();
    ring_db(0, 0);
    stats.cmd_submitted++;

    /* Poll for Command Complete event, skipping non-command events
     * (e.g. Port Status Change events from recent port resets) */
    for (u32 attempts = 0; attempts < 10; attempts++) {
        if (!evt_poll(evt, 500)) {
            stats.cmd_timeout++;
            uart_puts("[xhci] Cmd timeout after ");
            uart_hex(attempts);
            uart_puts(" events\n");
            return false;
        }
        u32 evt_type = TRB_GET_TYPE(evt->control);
        u32 cc = TRB_COMP_CODE(evt->status);

        uart_puts("[xhci] Event: type=");
        uart_hex(evt_type);
        uart_puts(" cc=");
        uart_hex(cc);
        uart_puts("\n");

        if (evt_type == TRB_CMD_COMPLETE) {
            stats.cmd_completed++;
            fb_set_color(0x00FFAA00, 0x00000000);
            fb_printf("xHCI slot cc=%X\n", cc);
            return (cc == CC_SUCCESS);
        }
        /* Not our event — skip and try again */
        uart_puts("[xhci] Skipping non-command event, retrying...\n");
    }
    stats.cmd_timeout++;
    uart_puts("[xhci] Too many non-command events\n");
    return false;
}

/* ---- DWC3 Init ---- */

static bool dwc3_init(u64 base) {
    u32 snpsid = mmio_read(base + DWC3_GSNPSID);
    uart_puts("[xhci] DWC3 GSNPSID=");
    uart_hex(snpsid);
    uart_puts("\n");

    if (snpsid == 0 || snpsid == 0xFFFFFFFF) {
        uart_puts("[xhci] DWC3 not responding\n");
        return false;
    }

    /* Core soft reset */
    u32 gctl = mmio_read(base + DWC3_GCTL);
    uart_puts("[xhci] DWC3 GCTL before=");
    uart_hex(gctl);
    uart_puts("\n");
    gctl |= GCTL_CORESOFTRESET;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_us(100);

    /* PHY soft reset */
    u32 phycfg = mmio_read(base + DWC3_GUSB2PHYCFG);
    mmio_write(base + DWC3_GUSB2PHYCFG, phycfg | GUSB2_PHYSOFTRST);
    timer_delay_us(100);
    mmio_write(base + DWC3_GUSB2PHYCFG, phycfg & ~GUSB2_PHYSOFTRST);
    timer_delay_ms(10);

    /* Clear core reset, set host mode */
    gctl = mmio_read(base + DWC3_GCTL);
    gctl &= ~(GCTL_CORESOFTRESET | GCTL_PRTCAP_MASK | GCTL_DSBLCLKGTNG);
    gctl |= GCTL_PRTCAP_HOST;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_ms(10);

    uart_puts("[xhci] DWC3 GCTL after=");
    uart_hex(mmio_read(base + DWC3_GCTL));
    uart_puts("\n");
    return true;
}

/* ---- Public: Init ---- */

bool xhci_init(void) {
    uart_puts("[xhci] Init USB0 (DWC3)...\n");

    /* Reset instrumentation counters */
    memset(&stats, 0, sizeof(stats));

    /* Enable USB VBUS power via GPIO 38 */
    uart_puts("[xhci] Enabling VBUS (GPIO38)...\n");
    rp1_gpio_set_function(USB_VBUS_GPIO, 5);
    rp1_gpio_set_dir_output(USB_VBUS_GPIO);
    rp1_gpio_write(USB_VBUS_GPIO, true);
    timer_delay_ms(100);  /* let VBUS stabilise and devices power up */

    xhci_base = RP1_BAR_BASE + XHCI_USB0_OFFSET;

    if (!dwc3_init(xhci_base)) return false;

    u32 caplength = xr(CAP_CAPLENGTH) & 0xFF;
    u32 hcsparams1 = xr(CAP_HCSPARAMS1);
    u32 hcsparams2 = xr(CAP_HCSPARAMS2);
    u32 hccparams1 = xr(CAP_HCCPARAMS1);

    hci_max_slots = hcsparams1 & 0xFF;
    hci_max_ports = (hcsparams1 >> 24) & 0xFF;
    ctx_size = (hccparams1 & (1 << 2)) ? 64 : 32;

    /* Validate context size — must be 32 or 64 per xHCI spec */
    if (ctx_size != 32 && ctx_size != 64) {
        uart_puts("[xhci] Bad ctx_size=");
        uart_hex(ctx_size);
        uart_puts("\n");
        return false;
    }

    op_base = xhci_base + caplength;
    rt_base = xhci_base + xr(CAP_RTSOFF);
    db_base = xhci_base + xr(CAP_DBOFF);

    uart_puts("[xhci] CapLen=");
    uart_hex(caplength);
    uart_puts(" Ports=");
    uart_hex(hci_max_ports);
    uart_puts(" Slots=");
    uart_hex(hci_max_slots);
    uart_puts(" CtxSize=");
    uart_hex(ctx_size);
    uart_puts("\n");

    /* Halt */
    opw(OP_USBCMD, opr(OP_USBCMD) & ~CMD_RUN);
    for (u32 i = 0; i < 1000; i++) {
        if (opr(OP_USBSTS) & STS_HCH) break;
        timer_delay_us(100);
    }
    uart_puts("[xhci] Halted STS=");
    uart_hex(opr(OP_USBSTS));
    uart_puts("\n");

    /* Reset */
    opw(OP_USBCMD, CMD_HCRST);
    for (u32 i = 0; i < 1000; i++) {
        if (!(opr(OP_USBCMD) & CMD_HCRST) && !(opr(OP_USBSTS) & STS_CNR)) break;
        timer_delay_us(100);
    }
    if (opr(OP_USBCMD) & CMD_HCRST) {
        uart_puts("[xhci] Reset timeout CMD=");
        uart_hex(opr(OP_USBCMD));
        uart_puts(" STS=");
        uart_hex(opr(OP_USBSTS));
        uart_puts("\n");
        return false;
    }
    uart_puts("[xhci] Reset OK\n");

    /* Allow multiple device slots — cap to hw max, but at least 4 */
    {
        u32 slots_en = hci_max_slots;
        if (slots_en > 8) slots_en = 8;
        opw(OP_CONFIG, slots_en);
    }

    /* DCBAA + scratchpad */
    for (u32 i = 0; i < 256; i++) dcbaa[i] = 0;
    u32 sp_hi = (hcsparams2 >> 21) & 0x1F;
    u32 sp_lo = (hcsparams2 >> 27) & 0x1F;
    u32 num_sp = (sp_hi << 5) | sp_lo;
    if (num_sp > 0) {
        if (num_sp > 4) num_sp = 4;
        for (u32 i = 0; i < num_sp; i++)
            scratchpad_array[i] = dma_addr(&scratchpad_bufs[i][0]);
        dcbaa[0] = dma_addr(scratchpad_array);
    }
    u64 dcbaa_dma = dma_addr(dcbaa);
    opw(OP_DCBAAP_LO, (u32)dcbaa_dma);
    opw(OP_DCBAAP_HI, (u32)(dcbaa_dma >> 32));

    /* Command ring */
    ring_init(cmd_ring, &cmd_enq, &cmd_cycle);
    u64 cp = dma_addr(cmd_ring);
    opw(OP_CRCR_LO, (u32)cp | cmd_cycle);
    opw(OP_CRCR_HI, (u32)(cp >> 32));

    /* Event ring */
    for (u32 i = 0; i < RING_SIZE; i++) {
        evt_ring[i].param = 0; evt_ring[i].status = 0; evt_ring[i].control = 0;
    }
    evt_deq = 0; evt_cycle = 1;
    erst[0].base = dma_addr(evt_ring);
    erst[0].size = RING_SIZE;
    erst[0].rsvd = 0;
    rtw(IR0_ERSTSZ, 1);
    u64 ep = dma_addr(evt_ring);
    rtw(IR0_ERDP_LO, (u32)ep);
    rtw(IR0_ERDP_HI, (u32)(ep >> 32));
    u64 eb = dma_addr(erst);
    rtw(IR0_ERSTBA_LO, (u32)eb);
    rtw(IR0_ERSTBA_HI, (u32)(eb >> 32));

    /* Enable interrupter 0 — required for the controller to write events */
    rtw(IR0_IMOD, 0);          /* no interrupt moderation */
    rtw(IR0_IMAN, IMAN_IE);    /* enable event delivery */

    /* EP ring pool */
    for (u32 i = 0; i < NUM_EP_RINGS; i++) ep_used[i] = false;
    for (u32 i = 0; i < 32; i++) dci_map[i] = 0xFF;

    /* Flush all xHCI data structures to RAM before starting the controller */
    dcache_clean_range((u64)(usize)dcbaa, sizeof(dcbaa));
    dcache_clean_range((u64)(usize)cmd_ring, sizeof(cmd_ring));
    dcache_clean_range((u64)(usize)evt_ring, sizeof(evt_ring));
    dcache_clean_range((u64)(usize)erst, sizeof(erst));
    dcache_clean_range((u64)(usize)scratchpad_array, sizeof(scratchpad_array));
    for (u32 i = 0; i < 4; i++)
        dcache_clean_range((u64)(usize)scratchpad_bufs[i], sizeof(scratchpad_bufs[i]));

    /* Start */
    opw(OP_USBCMD, CMD_RUN | CMD_INTE);
    timer_delay_ms(10);
    if (opr(OP_USBSTS) & STS_HCH) {
        uart_puts("[xhci] Failed to start\n");
        return false;
    }

    uart_puts("[xhci] Controller running");
    /* Dump PCIe inbound BAR config (set by firmware) for DMA validation */
    uart_puts(" BAR2_LO=");
    uart_hex(mmio_read(PCIE_RC_BASE + 0x4034));
    uart_puts(" BAR2_HI=");
    uart_hex(mmio_read(PCIE_RC_BASE + 0x4038));
    uart_puts(" IMAN=");
    uart_hex(mmio_read(rt_base + IR0_IMAN));
    uart_puts("\n");

    /* DMA address sanity check: verify RP1 inbound BAR covers our DMA range.
     * The firmware sets BAR2 to cover system RAM. Log the config so we can
     * detect identity-vs-offset DMA mapping issues at boot. */
    {
        u32 bar2_lo = mmio_read(PCIE_RC_BASE + 0x4034);
        u32 bar2_hi = mmio_read(PCIE_RC_BASE + 0x4038);
        u32 remap_lo = mmio_read(PCIE_RC_BASE + 0x40B4);
        u32 remap_hi = mmio_read(PCIE_RC_BASE + 0x40B0);
        uart_puts("[xhci] DMA check: BAR2=");
        uart_hex(bar2_hi); uart_puts(":"); uart_hex(bar2_lo);
        uart_puts(" REMAP=");
        uart_hex(remap_hi); uart_puts(":"); uart_hex(remap_lo);
        uart_puts(" offset=");
        uart_hex((u32)(RP1_DMA_OFFSET >> 32)); uart_puts(":");
        uart_hex((u32)RP1_DMA_OFFSET);
        uart_puts("\n");

        /* Warn if DCBAA DMA address looks unusually high (above 4GB) and
         * the BAR window doesn't appear to cover it */
        u64 test_dma = dma_addr(dcbaa);
        if ((test_dma >> 32) != 0 && bar2_hi == 0) {
            uart_puts("[xhci] WARNING: DCBAA DMA addr above 4GB but BAR2_HI=0\n");
            uart_puts("[xhci] DMA may fail — check RP1_DMA_OFFSET\n");
        }
    }

    /* Drain any stale events from prior operation */
    evt_drain();

    return true;
}

/* ---- Public: Port Operations ---- */

u32 xhci_port_count(void) { return hci_max_ports; }

bool xhci_port_connected(u32 port) {
    if (port >= hci_max_ports) return false;
    return (mmio_read(op_base + 0x400 + (u64)port * 0x10) & PORTSC_CCS) != 0;
}

bool xhci_port_reset(u32 port, u32 *speed) {
    u64 pa = op_base + 0x400 + (u64)port * 0x10;
    u32 sc = mmio_read(pa);
    sc &= ~PORTSC_RW1C_MASK;
    sc |= PORTSC_PR;
    mmio_write(pa, sc);
    stats.port_resets++;

    for (u32 i = 0; i < 200; i++) {
        timer_delay_ms(5);
        sc = mmio_read(pa);
        if (sc & PORTSC_PRC) {
            sc &= ~PORTSC_RW1C_MASK;
            sc |= PORTSC_PRC;
            mmio_write(pa, sc);
            *speed = (sc & PORTSC_SPEED_MASK) >> PORTSC_SPEED_SHIFT;
            return true;
        }
    }
    return false;
}

/* ---- Public: Slot Management ---- */

bool xhci_enable_slot(u32 *slot) {
    struct xhci_trb evt;
    if (!cmd_submit(0, 0, TRB_TYPE(TRB_ENABLE_SLOT), &evt))
        return false;
    *slot = (evt.control >> 24) & 0xFF;
    return true;
}

bool xhci_address_device(u32 slot, u32 port, u32 speed, u32 max_packet) {
    u8 *input = input_ctx_buf;
    for (u32 i = 0; i < sizeof(input_ctx_buf); i++) input[i] = 0;

    struct xhci_input_ctrl *ic = (struct xhci_input_ctrl *)input;
    ic->add_flags = (1 << 0) | (1 << 1);

    struct xhci_slot_ctx *sl = (struct xhci_slot_ctx *)(input + ctx_size);
    sl->f0 = (speed << 20) | (1U << 27);
    sl->f1 = (port + 1) << 16;

    /* EP0: allocate ring 0 */
    i32 ri = alloc_ep_ring();
    if (ri < 0) return false;
    dci_map[1] = (u8)ri;

    struct xhci_ep_ctx *ep0 = (struct xhci_ep_ctx *)(input + ctx_size * 2);
    ep0->f1 = (3U << 1) | (4U << 3) | (max_packet << 16);
    ep0->deq = dma_addr(ep_rings[ri]) | ep_cyc[ri];
    ep0->f3 = 8;

    /* Set DCBAA[slot] before Address Device */
    for (u32 i = 0; i < sizeof(dev_ctx_buf); i++) dev_ctx_buf[i] = 0;
    dcbaa[slot] = dma_addr(dev_ctx_buf);
    dcache_clean_range((u64)(usize)dcbaa, sizeof(dcbaa));
    dcache_clean_range((u64)(usize)dev_ctx_buf, sizeof(dev_ctx_buf));
    dcache_clean_range((u64)(usize)input, sizeof(input_ctx_buf));
    dcache_clean_range((u64)(usize)ep_rings[ri], sizeof(ep_rings[ri]));

    struct xhci_trb evt;
    return cmd_submit(dma_addr(input), 0,
                      TRB_TYPE(TRB_ADDRESS_DEV) | (slot << 24), &evt);
}

bool xhci_configure_endpoints(u32 slot, u32 port, u32 speed,
                               const struct xhci_ep_info *eps, u32 count) {
    u8 *input = input_ctx_buf;
    for (u32 i = 0; i < sizeof(input_ctx_buf); i++) input[i] = 0;

    struct xhci_input_ctrl *ic = (struct xhci_input_ctrl *)input;
    ic->add_flags = (1 << 0); /* slot context */

    u32 max_dci = 1;

    for (u32 i = 0; i < count; i++) {
        u8 addr = eps[i].address;
        u32 ep_num = addr & 0x0F;
        u32 dir = (addr & 0x80) ? 1 : 0;
        u32 dci = ep_num * 2 + dir;

        if (dci > max_dci) max_dci = dci;
        ic->add_flags |= (1 << dci);

        i32 ri = alloc_ep_ring();
        if (ri < 0) return false;
        dci_map[dci] = (u8)ri;

        /* EP type in xHCI: bulk_out=2, bulk_in=6, intr_out=3, intr_in=7 */
        u32 attr = eps[i].attributes & 0x03;
        u32 ep_type;
        if (attr == 2)      ep_type = dir ? 6 : 2; /* bulk */
        else if (attr == 3) ep_type = dir ? 7 : 3; /* interrupt */
        else                ep_type = dir ? 5 : 1; /* isoc */

        struct xhci_ep_ctx *epc = (struct xhci_ep_ctx *)(input + ctx_size * (dci + 1));
        epc->f0 = ((u32)eps[i].interval << 16);
        epc->f1 = (3U << 1) | (ep_type << 3) | ((u32)eps[i].max_packet << 16);
        epc->deq = dma_addr(ep_rings[ri]) | ep_cyc[ri];
        epc->f3 = eps[i].max_packet;
    }

    struct xhci_slot_ctx *sl = (struct xhci_slot_ctx *)(input + ctx_size);
    sl->f0 = (speed << 20) | (max_dci << 27);
    sl->f1 = (port + 1) << 16;

    dcache_clean_range((u64)(usize)input, sizeof(input_ctx_buf));
    /* Flush newly initialised EP rings so the controller sees valid data */
    for (u32 i = 0; i < count; i++) {
        u8 addr = eps[i].address;
        u32 ep_num = addr & 0x0F;
        u32 dir = (addr & 0x80) ? 1 : 0;
        u32 dci = ep_num * 2 + dir;
        u8 ri = dci_map[dci];
        if (ri < NUM_EP_RINGS)
            dcache_clean_range((u64)(usize)ep_rings[ri], sizeof(ep_rings[ri]));
    }

    struct xhci_trb evt;
    return cmd_submit(dma_addr(input), 0,
                      TRB_TYPE(TRB_CONFIG_EP) | (slot << 24), &evt);
}

/* ---- Public: Transfers ---- */

bool xhci_control_transfer(u32 slot, u8 bmReq, u8 bReq, u16 wVal,
                            u16 wIdx, u16 wLen, void *data, u32 *actual) {
    u8 ri = dci_map[1]; /* DCI 1 = EP0 */
    if (ri >= NUM_EP_RINGS) return false;
    struct xhci_trb *ring = ep_rings[ri];
    u32 *enq = &ep_enq[ri], *cyc = &ep_cyc[ri];

    /* Setup Stage — TRT: 0=no data, 2=OUT data, 3=IN data */
    u64 setup = (u64)bmReq | ((u64)bReq << 8) | ((u64)wVal << 16) |
                ((u64)wIdx << 32) | ((u64)wLen << 48);
    u32 sc = TRB_TYPE(TRB_SETUP) | TRB_IDT;
    if (wLen > 0)
        sc |= (bmReq & 0x80) ? (3U << 16) : (2U << 16);
    if (!ring_enqueue(ring, enq, cyc, setup, 8, sc))
        return false;

    /* Data Stage */
    if (wLen > 0 && data) {
        u32 dc = TRB_TYPE(TRB_DATA);
        if (bmReq & 0x80) dc |= TRB_DIR_IN;
        dcache_clean_range((u64)(usize)data, wLen);
        if (!ring_enqueue(ring, enq, cyc, dma_addr(data), wLen, dc))
            return false;
    }

    /* Status Stage — direction is opposite to data stage; for no-data
     * transfers (wLen==0) the spec requires DIR=IN (xHCI §4.11.2.2). */
    u32 stc = TRB_TYPE(TRB_STATUS) | TRB_IOC;
    if (wLen == 0 || !(bmReq & 0x80)) stc |= TRB_DIR_IN;
    if (!ring_enqueue(ring, enq, cyc, 0, 0, stc))
        return false;

    dcache_clean_range((u64)(usize)ring, sizeof(struct xhci_trb) * RING_SIZE);
    dmb();
    ring_db(slot, 1);

    struct xhci_trb evt;
    if (!evt_poll(&evt, 1000)) { stats.xfer_fail++; return false; }
    u32 cc = TRB_COMP_CODE(evt.status);
    if (cc == CC_STALL) { stats.ep_stalls++; stats.xfer_fail++; return false; }
    if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) { stats.xfer_fail++; return false; }

    if (data && (bmReq & 0x80))
        dcache_invalidate_range((u64)(usize)data, wLen);
    if (actual)
        *actual = wLen - (evt.status & 0xFFFFFF);
    stats.xfer_ok++;
    return true;
}

bool xhci_bulk_transfer(u32 slot, u8 ep_addr, void *data, u32 len, u32 *actual) {
    u32 ep_num = ep_addr & 0x0F;
    u32 dir = (ep_addr & 0x80) ? 1 : 0;
    u32 dci = ep_num * 2 + dir;

    u8 ri = dci_map[dci];
    if (ri >= NUM_EP_RINGS) return false;
    struct xhci_trb *ring = ep_rings[ri];
    u32 *enq = &ep_enq[ri], *cyc = &ep_cyc[ri];

    if (dir == 0) /* OUT: clean cache before DMA */
        dcache_clean_range((u64)(usize)data, len);
    else /* IN: invalidate cache so we see DMA result */
        dcache_invalidate_range((u64)(usize)data, len);

    /* Segment into ≤TRB_MAX_XFER_LEN chunks with TD chaining (TRB_CHAIN).
     * Only the last TRB gets IOC so we get a single completion event. */
    u32 remaining = len;
    u8 *ptr = (u8 *)data;
    while (remaining > 0) {
        u32 chunk = remaining;
        if (chunk > TRB_MAX_XFER_LEN) chunk = TRB_MAX_XFER_LEN;
        remaining -= chunk;

        u32 flags = TRB_TYPE(TRB_NORMAL);
        if (remaining > 0)
            flags |= TRB_CHAIN;
        else
            flags |= TRB_IOC;

        if (!ring_enqueue(ring, enq, cyc, dma_addr(ptr), chunk, flags)) {
            stats.xfer_fail++;
            return false;
        }
        ptr += chunk;
    }

    dcache_clean_range((u64)(usize)ring, sizeof(struct xhci_trb) * RING_SIZE);
    dmb();
    ring_db(slot, dci);

    struct xhci_trb evt;
    if (!evt_poll(&evt, 2000)) { stats.xfer_fail++; return false; }
    u32 cc = TRB_COMP_CODE(evt.status);
    if (cc == CC_STALL) { stats.ep_stalls++; stats.xfer_fail++; return false; }
    if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) { stats.xfer_fail++; return false; }

    if (dir == 1) /* IN: invalidate again after DMA */
        dcache_invalidate_range((u64)(usize)data, len);
    if (actual)
        *actual = len - (evt.status & 0xFFFFFF);
    stats.xfer_ok++;
    return true;
}

/* ---- Public: Error Recovery ---- */

bool xhci_reset_endpoint(u32 slot, u32 dci) {
    stats.ep_resets++;
    struct xhci_trb evt;
    if (!cmd_submit(0, 0,
                    TRB_TYPE(TRB_RESET_EP) | (slot << 24) | (dci << 16),
                    &evt))
        return false;

    /* After reset, re-sync the software enqueue pointer: the controller
     * has moved its dequeue pointer, so we re-init the ring. */
    u8 ri = dci_map[dci];
    if (ri < NUM_EP_RINGS) {
        ring_init(ep_rings[ri], &ep_enq[ri], &ep_cyc[ri]);
        dcache_clean_range((u64)(usize)ep_rings[ri], sizeof(ep_rings[ri]));
    }
    return true;
}

bool xhci_stop_endpoint(u32 slot, u32 dci) {
    struct xhci_trb evt;
    return cmd_submit(0, 0,
                      TRB_TYPE(TRB_STOP_EP) | (slot << 24) | (dci << 16),
                      &evt);
}

/* ---- Public: Instrumentation ---- */

const struct xhci_stats *xhci_get_stats(void) { return &stats; }
