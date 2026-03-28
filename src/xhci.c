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

#define IR0_ERSTSZ          0x28
#define IR0_ERSTBA_LO       0x30
#define IR0_ERSTBA_HI       0x34
#define IR0_ERDP_LO         0x38
#define IR0_ERDP_HI         0x3C

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
#define TRB_CMD_COMPLETE    33

#define TRB_CYCLE           (1U << 0)
#define TRB_IOC             (1U << 5)
#define TRB_IDT             (1U << 6)
#define TRB_DIR_IN          (1U << 16)

#define TRB_COMP_CODE(s)    (((s) >> 24) & 0xFF)
#define CC_SUCCESS          1
#define CC_SHORT_PACKET     13

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

#define NUM_EP_RINGS 5
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
    ring[RING_SIZE - 1].param = (u64)(usize)ring;
    ring[RING_SIZE - 1].control = TRB_TYPE(TRB_LINK) | (1U << 1) | TRB_CYCLE;
    *enq = 0; *cycle = 1;
}

static void ring_enqueue(struct xhci_trb *ring, u32 *enq, u32 *cycle,
                         u64 param, u32 status, u32 control) {
    u32 idx = *enq;
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

/* ---- Event Ring ---- */

static bool evt_poll(struct xhci_trb *out, u32 timeout_ms) {
    for (u32 i = 0; i < timeout_ms * 100; i++) {
        struct xhci_trb *trb = &evt_ring[evt_deq];
        dmb();
        if ((trb->control & TRB_CYCLE) == (evt_cycle ? 1U : 0U)) {
            *out = *trb;
            evt_deq++;
            if (evt_deq >= RING_SIZE) { evt_deq = 0; evt_cycle ^= 1; }
            u64 erdp = (u64)(usize)&evt_ring[evt_deq];
            rtw(IR0_ERDP_LO, (u32)erdp | (1U << 3));
            rtw(IR0_ERDP_HI, (u32)(erdp >> 32));
            return true;
        }
        timer_delay_us(10);
    }
    return false;
}

/* ---- Command Submission ---- */

static bool cmd_submit(u64 param, u32 status, u32 control, struct xhci_trb *evt) {
    ring_enqueue(cmd_ring, &cmd_enq, &cmd_cycle, param, status, control);
    dmb();
    ring_db(0, 0);
    if (!evt_poll(evt, 500)) {
        uart_puts("[xhci] Cmd timeout\n");
        return false;
    }
    if (TRB_GET_TYPE(evt->control) != TRB_CMD_COMPLETE)
        return false;
    u32 cc = TRB_COMP_CODE(evt->status);
    return (cc == CC_SUCCESS);
}

/* ---- DWC3 Init ---- */

static bool dwc3_init(u64 base) {
    uart_puts("[xhci] DWC3 ID: ");
    uart_hex(mmio_read(base + DWC3_GSNPSID));
    uart_puts("\n");

    u32 gctl = mmio_read(base + DWC3_GCTL);
    gctl |= GCTL_CORESOFTRESET;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_us(100);

    u32 phycfg = mmio_read(base + DWC3_GUSB2PHYCFG);
    mmio_write(base + DWC3_GUSB2PHYCFG, phycfg | GUSB2_PHYSOFTRST);
    timer_delay_us(100);
    mmio_write(base + DWC3_GUSB2PHYCFG, phycfg & ~GUSB2_PHYSOFTRST);
    timer_delay_ms(10);

    gctl = mmio_read(base + DWC3_GCTL);
    gctl &= ~(GCTL_CORESOFTRESET | GCTL_PRTCAP_MASK | GCTL_DSBLCLKGTNG);
    gctl |= GCTL_PRTCAP_HOST;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_ms(10);
    return true;
}

/* ---- Public: Init ---- */

bool xhci_init(void) {
    uart_puts("[xhci] Init USB0 (DWC3)...\n");
    xhci_base = RP1_BAR_BASE + XHCI_USB0_OFFSET;

    if (!dwc3_init(xhci_base)) return false;

    u32 caplength = xr(CAP_CAPLENGTH) & 0xFF;
    u32 hcsparams1 = xr(CAP_HCSPARAMS1);
    u32 hcsparams2 = xr(CAP_HCSPARAMS2);
    u32 hccparams1 = xr(CAP_HCCPARAMS1);

    hci_max_slots = hcsparams1 & 0xFF;
    hci_max_ports = (hcsparams1 >> 24) & 0xFF;
    ctx_size = (hccparams1 & (1 << 2)) ? 64 : 32;

    op_base = xhci_base + caplength;
    rt_base = xhci_base + xr(CAP_RTSOFF);
    db_base = xhci_base + xr(CAP_DBOFF);

    uart_puts("[xhci] Ports=");
    uart_hex(hci_max_ports);
    uart_puts(" CtxSize=");
    uart_hex(ctx_size);
    uart_puts("\n");

    /* Halt */
    opw(OP_USBCMD, opr(OP_USBCMD) & ~CMD_RUN);
    for (u32 i = 0; i < 1000; i++) {
        if (opr(OP_USBSTS) & STS_HCH) break;
        timer_delay_us(100);
    }

    /* Reset */
    opw(OP_USBCMD, CMD_HCRST);
    for (u32 i = 0; i < 1000; i++) {
        if (!(opr(OP_USBCMD) & CMD_HCRST) && !(opr(OP_USBSTS) & STS_CNR)) break;
        timer_delay_us(100);
    }
    if (opr(OP_USBCMD) & CMD_HCRST) {
        uart_puts("[xhci] Reset timeout\n");
        return false;
    }

    opw(OP_CONFIG, 1); /* MaxSlotsEn = 1 */

    /* DCBAA + scratchpad */
    for (u32 i = 0; i < 256; i++) dcbaa[i] = 0;
    u32 sp_hi = (hcsparams2 >> 21) & 0x1F;
    u32 sp_lo = (hcsparams2 >> 27) & 0x1F;
    u32 num_sp = (sp_hi << 5) | sp_lo;
    if (num_sp > 0) {
        if (num_sp > 4) num_sp = 4;
        for (u32 i = 0; i < num_sp; i++)
            scratchpad_array[i] = (u64)(usize)&scratchpad_bufs[i][0];
        dcbaa[0] = (u64)(usize)scratchpad_array;
    }
    opw(OP_DCBAAP_LO, (u32)(usize)dcbaa);
    opw(OP_DCBAAP_HI, (u32)((u64)(usize)dcbaa >> 32));

    /* Command ring */
    ring_init(cmd_ring, &cmd_enq, &cmd_cycle);
    u64 cp = (u64)(usize)cmd_ring;
    opw(OP_CRCR_LO, (u32)cp | cmd_cycle);
    opw(OP_CRCR_HI, (u32)(cp >> 32));

    /* Event ring */
    for (u32 i = 0; i < RING_SIZE; i++) {
        evt_ring[i].param = 0; evt_ring[i].status = 0; evt_ring[i].control = 0;
    }
    evt_deq = 0; evt_cycle = 1;
    erst[0].base = (u64)(usize)evt_ring;
    erst[0].size = RING_SIZE;
    erst[0].rsvd = 0;
    rtw(IR0_ERSTSZ, 1);
    u64 ep = (u64)(usize)evt_ring;
    rtw(IR0_ERDP_LO, (u32)ep);
    rtw(IR0_ERDP_HI, (u32)(ep >> 32));
    u64 eb = (u64)(usize)erst;
    rtw(IR0_ERSTBA_LO, (u32)eb);
    rtw(IR0_ERSTBA_HI, (u32)(eb >> 32));

    /* EP ring pool */
    for (u32 i = 0; i < NUM_EP_RINGS; i++) ep_used[i] = false;
    for (u32 i = 0; i < 32; i++) dci_map[i] = 0xFF;

    /* Start */
    opw(OP_USBCMD, CMD_RUN | CMD_INTE);
    timer_delay_ms(10);
    if (opr(OP_USBSTS) & STS_HCH) {
        uart_puts("[xhci] Failed to start\n");
        return false;
    }

    uart_puts("[xhci] Controller running\n");
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
    ep0->deq = (u64)(usize)ep_rings[ri] | ep_cyc[ri];
    ep0->f3 = 8;

    /* Set DCBAA[slot] before Address Device */
    for (u32 i = 0; i < sizeof(dev_ctx_buf); i++) dev_ctx_buf[i] = 0;
    dcbaa[slot] = (u64)(usize)dev_ctx_buf;
    dcache_clean_range((u64)(usize)dcbaa, sizeof(dcbaa));
    dcache_clean_range((u64)(usize)dev_ctx_buf, sizeof(dev_ctx_buf));
    dcache_clean_range((u64)(usize)input, sizeof(input_ctx_buf));
    dcache_clean_range((u64)(usize)ep_rings[ri], sizeof(ep_rings[ri]));

    struct xhci_trb evt;
    return cmd_submit((u64)(usize)input, 0,
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
        epc->deq = (u64)(usize)ep_rings[ri] | ep_cyc[ri];
        epc->f3 = eps[i].max_packet;
    }

    struct xhci_slot_ctx *sl = (struct xhci_slot_ctx *)(input + ctx_size);
    sl->f0 = (speed << 20) | (max_dci << 27);
    sl->f1 = (port + 1) << 16;

    dcache_clean_range((u64)(usize)input, sizeof(input_ctx_buf));

    struct xhci_trb evt;
    return cmd_submit((u64)(usize)input, 0,
                      TRB_TYPE(TRB_CONFIG_EP) | (slot << 24), &evt);
}

/* ---- Public: Transfers ---- */

bool xhci_control_transfer(u32 slot, u8 bmReq, u8 bReq, u16 wVal,
                            u16 wIdx, u16 wLen, void *data, u32 *actual) {
    u8 ri = dci_map[1]; /* DCI 1 = EP0 */
    if (ri >= NUM_EP_RINGS) return false;
    struct xhci_trb *ring = ep_rings[ri];
    u32 *enq = &ep_enq[ri], *cyc = &ep_cyc[ri];

    /* Setup Stage */
    u64 setup = (u64)bmReq | ((u64)bReq << 8) | ((u64)wVal << 16) |
                ((u64)wIdx << 32) | ((u64)wLen << 48);
    u32 sc = TRB_TYPE(TRB_SETUP) | TRB_IDT;
    if (wLen > 0)
        sc |= (bmReq & 0x80) ? (3U << 16) : (2U << 16);
    ring_enqueue(ring, enq, cyc, setup, 8, sc);

    /* Data Stage */
    if (wLen > 0 && data) {
        u32 dc = TRB_TYPE(TRB_DATA);
        if (bmReq & 0x80) dc |= TRB_DIR_IN;
        dcache_clean_range((u64)(usize)data, wLen);
        ring_enqueue(ring, enq, cyc, (u64)(usize)data, wLen, dc);
    }

    /* Status Stage */
    u32 stc = TRB_TYPE(TRB_STATUS) | TRB_IOC;
    if (wLen > 0 && !(bmReq & 0x80)) stc |= TRB_DIR_IN;
    ring_enqueue(ring, enq, cyc, 0, 0, stc);

    dmb();
    ring_db(slot, 1);

    struct xhci_trb evt;
    if (!evt_poll(&evt, 1000)) return false;
    u32 cc = TRB_COMP_CODE(evt.status);
    if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) return false;

    if (data && (bmReq & 0x80))
        dcache_invalidate_range((u64)(usize)data, wLen);
    if (actual)
        *actual = wLen - (evt.status & 0xFFFFFF);
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

    ring_enqueue(ring, enq, cyc,
                 (u64)(usize)data, len,
                 TRB_TYPE(TRB_NORMAL) | TRB_IOC);
    dmb();
    ring_db(slot, dci);

    struct xhci_trb evt;
    if (!evt_poll(&evt, 2000)) return false;
    u32 cc = TRB_COMP_CODE(evt.status);
    if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) return false;

    if (dir == 1) /* IN: invalidate again after DMA */
        dcache_invalidate_range((u64)(usize)data, len);
    if (actual)
        *actual = len - (evt.status & 0xFFFFFF);
    return true;
}
