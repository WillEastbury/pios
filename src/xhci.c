/*
 * xhci.c - xHCI USB Host Controller driver for RP1 DWC3
 *
 * Minimal single-device xHCI supporting USB keyboard input.
 *
 * Architecture:
 *   DWC3 wrapper (host mode) → xHCI init → port reset → enumerate → poll
 *
 * Memory: all DMA structures are statically allocated in BSS (identity-mapped).
 * Only slot 1 is used (one device). Only EP0 + one Interrupt IN endpoint.
 *
 * Reference:
 *   xHCI Spec 1.2 (Intel)
 *   Linux drivers/usb/dwc3/core.c, drivers/usb/host/xhci*.c
 *   OSDev wiki: eXtensible_Host_Controller_Interface
 *   rp1.dtsi: usb@200000 { compatible = "snps,dwc3"; dr_mode = "host"; }
 */

#include "xhci.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"
#include "mmu.h"

/* ================================================================
 * DWC3 Global Registers (at DWC3 base + 0xC100)
 * ================================================================ */

#define DWC3_GLOBALS        0xC100
#define DWC3_GSBUSCFG0      (DWC3_GLOBALS + 0x00)
#define DWC3_GCTL           (DWC3_GLOBALS + 0x10)
#define DWC3_GSNPSID        (DWC3_GLOBALS + 0x20)
#define DWC3_GUSB2PHYCFG    (DWC3_GLOBALS + 0x100)
#define DWC3_GUSB3PIPECTL   (DWC3_GLOBALS + 0x1C0)

/* GCTL fields */
#define GCTL_PRTCAP_MASK    (3U << 12)
#define GCTL_PRTCAP_HOST    (1U << 12)
#define GCTL_DSBLCLKGTNG    (1U << 0)
#define GCTL_CORESOFTRESET  (1U << 11)

/* GUSB2PHYCFG fields */
#define GUSB2_PHYSOFTRST    (1U << 31)
#define GUSB2_ULPI_UTMI     (1U << 4)
#define GUSB2_PHYIF16       (1U << 3)

/* ================================================================
 * xHCI Capability Register Offsets (from xHCI base = DWC3 base)
 * ================================================================ */

#define CAP_CAPLENGTH       0x00    /* u8: capability length */
#define CAP_HCIVERSION      0x02    /* u16 */
#define CAP_HCSPARAMS1      0x04
#define CAP_HCSPARAMS2      0x08
#define CAP_HCSPARAMS3      0x0C
#define CAP_HCCPARAMS1      0x10
#define CAP_DBOFF           0x14
#define CAP_RTSOFF          0x18

/* ================================================================
 * xHCI Operational Register Offsets (from op_base = xhci + caplength)
 * ================================================================ */

#define OP_USBCMD           0x00
#define OP_USBSTS           0x04
#define OP_PAGESIZE         0x08
#define OP_DNCTRL           0x14
#define OP_CRCR_LO          0x18
#define OP_CRCR_HI          0x1C
#define OP_DCBAAP_LO        0x30
#define OP_DCBAAP_HI        0x34
#define OP_CONFIG            0x38

/* USBCMD bits */
#define CMD_RUN             (1U << 0)
#define CMD_HCRST           (1U << 1)
#define CMD_INTE            (1U << 2)

/* USBSTS bits */
#define STS_HCH             (1U << 0)   /* halted */
#define STS_CNR             (1U << 11)  /* controller not ready */

/* Port registers at op_base + 0x400 + port*0x10 */
#define PORT_REGS_OFFSET    0x400
#define PORT_SC             0x00

/* PORTSC bits */
#define PORTSC_CCS          (1U << 0)   /* current connect status */
#define PORTSC_PED          (1U << 1)   /* port enabled */
#define PORTSC_PR           (1U << 4)   /* port reset */
#define PORTSC_PLS_MASK     (0xFU << 5)
#define PORTSC_PP           (1U << 9)   /* port power */
#define PORTSC_SPEED_MASK   (0xFU << 10)
#define PORTSC_SPEED_SHIFT  10
#define PORTSC_CSC          (1U << 17)  /* connect status change */
#define PORTSC_PRC          (1U << 21)  /* port reset change */
/* Write-1-to-clear bits we must preserve when doing RMW */
#define PORTSC_RW1C_MASK    (PORTSC_CSC | PORTSC_PRC | (1U<<18) | (1U<<19) | \
                             (1U<<20) | (1U<<22) | (1U<<23))
#define PORTSC_PRESERVE     (PORTSC_PP | PORTSC_CCS | PORTSC_PED)

/* Runtime register offsets (from rt_base = xhci + rtsoff) */
/* Interrupter 0 at rt_base + 0x20 */
#define IR0_IMAN            0x20
#define IR0_IMOD            0x24
#define IR0_ERSTSZ          0x28
#define IR0_ERSTBA_LO       0x30
#define IR0_ERSTBA_HI       0x34
#define IR0_ERDP_LO         0x38
#define IR0_ERDP_HI         0x3C

/* ================================================================
 * TRB Definitions
 * ================================================================ */

#define TRB_SIZE            16
#define RING_SIZE           64  /* TRBs per ring */

/* TRB type field: bits [15:10] of control dword */
#define TRB_TYPE(t)         ((u32)(t) << 10)
#define TRB_GET_TYPE(c)     (((c) >> 10) & 0x3F)

/* TRB types */
#define TRB_NORMAL          1
#define TRB_SETUP           2
#define TRB_DATA            3
#define TRB_STATUS          4
#define TRB_LINK            6
#define TRB_ENABLE_SLOT     9
#define TRB_DISABLE_SLOT    10
#define TRB_ADDRESS_DEV     11
#define TRB_CONFIG_EP       12
#define TRB_NOOP_CMD        23
#define TRB_TRANSFER_EVENT  32
#define TRB_CMD_COMPLETE    33
#define TRB_PORT_STATUS     34

/* TRB control flags */
#define TRB_CYCLE           (1U << 0)
#define TRB_IOC             (1U << 5)   /* interrupt on completion */
#define TRB_IDT             (1U << 6)   /* immediate data */
#define TRB_BSR             (1U << 9)   /* block set address request */
#define TRB_DIR_IN          (1U << 16)  /* data stage direction: IN */

/* Completion codes (from status field bits [31:24]) */
#define TRB_COMP_CODE(s)    (((s) >> 24) & 0xFF)
#define CC_SUCCESS          1
#define CC_SHORT_PACKET     13

/* ================================================================
 * USB Standard Definitions
 * ================================================================ */

#define USB_DIR_IN          0x80
#define USB_DIR_OUT         0x00

#define USB_REQ_GET_DESC    0x06
#define USB_REQ_SET_CONFIG  0x09

#define USB_DESC_DEVICE     1
#define USB_DESC_CONFIG     2

#define USB_CLASS_HID       3

/* USB speed values from PORTSC */
#define USB_SPEED_FULL      1
#define USB_SPEED_LOW       2
#define USB_SPEED_HIGH      3
#define USB_SPEED_SUPER     4

/* ================================================================
 * Data Structures (all 32-byte contexts, CSZ=0)
 * ================================================================ */

struct xhci_trb {
    u64 param;
    u32 status;
    u32 control;
} PACKED;

/* Slot Context (32 bytes) */
struct xhci_slot_ctx {
    u32 field0;     /* route_string[19:0], speed[23:20], MTT[25], hub[26], ctx_entries[31:27] */
    u32 field1;     /* max_exit_latency[15:0], root_hub_port[23:16], num_ports[31:24] */
    u32 field2;     /* TT fields */
    u32 field3;     /* slot_state[31:27], device_address[7:0] */
    u32 rsvd[4];
} PACKED;

/* Endpoint Context (32 bytes) */
struct xhci_ep_ctx {
    u32 field0;     /* ep_state[2:0], mult[9:8], max_pstreams[14:10], interval[23:16], max_esit_hi[31:24] */
    u32 field1;     /* cerr[2:1], ep_type[5:3], max_burst[15:8], max_packet[31:16] */
    u64 deq;        /* TR dequeue pointer[63:4], DCS[0] */
    u32 field3;     /* avg_trb_len[15:0], max_esit_lo[31:16] */
    u32 rsvd[3];
} PACKED;

/* Input Control Context (32 bytes) */
struct xhci_input_ctrl {
    u32 drop_flags;
    u32 add_flags;
    u32 rsvd[6];
} PACKED;

/* Event Ring Segment Table Entry */
struct xhci_erst_entry {
    u64 base;
    u32 size;
    u32 rsvd;
} PACKED;

/* USB Device Descriptor */
struct usb_dev_desc {
    u8  bLength;
    u8  bDescriptorType;
    u16 bcdUSB;
    u8  bDeviceClass;
    u8  bDeviceSubClass;
    u8  bDeviceProtocol;
    u8  bMaxPacketSize0;
    u16 idVendor;
    u16 idProduct;
    u16 bcdDevice;
    u8  iManufacturer;
    u8  iProduct;
    u8  iSerialNumber;
    u8  bNumConfigurations;
} PACKED;

/* USB Configuration Descriptor (partial, followed by iface + ep descs) */
struct usb_cfg_desc {
    u8  bLength;
    u8  bDescriptorType;
    u16 wTotalLength;
    u8  bNumInterfaces;
    u8  bConfigurationValue;
    u8  iConfiguration;
    u8  bmAttributes;
    u8  bMaxPower;
} PACKED;

struct usb_iface_desc {
    u8  bLength;
    u8  bDescriptorType;
    u8  bInterfaceNumber;
    u8  bAlternateSetting;
    u8  bNumEndpoints;
    u8  bInterfaceClass;
    u8  bInterfaceSubClass;
    u8  bInterfaceProtocol;
    u8  iInterface;
} PACKED;

struct usb_ep_desc {
    u8  bLength;
    u8  bDescriptorType;
    u8  bEndpointAddress;
    u8  bmAttributes;
    u16 wMaxPacketSize;
    u8  bInterval;
} PACKED;

/* ================================================================
 * Static Allocations (identity-mapped, physically contiguous)
 * ================================================================ */

static u64 dcbaa[256] ALIGNED(64);                             /* Device Context Base Address Array */
static struct xhci_trb cmd_ring[RING_SIZE] ALIGNED(64);        /* Command Ring */
static struct xhci_trb evt_ring[RING_SIZE] ALIGNED(64);        /* Event Ring */
static struct xhci_trb ep0_ring[RING_SIZE] ALIGNED(64);        /* EP0 Transfer Ring */
static struct xhci_trb int_ring[RING_SIZE] ALIGNED(64);        /* Interrupt IN Transfer Ring */
static struct xhci_erst_entry erst[1] ALIGNED(64);             /* Event Ring Segment Table */
static u8 dev_ctx_buf[1024] ALIGNED(64);                       /* Device Context (slot + 31 EPs) */
static u8 input_ctx_buf[1088] ALIGNED(64);                     /* Input Context (ctrl + slot + 31 EPs) */
static u8 scratchpad_bufs[4][4096] ALIGNED(4096);              /* Up to 4 scratchpad pages */
static u64 scratchpad_array[8] ALIGNED(64);                    /* Scratchpad buffer array */
static u8 usb_buf[512] ALIGNED(64);                            /* USB data buffer */

/* ================================================================
 * Controller State
 * ================================================================ */

static u64 xhci_base;      /* xHCI register base (= DWC3 base) */
static u64 op_base;         /* Operational registers */
static u64 rt_base;         /* Runtime registers */
static u64 db_base;         /* Doorbell registers */
static u32 max_ports;
static u32 max_slots;
static u32 ctx_size;        /* 32 or 64 bytes */
static u32 device_speed;
static u32 device_port;     /* port where device is connected */
static u32 device_slot;     /* assigned slot ID */
static u32 max_packet_ep0;  /* EP0 max packet size */
static u32 int_ep_addr;     /* interrupt IN endpoint address */
static u32 int_ep_maxpkt;   /* interrupt IN max packet size */
static u32 int_ep_interval; /* interrupt IN polling interval */
static bool device_ready;

/* Ring state */
static u32 cmd_enq;         /* command ring enqueue index */
static u32 cmd_cycle;       /* command ring cycle bit */
static u32 evt_deq;         /* event ring dequeue index */
static u32 evt_cycle;       /* event ring expected cycle */
static u32 ep0_enq;
static u32 ep0_cycle;
static u32 int_enq;
static u32 int_cycle;

/* ================================================================
 * Register Access Helpers
 * ================================================================ */

static inline u32 xr(u64 off) { return mmio_read(xhci_base + off); }
static inline void xw(u64 off, u32 v) { mmio_write(xhci_base + off, v); }
static inline u32 opr(u32 off) { return mmio_read(op_base + off); }
static inline void opw(u32 off, u32 v) { mmio_write(op_base + off, v); }
static inline u32 rtr(u32 off) { return mmio_read(rt_base + off); }
static inline void rtw(u32 off, u32 v) { mmio_write(rt_base + off, v); }
static inline void ring_db(u32 slot, u32 target) {
    mmio_write(db_base + slot * 4, target);
}

/* ================================================================
 * Ring Management
 * ================================================================ */

static void ring_init(struct xhci_trb *ring, u32 *enq, u32 *cycle) {
    for (u32 i = 0; i < RING_SIZE; i++) {
        ring[i].param = 0;
        ring[i].status = 0;
        ring[i].control = 0;
    }
    /* Last TRB is a Link TRB pointing back to start */
    ring[RING_SIZE - 1].param = (u64)(usize)ring;
    ring[RING_SIZE - 1].control = TRB_TYPE(TRB_LINK) | (1U << 1); /* TC=1 (toggle cycle) */
    *enq = 0;
    *cycle = 1;
}

static void ring_enqueue(struct xhci_trb *ring, u32 *enq, u32 *cycle,
                         u64 param, u32 status, u32 control) {
    u32 idx = *enq;
    ring[idx].param = param;
    ring[idx].status = status;
    /* Set cycle bit to match current producer cycle */
    ring[idx].control = control | (*cycle ? TRB_CYCLE : 0);
    dmb();

    idx++;
    if (idx >= RING_SIZE - 1) {
        /* Toggle cycle on the link TRB and wrap */
        ring[RING_SIZE - 1].control ^= TRB_CYCLE;
        *cycle ^= 1;
        idx = 0;
    }
    *enq = idx;
}

/* ================================================================
 * Event Ring
 * ================================================================ */

static bool evt_poll(struct xhci_trb *out, u32 timeout_ms) {
    u64 deadline = timeout_ms; /* simplified: loop counter */
    for (u64 i = 0; i < deadline * 100; i++) {
        struct xhci_trb *trb = &evt_ring[evt_deq];
        dmb();
        u32 c = trb->control;
        /* Check if cycle bit matches expected */
        if ((c & TRB_CYCLE) == (evt_cycle ? 1U : 0U)) {
            *out = *trb;
            evt_deq++;
            if (evt_deq >= RING_SIZE) {
                evt_deq = 0;
                evt_cycle ^= 1;
            }
            /* Update ERDP */
            u64 erdp = (u64)(usize)&evt_ring[evt_deq];
            rtw(IR0_ERDP_LO, (u32)erdp | (1U << 3)); /* EHB bit */
            rtw(IR0_ERDP_HI, (u32)(erdp >> 32));
            return true;
        }
        timer_delay_us(10);
    }
    return false;
}

/* ================================================================
 * Command Submission
 * ================================================================ */

/* Submit a command and wait for Command Completion Event */
static bool cmd_submit(u64 param, u32 status, u32 control, struct xhci_trb *evt) {
    ring_enqueue(cmd_ring, &cmd_enq, &cmd_cycle, param, status, control);
    dmb();
    ring_db(0, 0); /* doorbell 0, target 0 = host controller command */

    if (!evt_poll(evt, 500)) {
        uart_puts("[xhci] Command timeout\n");
        return false;
    }

    if (TRB_GET_TYPE(evt->control) != TRB_CMD_COMPLETE) {
        uart_puts("[xhci] Unexpected event type\n");
        return false;
    }

    u32 cc = TRB_COMP_CODE(evt->status);
    if (cc != CC_SUCCESS) {
        uart_puts("[xhci] Command failed, cc=");
        uart_hex(cc);
        uart_puts("\n");
        return false;
    }
    return true;
}

/* ================================================================
 * DWC3 Initialization
 * ================================================================ */

static bool dwc3_init(u64 base) {
    u32 snpsid = mmio_read(base + DWC3_GSNPSID);
    uart_puts("[xhci] DWC3 ID: ");
    uart_hex(snpsid);
    uart_puts("\n");

    /* Core soft reset */
    u32 gctl = mmio_read(base + DWC3_GCTL);
    gctl |= GCTL_CORESOFTRESET;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_us(100);

    /* PHY soft reset */
    u32 phycfg = mmio_read(base + DWC3_GUSB2PHYCFG);
    phycfg |= GUSB2_PHYSOFTRST;
    mmio_write(base + DWC3_GUSB2PHYCFG, phycfg);
    timer_delay_us(100);

    /* Deassert PHY reset */
    phycfg &= ~GUSB2_PHYSOFTRST;
    mmio_write(base + DWC3_GUSB2PHYCFG, phycfg);
    timer_delay_ms(10);

    /* Deassert core reset */
    gctl = mmio_read(base + DWC3_GCTL);
    gctl &= ~GCTL_CORESOFTRESET;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_ms(10);

    /* Set host mode */
    gctl = mmio_read(base + DWC3_GCTL);
    gctl &= ~GCTL_PRTCAP_MASK;
    gctl |= GCTL_PRTCAP_HOST;
    gctl &= ~GCTL_DSBLCLKGTNG;
    mmio_write(base + DWC3_GCTL, gctl);
    timer_delay_ms(10);

    return true;
}

/* ================================================================
 * xHCI Controller Init
 * ================================================================ */

static bool xhci_hw_init(void) {
    /* Read capability registers */
    u32 cap0 = xr(CAP_CAPLENGTH);
    u32 caplength = cap0 & 0xFF;
    u32 hcsparams1 = xr(CAP_HCSPARAMS1);
    u32 hcsparams2 = xr(CAP_HCSPARAMS2);
    u32 hccparams1 = xr(CAP_HCCPARAMS1);
    u32 dboff = xr(CAP_DBOFF);
    u32 rtsoff = xr(CAP_RTSOFF);

    max_slots = hcsparams1 & 0xFF;
    max_ports = (hcsparams1 >> 24) & 0xFF;
    ctx_size = (hccparams1 & (1 << 2)) ? 64 : 32;

    uart_puts("[xhci] CAPLENGTH=");
    uart_hex(caplength);
    uart_puts(" MaxSlots=");
    uart_hex(max_slots);
    uart_puts(" MaxPorts=");
    uart_hex(max_ports);
    uart_puts(" CtxSize=");
    uart_hex(ctx_size);
    uart_puts("\n");

    if (ctx_size != 32) {
        uart_puts("[xhci] WARNING: 64-byte contexts, using padded offsets\n");
    }

    op_base = xhci_base + caplength;
    rt_base = xhci_base + rtsoff;
    db_base = xhci_base + dboff;

    /* Halt controller */
    opw(OP_USBCMD, opr(OP_USBCMD) & ~CMD_RUN);
    for (u32 i = 0; i < 1000; i++) {
        if (opr(OP_USBSTS) & STS_HCH) break;
        timer_delay_us(100);
    }
    if (!(opr(OP_USBSTS) & STS_HCH)) {
        uart_puts("[xhci] Failed to halt\n");
        return false;
    }

    /* Reset controller */
    opw(OP_USBCMD, CMD_HCRST);
    for (u32 i = 0; i < 1000; i++) {
        if (!(opr(OP_USBCMD) & CMD_HCRST) && !(opr(OP_USBSTS) & STS_CNR))
            break;
        timer_delay_us(100);
    }
    if (opr(OP_USBCMD) & CMD_HCRST) {
        uart_puts("[xhci] Reset timeout\n");
        return false;
    }

    /* MaxSlotsEn = 1 (we only need one device) */
    opw(OP_CONFIG, 1);

    /* Init DCBAA */
    for (u32 i = 0; i < 256; i++) dcbaa[i] = 0;

    /* Handle scratchpad buffers */
    u32 sp_hi = (hcsparams2 >> 21) & 0x1F;
    u32 sp_lo = (hcsparams2 >> 27) & 0x1F;
    u32 num_sp = (sp_hi << 5) | sp_lo;
    if (num_sp > 0) {
        if (num_sp > 4) num_sp = 4; /* limit to our static pool */
        for (u32 i = 0; i < num_sp; i++)
            scratchpad_array[i] = (u64)(usize)&scratchpad_bufs[i][0];
        dcbaa[0] = (u64)(usize)scratchpad_array;
        uart_puts("[xhci] Scratchpad: ");
        uart_hex(num_sp);
        uart_puts(" pages\n");
    }

    u64 dcbaa_phys = (u64)(usize)dcbaa;
    opw(OP_DCBAAP_LO, (u32)dcbaa_phys);
    opw(OP_DCBAAP_HI, (u32)(dcbaa_phys >> 32));

    /* Init Command Ring */
    ring_init(cmd_ring, &cmd_enq, &cmd_cycle);
    u64 cmd_phys = (u64)(usize)cmd_ring;
    opw(OP_CRCR_LO, (u32)cmd_phys | cmd_cycle);
    opw(OP_CRCR_HI, (u32)(cmd_phys >> 32));

    /* Init Event Ring */
    for (u32 i = 0; i < RING_SIZE; i++) {
        evt_ring[i].param = 0;
        evt_ring[i].status = 0;
        evt_ring[i].control = 0;
    }
    evt_deq = 0;
    evt_cycle = 1;

    erst[0].base = (u64)(usize)evt_ring;
    erst[0].size = RING_SIZE;
    erst[0].rsvd = 0;

    rtw(IR0_ERSTSZ, 1);
    u64 erdp = (u64)(usize)evt_ring;
    rtw(IR0_ERDP_LO, (u32)erdp);
    rtw(IR0_ERDP_HI, (u32)(erdp >> 32));
    u64 erstba = (u64)(usize)erst;
    rtw(IR0_ERSTBA_LO, (u32)erstba);
    rtw(IR0_ERSTBA_HI, (u32)(erstba >> 32));

    /* Init EP0 and Interrupt IN transfer rings */
    ring_init(ep0_ring, &ep0_enq, &ep0_cycle);
    ring_init(int_ring, &int_enq, &int_cycle);

    /* Start controller */
    opw(OP_USBCMD, CMD_RUN | CMD_INTE);
    timer_delay_ms(10);

    if (opr(OP_USBSTS) & STS_HCH) {
        uart_puts("[xhci] Failed to start\n");
        return false;
    }

    uart_puts("[xhci] Controller running\n");
    return true;
}

/* ================================================================
 * Port Operations
 * ================================================================ */

static u64 portsc_addr(u32 port) {
    return op_base + PORT_REGS_OFFSET + (u64)port * 0x10 + PORT_SC;
}

static u32 portsc_read(u32 port) {
    return mmio_read(portsc_addr(port));
}

static void portsc_write(u32 port, u32 val) {
    mmio_write(portsc_addr(port), val);
}

/* Find a port with a connected device. Returns port index or -1. */
static i32 find_connected_port(void) {
    for (u32 p = 0; p < max_ports; p++) {
        u32 sc = portsc_read(p);
        if (sc & PORTSC_CCS) {
            uart_puts("[xhci] Device on port ");
            uart_hex(p);
            uart_puts(" PORTSC=");
            uart_hex(sc);
            uart_puts("\n");
            return (i32)p;
        }
    }
    return -1;
}

/* Reset a port and wait for it to complete */
static bool port_reset(u32 port) {
    u32 sc = portsc_read(port);

    /* Preserve power, clear change bits, assert reset */
    sc &= ~PORTSC_RW1C_MASK;
    sc |= PORTSC_PR;
    portsc_write(port, sc);

    /* Wait for PRC (Port Reset Change) */
    for (u32 i = 0; i < 200; i++) {
        timer_delay_ms(5);
        sc = portsc_read(port);
        if (sc & PORTSC_PRC) {
            /* Clear PRC */
            sc &= ~PORTSC_RW1C_MASK;
            sc |= PORTSC_PRC;
            portsc_write(port, sc);

            device_speed = (sc & PORTSC_SPEED_MASK) >> PORTSC_SPEED_SHIFT;
            uart_puts("[xhci] Port reset OK, speed=");
            uart_hex(device_speed);
            uart_puts("\n");
            return true;
        }
    }
    uart_puts("[xhci] Port reset timeout\n");
    return false;
}

/* ================================================================
 * USB Control Transfers on EP0
 * ================================================================ */

static bool ep0_control(u8 bmReq, u8 bReq, u16 wVal, u16 wIdx, u16 wLen,
                        void *data, u32 *actual) {
    /* Setup Stage TRB */
    u64 setup_param = (u64)bmReq | ((u64)bReq << 8) |
                      ((u64)wVal << 16) | ((u64)wIdx << 32) |
                      ((u64)wLen << 48);
    u32 setup_ctrl = TRB_TYPE(TRB_SETUP) | TRB_IDT;
    if (wLen > 0)
        setup_ctrl |= (bmReq & USB_DIR_IN) ? (3U << 16) : (2U << 16); /* TRT */
    ring_enqueue(ep0_ring, &ep0_enq, &ep0_cycle,
                 setup_param, 8, setup_ctrl);

    /* Data Stage TRB (if any) */
    if (wLen > 0 && data) {
        u32 data_ctrl = TRB_TYPE(TRB_DATA);
        if (bmReq & USB_DIR_IN)
            data_ctrl |= TRB_DIR_IN;
        ring_enqueue(ep0_ring, &ep0_enq, &ep0_cycle,
                     (u64)(usize)data, wLen, data_ctrl);
    }

    /* Status Stage TRB (direction opposite to data, or IN if no data) */
    u32 status_ctrl = TRB_TYPE(TRB_STATUS) | TRB_IOC;
    if (wLen == 0 || (bmReq & USB_DIR_IN))
        status_ctrl |= 0;          /* OUT direction for status */
    else
        status_ctrl |= TRB_DIR_IN; /* IN direction for status */
    ring_enqueue(ep0_ring, &ep0_enq, &ep0_cycle,
                 0, 0, status_ctrl);

    dmb();
    ring_db(device_slot, 1); /* doorbell slot N, target 1 = EP0 */

    /* Wait for Transfer Event */
    struct xhci_trb evt;
    if (!evt_poll(&evt, 1000)) {
        uart_puts("[xhci] EP0 transfer timeout\n");
        return false;
    }

    u32 cc = TRB_COMP_CODE(evt.status);
    if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) {
        uart_puts("[xhci] EP0 transfer error, cc=");
        uart_hex(cc);
        uart_puts("\n");
        return false;
    }

    if (actual)
        *actual = wLen - (evt.status & 0xFFFFFF);
    return true;
}

/* ================================================================
 * Device Enumeration
 * ================================================================ */

static bool enumerate_device(u32 port) {
    struct xhci_trb evt;

    /* Enable Slot */
    if (!cmd_submit(0, 0, TRB_TYPE(TRB_ENABLE_SLOT), &evt))
        return false;

    device_slot = (evt.control >> 24) & 0xFF;
    uart_puts("[xhci] Slot ");
    uart_hex(device_slot);
    uart_puts(" enabled\n");

    /* Build Input Context for Address Device */
    u8 *input = input_ctx_buf;
    for (u32 i = 0; i < sizeof(input_ctx_buf); i++) input[i] = 0;

    struct xhci_input_ctrl *ic = (struct xhci_input_ctrl *)input;
    ic->add_flags = (1 << 0) | (1 << 1); /* add slot ctx + EP0 ctx */

    /* Slot Context */
    struct xhci_slot_ctx *slot = (struct xhci_slot_ctx *)(input + ctx_size);
    u32 route = 0;
    u32 speed_val = device_speed;
    slot->field0 = route | (speed_val << 20) | (1U << 27); /* context_entries = 1 (just EP0) */
    slot->field1 = (port + 1) << 16; /* root hub port number (1-based) */

    /* EP0 Context */
    struct xhci_ep_ctx *ep0 = (struct xhci_ep_ctx *)(input + ctx_size * 2);
    /* Default max packet size based on speed */
    switch (device_speed) {
    case USB_SPEED_LOW:     max_packet_ep0 = 8; break;
    case USB_SPEED_FULL:    max_packet_ep0 = 8; break;
    case USB_SPEED_HIGH:    max_packet_ep0 = 64; break;
    case USB_SPEED_SUPER:   max_packet_ep0 = 512; break;
    default:                max_packet_ep0 = 8; break;
    }

    /* EP type 4 = Control Bidirectional, CErr = 3 */
    ep0->field1 = (3U << 1) | (4U << 3) | ((u32)max_packet_ep0 << 16);
    u64 ep0_phys = (u64)(usize)ep0_ring;
    ep0->deq = ep0_phys | ep0_cycle;
    ep0->field3 = 8; /* average TRB length = 8 for control */

    dcache_clean_range((u64)(usize)input, sizeof(input_ctx_buf));
    dcache_clean_range((u64)(usize)ep0_ring, sizeof(ep0_ring));

    /* Address Device command */
    u64 input_phys = (u64)(usize)input;
    if (!cmd_submit(input_phys, 0, TRB_TYPE(TRB_ADDRESS_DEV) | (device_slot << 24), &evt))
        return false;

    /* Point DCBAA to device context */
    dcbaa[device_slot] = (u64)(usize)dev_ctx_buf;
    dcache_clean_range((u64)(usize)dcbaa, sizeof(dcbaa));

    uart_puts("[xhci] Device addressed\n");

    /* GET_DESCRIPTOR: Device Descriptor */
    struct usb_dev_desc *dd = (struct usb_dev_desc *)usb_buf;
    u32 got = 0;
    if (!ep0_control(USB_DIR_IN, USB_REQ_GET_DESC,
                     (USB_DESC_DEVICE << 8), 0, 18, dd, &got)) {
        uart_puts("[xhci] GET_DESCRIPTOR(device) failed\n");
        return false;
    }

    uart_puts("[xhci] USB ");
    uart_hex(dd->idVendor);
    uart_puts(":");
    uart_hex(dd->idProduct);
    uart_puts(" class=");
    uart_hex(dd->bDeviceClass);
    uart_puts(" maxpkt0=");
    uart_hex(dd->bMaxPacketSize0);
    uart_puts("\n");

    max_packet_ep0 = dd->bMaxPacketSize0;

    /* GET_DESCRIPTOR: Configuration Descriptor (full) */
    u8 *cfg_buf = usb_buf;
    if (!ep0_control(USB_DIR_IN, USB_REQ_GET_DESC,
                     (USB_DESC_CONFIG << 8), 0, sizeof(usb_buf), cfg_buf, &got)) {
        uart_puts("[xhci] GET_DESCRIPTOR(config) failed\n");
        return false;
    }

    /* Parse configuration to find HID interrupt IN endpoint */
    struct usb_cfg_desc *cfg = (struct usb_cfg_desc *)cfg_buf;
    u8 config_val = cfg->bConfigurationValue;
    int_ep_addr = 0;

    u32 off = 0;
    while (off + 2 <= got) {
        u8 len = cfg_buf[off];
        u8 type = cfg_buf[off + 1];
        if (len == 0) break;

        if (type == 5 && len >= 7) { /* Endpoint descriptor */
            struct usb_ep_desc *ep = (struct usb_ep_desc *)(cfg_buf + off);
            if ((ep->bEndpointAddress & USB_DIR_IN) &&
                (ep->bmAttributes & 0x03) == 3) { /* Interrupt IN */
                int_ep_addr = ep->bEndpointAddress;
                int_ep_maxpkt = ep->wMaxPacketSize & 0x7FF;
                int_ep_interval = ep->bInterval;
                uart_puts("[xhci] Found INT IN EP ");
                uart_hex(int_ep_addr);
                uart_puts(" maxpkt=");
                uart_hex(int_ep_maxpkt);
                uart_puts("\n");
                break;
            }
        }
        off += len;
    }

    /* SET_CONFIGURATION */
    if (!ep0_control(USB_DIR_OUT, USB_REQ_SET_CONFIG,
                     config_val, 0, 0, NULL, NULL)) {
        uart_puts("[xhci] SET_CONFIGURATION failed\n");
        return false;
    }

    /* If we found an interrupt endpoint, configure it */
    if (int_ep_addr) {
        u32 ep_num = (int_ep_addr & 0x0F);
        u32 ep_dci = ep_num * 2 + 1; /* DCI for IN endpoint */

        /* Build Input Context for Configure Endpoint */
        for (u32 i = 0; i < sizeof(input_ctx_buf); i++) input[i] = 0;
        ic->add_flags = (1 << 0) | (1 << ep_dci); /* add slot ctx + EP */

        /* Update slot context entries count */
        slot = (struct xhci_slot_ctx *)(input + ctx_size);
        slot->field0 = route | (speed_val << 20) | (ep_dci << 27);
        slot->field1 = (port + 1) << 16;

        /* Interrupt IN EP context */
        struct xhci_ep_ctx *iep = (struct xhci_ep_ctx *)(input + ctx_size * (ep_dci + 1));
        /* EP type 7 = Interrupt IN, CErr = 3 */
        iep->field0 = ((u32)int_ep_interval << 16);
        iep->field1 = (3U << 1) | (7U << 3) | ((u32)int_ep_maxpkt << 16);
        u64 int_phys = (u64)(usize)int_ring;
        iep->deq = int_phys | int_cycle;
        iep->field3 = int_ep_maxpkt; /* average TRB length */

        dcache_clean_range((u64)(usize)input, sizeof(input_ctx_buf));
        dcache_clean_range((u64)(usize)int_ring, sizeof(int_ring));

        if (!cmd_submit((u64)(usize)input, 0,
                        TRB_TYPE(TRB_CONFIG_EP) | (device_slot << 24), &evt)) {
            uart_puts("[xhci] Configure Endpoint failed\n");
            int_ep_addr = 0;
        } else {
            uart_puts("[xhci] Interrupt EP configured\n");
        }
    }

    device_ready = true;
    return true;
}

/* ================================================================
 * HID Keyboard Polling
 * ================================================================ */

static bool keyboard_poll_pending = false;

static void queue_int_transfer(void) {
    if (!int_ep_addr || keyboard_poll_pending) return;
    u32 len = int_ep_maxpkt;
    if (len > sizeof(usb_buf)) len = sizeof(usb_buf);

    ring_enqueue(int_ring, &int_enq, &int_cycle,
                 (u64)(usize)usb_buf, len,
                 TRB_TYPE(TRB_NORMAL) | TRB_IOC);
    dmb();

    u32 ep_num = (int_ep_addr & 0x0F);
    u32 ep_dci = ep_num * 2 + 1;
    ring_db(device_slot, ep_dci);
    keyboard_poll_pending = true;
}

/* ================================================================
 * Public API
 * ================================================================ */

bool xhci_init(void) {
    uart_puts("[xhci] Init USB0 (DWC3)...\n");

    xhci_base = RP1_BAR_BASE + XHCI_USB0_OFFSET;
    device_ready = false;

    /* DWC3 initialization */
    if (!dwc3_init(xhci_base)) {
        uart_puts("[xhci] DWC3 init failed\n");
        return false;
    }

    /* xHCI initialization */
    if (!xhci_hw_init()) {
        uart_puts("[xhci] HW init failed\n");
        return false;
    }

    /* Wait for device connection */
    timer_delay_ms(200);

    i32 port = find_connected_port();
    if (port < 0) {
        uart_puts("[xhci] No USB device detected\n");
        return false;
    }

    device_port = (u32)port;
    if (!port_reset(device_port))
        return false;

    timer_delay_ms(50);

    /* Enumerate */
    if (!enumerate_device(device_port))
        return false;

    /* Start keyboard polling if interrupt EP found */
    if (int_ep_addr)
        queue_int_transfer();

    uart_puts("[xhci] USB device ready\n");
    return true;
}

bool xhci_device_connected(void) {
    return device_ready;
}

i32 xhci_get_keypress(void) {
    if (!device_ready || !int_ep_addr)
        return -1;

    /* Check for completed transfer event (non-blocking) */
    struct xhci_trb *trb = &evt_ring[evt_deq];
    dmb();
    if ((trb->control & TRB_CYCLE) != (evt_cycle ? 1U : 0U))
        return -1; /* no event ready */

    struct xhci_trb evt = *trb;
    evt_deq++;
    if (evt_deq >= RING_SIZE) {
        evt_deq = 0;
        evt_cycle ^= 1;
    }
    u64 erdp = (u64)(usize)&evt_ring[evt_deq];
    rtw(IR0_ERDP_LO, (u32)erdp | (1U << 3));
    rtw(IR0_ERDP_HI, (u32)(erdp >> 32));

    keyboard_poll_pending = false;

    u32 cc = TRB_COMP_CODE(evt.status);
    if (cc != CC_SUCCESS && cc != CC_SHORT_PACKET) {
        queue_int_transfer();
        return -1;
    }

    dcache_invalidate_range((u64)(usize)usb_buf, int_ep_maxpkt);

    /*
     * Standard HID keyboard report: 8 bytes
     *   [0] modifier keys (shift, ctrl, alt, etc.)
     *   [1] reserved
     *   [2..7] up to 6 simultaneous keycodes
     */
    i32 keycode = -1;
    if (usb_buf[2] != 0)
        keycode = (i32)usb_buf[2];

    /* Re-queue next poll */
    queue_int_transfer();
    return keycode;
}
