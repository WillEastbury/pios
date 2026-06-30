/*
 * virtio_net.c - virtio-net (virtio-mmio) NIC backend for QEMU `virt`.
 *
 * Polling-only, non-blocking RX/TX so the full kernel net stack runs unchanged
 * under QEMU. Compiled to nothing on platforms without virtio (PIOS_HAS_VIRTIO_NET).
 */

#include "virtio_net.h"

#if PIOS_HAS_VIRTIO_NET

#include "mmio.h"
#include "mmu.h"
#include "simd.h"
#include "uart.h"
#include "dtrace.h"

/* ---- virtio-mmio register offsets ---- */
#define VIRTIO_MMIO_MAGIC       0x74726976U   /* "virt" */
#define VR_MAGIC                0x000U
#define VR_VERSION              0x004U
#define VR_DEVICE_ID            0x008U
#define VR_DEVICE_FEAT          0x010U
#define VR_DEVICE_SEL           0x014U
#define VR_DRIVER_FEAT          0x020U
#define VR_DRIVER_SEL           0x024U
#define VR_GUEST_PAGE_SIZE      0x028U   /* legacy */
#define VR_QUEUE_SEL            0x030U
#define VR_QUEUE_NUM_MAX        0x034U
#define VR_QUEUE_NUM            0x038U
#define VR_QUEUE_ALIGN          0x03CU   /* legacy */
#define VR_QUEUE_PFN            0x040U   /* legacy */
#define VR_QUEUE_READY          0x044U   /* modern */
#define VR_QUEUE_NOTIFY         0x050U
#define VR_INT_STATUS           0x060U
#define VR_INT_ACK              0x064U
#define VR_STATUS               0x070U
#define VR_Q_DESC_LOW           0x080U
#define VR_Q_DESC_HIGH          0x084U
#define VR_Q_DRV_LOW            0x090U
#define VR_Q_DRV_HIGH           0x094U
#define VR_Q_DEV_LOW            0x0A0U
#define VR_Q_DEV_HIGH           0x0A4U
#define VR_CONFIG               0x100U

#define VIRTIO_NET_DEVICE_ID    1U
#define VST_ACK                 1U
#define VST_DRIVER              2U
#define VST_DRIVER_OK           4U
#define VST_FEAT_OK             8U

#define VIRTIO_NET_F_MAC        (1U << 5)
#define VIRTIO_F_VERSION_1_W1   (1U << 0)   /* bit 32 => word 1 bit 0 */

#define VIRTQ_DESC_F_NEXT       1U
#define VIRTQ_DESC_F_WRITE      2U

#define VQ_SIZE                 32U          /* must be power of two */
#define VQ_RX                   0U
#define VQ_TX                   1U
#define VNET_BUF_SIZE           2048U

struct vq_desc {
    u64 addr;
    u32 len;
    u16 flags;
    u16 next;
} PACKED;

struct vq_avail {
    u16 flags;
    u16 idx;
    u16 ring[VQ_SIZE];
    u16 used_event;
} PACKED;

struct vq_used_elem {
    u32 id;
    u32 len;
} PACKED;

struct vq_used {
    u16 flags;
    u16 idx;
    struct vq_used_elem ring[VQ_SIZE];
    u16 avail_event;
} PACKED;

/* Legacy single-region queue: desc, avail, then used aligned to 4KiB. */
struct vq_legacy {
    struct vq_desc desc[VQ_SIZE];
    struct vq_avail avail;
    u8 pad[4096U - (sizeof(struct vq_desc) * VQ_SIZE) - sizeof(struct vq_avail)];
    struct vq_used used;
} ALIGNED(4096);

/* Modern split-queue rings (separate device-visible regions). */
static struct vq_desc  rx_desc[VQ_SIZE] ALIGNED(64);
static struct vq_avail rx_avail ALIGNED(64);
static struct vq_used  rx_used  ALIGNED(64);
static struct vq_desc  tx_desc[VQ_SIZE] ALIGNED(64);
static struct vq_avail tx_avail ALIGNED(64);
static struct vq_used  tx_used  ALIGNED(64);

static struct vq_legacy rx_legacy;
static struct vq_legacy tx_legacy;

static u8 rx_buf[VQ_SIZE][VNET_BUF_SIZE] ALIGNED(64);
static u8 tx_buf[VQ_SIZE][VNET_BUF_SIZE] ALIGNED(64);

static u64  vnet_base;
static bool vnet_ready;
static bool vnet_legacy;
static u32  vnet_hdr_len;
static u32  vnet_diag;
static u8   vnet_mac[6] = {0x52, 0x54, 0x00, 0x12, 0x34, 0x56};

/* RX: index of the next used-ring entry we have not yet consumed. */
static u16 rx_used_seen;
/* TX: producer cursor and reclaim cursor (free in-flight descriptors). */
static u16 tx_prod;
static u16 tx_used_seen;
static u64 g_vnet_tx_ok;
static u64 g_vnet_tx_drop;
static u64 g_vnet_rx_ok;
static u64 g_vnet_rx_starve;   /* times the RX backlog hit the full ring (device may have dropped) */
static u16 g_vnet_rx_backlog_max;

static inline struct vq_desc  *rx_d(void) { return vnet_legacy ? rx_legacy.desc : rx_desc; }
static inline struct vq_avail *rx_a(void) { return vnet_legacy ? &rx_legacy.avail : &rx_avail; }
static inline struct vq_used  *rx_u(void) { return vnet_legacy ? &rx_legacy.used : &rx_used; }
static inline struct vq_desc  *tx_d(void) { return vnet_legacy ? tx_legacy.desc : tx_desc; }
static inline struct vq_avail *tx_a(void) { return vnet_legacy ? &tx_legacy.avail : &tx_avail; }
static inline struct vq_used  *tx_u(void) { return vnet_legacy ? &tx_legacy.used : &tx_used; }

static inline void clean(const void *p, u32 n)      { dcache_clean_range((u64)(usize)p, n); }
static inline void invalidate(const void *p, u32 n) { dcache_invalidate_range((u64)(usize)p, n); }

static void vnet_notify(u32 q)
{
    dsb();
    mmio_write(vnet_base + VR_QUEUE_NOTIFY, q);
}

static bool vnet_setup_queue(u32 q, struct vq_desc *d, struct vq_avail *a, struct vq_used *u)
{
    mmio_write(vnet_base + VR_QUEUE_SEL, q);
    if (mmio_read(vnet_base + VR_QUEUE_NUM_MAX) < VQ_SIZE)
        return false;
    mmio_write(vnet_base + VR_QUEUE_NUM, VQ_SIZE);

    if (vnet_legacy) {
        /* One contiguous, page-aligned region addressed by page-frame number. */
        mmio_write(vnet_base + VR_QUEUE_ALIGN, 4096);
        mmio_write(vnet_base + VR_QUEUE_PFN, (u32)((u64)(usize)d >> 12));
        return true;
    }

    mmio_write(vnet_base + VR_Q_DESC_LOW,  (u32)(usize)d);
    mmio_write(vnet_base + VR_Q_DESC_HIGH, (u32)((u64)(usize)d >> 32));
    mmio_write(vnet_base + VR_Q_DRV_LOW,   (u32)(usize)a);
    mmio_write(vnet_base + VR_Q_DRV_HIGH,  (u32)((u64)(usize)a >> 32));
    mmio_write(vnet_base + VR_Q_DEV_LOW,   (u32)(usize)u);
    mmio_write(vnet_base + VR_Q_DEV_HIGH,  (u32)((u64)(usize)u >> 32));
    mmio_write(vnet_base + VR_QUEUE_READY, 1);
    return true;
}

static void vnet_read_mac(void)
{
    /* config-space MAC is valid once VIRTIO_NET_F_MAC is negotiated. */
    for (u32 i = 0; i < 6; i++)
        vnet_mac[i] = mmio_read8(vnet_base + VR_CONFIG + i);
    /* If the device reported all-zero, keep the QEMU default. */
    u8 acc = 0;
    for (u32 i = 0; i < 6; i++)
        acc |= vnet_mac[i];
    if (acc == 0) {
        vnet_mac[0] = 0x52; vnet_mac[1] = 0x54; vnet_mac[2] = 0x00;
        vnet_mac[3] = 0x12; vnet_mac[4] = 0x34; vnet_mac[5] = 0x56;
    }
}

bool virtio_net_init(void)
{
    vnet_base = 0;
    vnet_ready = false;
    vnet_diag = 0;
    rx_used_seen = 0;
    tx_prod = 0;
    tx_used_seen = 0;

    for (u32 i = 0; i < PIOS_VIRTIO_MMIO_COUNT; i++) {
        u64 base = PIOS_VIRTIO_MMIO_BASE + (u64)i * PIOS_VIRTIO_MMIO_STRIDE;
        if (mmio_read(base + VR_MAGIC) == VIRTIO_MMIO_MAGIC &&
            mmio_read(base + VR_DEVICE_ID) == VIRTIO_NET_DEVICE_ID) {
            vnet_base = base;
            break;
        }
    }
    if (!vnet_base) { vnet_diag = 1; return false; }

    u32 ver = mmio_read(vnet_base + VR_VERSION);
    vnet_legacy = (ver == 1U);
    vnet_hdr_len = vnet_legacy ? 10U : 12U;

    /* Reset, then ACK + DRIVER. */
    mmio_write(vnet_base + VR_STATUS, 0);
    mmio_write(vnet_base + VR_STATUS, VST_ACK);
    mmio_write(vnet_base + VR_STATUS, VST_ACK | VST_DRIVER);

    if (vnet_legacy)
        mmio_write(vnet_base + VR_GUEST_PAGE_SIZE, 4096);

    /* Negotiate only the subset the device actually offers. */
    mmio_write(vnet_base + VR_DEVICE_SEL, 0);
    u32 dev0 = mmio_read(vnet_base + VR_DEVICE_FEAT);
    mmio_write(vnet_base + VR_DEVICE_SEL, 1);
    u32 dev1 = mmio_read(vnet_base + VR_DEVICE_FEAT);

    u32 drv0 = dev0 & VIRTIO_NET_F_MAC;
    u32 drv1 = vnet_legacy ? 0U : (dev1 & VIRTIO_F_VERSION_1_W1);

    mmio_write(vnet_base + VR_DRIVER_SEL, 0);
    mmio_write(vnet_base + VR_DRIVER_FEAT, drv0);
    mmio_write(vnet_base + VR_DRIVER_SEL, 1);
    mmio_write(vnet_base + VR_DRIVER_FEAT, drv1);

    if (!vnet_legacy) {
        mmio_write(vnet_base + VR_STATUS, VST_ACK | VST_DRIVER | VST_FEAT_OK);
        if ((mmio_read(vnet_base + VR_STATUS) & VST_FEAT_OK) == 0) {
            vnet_diag = 2;
            return false;
        }
    }

    if (drv0 & VIRTIO_NET_F_MAC)
        vnet_read_mac();

    simd_zero(rx_desc, sizeof(rx_desc));   simd_zero(&rx_avail, sizeof(rx_avail));
    simd_zero(&rx_used, sizeof(rx_used));  simd_zero(&rx_legacy, sizeof(rx_legacy));
    simd_zero(tx_desc, sizeof(tx_desc));   simd_zero(&tx_avail, sizeof(tx_avail));
    simd_zero(&tx_used, sizeof(tx_used));  simd_zero(&tx_legacy, sizeof(tx_legacy));

    if (!vnet_setup_queue(VQ_RX, rx_d(), rx_a(), rx_u())) { vnet_diag = 3; return false; }
    if (!vnet_setup_queue(VQ_TX, tx_d(), tx_a(), tx_u())) { vnet_diag = 4; return false; }

    /* Publish every RX buffer to the device. */
    struct vq_desc *rd = rx_d();
    struct vq_avail *ra = rx_a();
    for (u32 i = 0; i < VQ_SIZE; i++) {
        rd[i].addr  = (u64)(usize)rx_buf[i];
        rd[i].len   = VNET_BUF_SIZE;
        rd[i].flags = VIRTQ_DESC_F_WRITE;
        rd[i].next  = 0;
        ra->ring[i] = (u16)i;
    }
    ra->idx = VQ_SIZE;
    clean(rd, sizeof(struct vq_desc) * VQ_SIZE);
    clean(ra, sizeof(struct vq_avail));
    dsb();

    /* Match the proven legacy sequence: notify the device of the freshly
     * published RX buffers, THEN flip DRIVER_OK. */
    vnet_notify(VQ_RX);
    mmio_write(vnet_base + VR_STATUS, VST_ACK | VST_DRIVER | VST_FEAT_OK | VST_DRIVER_OK);

    vnet_ready = true;
    vnet_diag = 10;
    mmio_write(vnet_base + VR_QUEUE_SEL, VQ_RX);
    uart_puts("[vnet] online ");
    uart_puts(vnet_legacy ? "(legacy)" : "(modern)");
    uart_puts(" mac=");
    for (u32 i = 0; i < 6; i++) uart_hex(vnet_mac[i]);
    uart_puts("\n");
    return true;
}

bool virtio_net_present(void) { return vnet_ready; }
u32  virtio_net_diag_code(void) { return vnet_diag; }

/* Non-destructive scan: is a virtio-net device present on the MMIO bus? */
bool virtio_net_probe(void)
{
    for (u32 i = 0; i < PIOS_VIRTIO_MMIO_COUNT; i++) {
        u64 base = PIOS_VIRTIO_MMIO_BASE + (u64)i * PIOS_VIRTIO_MMIO_STRIDE;
        if (mmio_read(base + VR_MAGIC) == VIRTIO_MMIO_MAGIC &&
            mmio_read(base + VR_DEVICE_ID) == VIRTIO_NET_DEVICE_ID)
            return true;
    }
    return false;
}

void virtio_net_get_mac(u8 *mac) { if (mac) simd_memcpy(mac, vnet_mac, 6); }
bool virtio_net_link_up(void) { return vnet_ready; }

static void tx_reclaim(void)
{
    struct vq_used *tu = tx_u();
    invalidate(tu, sizeof(struct vq_used));
    /* Used.idx advances as the device finishes our TX descriptors. */
    tx_used_seen = tu->idx;
}

bool virtio_net_send(const u8 *frame, u32 len)
{
    if (!vnet_ready || !frame || len == 0 || len + vnet_hdr_len > VNET_BUF_SIZE)
        return false;

    tx_reclaim();
    /* In-flight = tx_prod - tx_used_seen. Block-free: drop if the ring is full.
     * A drop here loses a TCP segment -> peer retransmits after its RTO (~1s),
     * so this counter is the prime suspect for latency stalls under load. */
    if ((u16)(tx_prod - tx_used_seen) >= VQ_SIZE) {
        g_vnet_tx_drop++;
        DTRACE(DTRACE_CAT_MAC, DT_MAC_TX, len, tx_prod, (u16)(tx_prod - tx_used_seen), 1);
        return false;
    }

    u32 slot = tx_prod & (VQ_SIZE - 1);
    u8 *buf = tx_buf[slot];
    simd_zero(buf, vnet_hdr_len);
    simd_memcpy(buf + vnet_hdr_len, frame, len);

    struct vq_desc *td = tx_d();
    struct vq_avail *ta = tx_a();
    td[slot].addr  = (u64)(usize)buf;
    td[slot].len   = len + vnet_hdr_len;
    td[slot].flags = 0;
    td[slot].next  = 0;
    clean(buf, len + vnet_hdr_len);
    clean(&td[slot], sizeof(struct vq_desc));

    ta->ring[ta->idx & (VQ_SIZE - 1)] = (u16)slot;
    dsb();
    ta->idx++;
    clean(ta, sizeof(struct vq_avail));
    dsb();
    tx_prod++;

    vnet_notify(VQ_TX);
    g_vnet_tx_ok++;
    DTRACE(DTRACE_CAT_MAC, DT_MAC_TX, len, tx_prod, (u16)(tx_prod - tx_used_seen), 0);
    return true;
}

void virtio_net_counters(u64 *tx_ok, u64 *tx_drop, u64 *rx_ok, u64 *rx_starve)
{
    if (tx_ok) *tx_ok = g_vnet_tx_ok;
    if (tx_drop) *tx_drop = g_vnet_tx_drop;
    if (rx_ok) *rx_ok = g_vnet_rx_ok;
    if (rx_starve) *rx_starve = g_vnet_rx_starve;
}

bool virtio_net_recv(u8 *frame, u32 *len)
{
    if (!vnet_ready || !frame || !len)
        return false;

    struct vq_used *ru = rx_u();
    invalidate(ru, sizeof(struct vq_used));
    /* Legacy virtio-mmio: acknowledge the device's used-buffer interrupt, else
     * the device will not continue delivering received frames. */
    {
        u32 isr = mmio_read(vnet_base + VR_INT_STATUS);
        if (isr)
            mmio_write(vnet_base + VR_INT_ACK, isr);
    }
    if (ru->idx == rx_used_seen)
        return false;   /* nothing new */

    /* RX-side coverage: how far behind are we? If the device has filled the
     * whole ring before we drained it, inbound frames may have been dropped at
     * the device for lack of free buffers (the RX equivalent of the TX-drop
     * stall). Track the worst backlog + a starvation count. */
    u16 backlog = (u16)(ru->idx - rx_used_seen);
    if (backlog > g_vnet_rx_backlog_max)
        g_vnet_rx_backlog_max = backlog;
    if (backlog >= VQ_SIZE)
        g_vnet_rx_starve++;

    u32 slot = rx_used_seen & (VQ_SIZE - 1);
    struct vq_used_elem *e = &ru->ring[slot];
    u32 id = e->id & (VQ_SIZE - 1);
    u32 used_len = e->len;

    bool ok = false;
    if (used_len > vnet_hdr_len) {
        u32 payload = used_len - vnet_hdr_len;
        if (payload > VNET_BUF_SIZE - vnet_hdr_len)
            payload = VNET_BUF_SIZE - vnet_hdr_len;
        invalidate(rx_buf[id], used_len);
        simd_memcpy(frame, rx_buf[id] + vnet_hdr_len, payload);
        *len = payload;
        ok = true;
        g_vnet_rx_ok++;
        DTRACE(DTRACE_CAT_MAC, DT_MAC_RX, payload, g_vnet_rx_ok, backlog, 0);
    }

    /* Re-publish this RX buffer to the device. */
    struct vq_desc *rd = rx_d();
    struct vq_avail *ra = rx_a();
    rd[id].addr  = (u64)(usize)rx_buf[id];
    rd[id].len   = VNET_BUF_SIZE;
    rd[id].flags = VIRTQ_DESC_F_WRITE;
    rd[id].next  = 0;
    clean(&rd[id], sizeof(struct vq_desc));
    ra->ring[ra->idx & (VQ_SIZE - 1)] = (u16)id;
    dsb();
    ra->idx++;
    clean(ra, sizeof(struct vq_avail));
    dsb();
    vnet_notify(VQ_RX);

    rx_used_seen++;
    return ok;
}

#else  /* !PIOS_HAS_VIRTIO_NET */

bool virtio_net_init(void) { return false; }
bool virtio_net_present(void) { return false; }
bool virtio_net_probe(void) { return false; }
u32  virtio_net_diag_code(void) { return 0; }
bool virtio_net_send(const u8 *frame, u32 len) { (void)frame; (void)len; return false; }
bool virtio_net_recv(u8 *frame, u32 *len) { (void)frame; (void)len; return false; }
void virtio_net_get_mac(u8 *mac) { if (mac) { for (u32 i = 0; i < 6; i++) mac[i] = 0; } }
bool virtio_net_link_up(void) { return false; }
void virtio_net_counters(u64 *tx_ok, u64 *tx_drop, u64 *rx_ok, u64 *rx_starve)
{
    if (tx_ok) *tx_ok = 0;
    if (tx_drop) *tx_drop = 0;
    if (rx_ok) *rx_ok = 0;
    if (rx_starve) *rx_starve = 0;
}

#endif
