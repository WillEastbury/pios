/*
 * sd.c - Raw SD block I/O via BCM2712 SDHCI (EMMC2)
 * No filesystem, no partitions. Pure LBA block access.
 *
 * Timeout model:  All waits use the ARM generic timer counter
 *                 (CNTVCT_EL0 / CNTFRQ_EL0) for deterministic,
 *                 CPU-speed-independent deadlines.
 *
 * Error recovery: On CMD/DATA errors the driver resets the
 *                 relevant SDHCI lines, clears interrupts, and
 *                 retries up to SD_MAX_RETRIES times.
 *
 * Buffer contract: 4-byte aligned buffers use the fast 32-bit PIO
 *                  path; unaligned buffers use a safe byte-copy
 *                  fallback via a stack-local temporary word.
 *
 * Cache coherency: PIO only (CPU ↔ REG_DATA), no DMA — no
 *                  explicit cache maintenance required.
 */

#include "sd.h"
#include "mmio.h"
#include "uart.h"
#include "fb.h"
#include "platform.h"

/* SDHCI register offsets from EMMC2_BASE */
#define REG_ARG2            0x00
#define REG_BLKSIZECNT      0x04
#define REG_ARG1            0x08
#define REG_CMDTM           0x0C
#define REG_RESP0           0x10
#define REG_RESP1           0x14
#define REG_RESP2           0x18
#define REG_RESP3           0x1C
#define REG_DATA            0x20
#define REG_STATUS          0x24
#define REG_CONTROL0        0x28
#define REG_CONTROL1        0x2C
#define REG_INTERRUPT       0x30
#define REG_IRPT_MASK       0x34
#define REG_IRPT_EN         0x38
#define REG_CONTROL2        0x3C

/* Status register bits */
#define SR_CMD_INHIBIT      (1 << 0)
#define SR_DAT_INHIBIT      (1 << 1)
#define SR_DAT_ACTIVE       (1 << 2)
#define SR_READ_AVAILABLE   (1 << 11)
#define SR_WRITE_READY      (1 << 10)

/* Control1 bits */
#define C1_SRST_HC          (1 << 24)
#define C1_SRST_CMD         (1 << 25)
#define C1_SRST_DATA        (1 << 26)
#define C1_CLK_EN           (1 << 2)
#define C1_CLK_STABLE       (1 << 1)
#define C1_CLK_INTLEN       (1 << 0)
#define C1_TOUNIT(x)        ((x) << 16)

/* Interrupt bits */
#define INT_CMD_DONE        (1 << 0)
#define INT_DATA_DONE       (1 << 1)
#define INT_WRITE_RDY       (1 << 4)
#define INT_READ_RDY        (1 << 5)
#define INT_ERROR           0xFFFF0000
#define INT_ALL             0xFFFF00FF

/* Command encoding */
#define CMD(n)              ((n) << 24)
#define RSP_NONE            0
#define RSP_136             (1 << 16)
#define RSP_48              (2 << 16)
#define RSP_48_BUSY         (3 << 16)
#define CMD_ISDATA          (1 << 21)
#define CMD_IXCHK           (1 << 20)
#define CMD_CRCCHK          (1 << 19)
#define TM_READ             (1 << 4)
#define TM_MULTI            (1 << 5)
#define TM_BLKCNT           (1 << 1)
#define TM_AUTOCMD12        (1 << 2)

/* Standard SD commands */
#define SD_CMD0     (CMD(0)  | RSP_NONE)
#define SD_CMD2     (CMD(2)  | RSP_136 | CMD_CRCCHK)
#define SD_CMD3     (CMD(3)  | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD7     (CMD(7)  | RSP_48_BUSY | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD8     (CMD(8)  | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD9     (CMD(9)  | RSP_136 | CMD_CRCCHK)
#define SD_CMD12    (CMD(12) | RSP_48_BUSY | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD16    (CMD(16) | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD17    (CMD(17) | RSP_48 | CMD_ISDATA | TM_READ | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD18    (CMD(18) | RSP_48 | CMD_ISDATA | TM_READ | TM_MULTI | TM_BLKCNT | TM_AUTOCMD12 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD24    (CMD(24) | RSP_48 | CMD_ISDATA | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD25    (CMD(25) | RSP_48 | CMD_ISDATA | TM_MULTI | TM_BLKCNT | TM_AUTOCMD12 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD55    (CMD(55) | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_ACMD6    (CMD(6)  | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_ACMD41   (CMD(41) | RSP_48)

/* Timeout policy (microseconds) */
#define SD_TIMEOUT_CMD_US       100000      /* 100 ms — command complete   */
#define SD_TIMEOUT_DATA_US      500000      /* 500 ms — data transfer      */
#define SD_TIMEOUT_INIT_US      1000000     /* 1 s    — card init / reset  */
#define SD_TIMEOUT_BUSY_US      1000000     /* 1 s    — post-write busy    */
#define SD_TIMEOUT_CLK_US       100000      /* 100 ms — clock stabilise    */

/* Retry policy */
#define SD_MAX_RETRIES          3

/* ── state ────────────────────────────────────────────────────────── */

static sd_card_t  card;
static sd_stats_t stats;
static inline u64 sd_now_us(void);

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#define QEMU_RAM_SD_BLOCKS 32768U  /* 16 MiB: enough for 10MiB reserved + RAM WALFS */
#define QEMU_VIRTIO_MMIO_BASE       0x0A000000ULL
#define QEMU_VIRTIO_MMIO_STRIDE     0x200ULL
#define QEMU_VIRTIO_MMIO_COUNT      32U
#define QEMU_VIRTIO_MMIO_MAGIC      0x74726976U
#define QEMU_VIRTIO_BLK_DEVICE_ID   2U
#define QEMU_VIRTIO_STATUS_ACK      1U
#define QEMU_VIRTIO_STATUS_DRIVER   2U
#define QEMU_VIRTIO_STATUS_DRIVER_OK 4U
#define QEMU_VIRTIO_STATUS_FEAT_OK  8U
#define QEMU_VIRTQ_DESC_F_NEXT      1U
#define QEMU_VIRTQ_DESC_F_WRITE     2U
#define QEMU_BLK_VQ_SIZE            8U
#define QEMU_BLK_TIMEOUT_US         5000000ULL
#define QEMU_VIRTIO_REG_MAGIC       0x000U
#define QEMU_VIRTIO_REG_VERSION     0x004U
#define QEMU_VIRTIO_REG_DEVICE_ID   0x008U
#define QEMU_VIRTIO_REG_DEVICE_FEAT 0x010U
#define QEMU_VIRTIO_REG_DEVICE_SEL  0x014U
#define QEMU_VIRTIO_REG_DRIVER_FEAT 0x020U
#define QEMU_VIRTIO_REG_DRIVER_SEL  0x024U
#define QEMU_VIRTIO_REG_GUEST_PAGE  0x028U
#define QEMU_VIRTIO_REG_QUEUE_SEL   0x030U
#define QEMU_VIRTIO_REG_QUEUE_NUMMAX 0x034U
#define QEMU_VIRTIO_REG_QUEUE_NUM   0x038U
#define QEMU_VIRTIO_REG_QUEUE_ALIGN 0x03CU
#define QEMU_VIRTIO_REG_QUEUE_PFN   0x040U
#define QEMU_VIRTIO_REG_QUEUE_READY 0x044U
#define QEMU_VIRTIO_REG_QUEUE_NOTIFY 0x050U
#define QEMU_VIRTIO_REG_INT_STATUS  0x060U
#define QEMU_VIRTIO_REG_INT_ACK     0x064U
#define QEMU_VIRTIO_REG_STATUS      0x070U
#define QEMU_VIRTIO_REG_Q_DESC_LOW  0x080U
#define QEMU_VIRTIO_REG_Q_DESC_HIGH 0x084U
#define QEMU_VIRTIO_REG_Q_DRV_LOW   0x090U
#define QEMU_VIRTIO_REG_Q_DRV_HIGH  0x094U
#define QEMU_VIRTIO_REG_Q_DEV_LOW   0x0A0U
#define QEMU_VIRTIO_REG_Q_DEV_HIGH  0x0A4U
#define QEMU_VIRTIO_BLK_CONFIG_CAPACITY 0x100U
#define QEMU_VIRTIO_BLK_T_IN        0U
#define QEMU_VIRTIO_BLK_T_OUT       1U
#define QEMU_VIRTIO_BLK_S_OK        0U

static u8 qemu_ram_sd[QEMU_RAM_SD_BLOCKS * SD_BLOCK_SIZE] ALIGNED(64);
static bool qemu_ram_sd_ready;
static u64 qemu_blk_base;
static bool qemu_blk_legacy;
static bool qemu_blk_ready;
static u64 qemu_blk_sectors;
static u32 qemu_blk_diag;

struct qemu_vq_desc {
    u64 addr;
    u32 len;
    u16 flags;
    u16 next;
} PACKED;

struct qemu_vq_avail {
    u16 flags;
    u16 idx;
    u16 ring[QEMU_BLK_VQ_SIZE];
    u16 used_event;
} PACKED;

struct qemu_vq_used_elem {
    u32 id;
    u32 len;
} PACKED;

struct qemu_vq_used {
    u16 flags;
    u16 idx;
    struct qemu_vq_used_elem ring[QEMU_BLK_VQ_SIZE];
    u16 avail_event;
} PACKED;

struct qemu_vq_legacy {
    struct qemu_vq_desc desc[QEMU_BLK_VQ_SIZE];
    struct qemu_vq_avail avail;
    u8 pad[4096U - (sizeof(struct qemu_vq_desc) * QEMU_BLK_VQ_SIZE) - sizeof(struct qemu_vq_avail)];
    struct qemu_vq_used used;
    u8 tail_pad[128];
} ALIGNED(4096);

struct qemu_blk_req_hdr {
    u32 type;
    u32 reserved;
    u64 sector;
} PACKED;

static struct qemu_vq_desc qemu_blk_desc[QEMU_BLK_VQ_SIZE] ALIGNED(64);
static struct qemu_vq_avail qemu_blk_avail ALIGNED(64);
static struct qemu_vq_used qemu_blk_used ALIGNED(64);
static struct qemu_vq_legacy qemu_blk_legacy_q;
static struct qemu_blk_req_hdr qemu_blk_req ALIGNED(64);
static u8 qemu_blk_status[64] ALIGNED(64);
static u16 qemu_blk_used_idx;

static void qemu_dcache_clean_range(u64 start, u64 size)
{
    if (size == 0) return;
    u64 a = start & ~63ULL;
    u64 e = (start + size + 63ULL) & ~63ULL;
    while (a < e) {
        __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
        a += 64U;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void qemu_dcache_invalidate_range(u64 start, u64 size)
{
    if (size == 0) return;
    u64 a = start & ~63ULL;
    u64 e = (start + size + 63ULL) & ~63ULL;
    while (a < e) {
        __asm__ volatile("dc ivac, %0" :: "r"(a) : "memory");
        a += 64U;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void qemu_dcache_clean_invalidate_range(u64 start, u64 size)
{
    if (size == 0) return;
    u64 a = start & ~63ULL;
    u64 e = (start + size + 63ULL) & ~63ULL;
    while (a < e) {
        __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
        a += 64U;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static bool qemu_ram_lba_ok(u32 lba, u32 count)
{
    return count <= QEMU_RAM_SD_BLOCKS && lba <= QEMU_RAM_SD_BLOCKS - count;
}

static void qemu_ram_copy_out(u8 *dst, const u8 *src, u32 n)
{
    for (u32 i = 0; i < n; i++) dst[i] = src[i];
}

static void qemu_ram_copy_in(u8 *dst, const u8 *src, u32 n)
{
    for (u32 i = 0; i < n; i++) dst[i] = src[i];
}

static void qemu_blk_notify(u32 q)
{
    __asm__ volatile("dsb sy" ::: "memory");
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_NOTIFY, q);
}

static bool qemu_blk_setup_queue(struct qemu_vq_desc *d,
                                 struct qemu_vq_avail *a,
                                 struct qemu_vq_used *u)
{
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_SEL, 0);
    if (mmio_read(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_NUMMAX) < QEMU_BLK_VQ_SIZE)
        return false;
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_NUM, QEMU_BLK_VQ_SIZE);
    if (qemu_blk_legacy) {
        mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_GUEST_PAGE, 4096);
        mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_ALIGN, 4096);
        mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_PFN, (u32)((u64)(usize)d >> 12));
        return true;
    }
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_Q_DESC_LOW, (u32)(usize)d);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_Q_DESC_HIGH, (u32)((u64)(usize)d >> 32));
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_Q_DRV_LOW, (u32)(usize)a);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_Q_DRV_HIGH, (u32)((u64)(usize)a >> 32));
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_Q_DEV_LOW, (u32)(usize)u);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_Q_DEV_HIGH, (u32)((u64)(usize)u >> 32));
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_QUEUE_READY, 1);
    return true;
}

static bool qemu_blk_probe(void)
{
    u32 blocks = 0;
    u64 selected = 0;
    for (u32 i = 0; i < QEMU_VIRTIO_MMIO_COUNT; i++) {
        u64 base = QEMU_VIRTIO_MMIO_BASE + (u64)i * QEMU_VIRTIO_MMIO_STRIDE;
        if (mmio_read(base + QEMU_VIRTIO_REG_MAGIC) == QEMU_VIRTIO_MMIO_MAGIC &&
            mmio_read(base + QEMU_VIRTIO_REG_DEVICE_ID) == QEMU_VIRTIO_BLK_DEVICE_ID) {
            blocks++;
            if (selected == 0)
                selected = base;
        }
    }
    if (blocks < 2 || selected == 0)
        return false;
    qemu_blk_base = selected;
    qemu_blk_diag = blocks;
    return true;
}

static bool qemu_blk_init(void)
{
    if (qemu_blk_ready)
        return true;
    qemu_blk_diag = 0;
    if (!qemu_blk_probe()) {
        qemu_blk_diag = 1;
        return false;
    }

    u32 ver = mmio_read(qemu_blk_base + QEMU_VIRTIO_REG_VERSION);
    if (ver != 1U && ver != 2U) {
        qemu_blk_diag = 0x200U | ver;
        return false;
    }
    qemu_blk_legacy = (ver == 1U);

    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_DEVICE_SEL, 0);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_DEVICE_SEL, 1);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_STATUS, 0);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_STATUS, QEMU_VIRTIO_STATUS_ACK);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_STATUS, QEMU_VIRTIO_STATUS_ACK | QEMU_VIRTIO_STATUS_DRIVER);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_DRIVER_SEL, 0);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_DRIVER_FEAT, 0);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_DRIVER_SEL, 1);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_DRIVER_FEAT, qemu_blk_legacy ? 0U : 1U);
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_STATUS,
               QEMU_VIRTIO_STATUS_ACK | QEMU_VIRTIO_STATUS_DRIVER | QEMU_VIRTIO_STATUS_FEAT_OK);
    if ((mmio_read(qemu_blk_base + QEMU_VIRTIO_REG_STATUS) & QEMU_VIRTIO_STATUS_FEAT_OK) == 0) {
        qemu_blk_diag = 3;
        return false;
    }

    memset(qemu_blk_desc, 0, sizeof(qemu_blk_desc));
    memset(&qemu_blk_avail, 0, sizeof(qemu_blk_avail));
    memset(&qemu_blk_used, 0, sizeof(qemu_blk_used));
    memset(&qemu_blk_legacy_q, 0, sizeof(qemu_blk_legacy_q));
    qemu_blk_used_idx = 0;

    struct qemu_vq_desc *d = qemu_blk_legacy ? qemu_blk_legacy_q.desc : qemu_blk_desc;
    struct qemu_vq_avail *a = qemu_blk_legacy ? &qemu_blk_legacy_q.avail : &qemu_blk_avail;
    struct qemu_vq_used *u = qemu_blk_legacy ? &qemu_blk_legacy_q.used : &qemu_blk_used;
    if (!qemu_blk_setup_queue(d, a, u)) {
        qemu_blk_diag = 4;
        return false;
    }

    u32 cap_lo = mmio_read(qemu_blk_base + QEMU_VIRTIO_BLK_CONFIG_CAPACITY);
    u32 cap_hi = mmio_read(qemu_blk_base + QEMU_VIRTIO_BLK_CONFIG_CAPACITY + 4U);
    qemu_blk_sectors = ((u64)cap_hi << 32) | cap_lo;
    if (qemu_blk_sectors == 0) {
        qemu_blk_diag = 5;
        return false;
    }

    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_STATUS,
               QEMU_VIRTIO_STATUS_ACK | QEMU_VIRTIO_STATUS_DRIVER |
               QEMU_VIRTIO_STATUS_FEAT_OK | QEMU_VIRTIO_STATUS_DRIVER_OK);
    qemu_blk_ready = true;
    qemu_blk_diag = 10;
    return true;
}

static bool qemu_blk_lba_ok(u32 lba, u32 count)
{
    return count != 0 && (u64)count <= qemu_blk_sectors &&
           (u64)lba <= qemu_blk_sectors - (u64)count;
}

static bool qemu_blk_rw(u32 lba, u32 count, void *buf, bool write)
{
    if (!qemu_blk_ready || !buf || !qemu_blk_lba_ok(lba, count)) {
        qemu_blk_diag = 0x20;
        return false;
    }
    u64 bytes = (u64)count * (u64)SD_BLOCK_SIZE;
    if (bytes > 0xFFFFFFFFULL) {
        qemu_blk_diag = 0x21;
        return false;
    }

    struct qemu_vq_desc *d = qemu_blk_legacy ? qemu_blk_legacy_q.desc : qemu_blk_desc;
    struct qemu_vq_avail *a = qemu_blk_legacy ? &qemu_blk_legacy_q.avail : &qemu_blk_avail;
    struct qemu_vq_used *u = qemu_blk_legacy ? &qemu_blk_legacy_q.used : &qemu_blk_used;

    qemu_blk_req.type = write ? QEMU_VIRTIO_BLK_T_OUT : QEMU_VIRTIO_BLK_T_IN;
    qemu_blk_req.reserved = 0;
    qemu_blk_req.sector = lba;
    qemu_blk_status[0] = 0xFFU;

    d[0].addr = (u64)(usize)&qemu_blk_req;
    d[0].len = sizeof(qemu_blk_req);
    d[0].flags = QEMU_VIRTQ_DESC_F_NEXT;
    d[0].next = 1;
    d[1].addr = (u64)(usize)buf;
    d[1].len = (u32)bytes;
    d[1].flags = QEMU_VIRTQ_DESC_F_NEXT | (write ? 0U : QEMU_VIRTQ_DESC_F_WRITE);
    d[1].next = 2;
    d[2].addr = (u64)(usize)qemu_blk_status;
    d[2].len = 1;
    d[2].flags = QEMU_VIRTQ_DESC_F_WRITE;
    d[2].next = 0;

    a->ring[a->idx % QEMU_BLK_VQ_SIZE] = 0;
    a->idx++;
    qemu_dcache_clean_range((u64)(usize)&qemu_blk_req, sizeof(qemu_blk_req));
    qemu_dcache_clean_range((u64)(usize)d, sizeof(d[0]) * 3U);
    qemu_dcache_clean_range((u64)(usize)a, sizeof(*a));
    qemu_dcache_clean_invalidate_range((u64)(usize)qemu_blk_status, sizeof(qemu_blk_status));
    qemu_dcache_clean_invalidate_range((u64)(usize)u, sizeof(*u));
    if (write)
        qemu_dcache_clean_range((u64)(usize)buf, bytes);
    else
        qemu_dcache_clean_invalidate_range((u64)(usize)buf, bytes);
    qemu_blk_notify(0);

    u64 deadline = sd_now_us() + QEMU_BLK_TIMEOUT_US;
    while (sd_now_us() < deadline) {
        qemu_dcache_invalidate_range((u64)(usize)u, sizeof(*u));
        if (u->idx != qemu_blk_used_idx)
            break;
    }
    if (u->idx == qemu_blk_used_idx) {
        stats.data_timeouts++;
        qemu_blk_diag = 0x22;
        return false;
    }
    qemu_blk_used_idx = u->idx;
    mmio_write(qemu_blk_base + QEMU_VIRTIO_REG_INT_ACK,
               mmio_read(qemu_blk_base + QEMU_VIRTIO_REG_INT_STATUS));
    __asm__ volatile("dsb sy" ::: "memory");
    qemu_dcache_invalidate_range((u64)(usize)qemu_blk_status, sizeof(qemu_blk_status));
    if (!write)
        qemu_dcache_invalidate_range((u64)(usize)buf, bytes);
    if (qemu_blk_status[0] != QEMU_VIRTIO_BLK_S_OK)
        qemu_blk_diag = 0x30U | qemu_blk_status[0];
    return qemu_blk_status[0] == QEMU_VIRTIO_BLK_S_OK;
}

const char *sd_qemu_backend_name(void)
{
    return qemu_blk_ready ? "virtio-blk" : "ram";
}

bool sd_qemu_virtio_blk_ready(void)
{
    return qemu_blk_ready;
}

u32 sd_qemu_virtio_blk_diag(void)
{
    return qemu_blk_diag;
}
#endif

/* ── helpers ──────────────────────────────────────────────────────── */

static inline void sd_write(u32 off, u32 val) { mmio_write(EMMC2_BASE + off, val); }
static inline u32  sd_read(u32 off)           { return mmio_read(EMMC2_BASE + off); }

static u32 sd_mbps_x1000(u32 blocks, u64 elapsed_us)
{
    if (blocks == 0 || elapsed_us == 0)
        return 0;
    u64 bytes = (u64)blocks * (u64)SD_BLOCK_SIZE;
    u64 v = (bytes * 8000ULL) / elapsed_us;
    return v > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (u32)v;
}

static void sd_record_read_rate(u32 blocks, u64 start_us)
{
    u64 end = sd_now_us();
    u32 rate = sd_mbps_x1000(blocks, end > start_us ? end - start_us : 1);
    stats.read_last_mbps_x1000 = rate;
    if (rate > stats.read_peak_mbps_x1000)
        stats.read_peak_mbps_x1000 = rate;
}

static void sd_record_write_rate(u32 blocks, u64 start_us)
{
    u64 end = sd_now_us();
    u32 rate = sd_mbps_x1000(blocks, end > start_us ? end - start_us : 1);
    stats.write_last_mbps_x1000 = rate;
    if (rate > stats.write_peak_mbps_x1000)
        stats.write_peak_mbps_x1000 = rate;
}

/* Timer-based microsecond timestamp (reads ARM generic counter directly).
 * Returns ~UINT64_MAX when CNTFRQ_EL0 == 0 so deadlines never expire
 * spuriously — callers will block until the condition is met or another
 * exit path fires (error interrupt, etc.). */
static inline u64 sd_now_us(void) {
    u64 freq, cnt;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(cnt));
    if (unlikely(freq == 0)) return ~(u64)0;
    return cnt / (freq / 1000000ULL);
}

/* ── line resets ──────────────────────────────────────────────────── */

static void sd_reset_cmd_line(void) {
    sd_write(REG_CONTROL1, sd_read(REG_CONTROL1) | C1_SRST_CMD);
    u64 deadline = sd_now_us() + SD_TIMEOUT_CMD_US;
    while ((sd_read(REG_CONTROL1) & C1_SRST_CMD) && sd_now_us() < deadline)
        ;
    sd_write(REG_INTERRUPT, INT_ALL);
}

static void sd_reset_data_line(void) {
    sd_write(REG_CONTROL1, sd_read(REG_CONTROL1) | C1_SRST_DATA);
    u64 deadline = sd_now_us() + SD_TIMEOUT_CMD_US;
    while ((sd_read(REG_CONTROL1) & C1_SRST_DATA) && sd_now_us() < deadline)
        ;
    sd_write(REG_INTERRUPT, INT_ALL);
}

/* Full error recovery: reset both lines + clear interrupts. */
static void sd_recover(void) {
    sd_write(REG_CONTROL1,
             sd_read(REG_CONTROL1) | C1_SRST_CMD | C1_SRST_DATA);
    u64 deadline = sd_now_us() + SD_TIMEOUT_CMD_US;
    while ((sd_read(REG_CONTROL1) & (C1_SRST_CMD | C1_SRST_DATA))
           && sd_now_us() < deadline)
        ;
    sd_write(REG_INTERRUPT, INT_ALL);
}

/* ── inhibit waits (timer-based) ──────────────────────────────────── */

static bool sd_wait_cmd(void) {
    u64 deadline = sd_now_us() + SD_TIMEOUT_CMD_US;
    while ((sd_read(REG_STATUS) & SR_CMD_INHIBIT) && sd_now_us() < deadline)
        ;
    if (sd_read(REG_STATUS) & SR_CMD_INHIBIT) {
        uart_puts("[sd] wait_cmd timeout STATUS=");
        uart_hex(sd_read(REG_STATUS));
        uart_puts("\n");
        stats.cmd_timeouts++;
        sd_reset_cmd_line();
        return false;
    }
    return true;
}

static bool sd_wait_data(void) {
    u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    while ((sd_read(REG_STATUS) & SR_DAT_INHIBIT) && sd_now_us() < deadline)
        ;
    if (sd_read(REG_STATUS) & SR_DAT_INHIBIT) {
        uart_puts("[sd] wait_data timeout STATUS=");
        uart_hex(sd_read(REG_STATUS));
        uart_puts("\n");
        stats.data_timeouts++;
        sd_reset_data_line();
        return false;
    }
    return true;
}

/* ── command issue ────────────────────────────────────────────────── */

static bool sd_send_cmd(u32 cmd, u32 arg, u32 *resp) {
    if (!sd_wait_cmd())
        return false;

    sd_write(REG_INTERRUPT, INT_ALL);
    sd_write(REG_ARG1, arg);
    sd_write(REG_CMDTM, cmd);

    u64 deadline = sd_now_us() + SD_TIMEOUT_CMD_US;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            uart_puts("[sd] CMD error intr=");
            uart_hex(intr);
            uart_puts(" status=");
            uart_hex(sd_read(REG_STATUS));
            uart_puts("\n");
            sd_write(REG_INTERRUPT, INT_ERROR);
            stats.errors++;
            sd_reset_cmd_line();
            return false;
        }
    } while (!(intr & INT_CMD_DONE) && sd_now_us() < deadline);

    if (!(intr & INT_CMD_DONE)) {
        uart_puts("[sd] CMD timeout intr=");
        uart_hex(intr);
        uart_puts("\n");
        stats.cmd_timeouts++;
        sd_reset_cmd_line();
        return false;
    }

    sd_write(REG_INTERRUPT, INT_CMD_DONE);

    if (resp) {
        resp[0] = sd_read(REG_RESP0);
        resp[1] = sd_read(REG_RESP1);
        resp[2] = sd_read(REG_RESP2);
        resp[3] = sd_read(REG_RESP3);
    }
    return true;
}

static bool sd_send_acmd(u32 cmd, u32 arg, u32 *resp) {
    u32 r[4];
    if (!sd_send_cmd(SD_CMD55, card.rca << 16, r))
        return false;
    return sd_send_cmd(cmd, arg, resp);
}

/* ── post-write busy wait (DAT0) ─────────────────────────────────── */

static bool sd_wait_busy(void) {
    u64 deadline = sd_now_us() + SD_TIMEOUT_BUSY_US;
    while ((sd_read(REG_STATUS) & SR_DAT_INHIBIT) && sd_now_us() < deadline)
        ;
    return !(sd_read(REG_STATUS) & SR_DAT_INHIBIT);
}

/* ── clock configuration ──────────────────────────────────────────── */

static void sd_set_clock(u32 freq_khz) {
    u32 c1 = sd_read(REG_CONTROL1);
    c1 &= ~C1_CLK_EN;
    sd_write(REG_CONTROL1, c1);
    delay_cycles(1000);

    /* base clock ~200 MHz on Pi 5 */
    u32 base_khz = 200000;
    u32 div = base_khz / freq_khz;
    if (base_khz / div > freq_khz) div++;
    div = (div >> 1);
    if (div > 0x3FF) div = 0x3FF;

    u32 divider = ((div & 0xFF) << 8) | ((div >> 8) << 6);

    c1 = (c1 & 0xFFFF001F) | divider | C1_CLK_INTLEN | C1_TOUNIT(0xE);
    sd_write(REG_CONTROL1, c1);
    delay_cycles(1000);

    u64 deadline = sd_now_us() + SD_TIMEOUT_CLK_US;
    while (!(sd_read(REG_CONTROL1) & C1_CLK_STABLE) && sd_now_us() < deadline)
        ;

    c1 = sd_read(REG_CONTROL1);
    c1 |= C1_CLK_EN;
    sd_write(REG_CONTROL1, c1);
    delay_cycles(1000);
}

/* ── CSD parsing ──────────────────────────────────────────────────── */

/*
 * Extract a bit-field from the 128-bit CSD register.
 * SDHCI R2 response stores CSD[127:8] in resp[0..3] (CRC stripped),
 * so CSD bit N maps to response bit (N − 8).
 *   resp[0] = CSD[39:8], resp[1] = CSD[71:40],
 *   resp[2] = CSD[103:72], resp[3] = CSD[127:104] (top 8 bits = 0).
 */
static u32 csd_extract(const u32 *resp, u32 start, u32 width) {
    u32 val = 0;
    for (u32 i = 0; i < width; i++) {
        u32 csd_bit = start + i;
        if (csd_bit < 8) continue;        /* CRC zone — stripped by SDHCI */
        u32 bit = csd_bit - 8;            /* adjust for CRC strip */
        u32 word = bit / 32;
        u32 pos  = bit % 32;
        if (word < 4 && (resp[word] & (1u << pos)))
            val |= (1u << i);
    }
    return val;
}

static bool sd_read_csd(void) {
    u32 csd[4];
    if (!sd_send_cmd(SD_CMD9, card.rca << 16, csd)) {
        uart_puts("[sd] CMD9 (CSD) failed\n");
        return false;
    }

    u32 csd_ver = csd_extract(csd, 126, 2);

    if (csd_ver == 0) {
        /* CSD v1 — SDSC */
        u32 read_bl_len = csd_extract(csd, 80, 4);
        u32 c_size      = csd_extract(csd, 62, 12);
        u32 c_size_mult = csd_extract(csd, 47, 3);
        card.capacity   = (u64)(c_size + 1)
                          * (1ULL << (c_size_mult + 2))
                          * (1ULL << read_bl_len);
    } else if (csd_ver == 1) {
        /* CSD v2 — SDHC / SDXC */
        u32 c_size    = csd_extract(csd, 48, 22);
        card.capacity = (u64)(c_size + 1) * 512ULL * 1024ULL;
    } else {
        uart_puts("[sd] Unknown CSD version ");
        uart_hex(csd_ver);
        uart_puts("\n");
        card.capacity = 0;
        return false;
    }

    uart_puts("[sd] capacity=");
    uart_hex(card.capacity);
    uart_puts(" bytes\n");
    return true;
}

/* ── initialisation ───────────────────────────────────────────────── */

bool sd_init(void) {
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    card.type = 2;
    card.rca = 1;
    memset((void *)&stats, 0, sizeof(stats));
    if (qemu_blk_init()) {
        card.capacity = qemu_blk_sectors * (u64)SD_BLOCK_SIZE;
        uart_puts("[sd] qemu virtio-blk backend cap=");
        uart_hex(card.capacity);
        uart_puts("\n");
        return true;
    }
    if (qemu_blk_diag != 1U) {
        uart_puts("[sd] qemu virtio-blk unavailable diag=");
        uart_hex(qemu_blk_diag);
        uart_puts("; falling back to RAM\n");
    }
    card.capacity = (u64)QEMU_RAM_SD_BLOCKS * SD_BLOCK_SIZE;
    if (!qemu_ram_sd_ready) {
        memset(qemu_ram_sd, 0, sizeof(qemu_ram_sd));
        qemu_ram_sd_ready = true;
    }
    uart_puts("[sd] qemu RAM block backend cap=");
    uart_hex(card.capacity);
    uart_puts("\n");
    return true;
#else
    fb_puts("  [sd] reset SDHCI...\n");
    uart_puts("[sd] init\n");

    card.type     = 0;
    card.rca      = 0;
    card.capacity = 0;
    memset((void *)&stats, 0, sizeof(stats));

    /* Full HC reset */
    sd_write(REG_CONTROL1, C1_SRST_HC);
    u64 deadline = sd_now_us() + SD_TIMEOUT_INIT_US;
    while ((sd_read(REG_CONTROL1) & C1_SRST_HC) && sd_now_us() < deadline)
        ;
    if (sd_read(REG_CONTROL1) & C1_SRST_HC) {
        fb_puts("  [sd] reset timeout\n");
        uart_puts("[sd] reset timeout\n");
        return false;
    }
    fb_puts("  [sd] reset OK\n");

    /* Power on: SD Bus Power = 1, voltage = 3.3V */
    sd_write(REG_CONTROL0, 0x0F00);
    delay_cycles(10000);

    sd_write(REG_IRPT_MASK, INT_ALL);
    sd_write(REG_IRPT_EN, INT_ALL);

    /* Identification clock (400 kHz) */
    fb_puts("  [sd] clock 400kHz...\n");
    sd_set_clock(400);

    /* Let card see 74+ clock cycles */
    delay_cycles(500000);

    /* CMD0: GO_IDLE */
    fb_puts("  [sd] CMD0...\n");
    {
        sd_send_cmd(SD_CMD0, 0, NULL);
        sd_reset_cmd_line();
        delay_cycles(100000);
    }

    /* Ensure CMD inhibit is clear */
    if (sd_read(REG_STATUS) & SR_CMD_INHIBIT) {
        sd_recover();
        delay_cycles(100000);
    }

    if (!sd_wait_cmd()) {
        uart_puts("[sd] CMD inhibit stuck\n");
        return false;
    }

    /* CMD8: SEND_IF_COND */
    fb_puts("  [sd] CMD8...\n");
    u32 resp[4];
    bool sd_v2 = sd_send_cmd(SD_CMD8, 0x1AA, resp);
    if (sd_v2 && (resp[0] & 0xFFF) != 0x1AA) {
        fb_puts("  [sd] CMD8 bad resp\n");
        return false;
    }
    fb_printf("  [sd] v%d\n", sd_v2 ? 2 : 1);

    /* ACMD41: poll until card ready */
    fb_puts("  [sd] ACMD41...\n");
    u32 acmd41_arg = 0x00FF8000;          /* 3.2-3.4V window */
    if (sd_v2)
        acmd41_arg |= (1 << 30);         /* HCS: host supports SDHC */

    deadline = sd_now_us() + SD_TIMEOUT_INIT_US;
    do {
        if (!sd_send_acmd(SD_ACMD41, acmd41_arg, resp)) {
            uart_puts("[sd] ACMD41 failed\n");
            return false;
        }
        delay_cycles(100000);
    } while (!(resp[0] & (1u << 31)) && sd_now_us() < deadline);

    if (!(resp[0] & (1u << 31))) {
        uart_puts("[sd] Card init timeout\n");
        return false;
    }

    card.type = (resp[0] & (1 << 30)) ? 2 : 1;     /* SDHC or SDSC */

    /* CMD2: ALL_SEND_CID */
    if (!sd_send_cmd(SD_CMD2, 0, resp)) {
        uart_puts("[sd] CMD2 failed\n");
        return false;
    }

    /* CMD3: SEND_RELATIVE_ADDR */
    if (!sd_send_cmd(SD_CMD3, 0, resp)) {
        uart_puts("[sd] CMD3 failed\n");
        return false;
    }
    card.rca = resp[0] >> 16;

    /* CMD9: SEND_CSD — parse capacity (must be before CMD7 selects card) */
    if (!sd_read_csd())
        uart_puts("[sd] WARNING: CSD parse failed, capacity unknown\n");

    /* CMD7: SELECT_CARD */
    if (!sd_send_cmd(SD_CMD7, card.rca << 16, resp)) {
        uart_puts("[sd] CMD7 failed\n");
        return false;
    }

    /* Switch to high-speed clock (25 MHz) */
    sd_set_clock(25000);

    /* ACMD6: SET_BUS_WIDTH to 4-bit */
    if (sd_send_acmd(SD_ACMD6, 2, resp)) {
        u32 c0 = sd_read(REG_CONTROL0);
        c0 |= (1 << 1);                  /* 4-bit mode */
        sd_write(REG_CONTROL0, c0);
    }

    /* CMD16: SET_BLOCKLEN to 512 (SDSC only; SDHC fixed at 512) */
    if (card.type == 1) {
        sd_send_cmd(SD_CMD16, 512, resp);
    }

    uart_puts("[sd] Card ready: ");
    uart_puts(card.type == 2 ? "SDHC/SDXC" : "SDSC");
    uart_puts(" RCA=");
    uart_hex(card.rca);
    uart_puts(" cap=");
    uart_hex(card.capacity);
    uart_puts("\n");

    return true;
#endif
}

/* ── PIO helpers (aligned / unaligned) ────────────────────────────── */

static void pio_read_block(u8 *buf) {
    if (((usize)buf & 3) == 0) {
        u32 *p = (u32 *)buf;
        for (int i = 0; i < 128; i++)
            p[i] = sd_read(REG_DATA);
    } else {
        for (int i = 0; i < 128; i++) {
            u32 w = sd_read(REG_DATA);
            buf[0] = (u8)(w);
            buf[1] = (u8)(w >> 8);
            buf[2] = (u8)(w >> 16);
            buf[3] = (u8)(w >> 24);
            buf += 4;
        }
    }
}

static void pio_write_block(const u8 *buf) {
    if (((usize)buf & 3) == 0) {
        const u32 *p = (const u32 *)buf;
        for (int i = 0; i < 128; i++)
            sd_write(REG_DATA, p[i]);
    } else {
        for (int i = 0; i < 128; i++) {
            u32 w = (u32)buf[0]
                  | ((u32)buf[1] << 8)
                  | ((u32)buf[2] << 16)
                  | ((u32)buf[3] << 24);
            sd_write(REG_DATA, w);
            buf += 4;
        }
    }
}

/* ── single-block I/O (with retry) ───────────────────────────────── */

static bool sd_read_block_inner(u32 lba, u8 *buf) {
    u32 addr = (card.type == 2) ? lba : lba * 512;

    if (!sd_wait_data())
        return false;

    sd_write(REG_BLKSIZECNT, (1u << 16) | 512);
    sd_write(REG_INTERRUPT, INT_ALL);

    u32 resp[4];
    if (!sd_send_cmd(SD_CMD17, addr, resp))
        return false;

    /* Wait for read ready */
    u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sd_write(REG_INTERRUPT, INT_ERROR);
            stats.errors++;
            sd_reset_data_line();
            return false;
        }
    } while (!(intr & INT_READ_RDY) && sd_now_us() < deadline);

    if (!(intr & INT_READ_RDY)) {
        stats.data_timeouts++;
        sd_reset_data_line();
        return false;
    }

    pio_read_block(buf);

    /* Wait for transfer complete */
    deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    do {
        intr = sd_read(REG_INTERRUPT);
    } while (!(intr & INT_DATA_DONE) && sd_now_us() < deadline);

    sd_write(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sd_read_block(u32 lba, u8 *buf) {
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    u64 start = sd_now_us();
    if (qemu_blk_ready) {
        if (!buf || !qemu_blk_rw(lba, 1, buf, false)) {
            stats.errors++;
            return false;
        }
        stats.reads++;
        sd_record_read_rate(1, start);
        return true;
    }
    if (!buf || !qemu_ram_lba_ok(lba, 1)) {
        stats.errors++;
        return false;
    }
    qemu_ram_copy_out(buf, qemu_ram_sd + ((usize)lba * SD_BLOCK_SIZE), SD_BLOCK_SIZE);
    stats.reads++;
    sd_record_read_rate(1, start);
    return true;
#else
    u64 start = sd_now_us();
    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_read_block_inner(lba, buf)) {
            stats.reads++;
            sd_record_read_rate(1, start);
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    /* Last-ditch: full controller + card re-init.
     * Helps recover from DAT_INHIBIT-stuck state caused by abandoned
     * transfers or external bus disturbances (e.g. SDIO2/WiFi init). */
    uart_puts("[sd] read retries exhausted lba=");
    uart_hex(lba);
    uart_puts(" — full reinit\n");
    if (sd_init()) {
        if (sd_read_block_inner(lba, buf)) {
            stats.reads++;
            sd_record_read_rate(1, start);
            return true;
        }
    }
    return false;
#endif
}

static bool sd_write_block_inner(u32 lba, const u8 *buf) {
    u32 addr = (card.type == 2) ? lba : lba * 512;

    if (!sd_wait_data())
        return false;

    sd_write(REG_BLKSIZECNT, (1u << 16) | 512);
    sd_write(REG_INTERRUPT, INT_ALL);

    u32 resp[4];
    if (!sd_send_cmd(SD_CMD24, addr, resp))
        return false;

    /* Wait for write ready */
    u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sd_write(REG_INTERRUPT, INT_ERROR);
            stats.errors++;
            sd_reset_data_line();
            return false;
        }
    } while (!(intr & INT_WRITE_RDY) && sd_now_us() < deadline);

    if (!(intr & INT_WRITE_RDY)) {
        stats.data_timeouts++;
        sd_reset_data_line();
        return false;
    }

    pio_write_block(buf);

    /* Wait for transfer complete */
    deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    do {
        intr = sd_read(REG_INTERRUPT);
    } while (!(intr & INT_DATA_DONE) && sd_now_us() < deadline);

    sd_write(REG_INTERRUPT, INT_DATA_DONE);

    /* Wait for card not-busy (DAT0 released) */
    if (!sd_wait_busy()) {
        stats.data_timeouts++;
        return false;
    }
    return true;
}

bool sd_write_block(u32 lba, const u8 *buf) {
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    u64 start = sd_now_us();
    if (qemu_blk_ready) {
        if (!buf || !qemu_blk_rw(lba, 1, (void *)(usize)buf, true)) {
            stats.errors++;
            return false;
        }
        stats.writes++;
        sd_record_write_rate(1, start);
        return true;
    }
    if (!buf || !qemu_ram_lba_ok(lba, 1)) {
        stats.errors++;
        return false;
    }
    qemu_ram_copy_in(qemu_ram_sd + ((usize)lba * SD_BLOCK_SIZE), buf, SD_BLOCK_SIZE);
    stats.writes++;
    sd_record_write_rate(1, start);
    return true;
#else
    u64 start = sd_now_us();
    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_write_block_inner(lba, buf)) {
            stats.writes++;
            sd_record_write_rate(1, start);
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
#endif
}

/* ── multi-block I/O (CMD18 / CMD25) ─────────────────────────────── */

static bool sd_read_blocks_multi(u32 lba, u32 count, u8 *buf) {
    u32 addr = (card.type == 2) ? lba : lba * 512;

    if (!sd_wait_data())
        return false;

    sd_write(REG_BLKSIZECNT, (count << 16) | 512);
    sd_write(REG_INTERRUPT, INT_ALL);

    u32 resp[4];
    if (!sd_send_cmd(SD_CMD18, addr, resp))
        return false;

    for (u32 b = 0; b < count; b++) {
        u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
        u32 intr;
        do {
            intr = sd_read(REG_INTERRUPT);
            if (intr & INT_ERROR) {
                sd_write(REG_INTERRUPT, INT_ERROR);
                stats.errors++;
                sd_recover();
                return false;
            }
        } while (!(intr & INT_READ_RDY) && sd_now_us() < deadline);

        if (!(intr & INT_READ_RDY)) {
            stats.data_timeouts++;
            sd_recover();
            return false;
        }

        pio_read_block(buf + b * SD_BLOCK_SIZE);
        sd_write(REG_INTERRUPT, INT_READ_RDY);
    }

    /* Wait for transfer complete (auto-CMD12 sent by controller) */
    u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
    } while (!(intr & INT_DATA_DONE) && sd_now_us() < deadline);

    sd_write(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sd_read_blocks(u32 lba, u32 count, u8 *buf) {
    if (count == 0) return true;
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    u64 start = sd_now_us();
    if (qemu_blk_ready) {
        if (!buf || !qemu_blk_rw(lba, count, buf, false)) {
            stats.errors++;
            return false;
        }
        stats.reads += count;
        sd_record_read_rate(count, start);
        return true;
    }
    if (!buf || !qemu_ram_lba_ok(lba, count)) {
        stats.errors++;
        return false;
    }
    qemu_ram_copy_out(buf, qemu_ram_sd + ((usize)lba * SD_BLOCK_SIZE),
                      count * SD_BLOCK_SIZE);
    stats.reads += count;
    sd_record_read_rate(count, start);
    return true;
#else
    if (count == 1) return sd_read_block(lba, buf);

    u64 start = sd_now_us();
    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_read_blocks_multi(lba, count, buf)) {
            stats.reads += count;
            sd_record_read_rate(count, start);
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
#endif
}

static bool sd_write_blocks_multi(u32 lba, u32 count, const u8 *buf) {
    u32 addr = (card.type == 2) ? lba : lba * 512;

    if (!sd_wait_data())
        return false;

    sd_write(REG_BLKSIZECNT, (count << 16) | 512);
    sd_write(REG_INTERRUPT, INT_ALL);

    u32 resp[4];
    if (!sd_send_cmd(SD_CMD25, addr, resp))
        return false;

    for (u32 b = 0; b < count; b++) {
        u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
        u32 intr;
        do {
            intr = sd_read(REG_INTERRUPT);
            if (intr & INT_ERROR) {
                sd_write(REG_INTERRUPT, INT_ERROR);
                stats.errors++;
                sd_recover();
                return false;
            }
        } while (!(intr & INT_WRITE_RDY) && sd_now_us() < deadline);

        if (!(intr & INT_WRITE_RDY)) {
            stats.data_timeouts++;
            sd_recover();
            return false;
        }

        pio_write_block(buf + b * SD_BLOCK_SIZE);
        sd_write(REG_INTERRUPT, INT_WRITE_RDY);
    }

    /* Wait for transfer complete (auto-CMD12 sent by controller) */
    u64 deadline = sd_now_us() + SD_TIMEOUT_DATA_US;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
    } while (!(intr & INT_DATA_DONE) && sd_now_us() < deadline);

    sd_write(REG_INTERRUPT, INT_DATA_DONE);

    /* Wait for card not-busy (DAT0 released) */
    if (!sd_wait_busy()) {
        stats.data_timeouts++;
        return false;
    }
    return true;
}

bool sd_write_blocks(u32 lba, u32 count, const u8 *buf) {
    if (count == 0) return true;
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    u64 start = sd_now_us();
    if (qemu_blk_ready) {
        if (!buf || !qemu_blk_rw(lba, count, (void *)(usize)buf, true)) {
            stats.errors++;
            return false;
        }
        stats.writes += count;
        sd_record_write_rate(count, start);
        return true;
    }
    if (!buf || !qemu_ram_lba_ok(lba, count)) {
        stats.errors++;
        return false;
    }
    qemu_ram_copy_in(qemu_ram_sd + ((usize)lba * SD_BLOCK_SIZE), buf,
                     count * SD_BLOCK_SIZE);
    stats.writes += count;
    sd_record_write_rate(count, start);
    return true;
#else
    if (count == 1) return sd_write_block(lba, buf);

    u64 start = sd_now_us();
    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_write_blocks_multi(lba, count, buf)) {
            stats.writes += count;
            sd_record_write_rate(count, start);
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
#endif
}

/* ── accessors ────────────────────────────────────────────────────── */

const sd_card_t *sd_get_card_info(void) {
    return &card;
}

const sd_stats_t *sd_get_stats(void) {
    return &stats;
}
