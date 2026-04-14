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

/* ── helpers ──────────────────────────────────────────────────────── */

static inline void sd_write(u32 off, u32 val) { mmio_write(EMMC2_BASE + off, val); }
static inline u32  sd_read(u32 off)           { return mmio_read(EMMC2_BASE + off); }

/* Timer-based microsecond timestamp (reads ARM generic counter directly). */
static inline u64 sd_now_us(void) {
    u64 freq, cnt;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(cnt));
    if (unlikely(freq == 0)) return 0;
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
        u32 bit = start + i - 8;          /* adjust for CRC strip */
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
                          * (1u << (c_size_mult + 2))
                          * (1u << read_bl_len);
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
    sd_read_csd();

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
    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_read_block_inner(lba, buf)) {
            stats.reads++;
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
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
    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_write_block_inner(lba, buf)) {
            stats.writes++;
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
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

        sd_write(REG_INTERRUPT, INT_READ_RDY);
        pio_read_block(buf + b * SD_BLOCK_SIZE);
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
    if (count == 1) return sd_read_block(lba, buf);

    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_read_blocks_multi(lba, count, buf)) {
            stats.reads += count;
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
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

        sd_write(REG_INTERRUPT, INT_WRITE_RDY);
        pio_write_block(buf + b * SD_BLOCK_SIZE);
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
    if (count == 1) return sd_write_block(lba, buf);

    for (u32 try = 0; try < SD_MAX_RETRIES; try++) {
        if (sd_write_blocks_multi(lba, count, buf)) {
            stats.writes += count;
            return true;
        }
        stats.retries++;
        sd_recover();
    }
    return false;
}

/* ── accessors ────────────────────────────────────────────────────── */

const sd_card_t *sd_get_card_info(void) {
    return &card;
}

const sd_stats_t *sd_get_stats(void) {
    return &stats;
}
