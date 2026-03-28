/*
 * sd.c - Raw SD block I/O via BCM2712 SDHCI (EMMC2)
 * No filesystem, no partitions. Pure LBA block access.
 */

#include "sd.h"
#include "mmio.h"
#include "uart.h"

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
#define SR_READ_AVAILABLE   (1 << 11)
#define SR_WRITE_READY      (1 << 10)

/* Control1 bits */
#define C1_SRST_HC          (1 << 24)
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
#define SD_CMD12    (CMD(12) | RSP_48_BUSY | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD16    (CMD(16) | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD17    (CMD(17) | RSP_48 | CMD_ISDATA | TM_READ | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD18    (CMD(18) | RSP_48 | CMD_ISDATA | TM_READ | TM_MULTI | TM_BLKCNT | TM_AUTOCMD12 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD24    (CMD(24) | RSP_48 | CMD_ISDATA | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD25    (CMD(25) | RSP_48 | CMD_ISDATA | TM_MULTI | TM_BLKCNT | TM_AUTOCMD12 | CMD_IXCHK | CMD_CRCCHK)
#define SD_CMD55    (CMD(55) | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_ACMD6    (CMD(6)  | RSP_48 | CMD_IXCHK | CMD_CRCCHK)
#define SD_ACMD41   (CMD(41) | RSP_48)

static sd_card_t card;

static inline void sd_write(u32 off, u32 val) { mmio_write(EMMC2_BASE + off, val); }
static inline u32  sd_read(u32 off)           { return mmio_read(EMMC2_BASE + off); }

static bool sd_wait_cmd(void) {
    u32 timeout = 1000000;
    while ((sd_read(REG_STATUS) & SR_CMD_INHIBIT) && timeout--)
        delay_cycles(10);
    return timeout > 0;
}

static bool sd_wait_data(void) {
    u32 timeout = 1000000;
    while ((sd_read(REG_STATUS) & SR_DAT_INHIBIT) && timeout--)
        delay_cycles(10);
    return timeout > 0;
}

static bool sd_send_cmd(u32 cmd, u32 arg, u32 *resp) {
    if (!sd_wait_cmd())
        return false;

    /* Clear interrupts */
    sd_write(REG_INTERRUPT, INT_ALL);

    sd_write(REG_ARG1, arg);
    sd_write(REG_CMDTM, cmd);

    /* Wait for command complete or error */
    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sd_write(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        delay_cycles(10);
    } while (!(intr & INT_CMD_DONE) && timeout--);

    if (!timeout)
        return false;

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

static void sd_set_clock(u32 freq_khz) {
    /* Disable clock */
    u32 c1 = sd_read(REG_CONTROL1);
    c1 &= ~C1_CLK_EN;
    sd_write(REG_CONTROL1, c1);
    delay_cycles(1000);

    /* Calculate divider: base clock ~200MHz on Pi 5 */
    u32 base_khz = 200000;
    u32 div = base_khz / freq_khz;
    if (base_khz / div > freq_khz) div++;
    div = (div >> 1);
    if (div > 0x3FF) div = 0x3FF;

    u32 divider = ((div & 0xFF) << 8) | ((div >> 8) << 6);

    c1 = (c1 & 0xFFFF001F) | divider | C1_CLK_INTLEN | C1_TOUNIT(0xE);
    sd_write(REG_CONTROL1, c1);
    delay_cycles(1000);

    /* Wait for clock stable */
    u32 timeout = 100000;
    while (!(sd_read(REG_CONTROL1) & C1_CLK_STABLE) && timeout--)
        delay_cycles(10);

    /* Enable clock */
    c1 = sd_read(REG_CONTROL1);
    c1 |= C1_CLK_EN;
    sd_write(REG_CONTROL1, c1);
    delay_cycles(1000);
}

bool sd_init(void) {
    uart_puts("[sd] Initializing SDHCI...\n");

    card.type = 0;
    card.rca  = 0;

    /* Reset controller */
    sd_write(REG_CONTROL1, sd_read(REG_CONTROL1) | C1_SRST_HC);
    u32 timeout = 100000;
    while ((sd_read(REG_CONTROL1) & C1_SRST_HC) && timeout--)
        delay_cycles(10);
    if (!timeout) {
        uart_puts("[sd] Reset timeout\n");
        return false;
    }

    /* Enable interrupts we care about */
    sd_write(REG_IRPT_MASK, INT_ALL);
    sd_write(REG_IRPT_EN, 0);

    /* Set slow clock for identification (400 KHz) */
    sd_set_clock(400);

    /* CMD0: GO_IDLE */
    if (!sd_send_cmd(SD_CMD0, 0, NULL)) {
        uart_puts("[sd] CMD0 failed\n");
        return false;
    }

    /* CMD8: SEND_IF_COND (SD v2 check, 0x1AA = 2.7-3.6V + check pattern) */
    u32 resp[4];
    bool sd_v2 = sd_send_cmd(SD_CMD8, 0x1AA, resp);
    if (sd_v2 && (resp[0] & 0xFFF) != 0x1AA) {
        uart_puts("[sd] CMD8 bad response\n");
        return false;
    }

    /* ACMD41: SD_SEND_OP_COND - poll until ready */
    u32 acmd41_arg = 0x00FF8000;  /* 3.2-3.4V */
    if (sd_v2)
        acmd41_arg |= (1 << 30);  /* HCS: host supports SDHC */

    timeout = 100;
    do {
        if (!sd_send_acmd(SD_ACMD41, acmd41_arg, resp)) {
            uart_puts("[sd] ACMD41 failed\n");
            return false;
        }
        delay_cycles(100000);
    } while (!(resp[0] & (1 << 31)) && timeout--);

    if (!timeout) {
        uart_puts("[sd] Card init timeout\n");
        return false;
    }

    card.type = (resp[0] & (1 << 30)) ? 2 : 1;  /* SDHC or SDSC */

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
        c0 |= (1 << 1);   /* 4-bit mode */
        sd_write(REG_CONTROL0, c0);
    }

    /* CMD16: SET_BLOCKLEN to 512 (for SDSC cards) */
    if (card.type == 1)
        sd_send_cmd(SD_CMD16, 512, resp);

    uart_puts("[sd] Card ready: ");
    uart_puts(card.type == 2 ? "SDHC/SDXC" : "SDSC");
    uart_puts(" RCA=");
    uart_hex(card.rca);
    uart_puts("\n");

    return true;
}

bool sd_read_block(u32 lba, u8 *buf) {
    u32 addr = (card.type == 2) ? lba : lba * 512;

    if (!sd_wait_data())
        return false;

    sd_write(REG_BLKSIZECNT, (1 << 16) | 512);
    sd_write(REG_INTERRUPT, INT_ALL);

    u32 resp[4];
    if (!sd_send_cmd(SD_CMD17, addr, resp))
        return false;

    /* Wait for read ready */
    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sd_write(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        delay_cycles(10);
    } while (!(intr & INT_READ_RDY) && timeout--);

    if (!timeout)
        return false;

    /* Read 512 bytes (128 x 32-bit words) */
    u32 *buf32 = (u32 *)buf;
    for (int i = 0; i < 128; i++)
        buf32[i] = sd_read(REG_DATA);

    /* Wait for transfer complete */
    timeout = 1000000;
    do {
        intr = sd_read(REG_INTERRUPT);
        delay_cycles(10);
    } while (!(intr & INT_DATA_DONE) && timeout--);

    sd_write(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sd_write_block(u32 lba, const u8 *buf) {
    u32 addr = (card.type == 2) ? lba : lba * 512;

    if (!sd_wait_data())
        return false;

    sd_write(REG_BLKSIZECNT, (1 << 16) | 512);
    sd_write(REG_INTERRUPT, INT_ALL);

    u32 resp[4];
    if (!sd_send_cmd(SD_CMD24, addr, resp))
        return false;

    /* Wait for write ready */
    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sd_read(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sd_write(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        delay_cycles(10);
    } while (!(intr & INT_WRITE_RDY) && timeout--);

    if (!timeout)
        return false;

    /* Write 512 bytes */
    const u32 *buf32 = (const u32 *)buf;
    for (int i = 0; i < 128; i++)
        sd_write(REG_DATA, buf32[i]);

    /* Wait for transfer complete */
    timeout = 1000000;
    do {
        intr = sd_read(REG_INTERRUPT);
        delay_cycles(10);
    } while (!(intr & INT_DATA_DONE) && timeout--);

    sd_write(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sd_read_blocks(u32 lba, u32 count, u8 *buf) {
    for (u32 i = 0; i < count; i++) {
        if (!sd_read_block(lba + i, buf + i * SD_BLOCK_SIZE))
            return false;
    }
    return true;
}

bool sd_write_blocks(u32 lba, u32 count, const u8 *buf) {
    for (u32 i = 0; i < count; i++) {
        if (!sd_write_block(lba + i, buf + i * SD_BLOCK_SIZE))
            return false;
    }
    return true;
}

const sd_card_t *sd_get_card_info(void) {
    return &card;
}
