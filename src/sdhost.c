#include "sdhost.h"
#include "fb.h"
#include "mmio.h"
#include "uart.h"

#if PIOS_PLATFORM == PIOS_PLATFORM_PI3 || \
    PIOS_PLATFORM == PIOS_PLATFORM_PIZERO2W || \
    defined(PIOS_RUNTIME_MMIO_BOOTSTRAP)

#define SDCMD   0x00U
#define SDARG   0x04U
#define SDTOUT  0x08U
#define SDCDIV  0x0CU
#define SDRSP0  0x10U
#define SDRSP1  0x14U
#define SDRSP2  0x18U
#define SDRSP3  0x1CU
#define SDHSTS  0x20U
#define SDVDD   0x30U
#define SDEDM   0x34U
#define SDHCFG  0x38U
#define SDHBCT  0x3CU
#define SDDATA  0x40U
#define SDHBLC  0x50U

#define SDCMD_NEW_FLAG      0x8000U
#define SDCMD_FAIL_FLAG     0x4000U
#define SDCMD_BUSYWAIT      0x0800U
#define SDCMD_NO_RESPONSE   0x0400U
#define SDCMD_LONG_RESPONSE 0x0200U
#define SDCMD_WRITE_CMD     0x0080U
#define SDCMD_READ_CMD      0x0040U
#define SDCMD_CMD_MASK      0x003FU

#define SDHSTS_BUSY_IRPT    0x400U
#define SDHSTS_BLOCK_IRPT   0x200U
#define SDHSTS_SDIO_IRPT    0x100U
#define SDHSTS_REW_TIME_OUT 0x080U
#define SDHSTS_CMD_TIME_OUT 0x040U
#define SDHSTS_CRC16_ERROR  0x020U
#define SDHSTS_CRC7_ERROR   0x010U
#define SDHSTS_FIFO_ERROR   0x008U

#define SDHSTS_ERROR_MASK (SDHSTS_REW_TIME_OUT | SDHSTS_CMD_TIME_OUT | \
                           SDHSTS_CRC16_ERROR | SDHSTS_CRC7_ERROR | \
                           SDHSTS_FIFO_ERROR)
#define SDHSTS_CLEAR_MASK (SDHSTS_BUSY_IRPT | SDHSTS_BLOCK_IRPT | \
                           SDHSTS_SDIO_IRPT | SDHSTS_ERROR_MASK)

#define SDHCFG_BUSY_IRPT_EN  (1U << 10)
#define SDHCFG_BLOCK_IRPT_EN (1U << 8)
#define SDHCFG_SLOW_CARD     (1U << 3)
#define SDHCFG_WIDE_EXT_BUS  (1U << 2)
#define SDHCFG_WIDE_INT_BUS  (1U << 1)

#define SDEDM_FORCE_DATA_MODE      (1U << 19)
#define SDEDM_WRITE_THRESHOLD_SHIFT 9U
#define SDEDM_READ_THRESHOLD_SHIFT 14U
#define SDEDM_THRESHOLD_MASK        0x1FU
#define SDEDM_FIFO_FILL_SHIFT       4U
#define SDEDM_FIFO_FILL_MASK        0x1FU
#define SDEDM_FSM_MASK              0x0FU
#define SDEDM_FSM_IDENTMODE         0x00U
#define SDEDM_FSM_DATAMODE          0x01U
#define SDEDM_FSM_READDATA          0x02U
#define SDEDM_FSM_WRITEDATA         0x03U
#define SDEDM_FSM_READWAIT          0x04U
#define SDEDM_FSM_READCRC           0x05U
#define SDEDM_FSM_WRITECRC          0x06U
#define SDEDM_FSM_WRITEWAIT1        0x07U
#define SDEDM_FSM_WRITESTART1       0x0AU
#define SDEDM_FSM_WRITESTART2       0x0BU
#define SDEDM_FSM_WRITEWAIT2        0x0DU

#define SDHOST_FIFO_WORDS 16U
#define SDHOST_FIFO_BURST 8U
#define SDHOST_CMD_TIMEOUT_US 100000ULL
#define SDHOST_DATA_TIMEOUT_US 500000ULL
#define SDHOST_BUSY_TIMEOUT_US 1000000ULL
#define SDHOST_INIT_TIMEOUT_US 2000000ULL
#define SDHOST_RETRIES 3U

#define GPIO_BASE_DELTA 0x2000ULL
#define GPFSEL0  0x00U
#define GPPUD    0x94U
#define GPPUDCLK1 0x9CU
#define GPIO_ALT0 4U
#define GPIO_PULL_OFF 0U
#define GPIO_PULL_UP  2U

#define RESP_PRESENT (1U << 0)
#define RESP_LONG    (1U << 1)
#define RESP_BUSY    (1U << 2)
#define DATA_NONE  0U
#define DATA_READ  1U
#define DATA_WRITE 2U

static u64 g_base;
static u32 g_hcfg;
static u32 g_last_status;
static u32 g_last_command;
static bool g_ready;
static struct sdhost_card_info g_card;

static inline u32 reg_read(u32 offset)
{
    return mmio_read(g_base + offset);
}

static inline void reg_write(u32 offset, u32 value)
{
    mmio_write(g_base + offset, value);
}

static u64 now_us(void)
{
    u64 frequency;
    u64 counter;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(counter));
    if (frequency < 1000000ULL)
        return ~(u64)0;
    return counter / (frequency / 1000000ULL);
}

static bool timer_available(void)
{
    u64 frequency;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    return frequency >= 1000000ULL;
}

static void delay_us(u32 duration)
{
    u64 start = now_us();
    while (now_us() - start < duration)
        ;
}

static void report_error(const char *reason, u32 command)
{
    u32 cmd = reg_read(SDCMD);
    u32 status = reg_read(SDHSTS);
    u32 edm = reg_read(SDEDM);
    g_last_command = cmd;
    g_last_status = status;
    uart_puts("[sdh] ");
    uart_puts(reason);
    uart_puts(" c=");
    uart_hex(command);
    uart_puts(" cmd=");
    uart_hex(cmd);
    uart_puts(" h=");
    uart_hex(status);
    uart_puts(" e=");
    uart_hex(edm);
    uart_puts("\n");
    fb_printf("  [sdh] %s c=%x cmd=%x h=%x e=%x\n",
              reason, command, cmd, status, edm);
    fb_printf("  [sdh] d=%x t=%x v=%x cfg=%x\n",
              reg_read(SDCDIV), reg_read(SDTOUT),
              reg_read(SDVDD), reg_read(SDHCFG));
}

static inline u64 gpio_base(void)
{
    return g_base - GPIO_BASE_DELTA;
}

static void gpio_write(u32 offset, u32 value)
{
    mmio_write(gpio_base() + offset, value);
}

static u32 gpio_read(u32 offset)
{
    return mmio_read(gpio_base() + offset);
}

static void gpio_set_function(u32 pin, u32 function)
{
    u32 offset = GPFSEL0 + (pin / 10U) * 4U;
    u32 shift = (pin % 10U) * 3U;
    u32 value = gpio_read(offset);
    value &= ~(7U << shift);
    value |= function << shift;
    gpio_write(offset, value);
}

static void gpio_set_pull_mask(u32 pull, u32 bank1_mask)
{
    gpio_write(GPPUD, pull);
    delay_us(5U);
    gpio_write(GPPUDCLK1, bank1_mask);
    delay_us(5U);
    gpio_write(GPPUDCLK1, 0U);
    gpio_write(GPPUD, GPIO_PULL_OFF);
}

static void configure_card_pins(void)
{
    for (u32 pin = 48U; pin <= 53U; pin++)
        gpio_set_function(pin, GPIO_ALT0);

    u32 all_pins = 0x3FU << (48U - 32U);
    u32 cmd_data = 0x3EU << (48U - 32U);
    gpio_set_pull_mask(GPIO_PULL_OFF, all_pins);
    gpio_set_pull_mask(GPIO_PULL_UP, cmd_data);
}

static bool fsm_idle(void)
{
    u32 fsm = reg_read(SDEDM) & SDEDM_FSM_MASK;
    return fsm == SDEDM_FSM_IDENTMODE || fsm == SDEDM_FSM_DATAMODE;
}

static bool wait_command_clear(u64 timeout_us)
{
    u64 deadline = now_us() + timeout_us;
    while (reg_read(SDCMD) & SDCMD_NEW_FLAG) {
        if (now_us() >= deadline)
            return false;
    }
    return true;
}

static void recover_transfer(void)
{
    u32 edm = reg_read(SDEDM);
    if (!fsm_idle())
        reg_write(SDEDM, edm | SDEDM_FORCE_DATA_MODE);
    reg_write(SDCMD, 0U);
    reg_write(SDHBCT, 0U);
    reg_write(SDHBLC, 0U);
    reg_write(SDHSTS, SDHSTS_CLEAR_MASK);
    reg_write(SDHCFG, g_hcfg);
    delay_us(100U);
}

static bool finish_command(u32 command, u32 response_flags, u32 response[4])
{
    u64 timeout = (response_flags & RESP_BUSY) ?
                  SDHOST_BUSY_TIMEOUT_US : SDHOST_CMD_TIMEOUT_US;
    if (!wait_command_clear(timeout)) {
        report_error("cmd timeout", command);
        return false;
    }

    u32 cmd = reg_read(SDCMD);
    u32 status = reg_read(SDHSTS);
    u32 errors = status & SDHSTS_ERROR_MASK;
    if (command == 41U)
        errors &= ~SDHSTS_CRC7_ERROR;
    if (errors || ((cmd & SDCMD_FAIL_FLAG) && command != 41U)) {
        report_error("cmd failed", command);
        reg_write(SDHSTS, status & SDHSTS_CLEAR_MASK);
        return false;
    }

    if (response_flags & RESP_PRESENT) {
        if (response_flags & RESP_LONG) {
            response[0] = reg_read(SDRSP3);
            response[1] = reg_read(SDRSP2);
            response[2] = reg_read(SDRSP1);
            response[3] = reg_read(SDRSP0);
        } else {
            response[0] = reg_read(SDRSP0);
            response[1] = 0U;
            response[2] = 0U;
            response[3] = 0U;
        }
    }
    reg_write(SDHSTS, status & SDHSTS_CLEAR_MASK);
    return true;
}

static bool transfer_word_buffer(u32 direction, void *buffer)
{
    u32 words_left = 512U / sizeof(u32);
    u8 *read_buffer = (u8 *)buffer;
    const u8 *write_buffer = (const u8 *)buffer;
    u64 deadline = now_us() + SDHOST_DATA_TIMEOUT_US;

    while (words_left) {
        u32 status = reg_read(SDHSTS);
        if (status & SDHSTS_ERROR_MASK) {
            report_error("data failed", direction);
            return false;
        }

        u32 edm = reg_read(SDEDM);
        u32 fill = (edm >> SDEDM_FIFO_FILL_SHIFT) &
                   SDEDM_FIFO_FILL_MASK;
        if (fill > SDHOST_FIFO_WORDS) {
            report_error("fifo level", direction);
            return false;
        }
        u32 available = direction == DATA_READ ?
                        fill : SDHOST_FIFO_WORDS - fill;
        if (available == 0U) {
            if (now_us() >= deadline) {
                report_error("fifo timeout", direction);
                return false;
            }
            continue;
        }

        u32 take = available;
        if (take > SDHOST_FIFO_BURST)
            take = SDHOST_FIFO_BURST;
        if (take > words_left)
            take = words_left;
        for (u32 i = 0; i < take; i++) {
            if (direction == DATA_READ) {
                u32 word = reg_read(SDDATA);
                read_buffer[0] = (u8)word;
                read_buffer[1] = (u8)(word >> 8);
                read_buffer[2] = (u8)(word >> 16);
                read_buffer[3] = (u8)(word >> 24);
                read_buffer += sizeof(word);
            } else {
                u32 word = (u32)write_buffer[0] |
                           ((u32)write_buffer[1] << 8) |
                           ((u32)write_buffer[2] << 16) |
                           ((u32)write_buffer[3] << 24);
                reg_write(SDDATA, word);
                write_buffer += sizeof(word);
            }
        }
        words_left -= take;
    }
    return true;
}

static bool wait_transfer_complete(u32 direction)
{
    u64 deadline = now_us() + SDHOST_DATA_TIMEOUT_US;
    while (now_us() < deadline) {
        u32 status = reg_read(SDHSTS);
        if (status & SDHSTS_ERROR_MASK) {
            report_error("xfer failed", direction);
            return false;
        }
        u32 edm = reg_read(SDEDM);
        u32 fsm = edm & SDEDM_FSM_MASK;
        if (fsm == SDEDM_FSM_IDENTMODE || fsm == SDEDM_FSM_DATAMODE) {
            reg_write(SDHSTS, status & SDHSTS_CLEAR_MASK);
            return true;
        }
        if (status & SDHSTS_BLOCK_IRPT) {
            if (fsm == SDEDM_FSM_READWAIT ||
                fsm == SDEDM_FSM_WRITESTART1 ||
                fsm == SDEDM_FSM_READDATA) {
                reg_write(SDEDM, edm | SDEDM_FORCE_DATA_MODE);
                reg_write(SDHSTS, SDHSTS_BLOCK_IRPT);
                return true;
            }
            reg_write(SDHSTS, SDHSTS_BLOCK_IRPT);
        }
    }
    report_error("xfer timeout", direction);
    return false;
}

static bool issue_command(u32 command, u32 argument, u32 response_flags,
                          u32 direction, void *buffer, u32 response[4])
{
    if (!wait_command_clear(SDHOST_CMD_TIMEOUT_US)) {
        report_error("previous cmd", command);
        return false;
    }
    if (!fsm_idle()) {
        report_error("fsm busy", command);
        return false;
    }

    reg_write(SDHSTS, SDHSTS_CLEAR_MASK);
    if (direction == DATA_NONE) {
        reg_write(SDHBCT, 0U);
        reg_write(SDHBLC, 0U);
    } else {
        if (!buffer)
            return false;
        reg_write(SDHBCT, 512U);
        reg_write(SDHBLC, 1U);
    }
    reg_write(SDARG, argument);

    u32 descriptor = command & SDCMD_CMD_MASK;
    if ((response_flags & RESP_PRESENT) == 0U)
        descriptor |= SDCMD_NO_RESPONSE;
    if (response_flags & RESP_LONG)
        descriptor |= SDCMD_LONG_RESPONSE;
    if (response_flags & RESP_BUSY)
        descriptor |= SDCMD_BUSYWAIT;
    if (direction == DATA_READ)
        descriptor |= SDCMD_READ_CMD;
    else if (direction == DATA_WRITE)
        descriptor |= SDCMD_WRITE_CMD;
    reg_write(SDCMD, descriptor | SDCMD_NEW_FLAG);

    if (!finish_command(command, response_flags, response))
        return false;
    if (direction == DATA_NONE)
        return true;
    return transfer_word_buffer(direction, buffer) &&
           wait_transfer_complete(direction);
}

static bool send_acmd(u32 command, u32 argument, u32 response[4])
{
    u32 app_response[4];
    if (!issue_command(55U, g_card.rca << 16, RESP_PRESENT,
                       DATA_NONE, NULL, app_response))
        return false;
    if ((app_response[0] & (1U << 5)) == 0U) {
        report_error("not app cmd", command);
        return false;
    }
    return issue_command(command, argument, RESP_PRESENT,
                         DATA_NONE, NULL, response);
}

static bool set_clock(u32 core_hz, u32 target_hz)
{
    u32 divider;
    u32 actual;
    if (!sdhost_clock_divider(core_hz, target_hz, &divider, &actual))
        return false;
    reg_write(SDCDIV, divider);
    reg_write(SDTOUT, actual / 2U);
    if (target_hz == 400000U)
        fb_printf("  [sdh] core=%u cdiv=%u clk=%u\n",
                  core_hz, divider, actual);
    return true;
}

static void controller_reset(void)
{
    reg_write(SDVDD, 0U);
    reg_write(SDCMD, 0U);
    reg_write(SDARG, 0U);
    reg_write(SDTOUT, 0x00F00000U);
    reg_write(SDCDIV, 0x7FFU);
    reg_write(SDHSTS, SDHSTS_CLEAR_MASK);
    reg_write(SDHCFG, 0U);
    reg_write(SDHBCT, 0U);
    reg_write(SDHBLC, 0U);

    u32 edm = reg_read(SDEDM);
    edm &= ~((SDEDM_THRESHOLD_MASK << SDEDM_READ_THRESHOLD_SHIFT) |
             (SDEDM_THRESHOLD_MASK << SDEDM_WRITE_THRESHOLD_SHIFT));
    edm |= (4U << SDEDM_READ_THRESHOLD_SHIFT) |
           (4U << SDEDM_WRITE_THRESHOLD_SHIFT);
    reg_write(SDEDM, edm);
    delay_us(10000U);
    reg_write(SDVDD, 1U);
    delay_us(40000U);

    g_hcfg = SDHCFG_BUSY_IRPT_EN | SDHCFG_BLOCK_IRPT_EN |
             SDHCFG_WIDE_INT_BUS | SDHCFG_SLOW_CARD;
    reg_write(SDHCFG, g_hcfg);
}

bool sdhost_init(u64 base, u32 core_clock_hz,
                 struct sdhost_card_info *info)
{
    if (!base || (base & 3U) || !info ||
        core_clock_hz < 1000000U || core_clock_hz > 1000000000U)
        return false;

    g_base = base;
    g_ready = false;
    g_last_status = 0U;
    g_last_command = 0U;
    g_card.type = 0U;
    g_card.rca = 0U;
    g_card.capacity = 0U;
    if (!timer_available()) {
        uart_puts("[sdh] generic timer unavailable\n");
        fb_puts("  [sdh] generic timer unavailable\n");
        return false;
    }
    configure_card_pins();
    controller_reset();
    if (!set_clock(core_clock_hz, 400000U)) {
        report_error("clock", 0U);
        return false;
    }

    fb_puts("  [sdh] CMD0...\n");
    u32 response[4];
    if (!issue_command(0U, 0U, 0U, DATA_NONE, NULL, response))
        return false;
    delay_us(2000U);

    fb_puts("  [sdh] CMD8...\n");
    bool version2 = issue_command(8U, 0x1AAU, RESP_PRESENT,
                                  DATA_NONE, NULL, response);
    if (version2 && (response[0] & 0xFFFU) != 0x1AAU) {
        report_error("CMD8 response", 8U);
        return false;
    }
    if (!version2) {
        recover_transfer();
        reg_write(SDHSTS, SDHSTS_CLEAR_MASK);
    }

    fb_puts("  [sdh] ACMD41...\n");
    u32 argument = 0x00FF8000U;
    if (version2)
        argument |= 1U << 30;
    u64 deadline = now_us() + SDHOST_INIT_TIMEOUT_US;
    do {
        if (!send_acmd(41U, argument, response))
            return false;
        if (response[0] & (1U << 31))
            break;
        delay_us(1000U);
    } while (now_us() < deadline);
    if ((response[0] & (1U << 31)) == 0U) {
        report_error("card timeout", 41U);
        return false;
    }
    g_card.type = (response[0] & (1U << 30)) ? 2U : 1U;

    if (!issue_command(2U, 0U, RESP_PRESENT | RESP_LONG,
                       DATA_NONE, NULL, response) ||
        !issue_command(3U, 0U, RESP_PRESENT,
                       DATA_NONE, NULL, response))
        return false;
    g_card.rca = response[0] >> 16;
    if (g_card.rca == 0U) {
        report_error("zero RCA", 3U);
        return false;
    }

    if (issue_command(9U, g_card.rca << 16, RESP_PRESENT | RESP_LONG,
                      DATA_NONE, NULL, response)) {
        if (!sdhost_csd_capacity(response, &g_card.capacity))
            uart_puts("[sdh] CSD capacity unavailable\n");
    } else {
        uart_puts("[sdh] CMD9 failed; capacity unavailable\n");
        recover_transfer();
    }

    if (!issue_command(7U, g_card.rca << 16,
                       RESP_PRESENT | RESP_BUSY,
                       DATA_NONE, NULL, response))
        return false;
    if (!set_clock(core_clock_hz, 25000000U)) {
        report_error("data clock", 0U);
        return false;
    }

    if (send_acmd(6U, 2U, response)) {
        g_hcfg |= SDHCFG_WIDE_EXT_BUS;
        reg_write(SDHCFG, g_hcfg);
    } else {
        uart_puts("[sdh] 4-bit mode unavailable; using 1-bit\n");
        recover_transfer();
    }
    if (g_card.type == 1U &&
        !issue_command(16U, 512U, RESP_PRESENT,
                       DATA_NONE, NULL, response))
        return false;

    g_ready = true;
    *info = g_card;
    uart_puts("[sdh] card ready RCA=");
    uart_hex(g_card.rca);
    uart_puts(" cap=");
    uart_hex(g_card.capacity);
    uart_puts("\n");
    return true;
}

static bool block_address(u32 lba, u32 *address)
{
    if (!g_ready || !address)
        return false;
    if (g_card.capacity && (u64)lba >= g_card.capacity / 512ULL)
        return false;
    if (g_card.type == 2U) {
        *address = lba;
        return true;
    }
    if (lba > 0xFFFFFFFFU / 512U)
        return false;
    *address = lba * 512U;
    return true;
}

static bool transfer_block(u32 lba, void *buffer, u32 direction)
{
    u32 address;
    if (!buffer || !block_address(lba, &address))
        return false;
    u32 response[4];
    u32 command = direction == DATA_READ ? 17U : 24U;
    for (u32 attempt = 0; attempt < SDHOST_RETRIES; attempt++) {
        if (issue_command(command, address, RESP_PRESENT,
                          direction, buffer, response))
            return true;
        recover_transfer();
    }
    return false;
}

bool sdhost_read_block(u32 lba, u8 *buf)
{
    return transfer_block(lba, buf, DATA_READ);
}

bool sdhost_write_block(u32 lba, const u8 *buf)
{
    return transfer_block(lba, (void *)(usize)buf, DATA_WRITE);
}

bool sdhost_read_blocks(u32 lba, u32 count, u8 *buf)
{
    if (count == 0U)
        return true;
    if (!buf || lba > 0xFFFFFFFFU - (count - 1U) ||
        (usize)buf > ~(usize)0 - (usize)count * 512U)
        return false;
    for (u32 block = 0; block < count; block++) {
        if (!sdhost_read_block(lba + block, buf + (usize)block * 512U))
            return false;
    }
    return true;
}

bool sdhost_write_blocks(u32 lba, u32 count, const u8 *buf)
{
    if (count == 0U)
        return true;
    if (!buf || lba > 0xFFFFFFFFU - (count - 1U) ||
        (usize)buf > ~(usize)0 - (usize)count * 512U)
        return false;
    for (u32 block = 0; block < count; block++) {
        if (!sdhost_write_block(lba + block,
                                buf + (usize)block * 512U))
            return false;
    }
    return true;
}

#else

bool sdhost_init(u64 base, u32 core_clock_hz,
                 struct sdhost_card_info *info)
{
    (void)base;
    (void)core_clock_hz;
    (void)info;
    return false;
}

bool sdhost_read_block(u32 lba, u8 *buf)
{
    (void)lba;
    (void)buf;
    return false;
}

bool sdhost_write_block(u32 lba, const u8 *buf)
{
    (void)lba;
    (void)buf;
    return false;
}

bool sdhost_read_blocks(u32 lba, u32 count, u8 *buf)
{
    (void)lba;
    (void)count;
    (void)buf;
    return false;
}

bool sdhost_write_blocks(u32 lba, u32 count, const u8 *buf)
{
    (void)lba;
    (void)count;
    (void)buf;
    return false;
}

#endif
