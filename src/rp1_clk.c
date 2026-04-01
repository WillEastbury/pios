/*
 * rp1_clk.c - RP1 clock controller (minimal)
 *
 * The clock controller sits at RP1 BAR + 0x18000.
 * On Pi 5, the GPU firmware pre-configures all PLLs and dividers.
 * This driver provides clock gate enable/disable for RP1 peripherals.
 *
 * Clock register layout (per clock, from clk-rp1.c):
 *   CTRL reg:    bit 11 = enable
 *   DIV_INT reg: integer divider
 *   SEL reg:     clock source select
 *
 * Reference: Linux drivers/clk/clk-rp1.c
 *            rp1.dtsi: clocks@18000 { reg = <0xc0 0x40018000 0x0 0x10038>; }
 */

#include "rp1_clk.h"
#include "mmio.h"
#include "uart.h"
#include "fb.h"

/* Clock controller base within RP1 BAR */
#define CLK_BASE    0x018000

/* CTRL register: bit 11 = enable */
#define CLK_CTRL_ENABLE     (1U << 11)

/*
 * Clock register offsets relative to CLK_BASE.
 * Each clock has CTRL, DIV_INT, and SEL registers.
 * From clk-rp1.c header definitions.
 */
struct clk_regs {
    u32 ctrl;
    u32 div_int;
    u32 sel;
};

static const struct clk_regs clk_table[] = {
    [RP1_CLK_SYS]      = { 0x0014, 0x0018, 0x0020 },
    [RP1_CLK_SLOW_SYS] = { 0x0024, 0x0028, 0x0030 },
    [2]                 = { 0x0044, 0x0048, 0x0050 }, /* DMA */
    [RP1_CLK_UART]     = { 0x0054, 0x0058, 0x0060 },
    [RP1_CLK_ETH]      = { 0x0064, 0x0068, 0x0070 },
    [RP1_CLK_PWM0]     = { 0x0074, 0x0078, 0x0080 },
    [RP1_CLK_PWM1]     = { 0x0084, 0x0088, 0x0090 },
    [7]                 = { 0x0094, 0x0098, 0x00A0 }, /* AUDIO_IN */
    [8]                 = { 0x00A4, 0x00A8, 0x00B0 }, /* AUDIO_OUT */
    [RP1_CLK_I2S]      = { 0x00B4, 0x00B8, 0x00C0 },
};

#define CLK_TABLE_SIZE  (sizeof(clk_table) / sizeof(clk_table[0]))

/* Register helpers relative to RP1 BAR */
static inline u32 cr(u32 off) { return mmio_read(RP1_BAR_BASE + CLK_BASE + off); }
static inline void cw(u32 off, u32 val) { mmio_write(RP1_BAR_BASE + CLK_BASE + off, val); }

void rp1_clk_init(void) {
    fb_puts("  [rp1_clk] Reading UART clock ctrl register\n");
    /* Read CLK_UART to verify clock controller is accessible */
    u32 uart_ctrl = cr(clk_table[RP1_CLK_UART].ctrl);
    fb_printf("  [rp1_clk] UART clk ctrl=0x%x %s\n", uart_ctrl,
              (uart_ctrl & CLK_CTRL_ENABLE) ? "(enabled)" : "(disabled)");
    uart_puts("[rp1_clk] UART clk ctrl=");
    uart_hex(uart_ctrl);
    uart_puts(uart_ctrl & CLK_CTRL_ENABLE ? " (enabled)\n" : " (disabled)\n");
    fb_puts("  [rp1_clk] Clock controller ready\n");
}

bool rp1_clk_enable(u32 clk_id) {
    if (clk_id >= CLK_TABLE_SIZE || clk_table[clk_id].ctrl == 0)
        return false;
    u32 ctrl = cr(clk_table[clk_id].ctrl);
    ctrl |= CLK_CTRL_ENABLE;
    cw(clk_table[clk_id].ctrl, ctrl);
    return true;
}

void rp1_clk_disable(u32 clk_id) {
    if (clk_id >= CLK_TABLE_SIZE || clk_table[clk_id].ctrl == 0)
        return;
    u32 ctrl = cr(clk_table[clk_id].ctrl);
    ctrl &= ~CLK_CTRL_ENABLE;
    cw(clk_table[clk_id].ctrl, ctrl);
}

bool rp1_clk_is_enabled(u32 clk_id) {
    if (clk_id >= CLK_TABLE_SIZE || clk_table[clk_id].ctrl == 0)
        return false;
    return (cr(clk_table[clk_id].ctrl) & CLK_CTRL_ENABLE) != 0;
}
