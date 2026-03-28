/*
 * rp1_clk.h - RP1 clock and reset controller
 *
 * Minimal interface for enabling/disabling RP1 peripheral clocks.
 * On Pi 5, the GPU firmware pre-configures PLLs and most clocks
 * before handing off, so we mainly need clock gating control.
 *
 * Clock controller is at RP1 BAR + 0x18000.
 *
 * Reference: Linux drivers/clk/clk-rp1.c
 *            rp1.dtsi: clocks@18000
 */

#pragma once
#include "types.h"

/* Clock IDs (matching Linux dt-bindings/clock/rp1.h) */
#define RP1_CLK_SYS         0
#define RP1_CLK_SLOW_SYS    1
#define RP1_CLK_UART        3
#define RP1_CLK_ETH         4
#define RP1_CLK_PWM0         5
#define RP1_CLK_PWM1         6
#define RP1_CLK_I2S         9
#define RP1_CLK_ADC         16

void rp1_clk_init(void);
bool rp1_clk_enable(u32 clk_id);
void rp1_clk_disable(u32 clk_id);
bool rp1_clk_is_enabled(u32 clk_id);
