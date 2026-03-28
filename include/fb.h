#pragma once
#include "types.h"

/* HDMI framebuffer text console - 8x8 font, direct pixel writes */

bool fb_init(u32 width, u32 height);
void fb_clear(u32 color);
void fb_putc(char c);
void fb_puts(const char *s);
void fb_printf(const char *fmt, ...);
void fb_set_color(u32 fg, u32 bg);

/* Direct pixel access */
void fb_pixel(u32 x, u32 y, u32 color);
