#pragma once
#include "types.h"

/* HDMI framebuffer text console - 8x8 font, direct pixel writes */

bool fb_init(u32 width, u32 height);
bool fb_adopt(u64 base, u32 width, u32 height, u32 pitch, u32 size);

struct fb_mailbox_diag {
    u32 status;
    u32 message;
    u32 response;
    u32 allocation_addr;
    u32 allocation_size;
    u32 pitch;
};

void fb_mailbox_diag(struct fb_mailbox_diag *out);
void fb_clear(u32 color);
void fb_clear_row(u32 row);
void fb_putc(char c);
void fb_puts(const char *s);
void fb_printf(const char *fmt, ...);
void fb_set_color(u32 fg, u32 bg);
void fb_set_cursor(u32 col, u32 row);
void fb_set_reserved_rows(u32 rows);
u32 fb_cols(void);
u32 fb_rows(void);
u32 fb_reserved_rows(void);
void fb_hline(u32 n);
void fb_box(u32 width, u32 height, const char *title);
void fb_box_at(u32 col, u32 row, u32 width, u32 height, const char *title);

/* Direct pixel access */
void fb_pixel(u32 x, u32 y, u32 color);

/* Visual status strip — paints a small block at the bottom of the screen
 * each call, wrapping left-to-right and bottom-up. Used for network
 * activity heartbeat. Colors are 0x00RRGGBB. */
void fb_status_block(u32 color);

/* Get framebuffer physical address (for MMU mapping) */
u64 fb_get_phys_addr(void);

/* Push dirty rows from the cached back buffer to the VideoCore scanout buffer
 * (DMA blit). Cheap no-op when nothing changed or double-buffering is off. */
void fb_present(void);

/* Report whether double-buffering is active plus FB byte-size and pitch. */
void fb_debug_info(u32 *db, u32 *size, u32 *pitch);
void fb_display_info(u64 *base, u32 *width, u32 *height, u32 *pitch, u32 *size);

/* CNTPCT ticks taken by the most recent back→front blit. */
u64 fb_last_blit_ticks(void);

/* Set the ARM (A76) clock to the firmware's max rate; returns the rate set (Hz)
 * or 0 on failure. Bare-metal Pi 5 otherwise runs the CPU at a low default. */
u32 fb_set_arm_clock_max(void);
/* Read the current ARM clock rate (Hz). */
u32 fb_get_arm_clock(void);
/* GET_CLOCK_RATE for any clock id (3=ARM, 4=CORE). */
u32 fb_get_clock_rate_id(u32 clock_id);
/* GET_THROTTLED bitmap (under-voltage / freq-capped / throttled / temp-limit). */
u32 fb_get_throttled(void);
/* SoC temperature in millidegrees Celsius. */
u32 fb_get_temperature_mc(void);
