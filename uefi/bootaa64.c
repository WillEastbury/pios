/*
 * ARM64 BOOTAA64.EFI QEMU PIOS stager.
 *
 * This is not yet the final ExitBootServices handoff. It is a UEFI-hosted
 * stager that proves a visible PIOS environment under QEMU/EDK2: console,
 * platform identity, RAM-backed SD, real bcache/WALFS/LRU, and a file
 * round-trip. It parks instead of returning to the firmware Boot Manager.
 */
#include "../include/types.h"
#include "../include/platform.h"
#include "../include/sd.h"
#include "../include/bcache.h"
#include "../include/walfs.h"
#include "../include/fifo.h"
#include "../include/principal.h"

typedef u64 efi_status_t;
typedef void *efi_handle_t;

struct efi_table_header {
    u64 signature;
    u32 revision;
    u32 header_size;
    u32 crc32;
    u32 reserved;
};

typedef efi_status_t (*efi_text_reset_fn)(void *self, bool extended_verification);
typedef efi_status_t (*efi_text_string_fn)(void *self, const u16 *str);
typedef efi_status_t (*efi_text_query_mode_fn)(void *self, usize mode, usize *cols, usize *rows);
typedef efi_status_t (*efi_text_set_mode_fn)(void *self, usize mode);
typedef efi_status_t (*efi_text_set_attribute_fn)(void *self, usize attr);
typedef efi_status_t (*efi_text_clear_screen_fn)(void *self);
typedef efi_status_t (*efi_text_set_cursor_fn)(void *self, usize col, usize row);
typedef efi_status_t (*efi_text_enable_cursor_fn)(void *self, bool visible);

struct efi_simple_text_output {
    efi_text_reset_fn reset;
    efi_text_string_fn output_string;
    void *test_string;
    efi_text_query_mode_fn query_mode;
    efi_text_set_mode_fn set_mode;
    efi_text_set_attribute_fn set_attribute;
    efi_text_clear_screen_fn clear_screen;
    efi_text_set_cursor_fn set_cursor_position;
    efi_text_enable_cursor_fn enable_cursor;
    void *mode;
};

struct efi_guid {
    u32 data1;
    u16 data2;
    u16 data3;
    u8 data4[8];
};

typedef efi_status_t (*efi_locate_protocol_fn)(struct efi_guid *protocol,
                                               void *registration,
                                               void **interface);

struct efi_boot_services {
    struct efi_table_header hdr;
    void *raise_tpl;
    void *restore_tpl;
    void *allocate_pages;
    void *free_pages;
    void *get_memory_map;
    void *allocate_pool;
    void *free_pool;
    void *create_event;
    void *set_timer;
    void *wait_for_event;
    void *signal_event;
    void *close_event;
    void *check_event;
    void *install_protocol_interface;
    void *reinstall_protocol_interface;
    void *uninstall_protocol_interface;
    void *handle_protocol;
    void *reserved;
    void *register_protocol_notify;
    void *locate_handle;
    void *locate_device_path;
    void *install_configuration_table;
    void *load_image;
    void *start_image;
    void *exit;
    void *unload_image;
    void *exit_boot_services;
    void *get_next_monotonic_count;
    void *stall;
    void *set_watchdog_timer;
    void *connect_controller;
    void *disconnect_controller;
    void *open_protocol;
    void *close_protocol;
    void *open_protocol_information;
    void *protocols_per_handle;
    void *locate_handle_buffer;
    efi_locate_protocol_fn locate_protocol;
};

struct efi_pixel_bitmask {
    u32 red_mask;
    u32 green_mask;
    u32 blue_mask;
    u32 reserved_mask;
};

struct efi_gop_mode_info {
    u32 version;
    u32 horizontal_resolution;
    u32 vertical_resolution;
    u32 pixel_format;
    struct efi_pixel_bitmask pixel_information;
    u32 pixels_per_scan_line;
};

struct efi_gop_mode {
    u32 max_mode;
    u32 mode;
    struct efi_gop_mode_info *info;
    usize size_of_info;
    u64 framebuffer_base;
    usize framebuffer_size;
};

struct efi_graphics_output;

struct efi_blt_pixel {
    u8 blue;
    u8 green;
    u8 red;
    u8 reserved;
};

typedef efi_status_t (*efi_gop_blt_fn)(struct efi_graphics_output *self,
                                       struct efi_blt_pixel *buffer,
                                       u32 operation,
                                       usize source_x,
                                       usize source_y,
                                       usize dest_x,
                                       usize dest_y,
                                       usize width,
                                       usize height,
                                       usize delta);

struct efi_graphics_output {
    void *query_mode;
    void *set_mode;
    void *blt;
    struct efi_gop_mode *mode;
};

struct efi_system_table {
    struct efi_table_header hdr;
    u16 *firmware_vendor;
    u32 firmware_revision;
    u32 _pad0;
    efi_handle_t console_in_handle;
    void *con_in;
    efi_handle_t console_out_handle;
    struct efi_simple_text_output *con_out;
    efi_handle_t standard_error_handle;
    struct efi_simple_text_output *std_err;
    void *runtime_services;
    struct efi_boot_services *boot_services;
};

static struct efi_simple_text_output *g_con;
static struct efi_graphics_output *g_gop;
static bool g_log_quiet;

void *memset(void *dst, int c, usize n)
{
    u8 *d = (u8 *)dst;
    while (n--) *d++ = (u8)c;
    return dst;
}

void *memcpy(void *dst, const void *src, usize n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    while (n--) *d++ = *s++;
    return dst;
}

int memcmp(const void *a, const void *b, usize n)
{
    const u8 *x = (const u8 *)a;
    const u8 *y = (const u8 *)b;
    while (n--) {
        if (*x != *y) return (int)*x - (int)*y;
        x++; y++;
    }
    return 0;
}

u32 pios_strlen(const char *s)
{
    u32 n = 0;
    while (s && s[n]) n++;
    return n;
}

static void efi_puts16(const u16 *s)
{
    if (g_con && g_con->output_string)
        g_con->output_string(g_con, s);
}

static void efi_put_ascii_raw(const char *s)
{
    u16 buf[96];
    while (s && *s) {
        u32 n = 0;
        while (*s && n + 1 < (u32)(sizeof(buf) / sizeof(buf[0]))) {
            char c = *s++;
            if (c == '\n') {
                buf[n++] = '\r';
                if (n + 1 >= (u32)(sizeof(buf) / sizeof(buf[0]))) break;
            }
            buf[n++] = (u16)c;
        }
        buf[n] = 0;
        efi_puts16(buf);
    }
}

static void efi_put_ascii(const char *s)
{
    if (!g_log_quiet)
        efi_put_ascii_raw(s);
}

void uart_putc(char c)
{
    char s[2] = { c, 0 };
    efi_put_ascii(s);
}

void uart_puts(const char *s)
{
    efi_put_ascii(s);
}

void uart_hex(u64 v)
{
    static const char h[] = "0123456789ABCDEF";
    efi_put_ascii("0x");
    for (int i = 60; i >= 0; i -= 4)
        uart_putc(h[(v >> (u32)i) & 0xFULL]);
}

void fb_puts(const char *s) { uart_puts(s); }
void fb_printf(const char *fmt, ...) { uart_puts(fmt); }
void fb_set_color(u32 fg, u32 bg) { (void)fg; (void)bg; }
void fb_clear(u32 color) { (void)color; }
void fb_putc(char c) { uart_putc(c); }

void simd_memcpy(void *dst, const void *src, usize n) { (void)memcpy(dst, src, n); }
void simd_zero(void *dst, usize n) { (void)memset(dst, 0, n); }
void simd_memset(void *dst, u8 val, usize n) { (void)memset(dst, val, n); }

u32 hw_crc32c(const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    u32 crc = 0xFFFFFFFFU;
    for (u32 i = 0; i < len; i++) {
        crc ^= p[i];
        for (u32 b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0x82F63B78U & (0U - (crc & 1U)));
    }
    return crc ^ 0xFFFFFFFFU;
}

u64 timer_ticks(void)
{
    u64 c;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(c));
    return c;
}

u64 timer_monotonic_ms(void)
{
    u64 f;
    u64 c;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(c));
    return f ? (c * 1000ULL) / f : 0;
}

bool fifo_push(u32 src_core, u32 dst_core, const struct fifo_msg *msg)
{
    (void)src_core; (void)dst_core; (void)msg;
    return false;
}

bool fifo_pop(u32 dst_core, u32 src_core, struct fifo_msg *msg)
{
    (void)dst_core; (void)src_core; (void)msg;
    return false;
}

u32 principal_current_for(u32 core)
{
    (void)core;
    return PRINCIPAL_ROOT;
}

bool principal_has_cap(u32 id, u32 cap_flag)
{
    (void)id; (void)cap_flag;
    return true;
}

static void print_line(const char *name, bool ok)
{
    uart_puts(ok ? "[ OK ] " : "[FAIL] ");
    uart_puts(name);
    uart_puts("\n");
}

static void wb_puts(const char *s)
{
    efi_put_ascii_raw(s);
}

static void wb_hex(u64 v)
{
    static const char h[] = "0123456789ABCDEF";
    wb_puts("0x");
    for (int i = 60; i >= 0; i -= 4) {
        char c[2] = { h[(v >> (u32)i) & 0xFULL], 0 };
        wb_puts(c);
    }
}

static bool gop_init(struct efi_system_table *st)
{
    static struct efi_guid gop_guid = {
        0x9042A9DEU, 0x23DCU, 0x4A38U,
        {0x96U, 0xFBU, 0x7AU, 0xDEU, 0xD0U, 0x80U, 0x51U, 0x6AU}
    };
    void *iface = 0;
    if (!st || !st->boot_services || !st->boot_services->locate_protocol)
        return false;
    if (st->boot_services->locate_protocol(&gop_guid, 0, &iface) != 0 || !iface)
        return false;
    g_gop = (struct efi_graphics_output *)iface;
    return g_gop && g_gop->mode && g_gop->mode->info && g_gop->mode->framebuffer_base;
}

static void gop_set_largest_mode(void)
{
    if (!g_gop || !g_gop->mode || !g_gop->query_mode || !g_gop->set_mode)
        return;
    typedef efi_status_t (*gop_query_fn)(struct efi_graphics_output *, u32, usize *, struct efi_gop_mode_info **);
    typedef efi_status_t (*gop_set_fn)(struct efi_graphics_output *, u32);
    gop_query_fn query = (gop_query_fn)g_gop->query_mode;
    gop_set_fn set = (gop_set_fn)g_gop->set_mode;
    u64 best_area = 0;
    u32 best_mode = g_gop->mode->mode;
    for (u32 mode = 0; mode < g_gop->mode->max_mode; mode++) {
        usize sz = 0;
        struct efi_gop_mode_info *info = 0;
        if (query(g_gop, mode, &sz, &info) != 0 || !info)
            continue;
        u64 area = (u64)info->horizontal_resolution * (u64)info->vertical_resolution;
        if (area > best_area) {
            best_area = area;
            best_mode = mode;
        }
    }
    (void)set(g_gop, best_mode);
}

static u32 gop_color(u32 rgb)
{
    u32 r = (rgb >> 16) & 0xFFU;
    u32 g = (rgb >> 8) & 0xFFU;
    u32 b = rgb & 0xFFU;
    u32 fmt = g_gop && g_gop->mode && g_gop->mode->info ? g_gop->mode->info->pixel_format : 1U;
    if (fmt == 0U) return r | (g << 8) | (b << 16);
    return b | (g << 8) | (r << 16);
}

static void gop_rect_direct(u32 x, u32 y, u32 w, u32 h, u32 rgb)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info) return;
    struct efi_gop_mode_info *mi = g_gop->mode->info;
    if (x >= mi->horizontal_resolution || y >= mi->vertical_resolution) return;
    if (w > mi->horizontal_resolution - x) w = mi->horizontal_resolution - x;
    if (h > mi->vertical_resolution - y) h = mi->vertical_resolution - y;
    volatile u32 *fb = (volatile u32 *)(usize)g_gop->mode->framebuffer_base;
    u32 color = gop_color(rgb);
    for (u32 yy = 0; yy < h; yy++)
        for (u32 xx = 0; xx < w; xx++)
            fb[(usize)(y + yy) * mi->pixels_per_scan_line + x + xx] = color;
}

static void gop_rect(u32 x, u32 y, u32 w, u32 h, u32 rgb)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info) return;
    struct efi_gop_mode_info *mi = g_gop->mode->info;
    if (x >= mi->horizontal_resolution || y >= mi->vertical_resolution) return;
    if (w > mi->horizontal_resolution - x) w = mi->horizontal_resolution - x;
    if (h > mi->vertical_resolution - y) h = mi->vertical_resolution - y;
    if (g_gop->blt) {
        efi_gop_blt_fn blt = (efi_gop_blt_fn)g_gop->blt;
        struct efi_blt_pixel px = {
            .blue = (u8)(rgb & 0xFFU),
            .green = (u8)((rgb >> 8) & 0xFFU),
            .red = (u8)((rgb >> 16) & 0xFFU),
            .reserved = 0
        };
        if (blt(g_gop, &px, 0, 0, 0, x, y, w, h, 0) == 0)
            return;
    }
    gop_rect_direct(x, y, w, h, rgb);
}

static void font_rows(char c, u8 rows[7])
{
    for (u32 i = 0; i < 7; i++) rows[i] = 0;
    switch (c) {
    case 'A': { u8 r[7]={14,17,17,31,17,17,17}; memcpy(rows,r,7); break; }
    case 'B': { u8 r[7]={30,17,17,30,17,17,30}; memcpy(rows,r,7); break; }
    case 'C': { u8 r[7]={14,17,16,16,16,17,14}; memcpy(rows,r,7); break; }
    case 'D': { u8 r[7]={30,17,17,17,17,17,30}; memcpy(rows,r,7); break; }
    case 'E': { u8 r[7]={31,16,16,30,16,16,31}; memcpy(rows,r,7); break; }
    case 'F': { u8 r[7]={31,16,16,30,16,16,16}; memcpy(rows,r,7); break; }
    case 'G': { u8 r[7]={14,17,16,23,17,17,14}; memcpy(rows,r,7); break; }
    case 'H': { u8 r[7]={17,17,17,31,17,17,17}; memcpy(rows,r,7); break; }
    case 'I': { u8 r[7]={14,4,4,4,4,4,14}; memcpy(rows,r,7); break; }
    case 'J': { u8 r[7]={7,2,2,2,18,18,12}; memcpy(rows,r,7); break; }
    case 'K': { u8 r[7]={17,18,20,24,20,18,17}; memcpy(rows,r,7); break; }
    case 'L': { u8 r[7]={16,16,16,16,16,16,31}; memcpy(rows,r,7); break; }
    case 'M': { u8 r[7]={17,27,21,21,17,17,17}; memcpy(rows,r,7); break; }
    case 'N': { u8 r[7]={17,25,21,19,17,17,17}; memcpy(rows,r,7); break; }
    case 'O': { u8 r[7]={14,17,17,17,17,17,14}; memcpy(rows,r,7); break; }
    case 'P': { u8 r[7]={30,17,17,30,16,16,16}; memcpy(rows,r,7); break; }
    case 'Q': { u8 r[7]={14,17,17,17,21,18,13}; memcpy(rows,r,7); break; }
    case 'R': { u8 r[7]={30,17,17,30,20,18,17}; memcpy(rows,r,7); break; }
    case 'S': { u8 r[7]={15,16,16,14,1,1,30}; memcpy(rows,r,7); break; }
    case 'T': { u8 r[7]={31,4,4,4,4,4,4}; memcpy(rows,r,7); break; }
    case 'U': { u8 r[7]={17,17,17,17,17,17,14}; memcpy(rows,r,7); break; }
    case 'V': { u8 r[7]={17,17,17,17,17,10,4}; memcpy(rows,r,7); break; }
    case 'W': { u8 r[7]={17,17,17,21,21,21,10}; memcpy(rows,r,7); break; }
    case 'X': { u8 r[7]={17,17,10,4,10,17,17}; memcpy(rows,r,7); break; }
    case 'Y': { u8 r[7]={17,17,10,4,4,4,4}; memcpy(rows,r,7); break; }
    case 'Z': { u8 r[7]={31,1,2,4,8,16,31}; memcpy(rows,r,7); break; }
    case '0': { u8 r[7]={14,17,19,21,25,17,14}; memcpy(rows,r,7); break; }
    case '1': { u8 r[7]={4,12,4,4,4,4,14}; memcpy(rows,r,7); break; }
    case '2': { u8 r[7]={14,17,1,2,4,8,31}; memcpy(rows,r,7); break; }
    case '3': { u8 r[7]={30,1,1,14,1,1,30}; memcpy(rows,r,7); break; }
    case '4': { u8 r[7]={2,6,10,18,31,2,2}; memcpy(rows,r,7); break; }
    case '5': { u8 r[7]={31,16,16,30,1,1,30}; memcpy(rows,r,7); break; }
    case '6': { u8 r[7]={6,8,16,30,17,17,14}; memcpy(rows,r,7); break; }
    case '7': { u8 r[7]={31,1,2,4,8,8,8}; memcpy(rows,r,7); break; }
    case '8': { u8 r[7]={14,17,17,14,17,17,14}; memcpy(rows,r,7); break; }
    case '9': { u8 r[7]={14,17,17,15,1,2,12}; memcpy(rows,r,7); break; }
    case '[': { u8 r[7]={14,8,8,8,8,8,14}; memcpy(rows,r,7); break; }
    case ']': { u8 r[7]={14,2,2,2,2,2,14}; memcpy(rows,r,7); break; }
    case '-': { u8 r[7]={0,0,0,31,0,0,0}; memcpy(rows,r,7); break; }
    case '+': { u8 r[7]={0,4,4,31,4,4,0}; memcpy(rows,r,7); break; }
    case ':': { u8 r[7]={0,4,4,0,4,4,0}; memcpy(rows,r,7); break; }
    case '.': { u8 r[7]={0,0,0,0,0,12,12}; memcpy(rows,r,7); break; }
    case '/': { u8 r[7]={1,1,2,4,8,16,16}; memcpy(rows,r,7); break; }
    case ' ': default: break;
    }
}

static void draw_char(u32 x, u32 y, char c, u32 fg, u32 scale)
{
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    u8 rows[7];
    font_rows(c, rows);
    for (u32 ry = 0; ry < 7; ry++) {
        for (u32 rx = 0; rx < 5; rx++) {
            if (rows[ry] & (1U << (4U - rx)))
                gop_rect(x + rx * scale, y + ry * scale, scale, scale, fg);
        }
    }
}

static void draw_text(u32 x, u32 y, const char *s, u32 fg, u32 scale)
{
    while (s && *s) {
        draw_char(x, y, *s++, fg, scale);
        x += 6U * scale;
    }
}

static void draw_status(u32 y, const char *label, bool ok)
{
    gop_rect(8, y - 1, 300, 11, 0x101820);
    draw_text(14, y, ok ? "[OK]" : "[FAIL]", ok ? 0x4ADE80 : 0xF87171, 1);
    draw_text(50, y, label, 0xDDE7F0, 1);
}

static void gop_render_workbench(bool sd_ok, bool fmt_ok, bool mount_ok,
                                 bool create_ok, bool write_ok, bool read_ok,
                                 bool verify_ok, u32 records, u64 uptime)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info) return;
    struct efi_gop_mode_info *mi = g_gop->mode->info;
    gop_rect(0, 0, mi->horizontal_resolution, mi->vertical_resolution, 0x050607);
    gop_rect(0, 0, 320, 22, 0x141414);
    gop_rect(0, 22, 320, 2, 0xFD8EA1);
    draw_text(8, 7, "PIOS QEMU UEFI BOOTED", 0xFD8EA1, 1);
    draw_text(8, 34, "PLATFORM: QEMU-VIRT", 0xFFFFFF, 1);
    draw_text(8, 48, "BOOT: BOOTAA64.EFI", 0xFFFFFF, 1);
    draw_text(8, 62, "STAGE2: ONE KERNEL PGS2", 0xFFFFFF, 1);

    draw_status(88, "RAM SD BLOCK DEVICE", sd_ok);
    draw_status(106, "BLOCK CACHE INIT", true);
    draw_status(124, "RAM WALFS FORMAT", fmt_ok);
    draw_status(142, "RAM WALFS MOUNT", mount_ok);
    draw_status(160, "WALFS FILE CREATE", create_ok);
    draw_status(178, "WALFS FILE WRITE", write_ok);
    draw_status(196, "WALFS READBACK", read_ok);
    draw_status(214, "WALFS VERIFY", verify_ok);

    draw_text(8, 244, "RECORDS:", 0xB0B0B0, 1);
    wb_puts(""); /* keep compiler from considering wb_hex only serial-facing */
    char rec_digit[2] = { (char)('0' + (records % 10U)), 0 };
    draw_text(62, 244, rec_digit, 0x4ADE80, 1);
    draw_text(8, 264, "PARITY: PL011 RAMBLOCK WALFS", 0xDDE7F0, 1);
    draw_text(8, 282, "PARKED - RESET VM TO REBOOT", 0xFBBF24, 1);
    (void)uptime;
}

static void park(void)
{
    wb_puts("\nPIOS QEMU UEFI stager parked. Reset VM to reboot.\n");
    for (;;) __asm__ volatile("wfe");
}

static void park_silent(void)
{
    for (;;) __asm__ volatile("wfe");
}

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    (void)image;
    g_con = st ? st->con_out : 0;
    bool gop_ok = gop_init(st);
    if (gop_ok)
        gop_set_largest_mode();
    if (g_con) {
        if (g_con->reset) (void)g_con->reset(g_con, false);
        if (g_con->set_mode) (void)g_con->set_mode(g_con, 0);
        if (g_con->set_attribute) (void)g_con->set_attribute(g_con, 0x0F);
        if (g_con->clear_screen) (void)g_con->clear_screen(g_con);
        if (g_con->set_cursor_position) (void)g_con->set_cursor_position(g_con, 0, 0);
        if (g_con->enable_cursor) (void)g_con->enable_cursor(g_con, false);
    }

    g_log_quiet = true;

    bool sd_ok = sd_init();
    if (!sd_ok) park();

    bcache_init();

    bool fmt_ok = walfs_format_reserved();
    bool mount_ok = fmt_ok && walfs_init();
    if (!mount_ok) park();

    u64 id = walfs_create(WALFS_ROOT_INODE, "uefi.txt", WALFS_FILE, 0644);
    bool create_ok = id != 0;

    static const char payload[] = "hello from BOOTAA64 PIOS\n";
    bool write_ok = create_ok && walfs_replace(id, payload, (u32)sizeof(payload) - 1U);

    char out[64];
    u32 got = write_ok ? walfs_read(id, 0, out, (u32)sizeof(out)) : 0;
    bool read_ok = got == (u32)sizeof(payload) - 1U &&
                   memcmp(out, payload, (u32)sizeof(payload) - 1U) == 0;

    struct walfs_health wh;
    bool verify_ok = walfs_verify(&wh);
    if (gop_ok) {
        gop_render_workbench(sd_ok, fmt_ok, mount_ok, create_ok, write_ok, read_ok,
                             verify_ok, verify_ok ? wh.valid_records : 0,
                             timer_monotonic_ms());
        park_silent();
    }

    g_log_quiet = false;
    if (g_con) {
        if (g_con->clear_screen) (void)g_con->clear_screen(g_con);
        if (g_con->set_cursor_position) (void)g_con->set_cursor_position(g_con, 0, 0);
    }
    wb_puts("PIOS QEMU UEFI WORKBENCH - BOOTED\n");
    print_line("RAM-backed SD block device", sd_ok);
    print_line("Block cache initialized", true);
    print_line("RAM WALFS formatted", fmt_ok);
    print_line("RAM WALFS mounted", mount_ok);
    print_line("WALFS file create", create_ok);
    print_line("WALFS file write", write_ok);
    print_line("WALFS file readback", read_ok);
    print_line("WALFS verify", verify_ok);
    park();
    return 0;
}
