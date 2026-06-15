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

typedef efi_status_t (*efi_text_string_fn)(void *self, const u16 *str);

struct efi_simple_text_output {
    void *reset;
    efi_text_string_fn output_string;
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
};

static struct efi_simple_text_output *g_con;

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

static void efi_put_ascii(const char *s)
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

static void park(void)
{
    uart_puts("\nPIOS QEMU UEFI stager parked. Reset VM to reboot.\n");
    for (;;) __asm__ volatile("wfe");
}

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    (void)image;
    g_con = st ? st->con_out : 0;

    efi_puts16(L"\r\n\r\n");
    uart_puts("============================================================\n");
    uart_puts(" PIOS QEMU UEFI WORKBENCH\n");
    uart_puts("============================================================\n");
    uart_puts("platform      : ");
    uart_puts(PIOS_PLATFORM_NAME);
    uart_puts("\nboot path     : BOOTAA64.EFI / QEMU EDK2\nstage2 model  : one kernel, platform entry offsets via PGS2\n\n");

    bool sd_ok = sd_init();
    print_line("RAM-backed SD block device", sd_ok);
    if (!sd_ok) park();

    bcache_init();
    print_line("Block cache initialized", true);

    bool fmt_ok = walfs_format_reserved();
    print_line("RAM WALFS formatted", fmt_ok);
    bool mount_ok = fmt_ok && walfs_init();
    print_line("RAM WALFS mounted", mount_ok);
    if (!mount_ok) park();

    u64 id = walfs_create(WALFS_ROOT_INODE, "uefi.txt", WALFS_FILE, 0644);
    bool create_ok = id != 0;
    print_line("WALFS file create", create_ok);

    static const char payload[] = "hello from BOOTAA64 PIOS\n";
    bool write_ok = create_ok && walfs_replace(id, payload, (u32)sizeof(payload) - 1U);
    print_line("WALFS file write", write_ok);

    char out[64];
    u32 got = write_ok ? walfs_read(id, 0, out, (u32)sizeof(out)) : 0;
    bool read_ok = got == (u32)sizeof(payload) - 1U &&
                   memcmp(out, payload, (u32)sizeof(payload) - 1U) == 0;
    print_line("WALFS file readback", read_ok);

    struct walfs_health wh;
    bool verify_ok = walfs_verify(&wh);
    print_line("WALFS verify", verify_ok);
    uart_puts("\nrecords       : ");
    uart_hex(verify_ok ? wh.valid_records : 0);
    uart_puts("\nuptime_ms     : ");
    uart_hex(timer_monotonic_ms());
    uart_puts("\n\nThis is a booted QEMU PIOS stager with real RAM block + WALFS services.\n");
    park();
    return 0;
}
