/*
 * qemu_virt_walfs.c - QEMU virt RAM-backed WALFS smoke boot.
 *
 * Links the real sd/bcache/walfs/lru stack against a tiny QEMU console and
 * service stubs. This proves the QEMU RAM block backend can host WALFS before
 * virtio-blk exists.
 */
#include "types.h"
#include "platform.h"
#include "mmio.h"
#include "sd.h"
#include "bcache.h"
#include "walfs.h"
#include "fifo.h"
#include "principal.h"

#define QEMU_UART_DR   (PIOS_UART0_BASE + 0x00)
#define QEMU_UART_FR   (PIOS_UART0_BASE + 0x18)
#define QEMU_UART_TXFF (1U << 5)

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

static void qemu_putc(char c)
{
    if (c == '\n') qemu_putc('\r');
    while (mmio_read(QEMU_UART_FR) & QEMU_UART_TXFF) { }
    mmio_write(QEMU_UART_DR, (u32)c);
}

void uart_putc(char c) { qemu_putc(c); }

void uart_puts(const char *s)
{
    while (s && *s) qemu_putc(*s++);
}

void uart_hex(u64 v)
{
    static const char h[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (i32 i = 60; i >= 0; i -= 4)
        qemu_putc(h[(v >> (u32)i) & 0xFULL]);
}

void fb_puts(const char *s) { uart_puts(s); }
void fb_printf(const char *fmt, ...) { uart_puts(fmt); }
void fb_set_color(u32 fg, u32 bg) { (void)fg; (void)bg; }
void fb_clear(u32 color) { (void)color; }
void fb_putc(char c) { qemu_putc(c); }

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

static void qemu_fail(const char *msg)
{
    uart_puts("FAIL: ");
    uart_puts(msg);
    uart_puts("\n");
    for (;;) __asm__ volatile("wfe");
}

void qemu_virt_main(void)
{
    uart_puts("\nPIOS qemu-virt RAM WALFS smoke\n");
    if (!sd_init()) qemu_fail("sd_init");
    bcache_init();
    if (!walfs_format_reserved()) qemu_fail("walfs_format_reserved");
    if (!walfs_init()) qemu_fail("walfs_init");

    u64 id = walfs_create(WALFS_ROOT_INODE, "qemu.txt", WALFS_FILE, 0644);
    if (!id) qemu_fail("walfs_create");

    static const char payload[] = "hello qemu walfs\n";
    if (!walfs_replace(id, payload, (u32)sizeof(payload) - 1U))
        qemu_fail("walfs_replace");

    char out[32];
    u32 got = walfs_read(id, 0, out, (u32)sizeof(out));
    if (got != (u32)sizeof(payload) - 1U ||
        memcmp(out, payload, (u32)sizeof(payload) - 1U) != 0)
        qemu_fail("walfs_read");

    struct walfs_health wh;
    if (!walfs_verify(&wh)) qemu_fail("walfs_verify");
    uart_puts("WALFS mounted + roundtrip OK records=");
    uart_hex(wh.valid_records);
    uart_puts("\nqemu-virt RAM WALFS smoke complete; parking\n");
    for (;;) __asm__ volatile("wfe");
}
