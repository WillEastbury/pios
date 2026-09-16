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
#include "proc_buffer.h"

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

bool proc_buffer_ref_acquire(u32 core, u64 ptr, u32 len,
                             struct proc_buffer_ref *out)
{
    (void)core;
    (void)ptr;
    (void)len;
    if (out) {
        out->slot = 0U;
        out->generation = 0U;
    }
    return false;
}

bool proc_buffer_ref_validate(u32 core, u64 ptr, u32 len,
                              const struct proc_buffer_ref *ref)
{
    (void)core;
    (void)ptr;
    (void)len;
    (void)ref;
    return false;
}

void dcache_clean_range(u64 start, u64 size)
{
    (void)start;
    (void)size;
}

void dcache_invalidate_range(u64 start, u64 size)
{
    (void)start;
    (void)size;
}

static void qemu_fail(const char *msg)
{
    uart_puts("FAIL: ");
    uart_puts(msg);
    uart_puts("\n");
    for (;;) __asm__ volatile("wfe");
}

static void qemu_write_le32(u8 *dst, u32 value)
{
    dst[0] = (u8)value;
    dst[1] = (u8)(value >> 8);
    dst[2] = (u8)(value >> 16);
    dst[3] = (u8)(value >> 24);
}

/* This smoke image has a test-only RAM disk. Pre-create its legacy p1/p2
 * partition table before the explicit WALFS format; production never creates
 * or repartitions storage. */
static bool qemu_seed_legacy_mbr(void)
{
    u8 mbr[SD_BLOCK_SIZE] = {0};
    u8 *p1 = mbr + 0x1BEU;
    u8 *p2 = p1 + 16U;

    p1[4] = 0x0CU;
    qemu_write_le32(p1 + 8U, 2048U);
    qemu_write_le32(p1 + 12U, 2048U);
    p2[4] = 0xDAU;
    qemu_write_le32(p2 + 8U, 4096U);
    qemu_write_le32(p2 + 12U, 28672U);
    mbr[510] = 0x55U;
    mbr[511] = 0xAAU;
    return sd_write_block(0U, mbr);
}

void qemu_virt_main(void)
{
    uart_puts("\nPIOS qemu-virt RAM WALFS smoke\n");
    if (!sd_init()) qemu_fail("sd_init");
    uart_puts("[smoke] seed MBR\n");
    if (!qemu_seed_legacy_mbr()) qemu_fail("seed_legacy_mbr");
    uart_puts("[smoke] bcache\n");
    bcache_init();
    uart_puts("[smoke] format\n");
    if (!walfs_format_reserved()) qemu_fail("walfs_format_reserved");
    uart_puts("[smoke] mount\n");
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
