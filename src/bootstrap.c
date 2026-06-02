/*
 * bootstrap.c - tiny first-stage loader for PIOS.
 *
 * Pi firmware loads this as /kernel8.img. It initializes only SD, reads the
 * real kernel from the fixed raw slot at partition-2 root, then a high-memory
 * trampoline copies it down to 0x80000 and jumps there.
 */

#include "types.h"
#include "sd.h"
#include "mmio.h"
#include "fb.h"

#define BOOT_DST_ADDR       0x00080000ULL
#define BOOT_STAGING_ADDR   0x08000000ULL
#define BOOT_TRAMP_ADDR     0x07FFF000ULL
#define BOOT_SLOT_BYTES     (2U * 1024U * 1024U)
#define BOOT_SLOT_BLOCKS    (BOOT_SLOT_BYTES / SD_BLOCK_SIZE)
#define BOOT_FALLBACK_LBA   2048U
#define BOOT_SLOT_MAGIC     0x50494F53U /* PIOS */

extern u8 bootstrap_trampoline[];
extern u8 bootstrap_trampoline_end[];

u64 l1_table[512] ALIGNED(4096);
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;
u64 el2_boot_el_state;
static u8 mbr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 hdr[SD_BLOCK_SIZE] ALIGNED(64);

void *memset(void *dst, int c, usize n)
{
    u8 *p = (u8 *)dst;
    while (n--) *p++ = (u8)c;
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

static u32 read_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

void uart_putc(char c)
{
    while (mmio_read(UART0_BASE + 0x18) & (1U << 5)) ;
    mmio_write(UART0_BASE + 0x00, (u32)c);
}

void uart_puts(const char *s)
{
    while (s && *s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

void uart_hex(u64 v)
{
    static const char hx[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (i32 i = 60; i >= 0; i -= 4)
        uart_putc(hx[(v >> (u32)i) & 0xFULL]);
}

void kernel_fb_early(void)
{
    if (fb_init(1280, 720)) {
        fb_clear(0x00000000);
        fb_set_color(0x0000FF00, 0x00000000);
        fb_puts("PIOS stage0\n");
    }
}

void kernel_el2_crash(u64 esr, u64 elr, u64 far, u64 spsr)
{
    fb_set_color(0x00FF0000, 0x00000000);
    fb_puts("stage0 EL2 crash\n");
    fb_printf("esr=%X elr=%X far=%X spsr=%X\n", esr, elr, far, spsr);
    uart_puts("[boot] EL2 crash\n");
    for (;;) wfi();
}

static u32 discover_kernel_slot_lba(void)
{
    if (!sd_read_block(0, mbr))
        return BOOT_FALLBACK_LBA;
    if (mbr[510] != 0x55 || mbr[511] != 0xAA)
        return BOOT_FALLBACK_LBA;
    u32 p2_start = read_le32(&mbr[0x1CE + 8]);
    u32 p2_size = read_le32(&mbr[0x1CE + 12]);
    if (p2_start == 0 || p2_size < BOOT_SLOT_BLOCKS)
        return BOOT_FALLBACK_LBA;
    return p2_start;
}

void core1_main(void) { for (;;) wfi(); }
void core2_main(void) { for (;;) wfi(); }
void core3_main(void) { for (;;) wfi(); }
void el2_hvc_trap(u64 a, u64 b, u64 c, u64 d) { (void)a; (void)b; (void)c; (void)d; }
void el2_sync_fault_trap(u64 a, u64 b, u64 c, u64 d) { (void)a; (void)b; (void)c; (void)d; }
void irq_dispatch(void) {}
void sync_exception(u64 esr, u64 elr, u64 far, u64 spsr)
{
    fb_set_color(0x00FF0000, 0x00000000);
    fb_printf("stage0 sync esr=%X elr=%X far=%X spsr=%X\n", esr, elr, far, spsr);
    for (;;) wfi();
}
void serror_exception(u64 esr, u64 elr, u64 far, u64 spsr)
{
    fb_set_color(0x00FF0000, 0x00000000);
    fb_printf("stage0 serr esr=%X elr=%X far=%X spsr=%X\n", esr, elr, far, spsr);
    for (;;) wfi();
}

NORETURN void kernel_main(void)
{
    uart_puts("[boot] PIOS bootstrap\n");
    fb_set_color(0x0000FF00, 0x00000000);
    fb_puts("[stage0] start\n");
    if (!sd_init()) {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_puts("[stage0] sd init failed\n");
        uart_puts("[boot] sd init failed\n");
        for (;;) wfi();
    }

    u32 slot_lba = discover_kernel_slot_lba();
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_printf("[stage0] slot LBA=%u\n", slot_lba);
    uart_puts("[boot] slot LBA=");
    uart_hex(slot_lba);
    uart_puts("\n");

    if (!sd_read_block(slot_lba, hdr)) {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_puts("[stage0] header read failed\n");
        uart_puts("[boot] header read failed\n");
        for (;;) wfi();
    }
    u32 magic = read_le32(hdr);
    u32 image_len = read_le32(hdr + 4);
    if (magic != BOOT_SLOT_MAGIC || image_len == 0 || image_len > BOOT_SLOT_BYTES - SD_BLOCK_SIZE) {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_printf("[stage0] bad header magic=%x len=%u\n", magic, image_len);
        uart_puts("[boot] bad slot header magic=");
        uart_hex(magic);
        uart_puts(" len=");
        uart_hex(image_len);
        uart_puts("\n");
        for (;;) wfi();
    }
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("[stage0] image bytes=%u\n", image_len);
    uart_puts("[boot] image bytes=");
    uart_hex(image_len);
    uart_puts("\n");

    u8 *staging = (u8 *)(usize)BOOT_STAGING_ADDR;
    u32 blocks = (image_len + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
    for (u32 i = 0; i < blocks; i++) {
        if (!sd_read_block(slot_lba + 1 + i, staging + (i * SD_BLOCK_SIZE))) {
            fb_set_color(0x00FF0000, 0x00000000);
            fb_printf("[stage0] read failed block=%u\n", i);
            uart_puts("[boot] read failed block=");
            uart_hex(i);
            uart_puts("\n");
            for (;;) wfi();
        }
    }
    fb_set_color(0x0000FF00, 0x00000000);
    fb_puts("[stage0] jumping\n");

    u8 *tramp_dst = (u8 *)(usize)BOOT_TRAMP_ADDR;
    u32 tramp_len = (u32)(bootstrap_trampoline_end - bootstrap_trampoline);
    memcpy(tramp_dst, bootstrap_trampoline, tramp_len);
    dsb();
    isb();

    uart_puts("[boot] jumping real kernel\n");
    void (*tramp)(u64, u64, u64, u64) = (void (*)(u64, u64, u64, u64))(usize)BOOT_TRAMP_ADDR;
    tramp(BOOT_DST_ADDR, BOOT_STAGING_ADDR, image_len, BOOT_DST_ADDR);
    for (;;) wfi();
}
