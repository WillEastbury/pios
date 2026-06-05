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
#include "walfs.h"

#define BOOT_DST_ADDR       0x00080000ULL
#define BOOT_STAGING_ADDR   0x08000000ULL
#define BOOT_TRAMP_ADDR     0x07FFF000ULL
#define BOOT_SLOT_BYTES     (PIOS_STAGE2_END_OFFSET + 1U)
#define BOOT_SLOT_BLOCKS    (BOOT_SLOT_BYTES / SD_BLOCK_SIZE)
#define BOOT_FALLBACK_LBA   2048U
#define BOOT_SLOT_MAGIC     PIOS_RESERVED_HEADER_MAGIC
#define BOOT_WDOG_SECONDS   15U
#define PM_BASE             (PERIPH_BASE + 0x00100000UL)
#define PM_RSTC             (PM_BASE + 0x1CU)
#define PM_WDOG             (PM_BASE + 0x24U)
#define PM_PASSWORD         0x5A000000U
#define PM_RSTC_FULL        0x00000020U
#define PM_RSTC_WRCFG_MASK  0x00000030U

extern u8 bootstrap_trampoline[];
extern u8 bootstrap_trampoline_end[];

u64 l1_table[512] ALIGNED(4096);
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;
u64 el2_boot_el_state;
static u8 mbr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 hdr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 bootctl[SD_BLOCK_SIZE] ALIGNED(64);

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

static void write_le32(u8 *p, u32 v)
{
    p[0] = (u8)(v & 0xFF);
    p[1] = (u8)((v >> 8) & 0xFF);
    p[2] = (u8)((v >> 16) & 0xFF);
    p[3] = (u8)((v >> 24) & 0xFF);
}

static u32 bootctrl_checksum(const u8 *p)
{
    u32 sum = 0xB007C0DEU;
    for (u32 i = 0; i < PIOS_BOOTCTRL_CHECKSUM_OFF; i++)
        sum = (sum << 5) ^ (sum >> 27) ^ p[i];
    return sum;
}

static void stage0_watchdog_arm(u32 seconds)
{
    if (seconds == 0)
        seconds = 1;
    if (seconds > 15)
        seconds = 15;
    mmio_write(PM_WDOG, PM_PASSWORD | (seconds << 16));
    mmio_write(PM_RSTC, PM_PASSWORD |
                         (mmio_read(PM_RSTC) & ~PM_RSTC_WRCFG_MASK) |
                         PM_RSTC_FULL);
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

static u32 slot_offset(u32 slot)
{
    return slot == PIOS_BOOTCTRL_SLOT_B ? PIOS_BOOT_SLOT_B_OFFSET : PIOS_BOOT_SLOT_A_OFFSET;
}

static bool bootctrl_valid(const u8 *p)
{
    if (read_le32(p + PIOS_BOOTCTRL_MAGIC_OFF) != PIOS_BOOTCTRL_MAGIC)
        return false;
    if (read_le32(p + PIOS_BOOTCTRL_VERSION_OFF) != PIOS_BOOTCTRL_VERSION)
        return false;
    return read_le32(p + PIOS_BOOTCTRL_CHECKSUM_OFF) == bootctrl_checksum(p);
}

static void bootctrl_write(u32 root_lba, u8 *p)
{
    write_le32(p + PIOS_BOOTCTRL_CHECKSUM_OFF, bootctrl_checksum(p));
    (void)sd_write_block(root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE), p);
}

static void bootctrl_mark_fallback_a(u32 root_lba)
{
    u32 ctl_lba = root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE);
    if (!sd_read_block(ctl_lba, bootctl) || !bootctrl_valid(bootctl))
        return;
    u32 good = read_le32(bootctl + PIOS_BOOTCTRL_GOOD_MASK_OFF) |
               (1U << PIOS_BOOTCTRL_SLOT_A);
    u32 gen = read_le32(bootctl + PIOS_BOOTCTRL_GENERATION_OFF);
    write_le32(bootctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, PIOS_BOOTCTRL_SLOT_A);
    write_le32(bootctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
    write_le32(bootctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 0);
    write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, PIOS_BOOTCTRL_SLOT_A);
    write_le32(bootctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, good);
    write_le32(bootctl + PIOS_BOOTCTRL_GENERATION_OFF, gen + 1U);
    bootctrl_write(root_lba, bootctl);
}

static u32 select_kernel_slot_lba(u32 root_lba)
{
    u32 slot = PIOS_BOOTCTRL_SLOT_A;
    u32 ctl_lba = root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE);
    if (!sd_read_block(ctl_lba, bootctl) || !bootctrl_valid(bootctl))
        return root_lba + (slot_offset(slot) / SD_BLOCK_SIZE);

    u32 active = read_le32(bootctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF);
    u32 pending = read_le32(bootctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF);
    u32 tries = read_le32(bootctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF);
    u32 good = read_le32(bootctl + PIOS_BOOTCTRL_GOOD_MASK_OFF);

    if (active > PIOS_BOOTCTRL_SLOT_B || ((good & (1U << active)) == 0))
        active = PIOS_BOOTCTRL_SLOT_A;

    if (pending <= PIOS_BOOTCTRL_SLOT_B) {
        if (tries > 0) {
            slot = pending;
            write_le32(bootctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, tries - 1U);
            write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, slot);
            bootctrl_write(root_lba, bootctl);
        } else {
            slot = active;
            write_le32(bootctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
            write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, slot);
            bootctrl_write(root_lba, bootctl);
        }
    } else {
        slot = active;
        write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, slot);
        bootctrl_write(root_lba, bootctl);
    }

    return root_lba + (slot_offset(slot) / SD_BLOCK_SIZE);
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
    stage0_watchdog_arm(BOOT_WDOG_SECONDS);
    fb_set_color(0x0000FF00, 0x00000000);
    fb_puts("[stage0] start\n");
    if (!sd_init()) {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_puts("[stage0] sd init failed\n");
        uart_puts("[boot] sd init failed\n");
        for (;;) wfi();
    }

    u32 root_lba = discover_kernel_slot_lba();
    u32 slot_lba = select_kernel_slot_lba(root_lba);
    fb_set_color(0x0000CCFF, 0x00000000);
    fb_printf("[stage0] slot LBA=%u\n", slot_lba);
    uart_puts("[boot] slot LBA=");
    uart_hex(slot_lba);
    uart_puts("\n");

    if (!sd_read_block(slot_lba, hdr)) {
        if (slot_lba != root_lba) {
            fb_puts("[stage0] candidate read failed; falling back to A\n");
            uart_puts("[boot] candidate read failed; fallback A\n");
            bootctrl_mark_fallback_a(root_lba);
            slot_lba = root_lba;
            if (sd_read_block(slot_lba, hdr))
                goto have_header;
        }
        fb_set_color(0x00FF0000, 0x00000000);
        fb_puts("[stage0] header read failed\n");
        uart_puts("[boot] header read failed\n");
        for (;;) wfi();
    }
have_header:
    u32 magic = read_le32(hdr + PIOS_HDR_MAGIC_OFF);
    u32 image_len = read_le32(hdr + PIOS_HDR_STAGE2_LEN_OFF);
    u32 layout_ver = read_le32(hdr + PIOS_HDR_LAYOUT_VERSION_OFF);
    u32 stage2_off = read_le32(hdr + PIOS_HDR_STAGE2_OFFSET_OFF);
    u32 stage2_bytes = read_le32(hdr + PIOS_HDR_STAGE2_BYTES_OFF);
    bool bad_header = magic != BOOT_SLOT_MAGIC || image_len == 0 || image_len > PIOS_STAGE2_ZONE_BYTES ||
                      (layout_ver != 0 && layout_ver != PIOS_RESERVED_LAYOUT_VERSION) ||
                      (stage2_off != 0 && stage2_off != PIOS_STAGE2_OFFSET) ||
                      (stage2_bytes != 0 && stage2_bytes != PIOS_STAGE2_ZONE_BYTES);
    if (bad_header && slot_lba != root_lba) {
        fb_puts("[stage0] candidate bad; falling back to A\n");
        uart_puts("[boot] candidate bad; fallback A\n");
        bootctrl_mark_fallback_a(root_lba);
        slot_lba = root_lba;
        if (!sd_read_block(slot_lba, hdr)) {
            fb_set_color(0x00FF0000, 0x00000000);
            fb_puts("[stage0] fallback header read failed\n");
            for (;;) wfi();
        }
        magic = read_le32(hdr + PIOS_HDR_MAGIC_OFF);
        image_len = read_le32(hdr + PIOS_HDR_STAGE2_LEN_OFF);
        layout_ver = read_le32(hdr + PIOS_HDR_LAYOUT_VERSION_OFF);
        stage2_off = read_le32(hdr + PIOS_HDR_STAGE2_OFFSET_OFF);
        stage2_bytes = read_le32(hdr + PIOS_HDR_STAGE2_BYTES_OFF);
        bad_header = magic != BOOT_SLOT_MAGIC || image_len == 0 || image_len > PIOS_STAGE2_ZONE_BYTES ||
                     (layout_ver != 0 && layout_ver != PIOS_RESERVED_LAYOUT_VERSION) ||
                     (stage2_off != 0 && stage2_off != PIOS_STAGE2_OFFSET) ||
                     (stage2_bytes != 0 && stage2_bytes != PIOS_STAGE2_ZONE_BYTES);
    }
    if (bad_header) {
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
    if (layout_ver)
        fb_printf("[stage0] layout v%u walfs=0x%x\n", layout_ver, PIOS_WALFS_OFFSET);
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
