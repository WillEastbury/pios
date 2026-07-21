/*
 * provision_revert.c - one-off recovery provisioner.
 *
 * Built as a temporary /kernel8.img. It embeds the REVERT Pi5 stage2
 * (PIOS_PI5_STAGE2.BIN, NC .bss, no NIC wedge) and writes it into A/B
 * slot A via sd_write, then HALTS with an on-screen DONE message (no
 * reset, so it cannot re-provision in a loop). The operator then restores
 * the normal v3 stage0 to FAT kernel8.img and boots; the v3 stage0 boots
 * slot A (the revert). Slot B (old-good build) is left untouched as a
 * fallback, and bootctrl (active=A) is left untouched.
 *
 * SAFETY: payload is written first, the VALID slot header is written LAST,
 * and on any failure the header is invalidated so the v3 stage0 falls back
 * to slot B. Booted with NC-from-boot start.S so early SD DMA is coherent.
 */

#include "types.h"
#include "sd.h"
#include "mmio.h"
#include "walfs.h"
#include "stage2_manifest.h"
#include "fb.h"

#define BOOT_FALLBACK_LBA   2048U
#define BOOT_SLOT_MAGIC     PIOS_RESERVED_HEADER_MAGIC

extern const u8 provision_revert_payload_start[];
extern const u8 provision_revert_payload_end[];

u64 l1_table[512] ALIGNED(4096);
u64 l2_table_boot[512] ALIGNED(4096);
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;
u64 el2_boot_el_state;
static u8 mbr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 block[SD_BLOCK_SIZE] ALIGNED(64);
static u8 rb[SD_BLOCK_SIZE] ALIGNED(64);

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

static void write_reserved_header(u8 *out, u32 magic, u32 payload_len)
{
    memset(out, 0, SD_BLOCK_SIZE);
    write_le32(out + PIOS_HDR_MAGIC_OFF, magic);
    write_le32(out + PIOS_HDR_STAGE2_LEN_OFF, payload_len);
    write_le32(out + PIOS_HDR_LAYOUT_VERSION_OFF, PIOS_RESERVED_LAYOUT_VERSION);
    write_le32(out + PIOS_HDR_RESERVED_BYTES_OFF, PIOS_RESERVED_BYTES);
    write_le32(out + PIOS_HDR_STAGE2_OFFSET_OFF, PIOS_STAGE2_OFFSET);
    write_le32(out + PIOS_HDR_STAGE2_BYTES_OFF, PIOS_STAGE2_ZONE_BYTES);
    write_le32(out + PIOS_HDR_TCPIP_OFFSET_OFF, PIOS_TCPIP_STACK_OFFSET);
    write_le32(out + PIOS_HDR_TCPIP_BYTES_OFF, PIOS_TCPIP_STACK_BYTES);
    write_le32(out + PIOS_HDR_FIREWALL_OFFSET_OFF, PIOS_FIREWALL_CFG_OFFSET);
    write_le32(out + PIOS_HDR_FIREWALL_BYTES_OFF, PIOS_FIREWALL_CFG_BYTES);
    write_le32(out + PIOS_HDR_ADMIN_OFFSET_OFF, PIOS_ADMIN_HTTP_OFFSET);
    write_le32(out + PIOS_HDR_DEBUG_OFFSET_OFF, PIOS_KERNEL_DEBUG_OFFSET);
    write_le32(out + PIOS_HDR_USER_RECORDS_OFF, PIOS_USER_RECORDS_OFFSET);
    write_le32(out + PIOS_HDR_HOT_LOGS_OFF, PIOS_HOT_LOGS_OFFSET);
    write_le32(out + PIOS_HDR_HOT_LOGS_BYTES_OFF, PIOS_HOT_LOGS_BYTES);
    write_le32(out + PIOS_HDR_CRASHDUMP_OFF, PIOS_CRASHDUMP_OFFSET);
    write_le32(out + PIOS_HDR_FUTURE_OFF, PIOS_FUTURE_RESERVED_OFFSET);
    write_le32(out + PIOS_HDR_WALFS_OFF, PIOS_WALFS_OFFSET);
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

void fb_puts(const char *s);
void fb_printf(const char *fmt, ...);

void kernel_fb_early(void)
{
    if (fb_init(1280, 720)) {
        fb_clear(0x00000000);
        fb_set_color(0x0000FFFF, 0x00000000);
        fb_puts("PIOS revert provisioner\n");
    }
}

void kernel_el2_crash(u64 esr, u64 elr, u64 far, u64 spsr)
{
    fb_set_color(0x00FF0000, 0x00000000);
    fb_puts("prov EL2 crash\n");
    fb_printf("esr=%X elr=%X far=%X spsr=%X\n", esr, elr, far, spsr);
    uart_puts("[prov] EL2 crash\n");
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
    if (p2_start == 0 || p2_size < (PIOS_BOOT_SLOT_B_OFFSET / SD_BLOCK_SIZE))
        return BOOT_FALLBACK_LBA;
    return p2_start;
}

static NORETURN void prov_fail(const char *msg)
{
    fb_set_color(0x00FF0000, 0x00000000);
    fb_puts("\n  PROVISION FAILED:\n   ");
    fb_puts(msg);
    fb_puts("\n  Slot A header invalid -> stage0 falls back to slot B.\n");
    fb_puts("  Power off, move SD to reader.\n");
    uart_puts("[prov] FAIL: ");
    uart_puts(msg);
    uart_puts("\n");
    for (;;) wfi();
}

static NORETURN void prov_done(void)
{
    fb_set_color(0x0000FF00, 0x00000000);
    fb_puts("\n  ==========================================\n");
    fb_puts("   REVERT WRITTEN TO SLOT A -- SUCCESS\n");
    fb_puts("   build written (lease + macbdiag)\n");
    fb_puts("\n   POWER OFF and MOVE SD TO THE READER\n");
    fb_puts("  ==========================================\n");
    uart_puts("[prov] DONE: slot A committed. Power off, move SD to reader.\n");
    for (;;) wfi();
}

/* Invalidate slot header so a partial/aborted write falls back to slot B. */
static void invalidate_slot_header(u32 slot_lba)
{
    memset(block, 0, sizeof(block));
    (void)sd_write_block(slot_lba, block);
}

void core1_main(void) { for (;;) wfi(); }
void core2_main(void) { for (;;) wfi(); }
void core3_main(void) { for (;;) wfi(); }
void el2_hvc_trap(u64 a, u64 b, u64 c, u64 d) { (void)a; (void)b; (void)c; (void)d; }
void el2_sync_fault_trap(u64 a, u64 b, u64 c, u64 d) { (void)a; (void)b; (void)c; (void)d; }
void irq_dispatch(void) {}
void sync_exception(u64 esr, u64 elr, u64 far, u64 spsr) { (void)esr; (void)elr; (void)far; (void)spsr; for (;;) wfi(); }
void serror_exception(u64 esr, u64 elr, u64 far, u64 spsr) { (void)esr; (void)elr; (void)far; (void)spsr; for (;;) wfi(); }

NORETURN void kernel_main(void)
{
    const u8 *payload = provision_revert_payload_start;
    u32 payload_len = (u32)(provision_revert_payload_end - provision_revert_payload_start);

    uart_puts("[prov] PIOS revert provisioner (slot A)\n");
    uart_puts("[prov] payload bytes=");
    uart_hex(payload_len);
    uart_puts("\n");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("payload bytes=%u\n", payload_len);

    if (payload_len == 0 || payload_len > PIOS_STAGE2_ZONE_BYTES)
        prov_fail("bad payload length");

    fb_puts("init SD...\n");
    if (!sd_init())
        prov_fail("sd init failed");

    u32 slot_lba = discover_kernel_slot_lba();
    uart_puts("[prov] slot A LBA=");
    uart_hex(slot_lba);
    uart_puts("\n");
    fb_printf("slot A LBA=%u\n", slot_lba);

    /* 1) Invalidate slot A header first (commit-last safety). */
    invalidate_slot_header(slot_lba);

    /* 2) Write payload blocks (header is block 0; payload starts at +1). */
    fb_puts("writing payload (be patient ~15-30s)...\n");
    u32 blocks = (payload_len + SD_BLOCK_SIZE - 1U) / SD_BLOCK_SIZE;
    for (u32 b = 0; b < blocks; b++) {
        u32 off = b * SD_BLOCK_SIZE;
        u32 dst_lba = slot_lba + 1U + b;
        if (off + SD_BLOCK_SIZE <= payload_len) {
            if (!sd_write_block(dst_lba, payload + off))
                prov_fail("payload write failed");
        } else {
            memset(block, 0, sizeof(block));
            memcpy(block, payload + off, payload_len - off);
            if (!sd_write_block(dst_lba, block))
                prov_fail("payload tail write failed");
        }
    }
    uart_puts("[prov] payload written blocks=");
    uart_hex(blocks);
    uart_puts("\n");
    fb_printf("payload written blocks=%u\n", blocks);

    /* 3) Read-back verify first and last payload blocks. */
    fb_puts("verifying...\n");
    if (!sd_read_block(slot_lba + 1U, rb) ||
        memcmp(rb, payload, SD_BLOCK_SIZE) != 0) {
        invalidate_slot_header(slot_lba);
        prov_fail("readback block0 mismatch");
    }
    {
        u32 last = blocks - 1U;
        u32 off = last * SD_BLOCK_SIZE;
        u32 n = (off + SD_BLOCK_SIZE <= payload_len) ? SD_BLOCK_SIZE : (payload_len - off);
        if (!sd_read_block(slot_lba + 1U + last, rb) ||
            memcmp(rb, payload + off, n) != 0) {
            invalidate_slot_header(slot_lba);
            prov_fail("readback last block mismatch");
        }
    }

    /* 4) Write the VALID slot header LAST (commit). */
    write_reserved_header(block, BOOT_SLOT_MAGIC, payload_len);
    if (!sd_write_block(slot_lba, block)) {
        invalidate_slot_header(slot_lba);
        prov_fail("header write failed");
    }

    /* 5) Verify the committed header. */
    if (!sd_read_block(slot_lba, rb) ||
        read_le32(rb + PIOS_HDR_MAGIC_OFF) != BOOT_SLOT_MAGIC ||
        read_le32(rb + PIOS_HDR_STAGE2_LEN_OFF) != payload_len) {
        invalidate_slot_header(slot_lba);
        prov_fail("header verify failed");
    }

    prov_done();
}
