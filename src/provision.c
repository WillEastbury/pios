/*
 * provision.c - one-off stage-slot provisioner.
 *
 * This is built as a temporary /kernel8.img. It embeds the full PIOS kernel,
 * writes it into the fixed raw second-stage slot, then jumps to the embedded
 * kernel. No filesystem write is needed.
 */

#include "types.h"
#include "sd.h"
#include "mmio.h"
#include "walfs.h"
#include "stage2_manifest.h"

#define BOOT_DST_ADDR       0x00080000ULL
#define BOOT_STAGING_ADDR   0x08000000ULL
#define BOOT_TRAMP_ADDR     0x07FFF000ULL
#define BOOT_SLOT_BYTES     (PIOS_STAGE2_END_OFFSET + 1U)
#define BOOT_SLOT_BLOCKS    ((PIOS_STAGE2_ZONE_BYTES + SD_BLOCK_SIZE - 1U) / SD_BLOCK_SIZE)
#define BOOT_FALLBACK_LBA   2048U
#define PROVISION_WRITE_SLOT 0
#define BOOT_SLOT_MAGIC     PIOS_RESERVED_HEADER_MAGIC

extern u8 bootstrap_trampoline[];
extern u8 bootstrap_trampoline_end[];
extern const u8 provision_payload_start[];
extern const u8 provision_payload_end[];

u64 l1_table[512] ALIGNED(4096);
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;
u64 el2_boot_el_state;
static u8 mbr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 block[SD_BLOCK_SIZE] ALIGNED(64);

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

static u16 read_le16(const u8 *p)
{
    return (u16)((u16)p[0] | ((u16)p[1] << 8));
}

static u64 read_le64(const u8 *p)
{
    return (u64)read_le32(p) | ((u64)read_le32(p + 4) << 32);
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

void fb_puts(const char *s) { (void)s; }
void fb_printf(const char *fmt, ...) { (void)fmt; }

void kernel_fb_early(void) {}
void kernel_el2_crash(u64 esr, u64 elr, u64 far, u64 spsr)
{
    (void)esr; (void)elr; (void)far; (void)spsr;
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
    if (p2_start == 0 || p2_size < BOOT_SLOT_BLOCKS)
        return BOOT_FALLBACK_LBA;
    return p2_start;
}

static bool write_slot(u32 slot_lba, const u8 *payload, u32 payload_len)
{
    if (!payload || payload_len == 0 || payload_len > PIOS_STAGE2_ZONE_BYTES)
        return false;

    write_reserved_header(block, BOOT_SLOT_MAGIC, payload_len);
    if (!sd_write_block(slot_lba, block))
        return false;

    for (u32 b = 0; b < BOOT_SLOT_BLOCKS; b++) {
        u32 off = b * SD_BLOCK_SIZE;
        u32 dst_lba = slot_lba + 1 + b;
        if (off + SD_BLOCK_SIZE <= payload_len) {
            if (!sd_write_block(dst_lba, payload + off))
                return false;
        } else {
            memset(block, 0, sizeof(block));
            if (off < payload_len) {
                u32 n = payload_len - off;
                memcpy(block, payload + off, n);
            }
            if (!sd_write_block(dst_lba, block))
                return false;
        }
    }
    return true;
}

struct stage2_selection {
    u32 payload_offset;
    u32 payload_bytes;
    u32 entry_offset;
};

static bool select_pi_stage2_payload(const u8 *image, u32 image_len, struct stage2_selection *sel)
{
    if (!image || !sel)
        return false;
    sel->payload_offset = 0;
    sel->payload_bytes = image_len;
    sel->entry_offset = 0;
    if (image_len < sizeof(struct pios_stage2_manifest_header))
        return true;

    for (u32 off = 0; off + sizeof(struct pios_stage2_manifest_header) <= image_len; off += 8U) {
        const u8 *h = image + off;
        if (read_le32(h) != PIOS_STAGE2_MANIFEST_MAGIC)
            continue;
        u16 ver = read_le16(h + 4);
        u16 header_bytes = read_le16(h + 6);
        u16 entry_count = read_le16(h + 8);
        u16 entry_bytes = read_le16(h + 10);
        u32 flags = read_le32(h + 12);
        if (ver != PIOS_STAGE2_MANIFEST_VERSION ||
            header_bytes < sizeof(struct pios_stage2_manifest_header) ||
            entry_bytes < sizeof(struct pios_stage2_manifest_entry) ||
            entry_count == 0 || entry_count > 16U)
            continue;
        if ((flags & PIOS_STAGE2_MANIFEST_FLAG_PACKAGED) &&
            entry_bytes < PIOS_STAGE2_PACKAGED_ENTRY_BYTES)
            continue;
        u64 table_bytes = (u64)header_bytes + (u64)entry_count * (u64)entry_bytes;
        if (table_bytes > image_len - off)
            continue;

        for (u32 i = 0; i < entry_count; i++) {
            const u8 *e = h + header_bytes + i * (u32)entry_bytes;
            if (read_le32(e) != PIOS_STAGE2_PLATFORM_PI5)
                continue;
            u64 entry = read_le64(e + 8);
            if (flags & PIOS_STAGE2_MANIFEST_FLAG_PACKAGED) {
                u64 payload_off = read_le64(e + 64);
                u64 payload_size = read_le64(e + 72);
                u64 load_addr = read_le64(e + 80);
                if (payload_size == 0 || payload_off > image_len ||
                    payload_size > image_len - payload_off ||
                    entry >= payload_size ||
                    payload_size > PIOS_STAGE2_ZONE_BYTES ||
                    load_addr != BOOT_DST_ADDR)
                    return false;
                sel->payload_offset = (u32)payload_off;
                sel->payload_bytes = (u32)payload_size;
                sel->entry_offset = (u32)entry;
                return true;
            }
            if (entry >= image_len)
                return false;
            sel->entry_offset = (u32)entry;
            return true;
        }
        return false;
    }
    return true;
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
    const u8 *payload = provision_payload_start;
    u32 payload_len = (u32)(provision_payload_end - provision_payload_start);

    uart_puts("[prov] PIOS raw-slot provisioner\n");
    uart_puts("[prov] payload bytes=");
    uart_hex(payload_len);
    uart_puts("\n");

    if (PROVISION_WRITE_SLOT && !sd_init()) {
        uart_puts("[prov] sd init failed\n");
        for (;;) wfi();
    }

    if (PROVISION_WRITE_SLOT) {
        u32 slot_lba = discover_kernel_slot_lba();
        uart_puts("[prov] writing slot LBA=");
        uart_hex(slot_lba);
        uart_puts("\n");

        if (!write_slot(slot_lba, payload, payload_len)) {
            uart_puts("[prov] slot write failed\n");
            for (;;) wfi();
        }
        uart_puts("[prov] slot write complete\n");
    } else {
        uart_puts("[prov] write disabled; jump-only test\n");
    }

    struct stage2_selection sel;
    if (!select_pi_stage2_payload(payload, payload_len, &sel)) {
        uart_puts("[prov] stage2 package select failed\n");
        for (;;) wfi();
    }

    u8 *staging = (u8 *)(usize)BOOT_STAGING_ADDR;
    memcpy(staging, payload + sel.payload_offset, sel.payload_bytes);

    u8 *tramp_dst = (u8 *)(usize)BOOT_TRAMP_ADDR;
    u32 tramp_len = (u32)(bootstrap_trampoline_end - bootstrap_trampoline);
    memcpy(tramp_dst, bootstrap_trampoline, tramp_len);
    dsb();
    isb();

    uart_puts("[prov] jumping payload\n");
    void (*tramp)(u64, u64, u64, u64) = (void (*)(u64, u64, u64, u64))(usize)BOOT_TRAMP_ADDR;
    tramp(BOOT_DST_ADDR, BOOT_STAGING_ADDR, sel.payload_bytes, BOOT_DST_ADDR + sel.entry_offset);
    for (;;) wfi();
}
