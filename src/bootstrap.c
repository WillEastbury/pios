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
#include "stage2_manifest.h"
#include "stage0_diag.h"
#include "board_detect.h"
#include "platform.h"
#include "mailbox.h"

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
/* QEMU's -M virt machine has NO RAM below 0x40000000 (that range is GIC/
 * UART/virtio-mmio/reserved) -- unlike real Pi5/BCM2837 hardware, which has
 * RAM starting at physical 0. Stage0 itself must therefore be linked/loaded
 * at an address inside the actual RAM window (see link_bootstrap_qemu.ld,
 * matching link_qemu_full.ld's existing 0x40080000 stage2 load address),
 * and BOOT_DST_ADDR must match stage0's own link address exactly -- the
 * self-overwriting trampoline jump (see bootstrap_trampoline.S) relies on
 * the destination being wherever stage0's own code currently lives, so
 * that by the time the copy overwrites it, execution has already moved to
 * the relocated trampoline stub. Staging/trampoline addresses mirror the
 * real-hardware pattern (positioned ~128 MiB past the RAM base) rebased
 * onto QEMU's 0x40000000 RAM start instead of real hardware's 0. */
#define BOOT_DST_ADDR       0x40080000ULL
#define BOOT_STAGING_ADDR   0x48000000ULL
#define BOOT_TRAMP_ADDR     0x47FFF000ULL
#else
#define BOOT_DST_ADDR       0x00080000ULL
#define BOOT_STAGING_ADDR   0x08000000ULL
#define BOOT_TRAMP_ADDR     0x07FFF000ULL
#endif
#define BOOT_FALLBACK_LBA   2048U
#define BOOT_SLOT_MAGIC     PIOS_RESERVED_HEADER_MAGIC
#define BOOT_WDOG_SECONDS   15U
#ifndef PIOS_STAGE0_SKIP_INITIAL_WATCHDOG
#define PIOS_STAGE0_SKIP_INITIAL_WATCHDOG 0
#endif
/* PM_BASE/UART0_BASE used to be compile-time PERIPH_BASE-derived macros,
 * which only ever matched Pi5. Now resolved once at runtime from
 * g_board_bases (see board_detect_init(), called first thing in
 * bootstrap_main()) so the SAME bootstrap image works on Pi5 or the
 * BCM2837 family (Pi3/Pi Zero 2W). */
#define PM_BASE             (g_board_bases.pm_base)
#define PM_RSTC             (PM_BASE + 0x1CU)
#define PM_WDOG             (PM_BASE + 0x24U)
#define PM_PASSWORD         0x5A000000U
#define PM_RSTC_FULL        0x00000020U
#define PM_RSTC_WRCFG_MASK  0x00000030U
#define FAT_ATTR_LFN        0x0FU
#define FAT_ATTR_VOLUME     0x08U
#define FAT_CLUSTER_EOC     0x0FFFFFF8U
#define FAT_CLUSTER_BAD     0x0FFFFFF7U
#define FAT_MAX_CHAIN_STEPS 65536U

static const u8 stage2_fat_name[11] = {
    'P','I','O','S','S','T','G','2','P','K','G'
};

extern u8 bootstrap_trampoline[];
extern u8 bootstrap_trampoline_end[];

u64 l1_table[512] ALIGNED(4096);
/* BCM2837-family boot-only L2 split for L1[0] (see bootstrap_start.S): the
 * first 1GiB is RAM (Normal-NC) except its last 16MiB, which is the
 * "low peripheral" MMIO window (0x3F000000-0x3FFFFFFF on Pi3/Pi Zero 2W)
 * and must be Device-nGnRnE. Unused/never referenced on Pi5 (whose boot
 * path keeps the original simple 1GiB block descriptors). */
u64 l2_table_bcm2837_boot[512] ALIGNED(4096);
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;
u64 el2_boot_el_state;
static u8 mbr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 hdr[SD_BLOCK_SIZE] ALIGNED(64);
static u8 bootctl[SD_BLOCK_SIZE] ALIGNED(64);
static u8 fat_sector[SD_BLOCK_SIZE] ALIGNED(64);
static volatile u32 stage0_board_revision_mbox[7] ALIGNED(16);

#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
#define STAGE0_HELLO_MBOX_BASE    0x107C013880ULL
#define STAGE0_HELLO_MBOX0_READ   (STAGE0_HELLO_MBOX_BASE + 0x00U)
#define STAGE0_HELLO_MBOX0_STATUS (STAGE0_HELLO_MBOX_BASE + 0x18U)
#define STAGE0_HELLO_MBOX1_WRITE  (STAGE0_HELLO_MBOX_BASE + 0x20U)
#define STAGE0_HELLO_MBOX1_STATUS (STAGE0_HELLO_MBOX_BASE + 0x38U)
#define STAGE0_HELLO_MBOX_FULL    0x80000000U
#define STAGE0_HELLO_MBOX_EMPTY   0x40000000U
#define STAGE0_HELLO_MBOX_CH      8U
#define STAGE0_HELLO_POLL_LIMIT   10000U

static volatile u32 ALIGNED(16) stage0_hello_mbox[8];

static void stage0_hello_clean(const volatile void *ptr, u32 bytes)
{
    u64 addr = (u64)(usize)ptr;
    for (u32 off = 0; off < bytes; off += 64U)
        __asm__ volatile("dc civac, %0" :: "r"(addr + off));
    dsb();
}

static bool stage0_hello_mbox_call(void)
{
    stage0_hello_clean(stage0_hello_mbox, sizeof(stage0_hello_mbox));
    u32 message = (u32)(usize)stage0_hello_mbox | STAGE0_HELLO_MBOX_CH;
    u32 polls = STAGE0_HELLO_POLL_LIMIT;
    while ((mmio_read(STAGE0_HELLO_MBOX1_STATUS) & STAGE0_HELLO_MBOX_FULL) &&
           polls)
        polls--;
    if (polls == 0U)
        return false;
    mmio_write(STAGE0_HELLO_MBOX1_WRITE, message);

    polls = STAGE0_HELLO_POLL_LIMIT;
    while (polls--) {
        if (mmio_read(STAGE0_HELLO_MBOX0_STATUS) & STAGE0_HELLO_MBOX_EMPTY)
            continue;
        if (mmio_read(STAGE0_HELLO_MBOX0_READ) != message)
            continue;
        stage0_hello_clean(stage0_hello_mbox, sizeof(stage0_hello_mbox));
        return stage0_hello_mbox[1] == 0x80000000U;
    }
    return false;
}

static u8 stage0_hello_glyph(char c, u32 row)
{
    static const u8 h[7] = {17,17,17,31,17,17,17};
    static const u8 e[7] = {31,16,16,30,16,16,31};
    static const u8 l[7] = {16,16,16,16,16,16,31};
    static const u8 o[7] = {14,17,17,17,17,17,14};
    static const u8 f[7] = {31,16,16,30,16,16,16};
    static const u8 r[7] = {30,17,17,30,20,18,17};
    static const u8 m[7] = {17,27,21,21,17,17,17};
    static const u8 p[7] = {30,17,17,30,16,16,16};
    static const u8 i[7] = {31,4,4,4,4,4,31};
    static const u8 s[7] = {15,16,16,14,1,1,30};
    const u8 *glyph = NULL;
    if (c == 'H') glyph = h;
    else if (c == 'E') glyph = e;
    else if (c == 'L') glyph = l;
    else if (c == 'O') glyph = o;
    else if (c == 'F') glyph = f;
    else if (c == 'R') glyph = r;
    else if (c == 'M') glyph = m;
    else if (c == 'P') glyph = p;
    else if (c == 'I') glyph = i;
    else if (c == 'S') glyph = s;
    return glyph && row < 7U ? glyph[row] : 0U;
}

void stage0_firmware_hello(void)
{
    u64 midr;
    __asm__ volatile("mrs %0, midr_el1" : "=r"(midr));
    if (((midr >> 4) & 0xFFFU) != MIDR_PARTNUM_CORTEX_A76)
        return;
    struct stage0_diag *handoff =
        (struct stage0_diag *)(usize)STAGE0_DIAG_ADDR;
    handoff->magic = STAGE0_DIAG_MAGIC;
    handoff->version = STAGE0_DIAG_VERSION;
    handoff->hello_attempted = 1U;
    handoff->hello_status = 1U;
    handoff->hello_response = 0U;
    handoff->hello_allocation_addr = 0U;
    handoff->hello_allocation_size = 0U;
    dsb();

    /* Allocation-only is the earliest proven Pi 5 property request: firmware
     * chooses the configured mode, and no stage0 translation state exists yet. */
    stage0_hello_mbox[0] = 8U * 4U;
    stage0_hello_mbox[1] = 0U;
    stage0_hello_mbox[2] = 0x00040001U;
    stage0_hello_mbox[3] = 8U;
    stage0_hello_mbox[4] = 4U;
    stage0_hello_mbox[5] = 16U;
    stage0_hello_mbox[6] = 0U;
    stage0_hello_mbox[7] = 0U;
    if (!stage0_hello_mbox_call()) {
        handoff->hello_status = 2U;
        handoff->hello_response = stage0_hello_mbox[1];
        dsb();
        return;
    }

    u32 base = stage0_hello_mbox[5] & 0x3FFFFFFFU;
    u32 size = stage0_hello_mbox[6];
    handoff->hello_status = 3U;
    handoff->hello_response = stage0_hello_mbox[1];
    handoff->hello_allocation_addr = stage0_hello_mbox[5];
    handoff->hello_allocation_size = size;
    dsb();
    const u32 width = 1920U;
    const u32 pitch = width * 4U;
    const u32 rows = 96U;
    if (base == 0U || size < pitch * rows)
        return;

    volatile u32 *fb = (volatile u32 *)(usize)base;
    for (u32 y = 0; y < rows; y++)
        for (u32 x = 0; x < width; x++)
            fb[y * width + x] = 0x00000000U;

    static const char text[] = "HELLO FROM PIOS";
    const u32 scale = 3U;
    u32 cursor = 32U;
    for (u32 n = 0; text[n]; n++) {
        for (u32 gy = 0; gy < 7U; gy++) {
            u8 bits = stage0_hello_glyph(text[n], gy);
            for (u32 gx = 0; gx < 5U; gx++) {
                if ((bits & (1U << (4U - gx))) == 0U)
                    continue;
                for (u32 sy = 0; sy < scale; sy++)
                    for (u32 sx = 0; sx < scale; sx++)
                        fb[(24U + gy * scale + sy) * width +
                           cursor + gx * scale + sx] = 0x0000FF00U;
            }
        }
        cursor += 6U * scale;
    }
    stage0_hello_clean((const volatile void *)(usize)base, pitch * rows);
}
#else
void stage0_firmware_hello(void) {}
#endif

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

static void write_reserved_header(u8 *out, u32 payload_len)
{
    memset(out, 0, SD_BLOCK_SIZE);
    write_le32(out + PIOS_HDR_MAGIC_OFF, BOOT_SLOT_MAGIC);
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

static u32 stage0_platform_id(void)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* A stage0 binary compiled for QEMU_VIRT only ever runs under QEMU
     * (its own separately-built kernel8_qemu.img, distinct from the real-
     * hardware kernel8.img) -- QEMU's `-cpu cortex-a53` reports the SAME
     * MIDR_EL1 PartNum as real BCM2837 hardware, so runtime MIDR detection
     * cannot (and must not) be used to distinguish them here. Report the
     * platform id unconditionally instead of consulting g_board_family. */
    return PIOS_STAGE2_PLATFORM_QEMU_VIRT;
#else
    /* g_board_family is populated by board_detect_init(), called from
     * bootstrap_start.S before this point (and again, idempotently, at the
     * top of bootstrap_main()). Falls back to PI5 (the proven, currently
     * shipping path) if detection is somehow inconclusive, rather than an
     * undefined/zero platform id that would match no manifest entry. */
    if (g_board_family == BOARD_FAMILY_BCM2837) {
        /* MIDR distinguishes the BCM2837 family from Pi 5, but not Pi 3
         * versus Zero 2 W. Use the firmware board-revision model field to
         * select the payload whose WiFi power/core profile matches the host. */
        u64 base = g_board_bases.mbox_base;
        if (base != 0U) {
            volatile u32 *m = stage0_board_revision_mbox;
            m[0] = sizeof(stage0_board_revision_mbox);
            m[1] = 0U;
            m[2] = TAG_GET_BOARD_REV;
            m[3] = 4U;
            m[4] = 0U;
            m[5] = 0U;
            m[6] = TAG_END;
            stage0_hello_clean(m, sizeof(stage0_board_revision_mbox));
            u32 msg = ((u32)(u64)m & 0xFFFFFFF0U) | MBOX_CH_PROP;
            bool sent = false;
            for (u32 i = 0U; i < 10000U; i++) {
                if (!(mmio_read(base + 0x38U) & 0x80000000U)) {
                    mmio_write(base + 0x20U, msg);
                    sent = true;
                    break;
                }
            }
            if (sent) {
                for (u32 i = 0U; i < 10000U; i++) {
                    if (mmio_read(base + 0x18U) & 0x40000000U)
                        continue;
                    if (mmio_read(base + 0x00U) == msg) {
                        stage0_hello_clean(m, sizeof(stage0_board_revision_mbox));
                        if (m[1] == 0x80000000U &&
                            (m[4] & 0x80000000U)) {
                            if (board_model_from_revision(m[5]) ==
                                BOARD_MODEL_ZERO2W)
                                return PIOS_STAGE2_PLATFORM_PIZERO2W;
                        }
                        break;
                    }
                }
            }
        }
        return PIOS_STAGE2_PLATFORM_BCM2837_FAMILY;
    }
    return PIOS_STAGE2_PLATFORM_PI5;
#endif
}

struct stage2_selection {
    u32 payload_offset;
    u32 payload_bytes;
    u32 entry_offset;
};

static bool select_stage2_image(const u8 *image, u32 image_len, struct stage2_selection *sel)
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

        u32 want = stage0_platform_id();
        for (u32 i = 0; i < entry_count; i++) {
            const u8 *e = h + header_bytes + i * (u32)entry_bytes;
            u32 platform = read_le32(e);
            u64 entry = read_le64(e + 8);
            if (platform != want)
                continue;
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
            sel->payload_offset = 0;
            sel->payload_bytes = image_len;
            sel->entry_offset = (u32)entry;
            return true;
        }
        return false;
    }
    return true;
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
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* No Broadcom PM/watchdog block exists under QEMU's -M virt memory map
     * (PIOS_MBOX_BASE/PIOS_QA7_BASE are both 0 for this platform in
     * platform.h) -- board_detect_init()'s MIDR-based family detection
     * would otherwise misidentify QEMU's cortex-a53 as real BCM2837
     * hardware and populate g_board_bases.pm_base with an address that
     * doesn't exist here, faulting on first write. QEMU boots don't need
     * (or have) a hardware watchdog; a hung boot is caught by the test
     * harness's own timeout instead. */
    (void)seconds;
#else
    if (seconds == 0)
        seconds = 1;
    if (seconds > 15)
        seconds = 15;
    mmio_write(PM_WDOG, PM_PASSWORD | (seconds << 16));
    mmio_write(PM_RSTC, PM_PASSWORD |
                         (mmio_read(PM_RSTC) & ~PM_RSTC_WRCFG_MASK) |
                         PM_RSTC_FULL);
#endif
}

static void stage0_watchdog_disable(void)
{
#if PIOS_PLATFORM != PIOS_PLATFORM_QEMU_VIRT
    mmio_write(PM_WDOG, PM_PASSWORD);
    mmio_write(PM_RSTC, PM_PASSWORD |
                         (mmio_read(PM_RSTC) & ~PM_RSTC_WRCFG_MASK));
#endif
}

static void stage0_publish_framebuffer(void)
{
    u64 base;
    u32 width;
    u32 height;
    u32 pitch;
    u32 size;
    fb_display_info(&base, &width, &height, &pitch, &size);
    if (base && width && height && pitch && size)
        stage0_fb_handoff_publish(base, width, height, pitch, size);
}

void uart_putc(char c)
{
    while (mmio_read(g_board_bases.uart0_base + 0x18) & (1U << 5)) ;
    mmio_write(g_board_bases.uart0_base + 0x00, (u32)c);
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

static bool stage0_fb_ready;
static u32 stage0_fb_attempts;

static void stage0_diag_putc(char c)
{
    /* GPIO14/15 is on RP1 on Pi 5. Stage0 does not own RP1 yet, so touching
     * its UART before PCIe/RP1 initialization can external-abort. Pi 5 logs
     * are carried by the Normal-NC handoff record and printed by stage2. */
    if (g_board_family == BOARD_FAMILY_PI5)
        return;
    uart_putc(c);
}

static void stage0_diag_puts(const char *s)
{
    while (s && *s) {
        if (*s == '\n')
            stage0_diag_putc('\r');
        stage0_diag_putc(*s++);
    }
}

static void stage0_diag_hex(u64 v)
{
    static const char hex[] = "0123456789ABCDEF";
    stage0_diag_puts("0x");
    for (i32 shift = 60; shift >= 0; shift -= 4)
        stage0_diag_putc(hex[(v >> (u32)shift) & 0xFULL]);
}

void kernel_fb_early(void)
{
    u64 current_el;
    u64 sctlr_el1;
    struct stage0_diag *handoff =
        (struct stage0_diag *)(usize)STAGE0_DIAG_ADDR;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(current_el));
    __asm__ volatile("mrs %0, SCTLR_EL1" : "=r"(sctlr_el1));
    if (stage0_fb_attempts == 0U &&
        (handoff->magic != STAGE0_DIAG_MAGIC ||
         handoff->version != STAGE0_DIAG_VERSION)) {
        memset(handoff, 0, sizeof(*handoff));
        handoff->magic = STAGE0_DIAG_MAGIC;
        handoff->version = STAGE0_DIAG_VERSION;
    }
    u32 attempt_index = stage0_fb_attempts;
    stage0_fb_attempts++;
    handoff->attempt_count = stage0_fb_attempts;
    stage0_diag_puts("[boot] fb try=");
    stage0_diag_hex(stage0_fb_attempts);
    stage0_diag_puts(" EL=");
    stage0_diag_hex((current_el >> 2) & 3U);
    stage0_diag_puts(" SCTLR1=");
    stage0_diag_hex(sctlr_el1);
    stage0_diag_puts("\n");

    bool fb_ok = fb_init(1280, 720);
    struct fb_mailbox_diag diag;
    fb_mailbox_diag(&diag);
    if (attempt_index < STAGE0_DIAG_ATTEMPTS) {
        struct stage0_fb_attempt *attempt = &handoff->attempt[attempt_index];
        attempt->sctlr_el1 = sctlr_el1;
        attempt->current_el = (u32)((current_el >> 2) & 3U);
        attempt->status = diag.status;
        attempt->message = diag.message;
        attempt->response = diag.response;
        attempt->allocation_addr = diag.allocation_addr;
        attempt->allocation_size = diag.allocation_size;
        attempt->pitch = diag.pitch;
    }
    dsb();
    if (fb_ok) {
        stage0_fb_ready = true;
        handoff->framebuffer_ready = 1U;
        fb_clear(0x00000000);
        fb_set_color(0x0000FF00, 0x00000000);
        fb_puts("PIOS stage0\n");
        stage0_diag_puts("[boot] fb online\n");
        dsb();
        return;
    }

    stage0_diag_puts("[boot] fb failed status=");
    stage0_diag_hex(diag.status);
    stage0_diag_puts(" msg=");
    stage0_diag_hex(diag.message);
    stage0_diag_puts(" rsp=");
    stage0_diag_hex(diag.response);
    stage0_diag_puts(" alloc=");
    stage0_diag_hex(diag.allocation_addr);
    stage0_diag_puts("/");
    stage0_diag_hex(diag.allocation_size);
    stage0_diag_puts(" pitch=");
    stage0_diag_hex(diag.pitch);
    stage0_diag_puts("\n");
}

void kernel_el2_crash(u64 esr, u64 elr, u64 far, u64 spsr)
{
    fb_set_color(0x00FF0000, 0x00000000);
    fb_puts("stage0 EL2 crash\n");
    fb_printf("esr=%X elr=%X far=%X spsr=%X\n", esr, elr, far, spsr);
    uart_puts("[boot] EL2 crash\n");
    for (;;) wfi();
}

static bool discover_kernel_partition(u32 *root_lba)
{
    if (!root_lba || !sd_read_block(0, mbr))
        return false;
    if (mbr[510] != 0x55 || mbr[511] != 0xAA)
        return false;
    u32 p2_start = read_le32(&mbr[0x1CE + 8]);
    u32 p2_size = read_le32(&mbr[0x1CE + 12]);
    u64 required_bytes = (u64)PIOS_BOOT_SLOT_B_OFFSET +
                         PIOS_BOOT_SLOT_BYTES;
    u64 required_sectors = (required_bytes + SD_BLOCK_SIZE - 1U) /
                           SD_BLOCK_SIZE;
    if (p2_start == 0 || p2_size < required_sectors ||
        (u64)p2_start + p2_size > 0x100000000ULL)
        return false;
    *root_lba = p2_start;
    return true;
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

static bool bootctrl_write(u32 root_lba, u8 *p)
{
    write_le32(p + PIOS_BOOTCTRL_CHECKSUM_OFF, bootctrl_checksum(p));
    return sd_write_block(root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE), p);
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
    (void)bootctrl_write(root_lba, bootctl);
}

struct fat32_ro {
    u32 part_lba;
    u32 part_sectors;
    u32 fat_lba;
    u32 fat_sectors;
    u32 data_lba;
    u32 data_sectors;
    u32 root_cluster;
    u32 sectors_per_cluster;
    u32 cluster_count;
};

static bool fat32_cluster_valid(const struct fat32_ro *fs, u32 cluster)
{
    return fs && cluster >= 2U && cluster < fs->cluster_count + 2U;
}

static bool fat32_cluster_lba(const struct fat32_ro *fs, u32 cluster, u32 *lba)
{
    if (!lba || !fat32_cluster_valid(fs, cluster))
        return false;
    u64 rel = (u64)(cluster - 2U) * fs->sectors_per_cluster;
    if (rel >= fs->data_sectors)
        return false;
    u64 value = (u64)fs->data_lba + rel;
    if (value > 0xFFFFFFFFULL)
        return false;
    *lba = (u32)value;
    return true;
}

static bool fat32_next_cluster(const struct fat32_ro *fs, u32 cluster, u32 *next)
{
    if (!next || !fat32_cluster_valid(fs, cluster))
        return false;
    u64 off = (u64)cluster * 4U;
    u64 sector = (u64)fs->fat_lba + (off / SD_BLOCK_SIZE);
    if (sector >= (u64)fs->fat_lba + fs->fat_sectors ||
        sector > 0xFFFFFFFFULL)
        return false;
    if (!sd_read_block((u32)sector, fat_sector))
        return false;
    u32 pos = (u32)(off % SD_BLOCK_SIZE);
    if (pos > SD_BLOCK_SIZE - 4U)
        return false;
    *next = read_le32(fat_sector + pos) & 0x0FFFFFFFU;
    return true;
}

static bool fat32_mount(struct fat32_ro *fs)
{
    if (!fs || !sd_read_block(0, mbr))
        return false;
    if (mbr[510] != 0x55 || mbr[511] != 0xAA)
        return false;

    u32 part_lba = read_le32(&mbr[0x1BE + 8]);
    u32 part_sectors = read_le32(&mbr[0x1BE + 12]);
    if (part_lba == 0 || part_sectors < 128U ||
        !sd_read_block(part_lba, fat_sector))
        return false;
    if (fat_sector[510] != 0x55 || fat_sector[511] != 0xAA)
        return false;

    u32 bytes_per_sector = read_le16(fat_sector + 11);
    u32 sectors_per_cluster = fat_sector[13];
    u32 reserved = read_le16(fat_sector + 14);
    u32 fats = fat_sector[16];
    u32 total16 = read_le16(fat_sector + 19);
    u32 total32 = read_le32(fat_sector + 32);
    u32 fat16 = read_le16(fat_sector + 22);
    u32 fat32 = read_le32(fat_sector + 36);
    u32 root_cluster = read_le32(fat_sector + 44) & 0x0FFFFFFFU;
    u32 total = total16 ? total16 : total32;

    if (bytes_per_sector != SD_BLOCK_SIZE ||
        sectors_per_cluster == 0 ||
        (sectors_per_cluster & (sectors_per_cluster - 1U)) != 0 ||
        sectors_per_cluster > 128U || reserved == 0 ||
        fats == 0 || fats > 2U || fat16 != 0 || fat32 == 0 ||
        total == 0 || total > part_sectors)
        return false;

    u64 fat_span = (u64)fats * fat32;
    u64 data_off = (u64)reserved + fat_span;
    if (data_off >= total)
        return false;
    u32 data_sectors = total - (u32)data_off;
    u32 cluster_count = data_sectors / sectors_per_cluster;
    if (cluster_count == 0 || root_cluster < 2U ||
        root_cluster >= cluster_count + 2U)
        return false;

    u64 part_end = (u64)part_lba + part_sectors;
    u64 fat_lba = (u64)part_lba + reserved;
    u64 data_lba = (u64)part_lba + data_off;
    if (part_end > 0x100000000ULL || fat_lba > 0xFFFFFFFFULL ||
        data_lba > 0xFFFFFFFFULL ||
        data_lba + data_sectors > part_end)
        return false;

    fs->part_lba = part_lba;
    fs->part_sectors = part_sectors;
    fs->fat_lba = (u32)fat_lba;
    fs->fat_sectors = fat32;
    fs->data_lba = (u32)data_lba;
    fs->data_sectors = data_sectors;
    fs->root_cluster = root_cluster;
    fs->sectors_per_cluster = sectors_per_cluster;
    fs->cluster_count = cluster_count;
    return true;
}

static bool fat32_find_stage2(const struct fat32_ro *fs, u32 *first_cluster,
                              u32 *file_size)
{
    if (!fs || !first_cluster || !file_size)
        return false;
    u32 cluster = fs->root_cluster;
    u32 budget = fs->cluster_count;
    if (budget > FAT_MAX_CHAIN_STEPS)
        budget = FAT_MAX_CHAIN_STEPS;

    while (budget--) {
        u32 base_lba = 0;
        if (!fat32_cluster_lba(fs, cluster, &base_lba))
            return false;
        for (u32 sector = 0; sector < fs->sectors_per_cluster; sector++) {
            if (!sd_read_block(base_lba + sector, fat_sector))
                return false;
            for (u32 off = 0; off < SD_BLOCK_SIZE; off += 32U) {
                const u8 *entry = fat_sector + off;
                if (entry[0] == 0x00)
                    return false;
                if (entry[0] == 0xE5 || entry[11] == FAT_ATTR_LFN ||
                    (entry[11] & FAT_ATTR_VOLUME))
                    continue;
                if (memcmp(entry, stage2_fat_name, sizeof(stage2_fat_name)) != 0)
                    continue;
                u32 first = ((u32)read_le16(entry + 20) << 16) |
                            read_le16(entry + 26);
                u32 size = read_le32(entry + 28);
                if (!fat32_cluster_valid(fs, first) || size == 0 ||
                    size > PIOS_FAT_PACKAGE_MAX_BYTES)
                    return false;
                *first_cluster = first;
                *file_size = size;
                return true;
            }
        }
        u32 next = 0;
        if (!fat32_next_cluster(fs, cluster, &next))
            return false;
        if (next >= FAT_CLUSTER_EOC || next == FAT_CLUSTER_BAD)
            return false;
        if (!fat32_cluster_valid(fs, next) || next == cluster)
            return false;
        cluster = next;
    }
    return false;
}

static bool fat32_load_file(const struct fat32_ro *fs, u32 first_cluster,
                            u32 file_size, u8 *dst)
{
    if (!fs || !dst || !fat32_cluster_valid(fs, first_cluster) ||
        file_size == 0 || file_size > PIOS_FAT_PACKAGE_MAX_BYTES)
        return false;

    u32 remaining = file_size;
    u32 cluster = first_cluster;
    u32 budget = (file_size + fs->sectors_per_cluster * SD_BLOCK_SIZE - 1U) /
                 (fs->sectors_per_cluster * SD_BLOCK_SIZE);
    if (budget == 0 || budget > FAT_MAX_CHAIN_STEPS)
        return false;

    while (remaining && budget--) {
        u32 base_lba = 0;
        if (!fat32_cluster_lba(fs, cluster, &base_lba))
            return false;
        for (u32 sector = 0;
             sector < fs->sectors_per_cluster && remaining;
             sector++) {
            u32 take = remaining > SD_BLOCK_SIZE ? SD_BLOCK_SIZE : remaining;
            if (take == SD_BLOCK_SIZE) {
                if (!sd_read_block(base_lba + sector, dst))
                    return false;
            } else {
                if (!sd_read_block(base_lba + sector, fat_sector))
                    return false;
                memcpy(dst, fat_sector, take);
            }
            dst += take;
            remaining -= take;
            stage0_watchdog_arm(BOOT_WDOG_SECONDS);
        }
        if (!remaining)
            return true;
        u32 next = 0;
        if (!fat32_next_cluster(fs, cluster, &next) ||
            next >= FAT_CLUSTER_EOC || next == FAT_CLUSTER_BAD ||
            !fat32_cluster_valid(fs, next) || next == cluster)
            return false;
        cluster = next;
    }
    return remaining == 0;
}

static bool package_identity(const u8 *image, u32 image_len, u64 *package_id)
{
    if (!image || !package_id ||
        image_len < sizeof(struct pios_stage2_manifest_header) ||
        read_le32(image) != PIOS_STAGE2_MANIFEST_MAGIC ||
        read_le16(image + 4) != PIOS_STAGE2_MANIFEST_VERSION ||
        (read_le32(image + 12) & PIOS_STAGE2_MANIFEST_FLAG_PACKAGED) == 0)
        return false;
    u64 id = read_le64(image + 16);
    u64 declared = read_le64(image + 24);
    if (id == 0 || declared != image_len)
        return false;
    u64 computed = 0xCBF29CE484222325ULL;
    for (u32 i = 0; i < image_len; i++) {
        u8 byte = (i >= 16U && i < 24U) ? 0U : image[i];
        computed ^= byte;
        computed *= 0x100000001B3ULL;
        if ((i & 0xFFFFU) == 0)
            stage0_watchdog_arm(BOOT_WDOG_SECONDS);
    }
    if (computed == 0)
        computed = 1;
    if (computed != id)
        return false;
    *package_id = id;
    return true;
}

static bool slot_package_matches(u32 slot_lba, const u8 *image,
                                 u32 image_len, bool valid_header)
{
    if (!image || image_len == 0 || image_len > PIOS_STAGE2_ZONE_BYTES ||
        !sd_read_block(slot_lba, hdr))
        return false;
    u32 len = read_le32(hdr + PIOS_HDR_STAGE2_LEN_OFF);
    u32 want_magic = valid_header ? BOOT_SLOT_MAGIC : 0U;
    if (read_le32(hdr + PIOS_HDR_MAGIC_OFF) != want_magic ||
        len != image_len)
        return false;

    u32 blocks = (image_len + SD_BLOCK_SIZE - 1U) / SD_BLOCK_SIZE;
    for (u32 block = 0; block < blocks; block++) {
        u32 off = block * SD_BLOCK_SIZE;
        u32 take = image_len - off;
        if (take > SD_BLOCK_SIZE)
            take = SD_BLOCK_SIZE;
        if (!sd_read_block(slot_lba + 1U + block, fat_sector) ||
            memcmp(fat_sector, image + off, take) != 0)
            return false;
        stage0_watchdog_arm(BOOT_WDOG_SECONDS);
    }
    return true;
}

static bool write_slot_package(u32 slot_lba, const u8 *image, u32 image_len)
{
    if (!image || image_len == 0 || image_len > PIOS_STAGE2_ZONE_BYTES)
        return false;
    write_reserved_header(hdr, image_len);
    write_le32(hdr + PIOS_HDR_MAGIC_OFF, 0U);
    if (!sd_write_block(slot_lba, hdr))
        return false;

    u32 blocks = (image_len + SD_BLOCK_SIZE - 1U) / SD_BLOCK_SIZE;
    for (u32 block = 0; block < blocks; block++) {
        u32 off = block * SD_BLOCK_SIZE;
        u32 take = image_len - off;
        if (take > SD_BLOCK_SIZE)
            take = SD_BLOCK_SIZE;
        const u8 *src = image + off;
        if (take == SD_BLOCK_SIZE) {
            if (!sd_write_block(slot_lba + 1U + block, src))
                return false;
        } else {
            memset(fat_sector, 0, sizeof(fat_sector));
            memcpy(fat_sector, src, take);
            if (!sd_write_block(slot_lba + 1U + block, fat_sector))
                return false;
        }
        stage0_watchdog_arm(BOOT_WDOG_SECONDS);
    }
    if (!slot_package_matches(slot_lba, image, image_len, false))
        return false;
    write_reserved_header(hdr, image_len);
    if (!sd_write_block(slot_lba, hdr))
        return false;
    return slot_package_matches(slot_lba, image, image_len, true);
}

static bool bootctrl_activate_slot_a(u32 root_lba)
{
    u32 ctl_lba = root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE);
    if (!sd_read_block(ctl_lba, bootctl) || !bootctrl_valid(bootctl)) {
        memset(bootctl, 0, sizeof(bootctl));
        write_le32(bootctl + PIOS_BOOTCTRL_MAGIC_OFF, PIOS_BOOTCTRL_MAGIC);
        write_le32(bootctl + PIOS_BOOTCTRL_VERSION_OFF, PIOS_BOOTCTRL_VERSION);
    }
    u32 generation = read_le32(bootctl + PIOS_BOOTCTRL_GENERATION_OFF);
    u32 good = read_le32(bootctl + PIOS_BOOTCTRL_GOOD_MASK_OFF) |
               (1U << PIOS_BOOTCTRL_SLOT_A);
    write_le32(bootctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF,
               PIOS_BOOTCTRL_SLOT_A);
    write_le32(bootctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF,
               PIOS_BOOTCTRL_SLOT_NONE);
    write_le32(bootctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 0U);
    write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF,
               PIOS_BOOTCTRL_SLOT_A);
    write_le32(bootctl + PIOS_BOOTCTRL_GOOD_MASK_OFF, good);
    write_le32(bootctl + PIOS_BOOTCTRL_GENERATION_OFF, generation + 1U);
    return bootctrl_write(root_lba, bootctl);
}

static bool stage0_apply_fat_update(u32 root_lba)
{
    struct fat32_ro fs;
    u32 first_cluster = 0;
    u32 file_size = 0;
    u8 *staging = (u8 *)(usize)BOOT_STAGING_ADDR;
    if (!fat32_mount(&fs) ||
        !fat32_find_stage2(&fs, &first_cluster, &file_size))
        return false;

    fb_puts("[stage0] FAT update found\n");
    uart_puts("[boot] FAT PIOSSTG2.PKG found\n");
    if (!fat32_load_file(&fs, first_cluster, file_size, staging)) {
        fb_puts("[stage0] FAT update read failed\n");
        uart_puts("[boot] FAT update read failed\n");
        return false;
    }

    u64 update_id = 0;
    struct stage2_selection selection;
    if (!package_identity(staging, file_size, &update_id) ||
        !select_stage2_image(staging, file_size, &selection)) {
        fb_puts("[stage0] FAT package invalid\n");
        uart_puts("[boot] FAT package invalid\n");
        return false;
    }

    /* Write only the SELECTED platform's payload subset into the raw slot,
     * not the whole (potentially multi-platform) FAT package -- the raw
     * slot is sized for one platform's payload (PIOS_STAGE2_ZONE_BYTES,
     * ~3.5 MiB) and stays that size regardless of how many platforms the
     * FAT-resident PIOSSTG2.PKG carries. This is what makes it possible for
     * ONE PIOSSTG2.PKG to hold both a Pi5 and a BCM2837-family payload
     * (bigger than any single raw slot could hold) while this stage0
     * loader -- now genuinely multi-platform itself, see board_detect.c --
     * still only ever writes the one payload matching the board it's
     * actually running on. */
    const u8 *payload = staging + selection.payload_offset;
    u32 payload_len = selection.payload_bytes;

    u32 target_lba = root_lba +
                     (slot_offset(PIOS_BOOTCTRL_SLOT_A) / SD_BLOCK_SIZE);
    if (slot_package_matches(target_lba, payload, payload_len, true)) {
        uart_puts("[boot] FAT package already cached in slot A\n");
        return true;
    }

    fb_puts("[stage0] installing FAT package to slot A\n");
    uart_puts("[boot] installing FAT package id=");
    uart_hex(update_id);
    uart_puts("\n");
    if (!write_slot_package(target_lba, payload, payload_len) ||
        !bootctrl_activate_slot_a(root_lba)) {
        fb_puts("[stage0] FAT install failed\n");
        uart_puts("[boot] FAT install failed\n");
        return false;
    }
    fb_puts("[stage0] FAT install complete\n");
    uart_puts("[boot] FAT install complete\n");
    return true;
}

static bool slot_header_bootable(u32 slot_lba)
{
    if (!sd_read_block(slot_lba, hdr))
        return false;
    u32 magic = read_le32(hdr + PIOS_HDR_MAGIC_OFF);
    u32 image_len = read_le32(hdr + PIOS_HDR_STAGE2_LEN_OFF);
    u32 layout_ver = read_le32(hdr + PIOS_HDR_LAYOUT_VERSION_OFF);
    u32 stage2_off = read_le32(hdr + PIOS_HDR_STAGE2_OFFSET_OFF);
    u32 stage2_bytes = read_le32(hdr + PIOS_HDR_STAGE2_BYTES_OFF);
    return magic == BOOT_SLOT_MAGIC &&
           image_len > 0 && image_len <= PIOS_STAGE2_ZONE_BYTES &&
           (layout_ver == 0 || layout_ver == PIOS_RESERVED_LAYOUT_VERSION) &&
           (stage2_off == 0 || stage2_off == PIOS_STAGE2_OFFSET) &&
           (stage2_bytes == 0 || stage2_bytes == PIOS_STAGE2_ZONE_BYTES);
}

/* A staged OTA update has precedence over the FAT bootstrap package. The FAT
 * file is an installation/recovery source, not an authority that should
 * overwrite a valid raw slot on every reboot. Validate pending first, then the
 * known-good active slot, before suppressing FAT recovery. */
static bool bootctrl_has_bootable_raw_slot(u32 root_lba)
{
    u32 ctl_lba = root_lba + (PIOS_BOOTCTRL_OFFSET / SD_BLOCK_SIZE);
    if (!sd_read_block(ctl_lba, bootctl) || !bootctrl_valid(bootctl))
        return slot_header_bootable(root_lba +
                                    (slot_offset(PIOS_BOOTCTRL_SLOT_A) /
                                     SD_BLOCK_SIZE));
    u32 pending = read_le32(bootctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF);
    u32 tries = read_le32(bootctl + PIOS_BOOTCTRL_TRIES_LEFT_OFF);
    if (pending <= PIOS_BOOTCTRL_SLOT_B && tries > 0) {
        u32 pending_lba = root_lba + (slot_offset(pending) / SD_BLOCK_SIZE);
        if (slot_header_bootable(pending_lba))
            return true;
    }

    u32 active = read_le32(bootctl + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF);
    u32 good = read_le32(bootctl + PIOS_BOOTCTRL_GOOD_MASK_OFF);
    if (active <= PIOS_BOOTCTRL_SLOT_B && (good & (1U << active))) {
        u32 active_lba = root_lba + (slot_offset(active) / SD_BLOCK_SIZE);
        if (slot_header_bootable(active_lba))
            return true;
    }
    return false;
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
            if (!bootctrl_write(root_lba, bootctl))
                slot = active;
        } else {
            slot = active;
            write_le32(bootctl + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_NONE);
            write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, slot);
            (void)bootctrl_write(root_lba, bootctl);
        }
    } else {
        slot = active;
        write_le32(bootctl + PIOS_BOOTCTRL_LAST_BOOT_OFF, slot);
        (void)bootctrl_write(root_lba, bootctl);
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

NORETURN void bootstrap_main(void)
{
    /* Already called once from bootstrap_start.S (before the L1 page table
     * was built, since the table itself must differ by board -- see the
     * dual-path L1 construction there) but calling it again here is cheap,
     * deterministic, and idempotent; do so defensively so board detection
     * is never silently skipped if the asm path is ever refactored. */
    board_detect_init();
    if (!stage0_fb_ready)
        kernel_fb_early();
    stage0_publish_framebuffer();
    uart_puts("[boot] PIOS bootstrap\n");
#if !PIOS_STAGE0_SKIP_INITIAL_WATCHDOG
    stage0_watchdog_arm(BOOT_WDOG_SECONDS);
#endif
    fb_set_color(0x0000FF00, 0x00000000);
    fb_puts("[stage0] start\n");
    if (!sd_init()) {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_puts("[stage0] sd init failed\n");
        uart_puts("[boot] sd init failed\n");
        for (;;) wfi();
    }

    u32 root_lba = 0;
    bool partition_valid = discover_kernel_partition(&root_lba);
    if (!partition_valid)
        root_lba = BOOT_FALLBACK_LBA;
    else
        /*
         * The FAT package is the physical recovery/update authority. Its
         * installer validates the package and skips the raw-slot write when
         * the selected payload already matches, so a healthy raw slot must
         * never suppress an available staged update.
         */
        (void)stage0_apply_fat_update(root_lba);
    u32 slot_lba = partition_valid ? select_kernel_slot_lba(root_lba)
                                   : BOOT_FALLBACK_LBA;
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
    if (bad_header) {
        /* Bidirectional slot fallback: if the active slot's header is bad, try
         * the OTHER slot before giving up. The original code only fell back
         * B->A (slot_lba != root_lba), so a corrupt active slot A would halt
         * with no recovery — which is exactly the brick this fixes. */
        u32 other = (slot_lba == root_lba)
                        ? root_lba + (PIOS_BOOT_SLOT_B_OFFSET / SD_BLOCK_SIZE)
                        : root_lba;
        if (other != slot_lba) {
            fb_puts("[stage0] active slot bad; trying other slot\n");
            uart_puts("[boot] active slot bad; trying other\n");
            slot_lba = other;
            if (sd_read_block(slot_lba, hdr)) {
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
        }
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
    struct stage2_selection sel;
    if (!select_stage2_image(staging, image_len, &sel)) {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_puts("[stage0] stage2 manifest select failed\n");
        uart_puts("[boot] stage2 manifest select failed\n");
        for (;;) wfi();
    }
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("[stage0] jumping entry=0x%x\n", sel.entry_offset);

    u8 *tramp_dst = (u8 *)(usize)BOOT_TRAMP_ADDR;
    u32 tramp_len = (u32)(bootstrap_trampoline_end - bootstrap_trampoline);
    memcpy(tramp_dst, bootstrap_trampoline, tramp_len);
    dsb();
    isb();

    uart_puts("[boot] jumping real kernel\n");
    uart_puts("[boot] sel.payload_offset="); uart_hex(sel.payload_offset);
    uart_puts(" sel.payload_bytes="); uart_hex(sel.payload_bytes);
    uart_puts(" sel.entry_offset="); uart_hex(sel.entry_offset);
    uart_puts("\n");
    stage0_watchdog_disable();
    void (*tramp)(u64, u64, u64, u64) = (void (*)(u64, u64, u64, u64))(usize)BOOT_TRAMP_ADDR;
    tramp(BOOT_DST_ADDR, BOOT_STAGING_ADDR + sel.payload_offset, sel.payload_bytes,
          BOOT_DST_ADDR + sel.entry_offset);
    for (;;) wfi();
}
