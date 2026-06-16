/*
 * BOOTAA64.EFI - QEMU/UEFI stage1 loader.
 *
 * Loads one shared stage2 package, scans its PGS2 manifest, selects the QEMU
 * payload, and jumps into that platform image.
 */
#include "../include/types.h"
#include "../include/stage2_manifest.h"
#include "../include/bootinfo.h"

typedef u64 efi_status_t;
typedef void *efi_handle_t;
typedef void *efi_event_t;

#define EFI_SUCCESS 0ULL
#define EFI_LOAD_ERROR 1ULL
#define EFI_NOT_FOUND 14ULL
#define ALLOCATE_ADDRESS 2U
#define EFI_LOADER_DATA 2U
#define STAGE2_LOAD_ADDR 0x40080000ULL
#define STAGE2_MAX_BYTES (2U * 1024U * 1024U)
#define STAGE2_MAX_MEMORY_BYTES (32U * 1024U * 1024U)
#define STAGE2_PAGE_COUNT ((STAGE2_MAX_BYTES + 4095U) / 4096U)
#define STAGE2_MAX_PAGE_COUNT ((STAGE2_MAX_MEMORY_BYTES + 4095U) / 4096U)

extern const u8 pios_stage2_blob_start[];
extern const u8 pios_stage2_blob_end[];

struct efi_table_header { u64 sig; u32 rev; u32 size; u32 crc; u32 rsv; };
typedef efi_status_t (*efi_text_string_fn)(void *self, const u16 *str);
struct efi_simple_text_output { void *reset; efi_text_string_fn output_string; };

struct efi_guid { u32 data1; u16 data2; u16 data3; u8 data4[8]; };
typedef efi_status_t (*efi_allocate_pages_fn)(u32 type, u32 memtype, usize pages, u64 *memory);
typedef efi_status_t (*efi_handle_protocol_fn)(efi_handle_t handle, struct efi_guid *protocol, void **interface);
typedef efi_status_t (*efi_locate_protocol_fn)(struct efi_guid *protocol, void *registration, void **interface);
typedef efi_status_t (*efi_connect_controller_fn)(efi_handle_t controller,
                                                  efi_handle_t *drivers,
                                                  void *remaining_path,
                                                  bool recursive);

struct efi_boot_services {
    struct efi_table_header hdr;
    void *raise_tpl; void *restore_tpl;
    efi_allocate_pages_fn allocate_pages;
    void *free_pages; void *get_memory_map; void *allocate_pool; void *free_pool;
    void *create_event; void *set_timer; void *wait_for_event; void *signal_event;
    void *close_event; void *check_event; void *install_protocol_interface;
    void *reinstall_protocol_interface; void *uninstall_protocol_interface;
    efi_handle_protocol_fn handle_protocol;
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
    efi_connect_controller_fn connect_controller;
    void *disconnect_controller;
    void *open_protocol;
    void *close_protocol;
    void *open_protocol_information;
    void *protocols_per_handle;
    void *locate_handle_buffer;
    efi_locate_protocol_fn locate_protocol;
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

struct efi_graphics_output {
    void *query_mode;
    void *set_mode;
    void *blt;
    struct efi_gop_mode *mode;
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

static void puts16(const u16 *s)
{
    if (g_con && g_con->output_string) g_con->output_string(g_con, s);
}

static void puts_ascii(const char *s)
{
    u16 b[96];
    while (*s) {
        u32 n = 0;
        while (*s && n + 1 < (u32)(sizeof(b) / sizeof(b[0]))) {
            char c = *s++;
            if (c == '\n') b[n++] = '\r';
            b[n++] = (u16)c;
        }
        b[n] = 0;
        puts16(b);
    }
}

static u16 rd16(const u8 *p) { return (u16)(p[0] | ((u16)p[1] << 8)); }
static u32 rd32(const u8 *p) { return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24); }
static u64 rd64(const u8 *p) { return (u64)rd32(p) | ((u64)rd32(p + 4) << 32); }

struct stage2_selection {
    u32 payload_offset;
    u32 payload_bytes;
    u32 entry_offset;
    u64 load_addr;
    u64 memory_size;
    u32 payload_flags;
    u32 payload_codec;
    u32 uncompressed_bytes;
};

static u16 rd16le(const u8 *p) { return (u16)(p[0] | ((u16)p[1] << 8)); }

static bool decompress_picocompress(const u8 *src, u32 src_len, u8 *dst, u32 dst_cap, u32 *out_len)
{
    u32 ip = 0;
    u32 op = 0;
    while (ip < src_len) {
        if (src_len - ip < 4)
            return false;
        u32 raw_len = rd16le(src + ip);
        u32 comp_len = rd16le(src + ip + 2);
        ip += 4;
        if (raw_len == 0 || raw_len > 508U || op > dst_cap || raw_len > dst_cap - op)
            return false;
        if (comp_len == 0) {
            if (raw_len > src_len - ip)
                return false;
            memcpy(dst + op, src + ip, raw_len);
            ip += raw_len;
            op += raw_len;
            continue;
        }
        if (comp_len > src_len - ip)
            return false;
        u32 frame_end = ip + comp_len;
        u32 frame_start = op;
        while (ip < frame_end) {
            u8 token = src[ip++];
            if ((token & 0x80U) == 0) {
                u32 n = (u32)(token & 0x7FU) + 1U;
                if (n > frame_end - ip || n > dst_cap - op || op + n > frame_start + raw_len)
                    return false;
                memcpy(dst + op, src + ip, n);
                ip += n;
                op += n;
            } else {
                if (ip >= frame_end)
                    return false;
                u32 n = ((token >> 4U) & 0x07U) + 3U;
                u32 off = (((u32)token & 0x0FU) << 8U) | src[ip++];
                if (off == 0 || off > op - frame_start || n > dst_cap - op ||
                    op + n > frame_start + raw_len)
                    return false;
                for (u32 i = 0; i < n; i++) {
                    dst[op] = dst[op - off];
                    op++;
                }
            }
        }
        if (op != frame_start + raw_len)
            return false;
    }
    if (out_len)
        *out_len = op;
    return true;
}

static void dcache_clean_range(u64 start, u64 size)
{
    if (size == 0)
        return;
    u64 a = start & ~63ULL;
    u64 e = (start + size + 63ULL) & ~63ULL;
    while (a < e) {
        __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
        a += 64U;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void stage2_handoff_barrier(u64 addr, u64 size)
{
    dcache_clean_range(addr, size);
    dcache_clean_range(PIOS_BOOTINFO_ADDR, sizeof(struct pios_bootinfo));
    __asm__ volatile(
        "dsb sy\n"
        "ic iallu\n"
        "dsb sy\n"
        "isb\n"
        "mrs x8, sctlr_el1\n"
        "bic x8, x8, #(1 << 0)\n"
        "bic x8, x8, #(1 << 2)\n"
        "bic x8, x8, #(1 << 12)\n"
        "msr sctlr_el1, x8\n"
        "dsb sy\n"
        "isb\n"
        ::: "x8", "memory");
}

static void publish_bootinfo(struct efi_system_table *st)
{
    struct pios_bootinfo *bi = (struct pios_bootinfo *)(usize)PIOS_BOOTINFO_ADDR;
    memset(bi, 0, sizeof(*bi));
    bi->magic = PIOS_BOOTINFO_MAGIC;
    bi->version = PIOS_BOOTINFO_VERSION;
    if (!st || !st->boot_services || !st->boot_services->locate_protocol)
        return;

    static struct efi_guid gop_guid = {
        0x9042A9DEU, 0x23DCU, 0x4A38U,
        {0x96U, 0xFBU, 0x7AU, 0xDEU, 0xD0U, 0x80U, 0x51U, 0x6AU}
    };
    struct efi_graphics_output *gop = 0;
    if (st->boot_services->locate_protocol(&gop_guid, 0, (void **)&gop) != EFI_SUCCESS ||
        !gop || !gop->mode || !gop->mode->info || !gop->mode->framebuffer_base)
        return;

    bi->flags |= PIOS_BOOTINFO_FLAG_FRAMEBUFFER;
    bi->framebuffer_base = gop->mode->framebuffer_base;
    bi->framebuffer_width = gop->mode->info->horizontal_resolution;
    bi->framebuffer_height = gop->mode->info->vertical_resolution;
    bi->framebuffer_pitch = gop->mode->info->pixels_per_scan_line * 4U;
    bi->framebuffer_format = gop->mode->info->pixel_format;
}

static bool find_stage2_entry(const u8 *image, u32 len, struct stage2_selection *sel)
{
    if (!image || !sel)
        return false;
    sel->payload_offset = 0;
    sel->payload_bytes = len;
    sel->entry_offset = 0;
    sel->load_addr = STAGE2_LOAD_ADDR;
    sel->memory_size = STAGE2_MAX_MEMORY_BYTES;
    sel->payload_flags = 0;
    sel->payload_codec = 0;
    sel->uncompressed_bytes = len;
    for (u32 off = 0; off + sizeof(struct pios_stage2_manifest_header) <= len; off += 8) {
        const u8 *h = image + off;
        if (rd32(h) != PIOS_STAGE2_MANIFEST_MAGIC) continue;
        u16 ver = rd16(h + 4);
        u16 hb = rd16(h + 6);
        u16 cnt = rd16(h + 8);
        u16 eb = rd16(h + 10);
        u32 flags = rd32(h + 12);
        if (ver != PIOS_STAGE2_MANIFEST_VERSION || hb < 32 || eb < 64 || cnt == 0 || cnt > 16)
            continue;
        if ((flags & PIOS_STAGE2_MANIFEST_FLAG_PACKAGED) && eb < PIOS_STAGE2_PACKAGED_ENTRY_BYTES)
            continue;
        if ((u64)hb + (u64)cnt * eb > len - off) continue;
        u64 full_size = rd64(h + 24);
        if (full_size == 0 || full_size > STAGE2_MAX_MEMORY_BYTES)
            full_size = len;
        for (u32 i = 0; i < cnt; i++) {
            const u8 *e = h + hb + i * eb;
            if (rd32(e) != PIOS_STAGE2_PLATFORM_QEMU_VIRT) continue;
            u64 ent = rd64(e + 8);
            if (flags & PIOS_STAGE2_MANIFEST_FLAG_PACKAGED) {
                u64 payload_off = rd64(e + 64);
                u64 payload_size = rd64(e + 72);
                u64 load_addr = rd64(e + 80);
                u64 memory_size = rd64(e + 88);
                u32 payload_flags = 0;
                u32 payload_codec = 0;
                u64 uncompressed_size = payload_size;
                if (eb >= PIOS_STAGE2_PACKAGED_ENTRY_BYTES_V2) {
                    payload_flags = rd32(e + 96);
                    payload_codec = rd32(e + 100);
                    uncompressed_size = rd64(e + 104);
                }
                if (payload_size == 0 || payload_off > len ||
                    payload_size > len - payload_off ||
                    uncompressed_size == 0 ||
                    ent >= uncompressed_size ||
                    payload_size > STAGE2_MAX_BYTES ||
                    uncompressed_size > STAGE2_MAX_BYTES ||
                    memory_size == 0 ||
                    memory_size > STAGE2_MAX_MEMORY_BYTES ||
                    memory_size < uncompressed_size ||
                    load_addr == 0)
                    return false;
                if ((payload_flags & PIOS_STAGE2_PAYLOAD_FLAG_COMPRESSED) &&
                    payload_codec != PIOS_STAGE2_PAYLOAD_CODEC_PICOCOMPRESS)
                    return false;
                sel->payload_offset = (u32)payload_off;
                sel->payload_bytes = (u32)payload_size;
                sel->entry_offset = (u32)ent;
                sel->load_addr = load_addr;
                sel->memory_size = memory_size;
                sel->payload_flags = payload_flags;
                sel->payload_codec = payload_codec;
                sel->uncompressed_bytes = (u32)uncompressed_size;
                return true;
            }
            if (ent >= full_size) return false;
            sel->payload_offset = 0;
            sel->payload_bytes = len;
            sel->entry_offset = (u32)ent;
            sel->load_addr = STAGE2_LOAD_ADDR;
            sel->memory_size = full_size;
            sel->uncompressed_bytes = len;
            return true;
        }
        return false;
    }
    return false;
}

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    g_con = st ? st->con_out : 0;
    puts_ascii("PIOS BOOTAA64: loading embedded shared stage2...\n");
    if (!st || !st->boot_services) return EFI_LOAD_ERROR;
    usize sz = (usize)(pios_stage2_blob_end - pios_stage2_blob_start);
    if (sz == 0 || sz > STAGE2_MAX_BYTES) {
        puts_ascii("embedded stage2 size invalid\n");
        return EFI_LOAD_ERROR;
    }
    struct stage2_selection sel;
    if (!find_stage2_entry(pios_stage2_blob_start, (u32)sz, &sel)) {
        puts_ascii("stage2 PGS2 entry not found\n");
        return EFI_LOAD_ERROR;
    }

    u64 addr = sel.load_addr;
    usize pages = (usize)((sel.memory_size + 4095ULL) / 4096ULL);
    if (st->boot_services->allocate_pages &&
        st->boot_services->allocate_pages(ALLOCATE_ADDRESS, EFI_LOADER_DATA, pages, &addr) != EFI_SUCCESS) {
        puts_ascii("allocate stage2 pages failed\n");
        return EFI_LOAD_ERROR;
    }
    memset((void *)(usize)addr, 0, (usize)sel.memory_size);
    if (sel.payload_flags & PIOS_STAGE2_PAYLOAD_FLAG_COMPRESSED) {
        u32 out_len = 0;
        if (!decompress_picocompress(pios_stage2_blob_start + sel.payload_offset,
                                     sel.payload_bytes,
                                     (u8 *)(usize)addr,
                                     sel.uncompressed_bytes,
                                     &out_len) ||
            out_len != sel.uncompressed_bytes) {
            puts_ascii("stage2 decompress failed\n");
            return EFI_LOAD_ERROR;
        }
    } else {
        memcpy((void *)(usize)addr, pios_stage2_blob_start + sel.payload_offset, sel.payload_bytes);
    }
    publish_bootinfo(st);

    puts_ascii("stage2 selected; jumping\n");
    void (*entry)(efi_handle_t, struct efi_system_table *) =
        (void (*)(efi_handle_t, struct efi_system_table *))(usize)(addr + sel.entry_offset);
    stage2_handoff_barrier(addr, sel.memory_size);
    entry(image, st);
    return EFI_SUCCESS;
}
