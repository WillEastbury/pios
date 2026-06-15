/*
 * BOOTAA64.EFI - QEMU/UEFI stage1 loader.
 *
 * Loads one common stage2 image (PIOSSTG2.BIN), scans its PGS2 manifest, selects
 * the QEMU platform entry offset, and jumps into that image.
 */
#include "../include/types.h"
#include "../include/stage2_manifest.h"

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
#define STAGE2_MAX_MEMORY_BYTES (20U * 1024U * 1024U)
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

static bool find_stage2_entry(const u8 *image, u32 len, u32 *entry_off, u64 *image_size)
{
    for (u32 off = 0; off + sizeof(struct pios_stage2_manifest_header) <= len; off += 8) {
        const u8 *h = image + off;
        if (rd32(h) != PIOS_STAGE2_MANIFEST_MAGIC) continue;
        u16 ver = rd16(h + 4);
        u16 hb = rd16(h + 6);
        u16 cnt = rd16(h + 8);
        u16 eb = rd16(h + 10);
        if (ver != PIOS_STAGE2_MANIFEST_VERSION || hb < 32 || eb < 64 || cnt == 0 || cnt > 16)
            continue;
        if ((u64)hb + (u64)cnt * eb > len - off) continue;
        u64 full_size = rd64(h + 24);
        if (full_size == 0 || full_size > STAGE2_MAX_MEMORY_BYTES)
            full_size = len;
        for (u32 i = 0; i < cnt; i++) {
            const u8 *e = h + hb + i * eb;
            if (rd32(e) != PIOS_STAGE2_PLATFORM_QEMU_VIRT) continue;
            u64 ent = rd64(e + 8);
            if (ent >= full_size) return false;
            *entry_off = (u32)ent;
            if (image_size) *image_size = full_size;
            return true;
        }
        return false;
    }
    return false;
}

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    g_con = st ? st->con_out : 0;
    puts_ascii("PIOS BOOTAA64: loading embedded common stage2...\n");
    if (!st || !st->boot_services) return EFI_LOAD_ERROR;

    usize sz = (usize)(pios_stage2_blob_end - pios_stage2_blob_start);
    if (sz == 0 || sz > STAGE2_MAX_BYTES) {
        puts_ascii("embedded stage2 size invalid\n");
        return EFI_LOAD_ERROR;
    }
    u64 addr = STAGE2_LOAD_ADDR;
    if (st->boot_services->allocate_pages &&
        st->boot_services->allocate_pages(ALLOCATE_ADDRESS, EFI_LOADER_DATA, STAGE2_MAX_PAGE_COUNT, &addr) != EFI_SUCCESS) {
        puts_ascii("allocate stage2 pages failed\n");
        return EFI_LOAD_ERROR;
    }
    memset((void *)(usize)addr, 0, STAGE2_MAX_MEMORY_BYTES);
    memcpy((void *)(usize)addr, pios_stage2_blob_start, sz);

    u32 entry_off = 0;
    u64 image_size = 0;
    if (!find_stage2_entry((const u8 *)(usize)addr, (u32)sz, &entry_off, &image_size)) {
        puts_ascii("stage2 PGS2 entry not found\n");
        return EFI_LOAD_ERROR;
    }
    puts_ascii("stage2 selected; jumping\n");
    void (*entry)(efi_handle_t, struct efi_system_table *) =
        (void (*)(efi_handle_t, struct efi_system_table *))(usize)(addr + entry_off);
    entry(image, st);
    return EFI_SUCCESS;
}
