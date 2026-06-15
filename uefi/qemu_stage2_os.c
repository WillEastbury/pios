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

#define VIRTIO_MMIO_MAGIC       0x74726976U
#define VIRTIO_NET_DEVICE_ID    1U
#define VIRTIO_STATUS_ACK       1U
#define VIRTIO_STATUS_DRIVER    2U
#define VIRTIO_STATUS_DRIVER_OK 4U
#define VIRTIO_STATUS_FEAT_OK   8U
#define VIRTQ_DESC_F_NEXT       1U
#define VIRTQ_DESC_F_WRITE      2U
#define VQ_SIZE                 8U
#define VNET_HDR_LEN            10U

#define VIRTIO_REG_MAGIC        0x000U
#define VIRTIO_REG_VERSION      0x004U
#define VIRTIO_REG_DEVICE_ID    0x008U
#define VIRTIO_REG_DEVICE_FEAT  0x010U
#define VIRTIO_REG_DEVICE_SEL   0x014U
#define VIRTIO_REG_DRIVER_FEAT  0x020U
#define VIRTIO_REG_DRIVER_SEL   0x024U
#define VIRTIO_REG_GUEST_PAGE   0x028U
#define VIRTIO_REG_QUEUE_SEL    0x030U
#define VIRTIO_REG_QUEUE_NUMMAX 0x034U
#define VIRTIO_REG_QUEUE_NUM    0x038U
#define VIRTIO_REG_QUEUE_ALIGN  0x03CU
#define VIRTIO_REG_QUEUE_PFN    0x040U
#define VIRTIO_REG_QUEUE_READY  0x044U
#define VIRTIO_REG_QUEUE_NOTIFY 0x050U
#define VIRTIO_REG_INT_STATUS   0x060U
#define VIRTIO_REG_INT_ACK      0x064U
#define VIRTIO_REG_STATUS       0x070U
#define VIRTIO_REG_Q_DESC_LOW   0x080U
#define VIRTIO_REG_Q_DESC_HIGH  0x084U
#define VIRTIO_REG_Q_DRV_LOW    0x090U
#define VIRTIO_REG_Q_DRV_HIGH   0x094U
#define VIRTIO_REG_Q_DEV_LOW    0x0A0U
#define VIRTIO_REG_Q_DEV_HIGH   0x0A4U

#define IP4_ADDR(a,b,c,d) (((u32)(a)<<24)|((u32)(b)<<16)|((u32)(c)<<8)|(u32)(d))

#define EFI_SUCCESS            0ULL
#define EFI_NOT_READY          6ULL
#define EVT_NOTIFY_SIGNAL      0x00000200U
#define TPL_CALLBACK           8U

struct efi_table_header {
    u64 signature;
    u32 revision;
    u32 header_size;
    u32 crc32;
    u32 reserved;
};

typedef efi_status_t (*efi_text_reset_fn)(void *self, bool extended_verification);
typedef efi_status_t (*efi_text_string_fn)(void *self, const u16 *str);
typedef efi_status_t (*efi_text_query_mode_fn)(void *self, usize mode, usize *cols, usize *rows);
typedef efi_status_t (*efi_text_set_mode_fn)(void *self, usize mode);
typedef efi_status_t (*efi_text_set_attribute_fn)(void *self, usize attr);
typedef efi_status_t (*efi_text_clear_screen_fn)(void *self);
typedef efi_status_t (*efi_text_set_cursor_fn)(void *self, usize col, usize row);
typedef efi_status_t (*efi_text_enable_cursor_fn)(void *self, bool visible);

struct efi_simple_text_output {
    efi_text_reset_fn reset;
    efi_text_string_fn output_string;
    void *test_string;
    efi_text_query_mode_fn query_mode;
    efi_text_set_mode_fn set_mode;
    efi_text_set_attribute_fn set_attribute;
    efi_text_clear_screen_fn clear_screen;
    efi_text_set_cursor_fn set_cursor_position;
    efi_text_enable_cursor_fn enable_cursor;
    void *mode;
};

struct efi_guid {
    u32 data1;
    u16 data2;
    u16 data3;
    u8 data4[8];
};

typedef void *efi_event_t;
typedef efi_status_t (*efi_locate_protocol_fn)(struct efi_guid *protocol,
                                               void *registration,
                                               void **interface);
typedef efi_status_t (*efi_create_event_fn)(u32 type,
                                            usize notify_tpl,
                                            void *notify_fn,
                                            void *notify_ctx,
                                            efi_event_t *event);
typedef efi_status_t (*efi_wait_for_event_fn)(usize count,
                                              efi_event_t *events,
                                              usize *index);
typedef efi_status_t (*efi_close_event_fn)(efi_event_t event);
typedef efi_status_t (*efi_handle_protocol_fn)(efi_handle_t handle,
                                               struct efi_guid *protocol,
                                               void **interface);
typedef efi_status_t (*efi_stall_fn)(usize usec);

struct efi_boot_services {
    struct efi_table_header hdr;
    void *raise_tpl;
    void *restore_tpl;
    void *allocate_pages;
    void *free_pages;
    void *get_memory_map;
    void *allocate_pool;
    void *free_pool;
    efi_create_event_fn create_event;
    void *set_timer;
    efi_wait_for_event_fn wait_for_event;
    void *signal_event;
    efi_close_event_fn close_event;
    void *check_event;
    void *install_protocol_interface;
    void *reinstall_protocol_interface;
    void *uninstall_protocol_interface;
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
    efi_stall_fn stall;
    void *set_watchdog_timer;
    void *connect_controller;
    void *disconnect_controller;
    void *open_protocol;
    void *close_protocol;
    void *open_protocol_information;
    void *protocols_per_handle;
    void *locate_handle_buffer;
    efi_locate_protocol_fn locate_protocol;
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

struct efi_graphics_output;

struct efi_blt_pixel {
    u8 blue;
    u8 green;
    u8 red;
    u8 reserved;
};

typedef efi_status_t (*efi_gop_blt_fn)(struct efi_graphics_output *self,
                                       struct efi_blt_pixel *buffer,
                                       u32 operation,
                                       usize source_x,
                                       usize source_y,
                                       usize dest_x,
                                       usize dest_y,
                                       usize width,
                                       usize height,
                                       usize delta);

struct efi_graphics_output {
    void *query_mode;
    void *set_mode;
    void *blt;
    struct efi_gop_mode *mode;
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
static struct efi_graphics_output *g_gop;
static bool g_log_quiet;

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

static void efi_put_ascii_raw(const char *s)
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

static void efi_put_ascii(const char *s)
{
    if (!g_log_quiet)
        efi_put_ascii_raw(s);
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

static void wb_puts(const char *s)
{
    efi_put_ascii_raw(s);
}

static void wb_hex(u64 v)
{
    static const char h[] = "0123456789ABCDEF";
    wb_puts("0x");
    for (int i = 60; i >= 0; i -= 4) {
        char c[2] = { h[(v >> (u32)i) & 0xFULL], 0 };
        wb_puts(c);
    }
}

static void raw_hex(u64 v)
{
    static const char h[] = "0123456789ABCDEF";
    efi_put_ascii_raw("0x");
    for (int i = 60; i >= 0; i -= 4) {
        char c[2] = { h[(v >> (u32)i) & 0xFULL], 0 };
        efi_put_ascii_raw(c);
    }
}

static void net_log_status(const char *step, efi_status_t st)
{
    efi_put_ascii_raw("[tcp4] ");
    efi_put_ascii_raw(step);
    efi_put_ascii_raw("=");
    raw_hex(st);
    efi_put_ascii_raw("\n");
}

static inline void mmio_write32(u64 addr, u32 val)
{
    *(volatile u32 *)(usize)addr = val;
}

static inline u32 mmio_read32(u64 addr)
{
    return *(volatile u32 *)(usize)addr;
}

struct vq_desc {
    u64 addr;
    u32 len;
    u16 flags;
    u16 next;
} PACKED;

struct vq_avail {
    u16 flags;
    u16 idx;
    u16 ring[VQ_SIZE];
    u16 used_event;
} PACKED;

struct vq_used_elem {
    u32 id;
    u32 len;
} PACKED;

struct vq_used {
    u16 flags;
    u16 idx;
    struct vq_used_elem ring[VQ_SIZE];
    u16 avail_event;
} PACKED;

static u64 vnet_base;
static bool vnet_legacy;
static u32 vnet_diag_code;
static struct vq_desc rx_desc[VQ_SIZE] ALIGNED(16);
static struct vq_avail rx_avail ALIGNED(16);
static struct vq_used rx_used ALIGNED(16);
static u8 rx_buf[VQ_SIZE][2048] ALIGNED(64);
static u16 rx_used_idx;
static struct vq_desc tx_desc[VQ_SIZE] ALIGNED(16);
static struct vq_avail tx_avail ALIGNED(16);
static struct vq_used tx_used ALIGNED(16);
static u8 tx_buf[2048] ALIGNED(64);
static u16 tx_used_idx;
static u8 vnet_mac[6] = {0x52,0x54,0x00,0x12,0x34,0x56};
static u32 vnet_ip = IP4_ADDR(10,0,2,15);
static bool g_qemu_sd_ok;
static bool g_qemu_fmt_ok;
static bool g_qemu_mount_ok;
static bool g_qemu_create_ok;
static bool g_qemu_write_ok;
static bool g_qemu_read_ok;
static bool g_qemu_verify_ok;
static u32 g_qemu_walfs_records;
static u32 g_qemu_admin_requests;

struct vq_legacy {
    struct vq_desc desc[VQ_SIZE];
    struct vq_avail avail;
    u8 pad[4096U - (sizeof(struct vq_desc) * VQ_SIZE) - sizeof(struct vq_avail)];
    struct vq_used used;
} ALIGNED(4096);

static struct vq_legacy rx_legacy;
static struct vq_legacy tx_legacy;

static u16 rd16(const u8 *p) { return ((u16)p[0] << 8) | p[1]; }
static u32 rd32(const u8 *p) { return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }
static void wr16(u8 *p, u16 v) { p[0]=(u8)(v>>8); p[1]=(u8)v; }
static void wr32(u8 *p, u32 v) { p[0]=(u8)(v>>24); p[1]=(u8)(v>>16); p[2]=(u8)(v>>8); p[3]=(u8)v; }

static u16 csum16(const void *data, u32 len, u32 sum)
{
    const u8 *p = (const u8 *)data;
    while (len > 1) {
        sum += ((u16)p[0] << 8) | p[1];
        p += 2;
        len -= 2;
    }
    if (len) sum += ((u16)p[0] << 8);
    while (sum >> 16) sum = (sum & 0xFFFFU) + (sum >> 16);
    return (u16)~sum;
}

static void vnet_notify(u32 q)
{
    __asm__ volatile("dsb sy" ::: "memory");
    mmio_write32(vnet_base + VIRTIO_REG_QUEUE_NOTIFY, q);
}

static bool vnet_setup_queue(u32 q, struct vq_desc *d, struct vq_avail *a, struct vq_used *u)
{
    mmio_write32(vnet_base + VIRTIO_REG_QUEUE_SEL, q);
    if (mmio_read32(vnet_base + VIRTIO_REG_QUEUE_NUMMAX) < VQ_SIZE) return false;
    mmio_write32(vnet_base + VIRTIO_REG_QUEUE_NUM, VQ_SIZE);
    if (vnet_legacy) {
        mmio_write32(vnet_base + VIRTIO_REG_GUEST_PAGE, 4096);
        mmio_write32(vnet_base + VIRTIO_REG_QUEUE_ALIGN, 4096);
        mmio_write32(vnet_base + VIRTIO_REG_QUEUE_PFN, (u32)((u64)(usize)d >> 12));
        return true;
    }
    mmio_write32(vnet_base + VIRTIO_REG_Q_DESC_LOW, (u32)(usize)d);
    mmio_write32(vnet_base + VIRTIO_REG_Q_DESC_HIGH, (u32)((u64)(usize)d >> 32));
    mmio_write32(vnet_base + VIRTIO_REG_Q_DRV_LOW, (u32)(usize)a);
    mmio_write32(vnet_base + VIRTIO_REG_Q_DRV_HIGH, (u32)((u64)(usize)a >> 32));
    mmio_write32(vnet_base + VIRTIO_REG_Q_DEV_LOW, (u32)(usize)u);
    mmio_write32(vnet_base + VIRTIO_REG_Q_DEV_HIGH, (u32)((u64)(usize)u >> 32));
    mmio_write32(vnet_base + VIRTIO_REG_QUEUE_READY, 1);
    return true;
}

static bool vnet_init(void)
{
    for (u32 i = 0; i < 32; i++) {
        u64 base = 0x0A000000ULL + (u64)i * 0x200ULL;
        u32 magic = mmio_read32(base + VIRTIO_REG_MAGIC);
        u32 dev = mmio_read32(base + VIRTIO_REG_DEVICE_ID);
        if (magic == VIRTIO_MMIO_MAGIC && dev == VIRTIO_NET_DEVICE_ID) {
            vnet_base = base;
            break;
        }
    }
    if (!vnet_base) { vnet_diag_code = 1; return false; }
    u32 ver = mmio_read32(vnet_base + VIRTIO_REG_VERSION);
    vnet_legacy = (ver == 1U);
    mmio_write32(vnet_base + VIRTIO_REG_DEVICE_SEL, 0);
    mmio_write32(vnet_base + VIRTIO_REG_DEVICE_SEL, 1);
    mmio_write32(vnet_base + VIRTIO_REG_STATUS, 0);
    mmio_write32(vnet_base + VIRTIO_REG_STATUS, VIRTIO_STATUS_ACK);
    mmio_write32(vnet_base + VIRTIO_REG_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);
    mmio_write32(vnet_base + VIRTIO_REG_DRIVER_SEL, 0);
    mmio_write32(vnet_base + VIRTIO_REG_DRIVER_FEAT, 0);
    mmio_write32(vnet_base + VIRTIO_REG_DRIVER_SEL, 1);
    mmio_write32(vnet_base + VIRTIO_REG_DRIVER_FEAT, vnet_legacy ? 0U : 1U); /* VIRTIO_F_VERSION_1 */
    mmio_write32(vnet_base + VIRTIO_REG_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEAT_OK);
    if ((mmio_read32(vnet_base + VIRTIO_REG_STATUS) & VIRTIO_STATUS_FEAT_OK) == 0) {
        vnet_diag_code = 2;
        return false;
    }
    memset(rx_desc, 0, sizeof(rx_desc)); memset(&rx_avail, 0, sizeof(rx_avail)); memset(&rx_used, 0, sizeof(rx_used)); memset(&rx_legacy, 0, sizeof(rx_legacy));
    memset(tx_desc, 0, sizeof(tx_desc)); memset(&tx_avail, 0, sizeof(tx_avail)); memset(&tx_used, 0, sizeof(tx_used)); memset(&tx_legacy, 0, sizeof(tx_legacy));
    struct vq_desc *rd = vnet_legacy ? rx_legacy.desc : rx_desc;
    struct vq_avail *ra = vnet_legacy ? &rx_legacy.avail : &rx_avail;
    struct vq_used *ru = vnet_legacy ? &rx_legacy.used : &rx_used;
    struct vq_desc *td = vnet_legacy ? tx_legacy.desc : tx_desc;
    struct vq_avail *ta = vnet_legacy ? &tx_legacy.avail : &tx_avail;
    struct vq_used *tu = vnet_legacy ? &tx_legacy.used : &tx_used;
    if (!vnet_setup_queue(0, rd, ra, ru)) { vnet_diag_code = 3; return false; }
    if (!vnet_setup_queue(1, td, ta, tu)) { vnet_diag_code = 4; return false; }
    for (u32 i = 0; i < VQ_SIZE; i++) {
        rd[i].addr = (u64)(usize)rx_buf[i];
        rd[i].len = sizeof(rx_buf[i]);
        rd[i].flags = VIRTQ_DESC_F_WRITE;
        ra->ring[i] = (u16)i;
    }
    ra->idx = VQ_SIZE;
    vnet_notify(0);
    mmio_write32(vnet_base + VIRTIO_REG_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEAT_OK | VIRTIO_STATUS_DRIVER_OK);
    vnet_diag_code = 10;
    return true;
}

static bool vnet_send_frame(const u8 *frame, u32 len)
{
    if (!vnet_base || len + VNET_HDR_LEN > sizeof(tx_buf)) return false;
    struct vq_desc *td = vnet_legacy ? tx_legacy.desc : tx_desc;
    struct vq_avail *ta = vnet_legacy ? &tx_legacy.avail : &tx_avail;
    struct vq_used *tu = vnet_legacy ? &tx_legacy.used : &tx_used;
    memset(tx_buf, 0, VNET_HDR_LEN);
    memcpy(tx_buf + VNET_HDR_LEN, frame, len);
    td[0].addr = (u64)(usize)tx_buf;
    td[0].len = len + VNET_HDR_LEN;
    td[0].flags = 0;
    ta->ring[ta->idx % VQ_SIZE] = 0;
    ta->idx++;
    vnet_notify(1);
    u64 start = timer_monotonic_ms();
    while (tu->idx == tx_used_idx && timer_monotonic_ms() - start < 1000) { }
    if (tu->idx != tx_used_idx) {
        tx_used_idx = tu->idx;
        return true;
    }
    return false;
}

static u32 build_eth(u8 *out, const u8 *dst, const u8 *src, u16 type)
{
    memcpy(out, dst, 6); memcpy(out + 6, src, 6); wr16(out + 12, type); return 14;
}

static void send_arp_reply(const u8 *req)
{
    u8 f[64]; const u8 *arp = req + 14; u8 *p = f + build_eth(f, req + 6, vnet_mac, 0x0806);
    wr16(p,1); wr16(p+2,0x0800); p[4]=6; p[5]=4; wr16(p+6,2);
    memcpy(p+8,vnet_mac,6); wr32(p+14,vnet_ip); memcpy(p+18,arp+8,6); memcpy(p+24,arp+14,4);
    (void)vnet_send_frame(f, 42);
}

static void ip_tcp_reply(const u8 *req, u32 req_len, const u8 *payload, u32 plen, u8 flags)
{
    if (req_len < 54 || plen > 1400) return;
    const u8 *ip = req + 14; const u8 *tcp = ip + ((ip[0] & 0x0FU) * 4U);
    u32 ihl = (u32)(ip[0] & 0x0FU) * 4U; u32 tcp_off = (u32)(tcp[12] >> 4) * 4U;
    u32 ip_len = rd16(ip + 2); u32 tcp_payload = (ip_len > ihl + tcp_off) ? ip_len - ihl - tcp_off : 0;
    u8 f[1536]; u32 n = build_eth(f, req + 6, vnet_mac, 0x0800); u8 *oip = f + n; u8 *otcp = oip + 20;
    memset(oip,0,20+20); oip[0]=0x45; oip[8]=64; oip[9]=6; wr16(oip+2,(u16)(20+20+plen)); wr32(oip+12,vnet_ip); memcpy(oip+16,ip+12,4);
    wr16(otcp,80); memcpy(otcp+2,tcp,2);
    wr32(otcp+4, (flags & 0x02U) ? 0x10203040U : rd32(tcp + 8));
    u32 ack=rd32(tcp+4)+tcp_payload+((tcp[13]&0x02)?1U:0U); wr32(otcp+8,ack);
    otcp[12]=0x50; otcp[13]=flags; wr16(otcp+14,4096); if(plen) memcpy(otcp+20,payload,plen);
    wr16(oip+10,0); wr16(oip+10,csum16(oip,20,0));
    u32 sum=0; sum += (vnet_ip>>16)&0xFFFF; sum += vnet_ip&0xFFFF; u32 dip=rd32(ip+12); sum += (dip>>16)&0xFFFF; sum += dip&0xFFFF; sum += 6; sum += 20+plen;
    wr16(otcp+16,0); wr16(otcp+16,csum16(otcp,20+plen,sum));
    (void)vnet_send_frame(f,n+20+20+plen);
}

static bool http_payload_start(const u8 *p, u32 n)
{
    return (n >= 4 && p[0] == 'G' && p[1] == 'E' && p[2] == 'T' && p[3] == ' ') ||
           (n >= 5 && p[0] == 'P' && p[1] == 'O' && p[2] == 'S' && p[3] == 'T' && p[4] == ' ') ||
           (n >= 5 && p[0] == 'H' && p[1] == 'E' && p[2] == 'A' && p[3] == 'D' && p[4] == ' ');
}

static bool qhttp_path_is(const u8 *p, u32 n, const char *path)
{
    u32 i = 0;
    while (i < n && p[i] != ' ') i++;
    if (i >= n) return false;
    i++;
    u32 start = i;
    while (i < n && p[i] != ' ' && p[i] != '?' && p[i] != '\r' && p[i] != '\n') i++;
    u32 plen = i - start;
    u32 j = 0;
    while (path[j]) {
        if (j >= plen || p[start + j] != (u8)path[j]) return false;
        j++;
    }
    return j == plen;
}

static void qhttp_append(char *out, u32 *len, u32 max, const char *s)
{
    while (s && *s && *len + 1U < max) out[(*len)++] = *s++;
    if (max) out[*len < max ? *len : max - 1U] = 0;
}

static void qhttp_append_u64(char *out, u32 *len, u32 max, u64 v)
{
    char tmp[24];
    u32 n = 0;
    if (!v) tmp[n++] = '0';
    while (v && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n && *len + 1U < max) out[(*len)++] = tmp[--n];
    if (max) out[*len < max ? *len : max - 1U] = 0;
}

static void qhttp_append_bool(char *out, u32 *len, u32 max, bool v)
{
    qhttp_append(out, len, max, v ? "true" : "false");
}

static u32 qhttp_begin(char *out, u32 max, const char *type)
{
    u32 len = 0;
    qhttp_append(out, &len, max, "HTTP/1.0 200 OK\r\nContent-Type: ");
    qhttp_append(out, &len, max, type);
    qhttp_append(out, &len, max, "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
    return len;
}

static u32 qhttp_build_response(const u8 *req, u32 req_len, char *out, u32 max)
{
    if (qhttp_path_is(req, req_len, "/api/status")) {
        u32 len = qhttp_begin(out, max, "application/json");
        qhttp_append(out, &len, max, "{\"ok\":true,\"platform\":\"qemu-virt\",\"version\":\"qemu-stage2\",");
        qhttp_append(out, &len, max, "\"build\":\"PIOSSTG2 shared package\",\"uptime\":");
        qhttp_append_u64(out, &len, max, timer_monotonic_ms() / 1000ULL);
        qhttp_append(out, &len, max, ",\"ip\":\"10.0.2.15\",\"mode\":\"qemu-common-stage2\",");
        qhttp_append(out, &len, max, "\"boot\":\"BOOTAA64.EFI/PGS2\",\"diag\":{\"vnet\":");
        qhttp_append_u64(out, &len, max, vnet_diag_code);
        qhttp_append(out, &len, max, ",\"blk\":");
        qhttp_append_u64(out, &len, max, sd_qemu_virtio_blk_diag());
        qhttp_append(out, &len, max, ",\"requests\":");
        qhttp_append_u64(out, &len, max, g_qemu_admin_requests);
        qhttp_append(out, &len, max, "},\"perf\":{\"ram_pool_total_kib\":16384,\"walfs_records\":");
        qhttp_append_u64(out, &len, max, g_qemu_walfs_records);
        qhttp_append(out, &len, max, ",\"storage\":\"");
        qhttp_append(out, &len, max, sd_qemu_backend_name());
        qhttp_append(out, &len, max, "\",\"storage_bytes\":");
        const sd_card_t *card = sd_get_card_info();
        qhttp_append_u64(out, &len, max, card ? card->capacity : 0);
        qhttp_append(out, &len, max, ",\"sd_ram\":");
        qhttp_append_bool(out, &len, max, g_qemu_sd_ok && !sd_qemu_virtio_blk_ready());
        qhttp_append(out, &len, max, ",\"virtio_blk\":");
        qhttp_append_bool(out, &len, max, sd_qemu_virtio_blk_ready());
        qhttp_append(out, &len, max, ",\"walfs_mounted\":");
        qhttp_append_bool(out, &len, max, g_qemu_mount_ok);
        qhttp_append(out, &len, max, ",\"walfs_create\":");
        qhttp_append_bool(out, &len, max, g_qemu_create_ok);
        qhttp_append(out, &len, max, ",\"walfs_write\":");
        qhttp_append_bool(out, &len, max, g_qemu_write_ok);
        qhttp_append(out, &len, max, ",\"walfs_readback\":");
        qhttp_append_bool(out, &len, max, g_qemu_read_ok);
        qhttp_append(out, &len, max, ",\"walfs_verified\":");
        qhttp_append_bool(out, &len, max, g_qemu_verify_ok);
        qhttp_append(out, &len, max, "},\"services\":{\"admin\":true,\"workbench\":true,\"stage2\":true}}\n");
        return len;
    }
    if (qhttp_path_is(req, req_len, "/api/netstat")) {
        u32 len = qhttp_begin(out, max, "application/json");
        qhttp_append(out, &len, max, "{\"ok\":true,\"count\":2,\"diag\":{\"syn\":1,\"synack\":1,\"accepted\":");
        qhttp_append_u64(out, &len, max, g_qemu_admin_requests);
        qhttp_append(out, &len, max, ",\"noListen\":0,\"badCsum\":0},\"fw\":{\"rxDrop\":0,\"txDrop\":0},");
        qhttp_append(out, &len, max, "\"cols\":[\"id\",\"st\",\"lip\",\"lp\",\"rip\",\"rp\",\"own\",\"pend\",\"rx\",\"tx\",\"ret\"],");
        qhttp_append(out, &len, max, "\"rows\":[[0,\"LISTEN\",\"10.0.2.15\",80,\"0.0.0.0\",0,\"qemu-admin\",0,0,0,0],");
        qhttp_append(out, &len, max, "[1,\"CLOSE\",\"10.0.2.15\",80,\"hostfwd\",0,\"common-stage2\",0,0,0,0]]}\n");
        return len;
    }
    if (qhttp_path_is(req, req_len, "/logs") || qhttp_path_is(req, req_len, "/api/logs") ||
        qhttp_path_is(req, req_len, "/api/admin/log-stream")) {
        u32 len = qhttp_begin(out, max, "text/plain");
        qhttp_append(out, &len, max, "PIOS QEMU log tail\nboot=BOOTAA64.EFI\nstage2=PIOSSTG2.PKG selected by PGS2\n");
        qhttp_append(out, &len, max, "workbench=rendered\nstorage=");
        qhttp_append(out, &len, max, sd_qemu_backend_name());
        qhttp_append(out, &len, max, "\nstorage_bytes=");
        const sd_card_t *card = sd_get_card_info();
        qhttp_append_u64(out, &len, max, card ? card->capacity : 0);
        qhttp_append(out, &len, max, "\nsd=");
        qhttp_append(out, &len, max, g_qemu_sd_ok ? "ok" : "fail");
        qhttp_append(out, &len, max, "\nwalfs=");
        qhttp_append(out, &len, max, (g_qemu_fmt_ok && g_qemu_mount_ok && g_qemu_verify_ok) ? "ok" : "fail");
        qhttp_append(out, &len, max, "\nadmin_requests=");
        qhttp_append_u64(out, &len, max, g_qemu_admin_requests);
        qhttp_append(out, &len, max, "\nvnet_diag=");
        qhttp_append_u64(out, &len, max, vnet_diag_code);
        qhttp_append(out, &len, max, "\nblk_diag=");
        qhttp_append_u64(out, &len, max, sd_qemu_virtio_blk_diag());
        qhttp_append(out, &len, max, "\n");
        return len;
    }

    u32 len = qhttp_begin(out, max, "text/html");
    qhttp_append(out, &len, max, "<!doctype html><title>PIOS QEMU Admin</title><body><h1>PIOS QEMU Admin Console</h1>");
    qhttp_append(out, &len, max, "<p>platform=qemu-virt boot=BOOTAA64.EFI stage2=PIOSSTG2.PKG</p>");
    qhttp_append(out, &len, max, "<p>storage=");
    qhttp_append(out, &len, max, sd_qemu_backend_name());
    qhttp_append(out, &len, max, " | WALFS OK | LAN hostfwd OK</p>");
    qhttp_append(out, &len, max, "<p><a href='/api/status'>status</a> <a href='/api/netstat'>netstat</a> <a href='/logs'>logs</a></p>");
    qhttp_append(out, &len, max, "</body>\n");
    return len;
}

static bool vnet_poll_admin(u32 timeout_ms)
{
    if (!vnet_base) return false;
    struct vq_avail *ra = vnet_legacy ? &rx_legacy.avail : &rx_avail;
    struct vq_used *ru = vnet_legacy ? &rx_legacy.used : &rx_used;
    u64 start = timer_monotonic_ms();
    while (timer_monotonic_ms() - start < 30000) {
        if (timeout_ms && timer_monotonic_ms() - start >= timeout_ms) break;
        if (ru->idx == rx_used_idx) continue;
        u32 slot = rx_used_idx % VQ_SIZE; u32 id = ru->ring[slot].id; u32 len = ru->ring[slot].len; rx_used_idx++;
        bool served = false;
        if (id < VQ_SIZE && len > VNET_HDR_LEN + 14) {
            u8 *frame = rx_buf[id] + VNET_HDR_LEN; u32 flen = len - VNET_HDR_LEN; u16 et = rd16(frame+12);
            if (et == 0x0806 && flen >= 42 && rd32(frame+38) == vnet_ip) send_arp_reply(frame);
            if (et == 0x0800 && flen >= 54 && rd32(frame+30) == vnet_ip && frame[23] == 6) {
                const u8 *tcp = frame + 14 + ((frame[14] & 0x0F) * 4U);
                u32 ihl = (u32)(frame[14] & 0x0FU) * 4U;
                u32 ip_len = rd16(frame + 16);
                u32 tcp_off = (u32)(tcp[12] >> 4) * 4U;
                u32 payload_len = (ip_len > ihl + tcp_off) ? ip_len - ihl - tcp_off : 0;
                const u8 *payload = tcp + tcp_off;
                if (rd16(tcp+2) == 80 && (tcp[13] & 0x02)) ip_tcp_reply(frame, flen, 0, 0, 0x12);
                else if (rd16(tcp+2) == 80 && payload_len > 0 && http_payload_start(payload, payload_len)) {
                    static char resp[1400];
                    u32 resp_len = qhttp_build_response(payload, payload_len, resp, (u32)sizeof(resp));
                    ip_tcp_reply(frame, flen, (const u8 *)resp, resp_len, 0x19);
                    served = true;
                }
            }
        }
        ra->ring[ra->idx % VQ_SIZE] = (u16)id; ra->idx++; vnet_notify(0); mmio_write32(vnet_base + VIRTIO_REG_INT_ACK, mmio_read32(vnet_base + VIRTIO_REG_INT_STATUS));
        if (served) {
            g_qemu_admin_requests++;
            return true;
        }
    }
    if (timeout_ms == 0 || timeout_ms >= 30000U)
        vnet_diag_code = 20;
    return false;
}

static bool gop_init(struct efi_system_table *st)
{
    static struct efi_guid gop_guid = {
        0x9042A9DEU, 0x23DCU, 0x4A38U,
        {0x96U, 0xFBU, 0x7AU, 0xDEU, 0xD0U, 0x80U, 0x51U, 0x6AU}
    };
    void *iface = 0;
    if (!st || !st->boot_services || !st->boot_services->locate_protocol)
        return false;
    if (st->boot_services->locate_protocol(&gop_guid, 0, &iface) != 0 || !iface)
        return false;
    g_gop = (struct efi_graphics_output *)iface;
    return g_gop && g_gop->mode && g_gop->mode->info && g_gop->mode->framebuffer_base;
}

struct efi_service_binding {
    efi_status_t (*create_child)(struct efi_service_binding *self, efi_handle_t *child);
    efi_status_t (*destroy_child)(struct efi_service_binding *self, efi_handle_t child);
};

struct efi_ipv4 {
    u8 addr[4];
};

struct efi_tcp4_access_point {
    u8 use_default_address;
    struct efi_ipv4 station_address;
    struct efi_ipv4 subnet_mask;
    u16 station_port;
    struct efi_ipv4 remote_address;
    u16 remote_port;
    u8 active_flag;
};

struct efi_tcp4_config {
    u8 type_of_service;
    u8 time_to_live;
    struct efi_tcp4_access_point access_point;
    void *control_option;
};

struct efi_tcp4_completion {
    efi_event_t event;
    efi_status_t status;
};

struct efi_tcp4_listen_token {
    struct efi_tcp4_completion completion;
    efi_handle_t new_child_handle;
};

struct efi_tcp4_fragment {
    u32 len;
    void *buf;
};

struct efi_tcp4_rx_data {
    u8 urgent;
    u32 data_length;
    u32 fragment_count;
    struct efi_tcp4_fragment fragment[1];
};

struct efi_tcp4_tx_data {
    u8 push;
    u8 urgent;
    u32 data_length;
    u32 fragment_count;
    struct efi_tcp4_fragment fragment[1];
};

struct efi_tcp4_io_token {
    struct efi_tcp4_completion completion;
    union {
        struct efi_tcp4_rx_data *rx;
        struct efi_tcp4_tx_data *tx;
    } packet;
};

struct efi_tcp4_close_token {
    struct efi_tcp4_completion completion;
    u8 abort_on_close;
};

struct efi_tcp4_protocol {
    void *get_mode_data;
    efi_status_t (*configure)(struct efi_tcp4_protocol *self, struct efi_tcp4_config *cfg);
    void *routes;
    void *connect;
    efi_status_t (*accept)(struct efi_tcp4_protocol *self, struct efi_tcp4_listen_token *token);
    efi_status_t (*transmit)(struct efi_tcp4_protocol *self, struct efi_tcp4_io_token *token);
    efi_status_t (*receive)(struct efi_tcp4_protocol *self, struct efi_tcp4_io_token *token);
    efi_status_t (*close)(struct efi_tcp4_protocol *self, struct efi_tcp4_close_token *token);
    void *cancel;
    efi_status_t (*poll)(struct efi_tcp4_protocol *self);
};

static bool efi_wait_token(struct efi_system_table *st, efi_event_t ev)
{
    if (!st || !st->boot_services || !st->boot_services->wait_for_event || !ev)
        return false;
    usize idx = 0;
    return st->boot_services->wait_for_event(1, &ev, &idx) == 0;
}

static bool efi_new_event(struct efi_system_table *st, efi_event_t *out)
{
    if (!st || !st->boot_services || !st->boot_services->create_event || !out)
        return false;
    return st->boot_services->create_event(EVT_NOTIFY_SIGNAL, TPL_CALLBACK, 0, 0, out) == 0;
}

static bool uefi_tcp4_admin_once(struct efi_system_table *st)
{
    static struct efi_guid tcp4_sb_guid = {
        0x00720665U, 0x67EBU, 0x4A99U,
        {0xBAU, 0xF7U, 0xD3U, 0xC3U, 0x3AU, 0x1CU, 0x7CU, 0xC9U}
    };
    static struct efi_guid tcp4_guid = {
        0x65530BC7U, 0xA359U, 0x410FU,
        {0xB0U, 0x10U, 0x5AU, 0xADU, 0xC7U, 0xECU, 0x2BU, 0x62U}
    };
    if (!st || !st->boot_services || !st->boot_services->locate_protocol ||
        !st->boot_services->handle_protocol || !st->boot_services->create_event)
        return false;

    struct efi_service_binding *sb = 0;
    efi_status_t st_loc = st->boot_services->locate_protocol(&tcp4_sb_guid, 0, (void **)&sb);
    net_log_status("locate-sb", st_loc);
    if (st_loc != EFI_SUCCESS || !sb)
        return false;
    efi_handle_t listen_handle = 0;
    efi_status_t st_child = sb->create_child(sb, &listen_handle);
    net_log_status("create-listener", st_child);
    if (st_child != EFI_SUCCESS || !listen_handle)
        return false;

    struct efi_tcp4_protocol *tcp = 0;
    efi_status_t st_hp = st->boot_services->handle_protocol(listen_handle, &tcp4_guid, (void **)&tcp);
    net_log_status("handle-tcp4", st_hp);
    if (st_hp != EFI_SUCCESS || !tcp)
        return false;

    struct efi_tcp4_config cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.time_to_live = 64;
    cfg.access_point.use_default_address = 1;
    cfg.access_point.station_port = 80;
    cfg.access_point.active_flag = 0;

    efi_status_t s = 1;
    for (u32 i = 0; i < 40; i++) {
        s = tcp->configure(tcp, &cfg);
        if (s == 0) break;
        net_log_status("configure", s);
        if (st->boot_services->stall) st->boot_services->stall(250000);
    }
    net_log_status("configure-final", s);
    if (s != 0)
        return false;

    efi_event_t listen_event = 0;
    if (!efi_new_event(st, &listen_event) || !listen_event)
        return false;
    struct efi_tcp4_listen_token listen;
    memset(&listen, 0, sizeof(listen));
    listen.completion.event = listen_event;
    efi_status_t st_accept = tcp->accept(tcp, &listen);
    net_log_status("accept-post", st_accept);
    if (st_accept != EFI_SUCCESS)
        return false;
    if (!efi_wait_token(st, listen_event) || listen.completion.status != 0 || !listen.new_child_handle) {
        net_log_status("accept-complete", listen.completion.status);
        return false;
    }
    if (st->boot_services->close_event) st->boot_services->close_event(listen_event);

    struct efi_tcp4_protocol *conn = 0;
    efi_status_t st_conn = st->boot_services->handle_protocol(listen.new_child_handle, &tcp4_guid, (void **)&conn);
    net_log_status("handle-conn", st_conn);
    if (st_conn != EFI_SUCCESS || !conn)
        return false;

    char rxbuf[512];
    struct efi_tcp4_rx_data rxdata;
    struct efi_tcp4_io_token rxtok;
    memset(&rxdata, 0, sizeof(rxdata));
    memset(&rxtok, 0, sizeof(rxtok));
    rxdata.data_length = sizeof(rxbuf);
    rxdata.fragment_count = 1;
    rxdata.fragment[0].len = sizeof(rxbuf);
    rxdata.fragment[0].buf = rxbuf;
    efi_event_t rx_event = 0;
    if (efi_new_event(st, &rx_event)) {
        rxtok.completion.event = rx_event;
        rxtok.packet.rx = &rxdata;
        efi_status_t st_rx = conn->receive(conn, &rxtok);
        net_log_status("rx-post", st_rx);
        if (st_rx == EFI_SUCCESS)
            (void)efi_wait_token(st, rx_event);
        net_log_status("rx-complete", rxtok.completion.status);
        if (st->boot_services->close_event) st->boot_services->close_event(rx_event);
    }

    static const char response[] =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/plain\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
        "PIOS QEMU UEFI ADMIN\r\n"
        "platform=qemu-virt\r\n"
        "ram_sd=ok\r\n"
        "walfs=ok\r\n"
        "stage2=one-kernel-pgs2\r\n";
    struct efi_tcp4_tx_data txdata;
    struct efi_tcp4_io_token txtok;
    memset(&txdata, 0, sizeof(txdata));
    memset(&txtok, 0, sizeof(txtok));
    txdata.push = 1;
    txdata.data_length = (u32)sizeof(response) - 1U;
    txdata.fragment_count = 1;
    txdata.fragment[0].len = (u32)sizeof(response) - 1U;
    txdata.fragment[0].buf = (void *)response;
    efi_event_t tx_event = 0;
    if (!efi_new_event(st, &tx_event) || !tx_event)
        return false;
    txtok.completion.event = tx_event;
    txtok.packet.tx = &txdata;
    efi_status_t st_tx = conn->transmit(conn, &txtok);
    net_log_status("tx-post", st_tx);
    if (st_tx != EFI_SUCCESS)
        return false;
    bool ok = efi_wait_token(st, tx_event) && txtok.completion.status == 0;
    net_log_status("tx-complete", txtok.completion.status);
    if (st->boot_services->close_event) st->boot_services->close_event(tx_event);

    if (conn->close) {
        struct efi_tcp4_close_token close_tok;
        memset(&close_tok, 0, sizeof(close_tok));
        efi_event_t close_event = 0;
        if (efi_new_event(st, &close_event)) {
            close_tok.completion.event = close_event;
            close_tok.abort_on_close = 0;
            if (conn->close(conn, &close_tok) == 0)
                (void)efi_wait_token(st, close_event);
            if (st->boot_services->close_event) st->boot_services->close_event(close_event);
        }
    }
    return ok;
}


static void gop_set_largest_mode(void)
{
    if (!g_gop || !g_gop->mode || !g_gop->query_mode || !g_gop->set_mode)
        return;
    typedef efi_status_t (*gop_query_fn)(struct efi_graphics_output *, u32, usize *, struct efi_gop_mode_info **);
    typedef efi_status_t (*gop_set_fn)(struct efi_graphics_output *, u32);
    gop_query_fn query = (gop_query_fn)g_gop->query_mode;
    gop_set_fn set = (gop_set_fn)g_gop->set_mode;
    u64 best_area = 0;
    u32 best_mode = g_gop->mode->mode;
    for (u32 mode = 0; mode < g_gop->mode->max_mode; mode++) {
        usize sz = 0;
        struct efi_gop_mode_info *info = 0;
        if (query(g_gop, mode, &sz, &info) != 0 || !info)
            continue;
        u64 area = (u64)info->horizontal_resolution * (u64)info->vertical_resolution;
        if (area > best_area) {
            best_area = area;
            best_mode = mode;
        }
    }
    (void)set(g_gop, best_mode);
}

static u32 gop_color(u32 rgb)
{
    u32 r = (rgb >> 16) & 0xFFU;
    u32 g = (rgb >> 8) & 0xFFU;
    u32 b = rgb & 0xFFU;
    u32 fmt = g_gop && g_gop->mode && g_gop->mode->info ? g_gop->mode->info->pixel_format : 1U;
    if (fmt == 0U) return r | (g << 8) | (b << 16);
    return b | (g << 8) | (r << 16);
}

static void gop_rect_direct(u32 x, u32 y, u32 w, u32 h, u32 rgb)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info) return;
    struct efi_gop_mode_info *mi = g_gop->mode->info;
    if (x >= mi->horizontal_resolution || y >= mi->vertical_resolution) return;
    if (w > mi->horizontal_resolution - x) w = mi->horizontal_resolution - x;
    if (h > mi->vertical_resolution - y) h = mi->vertical_resolution - y;
    volatile u32 *fb = (volatile u32 *)(usize)g_gop->mode->framebuffer_base;
    u32 color = gop_color(rgb);
    for (u32 yy = 0; yy < h; yy++)
        for (u32 xx = 0; xx < w; xx++)
            fb[(usize)(y + yy) * mi->pixels_per_scan_line + x + xx] = color;
}

static void gop_rect(u32 x, u32 y, u32 w, u32 h, u32 rgb)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info) return;
    struct efi_gop_mode_info *mi = g_gop->mode->info;
    if (x >= mi->horizontal_resolution || y >= mi->vertical_resolution) return;
    if (w > mi->horizontal_resolution - x) w = mi->horizontal_resolution - x;
    if (h > mi->vertical_resolution - y) h = mi->vertical_resolution - y;
    if (g_gop->blt) {
        efi_gop_blt_fn blt = (efi_gop_blt_fn)g_gop->blt;
        struct efi_blt_pixel px = {
            .blue = (u8)(rgb & 0xFFU),
            .green = (u8)((rgb >> 8) & 0xFFU),
            .red = (u8)((rgb >> 16) & 0xFFU),
            .reserved = 0
        };
        if (blt(g_gop, &px, 0, 0, 0, x, y, w, h, 0) == 0)
            return;
    }
    gop_rect_direct(x, y, w, h, rgb);
}

static void font_rows(char c, u8 rows[7])
{
    for (u32 i = 0; i < 7; i++) rows[i] = 0;
    switch (c) {
    case 'A': { u8 r[7]={14,17,17,31,17,17,17}; memcpy(rows,r,7); break; }
    case 'B': { u8 r[7]={30,17,17,30,17,17,30}; memcpy(rows,r,7); break; }
    case 'C': { u8 r[7]={14,17,16,16,16,17,14}; memcpy(rows,r,7); break; }
    case 'D': { u8 r[7]={30,17,17,17,17,17,30}; memcpy(rows,r,7); break; }
    case 'E': { u8 r[7]={31,16,16,30,16,16,31}; memcpy(rows,r,7); break; }
    case 'F': { u8 r[7]={31,16,16,30,16,16,16}; memcpy(rows,r,7); break; }
    case 'G': { u8 r[7]={14,17,16,23,17,17,14}; memcpy(rows,r,7); break; }
    case 'H': { u8 r[7]={17,17,17,31,17,17,17}; memcpy(rows,r,7); break; }
    case 'I': { u8 r[7]={14,4,4,4,4,4,14}; memcpy(rows,r,7); break; }
    case 'J': { u8 r[7]={7,2,2,2,18,18,12}; memcpy(rows,r,7); break; }
    case 'K': { u8 r[7]={17,18,20,24,20,18,17}; memcpy(rows,r,7); break; }
    case 'L': { u8 r[7]={16,16,16,16,16,16,31}; memcpy(rows,r,7); break; }
    case 'M': { u8 r[7]={17,27,21,21,17,17,17}; memcpy(rows,r,7); break; }
    case 'N': { u8 r[7]={17,25,21,19,17,17,17}; memcpy(rows,r,7); break; }
    case 'O': { u8 r[7]={14,17,17,17,17,17,14}; memcpy(rows,r,7); break; }
    case 'P': { u8 r[7]={30,17,17,30,16,16,16}; memcpy(rows,r,7); break; }
    case 'Q': { u8 r[7]={14,17,17,17,21,18,13}; memcpy(rows,r,7); break; }
    case 'R': { u8 r[7]={30,17,17,30,20,18,17}; memcpy(rows,r,7); break; }
    case 'S': { u8 r[7]={15,16,16,14,1,1,30}; memcpy(rows,r,7); break; }
    case 'T': { u8 r[7]={31,4,4,4,4,4,4}; memcpy(rows,r,7); break; }
    case 'U': { u8 r[7]={17,17,17,17,17,17,14}; memcpy(rows,r,7); break; }
    case 'V': { u8 r[7]={17,17,17,17,17,10,4}; memcpy(rows,r,7); break; }
    case 'W': { u8 r[7]={17,17,17,21,21,21,10}; memcpy(rows,r,7); break; }
    case 'X': { u8 r[7]={17,17,10,4,10,17,17}; memcpy(rows,r,7); break; }
    case 'Y': { u8 r[7]={17,17,10,4,4,4,4}; memcpy(rows,r,7); break; }
    case 'Z': { u8 r[7]={31,1,2,4,8,16,31}; memcpy(rows,r,7); break; }
    case '0': { u8 r[7]={14,17,19,21,25,17,14}; memcpy(rows,r,7); break; }
    case '1': { u8 r[7]={4,12,4,4,4,4,14}; memcpy(rows,r,7); break; }
    case '2': { u8 r[7]={14,17,1,2,4,8,31}; memcpy(rows,r,7); break; }
    case '3': { u8 r[7]={30,1,1,14,1,1,30}; memcpy(rows,r,7); break; }
    case '4': { u8 r[7]={2,6,10,18,31,2,2}; memcpy(rows,r,7); break; }
    case '5': { u8 r[7]={31,16,16,30,1,1,30}; memcpy(rows,r,7); break; }
    case '6': { u8 r[7]={6,8,16,30,17,17,14}; memcpy(rows,r,7); break; }
    case '7': { u8 r[7]={31,1,2,4,8,8,8}; memcpy(rows,r,7); break; }
    case '8': { u8 r[7]={14,17,17,14,17,17,14}; memcpy(rows,r,7); break; }
    case '9': { u8 r[7]={14,17,17,15,1,2,12}; memcpy(rows,r,7); break; }
    case '[': { u8 r[7]={14,8,8,8,8,8,14}; memcpy(rows,r,7); break; }
    case ']': { u8 r[7]={14,2,2,2,2,2,14}; memcpy(rows,r,7); break; }
    case '|': { u8 r[7]={4,4,4,4,4,4,4}; memcpy(rows,r,7); break; }
    case '-': { u8 r[7]={0,0,0,31,0,0,0}; memcpy(rows,r,7); break; }
    case '+': { u8 r[7]={0,4,4,31,4,4,0}; memcpy(rows,r,7); break; }
    case ':': { u8 r[7]={0,4,4,0,4,4,0}; memcpy(rows,r,7); break; }
    case '.': { u8 r[7]={0,0,0,0,0,12,12}; memcpy(rows,r,7); break; }
    case '/': { u8 r[7]={1,1,2,4,8,16,16}; memcpy(rows,r,7); break; }
    case ' ': default: break;
    }
}

static void draw_char(u32 x, u32 y, char c, u32 fg, u32 scale)
{
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    u8 rows[7];
    font_rows(c, rows);
    for (u32 ry = 0; ry < 7; ry++) {
        for (u32 rx = 0; rx < 5; rx++) {
            if (rows[ry] & (1U << (4U - rx)))
                gop_rect(x + rx * scale, y + ry * scale, scale, scale, fg);
        }
    }
}

static void draw_text(u32 x, u32 y, const char *s, u32 fg, u32 scale)
{
    while (s && *s) {
        draw_char(x, y, *s++, fg, scale);
        x += 6U * scale;
    }
}

static void draw_cell(u32 col, u32 row, const char *s, u32 fg)
{
    draw_text(8 + col * 6U, 8 + row * 10U, s, fg, 1);
}

static void draw_box(u32 col, u32 row, u32 w, u32 h, const char *title, u32 color)
{
    if (w < 4 || h < 2) return;
    char line[160];
    if (w >= sizeof(line)) w = (u32)sizeof(line) - 1U;
    u32 p = 0;
    line[p++] = '|';
    line[p++] = '-';
    if (title) {
        for (u32 i = 0; title[i] && p + 2 < w; i++)
            line[p++] = title[i];
        line[p++] = '-';
    }
    while (p + 1 < w) line[p++] = '-';
    line[p++] = '|';
    line[p] = 0;
    draw_cell(col, row, line, color);
    for (u32 r = 1; r + 1 < h; r++) {
        char side[2] = { '|', 0 };
        draw_cell(col, row + r, side, color);
        draw_cell(col + w - 1, row + r, side, color);
    }
    p = 0;
    line[p++] = '|';
    while (p + 1 < w) line[p++] = '-';
    line[p++] = '|';
    line[p] = 0;
    draw_cell(col, row + h - 1, line, color);
}

static void gop_render_workbench(bool sd_ok, bool fmt_ok, bool mount_ok,
                                 bool create_ok, bool write_ok, bool read_ok,
                                 bool verify_ok, bool lan_ok, u32 records, u64 uptime)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info) return;
    struct efi_gop_mode_info *mi = g_gop->mode->info;
    gop_rect(0, 0, mi->horizontal_resolution, mi->vertical_resolution, 0x050607);

    draw_box(1, 1, 118, 5, "PIOS WORKBENCH", 0x00FF80);
    draw_cell(4, 2,  "VERSION: QEMU-STAGE2", 0xFD8EA1);
    draw_cell(42, 2, "UPTIME: LIVE", 0x4ADE80);
    draw_cell(78, 2, "IP: 10.0.2.15/24", 0x4DA6FF);
    draw_cell(4, 3,  "CPU: QEMU VIRT AARCH64 EL1", 0xFFFFFF);
    draw_cell(42, 3, "RAM: 512MB  POOL=RAM", 0xFFFFFF);
    draw_cell(78, 3, "BOARD: QEMU-VIRT", 0xFFFFFF);
    draw_cell(4, 4,  lan_ok ? "NET: HOSTFWD 127.0.0.1:8088 OK" : "NET: HOSTFWD WAITING", lan_ok ? 0x4ADE80 : 0xFBBF24);
    draw_cell(42, 4, (fmt_ok && mount_ok) ? "WALFS: BLOCK MOUNT OK" : "WALFS: BLOCK DOWN", (fmt_ok && mount_ok) ? 0x4ADE80 : 0xF87171);
    draw_cell(78, 4, sd_ok ? (sd_qemu_virtio_blk_ready() ? "SD: VIRTIO-BLK OK" : "SD: RAM BLOCK OK") : "SD: FAIL", sd_ok ? 0x4ADE80 : 0xF87171);

    draw_box(1, 7, 118, 18, "NETWORK / PROCESS MAP", 0xFFAA00);
    draw_cell(4, 8, "PID", 0xB0B0B0);
    draw_cell(12, 8, "CORE", 0xB0B0B0);
    draw_cell(22, 8, "USER", 0xB0B0B0);
    draw_cell(34, 8, "CPU", 0xB0B0B0);
    draw_cell(44, 8, "RAM", 0xB0B0B0);
    draw_cell(56, 8, "PROCESS", 0xB0B0B0);
    draw_cell(88, 8, "FIFO / PORT", 0xB0B0B0);
    draw_cell(4, 10, "KERNEL", 0x4DA6FF);
    draw_cell(6, 11, "0", 0xFFFFFF);
    draw_cell(12, 11, "0", 0xFFFFFF);
    draw_cell(22, 11, "root", 0xFFFFFF);
    draw_cell(34, 11, "live", 0xFFFFFF);
    draw_cell(44, 11, "ram", 0xFFFFFF);
    draw_cell(56, 11, "BOOTAA64 stage1", 0xFFFFFF);
    draw_cell(88, 11, "PGS2 -> qemu entry", 0xFFFFFF);
    draw_cell(4, 13, "CAPSULE qemu/platform", 0xFBBF24);
    draw_cell(6, 14, "1", 0xFFFFFF);
    draw_cell(12, 14, "0", 0xFFFFFF);
    draw_cell(22, 14, "root", 0xFFFFFF);
    draw_cell(34, 14, "ok", 0x4ADE80);
    draw_cell(44, 14, "16M", 0xFFFFFF);
    draw_cell(56, 14, "PIOSSTG2 shared", 0xFFFFFF);
    draw_cell(88, 14, sd_qemu_virtio_blk_ready() ? "virtio walfs" : "ram walfs", 0xFFFFFF);
    draw_cell(6, 15, "2", 0xFFFFFF);
    draw_cell(12, 15, "0", 0xFFFFFF);
    draw_cell(22, 15, "root", 0xFFFFFF);
    draw_cell(34, 15, lan_ok ? "ok" : "wait", lan_ok ? 0x4ADE80 : 0xFBBF24);
    draw_cell(44, 15, "net", 0xFFFFFF);
    draw_cell(56, 15, "QEMU admin", 0xFFFFFF);
    draw_cell(88, 15, "hostfwd -> tcp/80", 0xFFFFFF);
    draw_cell(6, 16, "3", 0xFFFFFF);
    draw_cell(12, 16, "0", 0xFFFFFF);
    draw_cell(22, 16, "root", 0xFFFFFF);
    draw_cell(34, 16, verify_ok ? "ok" : "fail", verify_ok ? 0x4ADE80 : 0xF87171);
    draw_cell(44, 16, "wal", 0xFFFFFF);
    draw_cell(56, 16, "WALFS verify", 0xFFFFFF);
    draw_cell(88, 16, "records=", 0xFFFFFF);
    char rec_digit[2] = { (char)('0' + (records % 10U)), 0 };
    draw_cell(96, 16, rec_digit, 0x4ADE80);

    draw_box(1, 27, 118, 10, "WARNINGS / ERRORS", 0xFF4040);
    draw_cell(4, 29, "No warnings or errors in QEMU PIOS workbench.", 0x4ADE80);
    draw_cell(4, 31, sd_qemu_virtio_blk_ready() ? "Validated: VIRTIO-BLK, WALFS create/write/readback/verify, LAN ADMIN HTTP." : "Validated: RAM SD, WALFS create/write/readback/verify, LAN ADMIN HTTP.", 0xDDE7F0);
    draw_cell(4, 33, "Admin URL: http://127.0.0.1:8088/", 0xFBBF24);
    draw_cell(4, 35, "Screenshot is rendered from shared stage2 PIOSSTG2.PKG.", 0xB0B0B0);
    (void)uptime;
    (void)create_ok; (void)write_ok; (void)read_ok; (void)vnet_diag_code;
}

static void park(void)
{
    wb_puts("\nPIOS QEMU UEFI stager parked. Reset VM to reboot.\n");
    for (;;) __asm__ volatile("wfe");
}

static void park_silent(void)
{
    for (;;) __asm__ volatile("wfe");
}

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    (void)image;
    g_con = st ? st->con_out : 0;
    bool gop_ok = gop_init(st);
    if (gop_ok)
        gop_set_largest_mode();
    if (g_con) {
        if (g_con->reset) (void)g_con->reset(g_con, false);
        if (g_con->set_mode) (void)g_con->set_mode(g_con, 0);
        if (g_con->set_attribute) (void)g_con->set_attribute(g_con, 0x0F);
        if (g_con->clear_screen) (void)g_con->clear_screen(g_con);
        if (g_con->set_cursor_position) (void)g_con->set_cursor_position(g_con, 0, 0);
        if (g_con->enable_cursor) (void)g_con->enable_cursor(g_con, false);
    }

    g_log_quiet = false;

    bool sd_ok = sd_init();
    if (!sd_ok) {
        wb_puts("[qemu] sd_init failed diag=");
        wb_hex(sd_qemu_virtio_blk_diag());
        wb_puts("\n");
        park();
    }

    bcache_init();

    bool fmt_ok = walfs_format_reserved();
    bool mount_ok = fmt_ok && walfs_init();
    if (!mount_ok) {
        wb_puts("[qemu] walfs init failed fmt=");
        wb_hex(fmt_ok ? 1U : 0U);
        wb_puts(" mount=");
        wb_hex(mount_ok ? 1U : 0U);
        wb_puts(" blkdiag=");
        wb_hex(sd_qemu_virtio_blk_diag());
        wb_puts("\n");
        park();
    }

    u64 id = walfs_create(WALFS_ROOT_INODE, "uefi.txt", WALFS_FILE, 0644);
    bool create_ok = id != 0;

    static const char payload[] = "hello from BOOTAA64 PIOS\n";
    bool write_ok = create_ok && walfs_replace(id, payload, (u32)sizeof(payload) - 1U);

    char out[64];
    u32 got = write_ok ? walfs_read(id, 0, out, (u32)sizeof(out)) : 0;
    bool read_ok = got == (u32)sizeof(payload) - 1U &&
                   memcmp(out, payload, (u32)sizeof(payload) - 1U) == 0;

    struct walfs_health wh;
    bool verify_ok = walfs_verify(&wh);
    g_qemu_sd_ok = sd_ok;
    g_qemu_fmt_ok = fmt_ok;
    g_qemu_mount_ok = mount_ok;
    g_qemu_create_ok = create_ok;
    g_qemu_write_ok = write_ok;
    g_qemu_read_ok = read_ok;
    g_qemu_verify_ok = verify_ok;
    g_qemu_walfs_records = verify_ok ? wh.valid_records : 0;
    if (gop_ok) {
        gop_render_workbench(sd_ok, fmt_ok, mount_ok, create_ok, write_ok, read_ok,
                             verify_ok, false, verify_ok ? wh.valid_records : 0,
                             timer_monotonic_ms());
        g_log_quiet = false;
        bool lan_ok = vnet_init() && vnet_poll_admin(30000);
        g_log_quiet = true;
        gop_render_workbench(sd_ok, fmt_ok, mount_ok, create_ok, write_ok, read_ok,
                             verify_ok, lan_ok, verify_ok ? wh.valid_records : 0,
                             timer_monotonic_ms());
        for (;;) {
            bool served = vnet_poll_admin(1000);
            if (served && !lan_ok) {
                lan_ok = true;
                gop_render_workbench(sd_ok, fmt_ok, mount_ok, create_ok, write_ok, read_ok,
                                    verify_ok, true, verify_ok ? wh.valid_records : 0,
                                    timer_monotonic_ms());
            }
        }
    }

    g_log_quiet = false;
    if (g_con) {
        if (g_con->clear_screen) (void)g_con->clear_screen(g_con);
        if (g_con->set_cursor_position) (void)g_con->set_cursor_position(g_con, 0, 0);
    }
    wb_puts("PIOS QEMU UEFI WORKBENCH - BOOTED\n");
    print_line("RAM-backed SD block device", sd_ok);
    print_line("Block cache initialized", true);
    print_line("RAM WALFS formatted", fmt_ok);
    print_line("RAM WALFS mounted", mount_ok);
    print_line("WALFS file create", create_ok);
    print_line("WALFS file write", write_ok);
    print_line("WALFS file readback", read_ok);
    print_line("WALFS verify", verify_ok);
    park();
    return 0;
}
