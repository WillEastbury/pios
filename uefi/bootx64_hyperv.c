/*
 * BOOTX64.EFI - PIOS Hyper-V amd64 probe.
 *
 * This is a UEFI probe only. It proves that a Gen2 Hyper-V VM can execute
 * PIOS-owned x86_64 UEFI code before the full kernel entry/MMU/interrupt/
 * timer/VMBus/netvsc layers are ported. It is not PIOS OS boot or online.
 */
#include "x86_kernel.h"
typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;
typedef unsigned long long u64;
typedef unsigned long long usize;
typedef u64                efi_status_t;
typedef void              *efi_handle_t;

#define EFI_SUCCESS 0ULL
#define EFI_ERROR(code) (0x8000000000000000ULL | (code))
#define EFI_INVALID_PARAMETER EFI_ERROR(2ULL)
#define EFI_BUFFER_TOO_SMALL EFI_ERROR(5ULL)
#define ALLOCATE_ANY_PAGES 0U
#define EFI_LOADER_DATA 2U

struct efi_table_header {
    u64 signature;
    u32 revision;
    u32 header_size;
    u32 crc32;
    u32 reserved;
};

struct efi_guid {
    u32 data1;
    u16 data2;
    u16 data3;
    u8 data4[8];
};

typedef efi_status_t (*efi_text_string_fn)(void *self, const u16 *str);
typedef efi_status_t (*efi_allocate_pages_fn)(u32 type, u32 memtype, usize pages, u64 *memory);
typedef efi_status_t (*efi_allocate_pool_fn)(u32 memtype, usize size, void **buffer);
typedef efi_status_t (*efi_get_memory_map_fn)(usize *size, void *map, usize *key,
                                               usize *descriptor_size, u32 *descriptor_version);
typedef efi_status_t (*efi_exit_boot_services_fn)(efi_handle_t image, usize key);
typedef efi_status_t (*efi_stall_fn)(usize usec);
typedef efi_status_t (*efi_locate_protocol_fn)(struct efi_guid *protocol,
                                                void *registration,
                                                void **interface);

struct efi_simple_text_output {
    void *reset;
    efi_text_string_fn output_string;
};

struct efi_boot_services {
    struct efi_table_header hdr;
    void *raise_tpl; void *restore_tpl;
    efi_allocate_pages_fn allocate_pages;
    void *free_pages;
    efi_get_memory_map_fn get_memory_map;
    efi_allocate_pool_fn allocate_pool;
    void *free_pool;
    void *create_event;
    void *set_timer;
    void *wait_for_event;
    void *signal_event;
    void *close_event;
    void *check_event;
    void *install_protocol_interface;
    void *reinstall_protocol_interface;
    void *uninstall_protocol_interface;
    void *handle_protocol;
    void *reserved;
    void *register_protocol_notify;
    void *locate_handle;
    void *locate_device_path;
    void *install_configuration_table;
    void *load_image;
    void *start_image;
    void *exit;
    void *unload_image;
    efi_exit_boot_services_fn exit_boot_services;
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

struct efi_configuration_table {
    struct efi_guid vendor_guid;
    void *vendor_table;
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
    usize number_of_table_entries;
    struct efi_configuration_table *configuration_table;
};

static struct efi_simple_text_output *g_con;
static int g_serial_ready;

struct acpi_summary {
    u32 tables;
    u32 checksum_ok;
    u32 checksum_bad;
    int found_facp;
    int found_oem0;
    int found_waet;
    int found_apic;
    int found_srat;
    int found_bgrt;
    u32 fadt_sci;
    u32 fadt_pm_profile;
    u64 fadt_dsdt;
    u64 fadt_x_dsdt;
    u32 madt_lapic_addr;
    u32 madt_flags;
    u32 madt_lapic;
    u32 madt_ioapic;
    u32 madt_iso;
    u32 madt_x2apic;
    u32 madt_other;
};

static struct acpi_summary g_acpi;

struct hyperv_summary {
    u32 max_leaf;
    u32 interface_id;
    u32 features_eax;
    u32 features_ebx;
    u32 features_ecx;
    u32 features_edx;
};

static struct hyperv_summary g_hv;
static struct x86_uefi_handoff g_handoff;
static struct efi_graphics_output *g_gop;

static void outb(u16 port, u8 val)
{
#if defined(__x86_64__) || defined(_M_X64)
    __asm__ volatile("outb %0, %1" :: "a"(val), "Nd"(port));
#else
    (void)port; (void)val;
#endif
}

static u8 inb(u16 port)
{
#if defined(__x86_64__) || defined(_M_X64)
    u8 val;
    __asm__ volatile("inb %1, %0" : "=a"(val) : "Nd"(port));
    return val;
#else
    (void)port;
    return 0;
#endif
}

static void serial_init(void)
{
    const u16 com1 = 0x3F8;
    outb(com1 + 1, 0x00);
    outb(com1 + 3, 0x80);
    outb(com1 + 0, 0x01);
    outb(com1 + 1, 0x00);
    outb(com1 + 3, 0x03);
    outb(com1 + 2, 0xC7);
    outb(com1 + 4, 0x0B);
    g_serial_ready = 1;
}

static void serial_putc(char c)
{
    const u16 com1 = 0x3F8;
    if (!g_serial_ready)
        return;
    for (u32 spin = 0; spin < 100000U; spin++) {
        if (inb(com1 + 5) & 0x20U) {
            outb(com1, (u8)c);
            return;
        }
    }
}

static int guid_eq(const struct efi_guid *a, const struct efi_guid *b)
{
    if (a->data1 != b->data1 || a->data2 != b->data2 || a->data3 != b->data3)
        return 0;
    for (u32 i = 0; i < 8; i++)
        if (a->data4[i] != b->data4[i])
            return 0;
    return 1;
}

static void puts16(const u16 *s)
{
    if (g_con && g_con->output_string)
        g_con->output_string(g_con, s);
}

static void puts_ascii(const char *s)
{
    u16 b[128];
    while (*s) {
        u32 n = 0;
        while (*s && n + 1 < (u32)(sizeof(b) / sizeof(b[0]))) {
            char c = *s++;
            if (c == '\n') {
                serial_putc('\r');
                serial_putc('\n');
                b[n++] = '\r';
            } else {
                serial_putc(c);
            }
            b[n++] = (u16)c;
        }
        b[n] = 0;
        puts16(b);
    }
}

static void put_hex64(u64 v)
{
    static const char hx[] = "0123456789ABCDEF";
    char s[19];
    s[0] = '0';
    s[1] = 'x';
    for (u32 i = 0; i < 16; i++)
        s[2 + i] = hx[(v >> ((15U - i) * 4U)) & 0xFULL];
    s[18] = 0;
    puts_ascii(s);
}

static void put_bool(int value)
{
    puts_ascii(value ? "yes" : "no");
}

static u16 rd16(const void *ptr)
{
    const u8 *p = (const u8 *)ptr;
    return (u16)p[0] | ((u16)p[1] << 8);
}

static u32 rd32(const void *ptr)
{
    const u8 *p = (const u8 *)ptr;
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u64 rd64(const void *ptr)
{
    const u8 *p = (const u8 *)ptr;
    return (u64)rd32(p) | ((u64)rd32(p + 4) << 32);
}

static int sig_eq(const void *ptr, const char *sig)
{
    const u8 *p = (const u8 *)ptr;
    for (u32 i = 0; i < 4; i++)
        if ((char)p[i] != sig[i])
            return 0;
    return 1;
}

static void put_sig4(const void *ptr)
{
    char s[5];
    const u8 *p = (const u8 *)ptr;
    for (u32 i = 0; i < 4; i++) {
        char c = (char)p[i];
        s[i] = (c >= 32 && c <= 126) ? c : '.';
    }
    s[4] = 0;
    puts_ascii(s);
}

static void put_oem6(const void *ptr)
{
    char s[7];
    const u8 *p = (const u8 *)ptr;
    for (u32 i = 0; i < 6; i++) {
        char c = (char)p[i];
        s[i] = (c >= 32 && c <= 126) ? c : '.';
    }
    s[6] = 0;
    puts_ascii(s);
}

static int checksum_ok(const void *ptr, u32 len)
{
    const u8 *p = (const u8 *)ptr;
    u8 sum = 0;
    if (!p || len == 0 || len > (1024U * 1024U))
        return 0;
    for (u32 i = 0; i < len; i++)
        sum = (u8)(sum + p[i]);
    return sum == 0;
}

static void cpuid(u32 leaf, u32 subleaf, u32 *a, u32 *b, u32 *c, u32 *d)
{
#if defined(__x86_64__) || defined(_M_X64)
    __asm__ volatile("cpuid"
                     : "=a"(*a), "=b"(*b), "=c"(*c), "=d"(*d)
                     : "a"(leaf), "c"(subleaf)
                     : "memory");
#else
    (void)leaf; (void)subleaf;
    *a = *b = *c = *d = 0;
#endif
}

static u64 rdmsr(u32 msr)
{
#if defined(__x86_64__) || defined(_M_X64)
    u32 lo, hi;
    __asm__ volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr) : "memory");
    return (u64)lo | ((u64)hi << 32);
#else
    (void)msr;
    return 0;
#endif
}

static void wrmsr(u32 msr, u64 value)
{
#if defined(__x86_64__) || defined(_M_X64)
    u32 lo = (u32)value;
    u32 hi = (u32)(value >> 32);
    __asm__ volatile("wrmsr" :: "c"(msr), "a"(lo), "d"(hi) : "memory");
#else
    (void)msr; (void)value;
#endif
}

static void zero_page(void *ptr)
{
    u64 *p = (u64 *)ptr;
    for (u32 i = 0; i < 4096U / sizeof(u64); i++)
        p[i] = 0;
}

static void vendor_from_regs(char out[13], u32 b, u32 c, u32 d)
{
    out[0] = (char)(b & 0xFF); out[1] = (char)((b >> 8) & 0xFF);
    out[2] = (char)((b >> 16) & 0xFF); out[3] = (char)((b >> 24) & 0xFF);
    out[4] = (char)(c & 0xFF); out[5] = (char)((c >> 8) & 0xFF);
    out[6] = (char)((c >> 16) & 0xFF); out[7] = (char)((c >> 24) & 0xFF);
    out[8] = (char)(d & 0xFF); out[9] = (char)((d >> 8) & 0xFF);
    out[10] = (char)((d >> 16) & 0xFF); out[11] = (char)((d >> 24) & 0xFF);
    out[12] = 0;
}

static void cpu_vendor_from_regs(char out[13], u32 b, u32 c, u32 d)
{
    vendor_from_regs(out, b, d, c);
}

static void print_cpuid_probe(void)
{
    u32 a, b, c, d;
    char vendor[13];

    cpuid(0, 0, &a, &b, &c, &d);
    cpu_vendor_from_regs(vendor, b, c, d);
    puts_ascii("[cpu] vendor=");
    puts_ascii(vendor);
    puts_ascii(" max=");
    put_hex64(a);
    puts_ascii("\n");

    cpuid(1, 0, &a, &b, &c, &d);
    int hypervisor_present = (c & (1U << 31)) != 0;
    puts_ascii("[hv] cpuid.hypervisor=");
    puts_ascii(hypervisor_present ? "1" : "0");
    puts_ascii(" leaf1.ecx=");
    put_hex64(c);
    puts_ascii("\n");

    cpuid(0x40000000U, 0, &a, &b, &c, &d);
    vendor_from_regs(vendor, b, c, d);
    puts_ascii("[hv] vendor=");
    puts_ascii(vendor);
    puts_ascii(" max=");
    put_hex64(a);
    puts_ascii("\n");
    g_hv.max_leaf = a;
    u32 max_hv_leaf = a;
    for (u32 leaf = 0x40000001U; leaf <= max_hv_leaf && leaf <= 0x40000006U; leaf++) {
        cpuid(leaf, 0, &a, &b, &c, &d);
        if (leaf == 0x40000001U)
            g_hv.interface_id = a;
        else if (leaf == 0x40000003U) {
            g_hv.features_eax = a;
            g_hv.features_ebx = b;
            g_hv.features_ecx = c;
            g_hv.features_edx = d;
        }
        puts_ascii("[hvcpuid] leaf=");
        put_hex64(leaf);
        puts_ascii(" eax=");
        put_hex64(a);
        puts_ascii(" ebx=");
        put_hex64(b);
        puts_ascii(" ecx=");
        put_hex64(c);
        puts_ascii(" edx=");
        put_hex64(d);
        if (leaf == 0x40000001U) {
            char iface[5];
            iface[0] = (char)(a & 0xFF);
            iface[1] = (char)((a >> 8) & 0xFF);
            iface[2] = (char)((a >> 16) & 0xFF);
            iface[3] = (char)((a >> 24) & 0xFF);
            iface[4] = 0;
            puts_ascii(" iface=");
            puts_ascii(iface);
        }
        puts_ascii("\n");
    }
}

static int hv_feature(u32 bit)
{
    return (g_hv.features_eax & (1U << bit)) != 0U;
}

static void print_hv_feature_bool(const char *name, int value)
{
    puts_ascii(" ");
    puts_ascii(name);
    puts_ascii("=");
    put_bool(value);
}

static void print_hv_msr_value(const char *name, u32 msr)
{
    puts_ascii("[hvmsr] ");
    puts_ascii(name);
    puts_ascii(" msr=");
    put_hex64(msr);
    puts_ascii(" value=");
    put_hex64(rdmsr(msr));
    puts_ascii("\n");
}

static void print_hyperv_summary(void)
{
    puts_ascii("[summary] hyperv max_leaf=");
    put_hex64(g_hv.max_leaf);
    puts_ascii(" iface=");
    {
        char iface[5];
        iface[0] = (char)(g_hv.interface_id & 0xFF);
        iface[1] = (char)((g_hv.interface_id >> 8) & 0xFF);
        iface[2] = (char)((g_hv.interface_id >> 16) & 0xFF);
        iface[3] = (char)((g_hv.interface_id >> 24) & 0xFF);
        iface[4] = 0;
        puts_ascii(iface);
    }
    puts_ascii(" features=");
    put_hex64(g_hv.features_eax);
    puts_ascii("\n");

    puts_ascii("[summary] hvfeat");
    print_hv_feature_bool("vp_runtime", hv_feature(0));
    print_hv_feature_bool("ref_count", hv_feature(1));
    print_hv_feature_bool("synic", hv_feature(2));
    print_hv_feature_bool("stimer", hv_feature(3));
    print_hv_feature_bool("apic_msr", hv_feature(4));
    print_hv_feature_bool("hypercall", hv_feature(5));
    print_hv_feature_bool("vp_index", hv_feature(6));
    print_hv_feature_bool("reset", hv_feature(7));
    print_hv_feature_bool("ref_tsc", hv_feature(9));
    print_hv_feature_bool("freq", hv_feature(11));
    puts_ascii("\n");

    puts_ascii("[summary] hvpriv ebx=");
    put_hex64(g_hv.features_ebx);
    puts_ascii(" ecx=");
    put_hex64(g_hv.features_ecx);
    puts_ascii(" edx=");
    put_hex64(g_hv.features_edx);
    puts_ascii("\n");
}

static void print_hv_msr_probe(void)
{
    puts_ascii("[hvmsr] read-only probe start\n");
    if (hv_feature(5)) {
        print_hv_msr_value("guest_os_id", 0x40000000U);
        print_hv_msr_value("hypercall", 0x40000001U);
    }
    if (hv_feature(6))
        print_hv_msr_value("vp_index", 0x40000002U);
    if (hv_feature(0))
        print_hv_msr_value("vp_runtime", 0x40000010U);
    if (hv_feature(1))
        print_hv_msr_value("time_ref_count", 0x40000020U);
    if (hv_feature(9))
        print_hv_msr_value("reference_tsc", 0x40000021U);
    if (hv_feature(11)) {
        print_hv_msr_value("tsc_frequency", 0x40000022U);
        print_hv_msr_value("apic_frequency", 0x40000023U);
    }
    puts_ascii("[hvmsr] read-only probe complete\n");
}

static void enable_hypercall_page(struct efi_system_table *st)
{
    const u64 guest_os_id = 0x50494F5300000001ULL; /* "PIOS", build 1 */
    u64 page = 0;
    if (!hv_feature(5)) {
        puts_ascii("[hypercall] skipped hypercall_feature=no\n");
        return;
    }
    if (!st || !st->boot_services || !st->boot_services->allocate_pages) {
        puts_ascii("[hypercall] failed no_allocate_pages\n");
        return;
    }
    efi_status_t status = st->boot_services->allocate_pages(ALLOCATE_ANY_PAGES,
                                                            EFI_LOADER_DATA,
                                                            1,
                                                            &page);
    puts_ascii("[hypercall] alloc status=");
    put_hex64(status);
    puts_ascii(" page=");
    put_hex64(page);
    puts_ascii("\n");
    if (status != EFI_SUCCESS || page == 0)
        return;
    zero_page((void *)(usize)page);
    wrmsr(0x40000000U, guest_os_id);
    u64 before = rdmsr(0x40000001U);
    wrmsr(0x40000001U, (page & ~0xFFFULL) | 1ULL);
    u64 after = rdmsr(0x40000001U);
    puts_ascii("[hypercall] guest_os_id=");
    put_hex64(rdmsr(0x40000000U));
    puts_ascii(" before=");
    put_hex64(before);
    puts_ascii(" after=");
    put_hex64(after);
    puts_ascii(" enabled=");
    put_bool((after & 1ULL) != 0);
    puts_ascii(" pfn=");
    put_hex64(after >> 12);
    puts_ascii("\n");
}

static void print_fadt_probe(const u8 *tbl, u32 len)
{
    puts_ascii(" detail=fadt");
    if (len >= 116U) {
        g_acpi.fadt_dsdt = rd32(tbl + 40);
        puts_ascii(" dsdt=");
        put_hex64(g_acpi.fadt_dsdt);
    }
    if (len >= 148U) {
        g_acpi.fadt_x_dsdt = rd64(tbl + 140);
        puts_ascii(" x_dsdt=");
        put_hex64(g_acpi.fadt_x_dsdt);
    }
    if (len >= 116U) {
        g_acpi.fadt_sci = rd16(tbl + 46);
        g_acpi.fadt_pm_profile = tbl[45];
        puts_ascii(" sci=");
        put_hex64(g_acpi.fadt_sci);
        puts_ascii(" pm_profile=");
        put_hex64(g_acpi.fadt_pm_profile);
    }
}

static void print_madt_probe(const u8 *tbl, u32 len)
{
    if (len < 44U)
        return;
    g_acpi.madt_lapic_addr = rd32(tbl + 36);
    g_acpi.madt_flags = rd32(tbl + 40);
    puts_ascii(" detail=madt lapic=");
    put_hex64(g_acpi.madt_lapic_addr);
    puts_ascii(" flags=");
    put_hex64(g_acpi.madt_flags);
    u32 off = 44;
    u32 lapic = 0, ioapic = 0, iso = 0, x2apic = 0, other = 0;
    while (off + 2 <= len) {
        u8 type = tbl[off];
        u8 n = tbl[off + 1];
        if (n < 2 || off + n > len)
            break;
        if (type == 0)
            lapic++;
        else if (type == 1)
            ioapic++;
        else if (type == 2)
            iso++;
        else if (type == 9)
            x2apic++;
        else
            other++;
        off += n;
    }
    puts_ascii(" lapic_n=");
    put_hex64(lapic);
    puts_ascii(" ioapic_n=");
    put_hex64(ioapic);
    puts_ascii(" iso_n=");
    put_hex64(iso);
    puts_ascii(" x2apic_n=");
    put_hex64(x2apic);
    puts_ascii(" other_n=");
    put_hex64(other);
    g_acpi.madt_lapic = lapic;
    g_acpi.madt_ioapic = ioapic;
    g_acpi.madt_iso = iso;
    g_acpi.madt_x2apic = x2apic;
    g_acpi.madt_other = other;
}

static void print_sdt_table(const u8 *tbl, u32 index)
{
    if (!tbl)
        return;
    u32 len = rd32(tbl + 4);
    int ok = checksum_ok(tbl, len);
    g_acpi.tables++;
    if (ok)
        g_acpi.checksum_ok++;
    else
        g_acpi.checksum_bad++;
    if (sig_eq(tbl, "FACP"))
        g_acpi.found_facp = 1;
    else if (sig_eq(tbl, "OEM0"))
        g_acpi.found_oem0 = 1;
    else if (sig_eq(tbl, "WAET"))
        g_acpi.found_waet = 1;
    else if (sig_eq(tbl, "APIC"))
        g_acpi.found_apic = 1;
    else if (sig_eq(tbl, "SRAT"))
        g_acpi.found_srat = 1;
    else if (sig_eq(tbl, "BGRT"))
        g_acpi.found_bgrt = 1;

    puts_ascii("[acpi] table[");
    put_hex64(index);
    puts_ascii("] sig=");
    put_sig4(tbl);
    puts_ascii(" addr=");
    put_hex64((u64)(usize)tbl);
    puts_ascii(" len=");
    put_hex64(len);
    puts_ascii(" rev=");
    put_hex64(tbl[8]);
    puts_ascii(" csum=");
    puts_ascii(ok ? "ok" : "BAD");
    puts_ascii(" oem=");
    put_oem6(tbl + 10);
    if (sig_eq(tbl, "FACP"))
        print_fadt_probe(tbl, len);
    else if (sig_eq(tbl, "APIC"))
        print_madt_probe(tbl, len);
    puts_ascii("\n");
}

static void print_acpi_summary(void)
{
    puts_ascii("[summary] acpi tables=");
    put_hex64(g_acpi.tables);
    puts_ascii(" ok=");
    put_hex64(g_acpi.checksum_ok);
    puts_ascii(" bad=");
    put_hex64(g_acpi.checksum_bad);
    puts_ascii("\n");

    puts_ascii("[summary] present FACP=");
    put_bool(g_acpi.found_facp);
    puts_ascii(" OEM0=");
    put_bool(g_acpi.found_oem0);
    puts_ascii(" WAET=");
    put_bool(g_acpi.found_waet);
    puts_ascii(" APIC=");
    put_bool(g_acpi.found_apic);
    puts_ascii(" SRAT=");
    put_bool(g_acpi.found_srat);
    puts_ascii(" BGRT=");
    put_bool(g_acpi.found_bgrt);
    puts_ascii("\n");

    puts_ascii("[summary] fadt sci=");
    put_hex64(g_acpi.fadt_sci);
    puts_ascii(" profile=");
    put_hex64(g_acpi.fadt_pm_profile);
    puts_ascii(" dsdt=");
    put_hex64(g_acpi.fadt_dsdt);
    puts_ascii(" x_dsdt=");
    put_hex64(g_acpi.fadt_x_dsdt);
    puts_ascii("\n");

    puts_ascii("[summary] madt lapic_addr=");
    put_hex64(g_acpi.madt_lapic_addr);
    puts_ascii(" flags=");
    put_hex64(g_acpi.madt_flags);
    puts_ascii(" lapic=");
    put_hex64(g_acpi.madt_lapic);
    puts_ascii(" ioapic=");
    put_hex64(g_acpi.madt_ioapic);
    puts_ascii(" iso=");
    put_hex64(g_acpi.madt_iso);
    puts_ascii(" x2apic=");
    put_hex64(g_acpi.madt_x2apic);
    puts_ascii(" other=");
    put_hex64(g_acpi.madt_other);
    puts_ascii("\n");
}

static void parse_rsdt(const u8 *rsdt)
{
    if (!rsdt)
        return;
    u32 len = rd32(rsdt + 4);
    u32 entries = (len >= 36U) ? (len - 36U) / 4U : 0;
    puts_ascii("[acpi] RSDT addr=");
    put_hex64((u64)(usize)rsdt);
    puts_ascii(" len=");
    put_hex64(len);
    puts_ascii(" entries=");
    put_hex64(entries);
    puts_ascii(" csum=");
    puts_ascii(checksum_ok(rsdt, len) ? "ok" : "BAD");
    puts_ascii("\n");
    if (entries > 32U)
        entries = 32U;
    for (u32 i = 0; i < entries; i++)
        print_sdt_table((const u8 *)(usize)rd32(rsdt + 36U + i * 4U), i);
}

static void parse_xsdt(const u8 *xsdt)
{
    if (!xsdt)
        return;
    u32 len = rd32(xsdt + 4);
    u32 entries = (len >= 36U) ? (len - 36U) / 8U : 0;
    puts_ascii("[acpi] XSDT addr=");
    put_hex64((u64)(usize)xsdt);
    puts_ascii(" len=");
    put_hex64(len);
    puts_ascii(" entries=");
    put_hex64(entries);
    puts_ascii(" csum=");
    puts_ascii(checksum_ok(xsdt, len) ? "ok" : "BAD");
    puts_ascii("\n");
    if (entries > 32U)
        entries = 32U;
    for (u32 i = 0; i < entries; i++)
        print_sdt_table((const u8 *)(usize)rd64(xsdt + 36U + i * 8U), i);
}

static void parse_rsdp(const void *rsdp)
{
    const u8 *p = (const u8 *)rsdp;
    if (!p)
        return;
    puts_ascii("[acpi] RSDP addr=");
    put_hex64((u64)(usize)p);
    puts_ascii(" sig=");
    for (u32 i = 0; i < 8; i++) {
        char s[2];
        s[0] = (char)p[i];
        s[1] = 0;
        puts_ascii(s);
    }
    puts_ascii(" rev=");
    put_hex64(p[15]);
    puts_ascii(" csum20=");
    puts_ascii(checksum_ok(p, 20U) ? "ok" : "BAD");
    puts_ascii(" oem=");
    put_oem6(p + 9);
    puts_ascii("\n");

    u32 rsdt_addr = rd32(p + 16);
    u32 rsdp_len = p[15] >= 2 ? rd32(p + 20) : 20U;
    u64 xsdt_addr = (p[15] >= 2 && rsdp_len >= 36U) ? rd64(p + 24) : 0;
    if (p[15] >= 2) {
        puts_ascii("[acpi] RSDP len=");
        put_hex64(rsdp_len);
        puts_ascii(" csumX=");
        puts_ascii(checksum_ok(p, rsdp_len) ? "ok" : "BAD");
        puts_ascii(" xsdt=");
        put_hex64(xsdt_addr);
        puts_ascii(" rsdt=");
        put_hex64(rsdt_addr);
        puts_ascii("\n");
    }
    if (xsdt_addr)
        parse_xsdt((const u8 *)(usize)xsdt_addr);
    else if (rsdt_addr)
        parse_rsdt((const u8 *)(usize)rsdt_addr);
}

static void print_acpi_probe(struct efi_system_table *st)
{
    static const struct efi_guid acpi20 = {
        0x8868E871U, 0xE4F1U, 0x11D3U,
        { 0xBCU, 0x22U, 0x00U, 0x80U, 0xC7U, 0x3CU, 0x88U, 0x81U }
    };
    static const struct efi_guid acpi10 = {
        0xEB9D2D30U, 0x2D88U, 0x11D3U,
        { 0x9AU, 0x16U, 0x00U, 0x90U, 0x27U, 0x3FU, 0xC1U, 0x4DU }
    };
    void *rsdp20 = 0;
    void *rsdp10 = 0;

    if (st && st->configuration_table) {
        for (usize i = 0; i < st->number_of_table_entries; i++) {
            struct efi_configuration_table *t = &st->configuration_table[i];
            if (guid_eq(&t->vendor_guid, &acpi20))
                rsdp20 = t->vendor_table;
            if (guid_eq(&t->vendor_guid, &acpi10))
                rsdp10 = t->vendor_table;
        }

    }

    puts_ascii("[uefi] config_tables=");
    put_hex64(st ? st->number_of_table_entries : 0);
    puts_ascii(" acpi2=");
    put_hex64((u64)(usize)rsdp20);
    puts_ascii(" acpi1=");
    put_hex64((u64)(usize)rsdp10);
    puts_ascii("\n");
    parse_rsdp(rsdp20 ? rsdp20 : rsdp10);
    print_acpi_summary();
}

static void gop_probe(struct efi_system_table *st)
{
    static struct efi_guid gop_guid = {
        0x9042A9DEU, 0x23DCU, 0x4A38U,
        { 0x96U, 0xFBU, 0x7AU, 0xDEU, 0xD0U, 0x80U, 0x51U, 0x6AU }
    };
    if (!st || !st->boot_services || !st->boot_services->locate_protocol ||
        st->boot_services->locate_protocol(&gop_guid, 0, (void **)&g_gop) != EFI_SUCCESS ||
        !g_gop || !g_gop->mode || !g_gop->mode->info) {
        puts_ascii("[gop] unavailable; x86 handoff has no visual marker\n");
        g_gop = 0;
        return;
    }
    puts_ascii("[gop] fb=");
    put_hex64(g_gop->mode->framebuffer_base);
    puts_ascii(" width=");
    put_hex64(g_gop->mode->info->horizontal_resolution);
    puts_ascii(" height=");
    put_hex64(g_gop->mode->info->vertical_resolution);
    puts_ascii("\n");
}

static void gop_marker(u32 bgr)
{
    if (!g_gop || !g_gop->mode || !g_gop->mode->info ||
        g_gop->mode->framebuffer_base == 0U ||
        g_gop->mode->info->horizontal_resolution < 64U ||
        g_gop->mode->info->vertical_resolution < 64U ||
        g_gop->mode->info->pixels_per_scan_line <
            g_gop->mode->info->horizontal_resolution)
        return;
    volatile u32 *pixels = (volatile u32 *)(usize)g_gop->mode->framebuffer_base;
    for (u32 y = 0; y < 64U; y++)
        for (u32 x = 0; x < 64U; x++)
            pixels[y * g_gop->mode->info->pixels_per_scan_line + x] = bgr;
}

static efi_status_t x86_exit_boot_services(efi_handle_t image,
                                           struct efi_system_table *st)
{
    if (!st || !st->boot_services || !st->boot_services->get_memory_map ||
        !st->boot_services->allocate_pool || !st->boot_services->exit_boot_services)
        return EFI_INVALID_PARAMETER;
    usize bytes = 0, key = 0, descriptor_bytes = 0;
    u32 descriptor_version = 0;
    efi_status_t status = st->boot_services->get_memory_map(
        &bytes, 0, &key, &descriptor_bytes, &descriptor_version);
    if (status != EFI_BUFFER_TOO_SMALL)
        return status;
    /* Hyper-V's first size-only query returns BUFFER_TOO_SMALL but may leave
     * DescriptorSize zero. UEFI v2 descriptors are at least 48 bytes; use a
     * bounded conservative fallback and let the second query be authoritative. */
    if (descriptor_bytes == 0)
        descriptor_bytes = 48U;
    if (bytes < descriptor_bytes)
        bytes = descriptor_bytes * 128U;
    if (bytes > 16ULL * 1024ULL * 1024ULL - descriptor_bytes * 8ULL)
        return EFI_BUFFER_TOO_SMALL;
    void *map = 0;
    bytes += descriptor_bytes * 8U;
    status = st->boot_services->allocate_pool(EFI_LOADER_DATA, bytes, &map);
    if (status != EFI_SUCCESS || !map)
        return status;
    g_handoff = (struct x86_uefi_handoff){
        .memory_map = (u64)(usize)map,
        .memory_map_bytes = 0U,
        .memory_map_key = 0U,
        .descriptor_bytes = descriptor_bytes,
        .descriptor_version = descriptor_version,
        .system_table = (u64)(usize)st,
        .image_handle = (u64)(usize)image,
        .framebuffer_base = g_gop ? g_gop->mode->framebuffer_base : 0U,
        .framebuffer_bytes = g_gop ? g_gop->mode->framebuffer_size : 0U,
        .framebuffer_width = g_gop ? g_gop->mode->info->horizontal_resolution : 0U,
        .framebuffer_height = g_gop ? g_gop->mode->info->vertical_resolution : 0U,
        .framebuffer_stride = g_gop ? g_gop->mode->info->pixels_per_scan_line : 0U,
        .framebuffer_format = g_gop ? g_gop->mode->info->pixel_format : 0U,
    };
    /* All diagnostics and table construction must precede the final memory
     * map query. UEFI output may allocate and invalidate its map key. */
    x86_kernel_prepare(&g_handoff);
    struct x86_kernel_boot_state state;
    x86_kernel_boot_snapshot(&state);
    puts_ascii("[handoff] cr3=");
    put_hex64(state.cr3);
    puts_ascii(" stack=");
    put_hex64(state.stack_top);
    puts_ascii(" map=");
    put_hex64(state.mapped_bytes);
    puts_ascii("\n");
    for (u32 attempt = 0; attempt < 3U; attempt++) {
        usize used = bytes;
        status = st->boot_services->get_memory_map(
            &used, map, &key, &descriptor_bytes, &descriptor_version);
        if (status == EFI_BUFFER_TOO_SMALL) {
            if (descriptor_bytes == 0)
                descriptor_bytes = 48U;
            if (used > 16ULL * 1024ULL * 1024ULL - descriptor_bytes * 8ULL)
                return EFI_BUFFER_TOO_SMALL;
            bytes = used + descriptor_bytes * 8U;
            map = 0;
            status = st->boot_services->allocate_pool(EFI_LOADER_DATA, bytes, &map);
            if (status != EFI_SUCCESS || !map)
                return status;
            continue;
        }
        if (status != EFI_SUCCESS || descriptor_bytes == 0)
            return status;
        g_handoff.memory_map = (u64)(usize)map;
        g_handoff.memory_map_bytes = used;
        g_handoff.memory_map_key = key;
        g_handoff.descriptor_bytes = descriptor_bytes;
        g_handoff.descriptor_version = descriptor_version;
        status = st->boot_services->exit_boot_services(image, key);
        if (status == EFI_SUCCESS) {
            gop_marker(0x00FFFFFFU); /* white: firmware returned after exit */
            x86_kernel_jump(&g_handoff, state.cr3, state.stack_top);
        }
        if (status != EFI_INVALID_PARAMETER)
            return status;
    }
    return EFI_INVALID_PARAMETER;
}

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    (void)image;
    serial_init();
    g_con = st ? st->con_out : 0;

    puts_ascii("\nPIOS Hyper-V amd64 probe\n");
    puts_ascii("========================\n");
    puts_ascii("[arch] x86_64 UEFI application loaded\n");
    puts_ascii("[platform] target=hyperv-amd64 local-test=yes\n");
    print_cpuid_probe();
    print_hyperv_summary();
    print_hv_msr_probe();
    enable_hypercall_page(st);
    print_acpi_probe(st);
    gop_probe(st);
    gop_marker(0x0000CCCCU); /* yellow: UEFI is about to prepare PIOS handoff */
    puts_ascii("[handoff] ExitBootServices -> PIOS x86 kernel boundary\n");
    efi_status_t handoff = x86_exit_boot_services(image, st);
    gop_marker(0x00CC00CCU); /* magenta: boot services exit/handoff returned */
    puts_ascii("[handoff] FAILED status=");
    put_hex64(handoff);
    puts_ascii("\n");
    /* Preserve the failure screen; returning sends control to Boot Manager. */
    for (;;)
#if defined(__x86_64__) || defined(_M_X64)
        __asm__ volatile("hlt");
#else
        ;
#endif
    return handoff;
}
