/*
 * Minimal ARM64 UEFI application for QEMU/EDK2 bring-up.
 *
 * It deliberately avoids gnu-efi dependencies: enough EFI table layout to print
 * to ConOut, then returns to firmware. Next slice can load/jump to PIOS.
 */
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;
typedef unsigned long long usize;
typedef u64 efi_status_t;
typedef void *efi_handle_t;

struct efi_table_header {
    u64 signature;
    u32 revision;
    u32 header_size;
    u32 crc32;
    u32 reserved;
};

typedef efi_status_t (*efi_text_string_fn)(void *self, const u16 *str);

struct efi_simple_text_output {
    void *reset;
    efi_text_string_fn output_string;
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
};

static const u16 hello[] =
    L"PIOS BOOTAA64.EFI: QEMU UEFI console online\r\n"
    L"Next: load unified stage2 and jump via PGS2 manifest\r\n";

efi_status_t efi_main(efi_handle_t image, struct efi_system_table *st)
{
    (void)image;
    if (st && st->con_out && st->con_out->output_string)
        st->con_out->output_string(st->con_out, hello);
    return 0;
}
