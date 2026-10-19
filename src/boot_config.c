#include "boot_config.h"
#include "fat32.h"

enum boot_config_status boot_config_load(struct boot_config *out)
{
    /* Boot-only, core-0 private; never publish the source text to a VM. */
    static u8 text[BOOT_CONFIG_MAX_BYTES];
    fat32_file_t file;
    enum boot_config_status status;

    if (!out)
        return BOOT_CONFIG_INVALID;
    boot_config_clear(out);
    if (!fat32_init() || !fat32_open("/pios.ini", &file))
        return BOOT_CONFIG_UNAVAILABLE;
    if (file.file_size > sizeof(text)) {
        fat32_close(&file);
        return BOOT_CONFIG_TOO_LARGE;
    }
    u32 length = file.file_size;
    u32 received = fat32_read(&file, text, length);
    fat32_close(&file);
    if (received != length)
        status = BOOT_CONFIG_IO;
    else
        status = boot_config_parse(text, length, out);
    volatile u8 *p = text;
    for (u32 i = 0; i < sizeof(text); i++)
        p[i] = 0;
    return status;
}
