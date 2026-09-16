#include "boot_precedence.h"

u32 bootctrl_checksum_versioned(const u8 *sector, u32 version)
{
    u32 off = version == PIOS_BOOTCTRL_VERSION_V1 ?
              PIOS_BOOTCTRL_V1_CHECKSUM_OFF : PIOS_BOOTCTRL_V2_CHECKSUM_OFF;
    u32 sum = 0xB007C0DEU;
    for (u32 i = 0; i < off; i++)
        sum = (sum << 5) ^ (sum >> 27) ^ sector[i];
    return sum;
}

static u32 bootctrl_read_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void bootctrl_write_le32(u8 *p, u32 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
    p[2] = (u8)(value >> 16);
    p[3] = (u8)(value >> 24);
}

bool bootctrl_validate_versioned(const u8 *sector)
{
    if (!sector ||
        bootctrl_read_le32(sector + PIOS_BOOTCTRL_MAGIC_OFF) !=
            PIOS_BOOTCTRL_MAGIC)
        return false;
    u32 version = bootctrl_read_le32(sector + PIOS_BOOTCTRL_VERSION_OFF);
    if (version != PIOS_BOOTCTRL_VERSION_V1 &&
        version != PIOS_BOOTCTRL_VERSION)
        return false;
    u32 off = version == PIOS_BOOTCTRL_VERSION_V1 ?
              PIOS_BOOTCTRL_V1_CHECKSUM_OFF : PIOS_BOOTCTRL_V2_CHECKSUM_OFF;
    return bootctrl_read_le32(sector + off) ==
           bootctrl_checksum_versioned(sector, version);
}

bool bootctrl_migrate_v1_to_v2(u8 *sector)
{
    if (!sector ||
        bootctrl_read_le32(sector + PIOS_BOOTCTRL_VERSION_OFF) !=
            PIOS_BOOTCTRL_VERSION_V1 ||
        !bootctrl_validate_versioned(sector))
        return false;
    for (u32 i = PIOS_BOOTCTRL_OVERRIDE_MODE_OFF;
         i < PIOS_BOOTCTRL_V2_CHECKSUM_OFF; i++)
        sector[i] = 0;
    bootctrl_write_le32(sector + PIOS_BOOTCTRL_VERSION_OFF,
                        PIOS_BOOTCTRL_VERSION);
    bootctrl_write_le32(sector + PIOS_BOOTCTRL_V2_CHECKSUM_OFF,
                        bootctrl_checksum_versioned(sector,
                                                     PIOS_BOOTCTRL_VERSION));
    return true;
}

bool bootctrl_parse_package_id(const char *text, u64 *package_id_out)
{
    if (!text || !*text || !package_id_out)
        return false;
    u32 base = 10U;
    if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
        base = 16U;
        text += 2;
        if (!*text)
            return false;
    }

    u64 value = 0U;
    while (*text) {
        u32 digit;
        char c = *text++;
        if (c >= '0' && c <= '9')
            digit = (u32)(c - '0');
        else if (base == 16U && c >= 'a' && c <= 'f')
            digit = (u32)(c - 'a' + 10);
        else if (base == 16U && c >= 'A' && c <= 'F')
            digit = (u32)(c - 'A' + 10);
        else
            return false;
        if (digit >= base || value > (~(u64)0 - digit) / base)
            return false;
        value = value * base + digit;
    }
    *package_id_out = value;
    return true;
}

enum boot_precedence_choice
boot_precedence_choose(const struct boot_precedence_input *input)
{
    if (!input)
        return BOOT_PRECEDENCE_NONE;
    if (input->override_armed && input->override_valid &&
        input->override_consumed)
        return BOOT_PRECEDENCE_OVERRIDE_O;
    if (input->pending_bootable)
        return BOOT_PRECEDENCE_PENDING;
    if (input->active_bootable)
        return BOOT_PRECEDENCE_ACTIVE;
    if (input->fat_bootable)
        return BOOT_PRECEDENCE_FAT_RECOVERY;
    return BOOT_PRECEDENCE_NONE;
}
