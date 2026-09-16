#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "boot_precedence.h"

static void put32(u8 *p, u32 v)
{
    p[0] = (u8)v; p[1] = (u8)(v >> 8);
    p[2] = (u8)(v >> 16); p[3] = (u8)(v >> 24);
}

static u32 get32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void seal(u8 *p, u32 version)
{
    u32 off = version == PIOS_BOOTCTRL_VERSION_V1 ?
              PIOS_BOOTCTRL_V1_CHECKSUM_OFF : PIOS_BOOTCTRL_V2_CHECKSUM_OFF;
    put32(p + off, bootctrl_checksum_versioned(p, version));
}

static void test_v1_migration(void)
{
    u8 sector[512];
    memset(sector, 0xA5, sizeof(sector));
    put32(sector + PIOS_BOOTCTRL_MAGIC_OFF, PIOS_BOOTCTRL_MAGIC);
    put32(sector + PIOS_BOOTCTRL_VERSION_OFF, PIOS_BOOTCTRL_VERSION_V1);
    put32(sector + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF, PIOS_BOOTCTRL_SLOT_B);
    put32(sector + PIOS_BOOTCTRL_PENDING_SLOT_OFF, PIOS_BOOTCTRL_SLOT_A);
    put32(sector + PIOS_BOOTCTRL_TRIES_LEFT_OFF, 1);
    put32(sector + PIOS_BOOTCTRL_LAST_BOOT_OFF, PIOS_BOOTCTRL_SLOT_B);
    put32(sector + PIOS_BOOTCTRL_GOOD_MASK_OFF, 3);
    put32(sector + PIOS_BOOTCTRL_GENERATION_OFF, 44);
    seal(sector, PIOS_BOOTCTRL_VERSION_V1);
    assert(bootctrl_validate_versioned(sector));
    assert(bootctrl_migrate_v1_to_v2(sector));
    assert(bootctrl_validate_versioned(sector));
    assert(get32(sector + PIOS_BOOTCTRL_VERSION_OFF) == PIOS_BOOTCTRL_VERSION);
    assert(get32(sector + PIOS_BOOTCTRL_OVERRIDE_MODE_OFF) ==
           PIOS_BOOTCTRL_OVERRIDE_NONE);
    assert(get32(sector + PIOS_BOOTCTRL_OVERRIDE_TRIES_OFF) == 0);
    assert(get32(sector + PIOS_BOOTCTRL_OVERRIDE_PACKAGE_ID_OFF) == 0);
    assert(get32(sector + PIOS_BOOTCTRL_ACTIVE_SLOT_OFF) ==
           PIOS_BOOTCTRL_SLOT_B);
    sector[PIOS_BOOTCTRL_V2_CHECKSUM_OFF] ^= 1;
    assert(!bootctrl_validate_versioned(sector));
}

static void test_precedence(void)
{
    struct boot_precedence_input in = {0};
    for (u32 mask = 0; mask < 64; mask++) {
        in.override_armed = (mask & 1U) != 0;
        in.override_valid = (mask & 2U) != 0;
        in.override_consumed = (mask & 4U) != 0;
        in.pending_bootable = (mask & 8U) != 0;
        in.active_bootable = (mask & 16U) != 0;
        in.fat_bootable = (mask & 32U) != 0;
        enum boot_precedence_choice got = boot_precedence_choose(&in);
        enum boot_precedence_choice want =
            (in.override_armed && in.override_valid && in.override_consumed) ?
                BOOT_PRECEDENCE_OVERRIDE_O :
            in.pending_bootable ? BOOT_PRECEDENCE_PENDING :
            in.active_bootable ? BOOT_PRECEDENCE_ACTIVE :
            in.fat_bootable ? BOOT_PRECEDENCE_FAT_RECOVERY :
            BOOT_PRECEDENCE_NONE;
        assert(got == want);
    }

    /* The stage0 write is the atomic consume point. A failed write is not an
     * O boot; after a successful O attempt, the next boot sees O unarmed. */
    in = (struct boot_precedence_input) {
        .override_armed = true, .override_valid = true,
        .override_consumed = true, .pending_bootable = true,
        .active_bootable = true, .fat_bootable = true,
    };
    assert(boot_precedence_choose(&in) == BOOT_PRECEDENCE_OVERRIDE_O);
    in.override_consumed = false;
    assert(boot_precedence_choose(&in) == BOOT_PRECEDENCE_PENDING);
    in.override_consumed = true;
    in.override_valid = false; /* missing, invalid, or ID-mismatched FAT */
    assert(boot_precedence_choose(&in) == BOOT_PRECEDENCE_PENDING);
    in.override_armed = false; /* next boot after O success/failure */
    assert(boot_precedence_choose(&in) == BOOT_PRECEDENCE_PENDING);
}

static void test_package_id_parser(void)
{
    u64 value = 0;
    assert(bootctrl_parse_package_id("18446744073709551615", &value));
    assert(value == ~(u64)0);
    assert(bootctrl_parse_package_id("0xffffffffffffffff", &value));
    assert(value == ~(u64)0);
    assert(bootctrl_parse_package_id("0X0123456789ABCDEF", &value));
    assert(value == 0x0123456789ABCDEFULL);
    assert(!bootctrl_parse_package_id("18446744073709551616", &value));
    assert(!bootctrl_parse_package_id("0x10000000000000000", &value));
    assert(!bootctrl_parse_package_id("", &value));
    assert(!bootctrl_parse_package_id("0x", &value));
    assert(!bootctrl_parse_package_id("123z", &value));
    assert(!bootctrl_parse_package_id("1", NULL));
}

int main(void)
{
    test_v1_migration();
    test_precedence();
    test_package_id_parser();
    puts("boot precedence: migration, checksum, decisions, package IDs PASS");
    return 0;
}
