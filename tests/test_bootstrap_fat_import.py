#!/usr/bin/env python3
"""Static regression for ADR-061 O/raw-first/FAT-recovery stage0 ordering."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
bootstrap = (ROOT / "src" / "bootstrap.c").read_text(encoding="utf-8")
bootstrap_start = (ROOT / "src" / "bootstrap_start.S").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


main = function_body(bootstrap, "NORETURN void bootstrap_main(void)")
assert "if (!partition_valid) {" in main
assert "root_lba = BOOT_FALLBACK_LBA;" in main
override = main.index("stage0_try_override_o(root_lba, &fat_package)")
shared = main.index("stage0_load_shared_from_fat(&fat_package)")
raw = main.index("stage0_select_raw_slot(root_lba, &slot_lba, &sel,")
recovery = main.index("stage0_apply_fat_update(root_lba, &fat_package)")
assert override < shared < raw < recovery
assert "if (!stage0_select_raw_slot(root_lba, &slot_lba, &sel," in main
assert "stage0_load_shared_from_fat(&fat_package)" in main
assert "stage0_apply_fat_update(root_lba, &fat_package)" in main
assert "memset(&fat_package, 0, sizeof(fat_package));" in main

update = function_body(bootstrap, "static bool stage0_apply_fat_update(u32 root_lba,")
assert "slot_package_matches(target_lba, payload, payload_len, true)" in update
assert "FAT package already cached in slot A" in update
assert "stage0_load_fat_package(package)" in update

override_body = function_body(bootstrap, "static bool stage0_try_override_o(")
assert "package->package_id == wanted" in override_body
assert "package->selected" in override_body
assert "PIOS_BOOTCTRL_SLOT_O" in override_body
assert "PIOS_BOOTCTRL_OVERRIDE_MODE_OFF" in override_body
assert "PIOS_BOOTCTRL_OVERRIDE_TRIES_DEFAULT" in override_body
assert "if (!bootctrl_write(root_lba, bootctl))" in override_body
assert override_body.index("bootctrl_write(root_lba, bootctl)") < override_body.index(
    "stage0_jump_staged(&package->selection)")
assert "stage0_load_shared_from_fat(package)" in override_body
for ab_field in ("ACTIVE_SLOT_OFF", "PENDING_SLOT_OFF", "TRIES_LEFT_OFF",
                 "GOOD_MASK_OFF"):
    assert ab_field not in override_body

raw_load = function_body(bootstrap, "static bool stage0_load_raw_slot(")
assert "slot_header_bootable" in raw_load
assert "sd_read_block" in raw_load
assert "select_stage2_image" in raw_load

raw_select = function_body(bootstrap, "static bool stage0_select_raw_slot(")
assert "stage0_load_raw_slot" in raw_select
assert "pending <= PIOS_BOOTCTRL_SLOT_B && tries > 0U" in raw_select
assert raw_select.index("stage0_load_raw_slot(pending_lba") < raw_select.index(
    "PIOS_BOOTCTRL_TRIES_LEFT_OFF, tries - 1U")
assert raw_select.index("stage0_load_raw_slot(active_lba") < raw_select.index(
    "PIOS_BOOTCTRL_LAST_BOOT_OFF, active")

stage0_read = function_body(bootstrap, "static bool bootctrl_read(u32 root_lba, u8 *p)")
assert "bootctrl_migrate_v1(p)" in stage0_read
assert "bootctrl_write(root_lba, p)" not in stage0_read
assert "|| !bootctrl_write(root_lba, p)" not in stage0_read

kernel = (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")
assert "pios_bootctrl_arm_o(u64 package_id)" in kernel
assert "pios_bootctrl_clear_o(void)" in kernel
assert 'bootctrl arm-o <package-id> confirm' in kernel
assert 'bootctrl clear-o' in kernel
http_u64 = function_body(kernel, "static bool http_parse_u64(const char *s, u64 *out)")
ui_u64 = function_body(kernel, "static bool ui_parse_u64(const char *s, u64 *out)\n{")
assert "if (v > (~(u64)0 - d) / base) return false;" in http_u64
assert "if (v > (~(u64)0 - d) / base) return false;" in ui_u64
assert kernel.count("bootctrl_parse_package_id(") >= 2
assert "if (booted == PIOS_BOOTCTRL_SLOT_O)\n        return;" in kernel
assert "PIOS_BOOTCTRL_V1_CHECKSUM_OFF" in kernel
assert "PIOS_BOOTCTRL_V2_CHECKSUM_OFF" in kernel
kernel_read = function_body(kernel, "static bool pios_bootctrl_read(u8 *out)")
assert "pios_bootctrl_migrate_v1(out)" in kernel_read
assert "sd_write_block(lba, out)" not in kernel_read
assert "|| !sd_write_block(lba, out)" not in kernel_read
arm = function_body(kernel, "static bool pios_bootctrl_arm_o(u64 package_id)\n{")
clear = function_body(kernel, "static bool pios_bootctrl_clear_o(void)\n{")
for body in (arm, clear):
    for ab_field in ("ACTIVE_SLOT_OFF", "PENDING_SLOT_OFF", "TRIES_LEFT_OFF",
                     "GOOD_MASK_OFF", "LAST_BOOT_OFF"):
        assert ab_field not in body
    assert "OVERRIDE_MODE_OFF" in body
    assert "OVERRIDE_TRIES_OFF" in body
    assert "OVERRIDE_PACKAGE_ID_OFF" in body
    assert "GENERATION_OFF" in body
status = function_body(
    kernel, "static void http_append_bootctrl_status(char *out, u32 *len, u32 max)\n{")
assert '" o="' in status and '" o_tries="' in status and '" o_id="' in status

assert "bl      kernel_fb_early" not in bootstrap_start
assert bootstrap_start.index("bl      stage0_firmware_hello") < bootstrap_start.index("mrs     x0, CurrentEL")
assert bootstrap_start.index("mrs     x0, CurrentEL") < bootstrap_start.index("bl      board_detect_init")
assert bootstrap_start.index("bl      board_detect_init") < bootstrap_start.index(".Lb_mmu_enable:")
assert "if (!stage0_fb_ready)\n        kernel_fb_early();" in main

print("Stage0 uses O -> validated raw A/B -> FAT recovery precedence")
print("Stage0 writes the firmware-entry hello before changing EL")
print("Stage0 attempts framebuffer once after EL1/MMU setup and records the result")
