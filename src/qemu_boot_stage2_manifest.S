/*
 * qemu_boot_stage2_manifest.S - embedded (non-packaged) stage2 manifest for
 * the QEMU-virt platform, analogous to stage2_manifest.S (Pi5). This is
 * NOT the same file as the vestigial qemu_stage2_manifest.S (which targets
 * the separate UEFI boot path, image_base_hint=0x40080000 but a different
 * entry/feature intent) -- this one is purpose-built for the stage0
 * raw-slot-jump path (see bootstrap.c), matching link_qemu_full.ld's
 * existing 0x40080000 load address exactly.
 *
 * Deliberately excluded from build_qemu_full.bat's plain direct-boot
 * wildcard compile (see the exclusion list there) so the existing,
 * regression-tested PIOS_QEMU_FULL.BIN stays byte-for-byte unaffected.
 * Only build_bootstrap_qemu.bat compiles this file in.
 */
.section .stage2_manifest, "a"
.balign 8
.global pios_stage2_manifest_start
.global pios_stage2_manifest_end

.equ PIOS_STAGE2_MANIFEST_MAGIC, 0x32534750
.equ PIOS_STAGE2_MANIFEST_VERSION, 1
.equ PIOS_STAGE2_PLATFORM_QEMU_VIRT, 2
.equ PIOS_STAGE2_FEAT_AARCH64, 0x0000000000000001
.equ PIOS_STAGE2_FEAT_GENERIC_TIMER, 0x0000000000000002
.equ PIOS_STAGE2_FEAT_GICV2, 0x0000000000000004
.equ PIOS_STAGE2_FEAT_PL011, 0x0000000000000008
.equ PIOS_STAGE2_FEAT_RAM_WALFS, 0x0000000000000400

pios_stage2_manifest_start:
    .word   PIOS_STAGE2_MANIFEST_MAGIC
    .hword  PIOS_STAGE2_MANIFEST_VERSION
    .hword  32
    .hword  1
    .hword  64
    .word   0
    .quad   0x0000000040080000
    .quad   0

    .word   PIOS_STAGE2_PLATFORM_QEMU_VIRT
    .word   0
    .quad   0
    .quad   PIOS_STAGE2_FEAT_AARCH64 | PIOS_STAGE2_FEAT_GENERIC_TIMER
    .quad   PIOS_STAGE2_FEAT_GICV2 | PIOS_STAGE2_FEAT_PL011 | \
            PIOS_STAGE2_FEAT_RAM_WALFS
    .asciz  "qemu-virt"
    .zero   20

pios_stage2_manifest_end:
