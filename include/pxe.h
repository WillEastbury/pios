/*
 * pxe.h - PXE executable format and module loader
 *
 * Defines the PXE binary format for loadable executables and kernel
 * modules, plus a minimal ELF64 loader for standard AArch64 binaries.
 */

#pragma once
#include "types.h"

#define PXE_MAGIC       0x50584521  /* 'PXE!' */
#define PXE_VERSION     1

/* Binary types */
#define PXE_EXEC        1
#define PXE_MODULE      2
#define PXE_LIBRARY     3

/* Header flags */
#define PXE_RELOCATABLE 0x01
#define PXE_NEEDS_NEON  0x02

/* Relocation types */
#define RELOC_ABS64     1
#define RELOC_REL26     2   /* no-op when code is contiguous */
#define RELOC_ADRP      3   /* reserved for future use */

struct pxe_header {
    u32 magic;
    u16 version;
    u16 type;
    u32 flags;
    u32 entry_offset;
    u32 code_offset;
    u32 code_size;
    u32 data_offset;
    u32 data_size;
    u32 bss_size;
    u32 reloc_offset;
    u32 reloc_count;
    u32 import_offset;
    u32 import_count;
    u32 stack_size;
    u32 min_memory;
    u32 crc32;
} PACKED;

struct pxe_reloc {
    u32 offset;
    u16 type;
    u16 section;    /* 0 = code, 1 = data */
} PACKED;

struct pxe_import {
    u32 name_hash;      /* CRC32C of symbol name */
    u32 patch_offset;   /* offset into code to patch */
} PACKED;

/* Module hook types */
#define MODULE_NET_FILTER   1
#define MODULE_FS_HOOK      2
#define MODULE_TIMER        3
#define MODULE_MAX_HOOKS    8

struct pxe_module_info {
    u32 target_core;
    u32 hook_type;
    u32 init_offset;
    u32 cleanup_offset;
} PACKED;

/* Load a PXE executable into memory at base. Returns entry point or 0. */
u64 pxe_load(const u8 *file, u32 file_size, u8 *base, u32 slot_size,
             void *syscall_table);

/* Load a minimal ELF64 binary. Returns entry point or 0. */
u64 elf64_load(const u8 *file, u32 file_size, u8 *base, u32 slot_size);

/* Load and install a kernel module. Returns true on success. */
bool module_load(const u8 *file, u32 file_size);

/* Call all registered hooks of a given type. */
void module_call_hooks(u32 hook_type, void *arg);

/* Init module system */
void module_init(void);
