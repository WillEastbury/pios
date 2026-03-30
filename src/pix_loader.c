/*
 * pix_loader.c - PIX executable loader with relocations + minimal ELF64 loader
 */

#include "pix.h"
#include "uart.h"
#include "simd.h"
#include "mmu.h"

/* ---- Import resolution table ---- */

typedef void (*func_ptr)(void);

struct api_entry {
    u32       name_hash;
    func_ptr  addr;
};

/* Forward declarations of kernel functions modules may import.
 * Actual addresses resolved at link time. */
extern void uart_puts(const char *s);
extern u32  walfs_read(u64 inode_id, u64 offset, void *buf, u32 len);
extern bool walfs_write(u64 inode_id, u64 offset, const void *data, u32 len);
extern void *memset(void *dst, int c, usize n);
extern void *memcpy(void *dst, const void *src, usize n);
extern u32  hw_crc32c(const void *data, u32 len);

/* Pre-computed CRC32C hashes of kernel API symbol name strings.
 * Generated offline via: hw_crc32c("uart_puts", 9) etc. */
#define HASH_UART_PUTS      0x5B1F29D3
#define HASH_WALFS_READ     0xA43E7C10
#define HASH_WALFS_WRITE    0x6D92B4A8
#define HASH_MEMSET         0x1C3D8E77
#define HASH_MEMCPY         0x2F5A4B19
#define HASH_HW_CRC32C      0x84D1F206

static const struct api_entry kernel_api_table_builtin[] = {
    { HASH_UART_PUTS,    (func_ptr)uart_puts    },
    { HASH_WALFS_READ,   (func_ptr)walfs_read   },
    { HASH_WALFS_WRITE,  (func_ptr)walfs_write  },
    { HASH_MEMSET,       (func_ptr)memset        },
    { HASH_MEMCPY,       (func_ptr)memcpy        },
    { HASH_HW_CRC32C,    (func_ptr)hw_crc32c    },
    { 0, NULL }
};

/* ---- I-cache flush ---- */

static void flush_icache(u64 start, u64 size)
{
    u64 addr;
    u64 end = start + size;

    /* Clean data cache and invalidate instruction cache line by line */
    for (addr = start & ~63UL; addr < end; addr += 64) {
        __asm__ volatile("dc civac, %0" :: "r"(addr) : "memory");
        __asm__ volatile("ic ivau, %0"  :: "r"(addr) : "memory");
    }
    __asm__ volatile("dsb ish" ::: "memory");
    __asm__ volatile("isb"     ::: "memory");
}

/* ---- CRC32 verification ---- */

static u32 compute_header_crc(const struct pix_header *h)
{
    /* CRC covers everything except the crc32 field itself (last 4 bytes) */
    u32 header_len = (u32)((u64)&h->crc32 - (u64)h);
    return hw_crc32c((const void *)h, header_len);
}

/* ---- Relocation ---- */

static bool apply_relocations(u8 *base, const u8 *file,
                              const struct pix_header *hdr, u64 load_base)
{
    const struct pix_reloc *relocs =
        (const struct pix_reloc *)(file + hdr->reloc_offset);

    u32 image_span = hdr->code_size + hdr->data_size;
    if (image_span < hdr->code_size)
        return false;

    for (u32 i = 0; i < hdr->reloc_count; i++) {
        const struct pix_reloc *r = &relocs[i];
        u32 off = r->offset;

        /* Bounds check */
        if (off > image_span - 8U)
            return false;

        switch (r->type) {
        case RELOC_ABS64: {
            u64 *slot = (u64 *)(base + off);
            *slot += load_base;
            break;
        }
        case RELOC_REL26:
            /* No-op when code + data are loaded contiguously */
            break;
        case RELOC_ADRP:
            /* Reserved for v2 — skip */
            break;
        default:
            return false;
        }
    }
    return true;
}

/* ---- Import resolution ---- */

static func_ptr resolve_import(u32 name_hash, void *user_table)
{
    /* Check user-provided table first (if any) */
    if (user_table) {
        const struct api_entry *e = (const struct api_entry *)user_table;
        for (; e->addr; e++) {
            if (e->name_hash == name_hash)
                return e->addr;
        }
    }
    /* Fall back to built-in table */
    const struct api_entry *e = kernel_api_table_builtin;
    for (; e->addr; e++) {
        if (e->name_hash == name_hash)
            return e->addr;
    }
    return NULL;
}

static bool resolve_imports(u8 *base, const u8 *file,
                            const struct pix_header *hdr, void *kernel_api_tbl)
{
    const struct pix_import *imports =
        (const struct pix_import *)(file + hdr->import_offset);
    u32 image_span = hdr->code_size + hdr->data_size;
    if (image_span < hdr->code_size)
        return false;

    for (u32 i = 0; i < hdr->import_count; i++) {
        func_ptr fn = resolve_import(imports[i].name_hash, kernel_api_tbl);
        if (!fn) {
            uart_puts("pix: unresolved import\n");
            return false;
        }

        u32 off = imports[i].patch_offset;
        if (off > image_span - 8U)
            return false;

        /* Patch with absolute function pointer (64-bit) */
        u64 *slot = (u64 *)(base + off);
        *slot = (u64)fn;
    }
    return true;
}

/* ---- PIX loader ---- */

u64 pix_load(const u8 *file, u32 file_size, u8 *base, u32 slot_size,
             void *kernel_api_table)
{
    if (!file || !base || slot_size == 0)
        return 0;
    if (file_size < sizeof(struct pix_header))
        return 0;

    const struct pix_header *hdr = (const struct pix_header *)file;

    /* Verify magic and version */
    if (hdr->magic != PIX_MAGIC || hdr->version != PIX_VERSION)
        return 0;

    /* Verify CRC */
    if (hdr->crc32 != compute_header_crc(hdr)) {
        uart_puts("pix: bad crc\n");
        return 0;
    }

    /* Check memory requirements (overflow-safe) */
    if (hdr->code_size > slot_size ||
        hdr->data_size > slot_size - hdr->code_size ||
        hdr->bss_size > slot_size - hdr->code_size - hdr->data_size ||
        hdr->min_memory > slot_size)
        return 0;

    /* Validate offsets within file */
    if (hdr->code_offset > file_size || hdr->code_size > file_size - hdr->code_offset)
        return 0;
    if (hdr->data_size &&
        (hdr->data_offset > file_size || hdr->data_size > file_size - hdr->data_offset))
        return 0;
    if (hdr->reloc_count) {
        u64 reloc_bytes = (u64)hdr->reloc_count * sizeof(struct pix_reloc);
        if (hdr->reloc_offset > file_size || reloc_bytes > (u64)file_size - hdr->reloc_offset)
            return 0;
    }
    if (hdr->import_count) {
        u64 import_bytes = (u64)hdr->import_count * sizeof(struct pix_import);
        if (hdr->import_offset > file_size || import_bytes > (u64)file_size - hdr->import_offset)
            return 0;
    }

    u32 image_span = hdr->code_size + hdr->data_size;
    if (image_span < hdr->code_size || image_span > slot_size)
        return 0;
    if (hdr->bss_size > slot_size - image_span)
        return 0;

    u64 load_base = (u64)base;

    /* Copy code section */
    simd_memcpy(base, file + hdr->code_offset, hdr->code_size);

    /* Copy data section (right after code) */
    if (hdr->data_size)
        simd_memcpy(base + hdr->code_size, file + hdr->data_offset, hdr->data_size);

    /* Zero BSS */
    if (hdr->bss_size)
        simd_zero(base + image_span, hdr->bss_size);

    /* Apply relocations */
    if (hdr->reloc_count) {
        if (!(hdr->flags & PIX_RELOCATABLE))
            return 0;
        if (!apply_relocations(base, file, hdr, load_base))
            return 0;
    }

    /* Resolve imports */
    if (hdr->import_count) {
        if (!resolve_imports(base, file, hdr, kernel_api_table))
            return 0;
    }

    /* Flush I-cache over code region */
    flush_icache(load_base, hdr->code_size);

    /* Entry point is offset into code */
    if (hdr->entry_offset >= hdr->code_size)
        return 0;

    return load_base + hdr->entry_offset;
}

/* ---- Minimal ELF64 loader ---- */

/* ELF64 structures (minimal, inline) */
#define EI_NIDENT       16
#define PT_LOAD         1
#define EM_AARCH64      0xB7
#define ELFCLASS64      2
#define ELFDATA2LSB     1

struct elf64_ehdr {
    u8  e_ident[EI_NIDENT];
    u16 e_type;
    u16 e_machine;
    u32 e_version;
    u64 e_entry;
    u64 e_phoff;
    u64 e_shoff;
    u32 e_flags;
    u16 e_ehsize;
    u16 e_phentsize;
    u16 e_phnum;
    u16 e_shentsize;
    u16 e_shnum;
    u16 e_shstrndx;
} PACKED;

struct elf64_phdr {
    u32 p_type;
    u32 p_flags;
    u64 p_offset;
    u64 p_vaddr;
    u64 p_paddr;
    u64 p_filesz;
    u64 p_memsz;
    u64 p_align;
} PACKED;

u64 elf64_load(const u8 *file, u32 file_size, u8 *base, u32 slot_size)
{
    if (!file || !base || slot_size == 0)
        return 0;
    if (file_size < sizeof(struct elf64_ehdr))
        return 0;

    const struct elf64_ehdr *ehdr = (const struct elf64_ehdr *)file;

    /* Verify ELF magic */
    if (ehdr->e_ident[0] != 0x7F || ehdr->e_ident[1] != 'E' ||
        ehdr->e_ident[2] != 'L'  || ehdr->e_ident[3] != 'F')
        return 0;

    /* Must be 64-bit, little-endian, AArch64 */
    if (ehdr->e_ident[4] != ELFCLASS64 || ehdr->e_ident[5] != ELFDATA2LSB)
        return 0;
    if (ehdr->e_machine != EM_AARCH64)
        return 0;

    if (ehdr->e_phoff == 0 || ehdr->e_phnum == 0)
        return 0;
    if (ehdr->e_phentsize < sizeof(struct elf64_phdr))
        return 0;

    /* Validate program header table is within file */
    u64 ph_bytes = (u64)ehdr->e_phnum * ehdr->e_phentsize;
    if (ehdr->e_phoff > file_size || ph_bytes > (u64)file_size - ehdr->e_phoff)
        return 0;

    /* Find lowest vaddr to compute base offset */
    u64 vaddr_min = ~0UL;
    const u8 *ph_base = file + ehdr->e_phoff;
    for (u16 i = 0; i < ehdr->e_phnum; i++) {
        const struct elf64_phdr *ph =
            (const struct elf64_phdr *)(ph_base + (u64)i * ehdr->e_phentsize);
        if (ph->p_type == PT_LOAD && ph->p_vaddr < vaddr_min)
            vaddr_min = ph->p_vaddr;
    }
    if (vaddr_min == ~0UL)
        return 0;

    u64 load_base = (u64)base;
    u64 code_end = 0;

    /* Load each PT_LOAD segment */
    for (u16 i = 0; i < ehdr->e_phnum; i++) {
        const struct elf64_phdr *ph =
            (const struct elf64_phdr *)(ph_base + (u64)i * ehdr->e_phentsize);
        if (ph->p_type != PT_LOAD)
            continue;

        u64 dest_off = ph->p_vaddr - vaddr_min;
        if (dest_off > (u64)slot_size || ph->p_memsz > (u64)slot_size - dest_off)
            return 0;
        if (ph->p_filesz > ph->p_memsz)
            return 0;
        if (ph->p_offset > file_size || ph->p_filesz > file_size - ph->p_offset)
            return 0;

        u8 *dest = base + dest_off;
        simd_memcpy(dest, file + ph->p_offset, (usize)ph->p_filesz);

        /* Zero remainder (BSS portion) */
        if (ph->p_memsz > ph->p_filesz)
            simd_zero(dest + ph->p_filesz, (usize)(ph->p_memsz - ph->p_filesz));

        u64 seg_end = dest_off + ph->p_memsz;
        if (seg_end > code_end)
            code_end = seg_end;
    }

    /* Flush I-cache over entire loaded region */
    flush_icache(load_base, code_end);

    /* Return rebased entry point */
    if (ehdr->e_entry < vaddr_min || ehdr->e_entry - vaddr_min >= code_end)
        return 0;
    return load_base + (ehdr->e_entry - vaddr_min);
}
