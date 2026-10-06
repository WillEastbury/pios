#pragma once

typedef unsigned char x86_u8;
typedef unsigned short x86_u16;
typedef unsigned int x86_u32;
typedef unsigned long long x86_u64;
typedef unsigned long long x86_usize;

struct x86_uefi_handoff {
    x86_u64 memory_map;
    x86_u64 memory_map_bytes;
    x86_u64 memory_map_key;
    x86_u64 descriptor_bytes;
    x86_u64 descriptor_version;
    x86_u64 system_table;
    x86_u64 image_handle;
    x86_u64 framebuffer_base;
    x86_u64 framebuffer_bytes;
    x86_u32 framebuffer_width;
    x86_u32 framebuffer_height;
    x86_u32 framebuffer_stride;
    x86_u32 framebuffer_format;
} __attribute__((packed));

struct x86_kernel_boot_state {
    x86_u64 magic;
    x86_u64 cr3;
    x86_u64 stack_top;
    x86_u64 mapped_bytes;
    x86_u64 handoff;
    x86_u32 idt_installed;
    x86_u32 gdt_installed;
    x86_u32 interrupts_enabled;
    x86_u32 online;
} __attribute__((packed));

#define X86_KERNEL_BOOT_MAGIC 0x50494F535836344BULL /* PIOSX64K */

/* The common USB layout does not make a universal binary: native, Hyper-V and
 * QEMU x86 images are compiled and packaged separately. */
enum x86_uefi_environment {
    X86_UEFI_ENV_NATIVE = 1U,
    X86_UEFI_ENV_HYPERV = 2U,
    X86_UEFI_ENV_QEMU = 3U,
};

void x86_kernel_prepare(const struct x86_uefi_handoff *handoff);
void x86_kernel_enter(const struct x86_uefi_handoff *handoff);
void x86_kernel_after_cr3(const struct x86_uefi_handoff *handoff);
void x86_kernel_boot_snapshot(struct x86_kernel_boot_state *out);
void x86_kernel_jump(const struct x86_uefi_handoff *handoff,
                     x86_u64 cr3, x86_u64 stack_top);
