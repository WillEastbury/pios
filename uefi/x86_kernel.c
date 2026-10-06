/*
 * First PIOS-owned x86_64 kernel boundary.  This code runs only after UEFI
 * ExitBootServices and intentionally keeps interrupts disabled until APIC and
 * an exception/IRQ ownership model are installed.
 */
#include "x86_kernel.h"

#define X86_KERNEL_IDENTITY_BYTES (512ULL * 1024ULL * 1024ULL * 1024ULL)
#define X86_KERNEL_PD_COUNT       512U
#define X86_PTE_PRESENT_RW        0x003ULL
#define X86_PDE_2M                0x083ULL

struct x86_desc_ptr {
    x86_u16 limit;
    x86_u64 base;
} __attribute__((packed));

struct x86_idt_gate {
    x86_u16 offset_lo;
    x86_u16 selector;
    x86_u8 ist;
    x86_u8 type;
    x86_u16 offset_mid;
    x86_u32 offset_hi;
    x86_u32 reserved;
} __attribute__((packed));

static x86_u64 pml4[512] __attribute__((aligned(4096)));
static x86_u64 pdpt[512] __attribute__((aligned(4096)));
static x86_u64 pd[X86_KERNEL_PD_COUNT][512] __attribute__((aligned(4096)));
static x86_u8 stack[64 * 1024] __attribute__((aligned(16)));
static x86_u64 gdt[] __attribute__((aligned(16))) = {
    0,
    0x00AF9A000000FFFFULL,
    0x00AF92000000FFFFULL,
};
static struct x86_idt_gate idt[256] __attribute__((aligned(16)));
static volatile struct x86_kernel_boot_state boot_state;
x86_u64 x86_diag_framebuffer;
x86_u32 x86_diag_stride;

extern void x86_kernel_reload_segments(void);
extern void x86_kernel_halt_exception(void);
extern void x86_kernel_halt_double_fault(void);
extern void x86_kernel_halt_gp(void);
extern void x86_kernel_halt_page_fault(void);
extern void x86_kernel_jump(const struct x86_uefi_handoff *handoff,
                            x86_u64 cr3, x86_u64 stack_top);

static void marker(const struct x86_uefi_handoff *handoff, x86_u32 bgr)
{
    if (!handoff || handoff->framebuffer_base == 0U ||
        handoff->framebuffer_width < 64U || handoff->framebuffer_height < 64U ||
        handoff->framebuffer_stride < handoff->framebuffer_width ||
        handoff->framebuffer_bytes <
            (x86_u64)handoff->framebuffer_stride * handoff->framebuffer_height * 4U)
        return;
    volatile x86_u32 *pixels =
        (volatile x86_u32 *)(x86_usize)handoff->framebuffer_base;
    for (x86_u32 y = 0; y < 64U; y++)
        for (x86_u32 x = 0; x < 64U; x++)
            pixels[y * handoff->framebuffer_stride + x] = bgr;
    __asm__ volatile("wbinvd" ::: "memory");
}

static void zero(void *ptr, x86_usize bytes)
{
    volatile x86_u8 *p = (volatile x86_u8 *)ptr;
    while (bytes--)
        *p++ = 0;
}

static void idt_set(x86_u32 vector, x86_u64 address)
{
    struct x86_idt_gate *entry = &idt[vector];
    entry->offset_lo = (x86_u16)address;
    entry->selector = 0x08;
    entry->ist = 0;
    entry->type = 0x8E;
    entry->offset_mid = (x86_u16)(address >> 16);
    entry->offset_hi = (x86_u32)(address >> 32);
    entry->reserved = 0;
}

void x86_kernel_prepare(const struct x86_uefi_handoff *handoff)
{
    struct x86_desc_ptr gdtr = {
        .limit = (x86_u16)(sizeof(gdt) - 1U),
        .base = (x86_u64)(x86_usize)gdt,
    };
    struct x86_desc_ptr idtr = {
        .limit = (x86_u16)(sizeof(idt) - 1U),
        .base = (x86_u64)(x86_usize)idt,
    };
    marker(handoff, 0x00CC0000U); /* red: PIOS table preparation began */
    x86_diag_framebuffer = handoff ? handoff->framebuffer_base : 0U;
    x86_diag_stride = handoff ? handoff->framebuffer_stride : 0U;
    zero(pml4, sizeof(pml4));
    zero(pdpt, sizeof(pdpt));
    zero(pd, sizeof(pd));
    zero(idt, sizeof(idt));
    pml4[0] = (x86_u64)(x86_usize)pdpt | X86_PTE_PRESENT_RW;
    for (x86_u32 upper = 0; upper < X86_KERNEL_PD_COUNT; upper++) {
        pdpt[upper] = (x86_u64)(x86_usize)pd[upper] | X86_PTE_PRESENT_RW;
        for (x86_u32 lower = 0; lower < 512U; lower++) {
            x86_u64 physical = ((x86_u64)upper << 30) | ((x86_u64)lower << 21);
            pd[upper][lower] = physical | X86_PDE_2M;
        }
    }
    for (x86_u32 vector = 0; vector < 256U; vector++)
        idt_set(vector, (x86_u64)(x86_usize)x86_kernel_halt_exception);
    idt_set(8U, (x86_u64)(x86_usize)x86_kernel_halt_double_fault);
    idt_set(13U, (x86_u64)(x86_usize)x86_kernel_halt_gp);
    idt_set(14U, (x86_u64)(x86_usize)x86_kernel_halt_page_fault);
    __asm__ volatile("lgdt %0" :: "m"(gdtr) : "memory");
    x86_kernel_reload_segments();
    __asm__ volatile("lidt %0" :: "m"(idtr) : "memory");
    marker(handoff, 0x000000CCU); /* blue: GDT/IDT now PIOS-owned */
    boot_state = (struct x86_kernel_boot_state){
        .magic = X86_KERNEL_BOOT_MAGIC,
        .cr3 = (x86_u64)(x86_usize)pml4,
        .stack_top = (x86_u64)(x86_usize)(stack + sizeof(stack)),
        .mapped_bytes = X86_KERNEL_IDENTITY_BYTES,
        .handoff = (x86_u64)(x86_usize)handoff,
        .idt_installed = 1,
        .gdt_installed = 1,
        .interrupts_enabled = 0,
        .online = 0,
    };
}

void x86_kernel_main(const struct x86_uefi_handoff *handoff)
{
    if (!handoff || boot_state.magic != X86_KERNEL_BOOT_MAGIC ||
        boot_state.handoff != (x86_u64)(x86_usize)handoff)
        for (;;) __asm__ volatile("cli; hlt");
    marker(handoff, 0x0000CC00U); /* green: PIOS code runs after CR3 switch */
    boot_state.online = 0;
    /* The next milestone owns APIC/VMBus/netvsc and then sets online. Do not
     * enable interrupts or call firmware after ExitBootServices. */
    for (;;) __asm__ volatile("cli; hlt");
}

void x86_kernel_after_cr3(const struct x86_uefi_handoff *handoff)
{
    marker(handoff, 0x0000CCCCU); /* cyan: PIOS C code ran after own CR3 */
}

void x86_kernel_enter(const struct x86_uefi_handoff *handoff)
{
    if (!handoff || boot_state.magic != X86_KERNEL_BOOT_MAGIC)
        for (;;) __asm__ volatile("cli; hlt");
    x86_kernel_jump(handoff, boot_state.cr3, boot_state.stack_top);
}

void x86_kernel_boot_snapshot(struct x86_kernel_boot_state *out)
{
    if (out)
        *out = boot_state;
}
