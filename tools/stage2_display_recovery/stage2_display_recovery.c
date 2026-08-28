#include "types.h"
#include "fb.h"
#include "pcie.h"
#include "rp1.h"
#include "uart.h"

u64 l1_table[512] ALIGNED(4096);
u64 l2_table_boot[512] ALIGNED(4096);
u64 shared_ttbr0;
u64 shared_mair;
u64 shared_tcr;
u64 el2_boot_el_state;

static NORETURN void display_halt(void)
{
    for (;;)
        wfi();
}

void kernel_fb_early(void)
{
    (void)fb_set_arm_clock_max();
    if (!fb_init(1920U, 1080U) && !fb_init(1280U, 720U) &&
        !fb_init(1024U, 768U))
        return;

    fb_clear(0x00000080U);
    fb_set_color(0x00FFFFFFU, 0x00000080U);
    fb_set_cursor(4U, 4U);
    fb_puts("PIOS STAGE2 DISPLAY RECOVERY\n");
    fb_set_cursor(4U, 6U);
    fb_puts("EARLY MAILBOX FRAMEBUFFER ONLINE\n");
    fb_present();
}

NORETURN void kernel_main(void)
{
    bool pcie_ok = pcie_init();
    bool rp1_ok = pcie_ok && rp1_init();
    if (rp1_ok)
        uart_init();
    uart_puts("\n[display-recovery] RP1 UART ready\n");

    (void)fb_set_arm_clock_max();
    bool ready = fb_init(1920U, 1080U) || fb_init(1280U, 720U) ||
                 fb_init(1024U, 768U);
    uart_puts(ready ? "[display-recovery] DISPLAY OK\n" :
                      "[display-recovery] DISPLAY FAIL\n");
    struct fb_mailbox_diag diag;
    fb_mailbox_diag(&diag);
    uart_puts("[display-recovery] mbox status=");
    uart_hex(diag.status);
    uart_puts(" msg=");
    uart_hex(diag.message);
    uart_puts(" rsp=");
    uart_hex(diag.response);
    uart_puts(" alloc=");
    uart_hex(diag.allocation_addr);
    uart_puts("/");
    uart_hex(diag.allocation_size);
    uart_puts(" pitch=");
    uart_hex(diag.pitch);
    uart_puts("\n");
    if (ready) {
        fb_clear(0x00000080U);
        fb_set_color(0x00FFFFFFU, 0x00000080U);
        fb_set_cursor(4U, 4U);
        fb_puts("PIOS STAGE2 DISPLAY RECOVERY\n");
        fb_set_cursor(4U, 6U);
        fb_puts("MAILBOX FRAMEBUFFER ONLINE\n");
        fb_present();
    }
    display_halt();
}

void kernel_el2_crash(u64 esr, u64 elr, u64 far, u64 spsr)
{
    (void)esr;
    (void)elr;
    (void)far;
    (void)spsr;
    display_halt();
}

void el2_hvc_trap(u64 fid, u64 x1, u64 x2, u64 x3, u64 x4)
{
    (void)fid;
    (void)x1;
    (void)x2;
    (void)x3;
    (void)x4;
}

void el2_sync_fault_trap(u64 esr, u64 elr, u64 far, u64 spsr)
{
    (void)esr;
    (void)elr;
    (void)far;
    (void)spsr;
    display_halt();
}

void irq_dispatch(void *frame)
{
    (void)frame;
}

void sync_exception(void *frame, u64 esr, u64 far)
{
    (void)frame;
    (void)esr;
    (void)far;
    display_halt();
}

void serror_exception(void *frame)
{
    (void)frame;
    display_halt();
}

void irq_register(u32 intid, void *handler)
{
    (void)intid;
    (void)handler;
}

void gic_enable_irq(u32 intid)
{
    (void)intid;
}

void gic_set_priority(u32 intid, u8 priority)
{
    (void)intid;
    (void)priority;
}

NORETURN void core1_main(void)
{
    display_halt();
}

NORETURN void core2_main(void)
{
    display_halt();
}

NORETURN void core3_main(void)
{
    display_halt();
}
