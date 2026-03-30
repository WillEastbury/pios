.section ".text.boot"
.global _start

/*
 * Canary v11: proof-of-life via PM watchdog reboot.
 * NO UART writes (those fault with PCIe BAR gone).
 * Triggers a hardware reboot via BCM2712 PM registers (on-SoC, no PCIe).
 * If the Pi reboots in a loop (firmware messages repeat every ~2s),
 * the kernel IS executing.
 *
 * PM registers (BCM2712 SoC):
 *   PM_BASE  = 0x107D200000
 *   PM_RSTC  = PM_BASE + 0x1C
 *   PM_WDOG  = PM_BASE + 0x24
 *   Password = 0x5A000000
 */
_start:
    /* Small delay so firmware UART output finishes */
    mov     x3, #0x200000
1:  sub     x3, x3, #1
    cbnz    x3, 1b

    /* PM_BASE = 0x107D200000 */
    ldr     x9, =0x107D200000

    /* PM_WDOG = password | 10 ticks (short timeout) */
    ldr     w0, =0x5A00000A
    str     w0, [x9, #0x24]

    /* PM_RSTC = password | 0x20 (full reset) */
    ldr     w0, =0x5A000020
    str     w0, [x9, #0x1C]

    /* Wait for reboot */
2:  wfe
    b       2b
