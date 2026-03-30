.section ".text.boot"
.global _start

/*
 * Canary v10: absolute minimum — first instruction writes UART.
 * No core check, no branching. Just write 'X' and hang.
 *
 * Try both:
 *   0x1F00030000 — RP1 UART (GPIO14/15, if firmware PCIe ATU survives)
 *   0x107D001000 — BCM2712 debug UART (3-pin JST header, not GPIO!)
 *
 * If you see nothing on GPIO serial, the PCIe BAR is gone after handoff.
 */
_start:
    /* RP1 UART — the one connected to GPIO header pins 8/10 */
    mov     x9, #0x1F00000000
    add     x9, x9, #0x30000
    mov     w0, #'X'
    str     w0, [x9]
    mov     w0, #'\r'
    str     w0, [x9]
    mov     w0, #'\n'
    str     w0, [x9]

    /* Infinite loop — ACT LED toggle via GPIO would go here */
halt:
    wfe
    b       halt
