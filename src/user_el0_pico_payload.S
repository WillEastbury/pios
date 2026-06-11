/*
 * user_el0_pico_payload.S - embed the EL0 PicoScript VM proof image.
 */
.section .rodata.user_el0_pico, "a"
.balign 64
.global user_el0_pico_start
.global user_el0_pico_end
user_el0_pico_start:
    .incbin "user_el0_pico.img"
.balign 64
user_el0_pico_end:
