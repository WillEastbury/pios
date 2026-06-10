/*
 * user_el0_probe_payload.S - embed the tiny EL0 probe flat binary.
 */
.section .rodata.user_el0_probe, "a"
.balign 64
.global user_el0_probe_start
.global user_el0_probe_end
user_el0_probe_start:
    .incbin "user_el0_probe.img"
.balign 64
user_el0_probe_end:
