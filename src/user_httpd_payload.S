/*
 * user_httpd_payload.S - embed the userland HTTP server flat binary.
 *
 * user_httpd.img is produced by the userland build (user/httpd.c linked flat at
 * the core-2 slot-0 base via user/user.ld). The kernel copies these bytes into
 * the process slot and execs them at EL0. See proc_exec_from_mem in src/proc.c
 * and the boot launch in core2_main (src/kernel.c).
 */
.section .rodata.user_httpd, "a"
.balign 64
.global user_httpd_start
.global user_httpd_end
user_httpd_start:
    .incbin "user_httpd.img"
.balign 64
user_httpd_end:
