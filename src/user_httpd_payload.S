/*
 * user_httpd_payload.S - embed the userland HTTP server flat binary.
 *
 * user_httpd_vm.img and user_httpd_native.img are produced by the userland build.
 * Both are linked at the same high EL0 VA; the kernel maps them into different
 * physical process slots/cores for benchmark comparison.
 */
.section .rodata.user_httpd, "a"
.balign 64
.global user_httpd_vm_start
.global user_httpd_vm_end
.global user_httpd_native_start
.global user_httpd_native_end
user_httpd_vm_start:
    .incbin "user_httpd_vm.img"
.balign 64
user_httpd_vm_end:
user_httpd_native_start:
    .incbin "user_httpd_native.img"
.balign 64
user_httpd_native_end:
