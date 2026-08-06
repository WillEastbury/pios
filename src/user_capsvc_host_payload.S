/*
 * user_capsvc_host_payload.S - embed the generic capsvc capsule host binary.
 *
 * user_capsvc_host0.img is produced by the userland build (build_bootstrap.bat),
 * compiled from user/capsvc_host.c with -DCAPSVC_SVC_IDX=0 for the admin
 * proof-of-concept service (see kernel.c CAPSVC_ADMIN_PORT / capsvc_register()).
 */
.section .rodata.user_capsvc_host, "a"
.balign 64
.global user_capsvc_host0_start
.global user_capsvc_host0_end
user_capsvc_host0_start:
    .incbin "user_capsvc_host0.img"
.balign 64
user_capsvc_host0_end:
