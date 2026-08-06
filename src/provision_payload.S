.section .rodata.provision_payload, "a"
.global provision_payload_start
.global provision_payload_end
.balign 16
provision_payload_start:
    .incbin "real_kernel.img"
.balign 16
provision_payload_end:
