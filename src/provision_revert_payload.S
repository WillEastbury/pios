.section .rodata.provision_revert_payload, "a"
.global provision_revert_payload_start
.global provision_revert_payload_end
.balign 16
provision_revert_payload_start:
    .incbin "revert_stage2.bin"
.balign 16
provision_revert_payload_end:
