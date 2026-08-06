.section .rdata,"dr"
.balign 16
.global pios_stage2_blob_start
.global pios_stage2_blob_end
pios_stage2_blob_start:
    .incbin "build_uefi/PIOSSTG2.PKG"
pios_stage2_blob_end:
