#ifndef PIOS_PLATFORM_PI5
#define PIOS_PLATFORM_PI5 1
#endif
#ifndef PIOS_PLATFORM
#define PIOS_PLATFORM PIOS_PLATFORM_PI5
#endif

#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
    .section .rodata.bmg_guc, "a"
    .balign 64
    .global bmg_guc_70_start
    .global bmg_guc_70_end
    .global bmg_guc_70_size
bmg_guc_70_start:
    .incbin "firmware/xe/bmg_guc_70.bin"
bmg_guc_70_end:
    .set bmg_guc_70_size, bmg_guc_70_end - bmg_guc_70_start
#endif
