#ifdef PIOS_EMBED_RAW_IDE_ASSETS
.section .rodata.ide_assets_pack, "a"
.balign 64
.global pios_ide_assets_brotli_start
.global pios_ide_assets_brotli_end
pios_ide_assets_brotli_start:
    .incbin "assets/pios_ide_assets.brp"
pios_ide_assets_brotli_end:
#endif
