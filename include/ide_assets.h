/* ide_assets.h -- declarations for hosted PicoScript WebIDE assets generated
 * by tools/gen_ide_assets.py into src/ide_assets.c.
 *
 * These are streamed verbatim by src/kernel.c (http_build_picoscript_response)
 * under the /picoscript route family:
 *   /picoscript, /picoscript/, /picoscript/index.html, /picoscript/playground.html
 *                                            -> IDE_HTML  (portal + injected bridge)
 *   /picoscript/picowal.html                 -> IDE_PICOWAL_HTML
 *   /picoscript/pico_hooks.js                -> IDE_PICO_HOOKS_JS
 *   /picoscript/baremetal-binary.js          -> IDE_BAREMETAL_BINARY_JS
 *
 * Regenerate after updating the sibling picoscript/baremetaljstools checkouts
 * or the tools/ide_*.{js,html} sources with:
 *   python tools/gen_ide_assets.py
 * and verify with:
 *   python tools/test_ide_assets.py
 */
#ifndef PIOS_IDE_ASSETS_H
#define PIOS_IDE_ASSETS_H

#include "types.h"
#include "platform.h"

/* Non-Pi5 builds bind the legacy shared package (or the QEMU fallback).
 * Pi5 serves the same named assets from WALFS after verified raw-stage2
 * extraction; these pointers remain NULL there. */
extern const u8 *IDE_HTML;
extern u32 IDE_HTML_LEN;

/* PIOS PicoWAL workspace page (tools/ide_picowal_workspace.html), opened from
 * the portal's PicoWAL tab in a same-origin iframe. */
extern const u8 *IDE_PICOWAL_HTML;
extern u32 IDE_PICOWAL_HTML_LEN;

/* Standalone host-hook table (../picoscript/vm/pico_hooks.js) for backward
 * compatibility / hook-namespace verification. */
extern const u8 *IDE_PICO_HOOKS_JS;
extern u32 IDE_PICO_HOOKS_JS_LEN;

/* BSO1 codec (../baremetaljstools/src/BareMetal.Binary.js), used by the
 * workspace's Fast Serial (BSO1) panel. */
extern const u8 *IDE_BAREMETAL_BINARY_JS;
extern u32 IDE_BAREMETAL_BINARY_JS_LEN;

void ide_assets_bind(void);
/* Pi5 extraction is called only after WALFS has mounted. */
bool ide_assets_install(void);
bool ide_assets_walfs_file(u32 asset_id, u64 *inode_out, u32 *bytes_out);
bool ide_assets_boot_ready(void);

#if PIOS_EMBED_IDE_ASSETS
extern const u8 IDE_HTML_EMBED[];
extern const u32 IDE_HTML_EMBED_LEN;
extern const u8 IDE_PICOWAL_HTML_EMBED[];
extern const u32 IDE_PICOWAL_HTML_EMBED_LEN;
extern const u8 IDE_PICO_HOOKS_JS_EMBED[];
extern const u32 IDE_PICO_HOOKS_JS_EMBED_LEN;
extern const u8 IDE_BAREMETAL_BINARY_JS_EMBED[];
extern const u32 IDE_BAREMETAL_BINARY_JS_EMBED_LEN;
#endif

#endif /* PIOS_IDE_ASSETS_H */
