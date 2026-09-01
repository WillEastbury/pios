/*
 * Bind PicoScript IDE assets from the shared PIAS pack that stage0 copies
 * to PIOS_SHARED_ASSET_BASE. QEMU direct-boot may fall back to compiled-in
 * blobs (PIOS_EMBED_IDE_ASSETS).
 */
#include "ide_assets.h"
#include "platform.h"

const u8 *IDE_HTML;
u32 IDE_HTML_LEN;
const u8 *IDE_PICOWAL_HTML;
u32 IDE_PICOWAL_HTML_LEN;
const u8 *IDE_PICO_HOOKS_JS;
u32 IDE_PICO_HOOKS_JS_LEN;
const u8 *IDE_BAREMETAL_BINARY_JS;
u32 IDE_BAREMETAL_BINARY_JS_LEN;

struct pios_asset_rec {
    u32 id;
    u32 offset;
    u32 length;
    u32 _pad;
} PACKED;

struct pios_asset_hdr {
    u32 magic;
    u16 version;
    u16 count;
    u32 bytes;
} PACKED;

static const u8 *pack_find(u32 id, u32 *len)
{
    const u8 *base = (const u8 *)(usize)PIOS_SHARED_ASSET_BASE;
    const struct pios_asset_hdr *h = (const struct pios_asset_hdr *)base;
    if (h->magic != PIOS_ASSET_PACK_MAGIC || h->version != PIOS_ASSET_PACK_VERSION)
        return NULL;
    if (h->count == 0 || h->count > 16U || h->bytes < sizeof(*h) ||
        h->bytes > PIOS_SHARED_ASSET_SIZE)
        return NULL;
    const struct pios_asset_rec *rec =
        (const struct pios_asset_rec *)(base + sizeof(*h));
    for (u32 i = 0; i < h->count; i++) {
        if (rec[i].id != id)
            continue;
        if (rec[i].offset >= h->bytes ||
            rec[i].length > h->bytes - rec[i].offset)
            return NULL;
        if (len)
            *len = rec[i].length;
        return base + rec[i].offset;
    }
    return NULL;
}

void ide_assets_bind(void)
{
    u32 n;
    const u8 *p;

    p = pack_find(PIOS_ASSET_IDE_HTML, &n);
    if (p) {
        IDE_HTML = p;
        IDE_HTML_LEN = n;
    }
    p = pack_find(PIOS_ASSET_IDE_PICOWAL, &n);
    if (p) {
        IDE_PICOWAL_HTML = p;
        IDE_PICOWAL_HTML_LEN = n;
    }
    p = pack_find(PIOS_ASSET_IDE_HOOKS, &n);
    if (p) {
        IDE_PICO_HOOKS_JS = p;
        IDE_PICO_HOOKS_JS_LEN = n;
    }
    p = pack_find(PIOS_ASSET_IDE_BAREMETAL, &n);
    if (p) {
        IDE_BAREMETAL_BINARY_JS = p;
        IDE_BAREMETAL_BINARY_JS_LEN = n;
    }

#if PIOS_EMBED_IDE_ASSETS
    if (!IDE_HTML) {
        IDE_HTML = IDE_HTML_EMBED;
        IDE_HTML_LEN = IDE_HTML_EMBED_LEN;
    }
    if (!IDE_PICOWAL_HTML) {
        IDE_PICOWAL_HTML = IDE_PICOWAL_HTML_EMBED;
        IDE_PICOWAL_HTML_LEN = IDE_PICOWAL_HTML_EMBED_LEN;
    }
    if (!IDE_PICO_HOOKS_JS) {
        IDE_PICO_HOOKS_JS = IDE_PICO_HOOKS_JS_EMBED;
        IDE_PICO_HOOKS_JS_LEN = IDE_PICO_HOOKS_JS_EMBED_LEN;
    }
    if (!IDE_BAREMETAL_BINARY_JS) {
        IDE_BAREMETAL_BINARY_JS = IDE_BAREMETAL_BINARY_JS_EMBED;
        IDE_BAREMETAL_BINARY_JS_LEN = IDE_BAREMETAL_BINARY_JS_EMBED_LEN;
    }
#endif
}
