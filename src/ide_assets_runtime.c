/*
 * Pi 5 stores the editor source pack in the raw stage2 image, then installs
 * it into WALFS after mount. QEMU keeps its direct-boot compiled fallback.
 */
#include "ide_assets.h"
#include "ide_asset_pack.h"
#include "brotli.h"
#include "platform.h"
#include "simd.h"
#include "walfs.h"

const u8 *IDE_HTML;
u32 IDE_HTML_LEN;
const u8 *IDE_PICOWAL_HTML;
u32 IDE_PICOWAL_HTML_LEN;
const u8 *IDE_PICO_HOOKS_JS;
u32 IDE_PICO_HOOKS_JS_LEN;
const u8 *IDE_BAREMETAL_BINARY_JS;
u32 IDE_BAREMETAL_BINARY_JS_LEN;

#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
extern const u8 pios_ide_assets_brotli_start[];
extern const u8 pios_ide_assets_brotli_end[];

#define IDE_ASSET_IO_CHUNK 16384U
static u8 ide_asset_unpack[PIOS_ASSET_PACK_MAX_BYTES] ALIGNED(64);
static u8 ide_asset_io[IDE_ASSET_IO_CHUNK] ALIGNED(64);
static bool ide_assets_ready;
static u32 ide_asset_lengths[4];

static const char *asset_path(u32 id)
{
    switch (id) {
    case PIOS_ASSET_IDE_HTML: return "/var/www/picoscript/index.html";
    case PIOS_ASSET_IDE_PICOWAL: return "/var/www/picoscript/picowal.html";
    case PIOS_ASSET_IDE_HOOKS: return "/var/www/picoscript/pico_hooks.js";
    case PIOS_ASSET_IDE_BAREMETAL: return "/var/www/picoscript/baremetal-binary.js";
    default: return NULL;
    }
}

static u64 asset_dir(u64 parent, const char *path, const char *leaf)
{
    u64 id = walfs_find(path);
    struct walfs_inode ino;
    if (id) {
        if (!walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR))
            return 0;
        return id;
    }
    return walfs_create(parent, leaf, WALFS_DIR, 0755);
}

static bool asset_file_matches(u64 id, const u8 *data, u32 bytes)
{
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR) || ino.size != bytes)
        return false;
    for (u32 off = 0; off < bytes;) {
        u32 chunk = bytes - off > IDE_ASSET_IO_CHUNK ? IDE_ASSET_IO_CHUNK : bytes - off;
        if (walfs_read(id, off, ide_asset_io, chunk) != chunk ||
            memcmp(ide_asset_io, data + off, chunk) != 0)
            return false;
        off += chunk;
    }
    return true;
}

static bool asset_install_file(u64 parent, u32 id, const u8 *data, u32 bytes)
{
    const char *path = asset_path(id);
    const char *leaf = path;
    while (*leaf) {
        if (*leaf++ == '/')
            path = leaf;
    }
    u64 file = walfs_find(asset_path(id));
    if (asset_file_matches(file, data, bytes))
        return true;
    if (file && !walfs_delete(file))
        return false;
    file = walfs_create(parent, path, WALFS_FILE, 0644);
    if (!file)
        return false;
    for (u32 off = 0; off < bytes;) {
        u32 chunk = bytes - off > IDE_ASSET_IO_CHUNK ? IDE_ASSET_IO_CHUNK : bytes - off;
        if (!walfs_write(file, off, data + off, chunk))
            return false;
        off += chunk;
    }
    return asset_file_matches(file, data, bytes);
}
#endif

void ide_assets_bind(void)
{
#if PIOS_PLATFORM != PIOS_PLATFORM_PI5
    const u8 *base = (const u8 *)(usize)PIOS_SHARED_ASSET_BASE;
    const struct pios_asset_hdr *h = (const struct pios_asset_hdr *)base;
    if (h->magic == 0x53414950U && h->version == 1U && h->count <= 16U &&
        h->bytes >= 16U && h->bytes <= PIOS_SHARED_ASSET_SIZE) {
        const struct pios_asset_rec *rec =
            (const struct pios_asset_rec *)(base + 16U);
        for (u32 i = 0; i < h->count; i++) {
            if (rec[i].offset >= h->bytes || rec[i].length > h->bytes - rec[i].offset)
                continue;
            const u8 *p = base + rec[i].offset;
            switch (rec[i].id) {
            case PIOS_ASSET_IDE_HTML: IDE_HTML = p; IDE_HTML_LEN = rec[i].length; break;
            case PIOS_ASSET_IDE_PICOWAL: IDE_PICOWAL_HTML = p; IDE_PICOWAL_HTML_LEN = rec[i].length; break;
            case PIOS_ASSET_IDE_HOOKS: IDE_PICO_HOOKS_JS = p; IDE_PICO_HOOKS_JS_LEN = rec[i].length; break;
            case PIOS_ASSET_IDE_BAREMETAL: IDE_BAREMETAL_BINARY_JS = p; IDE_BAREMETAL_BINARY_JS_LEN = rec[i].length; break;
            }
        }
    }
#endif
#if PIOS_EMBED_IDE_ASSETS
    if (!IDE_HTML) { IDE_HTML = IDE_HTML_EMBED; IDE_HTML_LEN = IDE_HTML_EMBED_LEN; }
    if (!IDE_PICOWAL_HTML) { IDE_PICOWAL_HTML = IDE_PICOWAL_HTML_EMBED; IDE_PICOWAL_HTML_LEN = IDE_PICOWAL_HTML_EMBED_LEN; }
    if (!IDE_PICO_HOOKS_JS) { IDE_PICO_HOOKS_JS = IDE_PICO_HOOKS_JS_EMBED; IDE_PICO_HOOKS_JS_LEN = IDE_PICO_HOOKS_JS_EMBED_LEN; }
    if (!IDE_BAREMETAL_BINARY_JS) { IDE_BAREMETAL_BINARY_JS = IDE_BAREMETAL_BINARY_JS_EMBED; IDE_BAREMETAL_BINARY_JS_LEN = IDE_BAREMETAL_BINARY_JS_EMBED_LEN; }
#endif
}

bool ide_assets_install(void)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
    const u8 *compressed;
    u32 compressed_bytes, unpacked_bytes, unpacked_crc;
    u32 blob_bytes = (u32)(pios_ide_assets_brotli_end - pios_ide_assets_brotli_start);
    if (!ide_asset_brotli_validate(pios_ide_assets_brotli_start, blob_bytes,
                                   &compressed, &compressed_bytes, &unpacked_bytes, &unpacked_crc))
        return false;
    i32 decoded = brotli_decode(compressed, compressed_bytes, ide_asset_unpack, unpacked_bytes);
    if (decoded != (i32)unpacked_bytes ||
        hw_crc32c(ide_asset_unpack, unpacked_bytes) != unpacked_crc ||
        !ide_asset_pack_validate(ide_asset_unpack, unpacked_bytes))
        return false;

    u64 var = asset_dir(WALFS_ROOT_INODE, "/var", "var");
    u64 www = var ? asset_dir(var, "/var/www", "www") : 0;
    u64 root = www ? asset_dir(www, "/var/www/picoscript", "picoscript") : 0;
    if (!root)
        return false;
    for (u32 id = PIOS_ASSET_IDE_HTML; id <= PIOS_ASSET_IDE_BAREMETAL; id++) {
        const u8 *data;
        u32 bytes;
        if (!ide_asset_pack_get(ide_asset_unpack, unpacked_bytes, id, &data, &bytes) ||
            !asset_install_file(root, id, data, bytes))
            return false;
        ide_asset_lengths[id - PIOS_ASSET_IDE_HTML] = bytes;
    }
    walfs_sync();
    ide_assets_ready = true;
    return true;
#else
    return IDE_HTML && IDE_HTML_LEN && IDE_PICOWAL_HTML && IDE_PICOWAL_HTML_LEN &&
           IDE_PICO_HOOKS_JS && IDE_PICO_HOOKS_JS_LEN &&
           IDE_BAREMETAL_BINARY_JS && IDE_BAREMETAL_BINARY_JS_LEN;
#endif
}

bool ide_assets_walfs_file(u32 id, u64 *inode_out, u32 *bytes_out)
{
    if (inode_out) *inode_out = 0;
    if (bytes_out) *bytes_out = 0;
#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
    const char *path = asset_path(id);
    struct walfs_inode ino;
    u64 id_out = path && ide_assets_ready ? walfs_find(path) : 0;
    if (!id_out || !walfs_stat(id_out, &ino) || (ino.flags & WALFS_DIR) ||
        ino.size == 0 || ino.size > 0xFFFFFFFFULL ||
        ino.size != ide_asset_lengths[id - PIOS_ASSET_IDE_HTML])
        return false;
    if (inode_out) *inode_out = id_out;
    if (bytes_out) *bytes_out = (u32)ino.size;
    return true;
#else
    (void)id;
    return false;
#endif
}

bool ide_assets_boot_ready(void)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
    return ide_assets_ready;
#else
    return IDE_HTML && IDE_HTML_LEN;
#endif
}
