#include "ide_asset_pack.h"
#include "simd.h"

static bool asset_header(const u8 *pack, u32 pack_bytes,
                         const struct pios_asset_hdr **hdr_out)
{
    if (!pack || pack_bytes < sizeof(struct pios_asset_hdr))
        return false;
    const struct pios_asset_hdr *h = (const struct pios_asset_hdr *)pack;
    if (h->magic != PIOS_ASSET_PACK_MAGIC || h->version != PIOS_ASSET_PACK_VERSION ||
        h->count != 4U || h->bytes != pack_bytes ||
        h->bytes < sizeof(*h) + h->count * sizeof(struct pios_asset_rec) ||
        h->bytes > PIOS_ASSET_PACK_MAX_BYTES ||
        hw_crc32c(pack + sizeof(*h), h->bytes - sizeof(*h)) != h->crc32c)
        return false;
    if (hdr_out) *hdr_out = h;
    return true;
}

bool ide_asset_brotli_validate(const u8 *blob, u32 blob_bytes,
                               const u8 **compressed_out, u32 *compressed_bytes_out,
                               u32 *uncompressed_bytes_out, u32 *uncompressed_crc_out)
{
    if (compressed_out) *compressed_out = NULL;
    if (compressed_bytes_out) *compressed_bytes_out = 0;
    if (uncompressed_bytes_out) *uncompressed_bytes_out = 0;
    if (uncompressed_crc_out) *uncompressed_crc_out = 0;
    if (!blob || blob_bytes < sizeof(struct pios_asset_brotli_hdr))
        return false;
    const struct pios_asset_brotli_hdr *h =
        (const struct pios_asset_brotli_hdr *)blob;
    if (h->magic != PIOS_ASSET_BROTLI_MAGIC ||
        h->version != PIOS_ASSET_BROTLI_VERSION ||
        h->header_bytes != sizeof(*h) ||
        h->compressed_bytes == 0 ||
        h->compressed_bytes > blob_bytes - h->header_bytes ||
        h->header_bytes + h->compressed_bytes != blob_bytes ||
        h->uncompressed_bytes == 0 ||
        h->uncompressed_bytes > PIOS_ASSET_PACK_MAX_BYTES ||
        hw_crc32c(blob, h->header_bytes - sizeof(h->header_crc32c)) != h->header_crc32c ||
        hw_crc32c(blob + h->header_bytes, h->compressed_bytes) != h->compressed_crc32c)
        return false;
    if (compressed_out) *compressed_out = blob + h->header_bytes;
    if (compressed_bytes_out) *compressed_bytes_out = h->compressed_bytes;
    if (uncompressed_bytes_out) *uncompressed_bytes_out = h->uncompressed_bytes;
    if (uncompressed_crc_out) *uncompressed_crc_out = h->uncompressed_crc32c;
    return true;
}

bool ide_asset_pack_validate(const u8 *pack, u32 pack_bytes)
{
    const struct pios_asset_hdr *h;
    if (!asset_header(pack, pack_bytes, &h))
        return false;
    const struct pios_asset_rec *rec =
        (const struct pios_asset_rec *)(pack + sizeof(*h));
    u32 seen = 0;
    for (u32 i = 0; i < h->count; i++) {
        if (rec[i].id < PIOS_ASSET_IDE_HTML || rec[i].id > PIOS_ASSET_IDE_BAREMETAL ||
            rec[i].length == 0 || rec[i].offset < sizeof(*h) + h->count * sizeof(*rec) ||
            rec[i].offset > h->bytes || rec[i].length > h->bytes - rec[i].offset)
            return false;
        u32 bit = 1U << (rec[i].id - PIOS_ASSET_IDE_HTML);
        if (seen & bit)
            return false;
        seen |= bit;
    }
    return seen == 0xFU;
}

bool ide_asset_pack_get(const u8 *pack, u32 pack_bytes, u32 id,
                        const u8 **data_out, u32 *len_out)
{
    if (data_out) *data_out = NULL;
    if (len_out) *len_out = 0;
    if (!ide_asset_pack_validate(pack, pack_bytes))
        return false;
    const struct pios_asset_hdr *h = (const struct pios_asset_hdr *)pack;
    const struct pios_asset_rec *rec =
        (const struct pios_asset_rec *)(pack + sizeof(*h));
    for (u32 i = 0; i < h->count; i++) {
        if (rec[i].id == id) {
            if (data_out) *data_out = pack + rec[i].offset;
            if (len_out) *len_out = rec[i].length;
            return true;
        }
    }
    return false;
}
