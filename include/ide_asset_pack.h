#pragma once

#include "types.h"

#define PIOS_ASSET_PACK_MAGIC       0x53414950U /* 'PIAS' */
#define PIOS_ASSET_PACK_VERSION     2U
#define PIOS_ASSET_BROTLI_MAGIC     0x50425250U /* 'PBRP' */
#define PIOS_ASSET_BROTLI_VERSION   1U
#define PIOS_ASSET_PACK_MAX_BYTES   (2100000U)
#define PIOS_ASSET_IDE_HTML         1U
#define PIOS_ASSET_IDE_PICOWAL      2U
#define PIOS_ASSET_IDE_HOOKS        3U
#define PIOS_ASSET_IDE_BAREMETAL    4U

struct pios_asset_rec {
    u32 id;
    u32 offset;
    u32 length;
    u32 _reserved;
} PACKED;

struct pios_asset_hdr {
    u32 magic;
    u16 version;
    u16 count;
    u32 bytes;
    u32 crc32c;
} PACKED;

struct pios_asset_brotli_hdr {
    u32 magic;
    u16 version;
    u16 header_bytes;
    u32 compressed_bytes;
    u32 uncompressed_bytes;
    u32 compressed_crc32c;
    u32 uncompressed_crc32c;
    u32 header_crc32c;
} PACKED;

_Static_assert(sizeof(struct pios_asset_hdr) == 16, "PIAS header ABI");
_Static_assert(sizeof(struct pios_asset_brotli_hdr) == 28, "PBRP header ABI");

bool ide_asset_brotli_validate(const u8 *blob, u32 blob_bytes,
                               const u8 **compressed_out, u32 *compressed_bytes_out,
                               u32 *uncompressed_bytes_out, u32 *uncompressed_crc_out);
bool ide_asset_pack_validate(const u8 *pack, u32 pack_bytes);
bool ide_asset_pack_get(const u8 *pack, u32 pack_bytes, u32 id,
                        const u8 **data_out, u32 *len_out);
