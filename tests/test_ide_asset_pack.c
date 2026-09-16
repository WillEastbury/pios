#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef NULL

#include "brotli.h"
#include "ide_asset_pack.h"

static int failures;

#define CHECK(x, what) do { \
    if (!(x)) { printf("  [FAIL] %s\n", what); failures++; } \
} while (0)

u32 hw_crc32c(const void *data, u32 len)
{
    const u8 *p = data;
    u32 crc = 0xFFFFFFFFU;
    while (len--) {
        crc ^= *p++;
        for (u32 bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0x82F63B78U : 0U);
    }
    return crc ^ 0xFFFFFFFFU;
}

int main(void)
{
    printf("test_ide_asset_pack:\n");
    FILE *f = fopen("assets/pios_ide_assets.brp", "rb");
    CHECK(f != NULL, "generated Brotli pack exists");
    if (!f) return 1;
    CHECK(fseek(f, 0, SEEK_END) == 0, "seek asset pack");
    long size = ftell(f);
    CHECK(size > 0 && size <= 0x7FFFFFFF, "asset pack bounded");
    rewind(f);
    u8 *blob = malloc((size_t)size);
    CHECK(blob != NULL, "asset pack allocation");
    if (!blob) { fclose(f); return 1; }
    CHECK(fread(blob, 1, (size_t)size, f) == (size_t)size, "read asset pack");
    fclose(f);

    const u8 *compressed = NULL;
    u32 compressed_bytes = 0, unpacked_bytes = 0, unpacked_crc = 0;
    CHECK(ide_asset_brotli_validate(blob, (u32)size, &compressed, &compressed_bytes,
                                    &unpacked_bytes, &unpacked_crc),
          "Brotli header and compressed CRC");
    u8 *raw = malloc(unpacked_bytes);
    CHECK(raw != NULL, "raw pack allocation");
    if (raw) {
        i32 decoded = brotli_decode(compressed, compressed_bytes, raw, unpacked_bytes);
        CHECK(decoded == (i32)unpacked_bytes, "Python compressor decodes in kernel Brotli");
        CHECK(hw_crc32c(raw, unpacked_bytes) == unpacked_crc, "uncompressed CRC");
        CHECK(ide_asset_pack_validate(raw, unpacked_bytes), "raw PIAS header and records");
        for (u32 id = PIOS_ASSET_IDE_HTML; id <= PIOS_ASSET_IDE_BAREMETAL; id++) {
            const u8 *asset = NULL;
            u32 bytes = 0;
            CHECK(ide_asset_pack_get(raw, unpacked_bytes, id, &asset, &bytes) &&
                  asset && bytes != 0, "all editor assets resolve");
        }
        raw[0] ^= 1U;
        CHECK(!ide_asset_pack_validate(raw, unpacked_bytes), "raw header corruption rejected");
        raw[0] ^= 1U;
        CHECK(ide_asset_pack_validate(raw, unpacked_bytes), "raw pack recovers after rejection");
        free(raw);
    }

    u8 saved = blob[0];
    blob[0] ^= 1U;
    CHECK(!ide_asset_brotli_validate(blob, (u32)size, NULL, NULL, NULL, NULL),
          "Brotli header corruption rejected");
    blob[0] = saved;
    CHECK(!ide_asset_brotli_validate(blob, (u32)size - 1U, NULL, NULL, NULL, NULL),
          "Brotli truncation rejected");
    saved = blob[sizeof(struct pios_asset_brotli_hdr)];
    blob[sizeof(struct pios_asset_brotli_hdr)] ^= 1U;
    CHECK(!ide_asset_brotli_validate(blob, (u32)size, NULL, NULL, NULL, NULL),
          "compressed payload corruption rejected");
    blob[sizeof(struct pios_asset_brotli_hdr)] = saved;
    CHECK(ide_asset_brotli_validate(blob, (u32)size, NULL, NULL, NULL, NULL),
          "Brotli pack recovers after rejection");
    free(blob);
    printf("  %d failed\n", failures);
    return failures ? 1 : 0;
}
