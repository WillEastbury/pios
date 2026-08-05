#include "keystore.h"
#include "crypto.h"
#include "mailbox.h"
#include "sd.h"
#include "timer.h"
#include "walfs.h"
#include "build_version.h"
#include "simd.h"
#include "uart.h"
#include "platform.h"

#define KEYSTORE_MAGIC   0x5254534BU /* 'KSTR' */
#define KEYSTORE_VERSION 1U

#define KS_ERR_NONE      0U
#define KS_ERR_LBA       1U
#define KS_ERR_READ      2U
#define KS_ERR_WRITE     3U
#define KS_ERR_DECRYPT   4U
#define KS_ERR_SERIAL    5U

struct keystore_record {
    u32 magic;
    u32 version;
    u32 generation;
    u32 flags;
    u8 nonce[12];
    u8 root_cipher[32];
    u8 tag[16];
    u8 reserved[512 - 4 * 4 - 12 - 32 - 16];
} PACKED;

static struct keystore_status g_ks;
static u8 ks_sector[SD_BLOCK_SIZE] ALIGNED(64);

static const u8 ks_salt[32] = {
    0x50,0x49,0x4F,0x53,0x2D,0x4B,0x45,0x59,0x53,0x54,0x4F,0x52,0x45,0x2D,0x76,0x31,
    0x9A,0x51,0xE7,0x3C,0x42,0x88,0x10,0xCD,0x6B,0x2F,0xA4,0xD9,0x31,0x77,0xBE,0x04
};

static void secure_zero(void *p, u32 n)
{
    volatile u8 *v = (volatile u8 *)p;
    while (n--) *v++ = 0;
}

static void put32(u8 *p, u32 v)
{
    p[0] = (u8)v;
    p[1] = (u8)(v >> 8);
    p[2] = (u8)(v >> 16);
    p[3] = (u8)(v >> 24);
}

static u32 get32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static bool board_serial(u8 out[8])
{
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* QEMU has no VideoCore board serial. The virtual-SD image builder
     * provisions a persistent, nonzero MBR disk signature; use that public
     * device identity exactly as real hardware uses its public board serial.
     * This stabilizes keystore wrapping across derives/reboots without
     * pretending to provide CSPRNG entropy (random.c still fails closed). */
    static u8 sector[SD_BLOCK_SIZE] ALIGNED(64);
    if (!sd_read_block(0, sector) || sector[510] != 0x55 || sector[511] != 0xAA)
        return false;
    u32 disk_id = get32(sector + 440);
    u32 p2_start = get32(sector + 0x1CE + 8);
    u32 p2_size = get32(sector + 0x1CE + 12);
    if (disk_id == 0 || p2_start == 0 || p2_size == 0)
        return false;
    put32(out + 0, disk_id);
    put32(out + 4, disk_id ^ p2_start ^ p2_size ^ 0x51454D55U);
    return true;
#elif !PIOS_HAS_MAILBOX_FB
    /* No VideoCore mailbox on this platform (QEMU/Hyper-V): MBOX_BASE is
     * literally 0 here (see platform.h), so calling mbox_call() at all
     * would dereference address 0 -- GCC's UB-based optimizer turns that
     * provably-null MMIO access into a trap (brk) rather than a normal
     * fault. derive_wrap_key() already falls back to the monotonic timer
     * when this returns false, so just report "no serial" up front. */
    (void)out;
    return false;
#else
    static volatile u32 buf[8] ALIGNED(16);
    simd_zero((void *)buf, sizeof(buf));
    buf[0] = sizeof(buf);
    buf[1] = 0;
    buf[2] = TAG_GET_BOARD_SERIAL;
    buf[3] = 8;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = TAG_END;
    if (!mbox_call(MBOX_CH_PROP, buf))
        return false;
    put32(out + 0, buf[5]);
    put32(out + 4, buf[6]);
    return true;
#endif
}

/* Returns the LBA of the sealed keystore record, or KEYSTORE_LBA_INVALID if
 * WALFS hasn't established a partition location yet. Delegates entirely to
 * walfs_status()'s already-authoritative discovery (walfs.c's
 * discover_partition(), which correctly handles the p1-single-volume and
 * whole-disk-fallback cases) instead of re-deriving/duplicating a
 * simplified MBR re-scan here.
 *
 * IMPORTANT: this must NOT treat partition_lba==0 as "not found". A
 * previous version conflated the two (returning 0 for both "WALFS isn't
 * mounted" and "found, genuinely at LBA 0") -- but partition_lba==0 is a
 * legitimate value on the QEMU no-attached-disk RAM fallback path (whole-
 * disk volume, no MBR partition table), which made keystore_init() (and
 * therefore x509/TLS 1.3 certificate provisioning) fail closed on every
 * QEMU boot even though WALFS itself had mounted successfully. Use the
 * explicit `mounted` flag as the only "is this valid" signal. */
#define KEYSTORE_LBA_INVALID 0xFFFFFFFFU

static u32 keystore_lba(void)
{
    struct walfs_status_snapshot ws;
    walfs_status(&ws);
    if (!ws.mounted) return KEYSTORE_LBA_INVALID;
    return ws.partition_lba + (PIOS_USER_RECORDS_OFFSET / SD_BLOCK_SIZE);
}

static void derive_wrap_key(u8 key[32], bool *serial_ok)
{
    u8 serial[8];
    u8 prk[32];
    static const u8 info[] = "PIOS keystore wrap key v1";
    bool ok = board_serial(serial);
    if (!ok) {
        u64 t = timer_monotonic_ms();
        put32(serial + 0, (u32)t);
        put32(serial + 4, (u32)(t >> 32));
    }
    hkdf_extract(ks_salt, sizeof(ks_salt), serial, sizeof(serial), prk);
    hkdf_expand(prk, sizeof(prk), info, (u32)(sizeof(info) - 1), key, 32);
    secure_zero(serial, sizeof(serial));
    secure_zero(prk, sizeof(prk));
    if (serial_ok) *serial_ok = ok;
}

static void make_nonce(u8 nonce[12], u32 generation)
{
    u64 t = timer_monotonic_ms();
    put32(nonce + 0, generation);
    put32(nonce + 4, (u32)t);
    put32(nonce + 8, (u32)(t >> 32) ^ 0xA5C35A96U);
}

static void make_root_key(u8 root[32], const u8 wrap_key[32], u32 generation)
{
    u8 seed[32];
    u64 t = timer_monotonic_ms();
    put32(seed + 0, generation);
    put32(seed + 4, (u32)t);
    put32(seed + 8, (u32)(t >> 32));
    put32(seed + 12, (u32)(usize)&g_ks);
    for (u32 i = 16; i < 32; i++) seed[i] = (u8)(0xD0U + i);
    hmac_sha256(wrap_key, 32, seed, sizeof(seed), root);
    secure_zero(seed, sizeof(seed));
}

static bool decrypt_root(const struct keystore_record *rec, const u8 wrap_key[32], u8 root[32])
{
    struct aes_gcm_ctx g;
    aes_gcm_init(&g, wrap_key, 256);
    bool ok = aes_gcm_decrypt(&g, rec->nonce, sizeof(rec->nonce),
                              (const u8 *)rec, 16,
                              rec->root_cipher, sizeof(rec->root_cipher),
                              root, rec->tag);
    secure_zero(&g, sizeof(g));
    return ok;
}

static bool seal_root(struct keystore_record *rec, const u8 wrap_key[32], const u8 root[32], u32 generation)
{
    simd_zero(rec, sizeof(*rec));
    rec->magic = KEYSTORE_MAGIC;
    rec->version = KEYSTORE_VERSION;
    rec->generation = generation;
    rec->flags = 1;
    make_nonce(rec->nonce, generation);
    struct aes_gcm_ctx g;
    aes_gcm_init(&g, wrap_key, 256);
    bool ok = aes_gcm_encrypt(&g, rec->nonce, sizeof(rec->nonce),
                              (const u8 *)rec, 16,
                              root, 32,
                              rec->root_cipher, rec->tag);
    secure_zero(&g, sizeof(g));
    return ok;
}

static u32 fingerprint_root(const u8 root[32])
{
    u8 h[32];
    static const u8 label[] = "PIOS keystore fingerprint";
    hmac_sha256(root, 32, label, (u32)(sizeof(label) - 1), h);
    u32 fp = get32(h);
    secure_zero(h, sizeof(h));
    return fp;
}

bool keystore_init(void)
{
    simd_zero(&g_ks, sizeof(g_ks));
    u32 lba = keystore_lba();
    g_ks.user_records_lba = lba;
    if (lba == KEYSTORE_LBA_INVALID) {
        g_ks.last_error = KS_ERR_LBA;
        return false;
    }

    bool serial_ok = false;
    u8 wrap_key[32];
    derive_wrap_key(wrap_key, &serial_ok);
    g_ks.board_serial_ok = serial_ok;

    if (!sd_read_block(lba, ks_sector)) {
        secure_zero(wrap_key, sizeof(wrap_key));
        g_ks.last_error = KS_ERR_READ;
        return false;
    }

    struct keystore_record rec;
    simd_memcpy(&rec, ks_sector, sizeof(rec));
    bool need_seed = rec.magic != KEYSTORE_MAGIC || rec.version != KEYSTORE_VERSION;
    if (!need_seed) {
        u8 root[32];
        if (!decrypt_root(&rec, wrap_key, root)) {
            secure_zero(root, sizeof(root));
            secure_zero(wrap_key, sizeof(wrap_key));
            g_ks.last_error = KS_ERR_DECRYPT;
            return false;
        }
        g_ks.fingerprint32 = fingerprint_root(root);
        g_ks.generation = rec.generation;
        g_ks.sealed = true;
        g_ks.initialized = true;
        g_ks.last_error = KS_ERR_NONE;
        secure_zero(root, sizeof(root));
        secure_zero(wrap_key, sizeof(wrap_key));
        return true;
    }

    u8 root[32];
    make_root_key(root, wrap_key, 1);
    if (!seal_root(&rec, wrap_key, root, 1)) {
        secure_zero(root, sizeof(root));
        secure_zero(wrap_key, sizeof(wrap_key));
        g_ks.last_error = KS_ERR_DECRYPT;
        return false;
    }
    simd_zero(ks_sector, sizeof(ks_sector));
    simd_memcpy(ks_sector, &rec, sizeof(rec));
    if (!sd_write_block(lba, ks_sector)) {
        secure_zero(root, sizeof(root));
        secure_zero(wrap_key, sizeof(wrap_key));
        secure_zero(&rec, sizeof(rec));
        g_ks.last_error = KS_ERR_WRITE;
        return false;
    }
    g_ks.fingerprint32 = fingerprint_root(root);
    g_ks.generation = rec.generation;
    g_ks.sealed = true;
    g_ks.initialized = true;
    g_ks.last_error = KS_ERR_NONE;
    secure_zero(root, sizeof(root));
    secure_zero(wrap_key, sizeof(wrap_key));
    secure_zero(&rec, sizeof(rec));
    return true;
}

void keystore_status(struct keystore_status *out)
{
    if (!out) return;
    *out = g_ks;
}

bool keystore_derive_fingerprint(const char *label, u32 *out_fingerprint)
{
    u8 derived[32];
    if (!out_fingerprint)
        return false;
    if (!keystore_derive_secret(label, derived, sizeof(derived)))
        return false;
    *out_fingerprint = get32(derived);
    secure_zero(derived, sizeof(derived));
    return true;
}

bool keystore_derive_secret(const char *label, u8 *out, u32 out_len)
{
    if (!label || !out || out_len == 0 || out_len > 32 || !g_ks.sealed)
        return false;
    u32 lba = keystore_lba();
    if (lba == KEYSTORE_LBA_INVALID || !sd_read_block(lba, ks_sector))
        return false;
    struct keystore_record rec;
    simd_memcpy(&rec, ks_sector, sizeof(rec));
    if (rec.magic != KEYSTORE_MAGIC || rec.version != KEYSTORE_VERSION)
        return false;
    bool serial_ok = false;
    u8 wrap_key[32];
    u8 root[32];
    u8 derived[32];
    derive_wrap_key(wrap_key, &serial_ok);
    bool ok = decrypt_root(&rec, wrap_key, root);
    if (ok) {
        hmac_sha256(root, 32, (const u8 *)label, pios_strlen(label), derived);
        simd_memcpy(out, derived, out_len);
    }
    secure_zero(wrap_key, sizeof(wrap_key));
    secure_zero(root, sizeof(root));
    secure_zero(derived, sizeof(derived));
    secure_zero(&rec, sizeof(rec));
    return ok;
}
