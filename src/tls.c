#include "tls.h"
#include "crypto.h"
#include "simd.h"
#include "timer.h"

#define TLS_MAGIC_CHLO   0x43484C4FU /* "CHLO" */
#define TLS_MAGIC_SHLO   0x53484C4FU /* "SHLO" */
#define TLS_HS_VER       1
#define TLS_KEY_LEN      16
#define TLS_IV_LEN       12
#define TLS_TAG_LEN      16
#define TLS_MAX_RECORD   1024
#define TLS_IO_TIMEOUT   5000

struct tls_handshake {
    u32 magic;
    u8  version;
    u8  random[32];
} PACKED;

struct tls_conn_state {
    bool in_use;
    bool established;
    bool is_client;
    tcp_conn_t tcp;

    struct aes_gcm_ctx tx_gcm;
    struct aes_gcm_ctx rx_gcm;
    u8 tx_iv[TLS_IV_LEN];
    u8 rx_iv[TLS_IV_LEN];
    u64 tx_seq;
    u64 rx_seq;
};

static struct tls_conn_state tls_conns[TLS_MAX_CONNECTIONS];

/* PSK placeholder for issue #19 minimal channel.
 * TODO(issue #19): Replace with per-principal secret provisioning. */
static const u8 tls_psk[] = "PIOS_TLS_PSK_v1";

static inline u16 load_be16(const u8 *p) {
    return (u16)(((u16)p[0] << 8) | (u16)p[1]);
}

static inline void store_be16(u8 *p, u16 v) {
    p[0] = (u8)(v >> 8);
    p[1] = (u8)v;
}

static inline void store_be64(u8 *p, u64 v) {
    p[0] = (u8)(v >> 56);
    p[1] = (u8)(v >> 48);
    p[2] = (u8)(v >> 40);
    p[3] = (u8)(v >> 32);
    p[4] = (u8)(v >> 24);
    p[5] = (u8)(v >> 16);
    p[6] = (u8)(v >> 8);
    p[7] = (u8)v;
}

static inline u32 store_rand32(void) {
    u64 t = timer_ticks();
    u32 seed = (u32)t ^ (u32)(t >> 32) ^ 0xB15A9C3DU;
    return hw_crc32c(&seed, sizeof(seed));
}

static bool tcp_write_all(tcp_conn_t tcp, const u8 *buf, u32 len) {
    u32 off = 0;
    u64 start = timer_ticks();

    while (off < len) {
        u32 n = tcp_write(tcp, buf + off, len - off);
        if (n > 0) {
            off += n;
            continue;
        }
        if ((timer_ticks() - start) > TLS_IO_TIMEOUT)
            return false;
    }
    return true;
}

static bool tcp_read_all(tcp_conn_t tcp, u8 *buf, u32 len) {
    u32 off = 0;
    u64 start = timer_ticks();

    while (off < len) {
        u32 n = tcp_read(tcp, buf + off, len - off);
        if (n > 0) {
            off += n;
            continue;
        }
        if ((timer_ticks() - start) > TLS_IO_TIMEOUT)
            return false;
    }
    return true;
}

static bool tls_valid(tls_conn_t conn) {
    return conn >= 0 && conn < TLS_MAX_CONNECTIONS && tls_conns[conn].in_use;
}

static tls_conn_t tls_alloc(void) {
    for (i32 i = 0; i < TLS_MAX_CONNECTIONS; i++) {
        if (!tls_conns[i].in_use) {
            simd_zero(&tls_conns[i], sizeof(tls_conns[i]));
            tls_conns[i].in_use = true;
            return i;
        }
    }
    return -1;
}

static void tls_make_nonce(u8 out[TLS_IV_LEN], const u8 iv[TLS_IV_LEN], u64 seq) {
    simd_memcpy(out, iv, TLS_IV_LEN);
    out[4]  ^= (u8)(seq >> 56);
    out[5]  ^= (u8)(seq >> 48);
    out[6]  ^= (u8)(seq >> 40);
    out[7]  ^= (u8)(seq >> 32);
    out[8]  ^= (u8)(seq >> 24);
    out[9]  ^= (u8)(seq >> 16);
    out[10] ^= (u8)(seq >> 8);
    out[11] ^= (u8)seq;
}

static bool tls_derive_keys(struct tls_conn_state *c,
                            const struct tls_handshake *ch,
                            const struct tls_handshake *sh) {
    u8 transcript[64 + 1];
    u8 transcript_hash[32];
    u8 prk[32];
    u8 keymat[(TLS_KEY_LEN + TLS_IV_LEN) * 2];
    u8 role = c->is_client ? 0x43 : 0x53;

    if (!c || !ch || !sh) return false;

    simd_memcpy(transcript, ch->random, 32);
    simd_memcpy(transcript + 32, sh->random, 32);
    transcript[64] = role;
    sha256(transcript, sizeof(transcript), transcript_hash);

    hkdf_extract(tls_psk, (u32)(sizeof(tls_psk) - 1), transcript_hash, sizeof(transcript_hash), prk);
    hkdf_expand(prk, sizeof(prk), (const u8 *)"PIOS-TLS-KEYMAT", 14, keymat, sizeof(keymat));

    if (c->is_client) {
        aes_gcm_init(&c->tx_gcm, keymat + 0, 128);
        aes_gcm_init(&c->rx_gcm, keymat + TLS_KEY_LEN, 128);
        simd_memcpy(c->tx_iv, keymat + TLS_KEY_LEN * 2, TLS_IV_LEN);
        simd_memcpy(c->rx_iv, keymat + TLS_KEY_LEN * 2 + TLS_IV_LEN, TLS_IV_LEN);
    } else {
        aes_gcm_init(&c->tx_gcm, keymat + TLS_KEY_LEN, 128);
        aes_gcm_init(&c->rx_gcm, keymat + 0, 128);
        simd_memcpy(c->tx_iv, keymat + TLS_KEY_LEN * 2 + TLS_IV_LEN, TLS_IV_LEN);
        simd_memcpy(c->rx_iv, keymat + TLS_KEY_LEN * 2, TLS_IV_LEN);
    }

    c->tx_seq = 0;
    c->rx_seq = 0;
    c->established = true;
    return true;
}

tls_conn_t tls_connect(tcp_conn_t tcp) {
    tls_conn_t id;
    struct tls_conn_state *c;
    struct tls_handshake ch;
    struct tls_handshake sh;

    if (tcp < 0) return -1;
    id = tls_alloc();
    if (id < 0) return -1;

    c = &tls_conns[id];
    c->tcp = tcp;
    c->is_client = true;

    ch.magic = TLS_MAGIC_CHLO;
    ch.version = TLS_HS_VER;
    for (u32 i = 0; i < sizeof(ch.random); i += 4) {
        u32 r = store_rand32();
        ch.random[i + 0] = (u8)r;
        ch.random[i + 1] = (u8)(r >> 8);
        ch.random[i + 2] = (u8)(r >> 16);
        ch.random[i + 3] = (u8)(r >> 24);
    }

    if (!tcp_write_all(tcp, (const u8 *)&ch, sizeof(ch))) {
        tls_close(id);
        return -1;
    }
    if (!tcp_read_all(tcp, (u8 *)&sh, sizeof(sh))) {
        tls_close(id);
        return -1;
    }
    if (sh.magic != TLS_MAGIC_SHLO || sh.version != TLS_HS_VER) {
        tls_close(id);
        return -1;
    }

    if (!tls_derive_keys(c, &ch, &sh)) {
        tls_close(id);
        return -1;
    }

    return id;
}

tls_conn_t tls_accept(tcp_conn_t tcp) {
    tls_conn_t id;
    struct tls_conn_state *c;
    struct tls_handshake ch;
    struct tls_handshake sh;

    if (tcp < 0) return -1;
    id = tls_alloc();
    if (id < 0) return -1;

    c = &tls_conns[id];
    c->tcp = tcp;
    c->is_client = false;

    if (!tcp_read_all(tcp, (u8 *)&ch, sizeof(ch))) {
        tls_close(id);
        return -1;
    }
    if (ch.magic != TLS_MAGIC_CHLO || ch.version != TLS_HS_VER) {
        tls_close(id);
        return -1;
    }

    sh.magic = TLS_MAGIC_SHLO;
    sh.version = TLS_HS_VER;
    for (u32 i = 0; i < sizeof(sh.random); i += 4) {
        u32 r = store_rand32();
        sh.random[i + 0] = (u8)r;
        sh.random[i + 1] = (u8)(r >> 8);
        sh.random[i + 2] = (u8)(r >> 16);
        sh.random[i + 3] = (u8)(r >> 24);
    }

    if (!tcp_write_all(tcp, (const u8 *)&sh, sizeof(sh))) {
        tls_close(id);
        return -1;
    }

    if (!tls_derive_keys(c, &ch, &sh)) {
        tls_close(id);
        return -1;
    }

    return id;
}

i32 tls_write(tls_conn_t conn, const void *data, u32 len) {
    u8 header[2];
    u8 nonce[TLS_IV_LEN];
    u8 cipher[TLS_MAX_RECORD];
    u8 tag[TLS_TAG_LEN];
    struct tls_conn_state *c;

    if (!tls_valid(conn) || !data)
        return -1;
    c = &tls_conns[conn];
    if (!c->established)
        return -1;
    if (len == 0 || len > TLS_MAX_RECORD)
        return -1;

    store_be16(header, (u16)len);
    tls_make_nonce(nonce, c->tx_iv, c->tx_seq);
    if (!aes_gcm_encrypt(&c->tx_gcm, nonce, sizeof(nonce),
                         header, sizeof(header),
                         (const u8 *)data, len, cipher, tag))
        return -1;

    if (!tcp_write_all(c->tcp, header, sizeof(header)) ||
        !tcp_write_all(c->tcp, cipher, len) ||
        !tcp_write_all(c->tcp, tag, sizeof(tag)))
        return -1;

    c->tx_seq++;
    return (i32)len;
}

i32 tls_read(tls_conn_t conn, void *buf, u32 len) {
    u8 header[2];
    u8 nonce[TLS_IV_LEN];
    u8 cipher[TLS_MAX_RECORD];
    u8 plain[TLS_MAX_RECORD];
    u8 tag[TLS_TAG_LEN];
    struct tls_conn_state *c;
    u16 frame_len;

    if (!tls_valid(conn) || !buf)
        return -1;
    c = &tls_conns[conn];
    if (!c->established)
        return -1;

    if (!tcp_read_all(c->tcp, header, sizeof(header)))
        return -1;
    frame_len = load_be16(header);
    if (frame_len == 0 || frame_len > TLS_MAX_RECORD || frame_len > len)
        return -1;

    if (!tcp_read_all(c->tcp, cipher, frame_len) ||
        !tcp_read_all(c->tcp, tag, sizeof(tag)))
        return -1;

    tls_make_nonce(nonce, c->rx_iv, c->rx_seq);
    if (!aes_gcm_decrypt(&c->rx_gcm, nonce, sizeof(nonce),
                         header, sizeof(header),
                         cipher, frame_len, plain, tag))
        return -1;

    simd_memcpy(buf, plain, frame_len);
    c->rx_seq++;
    return (i32)frame_len;
}

void tls_close(tls_conn_t conn) {
    if (!tls_valid(conn))
        return;

    if (tls_conns[conn].tcp >= 0)
        tcp_close(tls_conns[conn].tcp);

    simd_zero(&tls_conns[conn], sizeof(tls_conns[conn]));
}

void tls_init(void) {
    simd_zero(tls_conns, sizeof(tls_conns));
}
