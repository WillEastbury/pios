#include "tls.h"
#include "crypto.h"
#include "keystore.h"
#include "net.h"
#include "principal.h"
#include "simd.h"
#include "timer.h"
#include "picotlsserver.h"
#include "x509.h"

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

    /* Real RFC 8446 TLS 1.3 server path (tls_accept() only -- tls_connect()
     * still uses the fake CHLO/SHLO exchange above; see the "Next steps"
     * note in AGENTS.md about eventually porting the client role too). */
    bool is_real_tls13;
    picotlsserver_t tls13_server;
};

static struct tls_conn_state tls_conns[TLS_MAX_CONNECTIONS];
static struct tls_diag_snapshot tls_diag;

#define TLS_ERR_NONE        0U
#define TLS_ERR_ALLOC       1U
#define TLS_ERR_IO          2U
#define TLS_ERR_MAGIC       3U
#define TLS_ERR_KEY_DERIVE  4U
#define TLS_ERR_DECRYPT     5U
#define TLS_ERR_RECORD      6U

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
        net_poll();
        timer_delay_ms(1);
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
        net_poll();
        timer_delay_ms(1);
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

static void secure_zero(void *p, u32 n)
{
    volatile u8 *v = (volatile u8 *)p;
    while (n--) *v++ = 0;
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
    u8 transcript[64];
    u8 transcript_hash[32];
    u8 prk[32];
    u8 keymat[(TLS_KEY_LEN + TLS_IV_LEN) * 2];
    u8 psk[32];
    u32 pid = principal_current();

    if (!c || !ch || !sh) return false;

    simd_memcpy(transcript, ch->random, 32);
    simd_memcpy(transcript + 32, sh->random, 32);
    sha256(transcript, sizeof(transcript), transcript_hash);

    if (!principal_tls_psk(pid, psk, sizeof(psk)) &&
        !keystore_derive_secret("kernel-tls-psk-v1", psk, sizeof(psk))) {
        secure_zero(transcript, sizeof(transcript));
        secure_zero(transcript_hash, sizeof(transcript_hash));
        secure_zero(psk, sizeof(psk));
        return false;
    }
    hkdf_extract(psk, sizeof(psk), transcript_hash, sizeof(transcript_hash), prk);
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
    secure_zero(transcript, sizeof(transcript));
    secure_zero(transcript_hash, sizeof(transcript_hash));
    secure_zero(prk, sizeof(prk));
    secure_zero(keymat, sizeof(keymat));
    secure_zero(psk, sizeof(psk));
    return true;
}

tls_conn_t tls_connect(tcp_conn_t tcp) {
    tls_conn_t id;
    struct tls_conn_state *c;
    struct tls_handshake ch;
    struct tls_handshake sh;

    if (tcp < 0) return -1;
    tls_diag.connect_attempts++;
    id = tls_alloc();
    if (id < 0) {
        tls_diag.last_error = TLS_ERR_ALLOC;
        tls_diag.handshake_failures++;
        return -1;
    }

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
        tls_diag.last_error = TLS_ERR_IO;
        tls_diag.handshake_failures++;
        tls_close(id);
        return -1;
    }
    if (!tcp_read_all(tcp, (u8 *)&sh, sizeof(sh))) {
        tls_diag.last_error = TLS_ERR_IO;
        tls_diag.handshake_failures++;
        tls_close(id);
        return -1;
    }
    if (sh.magic != TLS_MAGIC_SHLO || sh.version != TLS_HS_VER) {
        tls_diag.last_error = TLS_ERR_MAGIC;
        tls_diag.handshake_failures++;
        tls_close(id);
        return -1;
    }

    if (!tls_derive_keys(c, &ch, &sh)) {
        tls_diag.last_error = TLS_ERR_KEY_DERIVE;
        tls_diag.handshake_failures++;
        tls_close(id);
        return -1;
    }

    tls_diag.handshakes_ok++;
    tls_diag.last_error = TLS_ERR_NONE;
    return id;
}

static bool pios_tls13_read_exact(void *ctx, u8 *out, u32 len)
{
    struct tls_conn_state *c = (struct tls_conn_state *)ctx;
    return c && tcp_read_all(c->tcp, out, len);
}

static bool pios_tls13_write_all(void *ctx, const u8 *data, u32 len)
{
    struct tls_conn_state *c = (struct tls_conn_state *)ctx;
    return c && tcp_write_all(c->tcp, data, len);
}

static bool pios_tls13_random(void *ctx, u8 *out, u32 len)
{
    (void)ctx;
    if (!out)
        return false;
    for (u32 off = 0; off < len; off += 4U) {
        u32 value = store_rand32();
        u32 take = len - off;
        if (take > 4U)
            take = 4U;
        for (u32 i = 0; i < take; i++)
            out[off + i] = (u8)(value >> (i * 8U));
    }
    return true;
}

static bool pios_tls13_identity(void *ctx, const u8 **cert_der, u32 *cert_len,
                                u8 p256_private_scalar[32])
{
    (void)ctx;
    if (!cert_der || !cert_len || !p256_private_scalar)
        return false;
    *cert_der = x509_certificate_der(cert_len);
    return *cert_der && *cert_len != 0 &&
           x509_p256_private_scalar(p256_private_scalar);
}

static struct picotlsserver_ops pios_tls13_ops(struct tls_conn_state *c)
{
    struct picotlsserver_ops ops = {
        .ctx = c,
        .read_exact = pios_tls13_read_exact,
        .write_all = pios_tls13_write_all,
        .random_bytes = pios_tls13_random,
        .identity = pios_tls13_identity,
    };
    return ops;
}

tls_conn_t tls_accept(tcp_conn_t tcp) {
    tls_conn_t id;
    struct tls_conn_state *c;

    if (tcp < 0) return -1;
    tls_diag.accept_attempts++;
    id = tls_alloc();
    if (id < 0) {
        tls_diag.last_error = TLS_ERR_ALLOC;
        tls_diag.handshake_failures++;
        return -1;
    }

    c = &tls_conns[id];
    c->tcp = tcp;
    c->is_client = false;
    picotlsserver_init(&c->tls13_server);

    struct picotlsserver_ops ops = pios_tls13_ops(c);
    if (!picotlsserver_accept(&c->tls13_server, &ops)) {
        tls_diag.last_error = picotlsserver_last_error(&c->tls13_server);
        if (tls_diag.last_error == PICOTLSSERVER_ERR_DECRYPT)
            tls_diag.decrypt_failures++;
        tls_diag.handshake_failures++;
        tls_close(id);
        return -1;
    }

    c->is_real_tls13 = true;
    c->established = true;
    tls_diag.handshakes_ok++;
    tls_diag.last_error = TLS_ERR_NONE;
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

    if (c->is_real_tls13) {
        struct picotlsserver_ops ops = pios_tls13_ops(c);
        i32 written = picotlsserver_write(&c->tls13_server, &ops, data, len);
        if (written < 0) {
            tls_diag.last_error = picotlsserver_last_error(&c->tls13_server);
            return -1;
        }
        tls_diag.records_tx++;
        return written;
    }

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
    tls_diag.records_tx++;
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

    if (c->is_real_tls13) {
        struct picotlsserver_ops ops = pios_tls13_ops(c);
        i32 received = picotlsserver_read(&c->tls13_server, &ops, buf, len);
        if (received < 0) {
            tls_diag.last_error = picotlsserver_last_error(&c->tls13_server);
            if (tls_diag.last_error == PICOTLSSERVER_ERR_DECRYPT)
                tls_diag.decrypt_failures++;
            return -1;
        }
        if (received > 0)
            tls_diag.records_rx++;
        tls_diag.last_error = TLS_ERR_NONE;
        return received;
    }

    if (!tcp_read_all(c->tcp, header, sizeof(header)))
        return -1;
    frame_len = load_be16(header);
    if (frame_len == 0 || frame_len > TLS_MAX_RECORD || frame_len > len)
    {
        tls_diag.last_error = TLS_ERR_RECORD;
        return -1;
    }

    if (!tcp_read_all(c->tcp, cipher, frame_len) ||
        !tcp_read_all(c->tcp, tag, sizeof(tag)))
        return -1;

    tls_make_nonce(nonce, c->rx_iv, c->rx_seq);
    if (!aes_gcm_decrypt(&c->rx_gcm, nonce, sizeof(nonce),
                         header, sizeof(header),
                         cipher, frame_len, plain, tag))
    {
        tls_diag.decrypt_failures++;
        tls_diag.last_error = TLS_ERR_DECRYPT;
        return -1;
    }

    simd_memcpy(buf, plain, frame_len);
    c->rx_seq++;
    tls_diag.records_rx++;
    return (i32)frame_len;
}

void tls_close(tls_conn_t conn) {
    if (!tls_valid(conn))
        return;

    if (tls_conns[conn].tcp >= 0)
        tcp_close(tls_conns[conn].tcp);

    simd_zero(&tls_conns[conn], sizeof(tls_conns[conn]));
    tls_diag.closes++;
}

void tls_init(void) {
    simd_zero(tls_conns, sizeof(tls_conns));
    simd_zero(&tls_diag, sizeof(tls_diag));
}

void tls_diag_snapshot(struct tls_diag_snapshot *out)
{
    if (!out) return;
    *out = tls_diag;
    out->active = 0;
    out->established = 0;
    for (u32 i = 0; i < TLS_MAX_CONNECTIONS; i++) {
        if (tls_conns[i].in_use) out->active++;
        if (tls_conns[i].in_use && tls_conns[i].established) out->established++;
    }
}

bool tls_selftest(void)
{
    tls_diag.selftests++;
    if (!crypto_selftest()) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_RECORD;
        return false;
    }
    struct tls_handshake ch;
    struct tls_handshake sh;
    struct tls_conn_state client;
    struct tls_conn_state server;
    u8 header[2];
    u8 nonce[TLS_IV_LEN];
    u8 cipher[32];
    u8 tag[TLS_TAG_LEN];
    u8 plain[32];
    static const u8 msg[] = "PIOS TLS SELFTEST";
    simd_zero(&client, sizeof(client));
    simd_zero(&server, sizeof(server));
    simd_zero(&ch, sizeof(ch));
    simd_zero(&sh, sizeof(sh));
    ch.magic = TLS_MAGIC_CHLO;
    ch.version = TLS_HS_VER;
    sh.magic = TLS_MAGIC_SHLO;
    sh.version = TLS_HS_VER;
    for (u32 i = 0; i < 32; i++) {
        ch.random[i] = (u8)(0x10U + i);
        sh.random[i] = (u8)(0xA0U ^ (i * 7U));
    }
    client.is_client = true;
    server.is_client = false;
    if (!tls_derive_keys(&client, &ch, &sh) || !tls_derive_keys(&server, &ch, &sh)) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_KEY_DERIVE;
        return false;
    }
    store_be16(header, (u16)(sizeof(msg) - 1));
    tls_make_nonce(nonce, client.tx_iv, 0);
    if (!aes_gcm_encrypt(&client.tx_gcm, nonce, sizeof(nonce),
                         header, sizeof(header), msg, (u32)(sizeof(msg) - 1), cipher, tag)) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_RECORD;
        return false;
    }
    tls_make_nonce(nonce, server.rx_iv, 0);
    if (!aes_gcm_decrypt(&server.rx_gcm, nonce, sizeof(nonce),
                         header, sizeof(header), cipher, (u32)(sizeof(msg) - 1), plain, tag)) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_DECRYPT;
        return false;
    }
    for (u32 i = 0; i < sizeof(msg) - 1; i++) {
        if (plain[i] != msg[i]) {
            tls_diag.selftest_failures++;
            tls_diag.last_error = TLS_ERR_DECRYPT;
            return false;
        }
    }
    tls_diag.last_error = TLS_ERR_NONE;
    secure_zero(&client, sizeof(client));
    secure_zero(&server, sizeof(server));
    secure_zero(cipher, sizeof(cipher));
    secure_zero(tag, sizeof(tag));
    secure_zero(plain, sizeof(plain));
    return true;
}

i32 tls_bridge_parse_request(const u8 *plain, u32 len, struct tls_bridge_request *out)
{
    if (!plain || !out || len == 0) {
        tls_diag.bridge_parse_error++;
        return TLS_BRIDGE_ERROR;
    }
    u32 line_end = 0;
    while (line_end + 1 < len && !(plain[line_end] == '\r' && plain[line_end + 1] == '\n'))
        line_end++;
    if (line_end + 1 >= len) {
        tls_diag.bridge_parse_need_more++;
        return TLS_BRIDGE_NEED_MORE;
    }
    u32 sp1 = 0;
    while (sp1 < line_end && plain[sp1] != ' ') sp1++;
    u32 sp2 = sp1 + 1;
    while (sp2 < line_end && plain[sp2] != ' ') sp2++;
    if (sp1 == 0 || sp1 >= line_end || sp2 <= sp1 + 1 || sp2 >= line_end) {
        tls_diag.bridge_parse_error++;
        return TLS_BRIDGE_ERROR;
    }
    u32 ml = sp1;
    if (ml >= sizeof(out->method)) ml = sizeof(out->method) - 1;
    for (u32 i = 0; i < ml; i++) out->method[i] = (char)plain[i];
    out->method[ml] = 0;
    u32 pl = sp2 - sp1 - 1;
    if (pl >= sizeof(out->path)) pl = sizeof(out->path) - 1;
    for (u32 i = 0; i < pl; i++) out->path[i] = (char)plain[sp1 + 1 + i];
    out->path[pl] = 0;
    out->header_bytes = 0;
    for (u32 i = 0; i + 3 < len; i++) {
        if (plain[i] == '\r' && plain[i + 1] == '\n' && plain[i + 2] == '\r' && plain[i + 3] == '\n') {
            out->header_bytes = i + 4;
            tls_diag.bridge_parse_ok++;
            return TLS_BRIDGE_OK;
        }
    }
    tls_diag.bridge_parse_need_more++;
    return TLS_BRIDGE_NEED_MORE;
}
