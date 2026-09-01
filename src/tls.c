#include "tls.h"
#include "crypto.h"
#include "keystore.h"
#include "picotlsserver.h"
#include "p256.h"
#include "random.h"
#include "simd.h"
#include "timer.h"
#include "tls13_client.h"
#include "tls13_handshake.h"
#include "tls13_record.h"
#include "tls13_verify.h"
#include "x509.h"

#define TLS_CONN_INDEX_BITS 3U
#define TLS_CONN_INDEX_MASK ((1U << TLS_CONN_INDEX_BITS) - 1U)
#define TLS_CONN_GENERATION_MASK 0x0fffffffU
#define TLS_COMPAT_MAX_STEPS 32U

enum tls_role {
    TLS_ROLE_NONE = 0,
    TLS_ROLE_CLIENT = 1,
    TLS_ROLE_SERVER = 2
};

union tls_protocol_state {
    picotlsserver_t server;
    tls13_client_t client;
};

struct tls_conn_state {
    union tls_protocol_state protocol;
    tcp_conn_t tcp;
    u64 deadline_ms;
    u32 generation;
    u32 random_counter;
    u32 last_error;
    u8 role;
    u8 state;
    bool in_use;
};

static struct tls_conn_state tls_conns[TLS_MAX_CONNECTIONS];
static struct tls_diag_snapshot tls_diag;
static u8 tls_random_seed[32];
static bool tls_random_seed_ready;

static void secure_zero(void *ptr, u32 len)
{
    volatile u8 *p = (volatile u8 *)ptr;
    while (len--) *p++ = 0;
}

static tls_conn_t tls_conn_make(u32 slot, u32 generation)
{
    if (slot >= TLS_MAX_CONNECTIONS || generation == 0U ||
        generation > TLS_CONN_GENERATION_MASK)
        return TLS_CONN_INVALID;
    return (tls_conn_t)((generation << TLS_CONN_INDEX_BITS) | slot);
}

static bool tls_conn_decode(tls_conn_t conn, u32 *slot, u32 *generation)
{
    if (conn < 0) return false;
    u32 raw = (u32)conn;
    u32 decoded_slot = raw & TLS_CONN_INDEX_MASK;
    u32 decoded_generation =
        (raw >> TLS_CONN_INDEX_BITS) & TLS_CONN_GENERATION_MASK;
    if (decoded_slot >= TLS_MAX_CONNECTIONS || decoded_generation == 0U)
        return false;
    if (slot) *slot = decoded_slot;
    if (generation) *generation = decoded_generation;
    return true;
}

static struct tls_conn_state *tls_lookup(tls_conn_t conn)
{
    u32 slot;
    u32 generation;
    if (!tls_conn_decode(conn, &slot, &generation))
        return NULL;
    struct tls_conn_state *c = &tls_conns[slot];
    if (!c->in_use || c->generation != generation)
        return NULL;
    return c;
}

static tls_conn_t tls_alloc(void)
{
    for (u32 slot = 0; slot < TLS_MAX_CONNECTIONS; slot++) {
        struct tls_conn_state *c = &tls_conns[slot];
        if (c->in_use) continue;
        u32 generation = (c->generation + 1U) &
                         TLS_CONN_GENERATION_MASK;
        if (generation == 0U) generation = 1U;
        simd_zero(c, sizeof(*c));
        c->tcp = TCP_CONN_INVALID;
        c->generation = generation;
        c->in_use = true;
        c->state = TLS_STATE_HANDSHAKE;
        return tls_conn_make(slot, generation);
    }
    return TLS_CONN_INVALID;
}

static struct tls_result tls_result_make(i32 status, u32 bytes, u32 error)
{
    struct tls_result out = {
        .status = status,
        .bytes = bytes,
        .error = error,
    };
    return out;
}

static u32 map_protocol_error(u32 error)
{
    switch (error) {
    case PICOTLSSERVER_OK: return TLS_ERR_NONE;
    case PICOTLSSERVER_ERR_IO: return TLS_ERR_IO;
    case PICOTLSSERVER_ERR_MALFORMED: return TLS_ERR_MALFORMED;
    case PICOTLSSERVER_ERR_KEY: return TLS_ERR_KEY;
    case PICOTLSSERVER_ERR_DECRYPT: return TLS_ERR_DECRYPT;
    case PICOTLSSERVER_ERR_RECORD: return TLS_ERR_RECORD;
    case PICOTLSSERVER_ERR_UNSUPPORTED: return TLS_ERR_UNSUPPORTED;
    case PICOTLSSERVER_ERR_CERTIFICATE: return TLS_ERR_CERTIFICATE;
    case PICOTLSSERVER_ERR_FINISHED: return TLS_ERR_FINISHED;
    case PICOTLSSERVER_ERR_BUSY: return TLS_ERR_BUSY;
    case PICOTLSSERVER_ERR_CANCELLED: return TLS_ERR_CANCELLED;
    case TLS13_CLIENT_ERR_PIN_MISMATCH: return TLS_ERR_PIN_MISMATCH;
    default: return TLS_ERR_RECORD;
    }
}

static struct tls_result stale_result(void)
{
    tls_diag.stale_handles++;
    tls_diag.last_error = TLS_ERR_STALE_HANDLE;
    return tls_result_make(TLS_STEP_ERROR, 0U, TLS_ERR_STALE_HANDLE);
}

static struct tls_result connection_fail(struct tls_conn_state *c,
                                         u32 error)
{
    if (c->state != TLS_STATE_ERROR) {
        tls_diag.handshake_failures +=
            c->state == TLS_STATE_HANDSHAKE;
        if (error == TLS_ERR_DECRYPT)
            tls_diag.decrypt_failures++;
    }
    c->state = TLS_STATE_ERROR;
    c->last_error = error;
    tls_diag.last_error = error;
    return tls_result_make(TLS_STEP_ERROR, 0U, error);
}

static i32 pios_tls_read_some(void *ctx, u8 *out, u32 cap)
{
    struct tls_conn_state *c = (struct tls_conn_state *)ctx;
    if (!c || !out || cap == 0U) return -1;
    u32 got = tcp_read(c->tcp, out, cap);
    if (got != 0U) return (i32)got;
    u32 state = tcp_state(c->tcp);
    if (state == TCP_CLOSED || state == TCP_CLOSE_WAIT ||
        state >= TCP_CLOSING)
        return -1;
    return 0;
}

static i32 pios_tls_write_some(void *ctx, const u8 *data, u32 len)
{
    struct tls_conn_state *c = (struct tls_conn_state *)ctx;
    if (!c || !data || len == 0U) return -1;
    u32 state = tcp_state(c->tcp);
    if (state == TCP_CLOSED || state == TCP_CLOSE_WAIT ||
        state >= TCP_CLOSING)
        return -1;
    return (i32)tcp_write(c->tcp, data, len);
}

static bool pios_tls_random(void *ctx, u8 *out, u32 len)
{
    struct tls_conn_state *c = (struct tls_conn_state *)ctx;
    if (!c || !out) return false;
    if (crypto_random_bytes(out, len))
        return true;
    if (!tls_random_seed_ready)
        return false;

    u32 off = 0U;
    while (off < len) {
        u8 input[24];
        u64 ticks = timer_ticks();
        for (u32 i = 0; i < 8U; i++)
            input[i] = (u8)(ticks >> (i * 8U));
        u64 deadline = c->deadline_ms;
        for (u32 i = 0; i < 8U; i++)
            input[8U + i] = (u8)(deadline >> (i * 8U));
        u32 counter = ++c->random_counter;
        input[16] = (u8)counter;
        input[17] = (u8)(counter >> 8);
        input[18] = (u8)(counter >> 16);
        input[19] = (u8)(counter >> 24);
        input[20] = (u8)c->generation;
        input[21] = (u8)(c->generation >> 8);
        input[22] = (u8)c->tcp;
        input[23] = (u8)((u32)c->tcp >> 8);
        u8 block[32];
        hmac_sha256(tls_random_seed, sizeof(tls_random_seed),
                    input, sizeof(input), block);
        u32 take = len - off;
        if (take > sizeof(block)) take = sizeof(block);
        simd_memcpy(out + off, block, take);
        secure_zero(block, sizeof(block));
        off += take;
    }
    return true;
}

static bool pios_tls_identity(void *ctx, const u8 **cert_der,
                              u32 *cert_len,
                              u8 p256_private_scalar[32])
{
    (void)ctx;
    if (!cert_der || !cert_len || !p256_private_scalar)
        return false;
    *cert_der = x509_certificate_der(cert_len);
    return *cert_der && *cert_len != 0U &&
           x509_p256_private_scalar(p256_private_scalar);
}

static struct picotls_io_ops pios_tls_ops(struct tls_conn_state *c)
{
    struct picotls_io_ops ops = {
        .ctx = c,
        .read_some = pios_tls_read_some,
        .write_some = pios_tls_write_some,
        .random_bytes = pios_tls_random,
        .server_identity = pios_tls_identity,
    };
    return ops;
}

tls_conn_t tls_client_start(tcp_conn_t tcp,
                            const struct tls_client_start_config *config)
{
    if (tcp < 0 || !config || config->deadline_ms == 0ULL ||
        config->server_name_len > TLS13_CLIENT_SERVER_NAME_MAX ||
        (config->server_name_len && !config->server_name) ||
        (config->pinned_p256_public_key &&
         p256_pubkey_validate(config->pinned_p256_public_key) != 0)) {
        tls_diag.last_error = TLS_ERR_MALFORMED;
        return TLS_CONN_INVALID;
    }
    tls_diag.connect_attempts++;
    tls_conn_t handle = tls_alloc();
    if (handle < 0) {
        tls_diag.handshake_failures++;
        tls_diag.last_error = TLS_ERR_ALLOC;
        return TLS_CONN_INVALID;
    }
    struct tls_conn_state *c = tls_lookup(handle);
    c->tcp = tcp;
    c->deadline_ms = config->deadline_ms;
    c->role = TLS_ROLE_CLIENT;
    tls13_client_init(&c->protocol.client);
    struct tls13_client_config client_config = {
        .server_name = config->server_name,
        .server_name_len = config->server_name_len,
        .pinned_p256_public_key = config->pinned_p256_public_key,
    };
    if (!tls13_client_start(&c->protocol.client, &client_config)) {
        (void)connection_fail(c, TLS_ERR_MALFORMED);
        return handle;
    }
    return handle;
}

tls_conn_t tls_server_start(tcp_conn_t tcp, u64 deadline_ms)
{
    if (tcp < 0 || deadline_ms == 0ULL)
        return TLS_CONN_INVALID;
    tls_diag.accept_attempts++;
    tls_conn_t handle = tls_alloc();
    if (handle < 0) {
        tls_diag.handshake_failures++;
        tls_diag.last_error = TLS_ERR_ALLOC;
        return TLS_CONN_INVALID;
    }
    struct tls_conn_state *c = tls_lookup(handle);
    c->tcp = tcp;
    c->deadline_ms = deadline_ms;
    c->role = TLS_ROLE_SERVER;
    picotlsserver_init(&c->protocol.server);
    if (!picotlsserver_start(&c->protocol.server))
        (void)connection_fail(c, TLS_ERR_MALFORMED);
    return handle;
}

struct tls_result tls_handshake_step(tls_conn_t conn, u64 now_ms)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) return stale_result();
    if (c->state == TLS_STATE_ESTABLISHED)
        return tls_result_make(TLS_STEP_DONE, 0U, TLS_ERR_NONE);
    if (c->state == TLS_STATE_ERROR ||
        c->state == TLS_STATE_CANCELLED)
        return tls_result_make(TLS_STEP_ERROR, 0U, c->last_error);
    if (now_ms >= c->deadline_ms) {
        tls_diag.timeouts++;
        if (c->role == TLS_ROLE_CLIENT)
            tls13_client_cancel(&c->protocol.client);
        else
            picotlsserver_cancel(&c->protocol.server);
        return connection_fail(c, TLS_ERR_TIMEOUT);
    }

    struct picotls_io_ops ops = pios_tls_ops(c);
    struct picotls_result step;
    if (c->role == TLS_ROLE_CLIENT) {
        step = tls13_client_handshake_step(&c->protocol.client, &ops);
    } else if (c->role == TLS_ROLE_SERVER) {
        step = picotlsserver_handshake_step(&c->protocol.server, &ops);
    } else {
        return connection_fail(c, TLS_ERR_MALFORMED);
    }

    if (step.status == PICOTLS_STEP_ERROR)
        return connection_fail(c, map_protocol_error(step.error));
    if (step.status == PICOTLS_STEP_DONE) {
        c->state = TLS_STATE_ESTABLISHED;
        c->last_error = TLS_ERR_NONE;
        tls_diag.handshakes_ok++;
        tls_diag.last_error = TLS_ERR_NONE;
        return tls_result_make(TLS_STEP_DONE, step.bytes, TLS_ERR_NONE);
    }
    tls_diag.pending_steps++;
    if (step.bytes == 0U) tls_diag.backpressure++;
    return tls_result_make(TLS_STEP_PENDING, step.bytes, TLS_ERR_NONE);
}

struct tls_result tls_write_step(tls_conn_t conn,
                                 const void *data, u32 len)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) return stale_result();
    if (c->state != TLS_STATE_ESTABLISHED)
        return tls_result_make(TLS_STEP_ERROR, 0U,
                               c->last_error ? c->last_error :
                                               TLS_ERR_BUSY);
    struct picotls_io_ops ops = pios_tls_ops(c);
    struct picotls_result step =
        c->role == TLS_ROLE_CLIENT
            ? tls13_client_write_step(&c->protocol.client, &ops,
                                      data, len)
            : picotlsserver_write_step(&c->protocol.server, &ops,
                                       data, len);
    if (step.status == PICOTLS_STEP_ERROR)
        return connection_fail(c, map_protocol_error(step.error));
    if (step.status == PICOTLS_STEP_DONE) {
        tls_diag.records_tx++;
        return tls_result_make(TLS_STEP_DONE, step.bytes, TLS_ERR_NONE);
    }
    tls_diag.pending_steps++;
    if (step.bytes == 0U) tls_diag.backpressure++;
    return tls_result_make(TLS_STEP_PENDING, step.bytes, TLS_ERR_NONE);
}

struct tls_result tls_read_step(tls_conn_t conn, void *buf, u32 len)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) return stale_result();
    if (c->state != TLS_STATE_ESTABLISHED)
        return tls_result_make(TLS_STEP_ERROR, 0U,
                               c->last_error ? c->last_error :
                                               TLS_ERR_BUSY);
    struct picotls_io_ops ops = pios_tls_ops(c);
    struct picotls_result step =
        c->role == TLS_ROLE_CLIENT
            ? tls13_client_read_step(&c->protocol.client, &ops,
                                     buf, len)
            : picotlsserver_read_step(&c->protocol.server, &ops,
                                      buf, len);
    if (step.status == PICOTLS_STEP_ERROR)
        return connection_fail(c, map_protocol_error(step.error));
    if (step.status == PICOTLS_STEP_DONE) {
        if (step.bytes != 0U) tls_diag.records_rx++;
        return tls_result_make(TLS_STEP_DONE, step.bytes, TLS_ERR_NONE);
    }
    tls_diag.pending_steps++;
    return tls_result_make(TLS_STEP_PENDING, step.bytes, TLS_ERR_NONE);
}

struct tls_result tls_close_step(tls_conn_t conn)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) return stale_result();
    if (c->state == TLS_STATE_ERROR ||
        c->state == TLS_STATE_CANCELLED)
        return tls_result_make(TLS_STEP_ERROR, 0U, c->last_error);

    struct picotls_io_ops ops = pios_tls_ops(c);
    struct picotls_result step =
        c->role == TLS_ROLE_CLIENT
            ? tls13_client_close_step(&c->protocol.client, &ops)
            : picotlsserver_close_step(&c->protocol.server, &ops);
    if (step.status == PICOTLS_STEP_ERROR) {
        u32 error = map_protocol_error(step.error);
        (void)connection_fail(c, error);
        return tls_result_make(TLS_STEP_ERROR, 0U, error);
    }
    if (step.status == PICOTLS_STEP_PENDING ||
        (c->tcp >= 0 && tcp_tx_pending(c->tcp) != 0U)) {
        tls_diag.pending_steps++;
        if (step.bytes == 0U) tls_diag.backpressure++;
        return tls_result_make(TLS_STEP_PENDING, step.bytes, TLS_ERR_NONE);
    }
    tls_cancel(conn, true);
    return tls_result_make(TLS_STEP_DONE, 0U, TLS_ERR_NONE);
}

bool tls_established(tls_conn_t conn)
{
    struct tls_conn_state *c = tls_lookup(conn);
    return c && c->state == TLS_STATE_ESTABLISHED;
}

bool tls_peer_closed(tls_conn_t conn)
{
    struct tls_conn_state *c = tls_lookup(conn);
    return c && ((c->role == TLS_ROLE_CLIENT &&
                  tls13_client_peer_closed(&c->protocol.client)) ||
                 (c->role == TLS_ROLE_SERVER &&
                  picotlsserver_peer_closed(&c->protocol.server)));
}

u32 tls_state(tls_conn_t conn)
{
    struct tls_conn_state *c = tls_lookup(conn);
    return c ? c->state : TLS_STATE_FREE;
}

u32 tls_last_error(tls_conn_t conn)
{
    struct tls_conn_state *c = tls_lookup(conn);
    return c ? c->last_error : TLS_ERR_STALE_HANDLE;
}

void tls_cancel(tls_conn_t conn, bool close_tcp)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) {
        tls_diag.stale_handles++;
        return;
    }
    tcp_conn_t tcp = c->tcp;
    u32 generation = c->generation;
    if (c->role == TLS_ROLE_CLIENT)
        tls13_client_cancel(&c->protocol.client);
    else if (c->role == TLS_ROLE_SERVER)
        picotlsserver_cancel(&c->protocol.server);
    simd_zero(c, sizeof(*c));
    c->tcp = TCP_CONN_INVALID;
    c->generation = generation;
    if (close_tcp && tcp >= 0)
        tcp_close(tcp);
    tls_diag.cancels++;
    tls_diag.closes++;
}

tls_conn_t tls_connect(tcp_conn_t tcp)
{
    struct tls_client_start_config config = {
        .server_name = NULL,
        .server_name_len = 0U,
        .pinned_p256_public_key = NULL,
        .deadline_ms = timer_monotonic_ms() +
                       TLS_DEFAULT_HANDSHAKE_TIMEOUT_MS,
    };
    return tls_client_start(tcp, &config);
}

static struct tls_result tls_compat_handshake_pump(tls_conn_t conn)
{
    struct tls_result step =
        tls_result_make(TLS_STEP_PENDING, 0U, TLS_ERR_NONE);
    for (u32 i = 0; i < TLS_COMPAT_MAX_STEPS; i++) {
        step = tls_handshake_step(conn, timer_monotonic_ms());
        if (step.status != TLS_STEP_PENDING)
            break;
    }
    return step;
}

tls_conn_t tls_accept(tcp_conn_t tcp)
{
    tls_conn_t conn = tls_server_start(
        tcp, timer_monotonic_ms() + TLS_DEFAULT_HANDSHAKE_TIMEOUT_MS);
    if (conn < 0) return conn;
    struct tls_result step = tls_compat_handshake_pump(conn);
    if (step.status == TLS_STEP_ERROR) {
        tls_cancel(conn, true);
        return TLS_CONN_INVALID;
    }
    return conn;
}

i32 tls_write(tls_conn_t conn, const void *data, u32 len)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) return -1;
    if (c->state == TLS_STATE_HANDSHAKE) {
        struct tls_result hs = tls_compat_handshake_pump(conn);
        if (hs.status == TLS_STEP_ERROR) return -1;
        if (hs.status == TLS_STEP_PENDING) return 0;
    }
    struct tls_result step = tls_write_step(conn, data, len);
    return step.status == TLS_STEP_ERROR ? -1 :
           step.status == TLS_STEP_PENDING ? 0 : (i32)step.bytes;
}

i32 tls_read(tls_conn_t conn, void *buf, u32 len)
{
    struct tls_conn_state *c = tls_lookup(conn);
    if (!c) return -1;
    if (c->state == TLS_STATE_HANDSHAKE) {
        struct tls_result hs = tls_compat_handshake_pump(conn);
        if (hs.status == TLS_STEP_ERROR) return -1;
        if (hs.status == TLS_STEP_PENDING) return 0;
    }
    struct tls_result step = tls_read_step(conn, buf, len);
    return step.status == TLS_STEP_ERROR ? -1 :
           step.status == TLS_STEP_PENDING ? 0 : (i32)step.bytes;
}

void tls_close(tls_conn_t conn)
{
    tls_cancel(conn, true);
}

void tls_init(void)
{
    simd_zero(tls_conns, sizeof(tls_conns));
    simd_zero(&tls_diag, sizeof(tls_diag));
    simd_zero(tls_random_seed, sizeof(tls_random_seed));
    tls_random_seed_ready = keystore_derive_secret(
        "tls13-ephemeral-drbg-v1", tls_random_seed,
        sizeof(tls_random_seed));
}

void tls_diag_snapshot(struct tls_diag_snapshot *out)
{
    if (!out) return;
    *out = tls_diag;
    out->active = 0U;
    out->established = 0U;
    for (u32 i = 0; i < TLS_MAX_CONNECTIONS; i++) {
        if (tls_conns[i].in_use) out->active++;
        if (tls_conns[i].in_use &&
            tls_conns[i].state == TLS_STATE_ESTABLISHED)
            out->established++;
    }
}

bool tls_selftest(void)
{
    tls_diag.selftests++;
    if (!crypto_selftest() || !tls13_record_selftest()) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_RECORD;
        return false;
    }

    u8 scalar[32];
    simd_zero(scalar, sizeof(scalar));
    scalar[31] = 7U;
    u8 public_key[65];
    if (p256_derive_pubkey(scalar, public_key) != 0) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_KEY;
        return false;
    }
    u8 hash[32];
    for (u32 i = 0; i < sizeof(hash); i++) hash[i] = (u8)(i * 7U + 3U);
    u8 cv[128];
    u32 cv_len = tls13_build_certificate_verify_p256(
        scalar, hash, cv, sizeof(cv));
    u8 message_type;
    u32 body_len;
    u16 scheme;
    const u8 *signature;
    u32 signature_len;
    if (cv_len == 0U ||
        !tls13_next_handshake_header(cv, cv_len,
                                     &message_type, &body_len) ||
        message_type != TLS13_HS_CERTIFICATE_VERIFY ||
        !tls13_parse_certificate_verify(cv + 4U, body_len, &scheme,
                                        &signature, &signature_len) ||
        scheme != TLS13_SIGALG_ECDSA_SECP256R1_SHA256 ||
        !tls13_verify_server_certificate_signature(
            public_key, hash, signature, signature_len)) {
        tls_diag.selftest_failures++;
        tls_diag.last_error = TLS_ERR_CERTIFICATE;
        return false;
    }
    tls_diag.last_error = TLS_ERR_NONE;
    return true;
}

i32 tls_bridge_parse_request(const u8 *plain, u32 len,
                             struct tls_bridge_request *out)
{
    if (!plain || !out || len == 0U) {
        tls_diag.bridge_parse_error++;
        return TLS_BRIDGE_ERROR;
    }
    u32 line_end = 0U;
    while (line_end + 1U < len &&
           !(plain[line_end] == '\r' &&
             plain[line_end + 1U] == '\n'))
        line_end++;
    if (line_end + 1U >= len) {
        tls_diag.bridge_parse_need_more++;
        return TLS_BRIDGE_NEED_MORE;
    }
    u32 sp1 = 0U;
    while (sp1 < line_end && plain[sp1] != ' ') sp1++;
    u32 sp2 = sp1 + 1U;
    while (sp2 < line_end && plain[sp2] != ' ') sp2++;
    if (sp1 == 0U || sp1 >= line_end || sp2 <= sp1 + 1U ||
        sp2 >= line_end) {
        tls_diag.bridge_parse_error++;
        return TLS_BRIDGE_ERROR;
    }
    u32 method_len = sp1;
    if (method_len >= sizeof(out->method))
        method_len = sizeof(out->method) - 1U;
    for (u32 i = 0; i < method_len; i++)
        out->method[i] = (char)plain[i];
    out->method[method_len] = 0;
    u32 path_len = sp2 - sp1 - 1U;
    if (path_len >= sizeof(out->path))
        path_len = sizeof(out->path) - 1U;
    for (u32 i = 0; i < path_len; i++)
        out->path[i] = (char)plain[sp1 + 1U + i];
    out->path[path_len] = 0;
    out->header_bytes = 0U;
    for (u32 i = 0; i + 3U < len; i++) {
        if (plain[i] == '\r' && plain[i + 1U] == '\n' &&
            plain[i + 2U] == '\r' && plain[i + 3U] == '\n') {
            out->header_bytes = i + 4U;
            tls_diag.bridge_parse_ok++;
            return TLS_BRIDGE_OK;
        }
    }
    tls_diag.bridge_parse_need_more++;
    return TLS_BRIDGE_NEED_MORE;
}
