#include "tls.h"
#include "random.h"
#include "keystore.h"
#include "x509.h"
#include <stdio.h>
#include <string.h>

static int failures;
static u32 tcp_close_calls;
static u64 fake_ticks;

#define CHECK(condition, message) do { \
    if (condition) printf("[PASS] %s\n", message); \
    else { printf("[FAIL] %s\n", message); failures++; } \
} while (0)

u32 tcp_read(tcp_conn_t conn, void *data, u32 len)
{
    (void)conn;
    (void)data;
    (void)len;
    return 0U;
}

u32 tcp_write(tcp_conn_t conn, const void *data, u32 len)
{
    (void)conn;
    (void)data;
    (void)len;
    return 0U;
}

u32 tcp_state(tcp_conn_t conn)
{
    (void)conn;
    return TCP_ESTABLISHED;
}

u32 tcp_tx_pending(tcp_conn_t conn)
{
    (void)conn;
    return 0U;
}

void tcp_close(tcp_conn_t conn)
{
    (void)conn;
    tcp_close_calls++;
}

u64 timer_ticks(void)
{
    return ++fake_ticks;
}

u64 timer_monotonic_ms(void)
{
    return fake_ticks;
}

bool crypto_random_bytes(void *out, u32 len)
{
    memset(out, 0x5a, len);
    return true;
}

bool keystore_derive_secret(const char *label, u8 *out, u32 out_len)
{
    (void)label;
    memset(out, 0xa5, out_len);
    return true;
}

const u8 *x509_certificate_der(u32 *len)
{
    if (len) *len = 0U;
    return NULL;
}

bool x509_p256_private_scalar(u8 out[32])
{
    memset(out, 0, 32U);
    return false;
}

int main(void)
{
    tls_init();

    tls_conn_t first = tls_server_start(1, 100U);
    CHECK(first >= 0 && tls_state(first) == TLS_STATE_HANDSHAKE,
          "server start allocates a live handshake handle");

    struct tls_result timeout = tls_handshake_step(first, 100U);
    CHECK(timeout.status == TLS_STEP_ERROR &&
          timeout.error == TLS_ERR_TIMEOUT &&
          tls_state(first) == TLS_STATE_ERROR,
          "deadline expiry is an explicit terminal timeout");

    tls_cancel(first, false);
    CHECK(tls_state(first) == TLS_STATE_FREE &&
          tls_last_error(first) == TLS_ERR_STALE_HANDLE,
          "cancel invalidates the released generation");
    CHECK(tcp_close_calls == 0U,
          "cancel can release TLS ownership without closing TCP");

    tls_conn_t second = tls_server_start(2, 200U);
    CHECK(second >= 0 && second != first,
          "slot reuse bumps the TLS handle generation");
    struct tls_result stale = tls_handshake_step(first, 0U);
    CHECK(stale.status == TLS_STEP_ERROR &&
          stale.error == TLS_ERR_STALE_HANDLE,
          "stale TLS handles fail closed after slot reuse");

    tls_close(second);
    CHECK(tcp_close_calls == 1U && tls_state(second) == TLS_STATE_FREE,
          "tls_close cancels state and closes the owned TCP connection");

    struct tls_diag_snapshot diag;
    tls_diag_snapshot(&diag);
    CHECK(diag.timeouts == 1U && diag.stale_handles >= 1U &&
          diag.cancels == 2U,
          "timeout, stale-handle, and cancellation diagnostics advance");

    if (failures == 0) {
        printf("test_tls_api: ALL PASS\n");
        return 0;
    }
    printf("test_tls_api: %d FAILURE(S)\n", failures);
    return 1;
}
