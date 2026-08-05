/*
 * Host test for tls13_keysched.c (HKDF-Expand-Label / TLS 1.3 key
 * schedule), checked byte-for-byte against the OFFICIAL RFC 8448 "Simple
 * 1-RTT Handshake" (Section 3) trace. This is the canonical reference
 * vector set used to validate independent TLS 1.3 implementations -- the
 * exact same handshake_secret IKM (the x25519 ECDHE shared secret from
 * that trace), transcript hashes, and every intermediate secret/traffic
 * key/Finished key are reproduced here and must match exactly. (This
 * server uses P-256 ECDHE, not x25519, but the key SCHEDULE math is
 * identical regardless of which group produced the shared secret -- it's
 * opaque IKM bytes to HKDF either way.)
 */
#include "tls13_keysched.h"
#include "crypto.h"
#include <stdio.h>
#include <string.h>

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void hex_to_bytes(const char *hex, u8 *out, u32 out_len) {
    for (u32 i = 0; i < out_len; i++)
        out[i] = (u8)((hex_nibble(hex[i * 2]) << 4) | hex_nibble(hex[i * 2 + 1]));
}

static int failures = 0;

static void check(const char *name, const u8 *got, const char *expect_hex, u32 len) {
    u8 expect[64];
    hex_to_bytes(expect_hex, expect, len);
    if (memcmp(got, expect, len) == 0) {
        printf("[PASS] %s\n", name);
    } else {
        printf("[FAIL] %s\n", name);
        failures++;
    }
}

int main(void) {
    /* RFC 8448 Section 3 "Simple 1-RTT Handshake" reference bytes. */
    const char *shared_x =
        "8bd4054fb55b9d63fdfbacf9f04b9f0d35e6d63f537563efd46272900f89492d";
    const char *ch_sh_transcript_hash =
        "860c06edc07858ee8e78f0e7428c58edd6b43f2ca3e6e95f02ed063cf0e1cad8";

    u8 shared_bytes[32];
    hex_to_bytes(shared_x, shared_bytes, 32);
    u8 ch_sh_hash[32];
    hex_to_bytes(ch_sh_transcript_hash, ch_sh_hash, 32);

    struct tls13_secrets s;
    memset(&s, 0, sizeof(s));

    if (!tls13_derive_handshake_secrets(&s, shared_bytes, ch_sh_hash)) {
        printf("[FAIL] tls13_derive_handshake_secrets returned false\n");
        failures++;
    } else {
        check("early_secret", s.early_secret,
              "33ad0a1c607ec03b09e6cd9893680ce210adf300aa1f2660e1b22e10f170f92a", 32);
        check("handshake_secret", s.handshake_secret,
              "1dc826e93606aa6fdc0aadc12f741b01046aa6b99f691ed221a9f0ca043fbeac", 32);
        check("client_hs_traffic_secret", s.client_hs_traffic,
              "b3eddb126e067f35a780b3abf45e2d8f3b1a950738f52e9600746a0e27a55a21", 32);
        check("server_hs_traffic_secret", s.server_hs_traffic,
              "b67b7d690cc16c4e75e54213cb2d37b4e9c912bcded9105d42befd59d391ad38", 32);
    }

    /* Traffic-key/IV and Finished-key derivations from server_hs_traffic,
     * also given in full by the RFC trace. */
    u8 hs_key[16], hs_iv[12];
    if (!tls13_derive_traffic_keys(s.server_hs_traffic, hs_key, hs_iv)) {
        printf("[FAIL] tls13_derive_traffic_keys returned false\n");
        failures++;
    } else {
        check("server hs write key", hs_key, "3fce516009c21727d0f2e4e86ee403bc", 16);
        check("server hs write iv", hs_iv, "5d313eb2671276ee13000b30", 12);
    }

    u8 finished_key_check[32];
    if (!tls13_hkdf_expand_label(s.server_hs_traffic, 32, "finished", 8U, NULL, 0U,
                                 finished_key_check, 32)) {
        printf("[FAIL] tls13_hkdf_expand_label(finished) returned false\n");
        failures++;
    } else {
        check("server finished_key", finished_key_check,
              "008d3b66f816ea559f96b537e885c31fc068bf492c652f01f288a1d8cdc19fc8", 32);
    }

    /* Also independently confirm SHA-256("") matches the well-known
     * constant used throughout the RFC trace as the empty-transcript
     * hash for the "derived" label. */
    u8 empty_hash[32];
    sha256(NULL, 0, empty_hash);
    check("SHA-256(\"\")", empty_hash,
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", 32);

    /* Negative test: tls13_hkdf_expand_label must fail closed on
     * oversized output length rather than silently truncating. */
    u8 scratch[300];
    if (tls13_hkdf_expand_label(s.server_hs_traffic, 32, "key", 3U, NULL, 0U, scratch, 300U)) {
        printf("[FAIL] tls13_hkdf_expand_label accepted out_len > 255\n");
        failures++;
    } else {
        printf("[PASS] tls13_hkdf_expand_label rejects out_len > 255\n");
    }

    /* tls13_consttime_eq sanity. */
    u8 a[8] = {1,2,3,4,5,6,7,8}, b[8] = {1,2,3,4,5,6,7,8}, c[8] = {1,2,3,4,5,6,7,9};
    if (!tls13_consttime_eq(a, b, 8) || tls13_consttime_eq(a, c, 8)) {
        printf("[FAIL] tls13_consttime_eq incorrect result\n");
        failures++;
    } else {
        printf("[PASS] tls13_consttime_eq\n");
    }

    if (failures == 0) {
        printf("test_tls13_keysched: ALL PASS (matches RFC 8448 Section 3 vectors)\n");
        return 0;
    }
    printf("test_tls13_keysched: %d FAILURE(S)\n", failures);
    return 1;
}
