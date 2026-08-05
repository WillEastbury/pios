/*
 * Host test for tls13_handshake.c's ClientHello parser, using the OFFICIAL
 * RFC 8448 Section 3 "Simple 1-RTT Handshake" ClientHello byte capture
 * (196 octets, including its 4-byte handshake header) as the test vector
 * -- a genuine wire capture, not a hand-built one. That ClientHello offers
 * x25519 (not secp256r1) for key_share, so has_p256_key_share is expected
 * to be false here; a second, hand-built ClientHello with a secp256r1
 * key_share is used to check the accept path.
 */
#include "tls13_handshake.h"
#include <stdio.h>
#include <string.h>

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static u32 hex_to_bytes(const char *hex, u8 *out, u32 out_cap) {
    u32 n = 0;
    while (hex[0] && hex[1] && n < out_cap) {
        out[n++] = (u8)((hex_nibble(hex[0]) << 4) | hex_nibble(hex[1]));
        hex += 2;
    }
    return n;
}

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (cond) { printf("[PASS] %s\n", msg); } \
    else { printf("[FAIL] %s\n", msg); failures++; } \
} while (0)

int main(void) {
    /* RFC 8448 Section 3 ClientHello, including its 4-byte handshake
     * header (01 00 00 c0 = ClientHello, length 0x0c0=192). Verified to
     * decode to exactly 196 bytes independently before use here. */
    static const char *ch_hex =
        "010000c00303cb34ecb1e78163ba1c38c6dacb196a6dffa21a8d9912ec18a2ef6283024dece7"
        "000006130113031302"
        "0100"
        "0091"
        "0000000b0009000006736572766572"
        "ff01000100"
        "000a0014001200 1d0017001800190100010101020103 0104"
        "00230000"
        "0033002600 24001d0020"
        "99381de560e4bd43d23d8e435a7dbafeb3c06e51c13cae4d5413691e529aaf2c"
        "002b0003020304"
        "000d0020001e040305030603020308040805080604010501060102010402050206020202"
        "002d00020101"
        "001c00024001";

    u8 buf[512];
    char cleaned[1024];
    u32 ci = 0;
    for (const char *p = ch_hex; *p; p++) {
        if (*p != ' ') cleaned[ci++] = *p;
    }
    cleaned[ci] = 0;
    u32 total_len = hex_to_bytes(cleaned, buf, sizeof(buf));
    printf("total bytes parsed from hex: %u (expect 196)\n", total_len);
    CHECK(total_len == 196U, "test vector hex decodes to 196 bytes");

    u8 msg_type = 0;
    u32 body_len = 0;
    bool hdr_ok = tls13_next_handshake_header(buf, total_len, &msg_type, &body_len);
    CHECK(hdr_ok, "tls13_next_handshake_header succeeds");
    CHECK(msg_type == TLS13_HS_CLIENT_HELLO, "msg_type == ClientHello (1)");
    CHECK(body_len == 192U, "body_len == 192");

    struct tls13_client_hello ch;
    bool parsed = tls13_parse_client_hello(buf + 4, body_len, &ch);
    CHECK(parsed, "tls13_parse_client_hello succeeds on real RFC 8448 ClientHello");

    if (parsed) {
        CHECK(ch.legacy_session_id_len == 0U, "legacy_session_id_len == 0");
        CHECK(ch.cipher_suite_count == 3U, "cipher_suite_count == 3");
        CHECK(ch.cipher_suite_count == 3U &&
              ch.cipher_suites[0] == 0x1301U && ch.cipher_suites[1] == 0x1303U &&
              ch.cipher_suites[2] == 0x1302U,
              "cipher_suites == [0x1301, 0x1303, 0x1302]");
        CHECK(ch.has_supported_versions && ch.offers_tls13, "offers TLS 1.3 via supported_versions");
        CHECK(ch.group_count == 9U && ch.groups[0] == 0x001dU && ch.groups[1] == 0x0017U,
              "supported_groups[0..1] == [x25519, secp256r1]");
        CHECK(!ch.has_p256_key_share, "no secp256r1 key_share offered (this ClientHello only offers x25519)");
        CHECK(ch.sig_alg_count == 15U && ch.sig_algs[0] == 0x0403U,
              "signature_algorithms[0] == ecdsa_secp256r1_sha256 (0x0403), count == 15");
        CHECK(ch.has_sni && ch.sni_len == 6U && memcmp(ch.sni, "server", 6) == 0,
              "SNI == \"server\"");
    }

    /* --- Hand-built minimal ClientHello WITH a secp256r1 key_share, to
     * check the accept path picks it up correctly. Deliberately minimal
     * (no SNI, single cipher suite) -- exercises the "small/empty vector"
     * edge cases the real capture above doesn't hit as directly. */
    {
        u8 mb[256];
        u32 n = 0;
        u8 body[256];
        u32 bn = 0;
        /* legacy_version */
        body[bn++] = 0x03; body[bn++] = 0x03;
        /* random (32 zero bytes, doesn't matter for this parser) */
        for (u32 i = 0; i < 32; i++) body[bn++] = 0;
        /* session_id: 1 byte, value 0xAA */
        body[bn++] = 1; body[bn++] = 0xAA;
        /* cipher_suites: 1 suite */
        body[bn++] = 0; body[bn++] = 2;
        body[bn++] = 0x13; body[bn++] = 0x01;
        /* compression methods: [0] */
        body[bn++] = 1; body[bn++] = 0;
        /* extensions: supported_versions (0x304) + key_share (secp256r1) */
        u32 ext_len_pos = bn;
        body[bn++] = 0; body[bn++] = 0; /* placeholder for extensions length */
        u32 ext_start = bn;
        /* supported_versions ext */
        body[bn++] = 0x00; body[bn++] = 0x2b;
        body[bn++] = 0x00; body[bn++] = 0x03; /* ext data len = 3 */
        body[bn++] = 0x02;                    /* versions list len = 2 */
        body[bn++] = 0x03; body[bn++] = 0x04; /* TLS 1.3 */
        /* key_share ext with one secp256r1 entry */
        body[bn++] = 0x00; body[bn++] = 0x33;
        body[bn++] = 0x00; body[bn++] = (u8)(2 + 2 + 2 + 65); /* ext data len */
        body[bn++] = 0x00; body[bn++] = (u8)(2 + 2 + 65);     /* client_shares vector len */
        body[bn++] = 0x00; body[bn++] = 0x17;                 /* group = secp256r1 */
        body[bn++] = 0x00; body[bn++] = 65;                   /* key_exchange len = 65 */
        body[bn++] = 0x04;                                    /* SEC1 uncompressed prefix */
        for (u32 i = 0; i < 64; i++) body[bn++] = (u8)(0x10 + i);
        u32 ext_total = bn - ext_start;
        body[ext_len_pos] = (u8)(ext_total >> 8);
        body[ext_len_pos + 1] = (u8)(ext_total & 0xFF);

        /* wrap in a handshake header */
        mb[n++] = TLS13_HS_CLIENT_HELLO;
        mb[n++] = (u8)(bn >> 16); mb[n++] = (u8)(bn >> 8); mb[n++] = (u8)bn;
        memcpy(mb + n, body, bn); n += bn;

        u8 mt = 0; u32 blen = 0;
        bool ok = tls13_next_handshake_header(mb, n, &mt, &blen);
        CHECK(ok && mt == TLS13_HS_CLIENT_HELLO && blen == bn, "hand-built ClientHello header parses");

        struct tls13_client_hello ch2;
        bool p2 = tls13_parse_client_hello(mb + 4, blen, &ch2);
        CHECK(p2, "hand-built ClientHello with secp256r1 key_share parses");
        if (p2) {
            CHECK(ch2.has_p256_key_share, "secp256r1 key_share detected");
            CHECK(ch2.p256_key_share[0] == 0x04, "key_share starts with SEC1 uncompressed prefix 0x04");
            bool bytes_ok = true;
            for (u32 i = 0; i < 64; i++)
                if (ch2.p256_key_share[1 + i] != (u8)(0x10 + i)) bytes_ok = false;
            CHECK(bytes_ok, "secp256r1 key_share X||Y bytes round-trip exactly");
        }
    }

    /* --- Negative tests: truncated/malformed input must fail closed, never
     * read out of bounds (ASan/valgrind-clean by construction, but this
     * asserts the *logical* rejection too). */
    {
        struct tls13_client_hello ch3;
        CHECK(!tls13_parse_client_hello(buf + 4, 10U, &ch3), "truncated ClientHello body rejected");
        CHECK(!tls13_parse_client_hello(buf + 4, body_len - 1U, &ch3), "ClientHello short by one byte rejected");
        CHECK(!tls13_next_handshake_header(buf, 3U, &msg_type, &body_len), "header rejected with <4 bytes available");
    }

    if (failures == 0) {
        printf("test_tls13_clienthello: ALL PASS\n");
        return 0;
    }
    printf("test_tls13_clienthello: %d FAILURE(S)\n", failures);
    return 1;
}
