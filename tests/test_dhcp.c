/*
 * Host unit test for the DHCP option parser in src/dhcp.c.
 *
 * The parser (parse_dhcp_options) is pure logic — it only reads a byte array
 * and fills a struct. The host test verifies correct decoding of all option
 * codes, proper bounds checks, and graceful handling of malformed inputs.
 *
 * Compiled natively (see tests/run_host_tests.py). tests/stubinc is on the
 * include path ahead of include/ so types.h and simd.h resolve to host shims.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "dhcp.h"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, name) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s (%s:%d)\n", (name), __FILE__, __LINE__); } \
} while (0)

/* Helper: build a raw DHCP option stream from (code, len, data...) triplets.
 * Terminated with OPT_END (0xFF). */
#define OPT_PAD         0
#define OPT_SUBNET      1
#define OPT_ROUTER      3
#define OPT_DNS         6
#define OPT_LEASE_TIME  51
#define OPT_MSG_TYPE    53
#define OPT_SERVER_ID   54
#define OPT_T1          58
#define OPT_T2          59
#define OPT_END         0xFF

static u32 wr32(u8 *p, u32 v) {
    p[0]=(u8)(v>>24); p[1]=(u8)(v>>16); p[2]=(u8)(v>>8); p[3]=(u8)v;
    return 4;
}

int main(void) {
    printf("test_dhcp (option parser):\n");
    dhcp_parsed_test_t d;

    /* --- 1. Full OFFER --- */
    {
        u8 opts[64]; u32 i = 0;
        /* msg_type = OFFER (2) */
        opts[i++]=OPT_MSG_TYPE; opts[i++]=1; opts[i++]=2;
        /* subnet 255.255.255.0 */
        opts[i++]=OPT_SUBNET; opts[i++]=4; i+=wr32(opts+i, 0xFFFFFF00u);
        /* router 192.168.0.1 */
        opts[i++]=OPT_ROUTER; opts[i++]=4; i+=wr32(opts+i, 0xC0A80001u);
        /* dns 8.8.8.8 */
        opts[i++]=OPT_DNS; opts[i++]=4; i+=wr32(opts+i, 0x08080808u);
        /* lease 3600s */
        opts[i++]=OPT_LEASE_TIME; opts[i++]=4; i+=wr32(opts+i, 3600);
        /* server_id 192.168.0.254 */
        opts[i++]=OPT_SERVER_ID; opts[i++]=4; i+=wr32(opts+i, 0xC0A800FEu);
        /* t1 1800 t2 3150 */
        opts[i++]=OPT_T1; opts[i++]=4; i+=wr32(opts+i, 1800);
        opts[i++]=OPT_T2; opts[i++]=4; i+=wr32(opts+i, 3150);
        opts[i++]=OPT_END;
        CHECK(dhcp_parse_options_test(opts, i, &d), "OFFER: parse ok");
        CHECK(d.msg_type   == 2,          "OFFER: msg_type=OFFER");
        CHECK(d.subnet     == 0xFFFFFF00u, "OFFER: subnet");
        CHECK(d.router     == 0xC0A80001u, "OFFER: router");
        CHECK(d.dns        == 0x08080808u, "OFFER: dns");
        CHECK(d.lease_time == 3600,        "OFFER: lease_time");
        CHECK(d.server_id  == 0xC0A800FEu, "OFFER: server_id");
        CHECK(d.t1         == 1800,        "OFFER: t1");
        CHECK(d.t2         == 3150,        "OFFER: t2");
    }

    /* --- 2. ACK (msg_type=5) with PAD bytes --- */
    {
        u8 opts[32]; u32 i = 0;
        opts[i++]=OPT_PAD; opts[i++]=OPT_PAD;   /* pads must be skipped */
        opts[i++]=OPT_MSG_TYPE; opts[i++]=1; opts[i++]=5;
        opts[i++]=OPT_LEASE_TIME; opts[i++]=4; i+=wr32(opts+i, 86400);
        opts[i++]=OPT_END;
        CHECK(dhcp_parse_options_test(opts, i, &d), "ACK: parse ok");
        CHECK(d.msg_type == 5,     "ACK: msg_type=ACK");
        CHECK(d.lease_time == 86400, "ACK: lease_time");
    }

    /* --- 3. Missing msg_type → should return false --- */
    {
        u8 opts[16]; u32 i = 0;
        opts[i++]=OPT_SUBNET; opts[i++]=4; i+=wr32(opts+i, 0xFFFFFF00u);
        opts[i++]=OPT_END;
        CHECK(!dhcp_parse_options_test(opts, i, &d), "no msg_type → false");
    }

    /* --- 4. Truncated length field → bounds reject --- */
    {
        u8 opts[4] = { OPT_MSG_TYPE, 5, 2, OPT_END }; /* len=5 but only 1 byte follows */
        CHECK(!dhcp_parse_options_test(opts, 4, &d), "truncated len → false");
    }

    /* --- 5. Empty options --- */
    {
        u8 opts[1] = { OPT_END };
        CHECK(!dhcp_parse_options_test(opts, 1, &d), "empty (only END) → false");
    }

    /* --- 6. Unknown option codes are skipped --- */
    {
        u8 opts[32]; u32 i = 0;
        opts[i++]=42; opts[i++]=3; opts[i++]=0xAA; opts[i++]=0xBB; opts[i++]=0xCC; /* unknown */
        opts[i++]=OPT_MSG_TYPE; opts[i++]=1; opts[i++]=3;   /* NACK */
        opts[i++]=OPT_END;
        CHECK(dhcp_parse_options_test(opts, i, &d), "unknown opts skipped");
        CHECK(d.msg_type == 3, "after unknown: msg_type=NACK");
    }

    /* --- 7. Zero-length options body --- */
    {
        CHECK(!dhcp_parse_options_test(NULL, 0, &d), "zero len → false (NULL)");
        u8 empty[0];
        CHECK(!dhcp_parse_options_test(empty, 0, &d), "zero len → false (empty)");
    }

    printf("  %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
