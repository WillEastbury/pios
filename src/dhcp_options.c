/*
 * dhcp_options.c — standalone DHCP option parser, extracted from src/dhcp.c.
 *
 * Contains only the pure-logic option-parsing code (no MMIO, no asm, no
 * network stack deps) so it can be compiled on the host for unit testing.
 * All other DHCP state-machine code remains in src/dhcp.c and is untouched.
 *
 * This file is the canonical implementation; src/dhcp.c #includes it instead
 * of reimplementing the parser. That guarantees there is exactly one copy of
 * the parser and the host unit test exercises the real production logic.
 */
#include "types.h"
#include "dhcp.h"

/* DHCP option codes used by the parser */
#define OPT_PAD         0
#define OPT_SUBNET      1
#define OPT_ROUTER      3
#define OPT_DNS         6
#define OPT_LEASE_TIME  51
#define OPT_MSG_TYPE    53
#define OPT_SERVER_ID   54
#define OPT_T1          58
#define OPT_T2          59
#define OPT_END         255

static u32 dhcp_read_u32_be(const u8 *p) {
    return ((u32)p[0]<<24) | ((u32)p[1]<<16) | ((u32)p[2]<<8) | p[3];
}

/*
 * Parse a raw DHCP option stream (everything after the 4-byte magic cookie).
 * Returns true when at least a message-type option was decoded.
 * Performs full bounds checks: a malformed stream (truncated length or data)
 * is rejected, never overflows out.
 */
bool dhcp_parse_options_test(const u8 *opts, u32 opts_len, dhcp_parsed_test_t *out) {
    if (!opts || !out) return false;
    /* zero all fields; failure check is msg_type == 0 */
    u32 i;
    for (i = 0; i < sizeof(*out); i++) ((u8 *)out)[i] = 0;
    i = 0;

    while (i < opts_len) {
        u8 code = opts[i++];
        if (code == OPT_PAD) continue;
        if (code == OPT_END) break;
        if (i >= opts_len) return false;           /* no length byte */
        u8 len = opts[i++];
        if (i + len > opts_len) return false;      /* data overrun */

        switch (code) {
        case OPT_MSG_TYPE:   if (len >= 1) out->msg_type   = opts[i]; break;
        case OPT_SUBNET:     if (len >= 4) out->subnet     = dhcp_read_u32_be(opts+i); break;
        case OPT_ROUTER:     if (len >= 4) out->router     = dhcp_read_u32_be(opts+i); break;
        case OPT_DNS:        if (len >= 4) out->dns        = dhcp_read_u32_be(opts+i); break;
        case OPT_LEASE_TIME: if (len >= 4) out->lease_time = dhcp_read_u32_be(opts+i); break;
        case OPT_SERVER_ID:  if (len >= 4) out->server_id  = dhcp_read_u32_be(opts+i); break;
        case OPT_T1:         if (len >= 4) out->t1         = dhcp_read_u32_be(opts+i); break;
        case OPT_T2:         if (len >= 4) out->t2         = dhcp_read_u32_be(opts+i); break;
        default: break;
        }
        i += len;
    }
    return out->msg_type != 0;
}
