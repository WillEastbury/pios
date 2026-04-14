/*
 * dhcp.c - Hardened DHCP client
 *
 * State machine: INIT → DISCOVER → SELECTING → REQUEST → BOUND
 * With RENEWING and REBINDING for lease maintenance.
 *
 * Constructs raw Ethernet/IP/UDP frames for DHCP since the client
 * must send from 0.0.0.0 (before it has an IP address).
 *
 * Reference: RFC 2131 (DHCP), RFC 2132 (DHCP Options)
 */

#include "dhcp.h"
#include "net.h"
#include "arp.h"
#include "nic.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"

/* ---- DHCP constants ---- */

#define DHCP_SERVER_PORT    67
#define DHCP_CLIENT_PORT    68

#define DHCP_MAGIC_COOKIE   0x63825363

/* DHCP message types (option 53) */
#define DHCPDISCOVER    1
#define DHCPOFFER       2
#define DHCPREQUEST     3
#define DHCPDECLINE     4
#define DHCPACK         5
#define DHCPNAK         6
#define DHCPRELEASE     7

/* DHCP options */
#define OPT_PAD         0
#define OPT_SUBNET      1
#define OPT_ROUTER      3
#define OPT_DNS         6
#define OPT_HOSTNAME    12
#define OPT_LEASE_TIME  51
#define OPT_MSG_TYPE    53
#define OPT_SERVER_ID   54
#define OPT_PARAM_LIST  55
#define OPT_T1          58
#define OPT_T2          59
#define OPT_END         255

/* Hardening limits */
#define MIN_LEASE_SEC   60
#define MAX_LEASE_SEC   (30 * 24 * 3600)  /* 30 days */
#define DISCOVER_INTERVAL_MS  5000
#define MAX_RETRIES     5

/* ---- DHCP message structure (bootp) ---- */

struct dhcp_msg {
    u8  op;             /* 1=BOOTREQUEST, 2=BOOTREPLY */
    u8  htype;          /* 1=Ethernet */
    u8  hlen;           /* 6 */
    u8  hops;
    u32 xid;
    u16 secs;
    u16 flags;
    u32 ciaddr;
    u32 yiaddr;         /* your (client) IP */
    u32 siaddr;         /* server IP */
    u32 giaddr;
    u8  chaddr[16];     /* client HW address (6 bytes used) */
    u8  sname[64];
    u8  file[128];
    u32 cookie;         /* magic cookie 0x63825363 */
    u8  options[312];
} PACKED;

/* ---- State ---- */

static u32 dhcp_state;
static dhcp_lease_t lease;
static u32 xid;
static u8  our_mac[6];
static u64 state_time;      /* ms when we entered current state */
static u64 bound_time;      /* ms when we became BOUND */
static u32 retries;

/* Frame buffer for DHCP TX */
static u8 dhcp_frame[600] ALIGNED(64);

/* ---- XID generation (hardware CRC32) ---- */

static u32 gen_xid(void) {
    u64 seed[2];
    seed[0] = timer_ticks();
    seed[1] = (u64)our_mac[0] | ((u64)our_mac[1] << 8) |
              ((u64)our_mac[2] << 16) | ((u64)our_mac[3] << 24) |
              ((u64)our_mac[4] << 32) | ((u64)our_mac[5] << 40);
    return hw_crc32c(seed, sizeof(seed));
}

/* ---- Raw frame send (src_ip=0.0.0.0, broadcast) ---- */

static void dhcp_send_raw(const struct dhcp_msg *msg, u32 msg_len, u32 src_ip) {
    u8 *f = dhcp_frame;

    /* Ethernet header */
    struct eth_hdr *eth = (struct eth_hdr *)f;
    u8 bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    simd_memcpy(eth->dst, bcast, 6);
    simd_memcpy(eth->src, our_mac, 6);
    eth->ethertype = htons(ETH_P_IP);

    /* IP header */
    u16 udp_len = (u16)(sizeof(struct udp_hdr) + msg_len);
    u16 ip_total = 20 + udp_len;

    struct ip_hdr *ip = (struct ip_hdr *)(f + sizeof(struct eth_hdr));
    ip->ver_ihl    = 0x45;
    ip->tos        = 0x10;  /* low delay */
    ip->total_len  = htons(ip_total);
    ip->id         = htons(0);
    ip->flags_frag = htons(0);
    ip->ttl        = 128;
    ip->protocol   = IP_PROTO_UDP;
    ip->checksum   = 0;
    ip->src_ip     = htonl(src_ip);
    ip->dst_ip     = htonl(0xFFFFFFFF);
    ip->checksum   = simd_checksum(ip, 20);

    /* UDP header */
    struct udp_hdr *udp = (struct udp_hdr *)(f + sizeof(struct eth_hdr) + 20);
    udp->src_port  = htons(DHCP_CLIENT_PORT);
    udp->dst_port  = htons(DHCP_SERVER_PORT);
    udp->length    = htons(udp_len);
    udp->checksum  = 0;

    /* DHCP message */
    u32 pay_off = sizeof(struct eth_hdr) + 20 + sizeof(struct udp_hdr);
    simd_memcpy(f + pay_off, msg, msg_len);

    u32 frame_len = pay_off + msg_len;
    if (frame_len < 60) frame_len = 60;

    nic_send(f, frame_len);
}

/* ---- DHCP message construction ---- */

static u32 build_base_msg(struct dhcp_msg *msg) {
    simd_zero(msg, sizeof(struct dhcp_msg));
    msg->op = 1;        /* BOOTREQUEST */
    msg->htype = 1;     /* Ethernet */
    msg->hlen = 6;
    msg->xid = htonl(xid);
    msg->flags = htons(0x8000);  /* broadcast flag */
    msg->cookie = htonl(DHCP_MAGIC_COOKIE);
    simd_memcpy(msg->chaddr, our_mac, 6);
    return 0; /* options offset */
}

static u32 add_opt_byte(u8 *opts, u32 opts_size, u32 off, u8 code, u8 val) {
    if (off + 3 > opts_size) return off;
    opts[off++] = code;
    opts[off++] = 1;
    opts[off++] = val;
    return off;
}

static u32 add_opt_u32(u8 *opts, u32 opts_size, u32 off, u8 code, u32 val) {
    if (off + 6 > opts_size) return off;
    opts[off++] = code;
    opts[off++] = 4;
    opts[off++] = (u8)(val >> 24);
    opts[off++] = (u8)(val >> 16);
    opts[off++] = (u8)(val >> 8);
    opts[off++] = (u8)val;
    return off;
}

static void send_discover(void) {
    struct dhcp_msg msg;
    build_base_msg(&msg);

    u32 cap = sizeof(msg.options);
    u32 off = 0;
    off = add_opt_byte(msg.options, cap, off, OPT_MSG_TYPE, DHCPDISCOVER);

    /* Parameter request list */
    if (off + 5 <= cap) {
        msg.options[off++] = OPT_PARAM_LIST;
        msg.options[off++] = 3;
        msg.options[off++] = OPT_SUBNET;
        msg.options[off++] = OPT_ROUTER;
        msg.options[off++] = OPT_DNS;
    }

    if (off + 1 <= cap)
        msg.options[off++] = OPT_END;

    /* Message size: fixed header (236) + cookie (4) + options */
    u32 msg_len = 240 + off;
    dhcp_send_raw(&msg, msg_len, 0);
    uart_puts("[dhcp] DISCOVER sent\n");
}

static void send_request(u32 requested_ip, u32 server_id) {
    struct dhcp_msg msg;
    build_base_msg(&msg);

    u32 cap = sizeof(msg.options);
    u32 off = 0;
    off = add_opt_byte(msg.options, cap, off, OPT_MSG_TYPE, DHCPREQUEST);
    off = add_opt_u32(msg.options, cap, off, OPT_SERVER_ID, server_id);

    /* Requested IP (option 50) */
    if (off + 6 <= cap) {
        msg.options[off++] = 50;
        msg.options[off++] = 4;
        msg.options[off++] = (u8)(requested_ip >> 24);
        msg.options[off++] = (u8)(requested_ip >> 16);
        msg.options[off++] = (u8)(requested_ip >> 8);
        msg.options[off++] = (u8)requested_ip;
    }

    if (off + 1 <= cap)
        msg.options[off++] = OPT_END;

    u32 msg_len = 240 + off;
    dhcp_send_raw(&msg, msg_len, 0);
    uart_puts("[dhcp] REQUEST sent\n");
}

/* ---- Option parsing ---- */

struct dhcp_parsed {
    u8  msg_type;
    u32 server_id;
    u32 offered_ip;
    u32 subnet;
    u32 router;
    u32 dns;
    u32 lease_time;
    u32 t1;
    u32 t2;
};

static u32 read_u32_be(const u8 *p) {
    return ((u32)p[0]<<24) | ((u32)p[1]<<16) | ((u32)p[2]<<8) | p[3];
}

static bool parse_dhcp_options(const u8 *opts, u32 opts_len, struct dhcp_parsed *out) {
    simd_zero(out, sizeof(*out));
    u32 i = 0;

    while (i < opts_len) {
        u8 code = opts[i++];
        if (code == OPT_PAD) continue;
        if (code == OPT_END) break;
        if (i >= opts_len) return false;
        u8 len = opts[i++];
        if (i + len > opts_len) return false; /* bounds check */

        switch (code) {
        case OPT_MSG_TYPE:  if (len >= 1) out->msg_type = opts[i]; break;
        case OPT_SUBNET:    if (len >= 4) out->subnet = read_u32_be(opts+i); break;
        case OPT_ROUTER:    if (len >= 4) out->router = read_u32_be(opts+i); break;
        case OPT_DNS:       if (len >= 4) out->dns = read_u32_be(opts+i); break;
        case OPT_LEASE_TIME:if (len >= 4) out->lease_time = read_u32_be(opts+i); break;
        case OPT_SERVER_ID: if (len >= 4) out->server_id = read_u32_be(opts+i); break;
        case OPT_T1:        if (len >= 4) out->t1 = read_u32_be(opts+i); break;
        case OPT_T2:        if (len >= 4) out->t2 = read_u32_be(opts+i); break;
        default: break;
        }
        i += len;
    }
    return out->msg_type != 0;
}

/* ---- UDP callback for DHCP responses ---- */

static void dhcp_udp_handler(u32 src_ip, u16 src_port, u16 dst_port,
                              const u8 *data, u16 len) {
    (void)src_ip;
    (void)src_port;
    if (dst_port != DHCP_CLIENT_PORT) return;
    if (len < 240) return; /* minimum bootp + cookie */

    const struct dhcp_msg *msg = (const struct dhcp_msg *)data;
    if (msg->op != 2) return;                         /* must be BOOTREPLY */
    if (ntohl(msg->xid) != xid) return;               /* XID must match */
    if (ntohl(msg->cookie) != DHCP_MAGIC_COOKIE) return;

    u32 opts_off = 240;
    u32 opts_len = (len > opts_off) ? len - opts_off : 0;
    struct dhcp_parsed parsed;
    if (!parse_dhcp_options(data + opts_off, opts_len, &parsed))
        return;

    u32 offered = ntohl(msg->yiaddr);

    switch (dhcp_state) {
    case DHCP_SELECTING:
        if (parsed.msg_type != DHCPOFFER) return;
        if (offered == 0) return;

        /* Lease sanity */
        if (parsed.lease_time > 0 &&
            (parsed.lease_time < MIN_LEASE_SEC || parsed.lease_time > MAX_LEASE_SEC))
            return;

        lease.ip = offered;
        lease.mask = parsed.subnet;
        lease.gateway = parsed.router;
        lease.dns = parsed.dns;
        lease.server_id = parsed.server_id;
        lease.lease_time = parsed.lease_time ? parsed.lease_time : 3600;

        uart_puts("[dhcp] OFFER: ");
        uart_hex(offered);
        uart_puts(" from server ");
        uart_hex(parsed.server_id);
        uart_puts("\n");

        /* Transition to REQUESTING */
        dhcp_state = DHCP_REQUESTING;
        send_request(offered, parsed.server_id);
        state_time = timer_ticks();
        retries = 0;
        break;

    case DHCP_REQUESTING:
        /* Server pinning: only accept from the server we requested from */
        if (parsed.server_id != lease.server_id) return;

        if (parsed.msg_type == DHCPNAK) {
            uart_puts("[dhcp] NAK received, restarting\n");
            dhcp_state = DHCP_INIT;
            return;
        }
        if (parsed.msg_type != DHCPACK) return;

        /* Update lease with final values from ACK */
        if (parsed.lease_time) lease.lease_time = parsed.lease_time;
        if (parsed.subnet) lease.mask = parsed.subnet;
        if (parsed.router) lease.gateway = parsed.router;
        if (parsed.dns) lease.dns = parsed.dns;
        lease.t1 = parsed.t1 ? parsed.t1 : lease.lease_time / 2;
        lease.t2 = parsed.t2 ? parsed.t2 : (lease.lease_time * 7) / 8;

        /* ARP conflict detection: probe the offered IP */
        const u8 *conflict = arp_resolve(lease.ip);
        if (conflict) {
            uart_puts("[dhcp] CONFLICT detected, declining\n");
            dhcp_state = DHCP_INIT;
            return;
        }

        /* BOUND! Apply configuration. */
        uart_puts("[dhcp] BOUND: IP=");
        uart_hex(lease.ip);
        uart_puts(" mask=");
        uart_hex(lease.mask);
        uart_puts(" gw=");
        uart_hex(lease.gateway);
        uart_puts(" lease=");
        uart_hex(lease.lease_time);
        uart_puts("s\n");

        /* Reconfigure network stack */
        net_init(lease.ip, lease.gateway, lease.mask, NULL);

        dhcp_state = DHCP_BOUND;
        bound_time = timer_ticks();
        break;

    case DHCP_RENEWING:
    case DHCP_REBINDING:
        if (parsed.server_id != lease.server_id && dhcp_state == DHCP_RENEWING)
            return;
        if (parsed.msg_type == DHCPACK) {
            if (parsed.lease_time) lease.lease_time = parsed.lease_time;
            lease.t1 = parsed.t1 ? parsed.t1 : lease.lease_time / 2;
            lease.t2 = parsed.t2 ? parsed.t2 : (lease.lease_time * 7) / 8;
            dhcp_state = DHCP_BOUND;
            bound_time = timer_ticks();
            uart_puts("[dhcp] Lease renewed\n");
        } else if (parsed.msg_type == DHCPNAK) {
            dhcp_state = DHCP_INIT;
        }
        break;

    default:
        break;
    }
}

/* ---- Public API ---- */

bool dhcp_start(u32 timeout_ms) {
    nic_get_mac(our_mac);
    xid = gen_xid();
    dhcp_state = DHCP_INIT;
    retries = 0;

    /* Register as UDP listener on port 68 */
    net_set_udp_callback(dhcp_udp_handler);

    u64 deadline = timer_ticks() + timeout_ms;

    while (timer_ticks() < deadline) {
        if (dhcp_state == DHCP_BOUND)
            return true;

        dhcp_poll();
        net_poll();
        timer_delay_ms(10);
    }

    uart_puts("[dhcp] Timeout\n");
    return (dhcp_state == DHCP_BOUND);
}

void dhcp_poll(void) {
    u64 now = timer_ticks();
    u64 elapsed = now - state_time;

    switch (dhcp_state) {
    case DHCP_INIT:
        send_discover();
        dhcp_state = DHCP_SELECTING;
        state_time = now;
        retries = 0;
        break;

    case DHCP_SELECTING:
        /* Retry DISCOVER if no offer after interval */
        if (elapsed > DISCOVER_INTERVAL_MS) {
            if (++retries > MAX_RETRIES) {
                dhcp_state = DHCP_INIT;
                retries = 0;
                state_time = now;
                return;
            }
            send_discover();
            state_time = now;
        }
        break;

    case DHCP_REQUESTING:
        /* Retry REQUEST */
        if (elapsed > 2000) {
            if (++retries > MAX_RETRIES) {
                dhcp_state = DHCP_INIT;
                return;
            }
            send_request(lease.ip, lease.server_id);
            state_time = now;
        }
        break;

    case DHCP_BOUND:
        /* Check T1 (renew) */
        if (now - bound_time > (u64)lease.t1 * 1000) {
            uart_puts("[dhcp] T1 expired, renewing\n");
            dhcp_state = DHCP_RENEWING;
            send_request(lease.ip, lease.server_id);
            state_time = now;
            retries = 0;
        }
        break;

    case DHCP_RENEWING:
        if (elapsed > 10000) {
            if (now - bound_time > (u64)lease.t2 * 1000) {
                uart_puts("[dhcp] T2 expired, rebinding\n");
                dhcp_state = DHCP_REBINDING;
                send_discover();
                state_time = now;
            } else {
                send_request(lease.ip, lease.server_id);
                state_time = now;
            }
        }
        break;

    case DHCP_REBINDING:
        if (elapsed > DISCOVER_INTERVAL_MS) {
            if (now - bound_time > (u64)lease.lease_time * 1000) {
                uart_puts("[dhcp] Lease expired\n");
                dhcp_state = DHCP_INIT;
            } else {
                send_discover();
                state_time = now;
            }
        }
        break;
    }
}

const dhcp_lease_t *dhcp_get_lease(void) {
    return (dhcp_state >= DHCP_BOUND) ? &lease : NULL;
}

u32 dhcp_get_state(void) {
    return dhcp_state;
}
