#pragma once
#include "types.h"

/*
 * pioscap.h - lightweight on-device Ethernet frame capture
 *
 * Captures RX/TX frames at the nic.c choke point (the same place
 * fb_pkt_dump() taps, so it sees every frame regardless of protocol) into a
 * fixed-size ring buffer, and can export the ring as a standard PCAP file
 * (readable directly by Wireshark/tcpdump) over HTTP. This requires no
 * host-side packet-capture driver (Npcap/WinPcap/pktmon) -- useful when the
 * dev machine is managed/locked down and cannot have one installed, or to
 * capture exactly what PIOS's own NIC driver saw rather than an external tap.
 *
 * Sized to comfortably fit one PCAP export inside a single 16000-byte HTTP
 * response buffer with no pagination: capacity * (16-byte pcap record
 * header + PIOSCAP_SNAPLEN) + 24-byte pcap global header must stay well
 * under that. 128 * (16 + 96) + 24 = 14360 bytes.
 */

#define PIOSCAP_MAX_ENTRIES   128U
#define PIOSCAP_SNAPLEN       96U   /* bytes captured per frame: full Eth+IP+TCP
                                       headers (54B) plus a little payload/options */

void pioscap_init(void);
void pioscap_enable(bool on);
bool pioscap_enabled(void);
void pioscap_clear(void);

/* Called from nic.c's RX/TX choke points. Cheap no-op when disabled. */
void pioscap_rx(const u8 *frame, u32 len);
void pioscap_tx(const u8 *frame, u32 len);

struct pioscap_status {
    bool enabled;
    u32  count;      /* frames currently held (<= PIOSCAP_MAX_ENTRIES) */
    u32  capacity;
    u64  dropped;    /* frames evicted because the ring was full */
    u64  total_rx;
    u64  total_tx;
};
void pioscap_status(struct pioscap_status *out);

/* Write the entire capture ring as a PCAP file into `out` (caller-provided
 * buffer), oldest frame first. Returns bytes written. */
u32 pioscap_export(u8 *out, u32 max);
