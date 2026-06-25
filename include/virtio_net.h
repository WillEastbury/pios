#pragma once
#include "types.h"
#include "platform.h"

/*
 * virtio-net (virtio-mmio) NIC backend for the QEMU `virt` platform.
 *
 * Provides the same poll-driven, non-blocking RX/TX contract that net.c /
 * nic.c expect from a hardware MAC, so the full kernel network stack
 * (net_poll -> nic_recv, tcp_output -> nic_send) runs unchanged under QEMU.
 *
 * Single-producer/single-consumer split virtqueues, driven entirely by
 * polling on core 0 (no MSI/IRQ) to mirror the Pi5 GENET/MACB polling model.
 *
 * Supports both legacy (virtio-mmio version 1, QueuePFN) and modern
 * (version 2, split-queue address registers) transports.
 */

bool virtio_net_init(void);
bool virtio_net_present(void);
bool virtio_net_probe(void);
u32  virtio_net_diag_code(void);

bool virtio_net_send(const u8 *frame, u32 len);
bool virtio_net_recv(u8 *frame, u32 *len);
void virtio_net_get_mac(u8 *mac);
bool virtio_net_link_up(void);
