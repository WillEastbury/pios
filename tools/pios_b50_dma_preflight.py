#!/usr/bin/env python3
"""Read-only live PCIe1 DMA configuration audit; never grants BME authority."""
import argparse
import json
from pathlib import Path
import socket

from debug_console_qemu_test import DebugConn
from pios_b50_mmio import Board, GPU, PATH

ROOT = 0x1000110000
EXPECTED = {
    0x402C: 0, 0x4030: 0,
    0x4034: 9, 0x4038: 0x10,
    0x403C: 0, 0x4040: 0,
    0x4044: 0, 0x4048: 0,
    0x40AC: 0, 0x40B0: 0,
    0x40B4: 0x13000001, 0x40B8: 0,
    0x40BC: 0, 0x40C0: 0,
}


def inspect(io):
    io.health()
    root = {offset: io.peek(ROOT + offset) for offset in EXPECTED}
    for offset, expected in EXPECTED.items():
        if root[offset] != expected:
            raise RuntimeError(
                f"root {offset:#x}: expected {expected:#x}, got {root[offset]:#x}")
    control = io.peek(ROOT + 0x4008)
    if (control >> 27) & 31 != 9:
        raise RuntimeError("SCB0 size does not match the 16MiB arena")
    commands = {}
    for bdf, identity in PATH + ((GPU, 0xE2128086),
                                 ((9, 0, 0), 0xE2128086),
                                 ((13, 0, 0), 0xE2128086)):
        if io.read(bdf, 0) != identity:
            raise RuntimeError(f"identity changed at {bdf}")
        command = io.read(bdf, 4) & 0xFFFF
        expected = 0 if bdf in ((9, 0, 0), (13, 0, 0)) else 2
        if command & 7 != expected:
            raise RuntimeError(f"unexpected IO/MEM/BME at {bdf}: {command:#x}")
        commands[f"{bdf[0]:02x}:{bdf[1]:02x}.{bdf[2]}"] = command
    io.health()
    result = {
        "root_registers": {hex(k): hex(v) for k, v in root.items()},
        "command_registers": commands,
        "inbound_pci_base": "0x1000000000",
        "inbound_cpu_base": "0x13000000",
        "inbound_bytes": 0x1000000,
        "bus_master": False,
        "hardware_activation_allowed": False,
        "dma_transfer_proven": False,
        "msi_delivery_proven": False,
    }
    io.record(preflight=result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    with args.log.open("x", encoding="ascii") as log:
        with socket.create_connection(("192.168.0.201", 2323), timeout=5) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            console = DebugConn(sock)
            console.read_until(b"debug> ")
            if "debug console unlocked" not in console.command(b"unlock pios\r"):
                raise RuntimeError("console locked")
            print(json.dumps(inspect(Board(console, log)), indent=2))


if __name__ == "__main__":
    main()
