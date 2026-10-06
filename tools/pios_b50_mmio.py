#!/usr/bin/env python3
"""One B50 GMD_ID read through the verified PLX hierarchy; BME stays off."""
import argparse
import json
from pathlib import Path
import socket
import time
import urllib.request

from debug_console_qemu_test import DebugConn

PATH = (((1, 0, 0), 0x874810B5),
        ((2, 8, 0), 0x874810B5),
        ((3, 0, 0), 0xE2FF8086),
        ((4, 1, 0), 0xE2F08086))
GPU = (5, 0, 0)
WINDOW = 0x80F08000
CPU_BASE = 0x1B80000000


def prove(io):
    io.health()
    saved = []
    mapped = False
    try:
        for bdf, identity in PATH:
            if io.read(bdf, 0) != identity or io.read(bdf, 4) & 7:
                raise RuntimeError("bridge identity/ownership gate")
            buses = io.read(bdf, 0x18)
            if not ((buses >> 8) & 255) <= GPU[0] <= ((buses >> 16) & 255):
                raise RuntimeError("bridge does not route selected GPU")
            saved.append((bdf, io.read(bdf, 0x20)))
        if io.read(GPU, 0) != 0xE2128086 or io.read(GPU, 4) & 7:
            raise RuntimeError("B50 identity/ownership gate")
        state = io.command("lzero status")
        for fact in ("5:0.0", "id=E2128086", "bar0=16777216", "mapped=0"):
            if fact not in state:
                raise RuntimeError("selected BAR0 proof changed")
        for bdf, _ in saved:
            io.write(bdf, 0x20, WINDOW)
            if io.read(bdf, 0x20) != WINDOW:
                raise RuntimeError("bridge window readback mismatch")
        reply = io.command("lzero map")
        mapped = "mapped=1" in reply
        if not mapped:
            raise RuntimeError("kernel active-Device translation/map gate refused: " + reply)
        if io.read(GPU, 0x10) & ~15 != 0x80000000 or io.read(GPU, 4) & 7 != 2:
            raise RuntimeError("BAR0/MEM/BME readback mismatch")
        readings = [io.peek(CPU_BASE + 0xD8C) for _ in range(3)]
        if len(set(readings)) != 1 or readings[0] in (0, 0xFFFFFFFF):
            raise RuntimeError("invalid/unstable Intel GMD_ID")
        architecture = readings[0] >> 22
        release = (readings[0] >> 14) & 255
        if architecture != 20 or release not in (1, 2):
            raise RuntimeError(f"GMD_ID is not documented Xe2 HPG: {readings[0]:#x}")
        for bdf, _ in PATH:
            if io.read(bdf, 4) & 7 != 2:
                raise RuntimeError("bridge DMA unexpectedly enabled")
        io.health()
        result = {"gpu": "05:00.0", "gmd_id": hex(readings[0]),
                  "architecture": architecture, "release": release,
                  "revision": readings[0] & 63, "reads": 3,
                  "bar0_bytes": 16777216, "bus_master": False}
        io.record(result=result)
        return result
    except Exception:
        # Revoke MEM before restoring forwarding windows. BAR assignment stays
        # diagnostic state; don't pretend a cached kernel mapping was revoked.
        if saved:
            command = io.read(GPU, 4)
            if command != 0xFFFFFFFF:
                io.write(GPU, 4, (command & 0xFFFF) & ~7)
            for bdf, window in reversed(saved):
                command = io.read(bdf, 4)
                if command != 0xFFFFFFFF:
                    io.write(bdf, 4, (command & 0xFFFF) & ~7)
                    io.write(bdf, 0x20, window)
        raise


class Board:
    def __init__(self, console, log):
        self.console, self.log = console, log
        self.deadline = time.monotonic() + 60
        self.before = self.status()

    def status(self):
        with urllib.request.urlopen("http://192.168.0.201/api/status", timeout=5) as r:
            return json.load(r)

    def record(self, **entry):
        self.log.write(json.dumps(entry) + "\n")
        self.log.flush()

    def command(self, text):
        if time.monotonic() >= self.deadline:
            raise RuntimeError("operator deadline")
        result = self.console.command(text.encode() + b"\r", timeout=5)
        if not result.startswith(text) or not result.endswith("pios> "):
            raise RuntimeError("console framing")
        return result[len(text):-6].strip()

    def peek(self, address):
        result = self.command(f"peek 0x{address:x} 4")
        prefix = f"0x{address:016X} = 0x"
        if not result.startswith(prefix):
            raise RuntimeError(result)
        value = int(result[len(prefix):], 16)
        self.record(read=address, value=value)
        return value

    def poke(self, address, value):
        self.record(write=address, value=value, phase="before")
        if not self.command(f"poke 0x{address:x} 0x{value:x} 4").startswith("OK:"):
            raise RuntimeError("write refused")

    def read(self, bdf, reg):
        bus, dev, fn = bdf
        self.poke(0x1000119000, (bus << 20) | (dev << 15) | (fn << 12))
        return self.peek(0x1000118000 + reg)

    def write(self, bdf, reg, value):
        if reg not in (4, 0x20):
            raise RuntimeError("unapproved register")
        bus, dev, fn = bdf
        self.poke(0x1000119000, (bus << 20) | (dev << 15) | (fn << 12))
        self.poke(0x1000118000 + reg, value)

    def health(self):
        state = self.status()
        if state["version"] != self.before["version"] or state["uptime"] < self.before["uptime"]:
            raise RuntimeError("board reboot")
        if state["diag"]["error"] or state["perf"]["nic_rx_wedge"]:
            raise RuntimeError("management degraded")
        if "uncorr=00000000 corr=00000000" not in self.command("pcie1 aer"):
            raise RuntimeError("AER not clean")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        print("No hardware access; fixed 05:00.0 B50 path, 16MiB BAR0, GMD_ID only.")
        return
    with args.log.open("x", encoding="ascii") as log:
        with socket.create_connection(("192.168.0.201", 2323), timeout=5) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            console = DebugConn(sock)
            console.read_until(b"debug> ")
            if "debug console unlocked" not in console.command(b"unlock pios\r"):
                raise RuntimeError("console locked")
            print(json.dumps(prove(Board(console, log)), indent=2))


if __name__ == "__main__":
    main()
