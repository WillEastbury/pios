#!/usr/bin/env python3
"""Explicit Pi5 PCIe1 bus numbering only; never BARs, Command, DMA or MSI.

Close sibling routes before opening one temporary subordinate range, recurse,
then tighten that range before visiting the next sibling. A failed operation
quarantines the changed routes; it never restores overlapping old bus numbers.
"""
import argparse
import json
from pathlib import Path
import socket
import time
import urllib.request

from debug_console_qemu_test import DebugConn

MAX_BUS = 63
MAX_FUNCTIONS = 64
MAX_DEPTH = 8
PLX_ID = 0x874810B5


class BusError(RuntimeError):
    pass


def present(identity):
    return identity & 0xFFFF not in (0, 0xFFFF)


def configure(io):
    functions = []
    changed = []
    next_bus = 2

    def write(bdf, value):
        io.write(bdf, 0x18, value)
        if io.read(bdf, 0x18) != value:
            raise BusError(f"bus-number readback failed at {bdf}")

    def walk(bus, depth):
        nonlocal next_bus
        if depth > MAX_DEPTH:
            raise BusError("bridge depth exhausted")
        local = []
        # Examine the whole immediate bus before opening any downstream route.
        for dev in range(32):
            identity = io.read((bus, dev, 0), 0)
            if not present(identity):
                continue
            header = io.read((bus, dev, 0), 12)
            if header == 0xFFFFFFFF:
                raise BusError("configuration disappeared")
            for fn in range(8 if header & 0x800000 else 1):
                bdf = (bus, dev, fn)
                ident = io.read(bdf, 0)
                if not present(ident):
                    continue
                if len(functions) >= MAX_FUNCTIONS:
                    raise BusError("function capacity exhausted")
                command, classrev, hdr = (io.read(bdf, reg) for reg in (4, 8, 12))
                if command == 0xFFFFFFFF or classrev == 0xFFFFFFFF or hdr == 0xFFFFFFFF:
                    raise BusError(f"configuration disappeared at {bdf}")
                if command & 7:
                    raise BusError(f"active device at {bdf}; Command={command & 0xFFFF:#x}")
                kind = (hdr >> 16) & 0x7F
                bridge = kind == 1 and classrev >> 16 == 0x0604
                if kind not in (0, 1) or (kind == 1 and not bridge):
                    raise BusError(f"unsupported header/class at {bdf}")
                record = {"bdf": list(bdf), "identity": ident, "class": classrev >> 8,
                          "command": command & 0xFFFF, "bridge": bridge}
                if bridge:
                    record["old_buses"] = io.read(bdf, 0x18)
                    if record["old_buses"] == 0xFFFFFFFF:
                        raise BusError("unreadable bridge buses")
                functions.append(record)
                local.append(record)
        for record in local:
            if record["bridge"]:
                changed.append(record)
                write(tuple(record["bdf"]), record["old_buses"] & 0xFF000000)
        for record in local:
            if not record["bridge"]:
                continue
            if next_bus > MAX_BUS:
                raise BusError("bus capacity exhausted")
            secondary = next_bus
            next_bus += 1
            bdf = tuple(record["bdf"])
            upper = record["old_buses"] & 0xFF000000
            write(bdf, upper | bus | (secondary << 8) | (MAX_BUS << 16))
            walk(secondary, depth + 1)
            subordinate = next_bus - 1
            write(bdf, upper | bus | (secondary << 8) | (subordinate << 16))
            record.update(secondary=secondary, subordinate=subordinate)

    if io.read((1, 0, 0), 0) != PLX_ID:
        raise BusError("expected live PLX 10B5:8748 at 01:00.0")
    try:
        walk(1, 0)
        for record in functions:
            bdf = tuple(record["bdf"])
            if io.read(bdf, 0) != record["identity"] or io.read(bdf, 4) & 7:
                raise BusError(f"identity/Command changed at {bdf}")
        return {"functions": functions, "buses_used": next_bus - 1,
                "b50_count": sum(r["identity"] == 0xE2128086 for r in functions),
                "dma_enabled": False}
    except Exception:
        # Children are still addressed through their parents here. Close them
        # first; if a link vanished, never claim rollback or successful repair.
        for record in reversed(changed):
            write(tuple(record["bdf"]), record["old_buses"] & 0xFF000000)
        raise


class Board:
    def __init__(self, host, console, log):
        self.host, self.console, self.log = host, console, log
        self.deadline = time.monotonic() + 120
        self.transactions = 0
        self.before = self.status()
        self.health()

    def record(self, **event):
        self.log.write(json.dumps(event) + "\n")
        self.log.flush()

    def status(self):
        with urllib.request.urlopen(f"http://{self.host}/api/status", timeout=5) as response:
            return json.load(response)

    def command(self, text):
        if time.monotonic() >= self.deadline or self.transactions >= 20000:
            raise BusError("operator time/transaction budget exhausted")
        result = self.console.command(text.encode("ascii") + b"\r", timeout=5)
        self.transactions += 1
        if not result.startswith(text) or not result.endswith("pios> "):
            raise BusError("console framing failed")
        result = result[len(text):-6].strip()
        if result.startswith("ERR"):
            raise BusError(result)
        return result

    def peek(self, address):
        result = self.command(f"peek 0x{address:x} 4")
        prefix = f"0x{address:016X} = 0x"
        if not result.startswith(prefix):
            raise BusError(f"unexpected read: {result}")
        return int(result[len(prefix):], 16)

    def poke(self, address, value):
        result = self.command(f"poke 0x{address:x} 0x{value:x} 4")
        if not result.startswith("OK:"):
            raise BusError(result)

    def health(self):
        current = self.status()
        if current["version"] != self.before["version"] or current["uptime"] < self.before["uptime"]:
            raise BusError("board rebooted")
        if current["diag"]["error"] or current["perf"]["nic_rx_wedge"]:
            raise BusError("management health degraded")
        link = self.peek(0x1000114068)
        if link == 0xFFFFFFFF or link & 0x30 != 0x30:
            raise BusError("PCIe1 link lost")
        aer = self.command("pcie1 aer")
        if "uncorr=00000000 corr=00000000" not in aer:
            raise BusError(f"PCIe error: {aer}")

    def select(self, bdf, reg):
        bus, dev, fn = bdf
        if not 1 <= bus <= MAX_BUS or not 0 <= dev < 32 or not 0 <= fn < 8 or reg not in (0, 4, 8, 12, 0x18):
            raise BusError("config access outside contract")
        self.poke(0x1000119000, (bus << 20) | (dev << 15) | (fn << 12))

    def read(self, bdf, reg):
        self.select(bdf, reg)
        return self.peek(0x1000118000 + reg)

    def write(self, bdf, reg, value):
        if reg != 0x18:
            raise BusError("only bridge bus numbers may be written")
        self.health()
        hdr, command = self.read(bdf, 12), self.read(bdf, 4)
        if hdr == 0xFFFFFFFF or (hdr >> 16) & 0x7F != 1 or command & 7:
            raise BusError("bridge not quiescent")
        self.record(bdf=bdf, reg=reg, value=value, phase="before")
        self.select(bdf, reg)
        self.poke(0x1000118018, value)
        self.record(bdf=bdf, phase="ack")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        print("No hardware access. --execute permits bridge bus-number writes only.")
        return
    with args.log.open("x", encoding="ascii") as log:
        with socket.create_connection((args.host, 2323), timeout=5) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            console = DebugConn(sock)
            console.read_until(b"debug> ")
            if "debug console unlocked" not in console.command(b"unlock pios\r"):
                raise BusError("console locked")
            board = Board(args.host, console, log)
            try:
                result = configure(board)
                board.health()
                board.record(result=result)
                print(json.dumps(result, indent=2), flush=True)
            except Exception as error:
                board.record(failure=str(error), retry_allowed=False)
                raise


if __name__ == "__main__":
    main()
