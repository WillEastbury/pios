#!/usr/bin/env python3
"""Run a reviewed K2000 VBIOS POST in bounded operator-driven transactions.

This bring-up harness does not enable PCI bus mastering, DMA or compute.
Default is structural preflight only. Execution requires an exact ROM digest
and a fresh cold-card check; failure is not retried or repaired automatically.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import time
import urllib.request

from kepler_post import Post, PostError
from pios_kepler_rom import MAX_BYTES, validate_rom, terminal

BAR0 = 0x1B80000000


class Board:
    def __init__(self, host, rom, log, execute):
        self.host, self.rom, self.log, self.execute = host, rom, log, execute
        self.transactions = 0
        self.deadline = 0
        self.version = None
        self.last_uptime = 0
        self.gpio = []
        self.i2c_primary = None

    def word(self, offset):
        if offset < 0 or offset + 2 > len(self.rom):
            raise PostError("board table pointer out of ROM")
        return int.from_bytes(self.rom[offset:offset + 2], "little")

    def table(self, pointer, min_header, min_stride, max_count):
        if pointer < 0 or pointer + 4 > len(self.rom):
            raise PostError("board table out of ROM")
        version, header, count, stride = self.rom[pointer:pointer + 4]
        if header < min_header or stride < min_stride or count > max_count or \
                pointer + header + count * stride > len(self.rom):
            raise PostError("board table length/count invalid")
        return version, header, count, stride

    def preflight(self, names):
        # Validate the particular board's GPIO/I2C routing, not a generic
        # guessed pinout. Other layouts require a separately reviewed adapter.
        dcb = self.word(0x36)
        version, header, _, _ = self.table(dcb, 12, 8, 32)
        if version != 0x40:
            raise PostError("unsupported GK107 DCB")
        if "GPIO" in names:
            gpio = self.word(dcb + 10)
            version, header, count, stride = self.table(gpio, 6, 5, 32)
            if version != 0x41:
                raise PostError("unsupported GPIO table")
            seen_lines = set()
            for i in range(count):
                p = gpio + header + i * stride
                entry = int.from_bytes(self.rom[p:p + 4], "little")
                function = (entry >> 8) & 255
                if function == 255:
                    continue
                line = entry & 63
                if line >= 32 or line in seen_lines:
                    raise PostError("duplicate or out-of-range GK107 GPIO line")
                seen_lines.add(line)
                default = int(bool(entry & 128))
                logic = (self.rom[p + 4] >> (4 + 2 * default)) & 3
                self.gpio.append((line, logic, (entry >> 16) & 255,
                                  (entry >> 24) & 31))
        if "I2C_BYTE" in names:
            i2c = self.word(dcb + 4)
            version, header, count, stride = self.table(i2c, 5, 4, 32)
            primary = self.rom[i2c + 4] & 15
            if version != 0x40 or primary >= count:
                raise PostError("unsupported primary I2C layout")
            p = i2c + header + primary * stride
            if self.rom[p + 3] != 5 or self.rom[p] & 15 != 2 or self.rom[p + 1] & 1:
                raise PostError("I2C is not the reviewed unshared GK107 drive 2")
            self.i2c_primary = primary

    def record(self, **event):
        self.log.write(json.dumps(event) + "\n")
        self.log.flush()

    def check_time(self):
        if time.monotonic() >= self.deadline or self.transactions >= 50000:
            raise PostError("operator transaction/time limit exceeded")

    def command(self, text):
        if not self.execute:
            raise PostError("no hardware access in structural preflight")
        self.check_time()
        result = self.transport(text)
        self.transactions += 1
        if result.startswith("ERR:") or result == "unknown command":
            raise PostError(result)
        return result

    def transport(self, text):
        return terminal(self.host, text)

    def read(self, offset, width):
        text = self.command(f"peek 0x{BAR0 + offset:x} {width}")
        match = re.fullmatch(r"0x([0-9A-Fa-f]+) = 0x([0-9A-Fa-f]+)", text)
        if not match or int(match[1], 16) != BAR0 + offset:
            raise PostError("unexpected MMIO response")
        value = int(match[2], 16)
        self.record(read=offset, width=width, value=value)
        return value

    def write(self, offset, value, width):
        self.record(write=offset, width=width, value=value, phase="before")
        text = self.command(f"poke 0x{BAR0 + offset:x} 0x{value:x} {width}")
        if not text.startswith("OK:"):
            raise PostError("MMIO write was not acknowledged")
        self.record(write=offset, phase="acknowledged")

    def read32(self, offset):
        return self.read(Post.register(offset), 4)

    def write32(self, offset, value):
        self.write(Post.register(offset), value, 4)
        if self.transactions % 64 == 0:
            self.check_health()

    def check_health(self):
        self.check_time()
        with urllib.request.urlopen(f"http://{self.host}/api/status", timeout=5) as r:
            state = json.load(r)
        if state.get("version") != self.version or state["uptime"] < self.last_uptime:
            raise PostError("board rebooted or changed image during POST")
        self.last_uptime = state["uptime"]
        if state["diag"]["error"] != 0 or state["perf"]["nic_rx_wedge"] != 0:
            raise PostError("management health degraded")
        self.command("poke 0x1000119000 0x00100000 4")
        cmd = self.command("peek 0x1000118004 4")
        if int(cmd.split("= 0x")[1], 16) & 7 != 2:
            raise PostError("GPU PCI command no longer memory-only")

    def port_read(self, port):
        if port not in (0x3C3, 0x3C5, 0x3CF, 0x3D5):
            raise PostError("unreviewed VGA read port")
        return self.read(0x601000 + port, 1)

    def port_write(self, port, value):
        if port not in (0x3C3, 0x3C4, 0x3C5, 0x3CE, 0x3CF, 0x3D4, 0x3D5):
            raise PostError("unreviewed VGA write port")
        if not 0 <= value <= 255:
            raise PostError("invalid VGA value")
        self.write(0x601000 + port, value, 1)

    def vga_read(self, port, index):
        self.port_write(port, index)
        return self.port_read(port + 1)

    def vga_write(self, port, index, value):
        self.port_write(port, index)
        self.port_write(port + 1, value)

    def gpio_reset(self, rom):
        if rom != self.rom:
            raise PostError("GPIO ROM changed")
        for line, logic, function, input_line in self.gpio:
            reg = 0xD610 + 4 * line
            direction, output = bool(logic & 2), logic & 1
            value = (int(not direction) << 13) | (output << 12)
            self.write32(reg, (self.read32(reg) & ~0x3000) | value)
            self.write32(0xD604, self.read32(0xD604) | 1)
            self.write32(reg, (self.read32(reg) & ~255) | function)
            if input_line:
                mux = 0xD740 + 4 * (input_line - 1)
                self.write32(mux, (self.read32(mux) & ~255) | line)

    def i2c(self, bus, address, reg, value=None):
        if bus not in (0x80, 0xFF, self.i2c_primary) or address != 0x4C or reg != 9:
            raise PostError("I2C operation outside reviewed POST profile")
        verb = f"read {reg}" if value is None else f"write {reg} {value}"
        result = self.command(f"kepler i2c {verb}")
        match = re.fullmatch(r"kepler i2c ok value=(\d+)", result)
        if not match or int(match[1]) > 255:
            raise PostError(f"GPU I2C failed: {result}")
        self.record(i2c_bus=bus, address=address, register=reg,
                    operation="read" if value is None else "write",
                    value=int(match[1]))
        return int(match[1])

    def i2c_read(self, bus, address, reg):
        return self.i2c(bus, address, reg)

    def i2c_write(self, bus, address, reg, value):
        self.i2c(bus, address, reg, value)

    def delay_us(self, usec):
        self.check_time()
        if usec > 2_000_000:
            raise PostError("single wait exceeds limit")
        time.sleep(usec / 1_000_000)
        self.check_time()

    def begin(self):
        self.deadline = time.monotonic() + Post.MAX_SECONDS
        with urllib.request.urlopen(f"http://{self.host}/api/status", timeout=5) as r:
            state = json.load(r)
        self.version = state["version"]
        self.last_uptime = state["uptime"]
        probe = self.command("kepler probe")
        if "kepler probe ok" not in probe or " posted=0 " not in probe or \
                "command=00000002" not in probe:
            raise PostError("a cold K2000 with bus mastering off is required")
        self.check_health()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rom", type=Path)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--log", type=Path, required=True,
                        help="new local-only transaction log; refuses overwrite")
    args = parser.parse_args()
    with args.rom.open("rb") as f:
        rom = f.read(MAX_BYTES + 1)
    if hashlib.sha256(rom).hexdigest() != args.expected_sha256.lower():
        parser.error("ROM digest mismatch")
    report = validate_rom(rom)
    # Interpreter pointers are relative to the first, legacy image only.
    legacy = rom[:report["images"][0]["bytes"]]
    with args.log.open("x", encoding="utf-8") as log:
        board = Board(args.host, legacy, log, args.execute)
        post = Post(rom, board)
        print(f"Preflight passed: {len(post.records)} instruction addresses, "
              f"{len(board.gpio)} GPIOs; live execution={args.execute}", flush=True)
        if not args.execute:
            return 0
        board.begin()
        try:
            result = post.run()
            board.check_health()
            marker = board.read32(0x2240C)
            if not marker & 2:
                raise PostError("scripts ended but POST marker remains clear")
            result["post_marker"] = marker
            result["compute_ready"] = False
            board.record(result=result)
            print(json.dumps(result), flush=True)
        except Exception as exc:
            board.record(failure=str(exc), pc=post.pc, steps=post.steps,
                         writes=post.writes, retry_allowed=False)
            raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
