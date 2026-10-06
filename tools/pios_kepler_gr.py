#!/usr/bin/env python3
"""Bounded GK107 GR/Falcon bring-up using hash-pinned Nouveau reference data.

Reference sources retain their MIT notices outside the repository. This tool
does not load proprietary firmware, grant PCI bus mastering or expose GR to apps.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import socket
import time

from pios_kepler_channel import Channel
from debug_console_qemu_test import DebugConn

REVISION = "ff47652a4b66c067c765a7ad464d930b5a9367cc"
SOURCES = {
    "gf100.c": "dbf3d62b04a065127a7872691fd49c7696e584e25e6a6b17c77f7e91de481a47",
    "gf108.c": "703873852db1ca971049823a1b99c5d5b15e0b328112cf5f292b8a6a353d60a0",
    "gf117.c": "fcf619c5071def99c3ce0011d560117b0ac88430a4fe60841a85e8c176594d33",
    "gf119.c": "a5c30dc78e69a7c72a9e44b3511c3a950267ddcb57aa3a8ce9101657f7836641",
    "gk104.c": "71b413d2d0a51e364d6b5bd93f3a59df555995ba603b185a4cf0b98a20f3ae34",
    "ctxgf100.c": "f22a88255ddcaf1089965903a172f79dee6103a1650b431c355d402689c71b6f",
    "ctxgf108.c": "c06cd3ae36dbbe57e44cff12fc513cb7eb5b63f3204126f308aaceebcdd9d989",
    "ctxgf117.c": "f8869c05203618ac55e762d13d31fcd0c93e6bbae600a6898423ce062860033c",
    "ctxgf119.c": "0b101fca41fc239266bb4dd8222fbb0e4f3163c692ae662dfcfe0179cb79fe39",
    "ctxgk104.c": "c2f9b1ff6d1aae759f5d4129ebd588d5e749a580547c296432e249c5e8df04d7",
    "fuc/hubgk104.fuc3.h": "93b6d541e025a5e66079b1307f783313a3e59703eef1bbf7703ec280bc8566e6",
    "fuc/gpcgk104.fuc3.h": "8474328f1496f51cfb6a7c45ba863b60b3fb7eca74ac4cbcf537d967387b3531",
}
DEBUG_READ = 0x300000
DEBUG_WRITE = 0x320000
DEBUG_BYTES = 0x20000


def number(text):
    text = text.strip()
    if not re.fullmatch(r"(?:0x[0-9a-fA-F]+|[0-9]+)", text):
        raise ValueError("non-literal initializer")
    value = int(text, 16 if text.startswith("0x") else 10)
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("initializer outside u32")
    return value


class Reference:
    def __init__(self, directory):
        self.tables, self.packs, self.firmware, self.clock_tables = {}, {}, {}, {}
        for name, digest in SOURCES.items():
            path = directory / ("gr-ref-" + name.replace("/", "-"))
            with path.open("rb") as file:
                data = file.read(100001)
            if len(data) > 100000 or hashlib.sha256(data).hexdigest() != digest:
                raise ValueError(f"unreviewed GR reference: {name}")
            self.parse(data.decode("ascii"))
        self.entries("gk104_gr_pack_mmio")
        for part in ("hub", "gpc_0", "gpc_1", "tpc", "ppc", "icmd", "mthd"):
            self.entries("gk104_grctx_pack_" + part)
        for key in ("gk104_grhub_code", "gk104_grhub_data", "gk104_grgpc_code", "gk104_grgpc_data"):
            if key not in self.firmware or not 0 < len(self.firmware[key]) <= 4096:
                raise ValueError("firmware missing or too large")

    def parse(self, source):
        source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
        pattern = r"struct\s+(gf100_gr_init|gf100_gr_pack|nvkm_therm_clkgate_init|nvkm_therm_clkgate_pack)\s+(\w+)\[\]\s*=\s*\{(.*?)\n\};"
        for kind, name, body in re.findall(pattern, source, re.S):
            rows = [r for r in re.findall(r"\{([^{}]*)\}", body) if r.strip()]
            if kind.endswith("_pack"):
                self.packs[name] = [(r.split(",")[0].strip(),
                                     number(r.split(",")[1]) if len(r.split(",")) > 1 else 0)
                                    for r in rows]
            else:
                table = [tuple(number(x) for x in r.split(",")) for r in rows]
                wanted = 4 if kind == "gf100_gr_init" else 3
                if any(len(row) != wanted for row in table):
                    raise ValueError("bad initializer shape")
                (self.tables if wanted == 4 else self.clock_tables)[name] = table
        for name, body in re.findall(r"uint32_t\s+(\w+)\[\]\s*=\s*\{(.*?)\n\};", source, re.S):
            self.firmware[name] = [number(x) for x in body.split(",") if x.strip()]

    def entries(self, pack):
        if pack not in self.packs:
            raise ValueError(f"missing pack {pack}")
        result = []
        for name, cls in self.packs[pack]:
            if name not in self.tables:
                raise ValueError(f"missing table {name}")
            for address, count, stride, value in self.tables[name]:
                if not 0 < count <= 4096 or not 0 < stride <= 128:
                    raise ValueError("register span exceeds budget")
                if address + (count - 1) * stride > 0xFFFFFF:
                    raise ValueError("register span overflow")
                result.extend((address + i * stride, value, cls) for i in range(count))
                if len(result) > 16384:
                    raise ValueError("register pack exceeds budget")
        return result

    def csdata(self, pack, base):
        addresses = [address - base for address, _, _ in self.entries(pack)]
        if not addresses or any(x < 0 or x & 3 or x >= (1 << 26) for x in addresses):
            raise ValueError("invalid context-save register")
        result = []
        first, prev, count = addresses[0], addresses[0], 1
        for address in addresses[1:]:
            if address == prev + 4 and count < 32:
                count += 1
            else:
                result.append(((count - 1) << 26) | first)
                first, count = address, 1
            prev = address
        result.append(((count - 1) << 26) | first)
        return result


class GR(Channel):
    def __init__(self, host, log, console=None):
        super().__init__(host, log)
        self.console = console

    def transport(self, text):
        if self.console is None:
            return super().transport(text)
        response = self.console.command(text.encode("ascii") + b"\r")
        if not response.startswith(text) or not response.endswith("pios> "):
            raise RuntimeError("unexpected TCP terminal framing")
        return response[len(text):-len("pios> ")].strip()

    def mask(self, reg, mask, value):
        self.write32(reg, (self.read32(reg) & ~mask) | value)

    def idle(self):
        end = time.monotonic() + 2
        for _ in range(128):
            self.read32(0x400700)
            if not self.read32(0x40060C) & 1 and not self.read32(0x2640) & 0x8000:
                return
            if time.monotonic() >= end:
                break
            time.sleep(0.005)
        raise RuntimeError("GR idle deadline")

    def mmio_pack(self, reference, name):
        for reg, value, _ in reference.entries(name):
            if reg & 3 or not 0x400000 <= reg < 0x420000:
                raise ValueError("GR MMIO pack outside engine registers")
            self.write32(reg, value)

    def firmware_load(self, base, data, code):
        self.write32(base + 0x1C0, 0x01000000)
        for value in data:
            self.write32(base + 0x1C4, value)
        self.write32(base + 0x180, 0x01000000)
        for index in range((len(code) + 63) & ~63):
            if index % 64 == 0:
                self.write32(base + 0x188, index // 64)
            self.write32(base + 0x184, code[index] if index < len(code) else 0)

    def csdata_load(self, base, slot, values):
        self.write32(base + 0x1C0, 0x02000000 + slot)
        start = max(self.read32(base + 0x1C4), self.read32(base + 0x1C4))
        minimum = 0x304 if base == 0x409000 else 0x6C
        if start & 3 or start < minimum or start + len(values) * 4 > 0x2000:
            raise RuntimeError("Falcon context-register list exceeds DMEM budget")
        self.write32(base + 0x1C0, 0x01000000 + start)
        for value in values:
            self.write32(base + 0x1C4, value)
        self.write32(base + 0x1C0, 0x01000004 + slot)
        self.write32(base + 0x1C4, start + len(values) * 4)

    def gr_diagnose(self):
        report = {hex(r): hex(self.read32(r)) for r in
                  (0x200, 0x400100, 0x400108, 0x400118, 0x400700, 0x40060C,
                   0x409100, 0x409800, 0x409804, 0x409808, 0x40980C,
                   0x409810, 0x409814, 0x409818, 0x40981C, 0x409C18,
                   0x502100, 0x502800, 0x502804, 0x502808, 0x50280C)}
        self.record(gr_diagnostic=report)
        print(json.dumps(report), flush=True)


def initialize(board, ref):
    # Resolve every list before changing the engine.
    lists = [(0x409000, 0, ref.csdata("gk104_grctx_pack_hub", 0)),
             (0x41A000, 0, ref.csdata("gk104_grctx_pack_gpc_0", 0x418000)),
             (0x41A000, 0, ref.csdata("gk104_grctx_pack_gpc_1", 0x418000)),
             (0x41A000, 4, ref.csdata("gk104_grctx_pack_tpc", 0x419800)),
             (0x41A000, 8, ref.csdata("gk104_grctx_pack_ppc", 0x41BE00))]
    board.begin_posted(for_copy=False)
    board.deadline = time.monotonic() + 180
    saved = {reg: board.read32(reg) for reg in (0x140, 0x200, 0x260)}
    try:
        board.zero(DEBUG_READ, DEBUG_BYTES)
        board.zero(DEBUG_WRITE, DEBUG_BYTES)
        board.write32(0x140, 0)
        board.write32(0x200, saved[0x200] | 0x1000)
        topology = (board.read32(0x409604), board.read32(0x502608) & 255,
                    board.read32(0x500C30))
        if topology != (0x20001, 2, 3):
            raise RuntimeError(f"unreviewed GR topology: {topology}")
        board.mask(0x400500, 0x10001, 0)
        for reg, value in ((0x418880, board.read32(0x100C80) & 1),
                           (0x4188A4, 0x03000000), (0x418888, 0), (0x41888C, 0),
                           (0x418890, 0), (0x418894, 0),
                           (0x4188B4, DEBUG_WRITE >> 8), (0x4188B8, DEBUG_READ >> 8)):
            board.write32(reg, value)
        board.mmio_pack(ref, "gk104_gr_pack_mmio")
        board.idle()
        for name, _ in ref.packs["gk104_clkgate_pack"]:
            for address, count, value in ref.clock_tables[name]:
                for i in range(count):
                    board.write32(address + i * 4, value)
        board.write32(0x503018, 1)
        for i, value in enumerate((0x10, 0, 0, 0)):
            board.write32(0x418980 + i * 4, value)
        for reg, value in ((0x500914, 0x102), (0x500910, 0x40002),
                           (0x500918, 0x400000), (0x41BFD4, 0x400000),
                           (0x4188AC, board.read32(0x100800))):
            board.write32(reg, value)
        for reg in (0x408850, 0x408958):
            board.mask(reg, 15, board.read32(0x120074))
        for reg, value in ((0x400500, 0x10001), (0x400100, 0xFFFFFFFF),
                           (0x40013C, 0xFFFFFFFF), (0x400124, 2),
                           (0x409FFC, 0), (0x409C14, 0x3E3E), (0x409C24, 0xF0001)):
            board.write32(reg, value)
        for reg in (0x404000, 0x404600, 0x408030, 0x406018, 0x404490, 0x405840):
            board.write32(reg, 0xC0000000)
        board.write32(0x407020, 0x40000000)
        board.write32(0x405844, 0xFFFFFF)
        board.mask(0x419CC0, 8, 8)
        board.mask(0x419EB4, 0x1000, 0x1000)
        board.write32(0x503038, 0xC0000000)
        for reg in (0x500420, 0x500900, 0x501028, 0x500824):
            board.write32(reg, 0xC0000000)
        for tpc in range(2):
            base = 0x504000 + tpc * 0x800
            for offset, value in ((0x48C, 0xC0000000), (0x508, 0xFFFFFFFF),
                                  (0x50C, 0xFFFFFFFF), (0x224, 0xC0000000),
                                  (0x084, 0xC0000000), (0x644, 0x1FFFFE), (0x64C, 15)):
                board.write32(base + offset, value)
        for reg in (0x502C90, 0x502C94):
            board.write32(reg, 0xFFFFFFFF)
        for rop in range(2):
            for offset, value in ((0x144, 0x40000000), (0x070, 0x40000000),
                                  (0x204, 0xFFFFFFFF), (0x208, 0xFFFFFFFF)):
                board.write32(0x410000 + rop * 0x400 + offset, value)
        for reg in (0x400108, 0x400138, 0x400118, 0x400130, 0x40011C, 0x400134):
            board.write32(reg, 0xFFFFFFFF)
        board.write32(0x400054, 0x34CE3464)
        # Raster ZBC tables are not consumed by this compute-only bootstrap.
        board.write32(0x260, 0)
        for base, name in ((0x409000, "gk104_grhub"), (0x41A000, "gk104_grgpc")):
            board.firmware_load(base, ref.firmware[name + "_data"], ref.firmware[name + "_code"])
        board.write32(0x260, 1)
        for base, slot, values in lists:
            board.csdata_load(base, slot, values)
        board.write32(0x40910C, 0)
        board.write32(0x409100, 2)
        board.poll(0x409800, 0x80000000, 0x80000000, seconds=2)
        size = board.read32(0x409804)
        if not 4096 <= size <= 0x40000 or size & 3:
            raise RuntimeError(f"invalid GR context size {size:#x}")
        board.check_health()
        result = {"fecs_ready": True, "context_bytes": size, "gpcs": 1, "tpcs": 2,
                  "generation": board.generation, "shader_executed": False,
                  "pci_dma_enabled": False, "compute_ready": False}
        board.record(result=result)
        return result
    except Exception as error:
        board.record(failure=str(error), retry_allowed=False)
        board.console = None  # Independent HTTP path for fault capture/reset.
        board.deadline = time.monotonic() + 15
        try:
            board.gr_diagnose()
        finally:
            board.write32(0x200, saved[0x200])
            board.write32(0x260, saved[0x260])
            board.write32(0x140, saved[0x140])
            board.check_health()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-dir", type=Path, required=True)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    ref = Reference(args.reference_dir)
    print(json.dumps({"reference": REVISION, "tables": len(ref.tables),
                      "firmware_words": {key: len(value) for key, value in ref.firmware.items()},
                      "execute": args.execute}), flush=True)
    if args.execute:
        with args.log.open("x", encoding="ascii") as log:
            with socket.create_connection((args.host, 2323), timeout=5) as sock:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                console = DebugConn(sock)
                console.read_until(b"debug> ")
                unlocked = console.command(b"unlock pios\r")
                if "debug console unlocked" not in unlocked:
                    raise RuntimeError("TCP console did not unlock")
                print(json.dumps(initialize(GR(args.host, log, console), ref)), flush=True)


if __name__ == "__main__":
    main()
