#!/usr/bin/env python3
"""One GK107 VRAM-only copy-channel brick test, with PCI Bus Master off.

Register/descriptor reference: Nouveau gk104 FIFO + gf100/gk104 VMM and
NVIDIA class A06F/A0B5 headers. No host memory is mapped into GPU page tables.
This is operator bring-up, not the runtime Tensor provider or IRQ adapter.
"""
import argparse
import json
from pathlib import Path
import re
import struct
import time
import urllib.request

from pios_kepler_post import Board

INST = 0x100000
BAR_INST = 0x101000
RUNLIST = 0x102000
USERD = 0x103000
GPFIFO = 0x104000
PUSH = 0x105000
SOURCE = 0x106000
DEST = 0x107000
FENCE = 0x108000
PGD = 0x120000
SPT = 0x140000
BAR_PGD = 0x160000
BAR_SPT = 0x180000
TABLE_BYTES = 128 * 1024
VA_LIMIT = 64 * 1024 * 1024
TOKEN = 0xC0010001


def page_entry(pa):
    if pa & 4095 or not 0x100000 <= pa < 0x500000:
        raise ValueError("PTE is not inside the reserved VRAM arena")
    return (pa >> 8) | 1 | (1 << 32)  # valid, VRAM aperture=0, volatile


def words(values):
    if any(not 0 <= n <= 0xFFFFFFFF for n in values):
        raise ValueError("invalid method word")
    return struct.pack("<" + "I" * len(values), *values)


def method(offset, values):
    if offset & 3 or offset < 0 or offset >= 0x2000 or not 0 < len(values) <= 2047:
        raise ValueError("invalid method packet")
    return [0x20000000 | (len(values) << 16) | (4 << 13) | (offset >> 2), *values]


def descriptors():
    pages = [GPFIFO, PUSH, SOURCE, DEST, FENCE]
    table = [(SPT + (pa // 4096) * 8, struct.pack("<Q", page_entry(pa))) for pa in pages]
    table += [(PGD, struct.pack("<II", 0, (SPT >> 8) | 1)),
              (BAR_PGD, struct.pack("<II", 0, (BAR_SPT >> 8) | 1)),
              (BAR_SPT, struct.pack("<Q", page_entry(USERD)))]
    instance = bytearray(4096)
    fields = {0x08: USERD, 0x0C: 0, 0x10: 0xFACE, 0x30: 0xFFFFF902,
              0x48: GPFIFO, 0x4C: 9 << 16, 0x84: 0x20400000,
              0x94: 0x30000FFF, 0x9C: 0x100, 0xAC: 0x1F,
              0xE4: 0x20, 0xE8: 0, 0xB8: 0xF8000000,
              0xF8: 0x10003080, 0xFC: 0x10000010}
    for offset, value in fields.items():
        struct.pack_into("<I", instance, offset, value)
    struct.pack_into("<QQ", instance, 0x200, PGD, VA_LIMIT - 1)
    bar = bytearray(4096)
    struct.pack_into("<QQ", bar, 0x200, BAR_PGD, 4095)
    stream = method(0, [0xA0B5])
    stream += method(0x240, [0, FENCE, TOKEN])
    stream += method(0x400, [0, SOURCE + 32, 0, DEST + 32, 64, 64, 64, 1])
    # Non-pipelined, flush, one-word completion, pitched input/output.
    stream += method(0x300, [2 | 4 | 8 | 0x80 | 0x100])
    command = words(stream)
    if len(command) > 4096:
        raise ValueError("pushbuffer page exceeded")
    table += [(INST, bytes(instance)), (BAR_INST, bytes(bar)),
              (RUNLIST, words([0, 0])), (PUSH, command),
              (GPFIFO, words([PUSH, len(stream) << 10]))]
    return table


class Channel(Board):
    def __init__(self, host, log):
        super().__init__(host, b"", log, True)
        self.generation = 0

    def begin_posted(self, for_copy=True):
        self.deadline = time.monotonic() + 180
        with urllib.request.urlopen(f"http://{self.host}/api/status", timeout=5) as r:
            status = json.load(r)
        self.version, self.last_uptime = status["version"], status["uptime"]
        s = self.command("kepler probe")
        if "kepler probe ok" not in s or " posted=1 " not in s or "command=00000002" not in s:
            raise RuntimeError("posted K2000 with Bus Master off required")
        self.generation = int(re.search(r"generation=(\d+)", s)[1])
        if self.read32(0x800000) or self.read32(0x800004) & (0xFFFFFFFF if for_copy else 1):
            raise RuntimeError("channel 0 already in use; refusing to replace it")
        if self.read32(0x204):
            raise RuntimeError("PBDMA is already enabled")
        if not for_copy and self.read32(0x200) & 0x1000:
            raise RuntimeError("GR already enabled; code upload refused")
        if self.read32(0x1704) or self.read32(0x2254):
            raise RuntimeError("BAR1/USERD already owned; refusing to replace")
        if self.read32(0x100C80) & 1 != 1:
            raise RuntimeError("expected 64 KiB large-page mode")
        if self.read32(0x2398) != 0x10:
            raise RuntimeError("PBDMA2 is not routed solely to CE0 runlist4")
        enum = None
        ce0 = None
        for i in range(64):
            data = self.read32(0x22700 + i * 4)
            if data & 3 == 2:
                enum = data
            elif data & 3 == 3 and (data & 0x7FFFFFFC) >> 2 == 1:
                ce0 = enum
        if ce0 is None or ce0 & 0x3C != 0x3C or \
                ((ce0 >> 26) & 15, (ce0 >> 21) & 15,
                 (ce0 >> 15) & 31, (ce0 >> 9) & 31) != (4, 4, 5, 6):
            raise RuntimeError("CE0 TOP engine/runlist/interrupt/reset profile differs")
        if self.read32(0x200) & 0x40:
            raise RuntimeError("CE0 already enabled; refusing to take ownership")
        self.check_health()

    def command(self, text):
        if len(text.encode("ascii")) >= 128:
            raise ValueError("command exceeds the shared terminal line")
        if text.startswith("kepler vram "):
            self.record(command=text, phase="before")
        return super().command(text)

    def check_health(self):
        super().check_health()
        aer = self.command("pcie1 aer")
        if not re.search(r"uncorr=0{8}\b", aer) or not re.search(r"corr=0{8}\b", aer):
            raise RuntimeError(f"PCIe status not clean: {aer}")

    def memread(self, addr):
        reply = self.command(f"kepler vram read {self.generation} {addr}")
        match = re.fullmatch(r"kepler vram ok ([0-9a-fA-F]{256})", reply)
        if not match:
            raise RuntimeError(f"VRAM read failed: {reply}")
        return bytes.fromhex(match[1])

    def memwrite(self, addr, data):
        if addr & 3 or len(data) & 3:
            raise ValueError("unaligned VRAM publication")
        while data:
            base = addr & ~31
            read_base = base & ~127
            within = base - read_base
            count = min(len(data), 32 - (addr - base))
            block = bytearray(self.memread(read_base)[within:within + 32])
            block[addr - base:addr - base + count] = data[:count]
            reply = self.command(f"kepler vram write {self.generation} {base} {block.hex()}")
            if reply != "kepler vram ok":
                raise RuntimeError(f"VRAM write failed: {reply}")
            if self.memread(read_base)[within:within + 32] != block:
                raise RuntimeError("VRAM publication readback mismatch")
            addr += count
            data = data[count:]

    def zero(self, start, length):
        for offset in range(0, length, 1024):
            reply = self.command(f"kepler vram zero {self.generation} {start + offset} {min(1024,length-offset)}")
            if reply != "kepler vram ok":
                raise RuntimeError(reply)

    def poll(self, reg, mask, expected, seconds=0.5):
        end = min(self.deadline, time.monotonic() + seconds)
        for _ in range(64):
            value = self.read32(reg)
            if value & mask == expected:
                return value
            if time.monotonic() >= end:
                break
            time.sleep(0.005)
        raise RuntimeError(f"GPU timeout reg={reg:#x}, mask={mask:#x}, value={value:#x}")

    def flush_bar(self):
        self.write32(0x70000, 1)
        self.poll(0x70000, 2, 0)

    def flush_vm(self, pgd, hub=False):
        end = time.monotonic() + 0.5
        for _ in range(64):
            if self.read32(0x100C80) & 0xFF0000:
                break
            if time.monotonic() >= end:
                raise RuntimeError("GPU MMU has no invalidate slots")
        else:
            raise RuntimeError("GPU MMU has no invalidate slots")
        self.write32(0x100CB8, (pgd >> 12) << 4)
        self.write32(0x100CBC, 0x80000001 | (4 if hub else 0))
        self.poll(0x100C80, 0x8000, 0x8000)

    def diagnose(self):
        result = {}
        for reg in (0x2100, 0x252C, 0x254C, 0x259C, 0x2284 + 4 * 8, 0x800000, 0x800004,
                    0x2640 + 4 * 8, 0x44108, 0x44148, 0x44150, 0x44154,
                    0x44000, 0x44014, 0x44048, 0x4404C, 0x44120, 0x3088,
                    0x28F0, 0x28F4, 0x28F8, 0x28FC,
                    0x2800, 0x104908, 0x104F14):
            result[hex(reg)] = hex(self.read32(reg))
        self.record(diagnostic=result)
        print(json.dumps(result), flush=True)


def run(board):
    board.begin_posted()
    plan = descriptors()
    print("Staging VRAM-only instance and page tables", flush=True)
    board.zero(INST, 0x10000)
    for base in (PGD, SPT, BAR_PGD, BAR_SPT):
        board.zero(base, TABLE_BYTES)
    for address, data in plan:
        for offset in range(0, len(data), 32):
            block = data[offset:offset + 32]
            if any(block):
                board.memwrite(address + offset, block)
    source = words([0xA55A0000 ^ (i * 0x01010101) for i in range(16)])
    guards = b"\xCC" * 32
    board.memwrite(SOURCE, guards + source + guards)
    board.memwrite(DEST, guards + b"\x00" * 64 + guards)
    board.memwrite(FENCE, words([0]))
    # Prepublish before channel activation. No speculative MMIO doorbell:
    # normal Nouveau runtime GPPut stores go through its BAR1 USERD mapping.
    board.memwrite(USERD + 0x8C, words([1]))
    board.flush_bar()
    board.flush_vm(PGD)
    board.flush_vm(BAR_PGD, True)
    saved = {reg: board.read32(reg) for reg in
             (0x204, 0x2140, 0x2A04, 0x2630, 0x4413C, 0x4410C, 0x4414C,
              0x200, 0x140)}
    published = False
    try:
        board.write32(0x140, 0)
        board.write32(0x2140, 0)   # no GPU interrupts routed by this brick test
        board.write32(0x200, saved[0x200] | 0x40)  # TOP CE0 reset/enable bit
        if board.read32(0x200) & 0x40 == 0:
            raise RuntimeError("CE0 enable readback failed")
        board.write32(0x104904, 0)
        board.write32(0x104908, 0xFFFFFFFF)
        board.write32(0x204, saved[0x204] | 4)
        board.write32(0x4413C, saved[0x4413C] & ~0x10000100)
        board.write32(0x44108, 0xFFFFFFFF)
        board.write32(0x4410C, 0)
        board.write32(0x44148, 0xFFFFFFFF)
        board.write32(0x4414C, 0)
        board.write32(0x2A04, 0)  # runlist completion is inspected, not interrupt-driven
        board.write32(0x1704, 0x80000000 | (BAR_INST >> 12))
        board.write32(0x2254, 0x10000000)  # USERD at VA 0 in BAR1 GPU VMM
        board.write32(0x800004, 4 << 16)
        board.write32(0x800000, 0x80000000 | (INST >> 12))
        board.write32(0x2630, saved[0x2630] & ~(1 << 4))
        board.write32(0x800004, (4 << 16) | 0x400)
        board.flush_bar()
        published = True
        board.write32(0x2270, RUNLIST >> 12)  # target 0 = VRAM, never host
        board.write32(0x2274, (4 << 20) | 1)
        board.poll(0x22A4, 0x100000, 0)
        print("Checking one prepublished VRAM copy", flush=True)
        end = time.monotonic() + 2
        for _ in range(32):
            fence = struct.unpack_from("<I", board.memread(FENCE))[0]
            if fence == TOKEN:
                break
            if time.monotonic() >= end:
                raise RuntimeError(f"copy completion absent: fence={fence:#x}")
            time.sleep(0.01)
        else:
            raise RuntimeError("copy completion iteration bound")
        result = board.memread(DEST)
        if result != guards + source + guards:
            raise RuntimeError("copy output or red zones differ")
        if board.memread(SOURCE) != guards + source + guards:
            raise RuntimeError("copy changed source or its guards")
        board.check_health()
        board.record(copy_bytes=64, fence=TOKEN, guards_unchanged=True,
                     source_unchanged=True, phase="copy-complete")
    except Exception as exc:
        board.record(failure=str(exc), retry_allowed=False)
        board.deadline = time.monotonic() + 10
        board.diagnose()
        raise
    finally:
        board.deadline = time.monotonic() + 15
        board.check_health()
        # Stop and remove the channel before releasing its bindings.
        board.write32(0x800004, board.read32(0x800004) | 0x800)
        board.write32(0x2634, 0)  # Nouveau gf100_chan_preempt(), channel 0
        board.poll(0x2634, 0x100000, 0)
        if published:
            board.write32(0x2270, RUNLIST >> 12)
            board.write32(0x2274, 4 << 20)
            board.poll(0x22A4, 0x100000, 0)
        board.write32(0x800000, 0)
        board.write32(0x2254, 0)
        board.write32(0x1704, 0)
        for reg, value in saved.items():
            board.write32(reg, value)
        board.check_health()
    board.record(result={"copy_bytes": 64, "cleanup_complete": True,
                         "pci_dma_enabled": False, "compute_ready": False})
    print("VRAM copy known-answer and guards passed; host DMA remains disabled", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    plan = descriptors()
    if not args.execute:
        print(json.dumps({"descriptor_spans": len(plan), "host_memory_mapped": False,
                          "execute": False, "compute_ready": False}))
        return
    with args.log.open("x", encoding="utf-8") as log:
        run(Channel(args.host, log))


if __name__ == "__main__":
    main()
