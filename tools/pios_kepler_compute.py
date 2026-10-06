#!/usr/bin/env python3
"""Single owned GK107 compute-channel proof; no host DMA or app capability."""
import argparse
import json
from pathlib import Path
import re
import socket
import struct
import time
import urllib.request

import pios_kepler_channel as fifo
from pios_kepler_gr import GR, Reference
from pios_kepler_load import bundle
from kepler_kernels import build as kernels
from debug_console_qemu_test import DebugConn

QMD = 0x240000
DONE = 0x241000
PAGEPOOL = 0x340000
BUNDLE = 0x348000
ATTRIB = 0x350000
PATCH = 0x380000
TLS = 0x3C0000
CONTEXT = 0x480000
CONTEXT_BYTES = 0x11A00
DONE_VALUE = 0xC0010002
CHANNEL = 1
USERD = fifo.USERD + 0x200


def patch_registers(ltc0, ltc1):
    return [(0x40800C, PAGEPOOL >> 8), (0x408010, 0x80000000),
            (0x419004, PAGEPOOL >> 8), (0x419008, 0),
            (0x4064CC, 0x80000000),
            (0x408004, BUNDLE >> 8), (0x408008, 0x80000030),
            (0x418808, BUNDLE >> 8), (0x41880C, 0x80000030),
            (0x4064C8, (0x180 << 16) | 0x600),
            (0x418810, 0x80000000 | (ATTRIB >> 12)),
            (0x419848, 0x10000000 | (ATTRIB >> 12)),
            (0x405830, (0x218 << 16) | 0x648),
            (0x4064C4, ((0x648 // 4) << 16) | 0xFFFF),
            (0x5030C0, (1 << 28) | (0x430 << 16)),
            (0x5030E4, (0xC90 << 16) | 0x648),
            (0x17E91C, ltc0), (0x17E920, ltc1)]


def qmd(profile, code_offset):
    result = 0

    def field(lo, width, value):
        nonlocal result
        if not 0 <= value < (1 << width) or lo + width > 2048:
            raise ValueError("QMD field overflow")
        result |= value << lo

    for bit in range(250, 256):
        field(bit, 1, 1)
    field(202, 1, 1)  # RELEASE0
    field(256, 32, code_offset)
    field(366, 1, 1)
    field(368, 2, 1)
    field(378, 1, 1)
    for lo, width in ((384, 32), (416, 16), (432, 16)):
        field(lo, width, 1)
    field(592, 16, profile["threads"])
    field(608, 16, 1)
    field(624, 16, 1)
    field(640, 1, 1)
    field(669, 3, 1)
    field(736, 32, DONE)
    field(799, 1, 1)
    field(800, 32, DONE_VALUE)
    field(928, 32, kernels.PARAMS)
    field(975, 17, 256)
    field(1496, 8, profile["registers"])
    field(1504, 24, 0x800)
    field(1528, 8, 0x30)
    return result.to_bytes(256, "little")


def pushbuffer():
    def method(address, data):
        if address & 3 or not 0 <= address < 0x2000 or not 0 < len(data) <= 2047:
            raise ValueError("invalid compute method")
        return [0x20000000 | (len(data) << 16) | (1 << 13) | (address >> 2), *data]

    data = method(0, [0xA0C0])
    data += method(0x790, [0, TLS])
    data += method(0x2E4, [0, 0x20000, 2])
    data += method(0x2F0, [0, 0x20000, 2])
    data += method(0x77C, [0xFF000000])
    data += method(0x214, [0xFE000000])
    data += method(0x1608, [0, kernels.CODE_BASE])
    data += method(0x310, [0x300])
    data += method(0x21C, [0x1011])
    data += method(0x2B4, [QMD >> 8])
    data += method(0x2BC, [3])
    data += method(0x110, [0])
    return fifo.words(data)


def mapping_pages():
    ranges = [(fifo.GPFIFO, 4096, False), (fifo.PUSH, 4096, False),
              (kernels.CODE_BASE, 0x3000, False), (kernels.PARAMS, 4096, False),
              (kernels.INPUT_A, 0x10000, False), (kernels.INPUT_B, 0x2000, False),
              (QMD, 0x2000, False), (PAGEPOOL, 0x8000, True),
              (BUNDLE, 0x3000, True), (ATTRIB, 0x2D000, False),
              (PATCH, 0x1000, True), (TLS, 0x40000, False),
              (0x400000, 0x92000, True)]
    result = {}
    for address, length, privileged in ranges:
        if address & 4095 or length & 4095:
            raise ValueError("unaligned GPU range")
        for page in range(address, address + length, 4096):
            if page in result:
                raise ValueError("overlapping GPU range")
            result[page] = fifo.page_entry(page) | (2 if privileged else 0)
    return result


def stage_context_memory(board):
    instance = bytearray(dict(fifo.descriptors())[fifo.INST])
    struct.pack_into("<I", instance, 8, USERD)
    struct.pack_into("<I", instance, 0xE8, CHANNEL)
    struct.pack_into("<Q", instance, 0x210, CONTEXT | 4)
    for start, length in ((fifo.INST, 0x10000), (fifo.PGD, fifo.TABLE_BYTES),
                          (fifo.SPT, fifo.TABLE_BYTES), (fifo.BAR_PGD, fifo.TABLE_BYTES),
                          (fifo.BAR_SPT, fifo.TABLE_BYTES), (PAGEPOOL, 0x8000),
                          (BUNDLE, 0x3000), (ATTRIB, 0x2D000), (PATCH, 4096),
                          (TLS, 0x40000), (0x400000, 0x92000)):
        board.zero(start, length)
    board.memwrite(fifo.PGD, struct.pack("<II", 0, (fifo.SPT >> 8) | 1))
    board.memwrite(fifo.BAR_PGD, struct.pack("<II", 0, (fifo.BAR_SPT >> 8) | 1))
    board.memwrite(fifo.BAR_SPT, struct.pack("<Q", fifo.page_entry(fifo.USERD)))
    for address, value in mapping_pages().items():
        board.memwrite(fifo.SPT + (address // 4096) * 8, struct.pack("<Q", value))
    for address, data in ((fifo.INST, instance),
                          (fifo.BAR_INST, dict(fifo.descriptors())[fifo.BAR_INST])):
        for offset in range(0, len(data), 32):
            if any(data[offset:offset + 32]):
                board.memwrite(address + offset, data[offset:offset + 32])
    board.flush_bar()
    board.flush_vm(fifo.PGD)
    board.flush_vm(fifo.BAR_PGD, True)


def floorsweep(board):
    for sm in range(2):
        for reg in (0x504698 + sm * 0x800, 0x5044E8 + sm * 0x800,
                    0x500C10 + sm * 4, 0x504088 + sm * 0x800):
            board.write32(reg, sm)
    for reg in (0x500C08, 0x500C8C):
        board.write32(reg, 2)
    for base in (0x406028, 0x405870):
        for i in range(4):
            board.write32(base + i * 4, 2 if i == 0 else 0)
    for reg, value in ((0x418BB8, 0x201), (0x41BFD0, 0x700201),
                       (0x41BFE4, 0), (0x4078BC, 0x201), (0x405B00, 0x201)):
        board.write32(reg, value)
    for base in (0x418B08, 0x41BF00, 0x40780C):
        for i in range(6):
            board.write32(base + i * 4, 0)
    for i in range(32):
        board.write32(0x406800 + i * 0x20, 0)
        board.write32(0x406C00 + i * 0x20, 3)
    for i in range(8):
        board.write32(0x4064D0 + i * 4, 0)
    board.mask(0x419F78, 9, 0)


def create_context(board, reference):
    print("Staging contained GR context mappings", flush=True)
    stage_context_memory(board)
    board.write32(0x404170, 0x12)
    board.poll(0x404170, 0x10, 0, seconds=2)
    board.write32(0x409614, 0x70)
    time.sleep(0.00001)
    board.mask(0x409614, 0x700, 0x700)
    time.sleep(0.00001)
    board.read32(0x409614)
    board.write32(0x404170, 0x10)
    board.poll(0x404170, 0x10, 0, seconds=2)
    board.write32(0x40802C, 1)
    board.write32(0x409840, 0x80000000)
    board.write32(0x409500, 0x80000000 | (fifo.INST >> 12))
    board.write32(0x409504, 1)
    board.poll(0x409800, 0x80000000, 0x80000000, seconds=2)
    print("Generating initial GR register/context image", flush=True)
    board.write32(0x260, 0)
    for name in ("hub", "gpc_0", "zcull", "gpc_1", "tpc", "ppc"):
        board.mmio_pack(reference, ("gf100" if name == "zcull" else "gk104") + "_grctx_pack_" + name)
    board.idle()
    timeout = board.read32(0x404154)
    board.write32(0x404154, 0)
    patches = patch_registers(board.read32(0x17E91C), board.read32(0x17E920))
    for reg, value in patches:
        board.write32(reg, value)
    for reg, mask in ((0x418C6C, 1), (0x41980C, 0x10), (0x41BE08, 4),
                      (0x4064C0, 0x80000000), (0x405800, 0x08000000), (0x419C00, 8)):
        board.mask(reg, mask, mask)
    floorsweep(board)
    board.idle()
    board.write32(0x400208, 0x80000000)
    prior = None
    for address, value, _ in reference.entries("gk104_grctx_pack_icmd"):
        if value != prior:
            board.write32(0x400204, value)
            prior = value
        board.write32(0x400200, address)
        if address & 0xFFFF == 0xE100:
            board.idle()
        board.poll(0x400700, 4, 0, seconds=2)
    board.write32(0x400208, 0)
    board.write32(0x404154, timeout)
    prior = None
    for address, value, cls in reference.entries("gk104_grctx_pack_mthd"):
        if value != prior:
            board.write32(0x40448C, value)
            prior = value
        board.write32(0x404488, 0x80000000 | cls | (address << 14))
    board.write32(0x260, 1)
    board.idle()
    board.mask(0x409B04, 0x80000000, 0)
    board.write32(0x409000, 0x100)
    board.poll(0x409B00, 0x80000000, 0, seconds=2)
    board.flush_bar()
    if not any(board.memread(CONTEXT + 0x100)):
        raise RuntimeError("golden GR context remained empty")
    board.memwrite(PATCH, b"".join(struct.pack("<II", reg, value) for reg, value in patches))
    board.memwrite(CONTEXT, struct.pack("<II", len(patches), PATCH >> 8))
    board.flush_bar()
    board.record(context_ready=True, context_bytes=CONTEXT_BYTES, patches=len(patches))
    print("GR context saved and patch list published", flush=True)


def launch(board, entry, code, case):
    actual_code = b"".join(board.memread(entry["vram_offset"] + i)
                           for i in range(0, len(code), 128))[:len(code)]
    if actual_code != code:
        raise RuntimeError("prepared kernel differs in VRAM")
    board.zero(kernels.PARAMS, 256)
    board.memwrite(kernels.PARAMS, bytes.fromhex(case["params_hex"]))
    for address, field in ((kernels.INPUT_A, "input_a_hex"), (kernels.INPUT_B, "input_b_hex")):
        data = bytes.fromhex(case[field])
        if data:
            board.memwrite(address, data + b"\0" * (-len(data) % 4))
    expected = bytes.fromhex(case["expected_le_hex"])
    board.memwrite(kernels.OUTPUT - 64, b"\xCC" * 64 + b"\xCD" * len(expected) + b"\xCC" * 64)
    board.zero(DONE, 128)
    board.memwrite(QMD, qmd(entry, entry["vram_offset"] - kernels.CODE_BASE))
    stream = pushbuffer()
    board.memwrite(fifo.PUSH, stream)
    board.memwrite(fifo.GPFIFO, fifo.words([fifo.PUSH, (len(stream) // 4) << 10]))
    board.memwrite(fifo.RUNLIST, fifo.words([CHANNEL, 0]))
    board.memwrite(USERD + 0x8C, fifo.words([1]))
    board.flush_bar()
    board.write32(0x2140, 0)
    board.write32(0x204, 1)
    board.mask(0x4013C, 0x10000100, 0)
    for reg in (0x40108, 0x40148):
        board.write32(reg, 0xFFFFFFFF)
    for reg in (0x4010C, 0x4014C, 0x2A04):
        board.write32(reg, 0)
    board.write32(0x1704, 0x80000000 | (fifo.BAR_INST >> 12))
    board.write32(0x2254, 0x10000000)
    board.write32(0x80000C, 0)
    board.write32(0x800008, 0x80000000 | (fifo.INST >> 12))
    board.mask(0x2630, 1, 0)
    board.write32(0x80000C, 0x400)
    board.write32(0x2270, fifo.RUNLIST >> 12)
    board.write32(0x2274, 1)
    board.poll(0x2284, 0x100000, 0, seconds=2)
    print(f"Published {entry['name']} QMD", flush=True)
    end = time.monotonic() + 3
    for _ in range(64):
        if struct.unpack_from("<I", board.memread(DONE))[0] == DONE_VALUE:
            break
        if time.monotonic() >= end:
            raise RuntimeError("compute completion deadline")
        time.sleep(0.01)
    else:
        raise RuntimeError("compute completion iteration bound")
    length = len(expected) + 128
    actual = b"".join(board.memread(kernels.OUTPUT - 64 + i)
                      for i in range(0, length, 128))[:length]
    if actual != b"\xCC" * 64 + expected + b"\xCC" * 64:
        raise RuntimeError("GPU result or red zones differ from CPU reference")
    for address, field in ((kernels.INPUT_A, "input_a_hex"), (kernels.INPUT_B, "input_b_hex")):
        data = bytes.fromhex(case[field])
        actual_input = b"".join(board.memread(address + i) for i in range(0, len(data), 128))
        if actual_input[:len(data)] != data:
            raise RuntimeError("GPU modified a read-only input span")
    board.check_health()
    board.record(shader_passed=entry["name"], output=expected.hex(), guard_bytes=128)
    print(f"GPU {entry['name']} known-answer passed", flush=True)


def proof_case(name):
    if name == "store":
        return kernels.make_case("store_const")
    if name == "vector":
        a = struct.pack("<64I", *([0x7FFFFFFF, 0xFFFFFFFF, 0, 7] * 16))
        b = struct.pack("<64I", *([1, 1, 0x80000000, 0xFFFFFFF9] * 16))
        return kernels.make_case("vector_add_i32", a, b, cols=64)
    if name == "dot":
        return kernels.make_case("dot_i8", b"\x80" * 1024, b"\x80" * 1024, cols=1024)
    if name == "bitnet":
        return kernels.bitnet_case()
    raise ValueError("unsupported proof case")


def attach(board, generation):
    board.deadline = time.monotonic() + 600
    with urllib.request.urlopen(f"http://{board.host}/api/status", timeout=5) as response:
        status = json.load(response)
    board.version, board.last_uptime = status["version"], status["uptime"]
    state = board.command("kepler status")
    if not re.search(rf"\bgeneration={generation}\b", state) or "probe_ok=1" not in state:
        raise RuntimeError("GR ownership generation changed")
    board.generation = generation
    board.memread(kernels.CODE_BASE)
    if board.read32(0x204) or board.read32(0x800008):
        raise RuntimeError("channel/PBDMA already owned")
    if not board.read32(0x200) & 0x1000 or board.read32(0x409800) & 0x80000000 == 0 or \
            board.read32(0x409804) != CONTEXT_BYTES:
        raise RuntimeError("expected initialized GK107 GR context controller")
    board.check_health()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-dir", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, default=Path("build_kepler") / "kernels")
    parser.add_argument("--generation", type=int, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--case", choices=("store", "vector", "dot", "bitnet"), default="store")
    args = parser.parse_args()
    ref = Reference(args.reference_dir)
    images = bundle(args.bundle)
    case = proof_case(args.case)
    image = next((entry, code) for entry, code in images if entry["name"] == case["kernel"])
    pages = mapping_pages()
    if not args.execute:
        print(json.dumps({"pages": len(pages), "push_bytes": len(pushbuffer()),
                          "qmd_bytes": len(qmd(image[0], 0)), "case": args.case, "execute": False}))
        return
    with args.log.open("x", encoding="ascii") as log:
        with socket.create_connection((args.host, 2323), timeout=5) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            console = DebugConn(sock)
            console.read_until(b"debug> ")
            if "debug console unlocked" not in console.command(b"unlock pios\r"):
                raise RuntimeError("console unlock failed")
            board = GR(args.host, log, console)
            attach(board, args.generation)
            try:
                create_context(board, ref)
                launch(board, *image, case)
            except Exception as error:
                board.record(failure=str(error), retry_allowed=False)
                board.console = None
                board.deadline = time.monotonic() + 20
                board.gr_diagnose()
                for reg in (0x2100, 0x259C, 0x400108, 0x40108, 0x40148, 0x800008, 0x80000C,
                            0x2640, 0x2800, 0x2804, 0x2808, 0x280C):
                    print(hex(reg), hex(board.read32(reg)), flush=True)
                raise
            finally:
                board.console = None
                board.deadline = time.monotonic() + 20
                board.write32(0x80000C, board.read32(0x80000C) | 0x800)
                board.write32(0x2634, CHANNEL)
                board.poll(0x2634, 0x100000, 0, seconds=2)
                board.write32(0x2270, fifo.RUNLIST >> 12)
                board.write32(0x2274, 0)
                board.poll(0x2284, 0x100000, 0, seconds=2)
                board.write32(0x800008, 0)
                board.write32(0x204, 0)
                board.write32(0x2254, 0)
                board.write32(0x1704, 0)
                board.check_health()
            result = {"case": args.case, "shader_executed": True,
                      "cleanup_complete": True, "host_dma_enabled": False,
                      "output_le_hex": case["expected_le_hex"]}
            if args.case == "bitnet":
                result.update(argmax=case["argmax"], checksum=case["checksum"])
            board.record(result=result)
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
