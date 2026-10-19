#!/usr/bin/env python3
"""Offline Intel-generated Xe2 oracle. Supply the pinned header; no network/I/O
to a board. Derived C++ stays under the caller's repository-local workdir."""
import argparse
import hashlib
import pathlib
import subprocess

PIN = "694afcd0816c977da61fed759513921159044dcfa4d841dac3d1683eb2062e0c"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("header", type=pathlib.Path)
    parser.add_argument("--workdir", type=pathlib.Path,
                        default=pathlib.Path("tests/_build/walker_oracle"))
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parent.parent
    work = args.workdir.resolve()
    if not work.is_relative_to(root):
        parser.error("workdir must be inside the repository")
    raw = args.header.read_bytes()
    if hashlib.sha256(raw).hexdigest() != PIN:
        parser.error("generated Xe2 header does not match pinned SHA-256")
    text = raw.decode()
    names = ("POSTSYNC_DATA", "INTERFACE_DESCRIPTOR_DATA", "COMPUTE_WALKER")
    definitions = []
    for name in names:
        start = text.index("typedef struct tag" + name + " {")
        end = text.index("\nSTATIC_ASSERT(", start)
        definitions.append(text[start:end])
    prefix = """#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <type_traits>
#define BITFIELD_RANGE(a,b) ((b)-(a)+1)
#define UNRECOVERABLE_IF(x) do { if (x) std::abort(); } while (0)
#pragma pack(push,1)
"""
    forwards = "\n".join(f"typedef struct tag{n} {n};" for n in names)
    main_cpp = """
#pragma pack(pop)
static_assert(sizeof(COMPUTE_WALKER)==160);
int main() {
    for (unsigned c=0;c<2;c++) {
        auto w=COMPUTE_WALKER::sInit();
        w.setIndirectDataLength(224);
        w.setIndirectDataStartAddress(c ? 0xfffffec0U : 0x800U);
        w.setMessageSimd(COMPUTE_WALKER::MESSAGE_SIMD_SIMT32);
        w.setEmitInlineParameter(true);
        w.setSimdSize(COMPUTE_WALKER::SIMD_SIZE_SIMT32);
        w.setExecutionMask(0xffff);
        w.setThreadGroupIdXDimension(c ? 4096 : 3);
        w.setThreadGroupIdYDimension(1);
        w.setThreadGroupIdZDimension(1);
        auto &id=w.getInterfaceDescriptor();
        id.setKernelStartPointer(c ? 0xfffffac0ULL : 0x140ULL);
        id.setNumberOfThreadsInGpgpuThreadGroup(1);
        id.setThreadGroupDispatchSize(
            INTERFACE_DESCRIPTOR_DATA::THREAD_GROUP_DISPATCH_SIZE_TG_SIZE_1);
        auto *p=reinterpret_cast<unsigned char *>(w.getInlineDataPointer());
        for (unsigned i=0;i<32;i++) p[i]=static_cast<unsigned char>(i+c*128);
        std::printf("case%u:",c);
        auto *b=reinterpret_cast<unsigned char *>(&w);
        for (unsigned i=0;i<sizeof(w);i++) std::printf("%02x",b[i]);
        std::puts("");
    }
}
"""
    work.mkdir(parents=True, exist_ok=True)
    source = work / "oracle.cpp"
    binary = work / "oracle.exe"
    source.write_text(prefix + forwards + "\n" + "\n".join(definitions) +
                      main_cpp, encoding="utf-8")
    subprocess.run(["clang++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(source), "-o", str(binary)], check=True)
    result = subprocess.check_output([str(binary)], text=True)
    fixture = root / "tests" / "fixtures" / "intel_bmg_walker.hex"
    if result.replace("\r\n", "\n") != fixture.read_text():
        raise SystemExit("FAIL: generated-header bytes differ from fixture")
    print("PASS: both complete 160-byte walkers match pinned Intel oracle")


if __name__ == "__main__":
    main()
