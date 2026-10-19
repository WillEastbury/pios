#!/usr/bin/env python3
"""Compare native state encoders directly with the pinned Intel generated header."""
import hashlib
import pathlib
import subprocess
import sys

from check_b50_walker_oracle import PIN

root = pathlib.Path(__file__).resolve().parent.parent
raw = pathlib.Path(sys.argv[1]).read_bytes()
if hashlib.sha256(raw).hexdigest() != PIN:
    raise SystemExit("wrong pinned header digest")
names = ("CFE_STATE", "STATE_BASE_ADDRESS", "PIPELINE_SELECT", "PIPE_CONTROL",
         "MI_BATCH_BUFFER_START", "MI_BATCH_BUFFER_END")
text = raw.decode()
definitions = []
for name in names:
    start = text.index("typedef struct tag" + name + " {")
    definitions.append(text[start:text.index("\nSTATIC_ASSERT(", start)])
prefix = """#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#define BITFIELD_RANGE(a,b) ((b)-(a)+1)
#define UNRECOVERABLE_IF(x) do { if(x) std::abort(); } while(0)
#pragma pack(push,1)
"""
main = """
#pragma pack(pop)
template<class T> void dump(T &s) {
    auto *p=reinterpret_cast<unsigned char *>(&s);
    for(unsigned i=0;i<sizeof(s);i++) std::printf("%02x",p[i]);
    std::puts("");
}
int main() {
 for(unsigned c=0;c<2;c++) {
    auto fe=CFE_STATE::sInit();
    fe.setMaximumNumberOfThreads(c?65535:64); dump(fe);
    auto s=STATE_BASE_ADDRESS::sInit();
    s.setInstructionBaseAddress(c?0xfffffff00000ULL:0x200000ULL);
    s.setInstructionBaseAddressModifyEnable(true);
    s.setInstructionMemoryObjectControlState(c?126:2);
    s.setInstructionBufferSize(c?255:2);
    s.setInstructionBufferSizeModifyEnable(true); dump(s);
    auto ps=PIPELINE_SELECT::sInit();
    ps.setPipelineSelection(PIPELINE_SELECT::PIPELINE_SELECTION_GPGPU);
    ps.setMaskBits(3); dump(ps);
    auto pc=PIPE_CONTROL::sInit();
    if(c) {
      pc.setStateCacheInvalidationEnable(true);
      pc.setConstantCacheInvalidationEnable(true);
      pc.setTextureCacheInvalidationEnable(true);
      pc.setInstructionCacheInvalidateEnable(true);
      pc.setDataportFlush(true);
      pc.setUnTypedDataPortCacheFlush(true);
      pc.setAmfsFlushEnable(true);
      pc.setPostSyncOperation(PIPE_CONTROL::POST_SYNC_OPERATION_WRITE_IMMEDIATE_DATA);
      pc.setAddress(0xfff00008U);
      pc.setAddressHigh(0xffffU);
      pc.setImmediateData(0x123456789abcdef0ULL);
    }
    dump(pc);
    auto bs=MI_BATCH_BUFFER_START::sInit();
    bs.setAddressSpaceIndicator(MI_BATCH_BUFFER_START::ADDRESS_SPACE_INDICATOR_PPGTT);
    bs.setBatchBufferStartAddress(c?0xfffffff00004ULL:0x400000ULL); dump(bs);
    auto be=MI_BATCH_BUFFER_END::sInit(); dump(be);
 }
 for(unsigned c=0;c<2;c++) {
    auto s=STATE_BASE_ADDRESS::sInit();
    s.setInstructionBaseAddress(c?0xfffffff00000ULL:0x200000ULL);
    s.setInstructionBaseAddressModifyEnable(true);
    s.setInstructionMemoryObjectControlState(c?126:2);
    s.setInstructionBufferSize(c?255:2);
    s.setInstructionBufferSizeModifyEnable(true);
    s.setGeneralStateBaseAddress(c?0xfffe00000000ULL:0x300000ULL);
    s.setGeneralStateBaseAddressModifyEnable(true);
    s.setGeneralStateMemoryObjectControlState(c?124:4);
    s.setGeneralStateBufferSize(c?256:1);
    s.setGeneralStateBufferSizeModifyEnable(true);
    s.setStatelessDataPortAccessMemoryObjectControlState(c?122:6);
    s.setL1CacheControlCachePolicy(c?STATE_BASE_ADDRESS::L1_CACHE_CONTROL_WS:
                                   STATE_BASE_ADDRESS::L1_CACHE_CONTROL_UC);
    dump(s);
 }
}
"""
work = root / "tests" / "_build" / "state_oracle"
work.mkdir(parents=True, exist_ok=True)
source = work / "oracle.cpp"
source.write_text(prefix + "\n".join(f"typedef struct tag{n} {n};" for n in names)
                  + "\n" + "\n".join(definitions) + main)
oracle = work / "oracle.exe"
native = work / "native.exe"
subprocess.run(["clang++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                str(source), "-o", str(oracle)], check=True)
subprocess.run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I" + str(root / "tests" / "stubinc"), "-I" + str(root / "include"),
                str(root / "tests" / "test_intel_bmg_state.c"),
                str(root / "src" / "intel_bmg_state.c"), "-o", str(native)], check=True)
expected = subprocess.check_output([str(oracle)])
actual = subprocess.check_output([str(native)])
if expected != actual:
    raise SystemExit("FAIL: native command bytes differ from Intel oracle")
print("PASS: six state commands plus full IOH/general SBA, two cases, every byte; rejection tests passed")
