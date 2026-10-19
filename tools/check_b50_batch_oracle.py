#!/usr/bin/env python3
"""Compare two complete offline batch fixtures with the pinned Intel header.
This verifies command bytes, not the safety of this inspection-only ordering.
"""
import hashlib
import pathlib
import subprocess
import sys
from check_b50_walker_oracle import PIN

root = pathlib.Path(__file__).resolve().parent.parent
raw = pathlib.Path(sys.argv[1]).read_bytes()
if hashlib.sha256(raw).hexdigest() != PIN:
    raise SystemExit("wrong pinned header digest")
text = raw.decode()
names = ("POSTSYNC_DATA", "INTERFACE_DESCRIPTOR_DATA", "COMPUTE_WALKER",
         "CFE_STATE", "STATE_BASE_ADDRESS", "PIPELINE_SELECT", "PIPE_CONTROL",
         "MI_BATCH_BUFFER_END")
definitions = []
for name in names:
    start = text.index("typedef struct tag" + name + " {")
    definitions.append(text[start:text.index("\nSTATIC_ASSERT(", start)])
prefix = """#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <type_traits>
#define BITFIELD_RANGE(a,b) ((b)-(a)+1)
#define UNRECOVERABLE_IF(x) do { if(x) std::abort(); } while(0)
#pragma pack(push,1)
"""
main = """
#pragma pack(pop)
template<class T> void dump(T &s) {
    auto *p=reinterpret_cast<unsigned char *>(&s);
    for(unsigned i=0;i<sizeof(s);i++) std::printf("%02x",p[i]);
}
int main() {
 for(unsigned c=0;c<2;c++) {
    auto pre=PIPE_CONTROL::sInit(); dump(pre);
    auto ps=PIPELINE_SELECT::sInit();
    ps.setPipelineSelection(PIPELINE_SELECT::PIPELINE_SELECTION_GPGPU);
    ps.setMaskBits(3); dump(ps);
    auto fe=CFE_STATE::sInit();
    fe.setMaximumNumberOfThreads(c?65535:64); dump(fe);
    auto pc=PIPE_CONTROL::sInit();
    pc.setStateCacheInvalidationEnable(true);
    pc.setConstantCacheInvalidationEnable(true);
    pc.setTextureCacheInvalidationEnable(true);
    pc.setInstructionCacheInvalidateEnable(true); dump(pc);
    auto s=STATE_BASE_ADDRESS::sInit();
    s.setInstructionBaseAddress(0x200000);
    s.setInstructionBaseAddressModifyEnable(true);
    s.setInstructionMemoryObjectControlState(2);
    s.setInstructionBufferSize(1);
    s.setInstructionBufferSizeModifyEnable(true);
    s.setGeneralStateBaseAddress(0x300000);
    s.setGeneralStateBaseAddressModifyEnable(true);
    s.setGeneralStateMemoryObjectControlState(4);
    s.setGeneralStateBufferSize(1);
    s.setGeneralStateBufferSizeModifyEnable(true);
    s.setStatelessDataPortAccessMemoryObjectControlState(6);
    s.setL1CacheControlCachePolicy(STATE_BASE_ADDRESS::L1_CACHE_CONTROL_UC); dump(s);
    auto w=COMPUTE_WALKER::sInit();
    w.setIndirectDataLength(224);
    w.setIndirectDataStartAddress(c?3840:128);
    w.setMessageSimd(COMPUTE_WALKER::MESSAGE_SIMD_SIMT32);
    w.setEmitInlineParameter(true);
    w.setSimdSize(COMPUTE_WALKER::SIMD_SIZE_SIMT32);
    w.setExecutionMask(0xffff);
    w.setThreadGroupIdXDimension(1);
    w.setThreadGroupIdYDimension(1);
    w.setThreadGroupIdZDimension(1);
    auto &id=w.getInterfaceDescriptor();
    id.setKernelStartPointer(c?2816:64);
    id.setNumberOfThreadsInGpgpuThreadGroup(1);
    id.setThreadGroupDispatchSize(
        INTERFACE_DESCRIPTOR_DATA::THREAD_GROUP_DISPATCH_SIZE_TG_SIZE_1);
    auto *in=reinterpret_cast<unsigned char *>(w.getInlineDataPointer());
    const uint64_t pointers[2]={0x10000000,0x10001000};
    for(unsigned a=0;a<2;a++)
        for(unsigned b=0;b<8;b++) in[16+a*8+b]=pointers[a]>>(8*b);
    dump(w);
    auto fence=PIPE_CONTROL::sInit();
    fence.setDataportFlush(true);
    fence.setUnTypedDataPortCacheFlush(true);
    fence.setPostSyncOperation(PIPE_CONTROL::POST_SYNC_OPERATION_WRITE_IMMEDIATE_DATA);
    fence.setAddress(c?0xfffffff8:0x500000);
    fence.setAddressHigh(c?0xffff:0);
    fence.setImmediateData(0x123456789abcdef0ULL); dump(fence);
    auto end=MI_BATCH_BUFFER_END::sInit(); dump(end);
    std::puts("");
 }
}
"""
work = root / "tests" / "_build" / "batch_oracle"
work.mkdir(parents=True, exist_ok=True)
source = work / "oracle.cpp"
source.write_text(prefix + "\n".join(f"typedef struct tag{n} {n};" for n in names)
                  + "\n" + "\n".join(definitions) + main)
oracle = work / "oracle.exe"
native = work / "native.exe"
subprocess.run(["clang++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                str(source), "-o", str(oracle)], check=True)
sources = ("tests/test_intel_bmg_batch.c", "src/intel_bmg_batch.c",
           "src/intel_bmg_state.c", "src/intel_bmg_walker.c",
           "src/intel_bmg_tensor_plan.c", "src/intel_bmg_vm.c")
subprocess.run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I" + str(root / "tests" / "stubinc"), "-I" + str(root / "include"),
                *(str(root / p) for p in sources), "-o", str(native)], check=True)
if subprocess.check_output([str(oracle)]) != subprocess.check_output([str(native), "--dump"]):
    raise SystemExit("FAIL: full offline stream differs from Intel oracle")
print("PASS: both complete 352-byte batches match Intel oracle; rejection tests passed")
