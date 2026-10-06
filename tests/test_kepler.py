"""Run the real diagnostic backend against fake PCIe/MMIO, no DMA."""
from pathlib import Path
import subprocess
import tempfile
import re
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
harness = r"""
#include <assert.h>
#include "types.h"
#include "platform.h"
#include "kepler.h"
#include "lzero.h"
static u32 actual_core, pci_id=0x0ffe10de, command=2, bar=0x80000000;
static u32 shadow, boot0=0x0e73e0a2, faults, reads, maps;
static bool translated=true, linked=true;
static u32 i2c_drive=7, i2c_writes;
static u64 clock_us;
static bool i2c_stuck, i2c_nack;
static u32 gpu_post, selector, vram_words[256], vram_writes;
static bool corrupt_vram;
u64 timer_monotonic_ms(void) { return clock_us / 1000; }
void timer_delay_us(u64 us) { clock_us += us; }
void mmio_write(u64 addr,u32 value) {
    if(addr==PIOS_PCIE1_CPU_WIN_BASE+0x1700) { selector=value; return; }
    if(addr>=PIOS_PCIE1_CPU_WIN_BASE+0x700000 &&
       addr<PIOS_PCIE1_CPU_WIN_BASE+0x700400) {
        assert(selector==0x10);
        vram_words[(addr-PIOS_PCIE1_CPU_WIN_BASE-0x700000)/4]=value ^ corrupt_vram;
        vram_writes++;return;
    }
    assert(addr==PIOS_PCIE1_CPU_WIN_BASE+0xd054);
    i2c_drive=value; i2c_writes++;
}
u32 pios_host_core_id(void) { return actual_core; }
void uart_puts(const char *s) { (void)s; }
void lzero_status(struct lzero_status *out) {
    *out=(struct lzero_status){0};
    out->gpu_found=true; out->vendor_id=0x10de; out->device_id=0x0ffe;
    out->gpu_bus=1; out->bar0_size=0x1000000; out->bar0_mapped=true;
}
bool lzero_map_bar0(void) { maps++; return true; }
bool pcie1_link_up(void) { return linked; }
u32 pcie1_cfg_read(u32 b,u32 d,u32 f,u32 reg) {
    assert(b==1 && d==0 && f==0);
    if (reg==0) return pci_id;
    if (reg==4) return command;
    if (reg==0x10) return bar;
    assert(reg==0x50); return shadow;
}
void pcie1_aer_snapshot(struct pcie1_aer_snapshot *s, bool clear) {
    assert(!clear); *s=(struct pcie1_aer_snapshot){0}; s->uncorr=faults;
}
bool mmu_device_read32_valid(u64 addr) {
    assert(addr>=PIOS_PCIE1_CPU_WIN_BASE);
    return translated;
}
bool lzero_bar0_read32(u32 off,u32 *out) {
    reads++;
    if(off==0) *out=boot0;
    else if(off==0x2240c) *out=gpu_post;
    else if(off==0x200) *out=0xd0002020;
    else if(off==0x22500) *out=0x200;
    else { assert(off==0x619f04); *out=1; }
    return true;
}
u32 mmio_read(u64 addr) {
    u64 off=addr-PIOS_PCIE1_CPU_WIN_BASE;
    if(off==0x1700) return selector;
    if(off==0x2240c) return gpu_post;
    if(off==0x22438 || off==0x2243c) return 2;
    if(off==0x22554) return 0;
    if(off==0x11020c || off==0x11120c) return 1024;
    if(off==0x619f04) return 1;
    if(off>=0x700000 && off<0x700400) {
        assert(selector==0x10);return vram_words[(off-0x700000)/4];
    }
    if (addr==PIOS_PCIE1_CPU_WIN_BASE+0xd054)
        return i2c_drive | (!i2c_stuck && (i2c_drive & 1) ? 0x10 : 0) |
               (i2c_nack ? 0x20 : 0);
    assert(addr>=PIOS_PCIE1_CPU_WIN_BASE+0x300000 &&
           addr<PIOS_PCIE1_CPU_WIN_BASE+0x400000);
    reads++; return 0xffffffff; /* erased ROM data is legal */
}
int main(void) {
    struct kepler_status s;
    u8 output[132]; memset(output,0,sizeof(output)); output[128]=0xac;
    assert(kepler_rom_read(0,output,128)==KEPLER_NOT_READY);
    actual_core=1; assert(kepler_probe()==KEPLER_WRONG_OWNER && maps==0);
    actual_core=0; command=6;
    assert(kepler_probe()==KEPLER_DMA_ENABLED && maps==0);
    command=2; assert(kepler_probe()==KEPLER_OK);
    kepler_snapshot(&s);
    assert(s.probe_ok && !s.posted && s.rom_available && !s.compute_ready);
    assert(s.boot0==boot0 && s.bdf==0x100);
    assert(kepler_rom_read(0,output,128)==KEPLER_OK);
    assert(output[0]==255 && output[127]==255 && output[128]==0xac);
    u32 before=reads;
    assert(kepler_rom_read(1,output,128)==KEPLER_RANGE);
    assert(kepler_rom_read(0,output,129)==KEPLER_RANGE);
    assert(kepler_rom_read(0xffffc,output,128)==KEPLER_RANGE);
    assert(kepler_rom_read(0,output,0)==KEPLER_RANGE);
    assert(kepler_rom_read(0,NULL,4)==KEPLER_RANGE && reads==before);
    actual_core=1;
    assert(kepler_rom_read(0,output,4)==KEPLER_WRONG_OWNER && reads==before);
    actual_core=0; shadow=1;
    assert(kepler_rom_read(0,output,4)==KEPLER_ROM_SHADOWED && reads==before);
    shadow=0; assert(kepler_probe()==KEPLER_OK);
    before=reads; faults=1;
    assert(kepler_rom_read(0,output,4)==KEPLER_AER && reads==before);
    faults=0; assert(kepler_probe()==KEPLER_OK);
    before=reads; translated=false;
    assert(kepler_rom_read(0,output,4)==KEPLER_MAPPING && reads==before);
    translated=true; boot0=0xe4000a1;
    assert(kepler_probe()==KEPLER_IDENTITY);
    kepler_snapshot(&s); assert(!s.probe_ok && !s.compute_ready);
    boot0=0x0e73e0a2; assert(kepler_probe()==KEPLER_OK);
    u8 byte=0;
    assert(kepler_i2c_byte(false,0,&byte)==KEPLER_RANGE && i2c_writes==0);
    assert(kepler_i2c_byte(false,9,&byte)==KEPLER_OK && byte==0);
    assert(clock_us<2000 && i2c_drive==7);
    byte=0x40; clock_us=0;
    assert(kepler_i2c_byte(true,9,&byte)==KEPLER_OK && i2c_drive==7);
    i2c_nack=true;
    assert(kepler_i2c_byte(false,9,&byte)==KEPLER_I2C_NACK && i2c_drive==7);
    assert(kepler_probe()==KEPLER_OK); i2c_nack=false; i2c_stuck=true;
    clock_us=0;
    assert(kepler_i2c_byte(false,9,&byte)==KEPLER_I2C_TIMEOUT);
    assert(i2c_drive==7 && clock_us<=220 && i2c_writes>0);
    gpu_post=2; i2c_stuck=false; assert(kepler_probe()==KEPLER_OK);
    kepler_snapshot(&s);selector=0x30;
    memset(output,0xa5,128);
    assert(kepler_vram(s.generation,1,0x100000,output,128)==KEPLER_OK);
    assert(selector==0x30 && vram_writes==32);
    memset(output,0,128);
    assert(kepler_vram(s.generation,0,0x100000,output,128)==KEPLER_OK);
    assert(output[0]==0xa5 && output[127]==0xa5 && selector==0x30);
    assert(kepler_vram(s.generation,2,0x100000,NULL,1024)==KEPLER_OK);
    assert(vram_words[0]==0 && vram_words[255]==0);
    u32 old_writes=vram_writes;
    assert(kepler_vram(s.generation-1,1,0x100000,output,128)==KEPLER_NOT_READY);
    assert(kepler_vram(s.generation,1,0x1ffffc,output,128)==KEPLER_RANGE);
    assert(kepler_vram(s.generation,1,0x500000,output,128)==KEPLER_RANGE);
    assert(kepler_vram(s.generation,1,0xfffffffc,output,128)==KEPLER_RANGE);
    assert(kepler_vram(s.generation,1,0x100000,output,132)==KEPLER_RANGE);
    assert(kepler_vram(s.generation,1,0,NULL,4)==KEPLER_RANGE);
    assert(vram_writes==old_writes && selector==0x30);
    corrupt_vram=true;
    assert(kepler_vram(s.generation,2,0x100000,NULL,4)==KEPLER_VRAM_VERIFY);
    assert(selector==0x30);
    corrupt_vram=false;
    assert(kepler_probe()==KEPLER_OK);
    kepler_snapshot(&s);old_writes=vram_writes;
    gpu_post=0;
    assert(kepler_vram(s.generation,1,0x100000,output,128)==KEPLER_NOT_READY);
    assert(vram_writes==old_writes);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-kepler-") as tmp:
    tmp = Path(tmp)
    (tmp / "mmio.h").write_text('#include "types.h"\nu32 mmio_read(u64);\nvoid mmio_write(u64,u32);\n')
    (tmp / "mmu.h").write_text('#include "types.h"\nbool mmu_device_read32_valid(u64);\n')
    (tmp / "test.c").write_text(harness)
    exe = tmp / "test.exe"
    subprocess.run([find_clang(), "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-DPIOS_PLATFORM=1", "-DPIOS_HOST_CORE_ID_FN",
                    "-include", str(ROOT / "tests/stubinc/types.h"),
                    "-I", str(tmp), "-I", str(ROOT / "include"),
                    str(tmp / "test.c"), str(ROOT / "src/kepler.c"),
                    "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("Kepler: real diagnostic backend identity, bounds, no-DMA and MMIO gates passed")

kernel = (ROOT / "src/kernel.c").read_text()


def function(name):
    start = kernel.index("static ", kernel.index(name + "(") - 20)
    body = kernel.index("{", start)
    depth = 1
    end = body + 1
    while depth:
        depth += (kernel[end] == "{") - (kernel[end] == "}")
        end += 1
    return kernel[start:end]


start_marker = '} else if (http_starts_with(cmd, "kepler vram ")) {'
start = kernel.index(start_marker) + len(start_marker)
end = kernel.index('} else if (http_starts_with(cmd, "kepler i2c ")) {', start)
response = kernel[kernel.index("static u32 http_build_terminal_response("):]
capacity = int(re.search(r"char cmd\[(\d+)\];", response)[1])
parser = r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "kepler.h"
static u32 calls;
u32 pios_strlen(const char *s) { return (u32)strlen(s); }
static void http_append(char *o,u32 *n,u32 cap,const char *s) {
    while(*s && *n+1<cap) o[(*n)++]=*s++;
    o[*n]=0;
}
static void http_append_hex8(char *o,u32 *n,u32 cap,u8 x) {
    (void)x; http_append(o,n,cap,"00");
}
const char *kepler_result_name(enum kepler_result r) { (void)r; return "failure"; }
enum kepler_result kepler_vram(u32 gen,u32 mode,u32 off,u8 *data,u32 len) {
    assert(gen==0xffffffffU && mode==1 && off==1312768 && len==32);
    for(u32 i=0;i<len;i++) assert(data[i]==0xab);
    calls++; return KEPLER_OK;
}
"""
parser += "\n".join(function(name) for name in
                    ("http_parse_u32", "http_streq", "http_split_args",
                     "pixe_hex_nibble", "pixe_parse_hex_byte_pair"))
parser += "\nstatic void parse(char *cmd) { char out[1024]; u32 len=0,max=sizeof(out);\n"
parser += kernel[start:end] + "\n}\n"
parser += f"#define CMD_BYTES {capacity}\n"
parser += r"""
int main(void) {
    char cmd[CMD_BYTES], hex[257]; memset(hex,'a',sizeof(hex));
    for(u32 i=0;i<32;i++) { hex[2*i]='a'; hex[2*i+1]='b'; }
    hex[64]=0;
    int length=snprintf(cmd,sizeof(cmd),"kepler vram write 4294967295 1312768 %s",hex);
    assert(length>0 && (u32)length<sizeof(cmd));
    parse(cmd); assert(calls==1);
    hex[63]='g';
    snprintf(cmd,sizeof(cmd),"kepler vram write 4294967295 1312768 %s",hex);
    parse(cmd); assert(calls==1);
    memset(hex,'a',128);hex[128]=0;
    length=snprintf(cmd,sizeof(cmd),"kepler vram write 4294967295 1312768 %s",hex);
    assert((u32)length>=sizeof(cmd));
    parse(cmd); assert(calls==1);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-kepler-parser-") as tmp:
    tmp = Path(tmp)
    (tmp / "test.c").write_text(parser)
    exe = tmp / "test.exe"
    subprocess.run([find_clang(), "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-include", str(ROOT / "tests/stubinc/types.h"),
                    "-I", str(ROOT / "include"), str(tmp / "test.c"),
                    "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("Kepler terminal: actual HTTP capacity and production parser accept complete writes only")
