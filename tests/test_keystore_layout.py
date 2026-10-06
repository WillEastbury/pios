"""Exercise the real keystore storage/migration paths with injected crypto/I/O."""
from pathlib import Path
import subprocess
import tempfile
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
HARNESS = r"""
#include <assert.h>
#include <string.h>
#include "keystore.h"
#include "crypto.h"
#include "walfs.h"
#include "sd.h"
static u8 current[512], legacy[512], mbr[512], original[512];
static bool mounted=true, decrypt_ok=true, fail_read, fail_write, corrupt_write;
static u32 base=1064960, blocks=4194304, writes, decrypts, seals, legacy_reads;
static u32 current_lba(void) { return base+PIOS_KEYSTORE_OFFSET/512; }
static u32 legacy_lba(void) { return base+PIOS_KEYSTORE_LEGACY_OFFSET/512; }
void walfs_status(struct walfs_status_snapshot *s) {
    *s=(struct walfs_status_snapshot){0};
    s->mounted=mounted; s->partition_lba=base; s->partition_blocks=blocks;
}
bool sd_read_block(u32 lba,u8 *out) {
    if(lba==0) { memcpy(out,mbr,512);return true; }
    if(fail_read) return false;
    if(lba==current_lba()) memcpy(out,current,512);
    else { assert(lba==legacy_lba());legacy_reads++;memcpy(out,legacy,512); }
    return true;
}
bool sd_write_block(u32 lba,const u8 *in) {
    assert(lba==current_lba() && lba!=legacy_lba());
    assert(lba>=base+(PIOS_BOOT_SLOT_B_OFFSET+PIOS_BOOT_SLOT_BYTES)/512);
    writes++;
    if(fail_write) return false;
    memcpy(current,in,512);
    if(corrupt_write) current[128]^=1;
    return true;
}
void simd_zero(void *p,usize n) { memset(p,0,n); }
void simd_memcpy(void *d,const void *s,usize n) { memcpy(d,s,n); }
void uart_puts(const char *s) { (void)s; }
u64 timer_monotonic_ms(void) { return 12345; }
u32 pios_strlen(const char *s) { return (u32)strlen(s); }
void hkdf_extract(const u8 *s,u32 sn,const u8 *i,u32 in,u8 *out) {
    (void)s;(void)sn;(void)i;(void)in;memset(out,0x42,32);
}
void hkdf_expand(const u8 *p,u32 pn,const u8 *i,u32 in,u8 *out,u32 n) {
    (void)p;(void)pn;(void)i;(void)in;memset(out,0x43,n);
}
void hmac_sha256(const u8 *k,u32 kn,const u8 *d,u32 dn,u8 *out) {
    (void)kn;(void)d;(void)dn;memcpy(out,k,32);
}
void aes_gcm_init(struct aes_gcm_ctx *g,const u8 *key,u32 bits) {
    (void)g;(void)key;assert(bits==256);
}
bool aes_gcm_decrypt(struct aes_gcm_ctx *g,const u8 *nonce,u32 nn,
    const u8 *aad,u32 an,const u8 *cipher,u32 n,u8 *plain,const u8 *tag) {
    (void)g;(void)nonce;(void)aad;(void)tag;assert(nn==12 && an==16 && n==32);
    decrypts++;memcpy(plain,cipher,n);return decrypt_ok;
}
bool aes_gcm_encrypt(struct aes_gcm_ctx *g,const u8 *nonce,u32 nn,
    const u8 *aad,u32 an,const u8 *plain,u32 n,u8 *cipher,u8 *tag) {
    (void)g;(void)nonce;(void)aad;assert(nn==12 && an==16 && n==32);
    seals++;memcpy(cipher,plain,n);memset(tag,0x44,16);return true;
}
static void put32(u8 *p,u32 v) { for(u32 i=0;i<4;i++)p[i]=(u8)(v>>(i*8)); }
static void reset(void) {
    memset(current,0,512);memset(legacy,0,512);
    mounted=decrypt_ok=true;fail_read=fail_write=corrupt_write=false;
    writes=decrypts=seals=legacy_reads=0;base=1064960;blocks=4194304;
}
static void record(u8 *sector,u32 generation) {
    memset(sector,0,512);put32(sector,0x5254534b);put32(sector+4,1);
    put32(sector+8,generation);put32(sector+12,1);
    memset(sector+28,0x5a,32);
}
int main(void) {
    mbr[510]=0x55;mbr[511]=0xaa;put32(mbr+440,42);
    put32(mbr + 0x1ce + 8,1064960);put32(mbr + 0x1ce + 12,4194304);
    struct keystore_status status;u8 derived[32];
    reset();record(legacy,9);memcpy(original,legacy,512);
    assert(keystore_init());keystore_status(&status);
    assert(status.generation==9 && status.initialized && status.sealed);
    assert(status.user_records_lba==current_lba() && writes==1 && decrypts==1 && seals==0);
    assert(!memcmp(current,original,512) && !memcmp(legacy,original,512));
    assert(keystore_derive_secret("fixture",derived,32) && derived[0]==0x5a);
    assert(keystore_init() && writes==1 && legacy_reads==1);
    reset();record(legacy,9);decrypt_ok=false;
    assert(!keystore_init() && writes==0 && seals==0);
    reset();record(legacy,9);fail_write=true;
    assert(!keystore_init() && writes==1);keystore_status(&status);
    assert(!status.initialized && !status.sealed);
    reset();record(legacy,9);corrupt_write=true;
    assert(!keystore_init());keystore_status(&status);assert(status.last_error==3);
    reset();record(legacy,9);put32(current,0x12345678);
    assert(!keystore_init() && writes==0 && legacy_reads==0);
    reset();record(current,10);put32(current+4,2);
    assert(!keystore_init() && writes==0 && seals==0);
    reset();record(legacy,9);put32(legacy+4,2);
    assert(!keystore_init() && writes==0 && seals==0);
    reset();record(current,10);decrypt_ok=false;
    assert(!keystore_init() && writes==0 && seals==0);
    reset();memset(current,0xff,512);record(legacy,9);
    assert(keystore_init() && writes==1 && seals==0);
    reset();memset(legacy,0xa9,512);
    assert(keystore_init() && writes==1 && seals==1);
    reset();mounted=false;assert(!keystore_init() && writes==0);
    reset();base=0xfffff000;assert(!keystore_init() && writes==0);
    reset();blocks=PIOS_KEYSTORE_OFFSET/512;assert(!keystore_init() && writes==0);
    reset();base=0;assert(keystore_init() && writes==1);
    reset();fail_read=true;assert(!keystore_init() && writes==0);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-keystore-layout-") as directory:
    directory = Path(directory)
    source, executable = directory / "test.c", directory / "test.exe"
    source.write_text(HARNESS, encoding="ascii")
    declarations = directory / "declarations.h"
    declarations.write_text("u32 pios_strlen(const char *);\n", encoding="ascii")
    subprocess.run([find_clang(), "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-DPIOS_PLATFORM=2", "-include", str(ROOT / "tests" / "stubinc" / "types.h"),
                    "-include", str(declarations),
                    "-I", str(ROOT / "include"), str(source), str(ROOT / "src" / "keystore.c"),
                    "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
print("Keystore: disjoint write LBA, exact migration, reboot/derive, corruption and bounds passed")
