"""Exercise CYW key wire layout and the firmware-acknowledgement gate."""
from pathlib import Path
import subprocess
import tempfile
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "cyw43.c").read_text(encoding="utf-8")


def extract(signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
            if depth == 0:
                return source[start:end + 1]
    raise AssertionError(signature)


harness = r"""
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#define CYW_MAC_LEN 6
#define CYW_LINK_JOINING 1
#define CYW_LINK_UP 2
#define CYW_LINK_AUTH_FAIL 3
""" + extract("struct cyw_wpa_host {") + ";\n" + extract("struct cyw_wsec_key {") + r""";
static struct cyw_wpa_host wpa_host;
static u32 cyw_link, replies, rejection;
static u64 now, join_deadline_ms;
static u8 wire[164];
static u32 writes;
static void uart_puts(const char *s) { (void)s; }
static u64 timer_monotonic_ms(void) { return now; }
static bool cyw_join_fail(void) {
    cyw_link = CYW_LINK_AUTH_FAIL;
    memset(&wpa_host, 0, sizeof(wpa_host));
    return false;
}
static bool bcdc_set_iovar_request(const char *name, const void *p, u32 len,
                                    bool wait, u16 *id) {
    assert(strcmp(name,"wsec_key") == 0 && !wait);
    assert(len == sizeof(wire));
    memcpy(wire,p,len);
    *id = (u16)(40 + writes++);
    return true;
}
static bool bcdc_take_response(u16 id, u8 *p, u32 *len, u32 *status) {
    assert(p == NULL && len == NULL);
    assert(id == 40 || id == 41);
    if (!(replies & (1U << (id - 40)))) return false;
    replies &= ~(1U << (id - 40));
    *status = rejection;
    return true;
}
""" + extract("static bool wpa_install_key(") + "\n" + \
    extract("static void wpa_keys_poll(void)") + "\n" + \
    extract("void cyw43_check_timeouts(void)") + r"""
int main(void) {
    assert(sizeof(struct cyw_wsec_key) == 164);
    assert(offsetof(struct cyw_wsec_key, algo) == 112);
    assert(offsetof(struct cyw_wsec_key, flags) == 116);
    assert(offsetof(struct cyw_wsec_key, rxiv_hi) == 140);
    assert(offsetof(struct cyw_wsec_key, peer) == 156);
    u8 key[16] = {1,2,3}, peer[6] = {2,3,4,5,6,7};
    assert(wpa_install_key(0,key,16,true,peer,0,&wpa_host.key_request_id[0]));
    assert(memcmp(wire+8,key,16) == 0 && wire[112] == 4 && wire[116] == 0);
    assert(memcmp(wire+156,peer,6) == 0);
    assert(wpa_install_key(2,key,16,false,NULL,0x060504030201ULL,
                           &wpa_host.key_request_id[1]));
    assert(wire[0] == 2 && wire[112] == 4 && wire[116] == 2 && wire[132] == 1);
    assert(wire[140] == 3 && wire[143] == 6 && wire[144] == 1 && wire[145] == 2);
    assert(!wpa_install_key(0,key,33,true,peer,0,&wpa_host.key_request_id[0]));
    assert(writes == 2);
    wpa_host.keys_pending = true; cyw_link = CYW_LINK_JOINING;
    wpa_keys_poll();
    assert(cyw_link == CYW_LINK_JOINING && !wpa_host.keys_installed);
    replies = 1; wpa_keys_poll();
    assert(cyw_link == CYW_LINK_JOINING && wpa_host.key_ack_mask == 1);
    replies = 2; wpa_keys_poll();
    assert(cyw_link == CYW_LINK_UP && wpa_host.keys_installed);
    wpa_host.keys_pending = true; wpa_host.key_ack_mask = 0;
    cyw_link = CYW_LINK_JOINING; rejection = 1; replies = 1;
    wpa_keys_poll();
    assert(cyw_link == CYW_LINK_AUTH_FAIL && !wpa_host.keys_installed);
    cyw_link = CYW_LINK_JOINING; join_deadline_ms = 100; now = 100;
    cyw43_check_timeouts();
    assert(cyw_link == CYW_LINK_AUTH_FAIL);
    cyw_link = CYW_LINK_JOINING; join_deadline_ms = 200;
    wpa_host.keys_pending = true; wpa_host.key_deadline_ms = 110; now = 110;
    cyw43_check_timeouts();
    assert(cyw_link == CYW_LINK_AUTH_FAIL);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-cyw-key-") as tmp:
    cfile = Path(tmp) / "keys.c"
    exe = Path(tmp) / "keys.exe"
    cfile.write_text(harness, encoding="utf-8")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    str(cfile), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("CYW keys: 164-byte ABI, key acknowledgement gate and deadlines passed")
