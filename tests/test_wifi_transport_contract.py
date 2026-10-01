"""Exercise the real SDHCI mask helpers against a read-only card-IRQ latch."""
from pathlib import Path
import re
import subprocess
import tempfile

from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
sdio = (ROOT / "src" / "sdio.c").read_text(encoding="utf-8")
cyw = (ROOT / "src" / "cyw43.c").read_text(encoding="utf-8")


def function(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
            if not depth:
                return source[start:end + 1]
    raise AssertionError(signature)


helpers = "\n".join(function(sdio, signature) for signature in (
    "void sdio_card_irq_ack(void)",
    "void sdio_card_irq_mask(void)",
    "static void sdio_card_irq_host_unmask(void)",
))
harness = r"""
#include <stdint.h>
#include <assert.h>
typedef uint16_t u16;
typedef uint32_t u32;
#define PIOS_HAS_WIFI_SDIO 1
#define PIOS_HAS_GIC 1
#define PIOS_WIFI_SDIO_IRQ 306
#define REG_INTERRUPT 0
#define REG_IRPT_MASK 1
#define REG_IRPT_EN 2
#define INT_CARD 0x100
static u16 regs[3] = {INT_CARD, 0xffff, 0x100};
static int card_asserted;
static u16 sr16(u32 off) { return regs[off]; }
static void sw16(u32 off, u16 value) {
    assert(off != REG_INTERRUPT); /* Card status is read-only, never W1C. */
    regs[off] = value;
    if (off == REG_IRPT_MASK) {
        if (!(value & INT_CARD)) regs[REG_INTERRUPT] &= ~INT_CARD;
        else if (card_asserted) regs[REG_INTERRUPT] |= INT_CARD;
    }
}
""" + helpers + r"""
int main(void) {
    sdio_card_irq_mask();
    assert(!(regs[REG_INTERRUPT] & INT_CARD));
    assert(regs[REG_IRPT_MASK] == 0xfeff);
    assert(regs[REG_IRPT_EN] == 0);
    sdio_card_irq_host_unmask();
    assert(regs[REG_IRPT_MASK] == 0xffff);
    assert(regs[REG_IRPT_EN] == INT_CARD);
    assert(!(regs[REG_INTERRUPT] & INT_CARD)); /* No stale interrupt. */
    card_asserted = 1;
    sdio_card_irq_mask();
    sdio_card_irq_host_unmask();
    assert(regs[REG_INTERRUPT] & INT_CARD); /* A new arrival is retained. */
    return 0;
}
"""

# The CLM data copy plus its 12-byte header must fit the actual buffer, and
# the whole frame must remain a byte-mode function-2 transfer.
size = re.search(r"clm_chunk\[(\d+) \+ (\d+)\]", cyw)
assert size
payload, header = map(int, size.groups())
assert header == 12 and payload + header + len("clmload\0") + 16 + 12 <= 512
assert "if (chunk > sizeof(clm_chunk) - 12U)" in cyw
assert "chunk = sizeof(clm_chunk) - 12U;" in cyw
for total in (1, payload, payload + 1, 2676, 16384):
    offset = 0
    while offset < total:
        chunk = min(total - offset, payload)
        assert 12 + chunk <= payload + header
        offset += chunk
    assert offset == total

with tempfile.TemporaryDirectory(prefix="pios-wifi-irq-") as tmp:
    source = Path(tmp) / "irq.c"
    exe = Path(tmp) / "irq.exe"
    source.write_text(harness, encoding="ascii")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    str(source), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

command_harness = r"""
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
typedef uint32_t u32;
#define REG_STATUS 0
#define REG_INTERRUPT 1
#define REG_ARG1 2
#define REG_CMDTM 3
#define REG_RESP0 4
#define REG_RESP1 5
#define REG_RESP2 6
#define REG_RESP3 7
#define SR_DAT_INHIBIT 2
#define CMD_ISDATA (1U << 21)
#define SDHCI_SOFTWARE_RESET 0
#define SDHCI_RESET_DATA 4
#define INT_ALL 0xffffffffU
#define INT_ERROR 0xffff8000U
#define INT_CMD_DONE 1
#define RSP_48_BUSY (3U << 16)
static bool issued, stuck;
static u32 waits;
static bool sdio_wait_cmd(void) { return true; }
static void delay_cycles(u32 n) { (void)n; }
static void uart_puts(const char *p) { (void)p; }
static void sw8(u32 a, u32 b) { (void)a; (void)b; }
static u32 sr8(u32 a) { (void)a; return 0; }
static void sw(u32 off, u32 value) {
    (void)value;
    if (off == REG_CMDTM) issued = true;
}
static u32 sr(u32 off) {
    if (off == REG_INTERRUPT) return INT_CMD_DONE;
    if (off == REG_STATUS && issued) {
        waits++;
        return stuck || waits < 3 ? SR_DAT_INHIBIT : 0;
    }
    return 0;
}
""" + function(sdio, "static bool sdio_send_cmd(") + r"""
int main(void) {
    u32 response[4];
    stuck = true;
    assert(sdio_send_cmd((53U << 24) | (2U << 16) | CMD_ISDATA, 0, response));
    assert(waits == 0); /* R5 data completion must wait until FIFO is serviced. */
    issued = false; stuck = false; waits = 0;
    assert(sdio_send_cmd((7U << 24) | RSP_48_BUSY, 0, response));
    assert(waits == 3);
    issued = false; stuck = true; waits = 0;
    assert(!sdio_send_cmd((7U << 24) | RSP_48_BUSY, 0, response));
    assert(waits == 1000000); /* Expiry must not wrap to a success-shaped value. */
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-wifi-cmd-") as tmp:
    source = Path(tmp) / "cmd.c"
    exe = Path(tmp) / "cmd.exe"
    source.write_text(command_harness, encoding="ascii")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    str(source), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("WiFi transport: read-only IRQ latch, re-arm and CLM bounds passed")
print("WiFi commands: R5 skips busy wait; R1b expiry fails closed")
