"""Exercise bounded, cached PCIe1 dashboard summaries without hardware."""
from pathlib import Path
import subprocess
import tempfile

from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
kernel = (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")


def function(signature: str) -> str:
    start = kernel.index(signature)
    opening = kernel.index("{", start)
    depth = 0
    for index in range(opening, len(kernel)):
        if kernel[index] == "{":
            depth += 1
        elif kernel[index] == "}":
            depth -= 1
            if depth == 0:
                return kernel[start:index + 1]
    raise AssertionError(f"unterminated {signature}")


dashboard = function("static void hdmi_dashboard_render(void)")
assert "pcie1_status(&p1);" in dashboard
assert "dash_pcie1_summary(p1caps" in dashboard
assert "pcie1_rescan(" not in dashboard
assert "pcie1_cfg_read(" not in dashboard and "pcie1_cfg_write(" not in dashboard

harness = r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "types.h"
struct pcie1_status {
    bool present, inited, link_up;
    u32 link_speed, link_width, ep_count;
    bool b50_found, scan_truncated;
};
""" + "\n".join(function(signature) for signature in (
    "static void dash_pcie1_append_text(",
    "static void dash_pcie1_append_u32(",
    "static u32 dash_pcie1_summary(",
)) + r"""
static void expect(const char *want, const struct pcie1_status *status) {
    char out[48];
    u32 len = dash_pcie1_summary(out, sizeof(out), status);
    assert(len == strlen(want));
    assert(strcmp(out, want) == 0);
}
int main(void) {
    struct pcie1_status status = {0};
    expect("PCIe1 not present", &status);
    status.present = true;
    expect("No link; check pciex1 + powered HAT", &status);
    status.link_up = true;
    status.link_speed = 2;
    status.link_width = 1;
    expect("Gen2 x1; no endpoints", &status);
    status.ep_count = 2;
    status.scan_truncated = true;
    status.b50_found = true;
    expect("Gen2 x1; 2 functions (+more); B50", &status);
    status.link_speed = 7;
    expect("Gen? x1; 2 functions (+more); B50", &status);
    char tiny[8];
    assert(dash_pcie1_summary(tiny, sizeof(tiny), &status) == 7);
    assert(strcmp(tiny, "Gen? x1") == 0);
    char one[1] = {'X'};
    assert(dash_pcie1_summary(one, sizeof(one), &status) == 0);
    assert(one[0] == '\0');
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-pcie-dashboard-") as tmp:
    source = Path(tmp) / "dashboard.c"
    exe = Path(tmp) / "dashboard.exe"
    source.write_text(harness, encoding="utf-8")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "tests" / "stubinc"),
                    str(source), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PCIe1 dashboard: cached capability summary formatting passed")
