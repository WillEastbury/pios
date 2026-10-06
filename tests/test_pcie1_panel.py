"""Compile the actual panel body and verify pagination/render bounds."""
from pathlib import Path
import subprocess
import tempfile
from run_host_tests import find_clang

root = Path(__file__).resolve().parent.parent
source = (root / "src/kernel.c").read_text()
start = source.index("    {\n        struct pcie1_status pcie;")
end = source.index("    dash_clear_body(tns_col", start)
panel = source[start:end]
assert "pcie1_rescan" not in panel and "pcie1_cfg_" not in panel
code = r"""
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pcie1.h"
static struct pcie1_status snapshot;
static u32 col,line,seen[64], page_bdf, width=100, column_end;
void pcie1_status(struct pcie1_status *out) { *out=snapshot; }
void dash_clear_body(u32 c,u32 r,u32 w,u32 h) {
    assert(c==0 && r==24 && w==width && h==11);
}
void fb_set_color(u32 f,u32 b) { (void)f;(void)b; }
void fb_set_cursor(u32 c,u32 r) {
    assert(c<width && r>=25 && r<34);col=c;line=r;
    u32 columns=(width-4)/76;if(!columns)columns=1;if(columns>3)columns=3;
    u32 cw=(width-4)/columns;
    column_end = r==25 ? width-2 : 2+((c-2)/cw+1)*cw-2;
}
void fb_puts(const char *s) {
    if(line>=26 && !strncmp(s,"p1 ",3)) {
        unsigned bus,dev,fn;assert(sscanf(s,"p1 %x:%x.%x",&bus,&dev,&fn)==3);
        assert(bus>=1 && bus<=64 && dev==0 && fn==0);seen[bus-1]++;page_bdf++;
    }
    col+=(u32)strlen(s);assert(col<=column_end);
}
void fb_printf(const char *fmt,...) {
    char b[256];va_list args;va_start(args,fmt);
    vsnprintf(b,sizeof(b),fmt,args);va_end(args);fb_puts(b);
}
void dash_put_trunc(const char *s,u32 n) {
    char b[256];assert(n<sizeof(b));snprintf(b,n+1,"%s",s);fb_puts(b);
}
static void render(u64 now_ms) {
    u32 pcie_row=24, pcie_h=11, header_w=width;
""" + panel + r"""
}
int main(void) {
    snapshot.present=true;snapshot.link_up=true;snapshot.ep_count=64;
    for(u32 i=0;i<64;i++)pcie1_fill_ep(&snapshot.eps[i],i+1,0,0,0xE2128086,0x03000000,0,0);
    const u32 widths[]={78,100,155,156,231,232,238,320};
    for(u32 w=0;w<sizeof(widths)/sizeof(widths[0]);w++) {
        width=widths[w];memset(seen,0,sizeof(seen));
        u32 columns=(width-4)/76;if(!columns)columns=1;if(columns>3)columns=3;
        u32 per_page=columns*8, pages=(64+per_page-1)/per_page;
        for(u32 page=0;page<pages;page++) {
            page_bdf=0;render(page*8000ULL);
            u32 remaining=64-page*per_page;
            assert(page_bdf==(remaining<per_page?remaining:per_page));
        }
        for(u32 i=0;i<64;i++)assert(seen[i]==1);
    }
    width=238;snapshot.ep_count=20;page_bdf=0;render(0);assert(page_bdf==20);
    snapshot.ep_count=1;page_bdf=0;render(64000);assert(page_bdf==1);
    snapshot.ep_count=0;snapshot.fail_reason="no link";page_bdf=0;
    render(72000);assert(page_bdf==0);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-pcie-panel-") as tmp:
    tmp = Path(tmp)
    (tmp / "test.c").write_text(code)
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-include", str(root / "tests/stubinc/types.h"), "-I", str(root / "include"),
                    str(tmp / "test.c"), "-o", str(tmp / "test.exe")], check=True)
    subprocess.run([str(tmp / "test.exe")], check=True)
print("PCIe panel: 1/2/3 columns, all 64 entries, 20-device single page and cell bounds passed")
