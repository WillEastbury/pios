/* Multi-target host emulators for C PicoVM (DateTime pure, Gpio, Stream, Ui,
 * Process/Env, Timer, Locale, Auth/X509 memory, Context←Req, Http client). */
#include "picovm_emu.h"
#include "pico_hooks.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#endif

/* ---- helpers (mirror picovm.c arena API via public surface) ------------ */

extern int pv_span_from_bytes(pv_ctx *ctx, const void *data, uint32_t len);

static int32_t sx32(int32_t v) { return v; }

static uint32_t span_ptr(pv_ctx *ctx, int h)
{
    return (h > 0 && h < ctx->span_count) ? ctx->span_ptr[h] : 0;
}
static int32_t span_len(pv_ctx *ctx, int h)
{
    return (h > 0 && h < ctx->span_count) ? ctx->span_len[h] : 0;
}
static void span_copy(pv_ctx *ctx, int h, char *out, int cap)
{
    int32_t n = span_len(ctx, h);
    uint32_t p = span_ptr(ctx, h);
    int i;
    if (n < 0) n = 0;
    if (n >= cap) n = cap - 1;
    for (i = 0; i < n; i++)
        out[i] = ctx->mem ? (char)ctx->mem[(p + (uint32_t)i) % (uint32_t)ctx->mem_size] : 0;
    out[n] = 0;
}

/* ---- civil calendar (UTC, proleptic Gregorian) ------------------------ */

static int is_leap(int y)
{
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

static void secs_to_ymd(int64_t sec, int *Y, int *M, int *D, int *hh, int *mm, int *ss)
{
    static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int64_t days, rem;
    int y = 1970, m;
    if (sec < 0) sec = 0;
    days = sec / 86400;
    rem = sec % 86400;
    *hh = (int)(rem / 3600);
    *mm = (int)((rem % 3600) / 60);
    *ss = (int)(rem % 60);
    while (1) {
        int diy = is_leap(y) ? 366 : 365;
        if (days < diy) break;
        days -= diy;
        y++;
    }
    *Y = y;
    for (m = 0; m < 12; m++) {
        int dim = mdays[m] + (m == 1 && is_leap(y) ? 1 : 0);
        if (days < dim) break;
        days -= dim;
    }
    *M = m + 1;
    *D = (int)days + 1;
}

static int day_of_year(int Y, int M, int D)
{
    static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int i, n = D;
    for (i = 0; i < M - 1; i++)
        n += mdays[i] + (i == 1 && is_leap(Y) ? 1 : 0);
    return n;
}

/* ISO weekday: Mon=1 .. Sun=7 (Python isoweekday) */
static int iso_weekday(int64_t sec)
{
    /* 1970-01-01 was Thursday = 4 */
    int64_t days = sec / 86400;
    int w = (int)((days + 3) % 7) + 1; /* Mon=1 */
    if (w <= 0) w += 7;
    return w;
}

static int parse_iso_basic(const char *s, int64_t *out)
{
    int Y, M, D, h = 0, m = 0, sec = 0;
    int n;
    char c;
    if (!s || !*s) return 0;
    if (s[0] == '+' || s[0] == '-' || (s[0] >= '0' && s[0] <= '9')) {
        int only = 1;
        const char *p = s;
        if (*p == '+' || *p == '-') p++;
        while (*p) {
            if (*p < '0' || *p > '9') { only = 0; break; }
            p++;
        }
        if (only && p > s) {
            *out = (int64_t)strtoll(s, 0, 10);
            return 1;
        }
    }
    n = sscanf(s, "%d-%d-%d%c%d:%d:%d", &Y, &M, &D, &c, &h, &m, &sec);
    if (n >= 3) {
        /* rough epoch: days since 1970 via cumulative years */
        int64_t days = 0;
        int y, mo;
        static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
        if (Y < 1970) return 0;
        for (y = 1970; y < Y; y++) days += is_leap(y) ? 366 : 365;
        for (mo = 1; mo < M; mo++)
            days += mdays[mo - 1] + (mo == 2 && is_leap(Y) ? 1 : 0);
        days += D - 1;
        *out = days * 86400 + h * 3600 + m * 60 + sec;
        return 1;
    }
    return 0;
}

static int emu_datetime(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    int32_t a = ctx->regs[rs1], b = ctx->regs[rs2];
    int Y, M, D, hh, mm, ss;
    char buf[64];
    int64_t sec;

    if (hook == PV_HOOK_DATETIME_ADDSECONDS) {
        ctx->regs[rd] = (int32_t)((uint32_t)(sx32(a) + sx32(b)));
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_ADDMINUTES) {
        ctx->regs[rd] = (int32_t)((uint32_t)(sx32(a) + sx32(b) * 60));
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_ADDHOURS) {
        ctx->regs[rd] = (int32_t)((uint32_t)(sx32(a) + sx32(b) * 3600));
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_ADDDAYS) {
        ctx->regs[rd] = (int32_t)((uint32_t)(sx32(a) + sx32(b) * 86400));
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_PARSE) {
        char raw[128];
        span_copy(ctx, a, raw, sizeof(raw));
        if (!parse_iso_basic(raw, &sec)) {
            ctx->regs[rd] = 0;
            ctx->host_status = 2;
            return 1;
        }
        ctx->regs[rd] = (int32_t)(uint32_t)sec;
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_FORMAT) {
        sec = (int64_t)(uint32_t)sx32(a);
        secs_to_ymd(sec, &Y, &M, &D, &hh, &mm, &ss);
        snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ", Y, M, D, hh, mm, ss);
        ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)strlen(buf));
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_GETDAYOFWEEK) {
        sec = (int64_t)(uint32_t)sx32(a);
        ctx->regs[rd] = iso_weekday(sec);
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_GETDAYOFYEAR) {
        sec = (int64_t)(uint32_t)sx32(a);
        secs_to_ymd(sec, &Y, &M, &D, &hh, &mm, &ss);
        ctx->regs[rd] = day_of_year(Y, M, D);
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_DIFFDAYS) {
        int64_t da = (int64_t)(uint32_t)sx32(a);
        int64_t db = (int64_t)(uint32_t)sx32(b);
        int64_t diff = da - db;
        int64_t q = (diff < 0 ? -diff : diff) / 86400;
        ctx->regs[rd] = (int32_t)(diff < 0 ? -q : q);
        return 1;
    }
    if (hook == PV_HOOK_DATETIME_YEAR || hook == PV_HOOK_DATETIME_MONTH ||
        hook == PV_HOOK_DATETIME_DAY) {
        sec = (int64_t)(uint32_t)sx32(a);
        secs_to_ymd(sec, &Y, &M, &D, &hh, &mm, &ss);
        if (hook == PV_HOOK_DATETIME_YEAR) ctx->regs[rd] = Y;
        else if (hook == PV_HOOK_DATETIME_MONTH) ctx->regs[rd] = M;
        else ctx->regs[rd] = D;
        ctx->host_status = 0;
        return 1;
    }
    return 0;
}

/* ---- GPIO emulator ---------------------------------------------------- */
#define EMU_GPIO_PINS 64
static int32_t g_gpio_dir[EMU_GPIO_PINS];
static int32_t g_gpio_pull[EMU_GPIO_PINS];
static int32_t g_gpio_val[EMU_GPIO_PINS];

static int emu_gpio(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    uint32_t pin;
    if (hook == PV_HOOK_GPIO_COUNT) {
        ctx->regs[rd] = 40;
        return 1;
    }
    pin = (uint32_t)ctx->regs[rs1] % EMU_GPIO_PINS;
    if (hook == PV_HOOK_GPIO_SETDIR) {
        g_gpio_dir[pin] = ctx->regs[rs2] ? 1 : 0;
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_GPIO_GETDIR) {
        ctx->regs[rd] = g_gpio_dir[pin];
        return 1;
    }
    if (hook == PV_HOOK_GPIO_SETPULL) {
        int p = ctx->regs[rs2];
        g_gpio_pull[pin] = (p >= 0 && p <= 2) ? p : 0;
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_GPIO_GETPULL) {
        ctx->regs[rd] = g_gpio_pull[pin];
        return 1;
    }
    if (hook == PV_HOOK_GPIO_WRITE) {
        int v = sx32(ctx->regs[rs2]);
        if (v < 0) v = 0;
        if (v > 1024) v = 1024;
        g_gpio_val[pin] = v;
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_GPIO_READ) {
        ctx->regs[rd] = g_gpio_val[pin];
        return 1;
    }
    return 0;
}

/* ---- Device / Stream emulator ----------------------------------------- */
#define EMU_MAX_DEV 32
#define EMU_MAX_STREAM 32
#define EMU_MAX_LEASE 64
static int g_dev_seq, g_stream_seq, g_lease_seq;
static int g_dev_open[EMU_MAX_DEV];
static struct {
    int dir, buf, frames, next;
} g_stream[EMU_MAX_STREAM];
static struct {
    int used, stream, idx, released;
    uint8_t data[256];
    int dlen;
    int span;
} g_lease[EMU_MAX_LEASE];
static int g_slice_off, g_slice_len;

static int emu_device_stream(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook == PV_HOOK_DEVICE_OPEN) {
        int h = ++g_dev_seq;
        if (h >= EMU_MAX_DEV) h = 1;
        g_dev_open[h] = 1;
        ctx->regs[rd] = h;
        return 1;
    }
    if (hook == PV_HOOK_DEVICE_CAPS) {
        int h = ctx->regs[rs1];
        ctx->regs[rd] = (h > 0 && h < EMU_MAX_DEV && g_dev_open[h]) ? 0x3 : 0;
        return 1;
    }
    if (hook == PV_HOOK_DEVICE_STATUS) {
        int h = ctx->regs[rs1];
        ctx->regs[rd] = (h > 0 && h < EMU_MAX_DEV && g_dev_open[h]) ? 0 : 1;
        return 1;
    }
    if (hook == PV_HOOK_DEVICE_CLOSE) {
        int h = ctx->regs[rs1];
        if (h > 0 && h < EMU_MAX_DEV && g_dev_open[h]) {
            g_dev_open[h] = 0;
            ctx->regs[rd] = 1;
        } else ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_STREAM_OPEN) {
        int dh = ctx->regs[rs1];
        uint32_t cfg = (uint32_t)ctx->regs[rs2];
        int h;
        if (!(dh > 0 && dh < EMU_MAX_DEV && g_dev_open[dh])) {
            ctx->regs[rd] = 0;
            ctx->host_status = 1;
            return 1;
        }
        h = ++g_stream_seq;
        if (h >= EMU_MAX_STREAM) h = 1;
        g_stream[h].dir = cfg & 1;
        g_stream[h].buf = (int)((cfg >> 1) & 0x7FFF);
        if (g_stream[h].buf > 256) g_stream[h].buf = 256;
        g_stream[h].frames = (int)((cfg >> 16) & 0xFFFF);
        g_stream[h].next = 0;
        ctx->regs[rd] = h;
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_STREAM_NEXT) {
        int sh = ctx->regs[rs1];
        int lh, i;
        if (!(sh > 0 && sh < EMU_MAX_STREAM) ||
            g_stream[sh].next >= g_stream[sh].frames) {
            ctx->regs[rd] = 0;
            ctx->host_status = 3;
            return 1;
        }
        lh = ++g_lease_seq;
        if (lh >= EMU_MAX_LEASE) lh = 1;
        g_lease[lh].used = 1;
        g_lease[lh].stream = sh;
        g_lease[lh].idx = g_stream[sh].next++;
        g_lease[lh].released = 0;
        g_lease[lh].span = 0;
        g_lease[lh].dlen = g_stream[sh].buf;
        for (i = 0; i < g_lease[lh].dlen; i++)
            g_lease[lh].data[i] = (uint8_t)((g_lease[lh].idx + i) & 0xFF);
        ctx->regs[rd] = lh;
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_STREAM_SPAN) {
        int lh = ctx->regs[rs1];
        if (!(lh > 0 && lh < EMU_MAX_LEASE) || !g_lease[lh].used ||
            g_lease[lh].released) {
            ctx->regs[rd] = 0;
            ctx->host_status = 1;
            return 1;
        }
        if (!g_lease[lh].span)
            g_lease[lh].span = pv_span_from_bytes(ctx, g_lease[lh].data,
                                                  (uint32_t)g_lease[lh].dlen);
        ctx->regs[rd] = g_lease[lh].span;
        return 1;
    }
    if (hook == PV_HOOK_STREAM_SETSLICE) {
        g_slice_off = sx32(ctx->regs[rs1]);
        if (g_slice_off < 0) g_slice_off = 0;
        g_slice_len = sx32(ctx->regs[rs2]);
        if (g_slice_len < 0) g_slice_len = 0;
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_STREAM_SLICE) {
        int lh = ctx->regs[rs1];
        int off, n;
        if (!(lh > 0 && lh < EMU_MAX_LEASE) || !g_lease[lh].used ||
            g_lease[lh].released) {
            ctx->regs[rd] = 0;
            ctx->host_status = 1;
            return 1;
        }
        off = g_slice_off;
        if (off > g_lease[lh].dlen) off = g_lease[lh].dlen;
        n = g_slice_len;
        if (off + n > g_lease[lh].dlen) n = g_lease[lh].dlen - off;
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_lease[lh].data + off, (uint32_t)n);
        return 1;
    }
    if (hook == PV_HOOK_STREAM_RELEASE) {
        int lh = ctx->regs[rs1];
        if (lh > 0 && lh < EMU_MAX_LEASE && g_lease[lh].used) {
            g_lease[lh].released = 1;
            ctx->regs[rd] = 1;
        } else ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_STREAM_CLOSE || hook == PV_HOOK_STREAM_SUBMIT) {
        ctx->regs[rd] = 1;
        return 1;
    }
    return 0;
}

/* ---- UI retained tree ------------------------------------------------- */
#define EMU_UI_MAX 128
static struct {
    int used, kind, id, x, y, w, h, value, parent;
    char text[64];
    int child[16];
    int nchild;
} g_ui[EMU_UI_MAX];
static int g_ui_seq;

static int ui_kind_from_hook(int hook)
{
    if (hook == PV_HOOK_UI_WINDOW) return 1;
    if (hook == PV_HOOK_UI_PANEL) return 2;
    if (hook == PV_HOOK_UI_LABEL) return 3;
    if (hook == PV_HOOK_UI_BUTTON) return 4;
    if (hook == PV_HOOK_UI_TEXTBOX) return 5;
    if (hook == PV_HOOK_UI_CHECKBOX) return 6;
    return 0;
}

static int emu_ui(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    int k = ui_kind_from_hook(hook);
    if (k) {
        int h = ++g_ui_seq;
        int parent = 0;
        char t[64];
        if (h >= EMU_UI_MAX) h = 1;
        memset(&g_ui[h], 0, sizeof(g_ui[h]));
        g_ui[h].used = 1;
        g_ui[h].kind = k;
        if (hook == PV_HOOK_UI_WINDOW) {
            span_copy(ctx, ctx->regs[rs1], t, sizeof(t));
            memcpy(g_ui[h].text, t, sizeof(t));
        } else {
            parent = ctx->regs[rs1];
            g_ui[h].parent = parent;
            if (hook != PV_HOOK_UI_PANEL) {
                span_copy(ctx, ctx->regs[rs2], t, sizeof(t));
                memcpy(g_ui[h].text, t, sizeof(t));
            }
            if (parent > 0 && parent < EMU_UI_MAX && g_ui[parent].used &&
                g_ui[parent].nchild < 16)
                g_ui[parent].child[g_ui[parent].nchild++] = h;
        }
        ctx->regs[rd] = h;
        return 1;
    }
    {
        int nid = ctx->regs[rs1];
        if (!(nid > 0 && nid < EMU_UI_MAX && g_ui[nid].used)) {
            if (hook == PV_HOOK_UI_SERIALIZE) {
                ctx->regs[rd] = 0;
                return 1;
            }
            ctx->regs[rd] = 0;
            return 1;
        }
        if (hook == PV_HOOK_UI_POS) {
            uint32_t v = (uint32_t)ctx->regs[rs2];
            g_ui[nid].x = (int)((v >> 16) & 0xFFFF);
            g_ui[nid].y = (int)(v & 0xFFFF);
            ctx->regs[rd] = 1;
            return 1;
        }
        if (hook == PV_HOOK_UI_SIZE) {
            uint32_t v = (uint32_t)ctx->regs[rs2];
            g_ui[nid].w = (int)((v >> 16) & 0xFFFF);
            g_ui[nid].h = (int)(v & 0xFFFF);
            ctx->regs[rd] = 1;
            return 1;
        }
        if (hook == PV_HOOK_UI_SETTEXT) {
            span_copy(ctx, ctx->regs[rs2], g_ui[nid].text, sizeof(g_ui[nid].text));
            ctx->regs[rd] = 1;
            return 1;
        }
        if (hook == PV_HOOK_UI_SETID) {
            g_ui[nid].id = ctx->regs[rs2] & 0xFFFF;
            ctx->regs[rd] = 1;
            return 1;
        }
        if (hook == PV_HOOK_UI_SETVALUE) {
            g_ui[nid].value = ctx->regs[rs2] & 0xFFFF;
            ctx->regs[rd] = 1;
            return 1;
        }
        if (hook == PV_HOOK_UI_SERIALIZE) {
            /* PSC1-ish preorder dump: u16 count + per-node PSC1 records */
            uint8_t out[4096];
            int order[EMU_UI_MAX], norder = 0, stack[EMU_UI_MAX], sp = 0, oi, pos = 2;
            stack[sp++] = nid;
            while (sp > 0 && norder < EMU_UI_MAX) {
                int cur = stack[--sp], c;
                order[norder++] = cur;
                for (c = g_ui[cur].nchild - 1; c >= 0; c--)
                    stack[sp++] = g_ui[cur].child[c];
            }
            out[0] = (uint8_t)((norder >> 8) & 0xff);
            out[1] = (uint8_t)(norder & 0xff);
            for (oi = 0; oi < norder && pos + 64 < (int)sizeof(out); oi++) {
                int id = order[oi];
                int tlen = (int)strlen(g_ui[id].text);
                /* magic PSC1 */
                if (pos + 4 + 20 + tlen > (int)sizeof(out)) break;
                out[pos++] = 'P'; out[pos++] = 'S'; out[pos++] = 'C'; out[pos++] = '1';
                out[pos++] = 0; out[pos++] = 9; /* field count */
                /* c=kind */
                out[pos++] = 1; out[pos++] = 'c'; out[pos++] = 1;
                out[pos++] = (uint8_t)((g_ui[id].kind >> 24) & 0xff);
                out[pos++] = (uint8_t)((g_ui[id].kind >> 16) & 0xff);
                out[pos++] = (uint8_t)((g_ui[id].kind >> 8) & 0xff);
                out[pos++] = (uint8_t)(g_ui[id].kind & 0xff);
                /* id */
                out[pos++] = 2; out[pos++] = 'i'; out[pos++] = 'd'; out[pos++] = 1;
                out[pos++] = 0; out[pos++] = 0;
                out[pos++] = (uint8_t)((g_ui[id].id >> 8) & 0xff);
                out[pos++] = (uint8_t)(g_ui[id].id & 0xff);
                /* t=text */
                out[pos++] = 1; out[pos++] = 't'; out[pos++] = 2;
                out[pos++] = (uint8_t)((tlen >> 8) & 0xff);
                out[pos++] = (uint8_t)(tlen & 0xff);
                memcpy(out + pos, g_ui[id].text, (size_t)tlen);
                pos += tlen;
            }
            ctx->regs[rd] = pv_span_from_bytes(ctx, out, (uint32_t)pos);
            return 1;
        }
    }
    return 0;
}

/* ---- Event / Process / Env / Timer / Principal ------------------------ */
#define EMU_PROC_MAX 32
#define EMU_ENV_MAX 64
#define EMU_TIMER_MAX 64
static int g_proc_self = 1, g_proc_parent = 0, g_proc_seq = 100;
static struct { int status, exit_code; } g_proc[EMU_PROC_MAX];
static char g_env_k[EMU_ENV_MAX][48];
static char g_env_v[EMU_ENV_MAX][128];
static int g_env_n;
static int g_timer_seq;
static struct { int ms, rem, rep, active; } g_timer[EMU_TIMER_MAX];
static int g_elapsed_ms;
static char g_principal_name[64] = "anonymous";
static char g_principal_roles[8][32];
static int g_principal_role_n;
static char g_principal_claim_k[8][32];
static char g_principal_claim_v[8][64];
static int g_principal_claim_n;
static uint32_t g_sandbox_denied;

static void emu_event_push(pv_ctx *ctx, int type, int target)
{
    int id = ++ctx->event_seq;
    if (id >= PV_MAX_EVENTS) id = 1;
    ctx->event_seq = id;
    ctx->event_used[id] = 1;
    ctx->event_type[id] = type;
    ctx->event_target[id] = target;
    ctx->event_span[id] = 0;
    if (((ctx->event_qtail + 1) % PV_MAX_EVENTS) != ctx->event_qhead) {
        ctx->event_queue[ctx->event_qtail] = id;
        ctx->event_qtail = (ctx->event_qtail + 1) % PV_MAX_EVENTS;
    }
}

static int emu_event(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    int id;
    if (hook == PV_HOOK_EVENT_POST) {
        emu_event_push(ctx, ctx->regs[rs1], ctx->regs[rs2]);
        ctx->regs[rd] = ctx->event_seq;
        return 1;
    }
    if (hook == PV_HOOK_EVENT_COUNT) {
        int n = ctx->event_qtail - ctx->event_qhead;
        if (n < 0) n += PV_MAX_EVENTS;
        ctx->regs[rd] = n;
        return 1;
    }
    if (hook == PV_HOOK_EVENT_NEXT) {
        if (ctx->event_qhead == ctx->event_qtail) {
            ctx->regs[rd] = 0;
            return 1;
        }
        id = ctx->event_queue[ctx->event_qhead];
        ctx->event_qhead = (ctx->event_qhead + 1) % PV_MAX_EVENTS;
        ctx->regs[rd] = id;
        return 1;
    }
    id = ctx->regs[rs1];
    if (!(id > 0 && id < PV_MAX_EVENTS && ctx->event_used[id])) {
        ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_EVENT_TYPE) { ctx->regs[rd] = ctx->event_type[id]; return 1; }
    if (hook == PV_HOOK_EVENT_TARGET) { ctx->regs[rd] = ctx->event_target[id]; return 1; }
    if (hook == PV_HOOK_EVENT_DATA || hook == PV_HOOK_EVENT_DATASLICE) { ctx->regs[rd] = ctx->event_span[id]; return 1; }
    if (hook == PV_HOOK_EVENT_DATALEN) { ctx->regs[rd] = span_len(ctx, ctx->event_span[id]); return 1; }
    if (hook == PV_HOOK_EVENT_SETDATA) { ctx->event_span[id] = ctx->regs[rs2]; ctx->regs[rd] = 1; return 1; }
    if (hook == PV_HOOK_EVENT_SETSLICE) { ctx->regs[rd] = 1; return 1; }
    return 0;
}

static int emu_process_env_timer(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook == PV_HOOK_PROCESS_SELF) { ctx->regs[rd] = g_proc_self; return 1; }
    if (hook == PV_HOOK_PROCESS_PARENT) { ctx->regs[rd] = g_proc_parent; return 1; }
    if (hook == PV_HOOK_PROCESS_SPAWN) {
        int pid = ++g_proc_seq;
        int slot = pid % EMU_PROC_MAX;
        g_proc[slot].status = 0;
        g_proc[slot].exit_code = 0;
        ctx->regs[rd] = pid;
        return 1;
    }
    if (hook == PV_HOOK_PROCESS_EXIT) {
        int slot = g_proc_self % EMU_PROC_MAX;
        g_proc[slot].status = 1;
        g_proc[slot].exit_code = sx32(ctx->regs[rs1]);
        ctx->halted = 1;
        ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_PROCESS_KILL) {
        int pid = ctx->regs[rs1];
        int slot = pid % EMU_PROC_MAX;
        if (g_proc[slot].status == 0) {
            g_proc[slot].status = 2;
            g_proc[slot].exit_code = -1;
            ctx->regs[rd] = 1;
        } else ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_PROCESS_STATUS) {
        int pid = ctx->regs[rs1];
        ctx->regs[rd] = g_proc[pid % EMU_PROC_MAX].status;
        return 1;
    }
    if (hook == PV_HOOK_PROCESS_WAIT) {
        int pid = ctx->regs[rs1];
        ctx->regs[rd] = g_proc[pid % EMU_PROC_MAX].exit_code;
        return 1;
    }
    if (hook == PV_HOOK_PROCESS_ARGS) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, "", 0);
        return 1;
    }
    if (hook == PV_HOOK_ENV_GET) {
        char key[48];
        int i;
        span_copy(ctx, ctx->regs[rs1], key, sizeof(key));
        for (i = 0; i < g_env_n; i++)
            if (strcmp(g_env_k[i], key) == 0) {
                ctx->regs[rd] = pv_span_from_bytes(ctx, g_env_v[i],
                                                   (uint32_t)strlen(g_env_v[i]));
                ctx->host_status = 0;
                return 1;
            }
        ctx->regs[rd] = 0;
        ctx->host_status = 1;
        return 1;
    }
    if (hook == PV_HOOK_ENV_SET) {
        char key[48], val[128];
        int i;
        span_copy(ctx, ctx->regs[rs1], key, sizeof(key));
        span_copy(ctx, ctx->regs[rs2], val, sizeof(val));
        for (i = 0; i < g_env_n; i++)
            if (strcmp(g_env_k[i], key) == 0) {
                strncpy(g_env_v[i], val, sizeof(g_env_v[i]) - 1);
                ctx->regs[rd] = 1;
                return 1;
            }
        if (g_env_n < EMU_ENV_MAX) {
            strncpy(g_env_k[g_env_n], key, sizeof(g_env_k[0]) - 1);
            strncpy(g_env_v[g_env_n], val, sizeof(g_env_v[0]) - 1);
            g_env_n++;
        }
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_ENV_COUNT) {
        ctx->regs[rd] = g_env_n;
        return 1;
    }
    if (hook == PV_HOOK_ENV_KEY) {
        int idx = ctx->regs[rs1];
        if (idx >= 0 && idx < g_env_n) {
            ctx->regs[rd] = pv_span_from_bytes(ctx, g_env_k[idx],
                                               (uint32_t)strlen(g_env_k[idx]));
            ctx->host_status = 0;
        } else {
            ctx->regs[rd] = 0;
            ctx->host_status = 1;
        }
        return 1;
    }
    if (hook == PV_HOOK_TIMER_AFTER || hook == PV_HOOK_TIMER_EVERY) {
        int h = ++g_timer_seq;
        if (h >= EMU_TIMER_MAX) h = 1;
        g_timer[h].ms = ctx->regs[rs1];
        g_timer[h].rem = ctx->regs[rs1];
        g_timer[h].rep = (hook == PV_HOOK_TIMER_EVERY);
        g_timer[h].active = 1;
        ctx->regs[rd] = h;
        return 1;
    }
    if (hook == PV_HOOK_TIMER_CANCEL) {
        int h = ctx->regs[rs1];
        if (h > 0 && h < EMU_TIMER_MAX && g_timer[h].active) {
            g_timer[h].active = 0;
            ctx->regs[rd] = 1;
        } else ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_TIMER_ELAPSED) {
        ctx->regs[rd] = g_elapsed_ms;
        return 1;
    }
    if (hook == PV_HOOK_SCHEDULER_TICK) {
        int delta = ctx->regs[rs1];
        int i, fired = 0;
        g_elapsed_ms += delta;
        for (i = 1; i < EMU_TIMER_MAX; i++) {
            if (!g_timer[i].active) continue;
            g_timer[i].rem -= delta;
            while (g_timer[i].rem <= 0 && g_timer[i].active) {
                fired++;
                emu_event_push(ctx, 100, i);
                if (g_timer[i].rep) g_timer[i].rem += g_timer[i].ms;
                else g_timer[i].active = 0;
            }
        }
        ctx->regs[rd] = fired;
        return 1;
    }
    if (hook == PV_HOOK_PRINCIPAL_CURRENT) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_principal_name, (uint32_t)strlen(g_principal_name));
        return 1;
    }
    if (hook == PV_HOOK_PRINCIPAL_HASROLE) {
        char role[32];
        int i;
        span_copy(ctx, ctx->regs[rs1], role, sizeof(role));
        for (i = 0; i < g_principal_role_n; i++) {
            if (strcmp(g_principal_roles[i], role) == 0) { ctx->regs[rd] = 1; return 1; }
        }
        ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_PRINCIPAL_CLAIMS) {
        char buf[512];
        int i, pos = 0;
        buf[0] = 0;
        for (i = 0; i < g_principal_claim_n && pos + 2 < (int)sizeof(buf); i++) {
            int n = snprintf(buf + pos, sizeof(buf) - (size_t)pos, "%s%s=%s", pos ? ";" : "", g_principal_claim_k[i], g_principal_claim_v[i]);
            if (n < 0 || pos + n >= (int)sizeof(buf)) break;
            pos += n;
        }
        ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)pos);
        return 1;
    }
    if (hook == PV_HOOK_CAPABILITY_HAS) {
        uint32_t cap_bit = (uint32_t)ctx->regs[rs1];
        ctx->regs[rd] = ((ctx->caps & cap_bit) && !(g_sandbox_denied & cap_bit)) ? 1 : 0;
        return 1;
    }
    if (hook == PV_HOOK_CAPABILITY_REQUEST) {
        uint32_t cap_bit = (uint32_t)ctx->regs[rs1];
        if ((g_sandbox_denied & cap_bit) || (cap_bit & ~ctx->cap_ceiling)) ctx->regs[rd] = 0;
        else { ctx->caps |= cap_bit; ctx->regs[rd] = 1; }
        return 1;
    }
    if (hook == PV_HOOK_CAPABILITY_DROP) {
        ctx->caps &= ~(uint32_t)ctx->regs[rs1];
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_SANDBOX_DENY) {
        uint32_t cap_bit = (uint32_t)ctx->regs[rs1];
        g_sandbox_denied |= cap_bit;
        ctx->caps &= ~cap_bit;
        ctx->regs[rd] = 1;
        return 1;
    }
    return 0;
}

/* ---- Locale (UTC-only lightweight) ------------------------------------ */
static char g_locale_tag[32] = "en-US";
static char g_locale_tz[48] = "UTC";

static int emu_locale(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    char buf[96];
    if (hook == PV_HOOK_LOCALE_SETLOCALE) {
        char spec[64];
        span_copy(ctx, ctx->regs[rs1], spec, sizeof(spec));
        if (spec[0]) {
            char *at = strchr(spec, '@');
            if (at) {
                *at = 0;
                strncpy(g_locale_tag, spec, sizeof(g_locale_tag) - 1);
                strncpy(g_locale_tz, at + 1, sizeof(g_locale_tz) - 1);
            } else {
                strncpy(g_locale_tag, spec, sizeof(g_locale_tag) - 1);
            }
        }
        ctx->regs[rd] = 1;
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_LOCALE_GETCURRENTLOCALE) {
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
        /* Refresh TZ label from OS when still default UTC */
        if (strcmp(g_locale_tz, "UTC") == 0) {
#ifdef _WIN32
            TIME_ZONE_INFORMATION tzi;
            if (GetTimeZoneInformation(&tzi) != TIME_ZONE_ID_INVALID) {
                int i;
                for (i = 0; i < 31 && tzi.StandardName[i]; i++)
                    g_locale_tz[i] = (char)(tzi.StandardName[i] & 0x7f);
                g_locale_tz[i] = 0;
                if (!g_locale_tz[0]) strcpy(g_locale_tz, "Local");
            }
#else
            time_t t = time(NULL);
            struct tm *lt = localtime(&t);
            if (lt && lt->tm_zone && lt->tm_zone[0])
                strncpy(g_locale_tz, lt->tm_zone, sizeof(g_locale_tz) - 1);
#endif
        }
#endif
        snprintf(buf, sizeof(buf), "%s@%s", g_locale_tag, g_locale_tz);
        ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)strlen(buf));
        return 1;
    }
    if (hook == PV_HOOK_LOCALE_FORMATNUMBER) {
        int32_t v = sx32(ctx->regs[rs1]);
        int32_t scale = sx32(ctx->regs[rs2]);
        if (scale > 0 && scale < 9) {
            int32_t div = 1, i;
            for (i = 0; i < scale; i++) div *= 10;
            snprintf(buf, sizeof(buf), "%d.%0*d", v / div, scale,
                     v >= 0 ? (v % div) : -(v % div));
        } else {
            snprintf(buf, sizeof(buf), "%d", v);
        }
        ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)strlen(buf));
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_LOCALE_FORMATCURRENCY) {
        int32_t v = sx32(ctx->regs[rs1]);
        snprintf(buf, sizeof(buf), "USD %d.%02d", v / 100, v >= 0 ? (v % 100) : -(v % 100));
        ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)strlen(buf));
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_LOCALE_FORMATDATE || hook == PV_HOOK_LOCALE_FORMATTIME) {
        int64_t sec = (int64_t)(uint32_t)sx32(ctx->regs[rs1]);
        time_t t = (time_t)sec;
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
        struct tm *lt = localtime(&t);
        if (lt) {
            if (hook == PV_HOOK_LOCALE_FORMATDATE)
                snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                         lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday);
            else
                snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                         lt->tm_hour, lt->tm_min, lt->tm_sec);
            ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)strlen(buf));
            ctx->host_status = 0;
            return 1;
        }
#endif
        {
            int Y, M, D, hh, mm, ss;
            secs_to_ymd(sec, &Y, &M, &D, &hh, &mm, &ss);
            if (hook == PV_HOOK_LOCALE_FORMATDATE)
                snprintf(buf, sizeof(buf), "%04d-%02d-%02d", Y, M, D);
            else
                snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hh, mm, ss);
            ctx->regs[rd] = pv_span_from_bytes(ctx, buf, (uint32_t)strlen(buf));
            ctx->host_status = 0;
            return 1;
        }
    }
    if (hook == PV_HOOK_LOCALE_TRANSLATE) {
        char key[64];
        span_copy(ctx, ctx->regs[rs1], key, sizeof(key));
        ctx->regs[rd] = pv_span_from_bytes(ctx, key, (uint32_t)strlen(key));
        ctx->host_status = key[0] ? 0 : 2;
        return 1;
    }
    return 0;
}

/* ---- Auth / X509 minimal in-memory ------------------------------------ */
static char g_auth_user[64] = "admin";
static char g_auth_pass[64] = "admin";
static char g_auth_token[64] = "tok-demo";
static int g_auth_valid = 1;
static char g_x509_pem[256] = "-----BEGIN CERT-----\nDEMO\n-----END CERT-----\n";

static int emu_auth_x509(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    (void)rs2;
    if (hook == PV_HOOK_AUTH_GETUSERCREDENTIALS) {
        char out[128];
        snprintf(out, sizeof(out), "%s:", g_auth_user);
        ctx->regs[rd] = pv_span_from_bytes(ctx, out, (uint32_t)strlen(out));
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_AUTH_VALIDATECREDENTIALS) {
        char u[64], p[64];
        span_copy(ctx, ctx->regs[rs1], u, sizeof(u));
        span_copy(ctx, ctx->regs[rs2], p, sizeof(p));
        ctx->regs[rd] = (strcmp(u, g_auth_user) == 0 && strcmp(p, g_auth_pass) == 0) ? 1 : 0;
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_AUTH_GETUSERPERMISSIONS) {
        const char *perm = "read,write,admin";
        ctx->regs[rd] = pv_span_from_bytes(ctx, perm, (uint32_t)strlen(perm));
        return 1;
    }
    if (hook == PV_HOOK_AUTH_REQUESTTOKEN || hook == PV_HOOK_AUTH_GETTOKEN ||
        hook == PV_HOOK_AUTH_REFRESHTOKEN) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_auth_token, (uint32_t)strlen(g_auth_token));
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_AUTH_VALIDATETOKEN) {
        char t[64];
        span_copy(ctx, ctx->regs[rs1], t, sizeof(t));
        ctx->regs[rd] = (strcmp(t, g_auth_token) == 0 && g_auth_valid) ? 1 : 0;
        return 1;
    }
    if (hook == PV_HOOK_AUTH_SWITCHUSERCONTEXT || hook == PV_HOOK_AUTH_SWITCHTOKENCONTEXT) {
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_AUTH_REVOKETOKEN) {
        g_auth_valid = 0;
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_X509_FETCHCERTIFICATE || hook == PV_HOOK_X509_GETCERTINFO ||
        hook == PV_HOOK_X509_GENERATECSR || hook == PV_HOOK_X509_GENERATEKEYPAIR) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_x509_pem, (uint32_t)strlen(g_x509_pem));
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_X509_STORECERTIFICATE || hook == PV_HOOK_X509_VERIFYCERTCHAIN) {
        ctx->regs[rd] = 1;
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_X509_ISCERTVALID) {
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_X509_GETKEYHANDLE) {
        ctx->regs[rd] = 1;
        return 1;
    }
    return 0;
}

/* ---- Context ← Req.* when HTTP pool populated ------------------------- */
static int g_scratch[64];

static int emu_context(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook == PV_HOOK_CONTEXT_GETVERB) {
        if (ctx->req_method && ctx->req_method_len > 0)
            ctx->regs[rd] = pv_span_from_bytes(ctx, ctx->req_method,
                                               (uint32_t)ctx->req_method_len);
        else ctx->regs[rd] = 0;
        ctx->host_status = ctx->regs[rd] ? 0 : 1;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETPATH) {
        if (ctx->req_path && ctx->req_path_len > 0)
            ctx->regs[rd] = pv_span_from_bytes(ctx, ctx->req_path,
                                               (uint32_t)ctx->req_path_len);
        else ctx->regs[rd] = 0;
        ctx->host_status = ctx->regs[rd] ? 0 : 1;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETHEADERS) {
        if (ctx->req_headers && ctx->req_headers_len > 0)
            ctx->regs[rd] = pv_span_from_bytes(ctx, ctx->req_headers,
                                               (uint32_t)ctx->req_headers_len);
        else ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETBODY) {
        if (ctx->req_body && ctx->req_body_len > 0)
            ctx->regs[rd] = pv_span_from_bytes(ctx, ctx->req_body,
                                               (uint32_t)ctx->req_body_len);
        else ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETHOST) {
        const char *h = "localhost";
        ctx->regs[rd] = pv_span_from_bytes(ctx, h, (uint32_t)strlen(h));
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETPORT) {
        ctx->regs[rd] = 80;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETREMOTEADDR) {
        const char *h = "127.0.0.1";
        ctx->regs[rd] = pv_span_from_bytes(ctx, h, (uint32_t)strlen(h));
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETUSER) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_auth_user, (uint32_t)strlen(g_auth_user));
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETPERMISSIONS) {
        const char *p = "read,write,admin";
        ctx->regs[rd] = pv_span_from_bytes(ctx, p, (uint32_t)strlen(p));
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETQUERYSTRING) {
        /* path may include ?query */
        if (ctx->req_path) {
            int i;
            for (i = 0; i < ctx->req_path_len; i++)
                if (ctx->req_path[i] == '?') {
                    ctx->regs[rd] = pv_span_from_bytes(
                        ctx, ctx->req_path + i + 1,
                        (uint32_t)(ctx->req_path_len - i - 1));
                    return 1;
                }
        }
        ctx->regs[rd] = 0;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_SETSCRATCHVALUE) {
        int k = ctx->regs[rs1] & 63;
        g_scratch[k] = ctx->regs[rs2];
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETSCRATCHVALUE) {
        int k = ctx->regs[rs1] & 63;
        ctx->regs[rd] = g_scratch[k];
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETREQUESTID || hook == PV_HOOK_CONTEXT_GETTRACEID) {
        char id[32];
        snprintf(id, sizeof(id), "req-%d", g_elapsed_ms);
        ctx->regs[rd] = pv_span_from_bytes(ctx, id, (uint32_t)strlen(id));
        return 1;
    }
    if (hook == PV_HOOK_CONTEXT_GETCLIENTCERT) {
        ctx->regs[rd] = 0;
        ctx->host_status = 1;
        return 1;
    }
    return 0;
}

/* ---- Http live client (hosted only) ----------------------------------- */
static int g_http_status;
static char g_http_headers[2048];
static char g_http_body[8192];
static int g_http_headers_len, g_http_body_len;

static int http_client_request(const char *host, int port, const char *path,
                               const char *method, const char *body, int body_len)
{
#if !defined(__STDC_HOSTED__) || !__STDC_HOSTED__
    (void)host; (void)port; (void)path; (void)method; (void)body; (void)body_len;
    return -1;
#else
#ifdef _WIN32
    WSADATA wsa;
    SOCKET s;
#else
    int s;
#endif
    struct addrinfo hints, *res = 0;
    char portstr[16], req[1024];
    int n, total = 0;
    char resp[16384];

    g_http_status = 0;
    g_http_headers_len = 0;
    g_http_body_len = 0;
    g_http_headers[0] = 0;
    g_http_body[0] = 0;

#ifdef _WIN32
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return -1;
#endif
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    snprintf(portstr, sizeof(portstr), "%d", port > 0 ? port : 80);
    if (getaddrinfo(host, portstr, &hints, &res) != 0) return -1;
#ifdef _WIN32
    s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s == INVALID_SOCKET) { freeaddrinfo(res); return -1; }
#else
    s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s < 0) { freeaddrinfo(res); return -1; }
#endif
    if (connect(s, res->ai_addr, (int)res->ai_addrlen) != 0) {
#ifdef _WIN32
        closesocket(s);
#else
        close(s);
#endif
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);
    if (!method) method = "GET";
    if (!path || !path[0]) path = "/";
    if (body && body_len > 0)
        n = snprintf(req, sizeof(req),
                     "%s %s HTTP/1.0\r\nHost: %s\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
                     method, path, host, body_len);
    else
        n = snprintf(req, sizeof(req),
                     "%s %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
                     method, path, host);
#ifdef _WIN32
    send(s, req, n, 0);
    if (body && body_len > 0) send(s, body, body_len, 0);
    while (total < (int)sizeof(resp) - 1) {
        int g = recv(s, resp + total, (int)sizeof(resp) - 1 - total, 0);
        if (g <= 0) break;
        total += g;
    }
    closesocket(s);
#else
    send(s, req, (size_t)n, 0);
    if (body && body_len > 0) send(s, body, (size_t)body_len, 0);
    while (total < (int)sizeof(resp) - 1) {
        int g = (int)recv(s, resp + total, sizeof(resp) - 1 - total, 0);
        if (g <= 0) break;
        total += g;
    }
    close(s);
#endif
    resp[total] = 0;
    if (total > 0) {
        char *blank = strstr(resp, "\r\n\r\n");
        sscanf(resp, "HTTP/%*s %d", &g_http_status);
        if (blank) {
            int hlen = (int)(blank - resp);
            if (hlen > (int)sizeof(g_http_headers) - 1) hlen = (int)sizeof(g_http_headers) - 1;
            memcpy(g_http_headers, resp, (size_t)hlen);
            g_http_headers[hlen] = 0;
            g_http_headers_len = hlen;
            g_http_body_len = total - hlen - 4;
            if (g_http_body_len > (int)sizeof(g_http_body) - 1)
                g_http_body_len = (int)sizeof(g_http_body) - 1;
            if (g_http_body_len > 0)
                memcpy(g_http_body, blank + 4, (size_t)g_http_body_len);
            g_http_body[g_http_body_len] = 0;
        }
    }
    return g_http_status > 0 ? 0 : -1;
#endif
}

static int emu_http_live(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook == PV_HOOK_HTTP_REQUEST) {
        /* rs1 = host span "host:port/path" or host; rs2 = method span optional */
        char endpoint[256], method[16] = "GET";
        char host[128], path[128] = "/";
        int port = 80;
        char *slash, *colon, *endpoint_start;
        span_copy(ctx, ctx->regs[rs1], endpoint, sizeof(endpoint));
        if (ctx->regs[rs2])
            span_copy(ctx, ctx->regs[rs2], method, sizeof(method));
        endpoint_start = endpoint;
        if (strncmp(endpoint_start, "http://", 7) == 0) endpoint_start += 7;
        else if (strncmp(endpoint_start, "https://", 8) == 0) {
            ctx->regs[rd] = 0;
            ctx->host_status = 2;
            return 1;
        }
        slash = strchr(endpoint_start, '/');
        if (slash) {
            strncpy(path, slash, sizeof(path) - 1);
            *slash = 0;
        }
        colon = strchr(endpoint_start, ':');
        if (colon && colon[1] >= '0' && colon[1] <= '9') {
            *colon = 0;
            port = atoi(colon + 1);
        }
        strncpy(host, endpoint_start, sizeof(host) - 1);
        if (http_client_request(host, port, path, method, 0, 0) == 0) {
            ctx->regs[rd] = g_http_status;
            ctx->host_status = 0;
        } else {
            ctx->regs[rd] = 0;
            ctx->host_status = 1;
        }
        return 1;
    }
    if (hook == PV_HOOK_HTTP_RESPSTATUS) {
        ctx->regs[rd] = g_http_status;
        return 1;
    }
    if (hook == PV_HOOK_HTTP_RESPHEADERS) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_http_headers, (uint32_t)g_http_headers_len);
        return 1;
    }
    if (hook == PV_HOOK_HTTP_RESPBODY) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_http_body, (uint32_t)g_http_body_len);
        return 1;
    }
    if (hook == PV_HOOK_HTTP_READHEADER) {
        /* return last response headers */
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_http_headers, (uint32_t)g_http_headers_len);
        return 1;
    }
    if (hook == PV_HOOK_HTTP_READBODY) {
        ctx->regs[rd] = pv_span_from_bytes(ctx, g_http_body, (uint32_t)g_http_body_len);
        return 1;
    }
    if (hook == PV_HOOK_HTTP_GENERATEHEADERS) {
        char hdr[256];
        snprintf(hdr, sizeof(hdr), "HTTP/1.1 %d OK\r\nContent-Type: text/plain\r\n\r\n",
                 ctx->regs[rs1] ? ctx->regs[rs1] : 200);
        ctx->regs[rd] = pv_span_from_bytes(ctx, hdr, (uint32_t)strlen(hdr));
        return 1;
    }
    if (hook == PV_HOOK_HTTP_GENERATERESPONSE) {
        /* rs1 status, rs2 body span — return body as response payload span */
        int h = ctx->regs[rs2];
        uint32_t p = span_ptr(ctx, h);
        int32_t n = span_len(ctx, h);
        if (n > 0 && ctx->mem)
            ctx->regs[rd] = pv_span_from_bytes(ctx, ctx->mem + p, (uint32_t)n);
        else ctx->regs[rd] = 0;
        g_http_status = ctx->regs[rs1] ? ctx->regs[rs1] : 200;
        return 1;
    }
    return 0;
}

/* ---- Card physical reader stub → soft card ---------------------------- */
static int emu_card(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    (void)rs1; (void)rs2;
    if (hook == PV_HOOK_CARD_READ) {
        const char *d = "SOFTCARD";
        ctx->regs[rd] = pv_span_from_bytes(ctx, d, 8);
        ctx->host_status = 0;
        return 1;
    }
    if (hook == PV_HOOK_CARD_WRITE) {
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_CARD_ADDRESS) {
        ctx->regs[rd] = 1;
        return 1;
    }
    return 0;
}

int pv_emu_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (!ctx) return 0;
    /* Pure DateTime (wall clock handled by pv_host_provider when installed) */
    if (hook >= PV_HOOK_DATETIME_PARSE && hook <= PV_HOOK_DATETIME_DAY)
        return emu_datetime(ctx, hook, rd, rs1, rs2);
    if (hook >= PV_HOOK_GPIO_COUNT && hook <= PV_HOOK_GPIO_READ)
        return emu_gpio(ctx, hook, rd, rs1, rs2);
    if ((hook >= PV_HOOK_DEVICE_OPEN && hook <= PV_HOOK_DEVICE_STATUS) ||
        (hook >= PV_HOOK_STREAM_OPEN && hook <= PV_HOOK_STREAM_SLICE))
        return emu_device_stream(ctx, hook, rd, rs1, rs2);
    if ((hook >= PV_HOOK_EVENT_POST && hook <= PV_HOOK_EVENT_COUNT) ||
        (hook >= PV_HOOK_EVENT_SETSLICE && hook <= PV_HOOK_EVENT_DATALEN))
        return emu_event(ctx, hook, rd, rs1, rs2);
    if (hook >= PV_HOOK_UI_WINDOW && hook <= PV_HOOK_UI_SERIALIZE)
        return emu_ui(ctx, hook, rd, rs1, rs2);
    if ((hook >= PV_HOOK_PROCESS_SELF && hook <= PV_HOOK_ENV_KEY) ||
        (hook >= PV_HOOK_TIMER_AFTER && hook <= PV_HOOK_SCHEDULER_TICK) ||
        (hook >= PV_HOOK_PRINCIPAL_CURRENT && hook <= PV_HOOK_SANDBOX_DENY))
        return emu_process_env_timer(ctx, hook, rd, rs1, rs2);
    if (hook >= PV_HOOK_LOCALE_GETCURRENTLOCALE && hook <= PV_HOOK_LOCALE_TRANSLATE)
        return emu_locale(ctx, hook, rd, rs1, rs2);
    if ((hook >= PV_HOOK_AUTH_GETUSERCREDENTIALS && hook <= PV_HOOK_AUTH_REVOKETOKEN) ||
        (hook >= PV_HOOK_X509_FETCHCERTIFICATE && hook <= PV_HOOK_X509_GETKEYHANDLE)) {
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
        {
            extern int pv_auth_store_dispatch(pv_ctx *c, int h, int r, int a, int b);
            if (pv_auth_store_dispatch(ctx, hook, rd, rs1, rs2)) return 1;
        }
#endif
        return emu_auth_x509(ctx, hook, rd, rs1, rs2);
    }
    if (hook >= PV_HOOK_CONTEXT_GETVERB && hook <= PV_HOOK_CONTEXT_GETTRACEID)
        return emu_context(ctx, hook, rd, rs1, rs2);
    if (hook >= PV_HOOK_HTTP_READHEADER && hook <= PV_HOOK_HTTP_RESPBODY &&
        !(hook >= PV_HOOK_HTTP_PARSEQUERY && hook <= PV_HOOK_HTTP_ENCODEJSON))
        return emu_http_live(ctx, hook, rd, rs1, rs2);
    if (hook == PV_HOOK_CARD_READ || hook == PV_HOOK_CARD_WRITE ||
        hook == PV_HOOK_CARD_ADDRESS)
        return emu_card(ctx, hook, rd, rs1, rs2);
    return 0;
}
