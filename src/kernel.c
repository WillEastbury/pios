/*
 * kernel.c - PIOS main entry point
 *
 * Boot flow (all on core 0):
 *   1. UART init (debug serial)
 *   2. Framebuffer init (HDMI text diagnostics)
 *   3. FIFO init (inter-core messaging)
 *   4. SD init (raw block storage)
 *   5. GENET init (Ethernet MAC/PHY)
 *   6. Net stack init (ARP/IP/UDP/ICMP)
 *   7. Boot diagnostics screen
 *   8. Start cores 1-3
 *   9. Core 0 enters network poll loop
 */

#include "types.h"
#include "uart.h"
#include "fb.h"
#include "fifo.h"
#include "sd.h"
#include "genet.h"
#include "net.h"
#include "tcp.h"
#include "dhcp.h"
#include "dns.h"
#include "core.h"
#include "core_env.h"
#include "simd.h"
#include "mmu.h"
#include "gic.h"
#include "exception.h"
#include "timer.h"
#include "dma.h"
#include "gpu.h"
#include "tensor.h"
#include "walfs.h"
#include "bcache.h"
#include "principal.h"
#include "proc.h"
#include "pix.h"
#include "pcie.h"
#include "rp1.h"
#include "rp1_gpio.h"
#include "rp1_clk.h"
#include "rp1_uart.h"
#include "usb.h"
#include "usb_storage.h"
#include "usb_kbd.h"
#include "ipc_queue.h"
#include "ipc_stream.h"
#include "ipc_proc.h"
#include "pipe.h"
#include "setup.h"
#include "ksem.h"
#include "workq.h"
#include "picowal_db.h"
#include "el2.h"
#include "crypto.h"

/* ---- libc replacements (linked globally for compiler-generated calls) ---- */

void *memset(void *dst, int c, usize n) {
    u8 *p = (u8 *)dst;
    while (n--) *p++ = (u8)c;
    return dst;
}

void *memcpy(void *dst, const void *src, usize n) {
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    while (n--) *d++ = *s++;
    return dst;
}

int memcmp(const void *a, const void *b, usize n) {
    const u8 *pa = (const u8 *)a;
    const u8 *pb = (const u8 *)b;
    while (n--) {
        if (*pa != *pb) return *pa - *pb;
        pa++; pb++;
    }
    return 0;
}

u32 pios_strlen(const char *s) {
    u32 n = 0;
    while (*s++) n++;
    return n;
}

/* ---- Network configuration (static - no ARP/DHCP) ---- */

#define MY_IP       IP4(10, 0, 0, 2)
#define MY_GW       IP4(10, 0, 0, 1)
#define MY_MASK     IP4(255, 255, 255, 0)

/* Gateway MAC - MUST be configured (no ARP to discover it) */
static const u8 MY_GW_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define UI_MODE_NONE          0
#define UI_MODE_PROC_VIEW     1
#define UI_MODE_PROC_MANAGER  2
#define UI_MODE_CONSOLE       3
#define UI_MODE_SCHEDULER     4
#define UI_SNAPSHOT_MAX       MAX_PROCS_PER_CORE
#define UI_CONSOLE_LINE_MAX   256
#define UI_CONSOLE_ARGV_MAX   16
#define UI_STREAM_IN_MAX      1472
#define UI_STREAM_OUT_MAX     2048
#define UI_ENV_MAX            32
#define UI_BATCH_MAX          32
#define UI_DB_UDP_VER         1
#define UI_SHELL_TEXT_COLOR   0x004C1966
#define UI_EDIT_MAX_LINES     128
#define UI_EDIT_LINE_MAX      120

static u32 ui_mode;
static u32 ui_selected;
static u64 ui_last_render;
static i32 ui_launch_idx;
static i32 ui_status_code;
static char ui_console_line[UI_CONSOLE_LINE_MAX];
static u32 ui_console_len;
static char ui_sched_line[UI_CONSOLE_LINE_MAX];
static u32 ui_sched_len;
static char ui_cwd[256] = "/";
static u32 ui_cfg_ip;
static u32 ui_cfg_mask;
static u32 ui_cfg_gw;
static u32 ui_cfg_dns;
static bool ui_cfg_dhcp;
static const char *ui_proc_state_str(u32 s);
static void ui_dump_sector(u32 lba);
static void ui_cmd_fsinspect(const char *path);
static void ui_cmd_netcfg(u32 argc, char **argv);
static void ui_cmd_disk(u32 argc, char **argv);
static void ui_cmd_db(u32 argc, char **argv);
static void ui_cmd_lsdir(const char *path);
static void ui_cmd_mkdir(const char *path);
static void ui_cmd_touch(const char *path);
static void ui_cmd_copy(const char *src, const char *dst);
static void ui_cmd_cpdir(const char *src, const char *dst);
static bool ui_path_resolve(const char *in, char *out, u32 out_max);
static void ui_cmd_cat(const char *path);
static void ui_cmd_stat(const char *path);
static void ui_cmd_rm(const char *path);
static void ui_cmd_stream(u32 argc, char **argv);
static void ui_cmd_mv(const char *src, const char *dst);
static void ui_cmd_hexdump(const char *path, u32 max_bytes);
static void ui_cmd_find(const char *base, const char *needle);
static void ui_cmd_df(void);
static void ui_cmd_mount(u32 argc, char **argv);
static void ui_cmd_env(u32 argc, char **argv);
static void ui_cmd_if(u32 argc, char **argv);
static void ui_cmd_for(u32 argc, char **argv);
static void ui_cmd_foreach(u32 argc, char **argv);
static void ui_cmd_source(const char *path);
static void ui_cmd_edit(const char *path);
static void ui_cmd_capsule(u32 argc, char **argv);
static void ui_console_exec(char *line);
static bool ui_parse_priority(const char *s, u32 *out_prio);
static const char *ui_priority_str(u32 p);
static bool ui_resolve_pis_path(const char *in, char *out, u32 out_max);
static bool ui_resolve_pix_path(const char *in, char *out, u32 out_max);
static bool ui_resolve_job_path(const char *in, char *out, u32 out_max, bool *is_script_out);
static void ui_cmd_batch(u32 argc, char **argv);
static void ui_batch_tick(void);
static void ui_cmd_svc(u32 argc, char **argv);
static void ui_service_tick(void);
static void ui_render_scheduler(void);
static void ui_scheduler_feed_char(i32 c);
static void disk_handle_request(u32 from_core);
static void ui_db_udp_cb(u32 src_ip, u16 src_port, u16 dst_port, const u8 *data, u16 len);
static bool ui_env_get(const char *key, const char **val_out);
static void ui_env_set(const char *key, const char *val, bool persistent);
static bool ui_env_save(void);
static bool ui_env_load(void);
static u32 ui_read_tty_line(char *out, u32 out_max, const char *prompt);
static bool ui_cap_manifest_validate_buf(const char *buf, u32 n, char *err, u32 err_max);

extern u8 __text_start;
extern u8 __text_end;
extern u8 __el2_integrity_start;
extern u8 __el2_integrity_end;

#define BOOT_POLICY_MAGIC   0x42504C59U /* BPLY */
#define BOOT_POLICY_VERSION 1U
#define BOOT_POLICY_CARD    0U
#define BOOT_POLICY_REC     10U
#define BOOT_ROLLBACK_REC   11U

struct boot_policy_record {
    u32 magic;
    u32 version;
    u32 el1_hash;
    u32 el2_hash;
    u8 mac[32];
} PACKED;

static const u8 boot_policy_hmac_key[32] = {
    0x79,0xA1,0x34,0x5D,0x9C,0xE2,0x11,0x6B,0x43,0x88,0x2F,0xC0,0x7A,0xD3,0x59,0xBE,
    0x10,0x4F,0x92,0xCC,0x61,0x2A,0xE7,0x35,0xB9,0x08,0xF1,0x6D,0x54,0xAB,0x3E,0xC7
};

static u32 boot_el1_expected_hash;
static u32 boot_el2_expected_hash;

#define UI_SVC_MAX 16
#define UI_SVC_TARGET_DEFAULT 1U
#define UI_SVC_TARGET_RESCUE  2U
#define UI_SVC_TARGET_ALL     3U
#define UI_SVC_RP_NEVER       0U
#define UI_SVC_RP_ONFAIL      1U
#define UI_SVC_RP_ALWAYS      2U

struct ui_service_unit {
    bool used;
    char name[32];
    char path[128];
    char depends[32];
    u32 target;
    u32 preferred_core;
    u32 principal_id;
    u32 priority_class;
    u32 restart_policy;
    u32 max_restarts;
    u32 backoff_ms;
    u32 state; /* 0 stopped, 1 running, 2 backoff, 3 failed */
    bool stop_requested;
    i32 pid;
    u32 restarts;
    u64 window_start_ms;
    u64 next_action_ms;
};
static struct ui_service_unit ui_services[UI_SVC_MAX];
static bool ui_service_running;
static u32 ui_service_target = UI_SVC_TARGET_DEFAULT;

struct ui_batch_job {
    bool used;
    u32 id;
    char path[128];
    u32 preferred_core; /* 0=auto, 1|2|3 fixed */
    u32 principal_id;   /* principal to run as */
    u32 priority_class; /* PROC_PRIO_* */
    u32 state;          /* 0 queued,1 running,2 done,3 failed,4 canceled */
    i32 pid;
    u32 attempts;
    u32 retries;
    i32 last_err;
    u64 next_due_ms;
    u32 interval_ms;    /* 0 one-shot, >0 recurring */
    bool is_script;
};
static struct ui_batch_job ui_batch_jobs[UI_BATCH_MAX];
static u32 ui_batch_next_id = 1;
static u32 ui_batch_parallel = 2;
static bool ui_batch_running;

struct ui_env_var {
    char key[32];
    char val[128];
    bool used;
    bool persistent;
};
static struct ui_env_var ui_env[UI_ENV_MAX];
static bool ui_env_loaded;
static u32 ui_script_depth;

struct ui_stream_udp_rx {
    bool waiting;
    bool ready;
    u32 src_ip;
    u16 src_port;
    u16 len;
    u8 data[1472];
};
static struct ui_stream_udp_rx ui_stream_udp;

static const char *const ui_launch_candidates[] = {
    "/bin/console.pix",
    "/bin/hexview.pix",
    "/bin/init.pix",
    "/bin/shell.pix",
    "/bin/edit.pix",
    "/bin/demo.pix",
    "/app/main.pix",
};

static void boot_measurements(u32 *el1_hash, u32 *el2_hash, u64 *el1_start, u32 *el1_len)
{
    u64 el1_s = (u64)(usize)&__text_start;
    u64 el1_e = (u64)(usize)&__text_end;
    u64 el2_s = (u64)(usize)&__el2_integrity_start;
    u64 el2_e = (u64)(usize)&__el2_integrity_end;
    if (el1_e <= el1_s || el2_e <= el2_s)
        exception_pisod("Boot integrity map invalid", 5, 0x3C, 0, 0, 0);
    if (el1_hash) *el1_hash = hw_crc32c((const void *)(usize)el1_s, (u32)(el1_e - el1_s));
    if (el2_hash) *el2_hash = hw_crc32c((const void *)(usize)el2_s, (u32)(el2_e - el2_s));
    if (el1_start) *el1_start = el1_s;
    if (el1_len) *el1_len = (u32)(el1_e - el1_s);
}

static void boot_policy_mac(const struct boot_policy_record *r, u8 out[32])
{
    if (!r || !out) return;
    hmac_sha256(boot_policy_hmac_key, sizeof(boot_policy_hmac_key),
                (const u8 *)r, (u32)(sizeof(*r) - sizeof(r->mac)), out);
}

static bool boot_policy_mac_ok(const struct boot_policy_record *r)
{
    if (!r) return false;
    u8 calc[32];
    boot_policy_mac(r, calc);
    u32 diff = 0;
    for (u32 i = 0; i < 32; i++)
        diff |= (u32)(calc[i] ^ r->mac[i]);
    return diff == 0;
}

static void boot_policy_verify_or_seed(void)
{
    u32 cur_el1 = 0, cur_el2 = 0;
    boot_measurements(&cur_el1, &cur_el2, NULL, NULL);
    struct boot_policy_record rec;
    i32 n = picowal_db_get(BOOT_POLICY_CARD, BOOT_POLICY_REC, &rec, sizeof(rec));
    if (n < (i32)sizeof(rec)) {
        rec.magic = BOOT_POLICY_MAGIC;
        rec.version = BOOT_POLICY_VERSION;
        rec.el1_hash = cur_el1;
        rec.el2_hash = cur_el2;
        boot_policy_mac(&rec, rec.mac);
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_POLICY_REC, &rec, sizeof(rec)) < 0)
            exception_pisod("Boot policy seed failed", 5, 0x39, 0, 0, 0);
    } else {
        if (rec.magic != BOOT_POLICY_MAGIC || !boot_policy_mac_ok(&rec))
            exception_pisod("Boot policy signature fail", 5, 0x38, 0, 0, 0);
        if (rec.el1_hash != cur_el1 || rec.el2_hash != cur_el2)
            exception_pisod("Boot policy hash mismatch", 5, 0x37, 0, 0, 0);
    }

    u32 rollback_floor = 0;
    i32 rn = picowal_db_get(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor));
    if (rn < (i32)sizeof(rollback_floor)) {
        rollback_floor = rec.version;
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor)) < 0)
            exception_pisod("Rollback floor seed failed", 5, 0x36, 0, 0, 0);
    } else if (rec.version < rollback_floor) {
        exception_pisod("Boot rollback blocked", 5, 0x35, 0, 0, 0);
    }
    if (rec.version > rollback_floor) {
        rollback_floor = rec.version;
        if (picowal_db_put(BOOT_POLICY_CARD, BOOT_ROLLBACK_REC, &rollback_floor, sizeof(rollback_floor)) < 0)
            exception_pisod("Rollback floor update fail", 5, 0x34, 0, 0, 0);
    }

    boot_el1_expected_hash = rec.el1_hash;
    boot_el2_expected_hash = rec.el2_hash;
}

static bool ui_streq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++;
        b++;
    }
    return (*a == 0 && *b == 0);
}

static bool ui_strneq(const char *a, const char *b, u32 n)
{
    if (!a || !b) return false;
    for (u32 i = 0; i < n; i++) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

static bool ui_parse_u32(const char *s, u32 *out)
{
    if (!s || !*s || !out) return false;
    u32 base = 10;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        if (!*s) return false;
    }
    u32 v = 0;
    while (*s) {
        u32 d;
        char c = *s++;
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return false;
        if (d >= base) return false;
        v = v * base + d;
    }
    *out = v;
    return true;
}

static bool ui_parse_ip4(const char *s, u32 *out)
{
    if (!s || !out) return false;
    u32 oct[4] = {0,0,0,0};
    u32 idx = 0;
    u32 cur = 0;
    bool have = false;
    while (*s) {
        char c = *s++;
        if (c >= '0' && c <= '9') {
            have = true;
            cur = cur * 10U + (u32)(c - '0');
            if (cur > 255U) return false;
        } else if (c == '.') {
            if (!have || idx >= 3) return false;
            oct[idx++] = cur;
            cur = 0;
            have = false;
        } else {
            return false;
        }
    }
    if (!have || idx != 3) return false;
    oct[3] = cur;
    *out = IP4(oct[0], oct[1], oct[2], oct[3]);
    return true;
}

static bool ui_parse_priority(const char *s, u32 *out_prio)
{
    if (!s || !out_prio) return false;
    if (ui_streq(s, "lazy")) { *out_prio = PROC_PRIO_LAZY; return true; }
    if (ui_streq(s, "low")) { *out_prio = PROC_PRIO_LOW; return true; }
    if (ui_streq(s, "normal")) { *out_prio = PROC_PRIO_NORMAL; return true; }
    if (ui_streq(s, "high")) { *out_prio = PROC_PRIO_HIGH; return true; }
    if (ui_streq(s, "realtime")) { *out_prio = PROC_PRIO_REALTIME; return true; }
    return false;
}

static const char *ui_priority_str(u32 p)
{
    if (p == PROC_PRIO_LAZY) return "lazy";
    if (p == PROC_PRIO_LOW) return "low";
    if (p == PROC_PRIO_HIGH) return "high";
    if (p == PROC_PRIO_REALTIME) return "realtime";
    return "normal";
}

static bool ui_parse_mac6(const char *s, u8 out[6])
{
    if (!s || !out) return false;
    for (u32 i = 0; i < 6; i++) {
        char a = *s++;
        char b = *s++;
        if (!a || !b) return false;
        u32 hi, lo;
        if (a >= '0' && a <= '9') hi = (u32)(a - '0');
        else if (a >= 'a' && a <= 'f') hi = (u32)(a - 'a' + 10);
        else if (a >= 'A' && a <= 'F') hi = (u32)(a - 'A' + 10);
        else return false;
        if (b >= '0' && b <= '9') lo = (u32)(b - '0');
        else if (b >= 'a' && b <= 'f') lo = (u32)(b - 'a' + 10);
        else if (b >= 'A' && b <= 'F') lo = (u32)(b - 'A' + 10);
        else return false;
        out[i] = (u8)((hi << 4) | lo);
        if (i < 5) {
            char sep = *s++;
            if (sep != ':') return false;
        }
    }
    return *s == 0;
}

static void ui_console_write(const char *s)
{
    uart_puts(s);
    fb_puts(s);
}

static void ui_console_prompt(void)
{
    ui_console_write("console> ");
}

static void ui_print_ip(u32 ip)
{
    u32 a = (ip >> 24) & 0xFF;
    u32 b = (ip >> 16) & 0xFF;
    u32 c = (ip >> 8) & 0xFF;
    u32 d = ip & 0xFF;
    fb_printf("%u.%u.%u.%u", a, b, c, d);
    uart_puts("[");
    uart_hex(a); uart_putc('.');
    uart_hex(b); uart_putc('.');
    uart_hex(c); uart_putc('.');
    uart_hex(d); uart_puts("]");
}

static void ui_console_print_ps(void)
{
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    ui_console_write("PID      AFF  PRI       CPU%  MEM(KiB)  STATE\n");
    for (u32 i = 0; i < n; i++) {
        fb_printf("0x%x   %u    %s   %u    %u       %s\n",
                  snap[i].pid, snap[i].affinity_core, ui_priority_str(snap[i].priority_class), snap[i].cpu_percent,
                  snap[i].mem_kib, ui_proc_state_str(snap[i].state));
        uart_hex(snap[i].pid);
        uart_puts(" aff=");
        uart_hex(snap[i].affinity_core);
        uart_puts(" pri=");
        uart_puts(ui_priority_str(snap[i].priority_class));
        uart_puts(" cpu=");
        uart_hex(snap[i].cpu_percent);
        uart_puts(" mem=");
        uart_hex(snap[i].mem_kib);
        uart_puts(" state=");
        uart_puts(ui_proc_state_str(snap[i].state));
        uart_puts("\n");
    }
}

struct fsinspect_ctx {
    u32 count;
    u32 max;
};
static struct fsinspect_ctx fs_ctx;

struct ui_dir_entry {
    u64 id;
    char name[128];
};
struct ui_dir_collect_ctx {
    struct ui_dir_entry *out;
    u32 max;
    u32 count;
};
static struct ui_dir_collect_ctx dir_collect_ctx;

static bool ui_path_split_parent(const char *path, char *parent, u32 parent_max, char *leaf, u32 leaf_max)
{
    if (!path || path[0] != '/' || !parent || !leaf) return false;
    u32 len = pios_strlen(path);
    if (len < 2) return false;
    while (len > 1 && path[len - 1] == '/') len--;
    if (len < 2) return false;
    i32 slash = -1;
    for (u32 i = 0; i < len; i++) if (path[i] == '/') slash = (i32)i;
    if (slash < 0 || (u32)slash >= len - 1) return false;
    u32 nlen = len - (u32)slash - 1;
    if (nlen + 1 > leaf_max) return false;
    for (u32 i = 0; i < nlen; i++) leaf[i] = path[(u32)slash + 1 + i];
    leaf[nlen] = 0;
    if (slash == 0) {
        if (parent_max < 2) return false;
        parent[0] = '/'; parent[1] = 0;
        return true;
    }
    if ((u32)slash + 1 > parent_max) return false;
    for (u32 i = 0; i < (u32)slash; i++) parent[i] = path[i];
    parent[slash] = 0;
    return true;
}

static bool ui_path_join(const char *base, const char *name, char *out, u32 out_max)
{
    if (!base || !name || !out || out_max < 2) return false;
    u32 bl = pios_strlen(base);
    u32 nl = pios_strlen(name);
    bool root = (bl == 1 && base[0] == '/');
    u32 need = bl + nl + (root ? 0 : 1) + 1;
    if (need > out_max) return false;
    u32 p = 0;
    for (u32 i = 0; i < bl; i++) out[p++] = base[i];
    if (!root) out[p++] = '/';
    for (u32 i = 0; i < nl; i++) out[p++] = name[i];
    out[p] = 0;
    return true;
}

static bool ui_fs_create_path(const char *path, bool is_dir)
{
    char parent[256];
    char leaf[128];
    if (!ui_path_split_parent(path, parent, sizeof(parent), leaf, sizeof(leaf)))
        return false;
    u64 parent_id = walfs_find(parent);
    if (!parent_id) return false;
    u32 flags = is_dir ? WALFS_DIR : WALFS_FILE;
    u64 id = walfs_create(parent_id, leaf, flags, 0644);
    return id != 0;
}

static bool ui_path_resolve(const char *in, char *out, u32 out_max)
{
    if (!in || !*in || !out || out_max < 2) return false;
    if (ui_streq(in, ".")) in = ui_cwd;
    if (in[0] == '/') {
        u32 n = pios_strlen(in);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i < n; i++) out[i] = in[i];
        while (n > 1 && out[n - 1] == '/') n--;
        out[n] = 0;
        return true;
    }
    if (ui_streq(in, "..")) {
        u32 n = pios_strlen(ui_cwd);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i < n; i++) out[i] = ui_cwd[i];
        if (n > 1) {
            while (n > 1 && out[n - 1] != '/') n--;
            if (n == 1) out[n] = 0;
            else out[n - 1] = 0;
        } else out[1] = 0;
        return true;
    }
    if (ui_cwd[0] == '/' && ui_cwd[1] == 0) {
        u32 nl = pios_strlen(in);
        if (nl + 2 > out_max) return false;
        out[0] = '/';
        for (u32 i = 0; i < nl; i++) out[i + 1] = in[i];
        out[nl + 1] = 0;
        return true;
    }
    u32 cl = pios_strlen(ui_cwd);
    u32 nl = pios_strlen(in);
    if (cl + nl + 2 > out_max) return false;
    u32 p = 0;
    for (u32 i = 0; i < cl; i++) out[p++] = ui_cwd[i];
    out[p++] = '/';
    for (u32 i = 0; i < nl; i++) out[p++] = in[i];
    out[p] = 0;
    return true;
}

static bool ui_has_suffix(const char *s, const char *suffix)
{
    if (!s || !suffix) return false;
    u32 sl = pios_strlen(s);
    u32 tl = pios_strlen(suffix);
    if (tl > sl) return false;
    for (u32 i = 0; i < tl; i++) {
        if (s[sl - tl + i] != suffix[i]) return false;
    }
    return true;
}

static bool ui_resolve_pis_path(const char *in, char *out, u32 out_max)
{
    char abs[256];
    if (!ui_path_resolve(in, abs, sizeof(abs))) return false;
    if (ui_has_suffix(abs, ".pis")) {
        u32 n = pios_strlen(abs);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = abs[i];
        return true;
    }
    if (pios_strlen(abs) + 4 + 1 > out_max) return false;
    u32 p = 0;
    while (abs[p]) { out[p] = abs[p]; p++; }
    out[p++] = '.';
    out[p++] = 'p';
    out[p++] = 'i';
    out[p++] = 's';
    out[p] = 0;
    if (walfs_find(out)) return true;
    p -= 4;
    out[p] = 0;
    return true;
}

static bool ui_resolve_pix_path(const char *in, char *out, u32 out_max)
{
    char abs[256];
    if (!ui_path_resolve(in, abs, sizeof(abs))) return false;
    if (ui_has_suffix(abs, ".pix")) {
        u32 n = pios_strlen(abs);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = abs[i];
        return true;
    }
    if (pios_strlen(abs) + 4 + 1 > out_max) return false;
    u32 p = 0;
    while (abs[p]) { out[p] = abs[p]; p++; }
    out[p++] = '.';
    out[p++] = 'p';
    out[p++] = 'i';
    out[p++] = 'x';
    out[p] = 0;
    if (walfs_find(out)) return true;
    p -= 4;
    out[p] = 0;
    return true;
}

static bool ui_resolve_job_path(const char *in, char *out, u32 out_max, bool *is_script_out)
{
    char pix[256];
    char pis[256];
    if (is_script_out) *is_script_out = false;
    if (!ui_resolve_pix_path(in, pix, sizeof(pix))) return false;
    if (walfs_find(pix)) {
        u32 n = pios_strlen(pix);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = pix[i];
        return true;
    }
    if (!ui_resolve_pis_path(in, pis, sizeof(pis))) return false;
    if (walfs_find(pis)) {
        u32 n = pios_strlen(pis);
        if (n + 1 > out_max) return false;
        for (u32 i = 0; i <= n; i++) out[i] = pis[i];
        if (is_script_out) *is_script_out = ui_has_suffix(pis, ".pis");
        return true;
    }
    return false;
}

static void ui_dir_collect_cb(const struct walfs_dirent *entry)
{
    if (!entry || dir_collect_ctx.count >= dir_collect_ctx.max) return;
    struct ui_dir_entry *d = &dir_collect_ctx.out[dir_collect_ctx.count++];
    d->id = entry->child_id;
    for (u32 i = 0; i < 127; i++) {
        d->name[i] = (char)entry->name[i];
        if (entry->name[i] == 0) break;
    }
    d->name[127] = 0;
}

static void ui_fsinspect_cb(const struct walfs_dirent *entry)
{
    if (!entry || fs_ctx.count >= fs_ctx.max)
        return;
    struct walfs_inode ino;
    if (!walfs_stat(entry->child_id, &ino))
        return;
    fb_printf("  %s  id=0x%x size=%u %s\n",
              ino.name, (u32)ino.inode_id, (u32)ino.size,
              (ino.flags & WALFS_DIR) ? "<dir>" : "<file>");
    uart_puts("  ");
    uart_puts((const char *)ino.name);
    uart_puts(" id=");
    uart_hex((u32)ino.inode_id);
    uart_puts(" size=");
    uart_hex((u32)ino.size);
    uart_puts((ino.flags & WALFS_DIR) ? " <dir>\n" : " <file>\n");
    fs_ctx.count++;
}

static void ui_cmd_fsinspect(const char *path)
{
    if (!path || !*path) {
        ui_console_write("ERR: usage fsinspect <path>\n");
        return;
    }
    u64 id = walfs_find(path);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino)) {
        ui_console_write("ERR: stat failed\n");
        return;
    }
    fb_printf("path=%s id=0x%x size=%u flags=0x%x mode=0x%x\n",
              path, (u32)id, (u32)ino.size, ino.flags, ino.mode);
    uart_puts("path=");
    uart_puts(path);
    uart_puts(" id=");
    uart_hex((u32)id);
    uart_puts(" size=");
    uart_hex((u32)ino.size);
    uart_puts(" flags=");
    uart_hex(ino.flags);
    uart_puts(" mode=");
    uart_hex(ino.mode);
    uart_puts("\n");

    if (ino.flags & WALFS_DIR) {
        fs_ctx.count = 0;
        fs_ctx.max = 64;
        walfs_readdir(id, ui_fsinspect_cb);
        fb_printf("entries=%u\n", fs_ctx.count);
        uart_puts("entries=");
        uart_hex(fs_ctx.count);
        uart_puts("\n");
    }
}

static void ui_cmd_netcfg(u32 argc, char **argv)
{
    if (argc >= 2 && ui_streq(argv[1], "set")) {
        if (argc < 4) {
            ui_console_write("ERR: usage netcfg set <ip|mask|gw|dns> <a.b.c.d>\n");
            return;
        }
        u32 v = 0;
        if (!ui_parse_ip4(argv[3], &v)) {
            ui_console_write("ERR: invalid ip value\n");
            return;
        }
        if (ui_streq(argv[2], "ip")) ui_cfg_ip = v;
        else if (ui_streq(argv[2], "mask")) ui_cfg_mask = v;
        else if (ui_streq(argv[2], "gw")) ui_cfg_gw = v;
        else if (ui_streq(argv[2], "dns")) ui_cfg_dns = v;
        else {
            ui_console_write("ERR: key must be ip|mask|gw|dns\n");
            return;
        }
        ui_console_write("OK: value staged (run 'netcfg apply')\n");
        return;
    }

    if (argc >= 2 && ui_streq(argv[1], "apply")) {
        net_init(ui_cfg_ip, ui_cfg_gw, ui_cfg_mask, NULL);
        dns_init(ui_cfg_dns);
        ui_cfg_dhcp = false;
        ui_console_write("OK: static net config applied\n");
        return;
    }

    if (argc >= 3 && ui_streq(argv[1], "dhcp")) {
        if (ui_streq(argv[2], "on")) {
            u32 timeout_ms = 5000;
            if (argc >= 4) {
                if (!ui_parse_u32(argv[3], &timeout_ms)) {
                    ui_console_write("ERR: invalid timeout\n");
                    return;
                }
            }
            ui_console_write("DHCP: requesting lease...\n");
            if (!dhcp_start(timeout_ms)) {
                ui_console_write("ERR: dhcp failed/timeout\n");
                return;
            }
            const dhcp_lease_t *lease = dhcp_get_lease();
            if (!lease) {
                ui_console_write("ERR: no lease\n");
                return;
            }
            ui_cfg_ip = lease->ip;
            ui_cfg_mask = lease->mask;
            ui_cfg_gw = lease->gateway;
            ui_cfg_dns = lease->dns;
            ui_cfg_dhcp = true;
            dns_init(ui_cfg_dns);
            ui_console_write("OK: dhcp lease applied\n");
            return;
        }
        if (ui_streq(argv[2], "off")) {
            ui_cfg_dhcp = false;
            ui_console_write("OK: dhcp disabled (run 'netcfg apply' for static)\n");
            return;
        }
        ui_console_write("ERR: usage netcfg dhcp <on|off> [timeout_ms]\n");
        return;
    }

    if (argc >= 2 && ui_streq(argv[1], "addnbr")) {
        if (argc < 4) {
            ui_console_write("ERR: usage netcfg addnbr <ip> <mac>\n");
            return;
        }
        u32 ip = 0;
        u8 mac[6];
        if (!ui_parse_ip4(argv[2], &ip) || !ui_parse_mac6(argv[3], mac)) {
            ui_console_write("ERR: invalid ip/mac\n");
            return;
        }
        net_add_neighbor(ip, mac);
        ui_console_write("OK: neighbor added\n");
        return;
    }

    const net_stats_t *st = net_get_stats();
    u8 mac[6];
    genet_get_mac(mac);
    fb_printf("link=%s mode=%s\n", genet_link_up() ? "up" : "down", ui_cfg_dhcp ? "dhcp" : "static");
    fb_printf("ip=");
    ui_print_ip(net_get_our_ip());
    fb_puts(" mask=");
    ui_print_ip(ui_cfg_mask);
    fb_puts(" gw=");
    ui_print_ip(ui_cfg_gw);
    fb_puts(" dns=");
    ui_print_ip(ui_cfg_dns);
    fb_puts("\n");
    fb_printf("mac=%x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    fb_printf("tx=%X rx=%X udp_tx=%X udp_rx=%X drops=%X\n",
              st->tx_packets, st->rx_packets, st->udp_sent, st->udp_recv,
              st->drop_runt + st->drop_bad_cksum + st->drop_fragment + st->drop_ip_options +
              st->drop_bad_src + st->drop_not_for_us + st->drop_bad_proto +
              st->drop_icmp_ratelimit + st->drop_no_neighbor + st->drop_udp_malformed + st->drop_oversized);

    uart_puts("net link=");
    uart_puts(genet_link_up() ? "up" : "down");
    uart_puts(" mode=");
    uart_puts(ui_cfg_dhcp ? "dhcp" : "static");
    uart_puts(" ip=");
    uart_hex(net_get_our_ip());
    uart_puts(" mask=");
    uart_hex(ui_cfg_mask);
    uart_puts(" gw=");
    uart_hex(ui_cfg_gw);
    uart_puts(" dns=");
    uart_hex(ui_cfg_dns);
    uart_puts(" tx=");
    uart_hex((u32)st->tx_packets);
    uart_puts(" rx=");
    uart_hex((u32)st->rx_packets);
    uart_puts("\n");
}

static void ui_cmd_disk(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "info")) {
        const sd_card_t *card = sd_get_card_info();
        fb_printf("disk type=%u rca=0x%x cap_bytes=%X\n", card->type, card->rca, card->capacity);
        uart_puts("disk type=");
        uart_hex(card->type);
        uart_puts(" rca=");
        uart_hex(card->rca);
        uart_puts(" cap=");
        uart_hex(card->capacity);
        uart_puts("\n");
        return;
    }
    if (ui_streq(argv[1], "sync")) {
        walfs_sync();
        ui_console_write("OK: walfs synced\n");
        return;
    }
    if (ui_streq(argv[1], "compact")) {
        bool ok = walfs_compact();
        ui_console_write(ok ? "OK: walfs compacted\n" : "ERR: compact failed\n");
        return;
    }
    if (ui_streq(argv[1], "read")) {
        if (argc < 3) {
            ui_console_write("ERR: usage disk read <lba>\n");
            return;
        }
        u32 lba = 0;
        if (!ui_parse_u32(argv[2], &lba)) {
            ui_console_write("ERR: invalid lba\n");
            return;
        }
        ui_dump_sector(lba);
        return;
    }
    if (ui_streq(argv[1], "writezero")) {
        if (argc < 4 || !ui_streq(argv[3], "--force")) {
            ui_console_write("ERR: usage disk writezero <lba> --force\n");
            return;
        }
        u32 lba = 0;
        if (!ui_parse_u32(argv[2], &lba)) {
            ui_console_write("ERR: invalid lba\n");
            return;
        }
        static u8 z[SD_BLOCK_SIZE] ALIGNED(64);
        simd_zero(z, SD_BLOCK_SIZE);
        bool ok = sd_write_block(lba, z);
        ui_console_write(ok ? "OK: sector zeroed\n" : "ERR: write failed\n");
        return;
    }
    ui_console_write("ERR: usage disk [info|sync|compact|read <lba>|writezero <lba> --force]\n");
}

static void ui_cmd_db(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("db key <card> <record>\n");
        ui_console_write("db put <card> <record> <text...>\n");
        ui_console_write("db putf <card> <record> <path>\n");
        ui_console_write("db get <card> <record>\n");
        ui_console_write("db getf <card> <record> <path>\n");
        ui_console_write("db del <card> <record>\n");
        ui_console_write("db list <card>\n");
        ui_console_write("udp: port 7001 op={1:get,2:put,3:del,4:list} ver=1\n");
        return;
    }

    if (ui_streq(argv[1], "list")) {
        if (argc < 3) {
            ui_console_write("ERR: usage db list <card>\n");
            return;
        }
        u32 card = 0;
        if (!ui_parse_u32(argv[2], &card) || card > PICOWAL_CARD_MAX) {
            ui_console_write("ERR: card out of range (0..1023)\n");
            return;
        }
        u32 ids[64];
        u32 n = picowal_db_list((u16)card, ids, 64);
        fb_printf("db card=%u count=%u\n", card, n);
        for (u32 i = 0; i < n; i++)
            fb_printf("  rec=%u\n", ids[i]);
        return;
    }

    if (argc < 4) {
        ui_console_write("ERR: usage db <op> <card> <record> ...\n");
        return;
    }

    u32 card = 0, rec = 0;
    if (!ui_parse_u32(argv[2], &card) || card > PICOWAL_CARD_MAX) {
        ui_console_write("ERR: card out of range (0..1023)\n");
        return;
    }
    if (!ui_parse_u32(argv[3], &rec) || rec > PICOWAL_RECORD_MAX) {
        ui_console_write("ERR: record out of range (0..4194303)\n");
        return;
    }

    if (ui_streq(argv[1], "key")) {
        u32 key = 0;
        if (!picowal_db_pack_key((u16)card, rec, &key)) {
            ui_console_write("ERR: key pack failed\n");
            return;
        }
        fb_printf("key=0x%x card=%u record=%u\n", key, card, rec);
        return;
    }

    if (ui_streq(argv[1], "del")) {
        if (!picowal_db_delete((u16)card, rec))
            ui_console_write("ERR: delete failed\n");
        else
            ui_console_write("OK: deleted\n");
        return;
    }

    if (ui_streq(argv[1], "put")) {
        if (argc < 5) {
            ui_console_write("ERR: usage db put <card> <record> <text...>\n");
            return;
        }
        static u8 data[PICOWAL_DATA_MAX];
        u32 p = 0;
        for (u32 i = 4; i < argc; i++) {
            const char *s = argv[i];
            while (*s && p < PICOWAL_DATA_MAX) data[p++] = (u8)*s++;
            if (i + 1 < argc && p < PICOWAL_DATA_MAX) data[p++] = ' ';
        }
        if (p == 0 || p >= PICOWAL_DATA_MAX) {
            ui_console_write("ERR: payload too large\n");
            return;
        }
        i32 n = picowal_db_put((u16)card, rec, data, p);
        if (n < 0) ui_console_write("ERR: put failed\n");
        else fb_printf("OK: wrote %u bytes\n", (u32)n);
        return;
    }

    if (ui_streq(argv[1], "putf")) {
        if (argc < 5) {
            ui_console_write("ERR: usage db putf <card> <record> <path>\n");
            return;
        }
        char abs[256];
        if (!ui_path_resolve(argv[4], abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        u64 id = walfs_find(abs);
        if (!id) {
            ui_console_write("ERR: source file not found\n");
            return;
        }
        static u8 data[PICOWAL_DATA_MAX];
        i32 got = (i32)walfs_read(id, 0, data, PICOWAL_DATA_MAX);
        if (got <= 0) {
            ui_console_write("ERR: source read failed\n");
            return;
        }
        i32 n = picowal_db_put((u16)card, rec, data, (u32)got);
        if (n < 0) ui_console_write("ERR: putf failed\n");
        else fb_printf("OK: wrote %u bytes\n", (u32)n);
        return;
    }

    if (ui_streq(argv[1], "get")) {
        static u8 data[PICOWAL_DATA_MAX];
        i32 n = picowal_db_get((u16)card, rec, data, PICOWAL_DATA_MAX);
        if (n < 0) {
            ui_console_write("ERR: get failed\n");
            return;
        }
        fb_printf("db card=%u rec=%u len=%u\n", card, rec, (u32)n);
        for (i32 i = 0; i < n; i++) {
            char c = (char)data[i];
            if (c < 0x20 || c > 0x7E) c = '.';
            uart_putc(c);
            fb_putc(c);
        }
        uart_puts("\n");
        fb_putc('\n');
        return;
    }

    if (ui_streq(argv[1], "getf")) {
        if (argc < 5) {
            ui_console_write("ERR: usage db getf <card> <record> <path>\n");
            return;
        }
        static u8 data[PICOWAL_DATA_MAX];
        i32 n = picowal_db_get((u16)card, rec, data, PICOWAL_DATA_MAX);
        if (n < 0) {
            ui_console_write("ERR: getf read failed\n");
            return;
        }
        char abs[256];
        if (!ui_path_resolve(argv[4], abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        u64 id = walfs_find(abs);
        if (!id) {
            if (!ui_fs_create_path(abs, false)) {
                ui_console_write("ERR: target create failed\n");
                return;
            }
            id = walfs_find(abs);
        }
        if (!id || !walfs_write(id, 0, data, (u32)n)) {
            ui_console_write("ERR: target write failed\n");
            return;
        }
        fb_printf("OK: wrote file bytes=%u\n", (u32)n);
        return;
    }

    ui_console_write("ERR: unknown db op\n");
}

static void ui_dump_sector(u32 lba)
{
    static u8 sector[SD_BLOCK_SIZE] ALIGNED(64);
    if (!sd_read_block(lba, sector)) {
        ui_console_write("ERR: sd_read_block failed\n");
        return;
    }
    static const char hex[] = "0123456789ABCDEF";
    fb_printf("LBA 0x%x\n", lba);
    uart_puts("LBA ");
    uart_hex(lba);
    uart_puts("\n");
    for (u32 off = 0; off < SD_BLOCK_SIZE; off += 16) {
        fb_printf("%x: ", off);
        uart_hex(off);
        uart_puts(": ");
        for (u32 i = 0; i < 16; i++) {
            u8 b = sector[off + i];
            fb_putc(hex[(b >> 4) & 0xF]);
            fb_putc(hex[b & 0xF]);
            fb_putc(' ');
            uart_putc(hex[(b >> 4) & 0xF]);
            uart_putc(hex[b & 0xF]);
            uart_putc(' ');
        }
        fb_puts(" |");
        uart_puts(" |");
        for (u32 i = 0; i < 16; i++) {
            char c = (char)sector[off + i];
            if (c < 0x20 || c > 0x7E) c = '.';
            fb_putc(c);
            uart_putc(c);
        }
        fb_puts("|\n");
        uart_puts("|\n");
    }
}

static void ui_cmd_lsdir(const char *path)
{
    char abs[256];
    if (!path || !*path) path = ".";
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    ui_cmd_fsinspect(abs);
}

static void ui_cmd_mkdir(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage mkdir <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    if (walfs_find(abs)) {
        ui_console_write("OK: already exists\n");
        return;
    }
    if (!ui_fs_create_path(abs, true)) {
        ui_console_write("ERR: mkdir failed\n");
        return;
    }
    ui_console_write("OK: directory created\n");
}

static void ui_cmd_touch(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage touch <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        if (!ui_fs_create_path(abs, false)) {
            ui_console_write("ERR: touch create failed\n");
            return;
        }
        id = walfs_find(abs);
        if (!id) {
            ui_console_write("ERR: touch resolve failed\n");
            return;
        }
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: touch target not file\n");
        return;
    }
    ui_console_write("OK: touched\n");
}

static bool ui_copy_file_internal(const char *src, const char *dst)
{
    char src_abs[256];
    char dst_abs[256];
    if (!ui_path_resolve(src, src_abs, sizeof(src_abs))) return false;
    if (!ui_path_resolve(dst, dst_abs, sizeof(dst_abs))) return false;
    u64 src_id = walfs_find(src_abs);
    if (!src_id) return false;
    struct walfs_inode s;
    if (!walfs_stat(src_id, &s) || (s.flags & WALFS_DIR)) return false;

    u64 dst_id = walfs_find(dst_abs);
    if (!dst_id) {
        if (!ui_fs_create_path(dst_abs, false)) return false;
        dst_id = walfs_find(dst_abs);
        if (!dst_id) return false;
    }

    static u8 buf[WALFS_DATA_MAX] ALIGNED(64);
    u64 off = 0;
    while (off < s.size) {
        u32 chunk = (u32)((s.size - off) > WALFS_DATA_MAX ? WALFS_DATA_MAX : (s.size - off));
        u32 n = walfs_read(src_id, off, buf, chunk);
        if (n == 0 && chunk != 0) break;
        if (!walfs_write(dst_id, off, buf, n)) return false;
        off += n;
        if (n < chunk) break;
    }
    return true;
}

static bool ui_cpdir_recursive(const char *src, const char *dst, u32 depth)
{
    if (depth > 8) return false;
    char src_abs[256];
    char dst_abs[256];
    if (!ui_path_resolve(src, src_abs, sizeof(src_abs))) return false;
    if (!ui_path_resolve(dst, dst_abs, sizeof(dst_abs))) return false;
    u64 src_id = walfs_find(src_abs);
    if (!src_id) return false;
    struct walfs_inode s;
    if (!walfs_stat(src_id, &s) || !(s.flags & WALFS_DIR)) return false;

    if (!walfs_find(dst_abs) && !ui_fs_create_path(dst_abs, true))
        return false;

    struct ui_dir_entry entries[64];
    dir_collect_ctx.out = entries;
    dir_collect_ctx.max = 64;
    dir_collect_ctx.count = 0;
    walfs_readdir(src_id, ui_dir_collect_cb);

    for (u32 i = 0; i < dir_collect_ctx.count; i++) {
        char src_child[256];
        char dst_child[256];
        if (!ui_path_join(src_abs, entries[i].name, src_child, sizeof(src_child))) return false;
        if (!ui_path_join(dst_abs, entries[i].name, dst_child, sizeof(dst_child))) return false;
        struct walfs_inode ino;
        if (!walfs_stat(entries[i].id, &ino)) return false;
        if (ino.flags & WALFS_DIR) {
            if (!ui_cpdir_recursive(src_child, dst_child, depth + 1)) return false;
        } else {
            if (!ui_copy_file_internal(src_child, dst_child)) return false;
        }
    }
    return true;
}

static void ui_cmd_copy(const char *src, const char *dst)
{
    if (!src || !dst || !*src || !*dst) {
        ui_console_write("ERR: usage copy <src> <dst>\n");
        return;
    }
    if (!ui_copy_file_internal(src, dst)) {
        ui_console_write("ERR: copy failed\n");
        return;
    }
    ui_console_write("OK: copied\n");
}

static void ui_cmd_cpdir(const char *src, const char *dst)
{
    if (!src || !dst || !*src || !*dst) {
        ui_console_write("ERR: usage cpdir <src_dir> <dst_dir>\n");
        return;
    }
    if (!ui_cpdir_recursive(src, dst, 0)) {
        ui_console_write("ERR: cpdir failed\n");
        return;
    }
    ui_console_write("OK: directory copied\n");
}

static void ui_cmd_cat(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage cat <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: cat target not file\n");
        return;
    }
    static char buf[129];
    u64 off = 0;
    char last = 0;
    while (off < ino.size) {
        u32 want = (u32)((ino.size - off) > 128 ? 128 : (ino.size - off));
        u32 n = walfs_read(id, off, buf, want);
        if (n == 0) break;
        last = buf[n - 1];
        buf[n] = 0;
        ui_console_write(buf);
        off += n;
    }
    if (ino.size == 0 || last != '\n')
        ui_console_write("\n");
}

static void ui_cmd_stat(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage stat <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    struct walfs_inode ino;
    if (!walfs_stat(id, &ino)) {
        ui_console_write("ERR: stat failed\n");
        return;
    }
    fb_printf("path=%s id=0x%x size=%u flags=0x%x mode=0x%x\n",
              abs, (u32)id, (u32)ino.size, ino.flags, ino.mode);
    uart_puts("path=");
    uart_puts(abs);
    uart_puts(" id=");
    uart_hex((u32)id);
    uart_puts(" size=");
    uart_hex((u32)ino.size);
    uart_puts(" flags=");
    uart_hex(ino.flags);
    uart_puts(" mode=");
    uart_hex(ino.mode);
    uart_puts("\n");
}

static void ui_cmd_rm(const char *path)
{
    char abs[256];
    if (!path || !*path) {
        ui_console_write("ERR: usage rm <path>\n");
        return;
    }
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    if (ui_streq(abs, "/")) {
        ui_console_write("ERR: refusing to remove root\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        ui_console_write("ERR: path not found\n");
        return;
    }
    if (!walfs_delete(id)) {
        ui_console_write("ERR: remove failed\n");
        return;
    }
    ui_console_write("OK: removed\n");
}

static void ui_stream_udp_cb(u32 src_ip, u16 src_port, u16 dst_port UNUSED, const u8 *data, u16 len)
{
    if (!ui_stream_udp.waiting || !data) return;
    if (len > sizeof(ui_stream_udp.data)) len = sizeof(ui_stream_udp.data);
    simd_memcpy(ui_stream_udp.data, data, len);
    ui_stream_udp.len = len;
    ui_stream_udp.src_ip = src_ip;
    ui_stream_udp.src_port = src_port;
    ui_stream_udp.ready = true;
}

static u16 ui_be16_read(const u8 *p)
{
    return (u16)(((u16)p[0] << 8) | (u16)p[1]);
}

static u32 ui_be32_read(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | (u32)p[3];
}

static void ui_be16_write(u8 *p, u16 v)
{
    p[0] = (u8)(v >> 8);
    p[1] = (u8)v;
}

static void ui_be32_write(u8 *p, u32 v)
{
    p[0] = (u8)(v >> 24);
    p[1] = (u8)(v >> 16);
    p[2] = (u8)(v >> 8);
    p[3] = (u8)v;
}

static void ui_db_udp_cb(u32 src_ip, u16 src_port, u16 dst_port, const u8 *data, u16 len)
{
    if (!data || len < 12 || dst_port != PICOWAL_KV_UDP_PORT)
        return;

    u8 op = data[0];
    u8 ver = data[1];
    u16 card = ui_be16_read(&data[4]);
    u32 rec = ui_be32_read(&data[6]);
    u16 val_len = ui_be16_read(&data[10]);

    u8 out[1472];
    u16 out_len = 10;
    out[0] = 1; /* status: error by default */
    out[1] = op;
    ui_be16_write(&out[2], 0);
    ui_be16_write(&out[4], card);
    ui_be32_write(&out[6], rec);

    if (ver != UI_DB_UDP_VER) {
        out[0] = 2; /* bad request/version */
        net_send_udp(src_ip, PICOWAL_KV_UDP_PORT, src_port, out, out_len);
        return;
    }
    if (card > PICOWAL_CARD_MAX || rec > PICOWAL_RECORD_MAX) {
        out[0] = 2;
        net_send_udp(src_ip, PICOWAL_KV_UDP_PORT, src_port, out, out_len);
        return;
    }

    if (op == 1) { /* GET */
        i32 n = picowal_db_get(card, rec, &out[10], (u32)(sizeof(out) - 10));
        if (n >= 0) {
            out[0] = 0;
            ui_be16_write(&out[2], (u16)n);
            out_len = (u16)(10 + (u16)n);
        }
    } else if (op == 2) { /* PUT */
        if ((u32)len < 12U + (u32)val_len || val_len == 0 || val_len > PICOWAL_DATA_MAX) {
            out[0] = 2;
        } else {
            i32 n = picowal_db_put(card, rec, &data[12], val_len);
            if (n >= 0) {
                out[0] = 0;
                ui_be16_write(&out[2], (u16)n);
            }
        }
    } else if (op == 3) { /* DEL */
        if (picowal_db_delete(card, rec))
            out[0] = 0;
    } else if (op == 4) { /* LIST card */
        u32 ids[64];
        u32 n = picowal_db_list(card, ids, 64);
        u32 max_entries = ((u32)sizeof(out) - 10U) / 4U;
        if (n > max_entries) n = max_entries;
        for (u32 i = 0; i < n; i++)
            ui_be32_write(&out[10 + i * 4], ids[i]);
        out[0] = 0;
        ui_be16_write(&out[2], (u16)n);
        out_len = (u16)(10 + n * 4U);
        rec = 0;
        ui_be32_write(&out[6], rec);
    } else {
        out[0] = 2;
    }

    net_send_udp(src_ip, PICOWAL_KV_UDP_PORT, src_port, out, out_len);
}

static bool ui_env_key_eq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static bool ui_env_get(const char *key, const char **val_out)
{
    if (!key || !*key) return false;
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (ui_env[i].used && ui_env_key_eq(ui_env[i].key, key)) {
            if (val_out) *val_out = ui_env[i].val;
            return true;
        }
    }
    return false;
}

static void ui_env_set(const char *key, const char *val, bool persistent)
{
    if (!key || !*key || !val) return;
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (ui_env[i].used && ui_env_key_eq(ui_env[i].key, key)) {
            u32 j = 0;
            for (; key[j] && j + 1 < sizeof(ui_env[i].key); j++) ui_env[i].key[j] = key[j];
            ui_env[i].key[j] = 0;
            j = 0;
            for (; val[j] && j + 1 < sizeof(ui_env[i].val); j++) ui_env[i].val[j] = val[j];
            ui_env[i].val[j] = 0;
            ui_env[i].persistent = persistent;
            return;
        }
    }
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (!ui_env[i].used) {
            u32 j = 0;
            for (; key[j] && j + 1 < sizeof(ui_env[i].key); j++) ui_env[i].key[j] = key[j];
            ui_env[i].key[j] = 0;
            j = 0;
            for (; val[j] && j + 1 < sizeof(ui_env[i].val); j++) ui_env[i].val[j] = val[j];
            ui_env[i].val[j] = 0;
            ui_env[i].used = true;
            ui_env[i].persistent = persistent;
            return;
        }
    }
}

static bool ui_env_save(void)
{
    char out[2048];
    u32 p = 0;
    for (u32 i = 0; i < UI_ENV_MAX; i++) {
        if (!ui_env[i].used || !ui_env[i].persistent) continue;
        const char *k = ui_env[i].key;
        const char *v = ui_env[i].val;
        while (*k && p + 1 < sizeof(out)) out[p++] = *k++;
        if (p + 1 >= sizeof(out)) break;
        out[p++] = '=';
        while (*v && p + 1 < sizeof(out)) out[p++] = *v++;
        if (p + 1 >= sizeof(out)) break;
        out[p++] = '\n';
    }
    return picowal_db_put(0, 1, out, p) >= 0;
}

static bool ui_env_load(void)
{
    if (ui_env_loaded) return true;
    ui_env_loaded = true;
    char buf[2048];
    i32 n_i = picowal_db_get(0, 1, buf, sizeof(buf) - 1);
    if (n_i < 0) return true; /* no persisted config yet */
    u32 n = (u32)n_i;
    buf[n] = 0;
    u32 i = 0;
    while (i < n) {
        char key[32]; char val[128];
        u32 kp = 0, vp = 0;
        while (i < n && buf[i] != '=' && buf[i] != '\n' && kp + 1 < sizeof(key)) key[kp++] = buf[i++];
        key[kp] = 0;
        if (i < n && buf[i] == '=') i++;
        while (i < n && buf[i] != '\n' && vp + 1 < sizeof(val)) val[vp++] = buf[i++];
        val[vp] = 0;
        while (i < n && buf[i] != '\n') i++;
        if (i < n && buf[i] == '\n') i++;
        if (key[0]) ui_env_set(key, val, true);
    }
    return true;
}

static u32 ui_read_tty_line(char *out, u32 out_max, const char *prompt)
{
    if (!out || out_max < 2) return 0;
    if (prompt) ui_console_write(prompt);
    u32 n = 0;
    while (1) {
        i32 c = usb_kbd_try_getc();
        if (c < 0) c = uart_try_getc();
        if (c < 0) {
            net_poll();
            timer_delay_ms(1);
            continue;
        }
        if (c == '\r' || c == '\n') {
            out[n] = 0;
            ui_console_write("\n");
            return n;
        }
        if (c == '\b' || c == 127) {
            if (n > 0) {
                n--;
                ui_console_write("\b \b");
            }
            continue;
        }
        if (c < 0x20 || c > 0x7E) continue;
        if (n + 1 >= out_max) continue;
        out[n++] = (char)c;
        char e[2] = {(char)c, 0};
        ui_console_write(e);
    }
}

static u32 ui_edit_line_len(const char *s)
{
    u32 n = 0;
    while (s[n]) n++;
    return n;
}

static bool ui_edit_insert_line(char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX], u32 *line_count, u32 at)
{
    if (!line_count || *line_count >= UI_EDIT_MAX_LINES || at > *line_count) return false;
    for (u32 i = *line_count; i > at; i--) {
        simd_memcpy(lines[i], lines[i - 1], UI_EDIT_LINE_MAX);
    }
    lines[at][0] = 0;
    (*line_count)++;
    return true;
}

static bool ui_edit_delete_line(char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX], u32 *line_count, u32 at)
{
    if (!line_count || *line_count == 0 || at >= *line_count) return false;
    if (*line_count == 1) {
        lines[0][0] = 0;
        return true;
    }
    for (u32 i = at; i + 1 < *line_count; i++) {
        simd_memcpy(lines[i], lines[i + 1], UI_EDIT_LINE_MAX);
    }
    (*line_count)--;
    lines[*line_count][0] = 0;
    return true;
}

static bool ui_edit_save_file(const char *abs_path, char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX], u32 line_count)
{
    if (!abs_path || !*abs_path) return false;
    u64 id = walfs_find(abs_path);
    struct walfs_inode ino;
    if (id) {
        if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) return false;
        if (!walfs_delete(id)) return false;
        id = 0;
    }
    if (!id) {
        if (!ui_fs_create_path(abs_path, false)) return false;
        id = walfs_find(abs_path);
        if (!id) return false;
    }

    char out[UI_EDIT_MAX_LINES * UI_EDIT_LINE_MAX];
    u32 p = 0;
    if (line_count == 0) line_count = 1;
    for (u32 i = 0; i < line_count; i++) {
        const char *ln = lines[i];
        u32 len = ui_edit_line_len(ln);
        if (p + len > sizeof(out)) return false;
        for (u32 j = 0; j < len; j++) out[p++] = ln[j];
        if (i + 1 < line_count) {
            if (p + 1 > sizeof(out)) return false;
            out[p++] = '\n';
        }
    }
    if (p == 0) return true;
    return walfs_write(id, 0, out, p);
}

static void ui_edit_render(const char *abs_path,
                           char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX],
                           u32 line_count, u32 cur_line, u32 cur_col, u32 view_top,
                           bool insert_mode, bool dirty, const char *status)
{
    fb_clear(0x00000000);
    fb_set_color(UI_SHELL_TEXT_COLOR, 0x00000000);
    fb_printf("PIOS edit.pix (kernel TUI)  %s\n", abs_path ? abs_path : "(null)");
    fb_printf("Ctrl+S save | Ctrl+Q exit | Ctrl+C copy line | Ctrl+X cut line | Ctrl+V paste line\n");
    fb_printf("Arrows move | Enter split line | Backspace/Delete erase | Insert toggles %s | %s%s\n",
              insert_mode ? "insert" : "overwrite", dirty ? "*" : "",
              status ? status : "");
    fb_printf("--------------------------------------------------------------------------------\n");

    fb_set_color(0x00FFFFFF, 0x00000000);
    u32 rows = 20;
    for (u32 r = 0; r < rows; r++) {
        u32 li = view_top + r;
        if (li >= line_count) {
            fb_printf("~\n");
            continue;
        }
        bool sel = (li == cur_line);
        fb_set_color(sel ? 0x00FF9900 : 0x00FFFFFF, 0x00000000);
        fb_printf("%c%u ", sel ? '>' : ' ', li + 1);
        char out[UI_EDIT_LINE_MAX + 2];
        u32 op = 0;
        u32 len = ui_edit_line_len(lines[li]);
        for (u32 i = 0; i < len && op + 1 < sizeof(out); i++) {
            if (sel && i == cur_col) out[op++] = '|';
            out[op++] = lines[li][i];
        }
        if (sel && cur_col >= len && op + 1 < sizeof(out)) out[op++] = '|';
        out[op] = 0;
        fb_printf("%s\n", out);
    }
}

static void ui_cmd_edit(const char *path)
{
    if (!path || !*path) {
        ui_console_write("ERR: usage edit <path>\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }

    char lines[UI_EDIT_MAX_LINES][UI_EDIT_LINE_MAX];
    simd_zero(lines, sizeof(lines));
    u32 line_count = 1;
    u32 cur_line = 0, cur_col = 0, view_top = 0;
    bool insert_mode = true;
    bool dirty = false;
    bool running = true;
    bool has_clip = false;
    bool redraw = true;
    char clip[UI_EDIT_LINE_MAX];
    char status[64];
    status[0] = 0;

    u64 id = walfs_find(abs);
    if (id) {
        struct walfs_inode ino;
        if (!walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
            ui_console_write("ERR: edit target not file\n");
            return;
        }
        u32 max_read = (u32)((ino.size > (sizeof(lines) - 1)) ? (sizeof(lines) - 1) : ino.size);
        char buf[UI_EDIT_MAX_LINES * UI_EDIT_LINE_MAX];
        u32 n = walfs_read(id, 0, buf, max_read);
        buf[n] = 0;
        line_count = 0;
        u32 li = 0, cj = 0;
        for (u32 i = 0; i < n && li < UI_EDIT_MAX_LINES; i++) {
            char c = buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                lines[li][cj] = 0;
                li++;
                cj = 0;
                continue;
            }
            if (c < 0x20 || c > 0x7E) c = '.';
            if (cj + 1 < UI_EDIT_LINE_MAX) lines[li][cj++] = c;
        }
        if (li < UI_EDIT_MAX_LINES) {
            lines[li][cj] = 0;
            line_count = li + 1;
        }
        if (line_count == 0) line_count = 1;
    }

    while (running) {
        if (cur_line < view_top) view_top = cur_line;
        if (cur_line >= view_top + 20) view_top = cur_line - 19;

        if (redraw) {
            ui_edit_render(abs, lines, line_count, cur_line, cur_col, view_top, insert_mode, dirty, status);
            redraw = false;
        }

        i32 key;
        while ((key = usb_kbd_try_getkey()) >= 0) {
            u32 len = ui_edit_line_len(lines[cur_line]);
            if (key == USB_KBD_KEY_LEFT) {
                if (cur_col > 0) cur_col--;
            } else if (key == USB_KBD_KEY_RIGHT) {
                if (cur_col < len) cur_col++;
            } else if (key == USB_KBD_KEY_UP) {
                if (cur_line > 0) cur_line--;
                len = ui_edit_line_len(lines[cur_line]);
                if (cur_col > len) cur_col = len;
            } else if (key == USB_KBD_KEY_DOWN) {
                if (cur_line + 1 < line_count) cur_line++;
                len = ui_edit_line_len(lines[cur_line]);
                if (cur_col > len) cur_col = len;
            } else if (key == USB_KBD_KEY_HOME) {
                cur_col = 0;
            } else if (key == USB_KBD_KEY_END) {
                cur_col = ui_edit_line_len(lines[cur_line]);
            } else if (key == USB_KBD_KEY_INSERT) {
                insert_mode = !insert_mode;
            } else if (key == USB_KBD_KEY_DELETE) {
                if (cur_col < len) {
                    for (u32 i = cur_col; i < len; i++) lines[cur_line][i] = lines[cur_line][i + 1];
                    dirty = true;
                } else if (cur_line + 1 < line_count) {
                    u32 nlen = ui_edit_line_len(lines[cur_line + 1]);
                    if (len + nlen + 1 < UI_EDIT_LINE_MAX) {
                        for (u32 i = 0; i < nlen; i++) lines[cur_line][len + i] = lines[cur_line + 1][i];
                        lines[cur_line][len + nlen] = 0;
                        ui_edit_delete_line(lines, &line_count, cur_line + 1);
                        dirty = true;
                    }
                }
            } else if (key == USB_KBD_KEY_F3) {
                running = false;
            }
            status[0] = 0;
            redraw = true;
        }

        i32 c;
        while ((c = usb_kbd_try_getc()) >= 0) {
            u32 len = ui_edit_line_len(lines[cur_line]);
            if (c == 19) { /* Ctrl+S */
                if (ui_edit_save_file(abs, lines, line_count)) {
                    const char *ok = "saved";
                    for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                    dirty = false;
                } else {
                    const char *er = "save failed";
                    for (u32 i = 0; er[i] && i + 1 < sizeof(status); i++) status[i] = er[i], status[i + 1] = 0;
                }
                redraw = true;
                continue;
            }
            if (c == 17 || c == 27) { /* Ctrl+Q / Esc */
                running = false;
                break;
            }
            if (c == 3) { /* Ctrl+C copy line */
                simd_memcpy(clip, lines[cur_line], UI_EDIT_LINE_MAX);
                has_clip = true;
                const char *ok = "copied line";
                for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                redraw = true;
                continue;
            }
            if (c == 24) { /* Ctrl+X cut line */
                simd_memcpy(clip, lines[cur_line], UI_EDIT_LINE_MAX);
                has_clip = true;
                ui_edit_delete_line(lines, &line_count, cur_line);
                if (cur_line >= line_count) cur_line = line_count - 1;
                cur_col = 0;
                dirty = true;
                const char *ok = "cut line";
                for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                redraw = true;
                continue;
            }
            if (c == 22) { /* Ctrl+V paste line below */
                if (has_clip && ui_edit_insert_line(lines, &line_count, cur_line + 1)) {
                    simd_memcpy(lines[cur_line + 1], clip, UI_EDIT_LINE_MAX);
                    cur_line++;
                    cur_col = 0;
                    dirty = true;
                    const char *ok = "pasted line";
                    for (u32 i = 0; ok[i] && i + 1 < sizeof(status); i++) status[i] = ok[i], status[i + 1] = 0;
                } else {
                    const char *er = "paste failed";
                    for (u32 i = 0; er[i] && i + 1 < sizeof(status); i++) status[i] = er[i], status[i + 1] = 0;
                }
                redraw = true;
                continue;
            }
            if (c == '\r' || c == '\n') {
                if (line_count < UI_EDIT_MAX_LINES && ui_edit_insert_line(lines, &line_count, cur_line + 1)) {
                    if (cur_col < len) {
                        u32 tail = len - cur_col;
                        if (tail + 1 < UI_EDIT_LINE_MAX) {
                            for (u32 i = 0; i < tail; i++) lines[cur_line + 1][i] = lines[cur_line][cur_col + i];
                            lines[cur_line + 1][tail] = 0;
                            lines[cur_line][cur_col] = 0;
                        }
                    }
                    cur_line++;
                    cur_col = 0;
                    dirty = true;
                }
                redraw = true;
                continue;
            }
            if (c == '\b' || c == 127) {
                if (cur_col > 0) {
                    for (u32 i = cur_col - 1; i < len; i++) lines[cur_line][i] = lines[cur_line][i + 1];
                    cur_col--;
                    dirty = true;
                } else if (cur_line > 0) {
                    u32 plen = ui_edit_line_len(lines[cur_line - 1]);
                    if (plen + len + 1 < UI_EDIT_LINE_MAX) {
                        for (u32 i = 0; i < len; i++) lines[cur_line - 1][plen + i] = lines[cur_line][i];
                        lines[cur_line - 1][plen + len] = 0;
                        ui_edit_delete_line(lines, &line_count, cur_line);
                        cur_line--;
                        cur_col = plen;
                        dirty = true;
                    }
                }
                redraw = true;
                continue;
            }
            if (c < 0x20 || c > 0x7E) continue;
            if (insert_mode) {
                if (len + 1 >= UI_EDIT_LINE_MAX) continue;
                for (u32 i = len + 1; i > cur_col; i--) lines[cur_line][i] = lines[cur_line][i - 1];
                lines[cur_line][cur_col] = (char)c;
                cur_col++;
                dirty = true;
            } else {
                if (cur_col < len) {
                    lines[cur_line][cur_col++] = (char)c;
                    dirty = true;
                } else if (len + 1 < UI_EDIT_LINE_MAX) {
                    lines[cur_line][len] = (char)c;
                    lines[cur_line][len + 1] = 0;
                    cur_col++;
                    dirty = true;
                }
            }
            redraw = true;
        }

        while ((c = uart_try_getc()) >= 0) {
            if (c == 19) {
                if (ui_edit_save_file(abs, lines, line_count)) dirty = false;
                redraw = true;
            } else if (c == 17 || c == 27) {
                running = false;
            } else if (c == '\r' || c == '\n') {
                if (line_count < UI_EDIT_MAX_LINES && ui_edit_insert_line(lines, &line_count, cur_line + 1)) {
                    u32 len = ui_edit_line_len(lines[cur_line]);
                    if (cur_col < len) {
                        u32 tail = len - cur_col;
                        if (tail + 1 < UI_EDIT_LINE_MAX) {
                            for (u32 i = 0; i < tail; i++) lines[cur_line + 1][i] = lines[cur_line][cur_col + i];
                            lines[cur_line + 1][tail] = 0;
                            lines[cur_line][cur_col] = 0;
                        }
                    }
                    cur_line++;
                    cur_col = 0;
                    dirty = true;
                }
                redraw = true;
            } else if (c == '\b' || c == 127) {
                u32 len = ui_edit_line_len(lines[cur_line]);
                if (cur_col > 0) {
                    for (u32 i = cur_col - 1; i < len; i++) lines[cur_line][i] = lines[cur_line][i + 1];
                    cur_col--;
                    dirty = true;
                    redraw = true;
                }
            } else if (c >= 0x20 && c <= 0x7E) {
                u32 len = ui_edit_line_len(lines[cur_line]);
                if (len + 1 < UI_EDIT_LINE_MAX) {
                    for (u32 i = len + 1; i > cur_col; i--) lines[cur_line][i] = lines[cur_line][i - 1];
                    lines[cur_line][cur_col++] = (char)c;
                    dirty = true;
                    redraw = true;
                }
            }
        }
        net_poll();
        workq_drain(2);
        timer_delay_ms(1);
    }

    fb_clear(0x00000000);
    fb_set_color(UI_SHELL_TEXT_COLOR, 0x00000000);
    ui_console_write("PIOS F3 Console (serial + HDMI)\n");
    ui_console_write("Type 'help' for commands.\n");
    ui_console_prompt();
}

static void ui_stream_emit(const u8 *data, u32 len, bool to_file, const char *path)
{
    if (!data) return;
    if (!to_file) {
        for (u32 i = 0; i < len; i++) {
            char c = (char)data[i];
            if (c < 0x20 && c != '\n' && c != '\r' && c != '\t') c = '.';
            char o[2] = {c, 0};
            ui_console_write(o);
        }
        if (len == 0 || data[len - 1] != '\n') ui_console_write("\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad output path\n");
        return;
    }
    u64 id = walfs_find(abs);
    if (!id) {
        if (!ui_fs_create_path(abs, false)) {
            ui_console_write("ERR: output create failed\n");
            return;
        }
        id = walfs_find(abs);
    }
    if (!id || !walfs_write(id, 0, data, len))
        ui_console_write("ERR: output write failed\n");
    else
        ui_console_write("OK: output saved\n");
}

static void ui_cmd_stream(u32 argc, char **argv)
{
    if (argc < 8 || !ui_streq(argv[4], "from")) {
        ui_console_write("ERR: stream <tcp|udp> <ip> <port> from <file|text|tty> <arg?> to <console|file> [path] [timeout_ms]\n");
        return;
    }
    char proto = argv[1][0];
    u32 dst_ip = 0, port = 0;
    if (!ui_parse_ip4(argv[2], &dst_ip) || !ui_parse_u32(argv[3], &port) || port > 65535) {
        ui_console_write("ERR: invalid ip/port\n");
        return;
    }
    u8 in_buf[UI_STREAM_IN_MAX];
    u32 in_len = 0;
    u32 i = 5;
    if (ui_streq(argv[i], "file")) {
        if (i + 1 >= argc) { ui_console_write("ERR: stream missing file path\n"); return; }
        char abs[256];
        if (!ui_path_resolve(argv[i + 1], abs, sizeof(abs))) { ui_console_write("ERR: bad file path\n"); return; }
        u64 id = walfs_find(abs);
        struct walfs_inode ino;
        if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) { ui_console_write("ERR: stream input file not found\n"); return; }
        in_len = (u32)((ino.size > UI_STREAM_IN_MAX) ? UI_STREAM_IN_MAX : ino.size);
        in_len = walfs_read(id, 0, in_buf, in_len);
        i += 2;
    } else if (ui_streq(argv[i], "text")) {
        if (i + 1 >= argc) { ui_console_write("ERR: stream missing text\n"); return; }
        const char *s = argv[i + 1];
        while (*s && in_len + 1 < sizeof(in_buf)) in_buf[in_len++] = (u8)*s++;
        i += 2;
    } else if (ui_streq(argv[i], "tty")) {
        char line[UI_STREAM_IN_MAX];
        in_len = ui_read_tty_line(line, sizeof(line), "tty> ");
        for (u32 j = 0; j < in_len; j++) in_buf[j] = (u8)line[j];
        i += 1;
    } else {
        ui_console_write("ERR: from must be file|text|tty\n");
        return;
    }
    if (i >= argc || !ui_streq(argv[i], "to") || i + 1 >= argc) {
        ui_console_write("ERR: stream missing to clause\n");
        return;
    }
    bool out_to_file = false;
    const char *out_path = NULL;
    if (ui_streq(argv[i + 1], "console")) {
        i += 2;
    } else if (ui_streq(argv[i + 1], "file")) {
        if (i + 2 >= argc) { ui_console_write("ERR: stream missing output file path\n"); return; }
        out_to_file = true;
        out_path = argv[i + 2];
        i += 3;
    } else {
        ui_console_write("ERR: to must be console|file\n");
        return;
    }
    u32 timeout_ms = 3000;
    if (i < argc && !ui_parse_u32(argv[i], &timeout_ms)) {
        ui_console_write("ERR: invalid timeout\n");
        return;
    }

    if (proto == 'u') {
        udp_recv_cb prev = net_swap_udp_callback(ui_stream_udp_cb);
        simd_zero(&ui_stream_udp, sizeof(ui_stream_udp));
        ui_stream_udp.waiting = true;
        bool sent = net_send_udp(dst_ip, 40000, (u16)port, in_buf, (u16)in_len);
        if (!sent) {
            net_set_udp_callback(prev);
            ui_console_write("ERR: udp send failed\n");
            return;
        }
        for (u32 t = 0; t < timeout_ms; t++) {
            net_poll();
            if (ui_stream_udp.ready) break;
            timer_delay_ms(1);
        }
        ui_stream_udp.waiting = false;
        net_set_udp_callback(prev);
        if (!ui_stream_udp.ready) {
            ui_console_write("ERR: udp recv timeout\n");
            return;
        }
        ui_stream_emit(ui_stream_udp.data, ui_stream_udp.len, out_to_file, out_path);
        return;
    }

    if (proto != 't') {
        ui_console_write("ERR: proto must be tcp|udp\n");
        return;
    }
    tcp_conn_t c = tcp_connect(dst_ip, (u16)port);
    if (c < 0) {
        ui_console_write("ERR: tcp connect failed\n");
        return;
    }
    bool up = false;
    for (u32 t = 0; t < timeout_ms; t++) {
        net_poll();
        u32 st = tcp_state(c);
        if (st == TCP_ESTABLISHED) { up = true; break; }
        if (st == TCP_CLOSED) break;
        timer_delay_ms(1);
    }
    if (!up) {
        tcp_close(c);
        ui_console_write("ERR: tcp connect timeout\n");
        return;
    }
    u32 off = 0;
    while (off < in_len) {
        u32 n = tcp_write(c, in_buf + off, in_len - off);
        if (n == 0) { net_poll(); timer_delay_ms(1); continue; }
        off += n;
        net_poll();
    }
    u8 out[UI_STREAM_OUT_MAX];
    u32 out_len = 0;
    u32 idle = 0;
    while (idle < 200 && out_len < sizeof(out)) {
        net_poll();
        u32 avail = tcp_readable(c);
        if (avail > 0) {
            u32 want = (u32)((sizeof(out) - out_len) < avail ? (sizeof(out) - out_len) : avail);
            u32 n = tcp_read(c, out + out_len, want);
            out_len += n;
            idle = 0;
        } else {
            idle++;
            timer_delay_ms(1);
        }
    }
    tcp_close(c);
    ui_stream_emit(out, out_len, out_to_file, out_path);
}

static void ui_cmd_mv(const char *src, const char *dst)
{
    if (!src || !dst || !*src || !*dst) {
        ui_console_write("ERR: usage mv <src> <dst>\n");
        return;
    }
    if (!ui_copy_file_internal(src, dst)) {
        ui_console_write("ERR: mv copy failed\n");
        return;
    }
    ui_cmd_rm(src);
}

static void ui_cmd_hexdump(const char *path, u32 max_bytes)
{
    if (!path || !*path) {
        ui_console_write("ERR: usage hexdump <path> [max_bytes]\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(path, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: hexdump file not found\n");
        return;
    }
    if (max_bytes == 0 || max_bytes > 1024) max_bytes = 256;
    u32 n = (u32)((ino.size < max_bytes) ? ino.size : max_bytes);
    u8 buf[1024];
    n = walfs_read(id, 0, buf, n);
    static const char hx[] = "0123456789ABCDEF";
    for (u32 off = 0; off < n; off += 16) {
        fb_printf("%x: ", off);
        uart_hex(off); uart_puts(": ");
        for (u32 i = 0; i < 16; i++) {
            if (off + i < n) {
                u8 b = buf[off + i];
                char o[4] = {hx[(b>>4)&0xF], hx[b&0xF], ' ', 0};
                ui_console_write(o);
            } else ui_console_write("   ");
        }
        ui_console_write("\n");
    }
}

static void ui_cmd_find(const char *base, const char *needle)
{
    char abs[256];
    if (!base || !*base || !needle || !*needle) {
        ui_console_write("ERR: usage find <dir> <needle>\n");
        return;
    }
    if (!ui_path_resolve(base, abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: find dir not found\n");
        return;
    }
    struct ui_dir_entry entries[64];
    dir_collect_ctx.out = entries;
    dir_collect_ctx.max = 64;
    dir_collect_ctx.count = 0;
    walfs_readdir(id, ui_dir_collect_cb);
    for (u32 i = 0; i < dir_collect_ctx.count; i++) {
        bool hit = false;
        const char *n = entries[i].name;
        for (u32 p = 0; n[p]; p++) {
            u32 j = 0;
            while (needle[j] && n[p + j] == needle[j]) j++;
            if (!needle[j]) { hit = true; break; }
        }
        if (!hit) continue;
        char full[256];
        if (!ui_path_join(abs, entries[i].name, full, sizeof(full))) continue;
        ui_console_write(full);
        ui_console_write("\n");
    }
}

static void ui_cmd_df(void)
{
    const sd_card_t *card = sd_get_card_info();
    fb_printf("df: cap_bytes=%X (walfs usage telemetry pending)\n", card->capacity);
    uart_puts("df cap=");
    uart_hex(card->capacity);
    uart_puts("\n");
}

static void ui_cmd_mount(u32 argc, char **argv)
{
    if (argc >= 1 && ui_streq(argv[0], "mount")) {
        ui_console_write("OK: walfs always mounted at /\n");
        return;
    }
    if (argc >= 1 && ui_streq(argv[0], "umount")) {
        ui_console_write("ERR: umount unsupported on running kernel rootfs\n");
        return;
    }
    ui_console_write("ERR: usage mount|umount\n");
}

static void ui_cmd_env(u32 argc, char **argv)
{
    ui_env_load();
    if (argc < 2 || ui_streq(argv[1], "list")) {
        for (u32 i = 0; i < UI_ENV_MAX; i++) {
            if (!ui_env[i].used) continue;
            ui_console_write(ui_env[i].persistent ? "P " : "T ");
            ui_console_write(ui_env[i].key);
            ui_console_write("=");
            ui_console_write(ui_env[i].val);
            ui_console_write("\n");
        }
        return;
    }
    if ((ui_streq(argv[1], "set") || ui_streq(argv[1], "pset")) && argc >= 4) {
        bool p = ui_streq(argv[1], "pset");
        ui_env_set(argv[2], argv[3], p);
        if (p && !ui_env_save()) ui_console_write("ERR: env save failed\n");
        else ui_console_write("OK: env set\n");
        return;
    }
    if (ui_streq(argv[1], "get") && argc >= 3) {
        const char *v = NULL;
        if (ui_env_get(argv[2], &v)) {
            ui_console_write(v);
            ui_console_write("\n");
        } else ui_console_write("ERR: var not found\n");
        return;
    }
    if (ui_streq(argv[1], "unset") && argc >= 3) {
        for (u32 i = 0; i < UI_ENV_MAX; i++) {
            if (ui_env[i].used && ui_env_key_eq(ui_env[i].key, argv[2])) ui_env[i].used = false;
        }
        ui_env_save();
        ui_console_write("OK: env unset\n");
        return;
    }
    if (ui_streq(argv[1], "save")) {
        ui_console_write(ui_env_save() ? "OK: env saved\n" : "ERR: env save failed\n");
        return;
    }
    if (ui_streq(argv[1], "load")) {
        ui_env_loaded = false;
        ui_console_write(ui_env_load() ? "OK: env loaded\n" : "ERR: env load failed\n");
        return;
    }
    ui_console_write("ERR: env [list|get|set|pset|unset|save|load]\n");
}

static void ui_exec_subcommand(u32 start, u32 argc, char **argv, const char *placeholder)
{
    char cmd[256];
    u32 p = 0;
    for (u32 i = start; i < argc; i++) {
        const char *s = argv[i];
        if (placeholder && ui_streq(s, "{}")) s = placeholder;
        while (*s && p + 1 < sizeof(cmd)) cmd[p++] = *s++;
        if (i + 1 < argc && p + 1 < sizeof(cmd)) cmd[p++] = ' ';
    }
    cmd[p] = 0;
    if (p == 0) return;
    if (ui_script_depth > 8) {
        ui_console_write("ERR: script depth exceeded\n");
        return;
    }
    ui_script_depth++;
    ui_console_exec(cmd);
    ui_script_depth--;
}

static bool ui_cmp_values(const char *a, const char *op, const char *b)
{
    u32 av = 0, bv = 0;
    bool na = ui_parse_u32(a, &av);
    bool nb = ui_parse_u32(b, &bv);
    if (ui_streq(op, "==")) return ui_streq(a, b);
    if (ui_streq(op, "!=")) return !ui_streq(a, b);
    if (!na || !nb) return false;
    if (ui_streq(op, ">")) return av > bv;
    if (ui_streq(op, "<")) return av < bv;
    if (ui_streq(op, ">=")) return av >= bv;
    if (ui_streq(op, "<=")) return av <= bv;
    return false;
}

static void ui_cmd_if(u32 argc, char **argv)
{
    if (argc < 6) {
        ui_console_write("ERR: usage if <a> <op> <b> <cmd...>\n");
        return;
    }
    if (ui_cmp_values(argv[1], argv[2], argv[3]))
        ui_exec_subcommand(4, argc, argv, NULL);
}

static void ui_cmd_for(u32 argc, char **argv)
{
    if (argc < 5) {
        ui_console_write("ERR: usage for <start> <end> <cmd... with {}>\n");
        return;
    }
    u32 s = 0, e = 0;
    if (!ui_parse_u32(argv[1], &s) || !ui_parse_u32(argv[2], &e)) {
        ui_console_write("ERR: invalid range\n");
        return;
    }
    char repl[16];
    for (u32 i = s; i <= e; i++) {
        u32 n = 0, v = i;
        char rev[16];
        do { rev[n++] = (char)('0' + (v % 10)); v /= 10; } while (v && n < sizeof(rev));
        for (u32 j = 0; j < n; j++) repl[j] = rev[n - 1 - j];
        repl[n] = 0;
        ui_exec_subcommand(3, argc, argv, repl);
    }
}

static void ui_cmd_foreach(u32 argc, char **argv)
{
    if (argc < 5 || !ui_streq(argv[1], "file")) {
        ui_console_write("ERR: usage foreach file <path> <cmd... with {}>\n");
        return;
    }
    char abs[256];
    if (!ui_path_resolve(argv[2], abs, sizeof(abs))) {
        ui_console_write("ERR: bad path\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: foreach file not found\n");
        return;
    }
    char buf[1024];
    u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
    n = walfs_read(id, 0, buf, n);
    buf[n] = 0;
    char line[128];
    u32 lp = 0;
    for (u32 i = 0; i <= n; i++) {
        char c = (i < n) ? buf[i] : '\n';
        if (c == '\n' || c == '\r') {
            line[lp] = 0;
            if (lp) ui_exec_subcommand(3, argc, argv, line);
            lp = 0;
        } else if (lp + 1 < sizeof(line)) {
            line[lp++] = c;
        }
    }
}

static void ui_cmd_source(const char *path)
{
    char abs[256];
    if (!path || !*path || !ui_resolve_pis_path(path, abs, sizeof(abs))) {
        ui_console_write("ERR: usage source <path>\n");
        return;
    }
    u64 id = walfs_find(abs);
    struct walfs_inode ino;
    if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
        ui_console_write("ERR: source file not found\n");
        return;
    }
    char buf[2048];
    u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
    n = walfs_read(id, 0, buf, n);
    buf[n] = 0;
    char line[256];
    u32 lp = 0;
    for (u32 i = 0; i <= n; i++) {
        char c = (i < n) ? buf[i] : '\n';
        if (c == '\n' || c == '\r') {
            line[lp] = 0;
            if (lp && line[0] != '#') ui_console_exec(line);
            lp = 0;
        } else if (lp + 1 < sizeof(line)) {
            line[lp++] = c;
        }
    }
}

static bool ui_cap_manifest_validate_buf(const char *buf, u32 n, char *err, u32 err_max)
{
    if (!buf || n == 0) {
        if (err && err_max) { err[0] = 'e'; err[1] = 'm'; err[2] = 'p'; err[3] = 't'; err[4] = 'y'; err[5] = 0; }
        return false;
    }
    u32 i = 0;
    while (i < n) {
        u32 ls = i;
        while (i < n && buf[i] != '\n' && buf[i] != '\r') i++;
        u32 le = i;
        while (i < n && (buf[i] == '\n' || buf[i] == '\r')) i++;
        while (ls < le && (buf[ls] == ' ' || buf[ls] == '\t')) ls++;
        while (le > ls && (buf[le - 1] == ' ' || buf[le - 1] == '\t')) le--;
        if (le <= ls || buf[ls] == '#')
            continue;
        u32 eq = ls;
        while (eq < le && buf[eq] != '=') eq++;
        if (eq >= le) {
            if (err && err_max) { err[0]='b'; err[1]='a'; err[2]='d'; err[3]='='; err[4]=0; }
            return false;
        }
        u32 klen = eq - ls;
        if (klen == 0 || klen > 16) {
            if (err && err_max) { err[0]='k'; err[1]='e'; err[2]='y'; err[3]=0; }
            return false;
        }
        bool key_ok = false;
        if (klen == 7 && ui_strneq(&buf[ls], "capsule", 7)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "spawn", 5)) key_ok = true;
        else if (klen == 4 && ui_strneq(&buf[ls], "wait", 4)) key_ok = true;
        else if (klen == 6 && ui_strneq(&buf[ls], "nprocs", 6)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "group", 5)) key_ok = true;
        else if (klen == 3 && ui_strneq(&buf[ls], "vfs", 3)) key_ok = true;
        else if (klen == 2 && ui_strneq(&buf[ls], "fs", 2)) key_ok = true;
        else if (klen == 3 && ui_strneq(&buf[ls], "ipc", 3)) key_ok = true;
        else if (klen == 4 && ui_strneq(&buf[ls], "pipe", 4)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "cards", 5)) key_ok = true;
        else if (klen == 5 && ui_strneq(&buf[ls], "ports", 5)) key_ok = true;
        else if (klen == 7 && ui_strneq(&buf[ls], "mem_kib", 7)) key_ok = true;
        else if (klen == 6 && ui_strneq(&buf[ls], "cpu_ms", 6)) key_ok = true;
        else if (klen == 8 && ui_strneq(&buf[ls], "ipc_objs", 8)) key_ok = true;
        else if (klen == 12 && ui_strneq(&buf[ls], "fs_write_kib", 12)) key_ok = true;
        if (!key_ok) {
            if (err && err_max) { err[0]='u'; err[1]='n'; err[2]='k'; err[3]=0; }
            return false;
        }
        u32 vlen = le - (eq + 1);
        if (vlen == 0) {
            if (err && err_max) { err[0]='v'; err[1]='a'; err[2]='l'; err[3]=0; }
            return false;
        }
    }
    if (err && err_max) err[0] = 0;
    return true;
}

static void ui_cmd_capsule(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("capsule ls | capsule status [id] | capsule check <path.cap>\n");
        return;
    }
    if (ui_streq(argv[1], "ls")) {
        struct proc_capsule_ui_entry e[UI_SNAPSHOT_MAX];
        u32 n = proc_capsule_snapshot(e, UI_SNAPSHOT_MAX);
        for (u32 i = 0; i < n; i++) {
            fb_printf("pid=0x%x core=%u cap=%u hash=0x%x grp=%s vfs=%s\n",
                      e[i].pid, e[i].affinity_core, e[i].capsule_id, e[i].capsule_hash,
                      e[i].group[0] ? e[i].group : "-", e[i].vfs_root[0] ? e[i].vfs_root : "-");
        }
        uart_puts("capsule ls done\n");
        return;
    }
    if (ui_streq(argv[1], "status")) {
        u32 id = 0;
        if (argc >= 3 && !ui_parse_u32(argv[2], &id)) {
            ui_console_write("ERR: invalid capsule id\n");
            return;
        }
        u64 st = 0, faults = 0;
        if (el2_hvc_call(EL2_HVC_STAGE2_STATUS, id, 0, 0, 0, &st) != 0) {
            ui_console_write("ERR: stage2 status unavailable\n");
            return;
        }
        (void)el2_hvc_call(EL2_HVC_STAGE2_FAULTS, 0, 0, 0, 0, &faults);
        fb_printf("capsule=%u st=0x%x faults=0x%x\n", id, st, faults);
        return;
    }
    if (ui_streq(argv[1], "check")) {
        if (argc < 3) {
            ui_console_write("ERR: usage capsule check <path.cap>\n");
            return;
        }
        char abs[256];
        if (!ui_path_resolve(argv[2], abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        u64 id = walfs_find(abs);
        struct walfs_inode ino;
        if (!id || !walfs_stat(id, &ino) || (ino.flags & WALFS_DIR)) {
            ui_console_write("ERR: manifest not found\n");
            return;
        }
        char buf[1024];
        u32 n = (u32)((ino.size > sizeof(buf) - 1) ? (sizeof(buf) - 1) : ino.size);
        n = walfs_read(id, 0, buf, n);
        buf[n] = 0;
        char err[16];
        if (!ui_cap_manifest_validate_buf(buf, n, err, sizeof(err))) {
            ui_console_write("ERR: invalid manifest ");
            ui_console_write(err);
            ui_console_write("\n");
            return;
        }
        ui_console_write("OK: manifest valid\n");
        return;
    }
    ui_console_write("ERR: unknown capsule subcommand\n");
}

static bool ui_batch_pid_active(i32 pid)
{
    if (pid <= 0) return false;
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    for (u32 i = 0; i < n; i++) {
        if ((i32)snap[i].pid == pid) return true;
    }
    return false;
}

static u32 ui_batch_running_count(void)
{
    u32 n = 0;
    for (u32 i = 0; i < UI_BATCH_MAX; i++)
        if (ui_batch_jobs[i].used && ui_batch_jobs[i].state == 1) n++;
    return n;
}

static u32 ui_batch_core_load(u32 core)
{
    u32 n = 0;
    for (u32 i = 0; i < UI_BATCH_MAX; i++) {
        if (!ui_batch_jobs[i].used || ui_batch_jobs[i].state != 1) continue;
        if ((((u32)ui_batch_jobs[i].pid) >> 16) == core) n++;
    }
    return n;
}

static struct ui_service_unit *ui_service_find(const char *name)
{
    if (!name || !name[0]) return NULL;
    for (u32 i = 0; i < UI_SVC_MAX; i++) {
        if (!ui_services[i].used) continue;
        if (ui_streq(ui_services[i].name, name)) return &ui_services[i];
    }
    return NULL;
}

static bool ui_service_pid_active(i32 pid)
{
    if (pid <= 0) return false;
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    for (u32 i = 0; i < n; i++) {
        if ((i32)snap[i].pid == pid) return true;
    }
    return false;
}

static bool ui_service_target_enabled(const struct ui_service_unit *s)
{
    if (!s || !s->used) return false;
    if (ui_service_target == UI_SVC_TARGET_ALL) return true;
    return s->target == ui_service_target;
}

static bool ui_service_dep_ready(const struct ui_service_unit *s)
{
    if (!s) return false;
    if (s->depends[0] == 0) return true;
    struct ui_service_unit *d = ui_service_find(s->depends);
    if (!d || !d->used) return false;
    return d->state == 1;
}

static void ui_service_tick(void)
{
    if (!ui_service_running) return;
    u64 now = timer_ticks();
    for (u32 i = 0; i < UI_SVC_MAX; i++) {
        struct ui_service_unit *s = &ui_services[i];
        if (!s->used) continue;
        if (s->state == 1 && s->pid > 0) {
            if (ui_service_pid_active(s->pid))
                continue;
            bool should_restart = (!s->stop_requested) &&
                                  (s->restart_policy == UI_SVC_RP_ALWAYS || s->restart_policy == UI_SVC_RP_ONFAIL);
            if (!should_restart) {
                s->state = s->stop_requested ? 0 : 3;
                s->pid = -1;
                s->stop_requested = false;
                continue;
            }
            if (s->window_start_ms == 0 || now - s->window_start_ms > 60000ULL) {
                s->window_start_ms = now;
                s->restarts = 0;
            }
            s->restarts++;
            s->pid = -1;
            if (s->restarts > s->max_restarts) {
                s->state = 3;
            } else {
                s->state = 2;
                s->next_action_ms = now + (u64)s->backoff_ms;
            }
        }
    }

    for (u32 i = 0; i < UI_SVC_MAX; i++) {
        struct ui_service_unit *s = &ui_services[i];
        if (!s->used) continue;
        if (!ui_service_target_enabled(s)) continue;
        if (!ui_service_dep_ready(s)) continue;
        if (s->state == 2 && now < s->next_action_ms) continue;
        if (s->state == 1 || s->state == 3) continue;
        i32 pid = proc_launch_on_core_as_prio(s->preferred_core, s->path, s->principal_id, s->priority_class);
        if (pid > 0) {
            s->pid = pid;
            s->state = 1;
            s->stop_requested = false;
            if (s->window_start_ms == 0) s->window_start_ms = now;
        } else {
            s->state = 2;
            s->next_action_ms = now + (u64)s->backoff_ms;
        }
    }
}

static void ui_batch_tick(void)
{
    ui_service_tick();
    u64 now = timer_ticks();
    for (u32 i = 0; i < UI_BATCH_MAX; i++) {
        struct ui_batch_job *j = &ui_batch_jobs[i];
        if (!j->used || j->state != 1) continue;
        if (!ui_batch_pid_active(j->pid)) {
            if (j->interval_ms > 0) {
                j->state = 0;
                j->next_due_ms = now + j->interval_ms;
            } else {
                j->state = 2;
            }
            j->pid = -1;
        }
    }

    if (!ui_batch_running) return;

    while (ui_batch_running_count() < ui_batch_parallel) {
        struct ui_batch_job *pick = NULL;
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            if (ui_batch_jobs[i].used && ui_batch_jobs[i].state == 0 &&
                ui_batch_jobs[i].next_due_ms <= now) {
                pick = &ui_batch_jobs[i];
                break;
            }
        }
        if (!pick) break;

        if (pick->is_script) {
            ui_cmd_source(pick->path);
            pick->attempts++;
            pick->last_err = 0;
            if (pick->interval_ms > 0) {
                pick->next_due_ms = now + pick->interval_ms;
                pick->state = 0;
            } else {
                pick->state = 2;
            }
            continue;
        }

        u32 core = pick->preferred_core;
        if (core != CORE_USERM && core != CORE_USER0 && core != CORE_USER1) {
            u32 l1 = ui_batch_core_load(CORE_USERM);
            u32 l2 = ui_batch_core_load(CORE_USER0);
            u32 l3 = ui_batch_core_load(CORE_USER1);
            core = CORE_USERM;
            if (l2 < l1) core = CORE_USER0;
            if (l3 < ui_batch_core_load(core)) core = CORE_USER1;
        }
        i32 pid = proc_launch_on_core_as_prio(core, pick->path, pick->principal_id, pick->priority_class);
        if (pid > 0) {
            pick->state = 1;
            pick->pid = pid;
            pick->attempts++;
            pick->last_err = 0;
        } else {
            pick->attempts++;
            pick->last_err = -1;
            if (pick->attempts > (pick->retries + 1))
                pick->state = 3;
        }
    }
}

static void ui_cmd_batch(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("batch add <path> [1|2|3|auto] [lazy|low|normal|high|realtime] [principal] [retries]\n");
        ui_console_write("batch at <delay_ms> <path> [1|2|3|auto] [lazy|low|normal|high|realtime] [principal] [retries]\n");
        ui_console_write("batch every <interval_ms> <path> [1|2|3|auto] [lazy|low|normal|high|realtime] [principal]\n");
        ui_console_write("batch run [parallel] | batch stop | batch status | batch list | batch clear\n");
        return;
    }
    if (ui_streq(argv[1], "add") || ui_streq(argv[1], "at") || ui_streq(argv[1], "every")) {
        u32 path_idx = 2;
        u32 delay_ms = 0;
        u32 interval_ms = 0;
        if (ui_streq(argv[1], "at")) {
            if (argc < 4 || !ui_parse_u32(argv[2], &delay_ms)) {
                ui_console_write("ERR: usage batch at <delay_ms> <path> [1|2|3|auto] [priority] [principal] [retries]\n");
                return;
            }
            path_idx = 3;
        } else if (ui_streq(argv[1], "every")) {
            if (argc < 4 || !ui_parse_u32(argv[2], &interval_ms) || interval_ms == 0) {
                ui_console_write("ERR: usage batch every <interval_ms> <path> [1|2|3|auto] [priority] [principal]\n");
                return;
            }
            path_idx = 3;
        }
        if (argc <= path_idx) {
            ui_console_write("ERR: batch path missing\n");
            return;
        }
        u32 slot = UI_BATCH_MAX;
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            if (!ui_batch_jobs[i].used || ui_batch_jobs[i].state >= 2) { slot = i; break; }
        }
        if (slot == UI_BATCH_MAX) {
            ui_console_write("ERR: batch queue full\n");
            return;
        }
        char path[128];
        bool is_script = false;
        if (!ui_resolve_job_path(argv[path_idx], path, sizeof(path), &is_script)) {
            ui_console_write("ERR: bad path\n");
            return;
        }
        if (!walfs_find(path)) {
            ui_console_write("ERR: job target not found\n");
            return;
        }
        u32 core = 0;
        u32 argi = path_idx + 1;
        if (argc > argi) {
            if (argv[argi][0] == '1') core = CORE_USERM;
            else if (argv[argi][0] == '2') core = CORE_USER0;
            else if (argv[argi][0] == '3') core = CORE_USER1;
            else if (!ui_streq(argv[argi], "auto")) {
                ui_console_write("ERR: core must be 1|2|3|auto\n");
                return;
            }
            argi++;
        }
        u32 priority_class = PROC_PRIO_NORMAL;
        if (argc > argi) {
            u32 ptmp = 0;
            if (ui_parse_priority(argv[argi], &ptmp)) {
                priority_class = ptmp;
                argi++;
            }
        }
        u32 principal_id = principal_current();
        if (argc > argi) {
            if (!ui_parse_u32(argv[argi], &principal_id) || principal_id >= PRINCIPAL_MAX) {
                ui_console_write("ERR: invalid principal\n");
                return;
            }
            if (!principal_has_cap(principal_id, PRINCIPAL_EXEC)) {
                ui_console_write("ERR: principal missing EXEC capability\n");
                return;
            }
            argi++;
        }
        u32 retries = 0;
        if (argc > argi && !ui_parse_u32(argv[argi], &retries)) {
            ui_console_write("ERR: invalid retries\n");
            return;
        }
        struct ui_batch_job *j = &ui_batch_jobs[slot];
        simd_zero(j, sizeof(*j));
        j->used = true;
        j->id = ui_batch_next_id++;
        for (u32 i = 0; path[i] && i + 1 < sizeof(j->path); i++) j->path[i] = path[i];
        j->preferred_core = core;
        j->priority_class = priority_class;
        j->principal_id = principal_id;
        j->state = 0;
        j->pid = -1;
        j->retries = retries;
        j->interval_ms = interval_ms;
        j->next_due_ms = timer_ticks() + delay_ms;
        j->is_script = is_script;
        fb_printf("OK: batch id=%u queued\n", j->id);
        uart_puts("OK: batch id=");
        uart_hex(j->id);
        uart_puts(" queued\n");
        return;
    }
    if (ui_streq(argv[1], "run")) {
        if (argc >= 3) {
            u32 p = 0;
            if (!ui_parse_u32(argv[2], &p) || p == 0 || p > 8) {
                ui_console_write("ERR: parallel must be 1..8\n");
                return;
            }
            ui_batch_parallel = p;
        }
        ui_batch_running = true;
        ui_console_write("OK: batch scheduler running\n");
        return;
    }
    if (ui_streq(argv[1], "stop")) {
        ui_batch_running = false;
        ui_console_write("OK: batch scheduler paused\n");
        return;
    }
    if (ui_streq(argv[1], "clear")) {
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            if (ui_batch_jobs[i].used && ui_batch_jobs[i].state == 1 && ui_batch_jobs[i].pid > 0)
                proc_kill_pid((u32)ui_batch_jobs[i].pid, 0xFFFF3000U);
            ui_batch_jobs[i].used = false;
        }
        ui_console_write("OK: batch queue cleared\n");
        return;
    }
    if (ui_streq(argv[1], "status") || ui_streq(argv[1], "list")) {
        u32 q = 0, r = 0, d = 0, f = 0;
        for (u32 i = 0; i < UI_BATCH_MAX; i++) {
            struct ui_batch_job *j = &ui_batch_jobs[i];
            if (!j->used) continue;
            if (j->state == 0) q++;
            else if (j->state == 1) r++;
            else if (j->state == 2) d++;
            else if (j->state == 3) f++;
            if (ui_streq(argv[1], "list")) {
                const char *st = (j->state == 0) ? "queued" :
                                 (j->state == 1) ? "running" :
                                 (j->state == 2) ? "done" :
                                 (j->state == 3) ? "failed" : "canceled";
                fb_printf("#%u %s type=%s core=%u pri=%s princ=%u pid=0x%x tries=%u/%u interval=%u %s\n",
                          j->id, st, j->is_script ? "pis" : "pix",
                          j->preferred_core ? j->preferred_core : 0,
                          ui_priority_str(j->priority_class),
                          j->principal_id, (u32)j->pid, j->attempts, j->retries + 1,
                          j->interval_ms, j->path);
                uart_puts("#"); uart_hex(j->id); uart_puts(" ");
                uart_puts(st); uart_puts(" path="); uart_puts(j->path); uart_puts("\n");
            }
        }
        fb_printf("batch: mode=%s parallel=%u queued=%u running=%u done=%u failed=%u\n",
                  ui_batch_running ? "run" : "pause", ui_batch_parallel, q, r, d, f);
        uart_puts("batch mode=");
        uart_puts(ui_batch_running ? "run" : "pause");
        uart_puts(" parallel="); uart_hex(ui_batch_parallel);
        uart_puts(" q="); uart_hex(q);
        uart_puts(" r="); uart_hex(r);
        uart_puts(" d="); uart_hex(d);
        uart_puts(" f="); uart_hex(f);
        uart_puts("\n");
        return;
    }
    ui_console_write("ERR: unknown batch subcommand\n");
}

static void ui_cmd_svc(u32 argc, char **argv)
{
    if (argc < 2 || ui_streq(argv[1], "help")) {
        ui_console_write("svc add <name> <path> [dep|-] [target:default|rescue|all] [1|2|3] [priority] [principal] [restart] [max_restarts] [backoff_ms]\n");
        ui_console_write("svc start|stop|restart <name> | svc run|pause | svc target <default|rescue|all> | svc list | svc clear\n");
        return;
    }
    if (ui_streq(argv[1], "add")) {
        if (argc < 4) { ui_console_write("ERR: usage svc add <name> <path> ...\n"); return; }
        if (ui_service_find(argv[2])) { ui_console_write("ERR: service exists\n"); return; }
        i32 slot = -1;
        for (u32 i = 0; i < UI_SVC_MAX; i++) if (!ui_services[i].used) { slot = (i32)i; break; }
        if (slot < 0) { ui_console_write("ERR: service table full\n"); return; }

        char path[128];
        bool is_script = false;
        if (!ui_resolve_job_path(argv[3], path, sizeof(path), &is_script) || !walfs_find(path)) {
            ui_console_write("ERR: service path not found\n");
            return;
        }
        if (is_script) {
            ui_console_write("ERR: svc requires executable .pix path\n");
            return;
        }

        struct ui_service_unit *s = &ui_services[(u32)slot];
        simd_zero(s, sizeof(*s));
        s->used = true;
        for (u32 i = 0; argv[2][i] && i + 1 < sizeof(s->name); i++) s->name[i] = argv[2][i];
        for (u32 i = 0; path[i] && i + 1 < sizeof(s->path); i++) s->path[i] = path[i];
        s->target = UI_SVC_TARGET_DEFAULT;
        s->preferred_core = CORE_USERM;
        s->principal_id = principal_current();
        s->priority_class = PROC_PRIO_NORMAL;
        s->restart_policy = UI_SVC_RP_ONFAIL;
        s->max_restarts = 5;
        s->backoff_ms = 1000;
        s->state = 0;
        s->pid = -1;

        if (argc >= 5 && !ui_streq(argv[4], "-")) {
            for (u32 i = 0; argv[4][i] && i + 1 < sizeof(s->depends); i++) s->depends[i] = argv[4][i];
        }
        if (argc >= 6) {
            if (ui_streq(argv[5], "default")) s->target = UI_SVC_TARGET_DEFAULT;
            else if (ui_streq(argv[5], "rescue")) s->target = UI_SVC_TARGET_RESCUE;
            else if (ui_streq(argv[5], "all")) s->target = UI_SVC_TARGET_ALL;
        }
        if (argc >= 7) {
            if (argv[6][0] == '1') s->preferred_core = CORE_USERM;
            else if (argv[6][0] == '2') s->preferred_core = CORE_USER0;
            else if (argv[6][0] == '3') s->preferred_core = CORE_USER1;
        }
        if (argc >= 8) {
            u32 p = 0;
            if (ui_parse_priority(argv[7], &p)) s->priority_class = p;
        }
        if (argc >= 9) {
            u32 pr = 0;
            if (ui_parse_u32(argv[8], &pr) && pr < PRINCIPAL_MAX && principal_has_cap(pr, PRINCIPAL_EXEC))
                s->principal_id = pr;
        }
        if (argc >= 10) {
            if (ui_streq(argv[9], "never")) s->restart_policy = UI_SVC_RP_NEVER;
            else if (ui_streq(argv[9], "onfail")) s->restart_policy = UI_SVC_RP_ONFAIL;
            else if (ui_streq(argv[9], "always")) s->restart_policy = UI_SVC_RP_ALWAYS;
        }
        if (argc >= 11) { u32 v = 0; if (ui_parse_u32(argv[10], &v)) s->max_restarts = v; }
        if (argc >= 12) { u32 v = 0; if (ui_parse_u32(argv[11], &v)) s->backoff_ms = v; }
        ui_console_write("OK: service added\n");
        return;
    }
    if (ui_streq(argv[1], "run")) { ui_service_running = true; ui_console_write("OK: supervisor running\n"); return; }
    if (ui_streq(argv[1], "pause")) { ui_service_running = false; ui_console_write("OK: supervisor paused\n"); return; }
    if (ui_streq(argv[1], "target")) {
        if (argc < 3) { ui_console_write("ERR: usage svc target <default|rescue|all>\n"); return; }
        if (ui_streq(argv[2], "default")) ui_service_target = UI_SVC_TARGET_DEFAULT;
        else if (ui_streq(argv[2], "rescue")) ui_service_target = UI_SVC_TARGET_RESCUE;
        else if (ui_streq(argv[2], "all")) ui_service_target = UI_SVC_TARGET_ALL;
        else { ui_console_write("ERR: bad target\n"); return; }
        ui_console_write("OK: target updated\n");
        return;
    }
    if (ui_streq(argv[1], "clear")) {
        for (u32 i = 0; i < UI_SVC_MAX; i++) {
            if (!ui_services[i].used) continue;
            if (ui_services[i].pid > 0) (void)proc_kill_pid((u32)ui_services[i].pid, 0xFFFF4000U);
            ui_services[i].used = false;
        }
        ui_console_write("OK: services cleared\n");
        return;
    }
    if (ui_streq(argv[1], "list")) {
        for (u32 i = 0; i < UI_SVC_MAX; i++) {
            struct ui_service_unit *s = &ui_services[i];
            if (!s->used) continue;
            fb_printf("svc %-12s state=%u pid=%x dep=%s path=%s\n",
                      s->name, s->state, (u32)(s->pid > 0 ? s->pid : 0),
                      s->depends[0] ? s->depends : "-", s->path);
        }
        return;
    }
    if ((ui_streq(argv[1], "start") || ui_streq(argv[1], "stop") || ui_streq(argv[1], "restart")) && argc >= 3) {
        struct ui_service_unit *s = ui_service_find(argv[2]);
        if (!s) { ui_console_write("ERR: service not found\n"); return; }
        if (ui_streq(argv[1], "start")) {
            s->state = 0;
            s->stop_requested = false;
            s->next_action_ms = timer_ticks();
            ui_console_write("OK: service start queued\n");
        } else if (ui_streq(argv[1], "stop")) {
            s->stop_requested = true;
            if (s->pid > 0) (void)proc_kill_pid((u32)s->pid, 0xFFFF4001U);
            s->state = 0;
            s->pid = -1;
            ui_console_write("OK: service stopped\n");
        } else {
            s->stop_requested = true;
            if (s->pid > 0) (void)proc_kill_pid((u32)s->pid, 0xFFFF4002U);
            s->pid = -1;
            s->stop_requested = false;
            s->state = 0;
            s->next_action_ms = timer_ticks();
            ui_console_write("OK: service restart queued\n");
        }
        return;
    }
    ui_console_write("ERR: unknown svc subcommand\n");
}

static void ui_scheduler_exec_line(const char *line)
{
    if (!line || !*line) return;
    char cmd[320];
    u32 p = 0;
    const char *prefix = "batch ";
    for (u32 i = 0; prefix[i] && p + 1 < sizeof(cmd); i++) cmd[p++] = prefix[i];
    const char *s = line;
    bool known =
        (s[0]=='a'&&s[1]=='d'&&s[2]=='d'&&s[3]==' ') ||
        (s[0]=='a'&&s[1]=='t'&&s[2]==' ') ||
        (s[0]=='e'&&s[1]=='v'&&s[2]=='e'&&s[3]=='r'&&s[4]=='y'&&s[5]==' ') ||
        ui_streq(s, "run") || ui_streq(s, "stop") || ui_streq(s, "status") ||
        ui_streq(s, "list") || ui_streq(s, "clear") || ui_streq(s, "help");
    if (!known) {
        const char *add = "add ";
        for (u32 i = 0; add[i] && p + 1 < sizeof(cmd); i++) cmd[p++] = add[i];
    }
    while (*s && p + 1 < sizeof(cmd)) cmd[p++] = *s++;
    cmd[p] = 0;
    ui_console_exec(cmd);
}

static void ui_scheduler_feed_char(i32 c)
{
    if (c < 0) return;
    if (c == '\r' || c == '\n') {
        ui_sched_line[ui_sched_len] = 0;
        ui_scheduler_exec_line(ui_sched_line);
        ui_sched_len = 0;
        ui_last_render = 0;
        return;
    }
    if (c == '\b' || c == 127) {
        if (ui_sched_len > 0) ui_sched_len--;
        ui_last_render = 0;
        return;
    }
    if (c < 0x20 || c > 0x7E) return;
    if (ui_sched_len + 1 >= UI_CONSOLE_LINE_MAX) return;
    ui_sched_line[ui_sched_len++] = (char)c;
    ui_last_render = 0;
}

static void ui_render_scheduler(void)
{
    fb_clear(0x00000000);
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("PIOS Scheduler (F4)\n");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("========================================\n");
    fb_printf("Type: <path> OR add/at/every/run/stop/status/list/clear then Enter\n");
    fb_printf("Examples: /bin/demo.pix | at 5000 /bin/demo.pix 2 0 1 | every 10000 /bin/job.pis auto 0\n");
    fb_printf("F3 console | F2 manager | F1 process view\n\n");

    u32 q = 0, r = 0, d = 0, f = 0;
    for (u32 i = 0; i < UI_BATCH_MAX; i++) {
        struct ui_batch_job *j = &ui_batch_jobs[i];
        if (!j->used) continue;
        if (j->state == 0) q++;
        else if (j->state == 1) r++;
        else if (j->state == 2) d++;
        else if (j->state == 3) f++;
    }
    fb_printf("mode=%s parallel=%u queued=%u running=%u done=%u failed=%u\n\n",
              ui_batch_running ? "run" : "pause", ui_batch_parallel, q, r, d, f);
    fb_printf("sched> %s", ui_sched_line);
}

static void ui_console_exec(char *line)
{
    char *argv[UI_CONSOLE_ARGV_MAX];
    u32 argc = 0;
    char *p = line;

    while (*p && argc < UI_CONSOLE_ARGV_MAX) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = 0;
    }
    if (argc == 0) return;

    if (ui_streq(argv[0], "help")) {
        ui_console_write("help echo clear time ps kill launch run pwd cd lsdir mkdir touch\n");
        ui_console_write("copy cp cpdir mv cat stat rm find hexdump df mount umount\n");
        ui_console_write("stream if for foreach source env batch svc edit\n");
        ui_console_write("hexsec fsinspect netcfg disk db capsule\n");
        ui_console_write("netcfg set <ip|mask|gw|dns> <a.b.c.d> | netcfg apply\n");
        ui_console_write("netcfg dhcp <on|off> [timeout_ms] | netcfg addnbr <ip> <mac>\n");
        ui_console_write("stream <tcp|udp> <ip> <port> from <file|text|tty> <arg?> to <console|file> [path] [timeout_ms]\n");
        ui_console_write("batch add|at|every supports [core] [priority] [principal] [retries]\n");
        ui_console_write("batch run [parallel] | batch stop | batch status | batch list\n");
        ui_console_write("svc add|start|stop|restart|run|pause|target|list|clear\n");
        ui_console_write("launch <path> [1|2|3] [lazy|low|normal|high|realtime]\n");
        ui_console_write("prio <pid> <lazy|low|normal|high|realtime> | affinity <pid> <1|2|3>\n");
        ui_console_write("source <script[.pis]> (auto-adds .pis)\n");
        ui_console_write("env [list|get|set|pset|unset|save|load]\n");
        ui_console_write("edit|edit.pix <path> (Ctrl+S save, Ctrl+Q exit)\n");
        ui_console_write("disk writezero requires --force\n");
    } else if (ui_streq(argv[0], "pwd")) {
        ui_console_write(ui_cwd);
        ui_console_write("\n");
    } else if (ui_streq(argv[0], "cd")) {
        char abs[256];
        const char *target = (argc < 2) ? "/" : argv[1];
        if (!ui_path_resolve(target, abs, sizeof(abs))) {
            ui_console_write("ERR: bad path\n");
        } else {
            u64 id = walfs_find(abs);
            struct walfs_inode ino;
            if (!id || !walfs_stat(id, &ino) || !(ino.flags & WALFS_DIR)) {
                ui_console_write("ERR: directory not found\n");
            } else {
                u32 n = pios_strlen(abs);
                for (u32 i = 0; i <= n; i++) ui_cwd[i] = abs[i];
                ui_console_write("OK: cwd updated\n");
            }
        }
    } else if (ui_streq(argv[0], "echo")) {
        if (argc >= 4 && ui_streq(argv[argc - 2], ">")) {
            char dst[256];
            if (!ui_path_resolve(argv[argc - 1], dst, sizeof(dst))) {
                ui_console_write("ERR: bad path\n");
                return;
            }
            u64 dst_id = walfs_find(dst);
            if (!dst_id) {
                if (!ui_fs_create_path(dst, false)) {
                    ui_console_write("ERR: echo redirect create failed\n");
                    return;
                }
                dst_id = walfs_find(dst);
            }
            if (!dst_id) {
                ui_console_write("ERR: echo redirect target missing\n");
                return;
            }
            char out[256];
            u32 p = 0;
            for (u32 i = 1; i + 2 < argc; i++) {
                const char *s = argv[i];
                while (*s && p + 1 < sizeof(out)) out[p++] = *s++;
                if (i + 3 < argc && p + 1 < sizeof(out)) out[p++] = ' ';
            }
            if (p + 1 < sizeof(out)) out[p++] = '\n';
            out[p] = 0;
            if (!walfs_write(dst_id, 0, out, p)) {
                ui_console_write("ERR: echo redirect write failed\n");
                return;
            }
            ui_console_write("OK: wrote file\n");
        } else {
            for (u32 i = 1; i < argc; i++) {
                ui_console_write(argv[i]);
                if (i + 1 < argc) ui_console_write(" ");
            }
            ui_console_write("\n");
        }
    } else if (ui_streq(argv[0], "clear")) {
        fb_clear(0x00000000);
    } else if (ui_streq(argv[0], "time")) {
        u64 t = timer_ticks();
        fb_printf("ticks=%X\n", t);
        uart_puts("ticks=");
        uart_hex(t);
        uart_puts("\n");
    } else if (ui_streq(argv[0], "ps")) {
        ui_console_print_ps();
    } else if (ui_streq(argv[0], "kill")) {
        if (argc < 2) {
            ui_console_write("ERR: usage kill <pid>\n");
        } else {
            u32 pid = 0;
            if (!ui_parse_u32(argv[1], &pid) || !proc_kill_pid(pid, 0xFFFF2000U)) {
                ui_console_write("ERR: kill failed\n");
            } else {
                ui_console_write("OK: killed\n");
            }
        }
    } else if (ui_streq(argv[0], "launch") || ui_streq(argv[0], "run")) {
        if (argc < 2) {
            ui_console_write("ERR: usage launch <path> [1|2|3]\n");
        } else {
            if (ui_streq(argv[0], "run")) {
                char sp[256];
                if (ui_resolve_pis_path(argv[1], sp, sizeof(sp)) && ui_has_suffix(sp, ".pis") && walfs_find(sp)) {
                    ui_cmd_source(sp);
                    return;
                }
            }
            u32 target_core = CORE_USERM;
            if (argc >= 3) {
                if (argv[2][0] == '1') target_core = CORE_USERM;
                else if (argv[2][0] == '2') target_core = CORE_USER0;
                else if (argv[2][0] == '3') target_core = CORE_USER1;
            }
            u32 prio = PROC_PRIO_NORMAL;
            if (argc >= 4) {
                if (!ui_parse_priority(argv[3], &prio)) {
                    ui_console_write("ERR: invalid priority\n");
                    return;
                }
            } else if (argc >= 3 && argv[2][0] != '1' && argv[2][0] != '2' && argv[2][0] != '3') {
                if (!ui_parse_priority(argv[2], &prio)) {
                    ui_console_write("ERR: invalid core/priority\n");
                    return;
                }
            }
            i32 pid = proc_launch_on_core_as_prio(target_core, argv[1], principal_current(), prio);
            if (pid <= 0) {
                ui_console_write("ERR: launch failed\n");
            } else {
                fb_printf("OK: pid=0x%x\n", (u32)pid);
                uart_puts("OK: pid=");
                uart_hex((u32)pid);
                uart_puts("\n");
            }
        }
    } else if (ui_streq(argv[0], "prio")) {
        if (argc < 3) {
            ui_console_write("ERR: usage prio <pid> <lazy|low|normal|high|realtime>\n");
        } else {
            u32 pid = 0, p = 0;
            if (!ui_parse_u32(argv[1], &pid) || !ui_parse_priority(argv[2], &p) || !proc_set_priority(pid, p))
                ui_console_write("ERR: prio update failed\n");
            else
                ui_console_write("OK: priority updated\n");
        }
    } else if (ui_streq(argv[0], "affinity")) {
        if (argc < 3) {
            ui_console_write("ERR: usage affinity <pid> <1|2|3>\n");
        } else {
            u32 pid = 0;
            u32 core = CORE_USERM;
            if (!ui_parse_u32(argv[1], &pid)) {
                ui_console_write("ERR: invalid pid\n");
            } else {
                if (argv[2][0] == '1') core = CORE_USERM;
                else if (argv[2][0] == '2') core = CORE_USER0;
                else if (argv[2][0] == '3') core = CORE_USER1;
                else {
                    ui_console_write("ERR: core must be 1|2|3\n");
                    return;
                }
                if (!proc_set_affinity(pid, core))
                    ui_console_write("ERR: affinity update failed\n");
                else
                    ui_console_write("OK: affinity updated\n");
            }
        }
    } else if (ui_streq(argv[0], "hexsec")) {
        if (argc < 2) {
            ui_console_write("ERR: usage hexsec <lba>\n");
        } else {
            u32 lba = 0;
            if (!ui_parse_u32(argv[1], &lba)) {
                ui_console_write("ERR: invalid lba\n");
            } else {
                ui_dump_sector(lba);
            }
        }
    } else if (ui_streq(argv[0], "fsinspect")) {
        if (argc < 2) ui_cmd_fsinspect(ui_cwd);
        else {
            char abs[256];
            if (!ui_path_resolve(argv[1], abs, sizeof(abs))) ui_console_write("ERR: bad path\n");
            else ui_cmd_fsinspect(abs);
        }
    } else if (ui_streq(argv[0], "lsdir")) {
        if (argc < 2) ui_cmd_lsdir(ui_cwd);
        else ui_cmd_lsdir(argv[1]);
    } else if (ui_streq(argv[0], "mkdir")) {
        if (argc < 2) ui_console_write("ERR: usage mkdir <path>\n");
        else ui_cmd_mkdir(argv[1]);
    } else if (ui_streq(argv[0], "touch")) {
        if (argc < 2) ui_console_write("ERR: usage touch <path>\n");
        else ui_cmd_touch(argv[1]);
    } else if (ui_streq(argv[0], "copy")) {
        if (argc < 3) ui_console_write("ERR: usage copy <src> <dst>\n");
        else ui_cmd_copy(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "cp")) {
        if (argc < 3) ui_console_write("ERR: usage cp <src> <dst>\n");
        else ui_cmd_copy(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "cpdir")) {
        if (argc < 3) ui_console_write("ERR: usage cpdir <src_dir> <dst_dir>\n");
        else ui_cmd_cpdir(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "mv")) {
        if (argc < 3) ui_console_write("ERR: usage mv <src> <dst>\n");
        else ui_cmd_mv(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "cat")) {
        if (argc < 2) ui_console_write("ERR: usage cat <path>\n");
        else ui_cmd_cat(argv[1]);
    } else if (ui_streq(argv[0], "stat")) {
        if (argc < 2) ui_console_write("ERR: usage stat <path>\n");
        else ui_cmd_stat(argv[1]);
    } else if (ui_streq(argv[0], "rm")) {
        if (argc < 2) ui_console_write("ERR: usage rm <path>\n");
        else ui_cmd_rm(argv[1]);
    } else if (ui_streq(argv[0], "find")) {
        if (argc < 3) ui_console_write("ERR: usage find <dir> <needle>\n");
        else ui_cmd_find(argv[1], argv[2]);
    } else if (ui_streq(argv[0], "hexdump")) {
        u32 max = 256;
        if (argc < 2) ui_console_write("ERR: usage hexdump <path> [max_bytes]\n");
        else {
            if (argc >= 3 && !ui_parse_u32(argv[2], &max)) {
                ui_console_write("ERR: invalid max_bytes\n");
            } else {
                ui_cmd_hexdump(argv[1], max);
            }
        }
    } else if (ui_streq(argv[0], "df")) {
        ui_cmd_df();
    } else if (ui_streq(argv[0], "mount")) {
        ui_cmd_mount(argc, argv);
    } else if (ui_streq(argv[0], "umount")) {
        ui_cmd_mount(argc, argv);
    } else if (ui_streq(argv[0], "stream")) {
        ui_cmd_stream(argc, argv);
    } else if (ui_streq(argv[0], "env")) {
        ui_cmd_env(argc, argv);
    } else if (ui_streq(argv[0], "if")) {
        ui_cmd_if(argc, argv);
    } else if (ui_streq(argv[0], "for")) {
        ui_cmd_for(argc, argv);
    } else if (ui_streq(argv[0], "foreach")) {
        ui_cmd_foreach(argc, argv);
    } else if (ui_streq(argv[0], "source")) {
        if (argc < 2) ui_console_write("ERR: usage source <path>\n");
        else ui_cmd_source(argv[1]);
    } else if (ui_streq(argv[0], "edit") || ui_streq(argv[0], "edit.pix")) {
        if (argc < 2) ui_console_write("ERR: usage edit <path>\n");
        else ui_cmd_edit(argv[1]);
    } else if (ui_streq(argv[0], "batch")) {
        ui_cmd_batch(argc, argv);
    } else if (ui_streq(argv[0], "svc")) {
        ui_cmd_svc(argc, argv);
    } else if (ui_streq(argv[0], "netcfg")) {
        ui_cmd_netcfg(argc, argv);
    } else if (ui_streq(argv[0], "disk")) {
        ui_cmd_disk(argc, argv);
    } else if (ui_streq(argv[0], "db")) {
        ui_cmd_db(argc, argv);
    } else if (ui_streq(argv[0], "capsule")) {
        ui_cmd_capsule(argc, argv);
    } else {
        ui_console_write("ERR: unknown command\n");
    }
}

static void ui_console_feed_char(i32 c)
{
    if (c < 0) return;
    if (c == '\r' || c == '\n') {
        ui_console_write("\n");
        ui_console_line[ui_console_len] = 0;
        ui_console_exec(ui_console_line);
        ui_console_len = 0;
        ui_console_prompt();
        return;
    }
    if (c == '\b' || c == 127) {
        if (ui_console_len > 0) {
            ui_console_len--;
            ui_console_write("\b \b");
        }
        return;
    }
    if (c < 0x20 || c > 0x7E)
        return;
    if (ui_console_len + 1 >= UI_CONSOLE_LINE_MAX) {
        ui_console_write("\nERR: line too long\n");
        ui_console_len = 0;
        ui_console_prompt();
        return;
    }
    ui_console_line[ui_console_len++] = (char)c;
    char out[2] = { (char)c, 0 };
    ui_console_write(out);
}

static void ui_console_pump_input(void)
{
    i32 c;
    while ((c = usb_kbd_try_getc()) >= 0)
        ui_console_feed_char(c);
    while ((c = uart_try_getc()) >= 0)
        ui_console_feed_char(c);
}

static const char *ui_proc_state_str(u32 s)
{
    if (s == PROC_READY) return "ready";
    if (s == PROC_RUNNING) return "running";
    if (s == PROC_BLOCKED) return "blocked";
    if (s == PROC_DEAD) return "dead";
    return "unknown";
}

static void ui_render_process_view(void)
{
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    fb_clear(0x00000000);
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("PIOS Process View (F1 cycle, F2 manager)\n");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("========================================\n\n");

    if (n == 0) {
        fb_printf("No active processes.\n");
        return;
    }
    if (ui_selected >= n)
        ui_selected = 0;

    struct proc_ui_entry *e = &snap[ui_selected];
    const char *keys[] = {
        "pid", "state", "affinity", "cpu_pct", "mem_kib", "principal", "preemptions"
    };
    u32 vals[] = {
        e->pid, e->state, e->affinity_core, e->cpu_percent, e->mem_kib, e->principal_id, e->preemptions
    };

    fb_printf("Process %u / %u\n\n", ui_selected + 1, n);
    for (u32 i = 0; i < 7; i++) {
        if (i == 1) {
            fb_printf("%s: %s\n", keys[i], ui_proc_state_str(vals[i]));
        } else if (i == 0) {
            fb_printf("%s: 0x%x\n", keys[i], vals[i]);
        } else {
            fb_printf("%s: %u\n", keys[i], vals[i]);
        }
    }
    fb_printf("runtime_ticks: %X\n", e->runtime_ticks);
}

static void ui_render_process_manager(void)
{
    struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
    u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
    fb_clear(0x00000000);
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("PIOS Process Manager (F2)\n");
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("========================================\n");
    fb_printf("Controls: F1 detail | K kill selected | L launch next | C open console\n\n");
    fb_printf("PID      AFF  CPU%%  MEM(KiB)  STATE\n");

    if (n == 0) {
        fb_printf("(no active processes)\n");
    } else {
        if (ui_selected >= n)
            ui_selected = 0;
        for (u32 i = 0; i < n; i++) {
            fb_set_color((i == ui_selected) ? 0x00FF9900 : 0x00FFFFFF, 0x00000000);
            fb_printf("0x%x   %u    %u     %u      %s\n",
                      snap[i].pid, snap[i].affinity_core, snap[i].cpu_percent,
                      snap[i].mem_kib, ui_proc_state_str(snap[i].state));
        }
        fb_set_color(0x00FFFFFF, 0x00000000);
    }

    if (ui_status_code == 1) fb_printf("\nstatus: killed selected pid\n");
    else if (ui_status_code == 2) fb_printf("\nstatus: launch request submitted\n");
    else if (ui_status_code == -1) fb_printf("\nstatus: action failed\n");
}

static void ui_handle_keys(void)
{
    i32 key;
    while ((key = usb_kbd_try_getkey()) >= 0) {
        if (key == USB_KBD_KEY_F1) {
            ui_mode = UI_MODE_PROC_VIEW;
            ui_selected++;
            ui_last_render = 0;
        } else if (key == USB_KBD_KEY_F2) {
            ui_mode = UI_MODE_PROC_MANAGER;
            ui_last_render = 0;
        } else if (key == USB_KBD_KEY_F3) {
            ui_mode = UI_MODE_CONSOLE;
            ui_console_len = 0;
            fb_clear(0x00000000);
            fb_set_color(UI_SHELL_TEXT_COLOR, 0x00000000);
            ui_console_write("PIOS F3 Console (serial + HDMI)\n");
            ui_console_write("Type 'help' for commands.\n");
            ui_console_prompt();
            ui_last_render = 1;
        } else if (key == USB_KBD_KEY_F4) {
            ui_mode = UI_MODE_SCHEDULER;
            ui_sched_len = 0;
            ui_sched_line[0] = 0;
            ui_last_render = 0;
        }
    }

    if (ui_mode == UI_MODE_CONSOLE) {
        ui_console_pump_input();
        return;
    }

    if (ui_mode == UI_MODE_SCHEDULER) {
        i32 c;
        while ((c = usb_kbd_try_getc()) >= 0)
            ui_scheduler_feed_char(c);
        while ((c = uart_try_getc()) >= 0)
            ui_scheduler_feed_char(c);
        return;
    }

    if (ui_mode != UI_MODE_PROC_MANAGER)
        return;

    i32 c;
    while ((c = usb_kbd_try_getc()) >= 0) {
        if (c == 'k' || c == 'K') {
            struct proc_ui_entry snap[UI_SNAPSHOT_MAX];
            u32 n = proc_snapshot(snap, UI_SNAPSHOT_MAX);
            if (n && ui_selected < n && proc_kill_pid(snap[ui_selected].pid, 0xFFFF1000U)) {
                ui_status_code = 1;
            } else {
                ui_status_code = -1;
            }
            ui_last_render = 0;
        } else if (c == 'l' || c == 'L') {
            ui_launch_idx++;
            if (ui_launch_idx >= (i32)(sizeof(ui_launch_candidates)/sizeof(ui_launch_candidates[0])))
                ui_launch_idx = 0;
            u32 core = (ui_launch_idx % 3 == 0) ? CORE_USERM :
                       (ui_launch_idx % 3 == 1) ? CORE_USER0 : CORE_USER1;
            i32 pid = proc_launch_on_core(core, ui_launch_candidates[ui_launch_idx]);
            ui_status_code = (pid > 0) ? 2 : -1;
            ui_last_render = 0;
        } else if (c == 'c' || c == 'C') {
            ui_mode = UI_MODE_CONSOLE;
            ui_console_len = 0;
            fb_clear(0x00000000);
            fb_set_color(UI_SHELL_TEXT_COLOR, 0x00000000);
            ui_console_write("PIOS F3 Console (serial + HDMI)\n");
            ui_console_write("Type 'help' for commands.\n");
            ui_console_prompt();
            ui_last_render = 1;
            return;
        } else if (c == 'j' || c == 'J') {
            ui_selected++;
            ui_last_render = 0;
        } else if ((c == 'u' || c == 'U') && ui_selected > 0) {
            ui_selected--;
            ui_last_render = 0;
        }
    }
}

/* ---- Core entry points ---- */

/* Core 0: Kernel services + network */
NORETURN void core0_main(void) {
    struct core_env *env = core_env_of(CORE_NET);
    ui_mode = UI_MODE_NONE;
    ui_selected = 0;
    ui_last_render = 0;
    ui_launch_idx = -1;
    ui_status_code = 0;
    for (;;) {
        net_poll();
        disk_handle_request(CORE_USERM);
        disk_handle_request(CORE_USER0);
        disk_handle_request(CORE_USER1);
        walfs_handle_fifo(CORE_NET);
        walfs_handle_fifo(CORE_USERM);
        walfs_handle_fifo(CORE_USER0);
        walfs_handle_fifo(CORE_USER1);
        workq_drain(8);
        ui_handle_keys();
        ui_batch_tick();
        if (ui_mode != UI_MODE_NONE) {
            u64 now = timer_ticks();
            if (ui_last_render == 0 || (now - ui_last_render) >= 150) {
                if (ui_mode == UI_MODE_PROC_VIEW)
                    ui_render_process_view();
                else if (ui_mode == UI_MODE_PROC_MANAGER)
                    ui_render_process_manager();
                else if (ui_mode == UI_MODE_SCHEDULER)
                    ui_render_scheduler();
                ui_last_render = now;
            }
        }
        env->poll_count++;
    }
}

/* Disk service loop helper (service core is CORE_DISK, currently core 0) */
static void disk_handle_request(u32 from_core) {
    struct fifo_msg msg;
    struct fifo_msg reply;
    /* Per-core block buffer in core 1's private RAM */
    static u8 block_buf[SD_BLOCK_SIZE] ALIGNED(64);

    if (!fifo_pop(CORE_DISK, from_core, &msg))
        return;

    reply.tag = msg.tag;
    reply.buffer = msg.buffer;
    reply.length = SD_BLOCK_SIZE;

    switch (msg.type) {
    case MSG_DISK_READ:
        if (sd_read_block(msg.param, block_buf)) {
            simd_memcpy((void *)(usize)msg.buffer, block_buf, SD_BLOCK_SIZE);
            reply.type   = MSG_DISK_DONE;
            reply.status = 0;
        } else {
            reply.type   = MSG_DISK_ERROR;
            reply.status = 1;
        }
        fifo_push(CORE_DISK, from_core, &reply);
        break;

    case MSG_DISK_WRITE:
        simd_memcpy(block_buf, (void *)(usize)msg.buffer, SD_BLOCK_SIZE);
        if (sd_write_block(msg.param, block_buf)) {
            reply.type   = MSG_DISK_DONE;
            reply.status = 0;
        } else {
            reply.type   = MSG_DISK_ERROR;
            reply.status = 1;
        }
        fifo_push(CORE_DISK, from_core, &reply);
        break;

    default:
        break;
    }
}

NORETURN void core1_main(void) {
    core_env_init(CORE_USERM);
    proc_init();
    timer_init(PROC_PREEMPT_TIMER_HZ);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* Core 2: User core 0 - process scheduler */
NORETURN void core2_main(void) {
    core_env_init(CORE_USER0);
    proc_init();
    timer_init(PROC_PREEMPT_TIMER_HZ);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* Core 3: User core 1 - process scheduler */
NORETURN void core3_main(void) {
    core_env_init(CORE_USER1);
    proc_init();
    timer_init(PROC_PREEMPT_TIMER_HZ);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* ---- Boot diagnostics display ---- */

static void print_ip(u32 ip) {
    fb_printf("%d.%d.%d.%d",
        (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
        (ip >> 8) & 0xFF, ip & 0xFF);
}

static void boot_diag(void) {
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("PIOS v0.2 - Pi 5 Bare Metal Microkernel\n");

    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("========================================\n\n");

    fb_set_color(0x00FF9900, 0x00000000);

    fb_printf("Core 0: Kernel/Net  [16MB @ 0x%x]\n", CORE0_RAM_BASE);
    fb_printf("Core 1: User        [16MB @ 0x%x]\n", CORE1_RAM_BASE);
    fb_printf("Core 2: User        [16MB @ 0x%x]\n", CORE2_RAM_BASE);
    fb_printf("Core 3: User        [16MB @ 0x%x]\n\n", CORE3_RAM_BASE);

    /* Network */
    u8 mac[6];
    genet_get_mac(mac);
    fb_printf("NET:  GENET v5 + NEON checksum\n");
    fb_printf("  IP:   ");
    print_ip(MY_IP);
    fb_printf(" / ");
    print_ip(MY_MASK);
    fb_printf("\n  GW:   ");
    print_ip(MY_GW);
    fb_printf("\n  PHY:  %s\n", genet_link_up() ? "Link UP" : "Link DOWN");
    fb_printf("  Mode: HARDENED (no ARP/TCP/DHCP/frag)\n");
    fb_printf("  ICMP: rate-limited 10/sec\n\n");

    /* SD */
    const sd_card_t *sd = sd_get_card_info();
    if (sd->type) {
        fb_printf("DISK: %s raw block (no FS)\n\n",
            sd->type == 2 ? "SDHC/SDXC" : "SDSC");
    } else {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_printf("DISK: NOT DETECTED\n\n");
        fb_set_color(0x00FF9900, 0x00000000);
    }

    fb_printf("FIFO: 12ch SPSC  depth=%u  msg=%u bytes\n",
              FIFO_CAPACITY, FIFO_MSG_SIZE);
    fb_printf("SIMD: NEON memcpy/zero/checksum + CRC32\n\n");

    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("System ready. Cores launching.\n");
    fb_set_color(0x00FF9900, 0x00000000);
}

/* ---- Main kernel entry (runs on core 0) ---- */

void kernel_main(void) {
    bool usb_ok = false;
    bool fb_ok = false;
    bool sd_ok = false;
    bool walfs_ok = false;
    bool genet_ok = false;

    /* 1. Debug serial */
    uart_init();
    uart_puts("\n\nPIOS v0.3 booting...\n");
    el2_init();
    uart_puts("[el2] boot EL=");
    uart_hex(el2_boot_el());
    uart_puts(el2_active() ? " (EL2 host active)\n" : " (running in EL1)\n");
    u64 cap_n = 0;
    if (el2_hvc_call(EL2_HVC_CAPSULE_COUNT, 0, 0, 0, 0, &cap_n) == 0) {
        uart_puts("[el2] capsule descriptors=");
        uart_hex(cap_n);
        uart_puts("\n");
    }

    /* 2. Exception vectors + GIC */
    exception_init();
    gic_init();
    uart_puts("[kernel] Exceptions + GIC ready\n");

    /* 3. MMU — identity map, enables caches */
    mmu_init();

    /* 3b. Boot integrity is verified and armed after WALFS + Picowal become available. */

    /* 4. Timer — 1000 Hz tick */
    timer_init(1000);

    /* 5. Unmask IRQs on core 0 (DAIF.I clear) */
    __asm__ volatile("msr daifclr, #2");

    /* 6. DMA engine */
    dma_init();

    /* 7. PCIe Root Complex + RP1 southbridge */
    if (pcie_init()) {
        if (rp1_init()) {
            rp1_clk_init();
            rp1_gpio_init();
            usb_storage_register();
            usb_kbd_register();
            usb_ok = usb_init();
        }
    }

    /* 8. HDMI framebuffer (1280x720) */
    if (fb_init(1280, 720)) {
        fb_ok = true;
        uart_puts("[fb] Framebuffer OK\n");
    } else {
        uart_puts("[fb] Framebuffer FAILED\n");
    }

    /* 8. Inter-core FIFOs */
    fifo_init_all();
    uart_puts("[fifo] Init OK\n");
    ipc_queue_init();
    ipc_stream_init();
    ipc_proc_init();
    pipe_init();
    uart_puts("[ipc] In-memory IPC ready\n");

    /* 9. SD card - raw block access */
    sd_ok = sd_init();
    if (!sd_ok)
        uart_puts("[sd] SD init FAILED (continuing)\n");
    else {
        bcache_init();
        bcache_pin(0);
        walfs_ok = walfs_init();
        if (walfs_ok) {
            principal_init();
            if (!picowal_db_init())
                exception_pisod("Picowal init failed", 5, 0x33, 0, 0, 0);
            boot_policy_verify_or_seed();
            u64 el1_s = 0;
            u32 el1_len = 0;
            boot_measurements(NULL, NULL, &el1_s, &el1_len);
            u64 ok = 0;
            if (el2_hvc_call(EL2_HVC_BOOT_INTEGRITY_SET, el1_s, el1_len,
                             boot_el1_expected_hash, boot_el2_expected_hash, &ok) != 0 || ok != 0)
                exception_pisod("Boot integrity arm failed", 5, 0x3B, 0, 0, 0);
        }
    }

    /* 10. Ethernet MAC */
    genet_ok = genet_init();
    if (!genet_ok)
        uart_puts("[genet] GENET init FAILED (continuing)\n");

    /* 11. Network stack (static IP, static neighbor, NO ARP) */
    net_init(MY_IP, MY_GW, MY_MASK, MY_GW_MAC);
    ui_cfg_ip = MY_IP;
    ui_cfg_mask = MY_MASK;
    ui_cfg_gw = MY_GW;
    ui_cfg_dns = MY_GW;
    ui_cfg_dhcp = false;
    dns_init(ui_cfg_dns);
    net_udp_subscribe(ui_db_udp_cb);

    /* 12. GPU + Tensor compute */
    tensor_init();

    /* Core 0 environment */
    core_env_init(CORE_NET);
    ksem_init_core();
    workq_init_core();

    /* Module system */
    module_init();

    /* First-boot setup flow (before launching user cores) */
    if (walfs_ok) {
        setup_run(fb_ok, genet_ok, usb_ok);
    } else if (sd_ok) {
        uart_puts("[setup] skipped: WALFS unavailable\n");
    } else {
        uart_puts("[setup] skipped: storage unavailable\n");
    }

    /* 13. Boot diagnostics on HDMI */
    boot_diag();

    /* 14. Start secondary cores (they get EL2→EL1, MMU, VBAR, SP from start.S) */
    uart_puts("[kernel] Starting secondary cores...\n");
    core_start_all();
    uart_puts("[kernel] All cores running. Entering net loop.\n");
    /* 15. Core 0 -> network poll loop (never returns) */
    core0_main();
}
