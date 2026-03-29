/*
 * setup.c - First-boot setup flow
 *
 * Runs once when /etc/setup_done is missing:
 *   - reports network/HDMI/USB+keyboard status
 *   - rotates root away from hardcoded default on first boot
 *   - writes /etc/setup_done marker after minimum criteria are met
 */

#include "setup.h"
#include "uart.h"
#include "fb.h"
#include "genet.h"
#include "usb_kbd.h"
#include "walfs.h"
#include "principal.h"
#include "simd.h"
#include "timer.h"

static bool setup_fb_console;

static inline u64 setup_read_cntvct(void)
{
    u64 v;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static void setup_log(const char *s)
{
    uart_puts(s);
    if (setup_fb_console)
        fb_puts(s);
}

static void setup_log_bool(const char *label, bool ok)
{
    setup_log(label);
    setup_log(ok ? "OK\n" : "NOT READY\n");
}

static u64 setup_next_rand(u64 *state)
{
    u64 x = *state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    return x * 0x2545F4914F6CDD1DULL;
}

static void setup_make_temp_token(char out[17])
{
    static const char hex[] = "0123456789ABCDEF";
    u64 s = setup_read_cntvct() ^ (timer_ticks() << 32) ^ 0xC0DEC0DE5EEDB007ULL;
    for (u32 i = 0; i < 16; i++) {
        if ((i & 3U) == 0)
            s = setup_next_rand(&s);
        out[i] = hex[(s >> ((i & 3U) * 4U)) & 0xF];
    }
    out[16] = '\0';
}

static void setup_print_temp_token(const char token[17])
{
    setup_log("[setup] One-time root token: ");
    setup_log(token);
    setup_log("\n");
    setup_log("[setup] Use this token to authenticate as root, then rotate via principal_set_password.\n");
}

static bool setup_write_marker(void)
{
    static const char marker_data[] = "setup=done\n";
    u64 etc_id = walfs_find("/etc");
    if (!etc_id) {
        etc_id = walfs_create(WALFS_ROOT_INODE, "etc", WALFS_DIR, 0755);
        if (!etc_id)
            return false;
    }

    u64 marker_id = walfs_find("/etc/setup_done");
    if (!marker_id) {
        marker_id = walfs_create(etc_id, "setup_done", WALFS_FILE, 0600);
        if (!marker_id)
            return false;
    }

    return walfs_write(marker_id, 0, marker_data, (u32)(sizeof(marker_data) - 1));
}

bool setup_run(bool fb_available, bool net_ready, bool usb_ready)
{
    setup_fb_console = fb_available;

    if (walfs_find("/etc/setup_done")) {
        setup_log("[setup] setup_done marker found, skipping first-boot flow.\n");
        return true;
    }

    setup_log("[setup] First boot detected (missing /etc/setup_done)\n");
    setup_log_bool("[setup] Network stack: ", net_ready);
    setup_log_bool("[setup] Network link: ", genet_link_up());
    setup_log_bool("[setup] HDMI/framebuffer: ", fb_available);
    setup_log_bool("[setup] USB subsystem: ", usb_ready);
    setup_log_bool("[setup] USB keyboard: ", usb_kbd_available());

    if (!usb_kbd_available()) {
        setup_log("[setup] Keyboard unavailable; continuing non-interactive boot.\n");
        setup_log("[setup] Next step: attach keyboard or serial console and rotate root secret.\n");
    }

    bool principal_ok = principal_root_present();
    setup_log_bool("[setup] Principal store: ", principal_ok);

    if (principal_ok && principal_root_uses_default_secret()) {
        char temp_token[17];
        setup_make_temp_token(temp_token);
        if (principal_set_password("root", temp_token)) {
            setup_log("[setup] Root default secret rotated.\n");
            setup_print_temp_token(temp_token);
        } else {
            principal_ok = false;
            setup_log("[setup] Root secret rotation FAILED.\n");
        }
        simd_zero(temp_token, sizeof(temp_token));
    }

    if (!principal_ok) {
        setup_log("[setup] Setup incomplete: principal root not ready.\n");
        return false;
    }

    if (!setup_write_marker()) {
        setup_log("[setup] Setup incomplete: could not persist /etc/setup_done.\n");
        return false;
    }

    setup_log("[setup] First-boot setup complete.\n");
    return true;
}
