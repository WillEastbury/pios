/*
 * principal.c - User / principal identity and capability system
 *
 * Persistent principal store in WALFS (/etc/principals).
 * On first boot, creates root principal with default credentials.
 * Per-core current principal tracked in a static array (no locks).
 */

#include "principal.h"
#include "walfs.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"

/* Constant-time comparison to prevent timing side-channel attacks */
static bool ct_eq(const u8 *a, const u8 *b, u32 len) {
    u8 diff = 0;
    for (u32 i = 0; i < len; i++) diff |= a[i] ^ b[i];
    return diff == 0;
}

static struct principal principals[PRINCIPAL_MAX];
static u32 principal_count;
static u32 current_principal[4];  /* one slot per core */

/* ---- Internal helpers ---- */

/* Iterate CRC32C 100K times with username salt for strengthened hash.
 * Still only 32 bits — real fix requires SHA-256 (issue #19).
 * 100K iterations at ~8B/cycle ≈ 40µs per attempt on A76. */
static void hash_pass_salted(const char *user, const char *pass, u8 out[4])
{
    u32 h = hw_crc32c(pass, pios_strlen(pass));
    h = hw_crc32c(&h, 4) ^ hw_crc32c(user, pios_strlen(user)); /* salt with username */
    for (u32 i = 0; i < 100000; i++)
        h = hw_crc32c(&h, 4);
    out[0] = (u8)(h);
    out[1] = (u8)(h >> 8);
    out[2] = (u8)(h >> 16);
    out[3] = (u8)(h >> 24);
}

/* Legacy compat wrapper — used during migration */
static void hash_pass(const char *pass, u8 out[4])
{
    hash_pass_salted("", pass, out);
}

/* Write principal table back to /etc/principals */
static bool flush_principals(void)
{
    u64 fid = walfs_find("/etc/principals");
    if (!fid) return false;
    return walfs_write(fid, 0, principals,
                       principal_count * sizeof(struct principal));
}

/* Read password from UART without echo (prints '*' per char) */
static u32 read_secret(char *buf, u32 max)
{
    u32 i = 0;
    while (i < max - 1) {
        char c = uart_getc();
        if (c == '\r' || c == '\n') break;
        if (c == '\b' || c == 127) {
            if (i > 0) { i--; uart_puts("\b \b"); }
            continue;
        }
        buf[i++] = c;
        uart_putc('*');
    }
    buf[i] = 0;
    uart_puts("\r\n");
    return i;
}

/* Find principal by name. Returns index or -1. */
static i32 find_by_name(const char *name)
{
    u32 len = pios_strlen(name) + 1;
    for (u32 i = 0; i < principal_count; i++) {
        if (memcmp(principals[i].name, name, len) == 0)
            return (i32)i;
    }
    return -1;
}

/* ---- Public API ---- */

bool principal_init(void)
{
    simd_zero(principals, sizeof(principals));
    simd_zero(current_principal, sizeof(current_principal));
    principal_count = 0;

    u64 fid = walfs_find("/etc/principals");
    if (fid) {
        u32 n = walfs_read(fid, 0, principals, sizeof(principals));
        principal_count = n / sizeof(struct principal);
        return principal_count > 0;
    }

    /* First boot — create /etc directory and principals file */
    u64 etc_id = walfs_find("/etc");
    if (!etc_id) {
        etc_id = walfs_create(WALFS_ROOT_INODE, "etc", WALFS_DIR, 0755);
        if (!etc_id) return false;
    }

    fid = walfs_create(etc_id, "principals", WALFS_FILE, 0600);
    if (!fid) return false;

    /* Seed with root principal (all capabilities) */
    struct principal *r = &principals[0];
    r->id    = PRINCIPAL_ROOT;
    r->flags = PRINCIPAL_ADMIN | PRINCIPAL_NET | PRINCIPAL_DISK |
               PRINCIPAL_EXEC | PRINCIPAL_IPC;
    memcpy(r->name, "root", 5);
    hash_pass("pios", r->secret_hash);
    principal_count = 1;

    uart_puts("[principal] created root account\r\n");
    return flush_principals();
}

bool principal_auth(const char *name, const char *pass, u32 *id_out)
{
    i32 idx = find_by_name(name);
    if (idx < 0) return false;

    u8 h[4];
    hash_pass(pass, h);
    if (!ct_eq(principals[idx].secret_hash, h, 4))
        return false;

    current_principal[core_id()] = principals[idx].id;
    if (id_out) *id_out = principals[idx].id;
    return true;
}

u32 principal_current(void)
{
    return current_principal[core_id()];
}

u32 principal_current_for(u32 core)
{
    if (core > 3) return PRINCIPAL_ROOT;
    return current_principal[core];
}

void principal_set_current(u32 id)
{
    current_principal[core_id()] = id;
}

bool principal_has_cap(u32 id, u32 cap_flag)
{
    if (id == PRINCIPAL_ROOT) return true;
    for (u32 i = 0; i < principal_count; i++) {
        if (principals[i].id == id) {
            if (cap_flag == PRINCIPAL_IPC &&
                (principals[i].flags & PRINCIPAL_ADMIN))
                return true;
            return (principals[i].flags & cap_flag) != 0;
        }
    }
    return false;
}

bool principal_create(const char *name, const char *pass, u32 flags)
{
    if (principal_count >= PRINCIPAL_MAX) return false;
    if (find_by_name(name) >= 0) return false;
    if (flags & PRINCIPAL_ADMIN) flags |= PRINCIPAL_IPC;

    struct principal *p = &principals[principal_count];
    simd_zero(p, sizeof(*p));
    p->id    = principal_count;
    p->flags = flags;

    u32 len = pios_strlen(name);
    if (len > 31) len = 31;
    memcpy(p->name, name, len);
    p->name[len] = 0;
    hash_pass(pass, p->secret_hash);
    principal_count++;

    return flush_principals();
}

/*
 * Permission model: root (id 0) bypasses all checks.
 * Non-root principals are checked against the "other" permission
 * bits of the inode mode (bits 2-0) since inodes carry no owner field.
 */

bool principal_can_read(u32 principal_id, const struct walfs_inode *inode)
{
    if (principal_id == PRINCIPAL_ROOT) return true;
    return (inode->mode & 0x04) != 0;  /* other-read */
}

bool principal_can_write(u32 principal_id, const struct walfs_inode *inode)
{
    if (principal_id == PRINCIPAL_ROOT) return true;
    return (inode->mode & 0x02) != 0;  /* other-write */
}

bool principal_can_exec(u32 principal_id, const struct walfs_inode *inode)
{
    if (principal_id == PRINCIPAL_ROOT) return true;
    return (inode->mode & 0x01) != 0;  /* other-exec */
}

void principal_login_prompt(void)
{
    char user[32], pass[32];
    u32 tries = 0;

    for (;;) {
        uart_puts("login: ");
        uart_getline(user, sizeof(user));
        uart_puts("password: ");
        read_secret(pass, sizeof(pass));

        /* Scrub password from stack after hashing */
        u32 id;
        if (principal_auth(user, pass, &id)) {
            simd_zero(pass, sizeof(pass));
            uart_puts("authenticated as ");
            uart_puts(user);
            uart_puts("\r\n");
            return;
        }

        simd_zero(pass, sizeof(pass));
        tries++;
        uart_puts("auth failed\r\n");
        if (tries >= 3) {
            uart_puts("locked out 10s\r\n");
            timer_delay_ms(10000);
            tries = 0;
        }
    }
}
