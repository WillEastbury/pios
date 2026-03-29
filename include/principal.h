/*
 * principal.h - User / principal identity and capability system
 *
 * Simple multi-user model for the bare-metal OS.
 * Principals are stored persistently in WALFS at /etc/principals.
 * Authentication uses iterated CRC32C hashing (1000 rounds).
 * Per-core current-principal tracking (no locks needed).
 */

#pragma once
#include "types.h"

#define PRINCIPAL_ADMIN  0x01
#define PRINCIPAL_NET    0x02
#define PRINCIPAL_DISK   0x04
#define PRINCIPAL_EXEC   0x08
#define PRINCIPAL_IPC    0x10
#define PRINCIPAL_MAX    16
#define PRINCIPAL_ROOT   0

struct principal {
    u32 id;
    u8  name[32];
    u8  secret_hash[4];   /* CRC32C iterated hash (stored as 4 bytes) */
    u32 flags;
};

/* Forward declaration for walfs inode */
struct walfs_inode;

bool principal_init(void);
bool principal_auth(const char *name, const char *pass, u32 *id_out);
u32  principal_current(void);
u32  principal_current_for(u32 core);  /* get principal for a specific core */
void principal_set_current(u32 id);
bool principal_has_cap(u32 id, u32 cap_flag);
bool principal_create(const char *name, const char *pass, u32 flags);
bool principal_set_password(const char *name, const char *pass);
bool principal_root_present(void);
bool principal_root_uses_default_secret(void);
bool principal_can_read(u32 principal_id, const struct walfs_inode *inode);
bool principal_can_write(u32 principal_id, const struct walfs_inode *inode);
bool principal_can_exec(u32 principal_id, const struct walfs_inode *inode);
void principal_login_prompt(void);
