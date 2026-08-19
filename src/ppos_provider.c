#include "ppos_provider.h"
#include "pico_hooks.h"
#include "picowal_db.h"
#include "walfs.h"

#define PPOS_WALFS_ROOT "/var/picowal/.ppos"
#define PPOS_DOC_CAP    PICOWAL_DATA_MAX

struct ppos_path_ctx {
    u16 pack;
    u16 field;
};

static u8 g_ppos_page[PPOS_PAGE_MAX_BYTES];
static u8 g_ppos_doc[PPOS_DOC_CAP];
static u32 g_ppos_results[PPOS_MAX_RESULTS];
static u32 g_ppos_result_count;
static u16 g_ppos_pack;
static u16 g_ppos_field;
static u32 g_ppos_mode;
static bool g_ppos_installed;

static u32 ppos_strlen(const char *s)
{
    u32 n = 0;
    if (!s) return 0;
    while (s[n]) n++;
    return n;
}

static u32 ppos_dec(char *out, u32 value)
{
    char reverse[12];
    u32 n = 0;
    if (value == 0) {
        out[0] = '0';
        out[1] = 0;
        return 1;
    }
    while (value) {
        reverse[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    for (u32 i = 0; i < n; i++) out[i] = reverse[n - i - 1U];
    out[n] = 0;
    return n;
}

static bool ppos_path(char *out, u32 cap, u16 pack, u16 field,
                      u32 document, bool is_document)
{
    const char *prefix = PPOS_WALFS_ROOT "/p";
    u32 p = 0;
    while (*prefix) {
        if (p + 1U >= cap) return false;
        out[p++] = *prefix++;
    }
    p += ppos_dec(out + p, pack);
    if (p + 3U >= cap) return false;
    out[p++] = '/';
    out[p++] = 'f';
    p += ppos_dec(out + p, field);
    if (is_document) {
        if (p + 3U >= cap) return false;
        out[p++] = '/';
        out[p++] = 'd';
        p += ppos_dec(out + p, document);
        if (p + 4U >= cap) return false;
        out[p++] = '.';
        out[p++] = 't';
        out[p++] = 'x';
        out[p++] = 't';
    } else {
        const char *name = "/index.ppos";
        while (*name) {
            if (p + 1U >= cap) return false;
            out[p++] = *name++;
        }
    }
    out[p] = 0;
    return true;
}

static bool ppos_ensure_dir(const char *path)
{
    if (!path || !path[0]) return false;
    if (walfs_find(path)) return true;
    u32 len = ppos_strlen(path);
    if (len < 2 || len >= 160U) return false;
    char parent[160];
    char name[64];
    u32 slash = len;
    while (slash > 0 && path[slash - 1U] != '/') slash--;
    if (slash == 0 || len - slash >= sizeof(name)) return false;
    if (slash == 1U) {
        parent[0] = '/';
        parent[1] = 0;
    } else {
        for (u32 i = 0; i < slash - 1U; i++) parent[i] = path[i];
        parent[slash - 1U] = 0;
    }
    for (u32 i = slash; i < len; i++) name[i - slash] = path[i];
    name[len - slash] = 0;
    if (!ppos_ensure_dir(parent)) return false;
    u64 parent_id = walfs_find(parent);
    return parent_id && walfs_create(parent_id, name, WALFS_DIR, 0755);
}

static bool ppos_ensure_index_dirs(u16 pack, u16 field)
{
    char path[160];
    if (!ppos_ensure_dir("/var")) return false;
    if (!ppos_ensure_dir("/var/picowal")) return false;
    if (!ppos_ensure_dir(PPOS_WALFS_ROOT)) return false;
    if (!ppos_path(path, sizeof(path), pack, field, 0, false)) return false;
    u32 n = ppos_strlen(path);
    while (n && path[n - 1U] != '/') n--;
    if (!n) return false;
    path[n - 1U] = 0;
    return ppos_ensure_dir(path);
}

static i32 ppos_read_source(void *opaque, u32 document_id,
                            u8 *out, u32 cap)
{
    struct ppos_path_ctx *ctx = (struct ppos_path_ctx *)opaque;
    char path[160];
    if (!ctx || !out || !cap ||
        !ppos_path(path, sizeof(path), ctx->pack, ctx->field,
                   document_id, true))
        return PPOS_INVALID;
    u64 inode = walfs_find(path);
    if (!inode) {
        /* Rebuilds also accept the authoritative PicoWAL card payload when
         * no dedicated text snapshot exists for this document. */
        i32 n = picowal_db_get(ctx->pack, document_id, out, cap);
        return n < 0 ? PPOS_NOT_FOUND : n;
    }
    struct walfs_inode stat;
    if (!walfs_stat(inode, &stat) || stat.size > cap)
        return stat.size > cap ? PPOS_BUFFER_TOO_SMALL : PPOS_IO_ERROR;
    u32 n = walfs_read(inode, 0, out, cap);
    return (i32)n;
}

static i32 ppos_save_source(u16 pack, u16 field, u32 document,
                            const void *data, u32 len)
{
    if (len > PPOS_DOC_CAP || (len && !data)) return PPOS_BUFFER_TOO_SMALL;
    if (!ppos_ensure_index_dirs(pack, field)) return PPOS_IO_ERROR;
    char path[160];
    if (!ppos_path(path, sizeof(path), pack, field, document, true))
        return PPOS_INVALID;
    u64 inode = walfs_find(path);
    if (!inode) {
        char parent[160];
        if (!ppos_path(parent, sizeof(parent), pack, field, 0, false))
            return PPOS_INVALID;
        u32 n = ppos_strlen(parent);
        while (n && parent[n - 1U] != '/') n--;
        if (!n) return PPOS_INVALID;
        parent[n - 1U] = 0;
        u64 parent_id = walfs_find(parent);
        if (!parent_id) return PPOS_IO_ERROR;
        const char *base = path;
        u32 slash = ppos_strlen(path);
        while (slash && path[slash - 1U] != '/') slash--;
        base = path + slash;
        inode = walfs_create(parent_id, base, WALFS_FILE, 0644);
        if (!inode) return PPOS_IO_ERROR;
    }
    return walfs_replace(inode, data, len) ? PPOS_OK : PPOS_IO_ERROR;
}

static i32 ppos_delete_source(u16 pack, u16 field, u32 document)
{
    char path[160];
    if (!ppos_path(path, sizeof(path), pack, field, document, true))
        return PPOS_INVALID;
    u64 inode = walfs_find(path);
    if (!inode) return PPOS_NOT_FOUND;
    return walfs_delete(inode) ? PPOS_OK : PPOS_IO_ERROR;
}

i32 ppos_walfs_save(u16 pack, u16 field, const void *page, u32 page_len,
                    u32 generation)
{
    if (!page || page_len < PPOS_HEADER_BYTES ||
        page_len > PPOS_PAGE_MAX_BYTES) return PPOS_INVALID;
    struct ppos_page checked;
    i32 rc = ppos_open(page, page_len, &checked);
    if (rc != PPOS_OK || checked.generation != generation) return PPOS_CORRUPT;
    if (!ppos_ensure_index_dirs(pack, field)) return PPOS_IO_ERROR;
    char path[160];
    if (!ppos_path(path, sizeof(path), pack, field, 0, false))
        return PPOS_INVALID;
    u64 inode = walfs_find(path);
    if (!inode) {
        u32 n = ppos_strlen(path);
        while (n && path[n - 1U] != '/') n--;
        if (!n) return PPOS_INVALID;
        path[n - 1U] = 0;
        u64 parent = walfs_find(path);
        if (!parent) return PPOS_IO_ERROR;
        inode = walfs_create(parent, "index.ppos", WALFS_FILE, 0644);
        if (!inode) return PPOS_IO_ERROR;
    }
    return walfs_replace(inode, page, page_len) ? PPOS_OK : PPOS_IO_ERROR;
}

i32 ppos_walfs_load(u16 pack, u16 field, void *page, u32 page_cap,
                    u32 *page_len, u32 *generation)
{
    if (page_len) *page_len = 0;
    if (generation) *generation = 0;
    if (!page || page_cap < PPOS_PAGE_MAX_BYTES || !page_len)
        return PPOS_INVALID;
    char path[160];
    if (!ppos_path(path, sizeof(path), pack, field, 0, false))
        return PPOS_INVALID;
    u64 inode = walfs_find(path);
    if (!inode) return PPOS_NOT_FOUND;
    struct walfs_inode stat;
    if (!walfs_stat(inode, &stat) || stat.size < PPOS_HEADER_BYTES ||
        stat.size > page_cap) return PPOS_CORRUPT;
    u32 n = walfs_read(inode, 0, page, (u32)stat.size);
    if (n != (u32)stat.size) return PPOS_IO_ERROR;
    struct ppos_page opened;
    i32 rc = ppos_open(page, n, &opened);
    if (rc != PPOS_OK) return rc;
    *page_len = n;
    if (generation) *generation = opened.generation;
    return PPOS_OK;
}

i32 ppos_walfs_rebuild(u16 pack, u16 field, u32 generation)
{
    struct ppos_path_ctx path_ctx = { pack, field };
    struct ppos_source source = {
        ppos_read_source, &path_ctx, g_ppos_doc, sizeof(g_ppos_doc),
        PPOS_MAX_DOCUMENTS
    };
    u32 len = 0;
    i32 rc = ppos_build(g_ppos_page, sizeof(g_ppos_page), generation,
                        &source, &len);
    if (rc != PPOS_OK) return rc;
    return ppos_walfs_save(pack, field, g_ppos_page, len, generation);
}

i32 ppos_walfs_rebuild_if_stale(u16 pack, u16 field, u32 expected_generation)
{
    u32 len = 0, generation = 0;
    i32 rc = ppos_walfs_load(pack, field, g_ppos_page,
                             sizeof(g_ppos_page), &len, &generation);
    if (rc == PPOS_OK && generation == expected_generation) return PPOS_OK;
    if (expected_generation == 0) expected_generation = 1;
    return ppos_walfs_rebuild(pack, field, expected_generation);
}

static i32 ppos_current_generation(u16 pack, u16 field)
{
    u32 len = 0, generation = 0;
    i32 rc = ppos_walfs_load(pack, field, g_ppos_page,
                             sizeof(g_ppos_page), &len, &generation);
    if (rc == PPOS_OK) return generation >= 0xFFFFFFFFU ? 1 : generation + 1U;
    return 1;
}

static i32 ppos_ensure_page(u16 pack, u16 field)
{
    u32 len = 0, generation = 0;
    i32 rc = ppos_walfs_load(pack, field, g_ppos_page,
                             sizeof(g_ppos_page), &len, &generation);
    if (rc == PPOS_OK) return PPOS_OK;
    return ppos_walfs_rebuild(pack, field, ppos_current_generation(pack, field));
}

static bool ppos_span(pv_ctx *ctx, int handle, const u8 **ptr, u32 *len)
{
    if (!ctx || handle <= 0 || handle >= ctx->span_count ||
        !ptr || !len || ctx->span_len[handle] < 0) return false;
    u64 p = ctx->span_ptr[handle];
    u64 n = (u32)ctx->span_len[handle];
    if (!ctx->mem || p > (u64)ctx->mem_size || n > (u64)ctx->mem_size - p)
        return false;
    *ptr = ctx->mem + p;
    *len = (u32)n;
    return true;
}

int ppos_storage_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (!ctx) return 0;
    if (hook == PV_HOOK_STORAGE_READY) {
        ctx->regs[rd] = walfs_partition_lba() ? 1 : 0;
        return 1;
    }
    if (hook == PV_HOOK_STORAGE_USEPACK) {
        if (ctx->regs[rs1] < 0 ||
            (u32)ctx->regs[rs1] > (u32)PICOWAL_CARD_MAX) {
            ctx->regs[rd] = 0;
            ctx->host_status = PPOS_INVALID;
            return 1;
        }
        g_ppos_pack = (u16)ctx->regs[rs1];
        ctx->regs[rd] = 1;
        return 1;
    }
    if (hook == PV_HOOK_STORAGE_FULLTEXTFIELD) {
        if (ctx->regs[rs1] < 0 || ctx->regs[rs1] > 0xFFFF) {
            ctx->regs[rd] = 0;
            ctx->host_status = PPOS_INVALID;
            return 1;
        }
        g_ppos_field = (u16)ctx->regs[rs1];
        ctx->regs[rd] = g_ppos_field;
        return 1;
    }
    if (hook == PV_HOOK_STORAGE_FULLTEXTMODE) {
        g_ppos_mode = (u32)ctx->regs[rs1];
        ctx->regs[rd] = g_ppos_mode <= PPOS_QUERY_NEAR ? 1 : 0;
        ctx->host_status = g_ppos_mode <= PPOS_QUERY_NEAR ? PPOS_OK : PPOS_UNSUPPORTED;
        return 1;
    }
    if (hook < PV_HOOK_STORAGE_FULLTEXTUPSERT ||
        hook > PV_HOOK_STORAGE_FULLTEXTRESULT) return 0;

    if (hook == PV_HOOK_STORAGE_FULLTEXTRESULT) {
        i32 index = ctx->regs[rs1];
        ctx->regs[rd] = index >= 0 && (u32)index < g_ppos_result_count ?
                        (i32)g_ppos_results[index] : -1;
        ctx->host_status = index >= 0 && (u32)index < g_ppos_result_count ?
                           PPOS_OK : PPOS_NOT_FOUND;
        return 1;
    }
    if (ctx->regs[rs1] < 0 || (u32)ctx->regs[rs1] >= PPOS_MAX_DOCUMENTS) {
        ctx->regs[rd] = 0;
        ctx->host_status = PPOS_INVALID;
        return 1;
    }
    u32 document = (u32)ctx->regs[rs1];
    i32 rc;
    if (hook == PV_HOOK_STORAGE_FULLTEXTUPSERT) {
        const u8 *text = 0;
        u32 text_len = 0;
        if (!ppos_span(ctx, ctx->regs[rs2], &text, &text_len)) {
            ctx->regs[rd] = 0;
            ctx->host_status = PPOS_INVALID;
            return 1;
        }
        rc = ppos_save_source(g_ppos_pack, g_ppos_field, document,
                              text, text_len);
        if (rc == PPOS_OK)
            rc = ppos_walfs_rebuild(g_ppos_pack, g_ppos_field,
                                    ppos_current_generation(g_ppos_pack,
                                                             g_ppos_field));
        ctx->regs[rd] = rc == PPOS_OK ? 1 : 0;
        ctx->host_status = rc;
        return 1;
    }
    if (hook == PV_HOOK_STORAGE_FULLTEXTDELETE) {
        rc = ppos_delete_source(g_ppos_pack, g_ppos_field, document);
        if (rc == PPOS_NOT_FOUND) rc = PPOS_OK;
        if (rc == PPOS_OK)
            rc = ppos_walfs_rebuild(g_ppos_pack, g_ppos_field,
                                    ppos_current_generation(g_ppos_pack,
                                                             g_ppos_field));
        ctx->regs[rd] = rc == PPOS_OK ? 1 : 0;
        ctx->host_status = rc;
        return 1;
    }

    const u8 *query = 0;
    u32 query_len = 0;
    if (!ppos_span(ctx, ctx->regs[rs1], &query, &query_len)) {
        ctx->regs[rd] = 0;
        ctx->host_status = PPOS_INVALID;
        return 1;
    }
    rc = ppos_ensure_page(g_ppos_pack, g_ppos_field);
    if (rc == PPOS_OK) {
        struct ppos_page page;
        u32 len = 0, generation = 0;
        rc = ppos_walfs_load(g_ppos_pack, g_ppos_field, g_ppos_page,
                             sizeof(g_ppos_page), &len, &generation);
        if (rc == PPOS_OK) {
            rc = ppos_open(g_ppos_page, len, &page);
            if (rc == PPOS_OK)
                rc = ppos_query(&page, query, query_len, g_ppos_mode, 1,
                                g_ppos_results, PPOS_MAX_RESULTS,
                                &g_ppos_result_count);
        }
    }
    ctx->regs[rd] = rc == PPOS_OK || rc == PPOS_PARTIAL ?
                    (i32)g_ppos_result_count : 0;
    ctx->host_status = rc;
    return 1;
}

void ppos_provider_install(void)
{
    if (!g_ppos_installed) {
        pv_storage_hook = ppos_storage_hook;
        g_ppos_installed = true;
    }
}
