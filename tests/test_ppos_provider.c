#include "ppos_provider.h"
#include "pico_hooks.h"
#include "picowal_db.h"
#include "walfs.h"
#include <stdio.h>
#include <string.h>

#define MOCK_NODE_MAX 32
#define MOCK_CARD_MAX 8

struct mock_node {
    bool live;
    u64 inode;
    u64 parent;
    u32 flags;
    u32 size;
    char path[160];
    u8 data[PPOS_PAGE_MAX_BYTES];
};

struct mock_card {
    bool live;
    u16 pack;
    u32 document;
    const char *text;
};

static struct mock_node g_nodes[MOCK_NODE_MAX];
static struct mock_card g_cards[MOCK_CARD_MAX];
static u64 g_next_inode;
static u64 g_short_read_inode;
static u64 g_stat_fail_inode;
static u32 g_replace_count;
static u32 g_partition_lba;
pv_storage_fn pv_storage_hook;

static int check(bool condition, const char *what)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", what);
        return 1;
    }
    return 0;
}

static struct mock_node *mock_by_inode(u64 inode)
{
    for (u32 i = 0; i < MOCK_NODE_MAX; i++)
        if (g_nodes[i].live && g_nodes[i].inode == inode) return &g_nodes[i];
    return 0;
}

static struct mock_node *mock_by_path(const char *path)
{
    for (u32 i = 0; i < MOCK_NODE_MAX; i++)
        if (g_nodes[i].live && strcmp(g_nodes[i].path, path) == 0)
            return &g_nodes[i];
    return 0;
}

static void mock_reset(void)
{
    memset(g_nodes, 0, sizeof(g_nodes));
    memset(g_cards, 0, sizeof(g_cards));
    g_nodes[0].live = true;
    g_nodes[0].inode = WALFS_ROOT_INODE;
    g_nodes[0].flags = WALFS_DIR;
    g_nodes[0].path[0] = '/';
    g_nodes[0].path[1] = 0;
    g_next_inode = WALFS_ROOT_INODE + 1U;
    g_short_read_inode = 0;
    g_stat_fail_inode = 0;
    g_replace_count = 0;
    g_partition_lba = 0;
}

static void mock_card_add(u16 pack, u32 document, const char *text)
{
    for (u32 i = 0; i < MOCK_CARD_MAX; i++) {
        if (!g_cards[i].live) {
            g_cards[i].live = true;
            g_cards[i].pack = pack;
            g_cards[i].document = document;
            g_cards[i].text = text;
            return;
        }
    }
}

u32 walfs_partition_lba(void)
{
    return g_partition_lba;
}

u64 walfs_find(const char *path)
{
    struct mock_node *node = mock_by_path(path);
    return node ? node->inode : 0;
}

u64 walfs_create(u64 parent_id, const char *name, u32 flags, u32 mode)
{
    (void)mode;
    struct mock_node *parent = mock_by_inode(parent_id);
    if (!parent || !(parent->flags & WALFS_DIR) || !name || !name[0])
        return 0;
    for (u32 i = 0; i < MOCK_NODE_MAX; i++) {
        if (!g_nodes[i].live) {
            struct mock_node *node = &g_nodes[i];
            int n = parent->path[1] ?
                    snprintf(node->path, sizeof(node->path), "%s/%s",
                             parent->path, name) :
                    snprintf(node->path, sizeof(node->path), "/%s", name);
            if (n < 0 || (u32)n >= sizeof(node->path) ||
                mock_by_path(node->path)) return 0;
            node->live = true;
            node->inode = g_next_inode++;
            node->parent = parent_id;
            node->flags = flags;
            return node->inode;
        }
    }
    return 0;
}

bool walfs_replace(u64 inode, const void *data, u32 len)
{
    struct mock_node *node = mock_by_inode(inode);
    if (!node || !(node->flags & WALFS_FILE) ||
        len > sizeof(node->data) || (len && !data)) return false;
    if (len) memcpy(node->data, data, len);
    node->size = len;
    g_replace_count++;
    return true;
}

u32 walfs_read(u64 inode, u64 offset, void *out, u32 len)
{
    struct mock_node *node = mock_by_inode(inode);
    if (!node || !out || offset > node->size) return 0;
    u32 available = node->size - (u32)offset;
    u32 n = len < available ? len : available;
    if (inode == g_short_read_inode && n) n--;
    if (n) memcpy(out, node->data + offset, n);
    return n;
}

bool walfs_delete(u64 inode)
{
    struct mock_node *node = mock_by_inode(inode);
    if (!node) return false;
    node->live = false;
    return true;
}

bool walfs_stat(u64 inode, struct walfs_inode *out)
{
    struct mock_node *node = mock_by_inode(inode);
    if (!node || !out || inode == g_stat_fail_inode) return false;
    memset(out, 0, sizeof(*out));
    out->inode_id = node->inode;
    out->parent_id = node->parent;
    out->flags = node->flags;
    out->size = node->size;
    return true;
}

i32 picowal_db_get(u16 pack, u32 document, void *out, u32 out_len)
{
    for (u32 i = 0; i < MOCK_CARD_MAX; i++) {
        if (g_cards[i].live && g_cards[i].pack == pack &&
            g_cards[i].document == document) {
            u32 n = (u32)strlen(g_cards[i].text);
            if (n > out_len) return PPOS_BUFFER_TOO_SMALL;
            if (n) memcpy(out, g_cards[i].text, n);
            return (i32)n;
        }
    }
    return PPOS_NOT_FOUND;
}

static int query_has(const u8 *page_bytes, u32 page_len, const char *query,
                     u32 expected_document)
{
    struct ppos_page page;
    u32 ids[4], count = 0;
    return ppos_open(page_bytes, page_len, &page) == PPOS_OK &&
           ppos_query(&page, query, (u32)strlen(query), PPOS_QUERY_ANY, 0,
                      ids, 4, &count) == PPOS_OK &&
           count == 1 && ids[0] == expected_document;
}

static int test_persistence(void)
{
    u8 page[PPOS_PAGE_MAX_BYTES];
    u8 valid[PPOS_PAGE_MAX_BYTES];
    u32 page_len = 0, generation = 0;

    mock_reset();
    mock_card_add(7, 3, "alpha beta");
    mock_card_add(7, 12, "beta gamma");
    if (check(ppos_walfs_rebuild(7, 9, 9) == PPOS_OK,
              "rebuild authoritative PicoWAL documents")) return 1;
    if (check(ppos_walfs_load(7, 9, page, sizeof(page), &page_len,
                              &generation) == PPOS_OK &&
              generation == 9 && query_has(page, page_len, "alpha", 3),
              "cold WALFS reload")) return 1;
    memcpy(valid, page, page_len);
    if (check(ppos_walfs_save(7, 9, valid, page_len, 10) == PPOS_CORRUPT,
              "save rejects mismatched generation")) return 1;

    struct mock_node *index =
        mock_by_path("/var/picowal/.ppos/p7/f9/index.ppos");
    if (check(index && index->size == page_len, "index persisted at namespace"))
        return 1;
    index->data[PPOS_HEADER_BYTES] ^= 1U;
    if (check(ppos_walfs_load(7, 9, page, sizeof(page), &page_len,
                              &generation) == PPOS_CORRUPT &&
              page_len == 0 && generation == 0,
              "CRC-corrupt index rejected")) return 1;
    if (check(ppos_walfs_rebuild_if_stale(7, 9, 10) == PPOS_OK &&
              ppos_walfs_load(7, 9, page, sizeof(page), &page_len,
                              &generation) == PPOS_OK &&
              generation == 10 && query_has(page, page_len, "gamma", 12),
              "corrupt index rebuilt at expected generation")) return 1;

    u32 writes = g_replace_count;
    if (check(ppos_walfs_rebuild_if_stale(7, 9, 10) == PPOS_OK &&
              g_replace_count == writes, "current generation is not rewritten"))
        return 1;
    index->size = PPOS_HEADER_BYTES - 1U;
    if (check(ppos_walfs_load(7, 9, page, sizeof(page), &page_len,
                              &generation) == PPOS_CORRUPT,
              "torn index rejected")) return 1;
    if (check(ppos_walfs_rebuild_if_stale(7, 9, 11) == PPOS_OK &&
              ppos_walfs_load(7, 9, page, sizeof(page), &page_len,
                              &generation) == PPOS_OK && generation == 11,
              "torn index rebuilt")) return 1;
    return 0;
}

static int hook(pv_ctx *ctx, int id, int rd, int rs1, int rs2)
{
    return ppos_storage_hook(ctx, id, rd, rs1, rs2);
}

static int test_hooks_and_bounds(void)
{
    static pv_ctx ctx;
    static u8 memory[PICOWAL_DATA_MAX + 1U];
    u8 page[PPOS_PAGE_MAX_BYTES];
    u32 page_len = 0, generation = 0;
    const char *needle = "needle beta";

    mock_reset();
    memset(&ctx, 0, sizeof(ctx));
    ctx.mem = memory;
    ctx.mem_size = sizeof(memory);
    ctx.span_count = 2;
    ctx.span_ptr[1] = 0;
    ctx.span_len[1] = (i32)strlen(needle);
    memcpy(memory, needle, (usize)ctx.span_len[1]);

    if (check(!pv_storage_hook, "storage hook initially absent")) return 1;
    ppos_provider_install();
    if (check(pv_storage_hook == ppos_storage_hook, "storage hook registered"))
        return 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_READY, 0, 0, 0) == 1 &&
              ctx.regs[0] == 0 && ctx.host_status == PPOS_OK,
              "storage readiness without partition")) return 1;
    g_partition_lba = 42;
    if (check(hook(&ctx, PV_HOOK_STORAGE_READY, 0, 0, 0) == 1 &&
              ctx.regs[0] == 1, "storage readiness with partition")) return 1;

    ctx.host_status = PPOS_INVALID;
    ctx.regs[1] = 23;
    if (check(hook(&ctx, PV_HOOK_STORAGE_USEPACK, 0, 1, 0) == 1 &&
              ctx.regs[0] == 1 && ctx.host_status == PPOS_OK,
              "select pack")) return 1;
    ctx.regs[1] = 12;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTFIELD, 0, 1, 0) == 1 &&
              ctx.regs[0] == 12 && ctx.host_status == PPOS_OK,
              "select field")) return 1;
    ctx.regs[1] = PPOS_QUERY_ANY;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTMODE, 0, 1, 0) == 1 &&
              ctx.regs[0] == 1 && ctx.host_status == PPOS_OK,
              "select query mode")) return 1;

    ctx.regs[1] = 4;
    ctx.regs[2] = 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTUPSERT, 0, 1, 2) == 1 &&
              ctx.regs[0] == 1 && ctx.host_status == PPOS_OK,
              "upsert snapshot and rebuild")) return 1;
    if (check(ppos_walfs_load(23, 12, page, sizeof(page), &page_len,
                              &generation) == PPOS_OK &&
              generation == 1 && query_has(page, page_len, "needle", 4),
              "upsert generation one")) return 1;

    ctx.regs[1] = 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTFIND, 0, 1, 0) == 1 &&
              ctx.regs[0] == 1 && ctx.host_status == PPOS_OK,
              "full-text find")) return 1;
    ctx.regs[1] = 0;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTRESULT, 0, 1, 0) == 1 &&
              ctx.regs[0] == 4 && ctx.host_status == PPOS_OK,
              "full-text result")) return 1;

    ctx.span_count = PV_MAX_SPANS + 1;
    ctx.regs[1] = 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTFIND, 0, 1, 0) == 1 &&
              ctx.regs[0] == 0 && ctx.host_status == PPOS_INVALID,
              "invalid span table rejected")) return 1;
    ctx.span_count = 2;
    ctx.regs[1] = 0;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTRESULT, 0, 1, 0) == 1 &&
              ctx.regs[0] == -1 && ctx.host_status == PPOS_NOT_FOUND,
              "failed find does not expose stale results")) return 1;

    struct mock_node *source =
        mock_by_path("/var/picowal/.ppos/p23/f12/d4.txt");
    if (check(source != 0, "upsert text snapshot persisted")) return 1;
    g_short_read_inode = source->inode;
    if (check(ppos_walfs_rebuild(23, 12, 99) == PPOS_IO_ERROR,
              "short source read rejected")) return 1;
    g_short_read_inode = 0;
    g_stat_fail_inode = source->inode;
    if (check(ppos_walfs_rebuild(23, 12, 99) == PPOS_IO_ERROR,
              "source stat failure rejected")) return 1;
    g_stat_fail_inode = 0;

    ctx.regs[1] = 4;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTDELETE, 0, 1, 0) == 1 &&
              ctx.regs[0] == 1 && ctx.host_status == PPOS_OK,
              "delete snapshot and rebuild")) return 1;
    if (check(ppos_walfs_load(23, 12, page, sizeof(page), &page_len,
                              &generation) == PPOS_OK && generation == 2,
              "delete increments generation")) return 1;

    ctx.regs[1] = (i32)PICOWAL_CARD_MAX + 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_USEPACK, 0, 1, 0) == 1 &&
              ctx.regs[0] == 0 && ctx.host_status == PPOS_INVALID,
              "invalid pack rejected")) return 1;
    ctx.regs[1] = 23;
    (void)hook(&ctx, PV_HOOK_STORAGE_USEPACK, 0, 1, 0);
    ctx.regs[1] = 99;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTMODE, 0, 1, 0) == 1 &&
              ctx.regs[0] == 0 && ctx.host_status == PPOS_UNSUPPORTED,
              "unsupported query mode")) return 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTFIND, PV_NUM_REGS, 1, 0) == 1 &&
              ctx.host_status == PPOS_INVALID, "invalid destination rejected"))
        return 1;

    ctx.regs[1] = 4;
    ctx.regs[2] = 1;
    ctx.span_len[1] = PICOWAL_DATA_MAX + 1;
    if (check(hook(&ctx, PV_HOOK_STORAGE_FULLTEXTUPSERT, 0, 1, 2) == 1 &&
              ctx.regs[0] == 0 && ctx.host_status == PPOS_BUFFER_TOO_SMALL,
              "oversized upsert rejected")) return 1;
    return 0;
}

int main(void)
{
    if (test_persistence()) return 1;
    if (test_hooks_and_bounds()) return 1;
    printf("PPOS provider: WALFS reload/rebuild, generations, hooks, bounds PASS\n");
    return 0;
}
