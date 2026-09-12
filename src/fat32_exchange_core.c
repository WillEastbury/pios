/*
 * Offline FAT32 root-directory mutation core.  It deliberately has no SD,
 * WALFS, bootstrap, kernel, partition-discovery, or live-fat32 dependency.
 */
#include "types.h"
#include "fat32_exchange_core.h"

#define FAT32_EOC              0x0FFFFFF8U
#define FAT32_BAD              0x0FFFFFF7U
#define FAT32_MASK             0x0FFFFFFFU
#define FAT32_DIR_ENTRY_BYTES  32U

struct dir_ref {
    u32 lba;
    u16 offset;
};

static u16 le16(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static u32 le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
           ((u32)p[3] << 24);
}

static void put_le16(u8 *p, u16 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
}

static void put_le32(u8 *p, u32 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
    p[2] = (u8)(value >> 16);
    p[3] = (u8)(value >> 24);
}

static void bytes_zero(u8 *p, u32 bytes)
{
    u32 i;

    for (i = 0U; i < bytes; i++)
        p[i] = 0U;
}

static void bytes_copy(u8 *dst, const u8 *src, u32 bytes)
{
    u32 i;

    for (i = 0U; i < bytes; i++)
        dst[i] = src[i];
}

static bool bytes_equal(const u8 *a, const u8 *b, u32 bytes)
{
    u32 i;

    for (i = 0U; i < bytes; i++) {
        if (a[i] != b[i])
            return false;
    }
    return true;
}

static enum fat32_exchange_result fault(struct fat32_exchange_volume *volume,
                                        enum fat32_exchange_result result)
{
    volume->faulted = 1U;
    return result;
}

void fat32_exchange_volume_init(struct fat32_exchange_volume *volume)
{
    if (volume)
        *volume = (struct fat32_exchange_volume){0};
}

static enum fat32_exchange_result sector_read(struct fat32_exchange_volume *v,
                                              u32 lba, u8 *out)
{
    if (!v->io.read || !v->io.read(v->io.context, lba, out))
        return fault(v, FAT32_EXCHANGE_IO);
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result sector_write(struct fat32_exchange_volume *v,
                                               u32 lba, const u8 *in)
{
    if (!v->io.write || !v->io.write(v->io.context, lba, in))
        return fault(v, FAT32_EXCHANGE_IO);
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result access_ok(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq)
{
    if (!v || !attachment)
        return FAT32_EXCHANGE_INVALID;
    if (!v->mounted)
        return FAT32_EXCHANGE_INVALID;
    if (v->faulted)
        return FAT32_EXCHANGE_FAULTED;
    if (caller_core != 0U || in_irq)
        return FAT32_EXCHANGE_OWNER;
    if (attachment->identity != v->identity || attachment->epoch != v->epoch ||
        attachment->first_lba != v->first_lba ||
        attachment->block_count != v->block_count)
        return FAT32_EXCHANGE_STALE;
    return FAT32_EXCHANGE_OK;
}

static bool power_of_two(u32 value)
{
    return value != 0U && (value & (value - 1U)) == 0U;
}

static bool lba_in_volume(const struct fat32_exchange_volume *v, u32 lba)
{
    return lba >= v->first_lba && lba - v->first_lba < v->block_count;
}

static bool lba_span_fits(u32 first, u32 block_count, u32 lba, u32 sectors)
{
    u32 relative;

    if (sectors == 0U || block_count == 0U || sectors > block_count ||
        lba < first)
        return false;
    relative = lba - first;
    return relative <= block_count - sectors;
}

static bool lba_range_representable(u32 first, u32 sectors)
{
    return sectors != 0U && first <= ~0U - (sectors - 1U);
}

static enum fat32_exchange_result cluster_lba(
    const struct fat32_exchange_volume *v, u32 cluster, u32 *out)
{
    u32 relative;
    u32 sectors;

    if (!out || cluster < 2U || cluster - 2U >= v->cluster_count)
        return FAT32_EXCHANGE_CORRUPT;
    relative = cluster - 2U;
    if (relative > (~0U / v->sectors_per_cluster))
        return FAT32_EXCHANGE_CORRUPT;
    sectors = relative * v->sectors_per_cluster;
    if (sectors > ~0U - v->data_lba ||
        !lba_span_fits(v->first_lba, v->block_count, v->data_lba + sectors,
                       v->sectors_per_cluster))
        return FAT32_EXCHANGE_CORRUPT;
    *out = v->data_lba + sectors;
    return FAT32_EXCHANGE_OK;
}

static bool fat_value_valid(const struct fat32_exchange_volume *v, u32 value)
{
    return value == 0U || value == FAT32_BAD || value >= FAT32_EOC ||
           (value >= 2U && value <= v->cluster_count + 1U);
}

static enum fat32_exchange_result fat_get(struct fat32_exchange_volume *v,
                                          u32 cluster, u32 *out)
{
    u32 offset;
    u32 sector;
    u32 index;
    enum fat32_exchange_result result;

    if (!out || cluster > v->cluster_count + 1U)
        return FAT32_EXCHANGE_CORRUPT;
    if (cluster > ~0U / 4U)
        return FAT32_EXCHANGE_CORRUPT;
    offset = cluster * 4U;
    sector = offset / FAT32_EXCHANGE_SECTOR_BYTES;
    index = offset % FAT32_EXCHANGE_SECTOR_BYTES;
    if (sector >= v->fat_sectors || index > FAT32_EXCHANGE_SECTOR_BYTES - 4U)
        return FAT32_EXCHANGE_CORRUPT;
    result = sector_read(v, v->fat_lba + sector, v->sector_a);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = sector_read(v, v->fat_lba + v->fat_sectors + sector, v->sector_b);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!bytes_equal(v->sector_a, v->sector_b, FAT32_EXCHANGE_SECTOR_BYTES))
        return fault(v, FAT32_EXCHANGE_FAT_MISMATCH);
    *out = le32(v->sector_a + index) & FAT32_MASK;
    return FAT32_EXCHANGE_OK;
}

static bool fsinfo_valid(const struct fat32_exchange_volume *v, const u8 *sector)
{
    u32 free_count = le32(sector + 488U);
    u32 next_free = le32(sector + 492U);

    return le32(sector) == 0x41615252U &&
           le32(sector + 484U) == 0x61417272U &&
           le32(sector + 508U) == 0xAA550000U &&
           (free_count == ~0U || free_count <= v->cluster_count) &&
           (next_free == ~0U ||
            (next_free >= 2U && next_free <= v->cluster_count + 1U));
}

/*
 * FAT32 permits unknown FSInfo hints.  Once a metadata mutation begins, do
 * not leave a known hint stale; write both hints unknown before any FAT copy.
 */
static enum fat32_exchange_result prepare_mutation(
    struct fat32_exchange_volume *v)
{
    enum fat32_exchange_result result;

    if (!v->fsinfo_needs_unknown)
        return FAT32_EXCHANGE_OK;
    if (v->fsinfo_lba == 0U)
        return fault(v, FAT32_EXCHANGE_CORRUPT);
    result = sector_read(v, v->fsinfo_lba, v->sector_a);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!fsinfo_valid(v, v->sector_a))
        return fault(v, FAT32_EXCHANGE_BPB);
    put_le32(v->sector_a + 488U, ~0U);
    put_le32(v->sector_a + 492U, ~0U);
    result = sector_write(v, v->fsinfo_lba, v->sector_a);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    v->fsinfo_needs_unknown = 0U;
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result fat_set(struct fat32_exchange_volume *v,
                                          u32 cluster, u32 value)
{
    u32 offset;
    u32 sector;
    u32 index;
    u32 old_a;
    u32 old_b;
    enum fat32_exchange_result result;

    if (cluster < 2U || cluster > v->cluster_count + 1U ||
        !(value == 0U || value >= FAT32_EOC ||
          (value >= 2U && value <= v->cluster_count + 1U)))
        return FAT32_EXCHANGE_CORRUPT;
    result = prepare_mutation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    offset = cluster * 4U;
    sector = offset / FAT32_EXCHANGE_SECTOR_BYTES;
    index = offset % FAT32_EXCHANGE_SECTOR_BYTES;
    if (sector >= v->fat_sectors || index > FAT32_EXCHANGE_SECTOR_BYTES - 4U)
        return FAT32_EXCHANGE_CORRUPT;
    result = sector_read(v, v->fat_lba + sector, v->sector_a);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = sector_read(v, v->fat_lba + v->fat_sectors + sector, v->sector_b);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    old_a = le32(v->sector_a + index);
    old_b = le32(v->sector_b + index);
    if (old_a != old_b)
        return fault(v, FAT32_EXCHANGE_FAT_MISMATCH);
    put_le32(v->sector_a + index, (old_a & ~FAT32_MASK) | value);
    put_le32(v->sector_b + index, (old_b & ~FAT32_MASK) | value);
    result = sector_write(v, v->fat_lba + sector, v->sector_a);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    /* A failure here deliberately faults this instance: copy one may persist. */
    return sector_write(v, v->fat_lba + v->fat_sectors + sector, v->sector_b);
}

static enum fat32_exchange_result chain_next(struct fat32_exchange_volume *v,
                                             u32 cluster, u32 *next,
                                             bool *at_end)
{
    u32 value;
    enum fat32_exchange_result result = fat_get(v, cluster, &value);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (value >= FAT32_EOC) {
        *at_end = true;
        *next = 0U;
        return FAT32_EXCHANGE_OK;
    }
    if (value < 2U || value == FAT32_BAD || value > v->cluster_count + 1U)
        return FAT32_EXCHANGE_CORRUPT;
    *at_end = false;
    *next = value;
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result chain_at(struct fat32_exchange_volume *v,
                                           u32 first, u32 index, u32 *out)
{
    u32 current = first;
    u32 i;

    if (first < 2U || first > v->cluster_count + 1U)
        return FAT32_EXCHANGE_CORRUPT;
    for (i = 0U; i < index; i++) {
        u32 next;
        bool end;
        enum fat32_exchange_result result = chain_next(v, current, &next, &end);

        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (end)
            return FAT32_EXCHANGE_CORRUPT;
        current = next;
    }
    *out = current;
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result count_free(struct fat32_exchange_volume *v,
                                             u32 *out)
{
    u32 sector;
    u32 count = 0U;

    if (!out)
        return FAT32_EXCHANGE_INVALID;
    for (sector = 0U; sector < v->fat_sectors; sector++) {
        u32 first_cluster = sector * (FAT32_EXCHANGE_SECTOR_BYTES / 4U);
        u32 cluster;
        u32 last_cluster;
        enum fat32_exchange_result result;

        result = sector_read(v, v->fat_lba + sector, v->sector_a);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        result = sector_read(v, v->fat_lba + v->fat_sectors + sector,
                             v->sector_b);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (!bytes_equal(v->sector_a, v->sector_b, FAT32_EXCHANGE_SECTOR_BYTES))
            return fault(v, FAT32_EXCHANGE_FAT_MISMATCH);
        if (first_cluster > v->cluster_count + 1U)
            break;
        if (first_cluster < 2U)
            first_cluster = 2U;
        last_cluster = sector * (FAT32_EXCHANGE_SECTOR_BYTES / 4U) +
                       (FAT32_EXCHANGE_SECTOR_BYTES / 4U - 1U);
        if (last_cluster > v->cluster_count + 1U)
            last_cluster = v->cluster_count + 1U;
        for (cluster = first_cluster; cluster <= last_cluster; cluster++) {
            u32 value = le32(v->sector_a + (cluster % 128U) * 4U) & FAT32_MASK;

            if (!fat_value_valid(v, value))
                return fault(v, FAT32_EXCHANGE_CORRUPT);
            if (value == 0U)
                count++;
        }
    }
    *out = count;
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result alloc_one(struct fat32_exchange_volume *v,
                                            u32 *out)
{
    u32 i;
    u32 cluster;

    if (v->alloc_hint < 2U || v->alloc_hint > v->cluster_count + 1U)
        v->alloc_hint = 2U;
    cluster = v->alloc_hint;
    for (i = 0U; i < v->cluster_count; i++) {
        u32 value;
        enum fat32_exchange_result result = fat_get(v, cluster, &value);

        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (value == 0U) {
            result = fat_set(v, cluster, FAT32_EOC);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            *out = cluster;
            v->alloc_hint = cluster == v->cluster_count + 1U ? 2U : cluster + 1U;
            return FAT32_EXCHANGE_OK;
        }
        cluster = cluster == v->cluster_count + 1U ? 2U : cluster + 1U;
    }
    return FAT32_EXCHANGE_NO_SPACE;
}

static enum fat32_exchange_result zero_cluster(struct fat32_exchange_volume *v,
                                                u32 cluster)
{
    u32 lba;
    u32 sector;
    enum fat32_exchange_result result = cluster_lba(v, cluster, &lba);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    bytes_zero(v->sector_a, FAT32_EXCHANGE_SECTOR_BYTES);
    for (sector = 0U; sector < v->sectors_per_cluster; sector++) {
        result = sector_write(v, lba + sector, v->sector_a);
        if (result != FAT32_EXCHANGE_OK)
            return result;
    }
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result build_chain(struct fat32_exchange_volume *v,
                                              u32 count, u32 *first_out)
{
    u32 free_count;
    u32 first = 0U;
    u32 last = 0U;
    u32 i;
    enum fat32_exchange_result result;

    if (count == 0U) {
        *first_out = 0U;
        return FAT32_EXCHANGE_OK;
    }
    result = count_free(v, &free_count);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (free_count < count)
        return FAT32_EXCHANGE_NO_SPACE;
    for (i = 0U; i < count; i++) {
        u32 current;

        result = alloc_one(v, &current);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        result = zero_cluster(v, current);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (last != 0U) {
            result = fat_set(v, last, current);
            if (result != FAT32_EXCHANGE_OK)
                return result;
        } else {
            first = current;
        }
        last = current;
    }
    *first_out = first;
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result free_chain(struct fat32_exchange_volume *v,
                                             u32 first)
{
    u32 current = first;
    u32 steps;

    if (first == 0U)
        return FAT32_EXCHANGE_OK;
    for (steps = 0U; steps < v->cluster_count; steps++) {
        u32 next;
        bool end;
        enum fat32_exchange_result result = chain_next(v, current, &next, &end);

        if (result != FAT32_EXCHANGE_OK)
            return result;
        result = fat_set(v, current, 0U);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (end)
            return FAT32_EXCHANGE_OK;
        current = next;
    }
    return fault(v, FAT32_EXCHANGE_CORRUPT);
}

static bool name_char_valid(u8 value)
{
    if (value >= 'A' && value <= 'Z')
        return true;
    if (value >= '0' && value <= '9')
        return true;
    return value == '!' || value == '#' || value == '$' || value == '%' ||
           value == '&' || value == '\'' || value == '(' || value == ')' ||
           value == '-' || value == '@' || value == '^' || value == '_' ||
           value == '`' || value == '{' || value == '}' || value == '~';
}

static enum fat32_exchange_result encode_name(const struct fat32_exchange_name *name,
                                              u8 out[FAT32_EXCHANGE_NAME_BYTES])
{
    u32 dot = ~0U;
    u32 i;
    u32 base;
    u32 ext;

    if (!name || !name->bytes || name->len == 0U || name->len > 12U)
        return FAT32_EXCHANGE_INVALID;
    for (i = 0U; i < name->len; i++) {
        if (name->bytes[i] == '.') {
            if (dot != ~0U)
                return FAT32_EXCHANGE_INVALID;
            dot = i;
        } else if (!name_char_valid(name->bytes[i])) {
            return FAT32_EXCHANGE_INVALID;
        }
    }
    base = dot == ~0U ? name->len : dot;
    ext = dot == ~0U ? 0U : name->len - dot - 1U;
    if (base == 0U || base > 8U || ext > 3U)
        return FAT32_EXCHANGE_INVALID;
    for (i = 0U; i < FAT32_EXCHANGE_NAME_BYTES; i++)
        out[i] = ' ';
    bytes_copy(out, name->bytes, base);
    for (i = 0U; i < ext; i++)
        out[8U + i] = name->bytes[dot + 1U + i];
    return FAT32_EXCHANGE_OK;
}

static bool regular_entry(const u8 *entry)
{
    return entry[0] != 0U && entry[0] != 0xE5U &&
           entry[11] != 0x0FU && (entry[11] & 0x18U) == 0U;
}

static bool active_lfn_entry(const u8 *entry)
{
    return entry[0] != 0U && entry[0] != 0xE5U && entry[11] == 0x0FU;
}

static u32 entry_cluster(const u8 *entry)
{
    return ((u32)le16(entry + 20U) << 16) | le16(entry + 26U);
}

static void entry_set_cluster(u8 *entry, u32 cluster)
{
    put_le16(entry + 20U, (u16)(cluster >> 16));
    put_le16(entry + 26U, (u16)cluster);
}

/*
 * target == NULL means find a reusable root slot.  A deleted entry is saved
 * but not returned before the full directory has been searched for duplicates.
 */
static enum fat32_exchange_result root_find(struct fat32_exchange_volume *v,
                                            const u8 *target, bool free_slot,
                                            struct dir_ref *out)
{
    struct dir_ref reusable = {0};
    bool have_reusable = false;
    u32 cluster = v->root_cluster;
    u32 hops;

    for (hops = 0U; hops < v->cluster_count; hops++) {
        u32 base;
        u32 sector;
        enum fat32_exchange_result result = cluster_lba(v, cluster, &base);

        if (result != FAT32_EXCHANGE_OK)
            return result;
        for (sector = 0U; sector < v->sectors_per_cluster; sector++) {
            u32 offset;

            result = sector_read(v, base + sector, v->sector_a);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            for (offset = 0U; offset < FAT32_EXCHANGE_SECTOR_BYTES;
                 offset += FAT32_DIR_ENTRY_BYTES) {
                u8 *entry = v->sector_a + offset;

                if (entry[0] == 0U) {
                    if (free_slot) {
                        if (have_reusable)
                            *out = reusable;
                        else {
                            out->lba = base + sector;
                            out->offset = (u16)offset;
                        }
                        return FAT32_EXCHANGE_OK;
                    }
                    return FAT32_EXCHANGE_NOT_FOUND;
                }
                if (entry[0] == 0xE5U) {
                    if (free_slot && !have_reusable) {
                        reusable.lba = base + sector;
                        reusable.offset = (u16)offset;
                        have_reusable = true;
                    }
                    continue;
                }
                if (target && regular_entry(entry) &&
                    bytes_equal(entry, target, FAT32_EXCHANGE_NAME_BYTES)) {
                    out->lba = base + sector;
                    out->offset = (u16)offset;
                    return FAT32_EXCHANGE_OK;
                }
            }
        }
        {
            u32 next;
            bool end;

            result = chain_next(v, cluster, &next, &end);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            if (end)
                break;
            cluster = next;
        }
    }
    if (hops == v->cluster_count)
        return FAT32_EXCHANGE_CORRUPT;
    if (free_slot && have_reusable) {
        *out = reusable;
        return FAT32_EXCHANGE_OK;
    }
    return FAT32_EXCHANGE_NOT_FOUND;
}

static enum fat32_exchange_result root_extend(struct fat32_exchange_volume *v,
                                              struct dir_ref *out)
{
    u32 tail = v->root_cluster;
    u32 hops;
    u32 new_cluster;
    u32 lba;
    enum fat32_exchange_result result;

    for (hops = 0U; hops < v->cluster_count; hops++) {
        u32 next;
        bool end;

        result = chain_next(v, tail, &next, &end);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (end)
            break;
        tail = next;
    }
    if (hops == v->cluster_count)
        return fault(v, FAT32_EXCHANGE_CORRUPT);
    result = build_chain(v, 1U, &new_cluster);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = fat_set(v, tail, new_cluster);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = cluster_lba(v, new_cluster, &lba);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    out->lba = lba;
    out->offset = 0U;
    return FAT32_EXCHANGE_OK;
}

/*
 * An LFN chain can cross sector or cluster boundaries.  Scan the actual root
 * order instead of deriving a predecessor LBA, so a short entry capability
 * cannot cause an orphaned LFN chain to be left behind by a mutation.
 */
static enum fat32_exchange_result root_lfn_precedes(
    struct fat32_exchange_volume *v, const struct dir_ref *target,
    bool *out_lfn)
{
    bool prior_lfn = false;
    u32 cluster = v->root_cluster;
    u32 hops;

    if (!target || !out_lfn)
        return FAT32_EXCHANGE_INVALID;
    for (hops = 0U; hops < v->cluster_count; hops++) {
        u32 base;
        u32 sector;
        enum fat32_exchange_result result = cluster_lba(v, cluster, &base);

        if (result != FAT32_EXCHANGE_OK)
            return result;
        for (sector = 0U; sector < v->sectors_per_cluster; sector++) {
            u32 offset;

            result = sector_read(v, base + sector, v->sector_b);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            for (offset = 0U; offset < FAT32_EXCHANGE_SECTOR_BYTES;
                 offset += FAT32_DIR_ENTRY_BYTES) {
                const u8 *entry = v->sector_b + offset;

                if (target->lba == base + sector && target->offset == offset) {
                    *out_lfn = prior_lfn;
                    return FAT32_EXCHANGE_OK;
                }
                if (entry[0] == 0U)
                    return FAT32_EXCHANGE_STALE;
                prior_lfn = active_lfn_entry(entry);
            }
        }
        {
            u32 next;
            bool end;

            result = chain_next(v, cluster, &next, &end);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            if (end)
                return FAT32_EXCHANGE_STALE;
            cluster = next;
        }
    }
    return fault(v, FAT32_EXCHANGE_CORRUPT);
}

static enum fat32_exchange_result read_ref(struct fat32_exchange_volume *v,
                                           const struct dir_ref *ref,
                                           u8 **entry)
{
    enum fat32_exchange_result result;

    if (!ref || ref->offset > FAT32_EXCHANGE_SECTOR_BYTES - FAT32_DIR_ENTRY_BYTES ||
        (ref->offset % FAT32_DIR_ENTRY_BYTES) != 0U || !lba_in_volume(v, ref->lba))
        return FAT32_EXCHANGE_CORRUPT;
    result = sector_read(v, ref->lba, v->sector_a);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    *entry = v->sector_a + ref->offset;
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result write_ref(struct fat32_exchange_volume *v,
                                            const struct dir_ref *ref)
{
    return sector_write(v, ref->lba, v->sector_a);
}

static enum fat32_exchange_result advance_generation(struct fat32_exchange_volume *v)
{
    if (v->generation == ~0U)
        return fault(v, FAT32_EXCHANGE_FAULTED);
    v->generation++;
    return FAT32_EXCHANGE_OK;
}

static void make_file(const struct fat32_exchange_volume *v,
                      const struct dir_ref *ref, const u8 name[11U],
                      struct fat32_exchange_file *out)
{
    u8 name_copy[FAT32_EXCHANGE_NAME_BYTES];

    bytes_copy(name_copy, name, FAT32_EXCHANGE_NAME_BYTES);
    bytes_zero((u8 *)out, (u32)sizeof(*out));
    out->identity = v->identity;
    out->epoch = v->epoch;
    out->mount_generation = v->mount_generation;
    out->generation = v->generation;
    out->dir_lba = ref->lba;
    out->dir_offset = ref->offset;
    bytes_copy(out->short_name, name_copy, FAT32_EXCHANGE_NAME_BYTES);
}

static enum fat32_exchange_result validate_file(
    struct fat32_exchange_volume *v, const struct fat32_exchange_file *file,
    struct dir_ref *ref, u8 **entry)
{
    enum fat32_exchange_result result;

    if (!file || file->identity != v->identity || file->epoch != v->epoch ||
        file->mount_generation != v->mount_generation ||
        file->generation != v->generation)
        return FAT32_EXCHANGE_STALE;
    ref->lba = file->dir_lba;
    ref->offset = file->dir_offset;
    result = read_ref(v, ref, entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!regular_entry(*entry) ||
        !bytes_equal(*entry, file->short_name, FAT32_EXCHANGE_NAME_BYTES))
        return FAT32_EXCHANGE_STALE;
    {
        bool lfn_precedes;

        result = root_lfn_precedes(v, ref, &lfn_precedes);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (lfn_precedes)
            return FAT32_EXCHANGE_LFN;
    }
    return FAT32_EXCHANGE_OK;
}

static u32 clusters_for_size(const struct fat32_exchange_volume *v, u32 size)
{
    u32 cluster_bytes = v->sectors_per_cluster * FAT32_EXCHANGE_SECTOR_BYTES;

    return size == 0U ? 0U : 1U + (size - 1U) / cluster_bytes;
}

/* Reject both a short chain and a cycle/overlong tail before touching data. */
static enum fat32_exchange_result validate_file_chain(
    struct fat32_exchange_volume *v, u32 first, u32 size)
{
    u32 count = clusters_for_size(v, size);
    u32 last;
    u32 next;
    bool end;
    enum fat32_exchange_result result;

    if (count == 0U)
        return first == 0U ? FAT32_EXCHANGE_OK : FAT32_EXCHANGE_CORRUPT;
    if (first == 0U)
        return FAT32_EXCHANGE_CORRUPT;
    result = chain_at(v, first, count - 1U, &last);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = chain_next(v, last, &next, &end);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    return end ? FAT32_EXCHANGE_OK : FAT32_EXCHANGE_CORRUPT;
}

static enum fat32_exchange_result transfer_bytes(
    struct fat32_exchange_volume *v, u32 first, u32 start, u8 *read_out,
    const u8 *write_in, u32 bytes)
{
    u32 cluster_bytes = v->sectors_per_cluster * FAT32_EXCHANGE_SECTOR_BYTES;
    u32 cluster_index = start / cluster_bytes;
    u32 in_cluster = start % cluster_bytes;
    u32 cluster;
    enum fat32_exchange_result result;

    if (bytes == 0U)
        return FAT32_EXCHANGE_OK;
    result = chain_at(v, first, cluster_index, &cluster);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    while (bytes != 0U) {
        u32 cluster_base;
        u32 sector = in_cluster / FAT32_EXCHANGE_SECTOR_BYTES;
        u32 offset = in_cluster % FAT32_EXCHANGE_SECTOR_BYTES;
        u32 take = FAT32_EXCHANGE_SECTOR_BYTES - offset;

        if (take > bytes)
            take = bytes;
        result = cluster_lba(v, cluster, &cluster_base);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (read_out) {
            result = sector_read(v, cluster_base + sector, v->sector_a);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            bytes_copy(read_out, v->sector_a + offset, take);
            read_out += take;
        } else {
            if (offset != 0U || take != FAT32_EXCHANGE_SECTOR_BYTES) {
                result = sector_read(v, cluster_base + sector, v->sector_a);
                if (result != FAT32_EXCHANGE_OK)
                    return result;
            }
            if (offset == 0U && take == FAT32_EXCHANGE_SECTOR_BYTES)
                bytes_copy(v->sector_a, write_in, take);
            else
                bytes_copy(v->sector_a + offset, write_in, take);
            result = sector_write(v, cluster_base + sector, v->sector_a);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            write_in += take;
        }
        bytes -= take;
        in_cluster += take;
        if (in_cluster == cluster_bytes && bytes != 0U) {
            u32 next;
            bool end;

            result = chain_next(v, cluster, &next, &end);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            if (end)
                return FAT32_EXCHANGE_CORRUPT;
            cluster = next;
            in_cluster = 0U;
        }
    }
    return FAT32_EXCHANGE_OK;
}

static enum fat32_exchange_result update_entry(
    struct fat32_exchange_volume *v, const struct dir_ref *ref, u32 cluster,
    u32 size)
{
    u8 *entry;
    enum fat32_exchange_result result = read_ref(v, ref, &entry);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!regular_entry(entry))
        return FAT32_EXCHANGE_STALE;
    entry_set_cluster(entry, cluster);
    put_le32(entry + 28U, size);
    return write_ref(v, ref);
}

enum fat32_exchange_result fat32_exchange_mount(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    const struct fat32_exchange_io *io, u32 caller_core, bool in_irq)
{
    u8 boot[FAT32_EXCHANGE_SECTOR_BYTES];
    u32 prior_mount_generation;
    u32 sectors_per_cluster;
    u32 reserved;
    u32 fats;
    u32 total;
    u32 fat_sectors;
    u32 fat_lba;
    u32 fat_span;
    u32 data_lba;
    u32 data_sectors;
    u32 clusters;
    u32 i;

    if (!v || !attachment || !io || !io->read || !io->write ||
        caller_core != 0U || in_irq || attachment->identity == 0U ||
        attachment->epoch == 0U || attachment->block_count == 0U ||
        !lba_range_representable(attachment->first_lba,
                                 attachment->block_count))
        return FAT32_EXCHANGE_INVALID;
    prior_mount_generation = v->mount_generation;
    *v = (struct fat32_exchange_volume){0};
    v->io = *io;
    v->identity = attachment->identity;
    v->epoch = attachment->epoch;
    v->first_lba = attachment->first_lba;
    v->block_count = attachment->block_count;
    if (prior_mount_generation == ~0U)
        return fault(v, FAT32_EXCHANGE_FAULTED);
    v->mount_generation = prior_mount_generation + 1U;
    v->generation = 1U;
    if (!io->read(io->context, attachment->first_lba, boot))
        return fault(v, FAT32_EXCHANGE_IO);
    if (boot[510U] != 0x55U || boot[511U] != 0xAAU ||
        le16(boot + 11U) != FAT32_EXCHANGE_SECTOR_BYTES ||
        boot[13U] == 0U || !power_of_two(boot[13U]) || boot[13U] > 128U ||
        boot[16U] != 2U || le16(boot + 17U) != 0U ||
        le16(boot + 22U) != 0U || le32(boot + 36U) == 0U ||
        boot[21U] == 0U || boot[66U] != 0x29U ||
        boot[82U] != 'F' || boot[83U] != 'A' || boot[84U] != 'T' ||
        boot[85U] != '3' || boot[86U] != '2' ||
        boot[87U] != ' ' || boot[88U] != ' ' || boot[89U] != ' ' ||
        !bytes_equal(boot + 71U, attachment->expected_label,
                     FAT32_EXCHANGE_NAME_BYTES))
        return fault(v, bytes_equal(boot + 71U, attachment->expected_label,
                                    FAT32_EXCHANGE_NAME_BYTES) ?
                     FAT32_EXCHANGE_BPB : FAT32_EXCHANGE_LABEL);
    sectors_per_cluster = boot[13U];
    reserved = le16(boot + 14U);
    fats = boot[16U];
    total = le16(boot + 19U) ? le16(boot + 19U) : le32(boot + 32U);
    fat_sectors = le32(boot + 36U);
    if (reserved == 0U || total == 0U || total > attachment->block_count ||
        !lba_range_representable(attachment->first_lba, total) ||
        fat_sectors > FAT32_EXCHANGE_MAX_FAT_SECTORS ||
        reserved > total || fat_sectors > (total - reserved) / fats)
        return fault(v, FAT32_EXCHANGE_BPB);
    if (attachment->first_lba > ~0U - reserved)
        return fault(v, FAT32_EXCHANGE_BPB);
    fat_lba = attachment->first_lba + reserved;
    fat_span = fats * fat_sectors;
    if (fat_lba > ~0U - fat_span)
        return fault(v, FAT32_EXCHANGE_BPB);
    data_lba = fat_lba + fat_span;
    if (data_lba < attachment->first_lba ||
        data_lba - attachment->first_lba >= total)
        return fault(v, FAT32_EXCHANGE_BPB);
    data_sectors = total - (data_lba - attachment->first_lba);
    if (!lba_span_fits(attachment->first_lba, attachment->block_count,
                       fat_lba, fat_span) ||
        !lba_span_fits(attachment->first_lba, attachment->block_count,
                       data_lba, data_sectors))
        return fault(v, FAT32_EXCHANGE_BPB);
    clusters = data_sectors / sectors_per_cluster;
    if (clusters < 65525U || clusters > FAT32_EXCHANGE_MAX_CLUSTERS ||
        le32(boot + 44U) < 2U || le32(boot + 44U) > clusters + 1U ||
        fat_sectors > ~0U / 128U || fat_sectors * 128U < clusters + 2U)
        return fault(v, FAT32_EXCHANGE_BPB);
    v->fat_lba = fat_lba;
    v->fat_sectors = fat_sectors;
    v->data_lba = data_lba;
    v->sectors_per_cluster = sectors_per_cluster;
    v->root_cluster = le32(boot + 44U);
    v->cluster_count = clusters;
    v->alloc_hint = 2U;
    v->fsinfo_lba = 0U;
    if (le16(boot + 48U) != 0U && le16(boot + 48U) != 0xFFFFU) {
        u32 fsinfo = le16(boot + 48U);

        if (fsinfo >= reserved || attachment->first_lba > ~0U - fsinfo)
            return fault(v, FAT32_EXCHANGE_BPB);
        if (!io->read(io->context, attachment->first_lba + fsinfo, v->sector_a))
            return fault(v, FAT32_EXCHANGE_IO);
        if (!fsinfo_valid(v, v->sector_a))
            return fault(v, FAT32_EXCHANGE_BPB);
        v->fsinfo_lba = attachment->first_lba + fsinfo;
        v->fsinfo_needs_unknown = le32(v->sector_a + 488U) != ~0U ||
                                  le32(v->sector_a + 492U) != ~0U;
    }
    for (i = 0U; i < fat_sectors; i++) {
        if (sector_read(v, fat_lba + i, v->sector_a) != FAT32_EXCHANGE_OK ||
            sector_read(v, fat_lba + fat_sectors + i, v->sector_b) != FAT32_EXCHANGE_OK)
            return FAT32_EXCHANGE_IO;
        if (!bytes_equal(v->sector_a, v->sector_b, FAT32_EXCHANGE_SECTOR_BYTES))
            return fault(v, FAT32_EXCHANGE_FAT_MISMATCH);
    }
    v->mounted = 1U;
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_open(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file)
{
    u8 encoded[11U];
    struct dir_ref ref;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!out_file)
        return FAT32_EXCHANGE_INVALID;
    result = encode_name(name, encoded);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = root_find(v, encoded, false, &ref);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    make_file(v, &ref, encoded, out_file);
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_create(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file)
{
    u8 encoded[11U];
    u8 *entry;
    struct dir_ref ref;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!out_file)
        return FAT32_EXCHANGE_INVALID;
    result = encode_name(name, encoded);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = root_find(v, encoded, false, &ref);
    if (result == FAT32_EXCHANGE_OK)
        return FAT32_EXCHANGE_EXISTS;
    if (result != FAT32_EXCHANGE_NOT_FOUND)
        return result;
    result = root_find(v, 0, true, &ref);
    if (result == FAT32_EXCHANGE_NOT_FOUND)
        result = root_extend(v, &ref);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    {
        bool lfn_precedes;

        result = root_lfn_precedes(v, &ref, &lfn_precedes);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (lfn_precedes)
            return FAT32_EXCHANGE_LFN;
    }
    result = prepare_mutation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = read_ref(v, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    bytes_zero(entry, FAT32_DIR_ENTRY_BYTES);
    bytes_copy(entry, encoded, FAT32_EXCHANGE_NAME_BYTES);
    entry[11U] = 0x20U;
    result = write_ref(v, &ref);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = advance_generation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    make_file(v, &ref, encoded, out_file);
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_read(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    u32 offset, u8 *out, u32 capacity, u32 requested, u32 *out_read)
{
    struct dir_ref ref;
    u8 *entry;
    u32 size;
    u32 first;
    u32 take;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (out_read)
        *out_read = 0U;
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!out_read || (requested != 0U && !out) || capacity < requested)
        return FAT32_EXCHANGE_INVALID;
    result = validate_file(v, file, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    size = le32(entry + 28U);
    first = entry_cluster(entry);
    if (offset > size)
        return FAT32_EXCHANGE_RANGE;
    result = validate_file_chain(v, first, size);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    take = requested;
    if (take > size - offset)
        take = size - offset;
    if (take != 0U) {
        result = transfer_bytes(v, first, offset, out, 0, take);
        if (result != FAT32_EXCHANGE_OK)
            return result;
    }
    *out_read = take;
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_rewrite(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file)
{
    struct dir_ref ref;
    u8 *entry;
    u32 old_cluster;
    u32 new_cluster;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!out_file || (data_len != 0U && !data) || data_len > FAT32_EXCHANGE_MAX_FILE_BYTES)
        return FAT32_EXCHANGE_INVALID;
    result = validate_file(v, file, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    old_cluster = entry_cluster(entry);
    result = validate_file_chain(v, old_cluster, le32(entry + 28U));
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = prepare_mutation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = build_chain(v, clusters_for_size(v, data_len), &new_cluster);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (data_len != 0U) {
        result = transfer_bytes(v, new_cluster, 0U, 0, data, data_len);
        if (result != FAT32_EXCHANGE_OK)
            return result;
    }
    result = update_entry(v, &ref, new_cluster, data_len);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = free_chain(v, old_cluster);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = advance_generation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    make_file(v, &ref, file->short_name, out_file);
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_append(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file)
{
    struct dir_ref ref;
    u8 *entry;
    u32 old_size;
    u32 new_size;
    u32 first;
    u32 old_clusters;
    u32 new_clusters;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!out_file || (data_len != 0U && !data))
        return FAT32_EXCHANGE_INVALID;
    result = validate_file(v, file, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    old_size = le32(entry + 28U);
    first = entry_cluster(entry);
    result = validate_file_chain(v, first, old_size);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (data_len > FAT32_EXCHANGE_MAX_FILE_BYTES - old_size)
        return FAT32_EXCHANGE_RANGE;
    new_size = old_size + data_len;
    old_clusters = clusters_for_size(v, old_size);
    new_clusters = clusters_for_size(v, new_size);
    if (old_clusters == 0U && first != 0U)
        return FAT32_EXCHANGE_CORRUPT;
    if (old_clusters != 0U && first == 0U)
        return FAT32_EXCHANGE_CORRUPT;
    if (data_len != 0U) {
        result = prepare_mutation(v);
        if (result != FAT32_EXCHANGE_OK)
            return result;
    }
    if (new_clusters > old_clusters) {
        u32 extension;

        result = build_chain(v, new_clusters - old_clusters, &extension);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (old_clusters == 0U) {
            first = extension;
        } else {
            u32 tail;

            result = chain_at(v, first, old_clusters - 1U, &tail);
            if (result != FAT32_EXCHANGE_OK)
                return result;
            result = fat_set(v, tail, extension);
            if (result != FAT32_EXCHANGE_OK)
                return result;
        }
    }
    if (data_len != 0U) {
        result = transfer_bytes(v, first, old_size, 0, data, data_len);
        if (result != FAT32_EXCHANGE_OK)
            return result;
    }
    if (data_len != 0U) {
        result = update_entry(v, &ref, first, new_size);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        result = advance_generation(v);
        if (result != FAT32_EXCHANGE_OK)
            return result;
    }
    make_file(v, &ref, file->short_name, out_file);
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_delete(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file)
{
    struct dir_ref ref;
    u8 *entry;
    u32 first;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = validate_file(v, file, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    first = entry_cluster(entry);
    result = validate_file_chain(v, first, le32(entry + 28U));
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = prepare_mutation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = free_chain(v, first);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = read_ref(v, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    entry[0] = 0xE5U;
    result = write_ref(v, &ref);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    return advance_generation(v);
}

enum fat32_exchange_result fat32_exchange_rename(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    const struct fat32_exchange_name *new_name,
    struct fat32_exchange_file *out_file)
{
    struct dir_ref ref;
    struct dir_ref existing;
    u8 *entry;
    u8 encoded[11U];
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (!out_file)
        return FAT32_EXCHANGE_INVALID;
    result = validate_file(v, file, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = encode_name(new_name, encoded);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    if (bytes_equal(encoded, file->short_name, FAT32_EXCHANGE_NAME_BYTES)) {
        make_file(v, &ref, encoded, out_file);
        return FAT32_EXCHANGE_OK;
    }
    result = root_find(v, encoded, false, &existing);
    if (result == FAT32_EXCHANGE_OK)
        return FAT32_EXCHANGE_EXISTS;
    if (result != FAT32_EXCHANGE_NOT_FOUND)
        return result;
    result = prepare_mutation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = read_ref(v, &ref, &entry);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    bytes_copy(entry, encoded, FAT32_EXCHANGE_NAME_BYTES);
    result = write_ref(v, &ref);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    result = advance_generation(v);
    if (result != FAT32_EXCHANGE_OK)
        return result;
    make_file(v, &ref, encoded, out_file);
    return FAT32_EXCHANGE_OK;
}

enum fat32_exchange_result fat32_exchange_flush(
    struct fat32_exchange_volume *v,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq)
{
    u32 i;
    enum fat32_exchange_result result = access_ok(v, attachment, caller_core, in_irq);

    if (result != FAT32_EXCHANGE_OK)
        return result;
    for (i = 0U; i < v->fat_sectors; i++) {
        result = sector_read(v, v->fat_lba + i, v->sector_a);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        result = sector_read(v, v->fat_lba + v->fat_sectors + i, v->sector_b);
        if (result != FAT32_EXCHANGE_OK)
            return result;
        if (!bytes_equal(v->sector_a, v->sector_b, FAT32_EXCHANGE_SECTOR_BYTES))
            return fault(v, FAT32_EXCHANGE_FAT_MISMATCH);
    }
    return FAT32_EXCHANGE_OK;
}
