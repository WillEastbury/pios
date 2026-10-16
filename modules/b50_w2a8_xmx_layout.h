/* Shared scalar layout helpers; no DPAS emulation or runtime orchestration. */
static uint b50_xmx_weight_code(__global const uchar *packed, uint row, uint col)
{
    uint thread = (row / 8) * 16 + (col / 16) * 8 + (row & 7);
    uint perm_row = thread / 2;
    uint perm_col = (thread & 1) * 16 + (col & 15);
    uint offset = perm_col & 15;
    uint shift = (offset & 3) * 8 + (offset / 4) * 2;
    uint byte = perm_row * 8 + (perm_col / 16) * 4 + shift / 8;
    return (packed[byte] >> (shift & 7)) & 3;
}

static uint b50_xmx_a_word(__global const char *activation, uint batch, uint lane)
{
    uint col = batch * 32 + lane * 2;
    return (uint)(uchar)activation[col] |
           ((uint)(uchar)activation[col + 1] << 8);
}

static uint b50_xmx_b_word(__global const uchar *packed, uint row, uint word)
{
    uint result = 0;
    for (uint byte = 0; byte < 4; ++byte) {
        int weight = (int)b50_xmx_weight_code(packed, row, word * 4 + byte) - 2;
        result |= (uint)(uchar)weight << (byte * 8);
    }
    return result;
}
