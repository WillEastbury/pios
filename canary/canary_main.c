/*
 * canary_main.c - Pi 5 bare-metal framebuffer test via VideoCore mailbox
 *
 * Mailbox at 0x107C013880 (BCM2712 on-SoC, no PCIe).
 * Allocates framebuffer, fills screen solid color.
 */
typedef unsigned int u32;
typedef unsigned long u64;

static inline void mmio_write(u64 addr, u32 val) {
    *(volatile u32 *)addr = val;
}
static inline u32 mmio_read(u64 addr) {
    return *(volatile u32 *)addr;
}

#define MBOX_BASE       0x107C013880UL
#define MBOX0_READ      (MBOX_BASE + 0x00)
#define MBOX0_STATUS    (MBOX_BASE + 0x18)
#define MBOX1_WRITE     (MBOX_BASE + 0x20)
#define MBOX1_STATUS    (MBOX_BASE + 0x38)
#define MBOX_FULL       0x80000000
#define MBOX_EMPTY      0x40000000
#define MBOX_CH         8
#define MBOX_REQUEST    0x00000000
#define MBOX_RESPONSE   0x80000000

/* Aligned mailbox buffer — must be 16-byte aligned, in first 1GB for VC */
static volatile u32 __attribute__((aligned(16))) mbox[64];

static void cache_flush(volatile void *addr, u32 size) {
    u64 p = (u64)addr;
    for (u32 i = 0; i < size; i += 64)
        __asm__ volatile("dc civac, %0" :: "r"(p + i));
    __asm__ volatile("dsb sy");
}

static int mbox_call(void) {
    cache_flush(mbox, sizeof(mbox));

    u32 addr = ((u32)(u64)mbox) | MBOX_CH;
    while (mmio_read(MBOX1_STATUS) & MBOX_FULL) {}
    mmio_write(MBOX1_WRITE, addr);

    while (1) {
        while (mmio_read(MBOX0_STATUS) & MBOX_EMPTY) {}
        if (mmio_read(MBOX0_READ) == addr) {
            cache_flush(mbox, sizeof(mbox));
            return mbox[1] == MBOX_RESPONSE;
        }
    }
}

void main(void) {
    /* Build tag buffer: set phys/virt size, depth, alloc FB, get pitch */
    int i = 0;
    mbox[i++] = 0;           /* [0] total size (filled below) */
    mbox[i++] = MBOX_REQUEST; /* [1] request code */

    /* Set physical display size */
    mbox[i++] = 0x00048003;  /* tag */
    mbox[i++] = 8;           /* value buffer size */
    mbox[i++] = 8;           /* request size */
    mbox[i++] = 1024;        /* width */
    mbox[i++] = 768;         /* height */

    /* Set virtual display size */
    mbox[i++] = 0x00048004;
    mbox[i++] = 8;
    mbox[i++] = 8;
    mbox[i++] = 1024;
    mbox[i++] = 768;

    /* Set depth */
    mbox[i++] = 0x00048005;
    mbox[i++] = 4;
    mbox[i++] = 4;
    mbox[i++] = 32;          /* 32 bits per pixel */

    /* Set pixel order (0=BGR, 1=RGB) */
    mbox[i++] = 0x00048006;
    mbox[i++] = 4;
    mbox[i++] = 4;
    mbox[i++] = 1;           /* RGB */

    /* Allocate framebuffer */
    int fb_idx = i + 3;      /* where base addr response will be */
    mbox[i++] = 0x00040001;
    mbox[i++] = 8;
    mbox[i++] = 4;
    mbox[i++] = 16;          /* alignment */
    mbox[i++] = 0;           /* size (response) */

    /* Get pitch */
    int pitch_idx = i + 3;
    mbox[i++] = 0x00040008;
    mbox[i++] = 4;
    mbox[i++] = 4;
    mbox[i++] = 0;           /* pitch (response) */

    /* End tag */
    mbox[i++] = 0;

    /* Fill in total size */
    mbox[0] = i * 4;

    if (!mbox_call()) {
        /* Mailbox call failed — spin */
        while (1) __asm__ volatile("wfe");
    }

    /* Read results */
    u32 fb_addr = mbox[fb_idx] & 0x3FFFFFFF;
    u32 fb_size = mbox[fb_idx + 1];
    u32 pitch   = mbox[pitch_idx];

    if (!fb_addr) {
        while (1) __asm__ volatile("wfe");
    }

    /* Fill entire framebuffer solid blue */
    volatile u32 *fb = (volatile u32 *)(u64)fb_addr;
    u32 pixels = fb_size / 4;
    if (!pixels) pixels = (1024 * 768); /* fallback */

    for (u32 p = 0; p < pixels; p++)
        fb[p] = 0xFF0000FF; /* ABGR: blue (since we set RGB order) */

    (void)pitch;

    while (1) __asm__ volatile("wfe");
}
