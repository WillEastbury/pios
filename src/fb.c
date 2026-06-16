/*
 * fb.c - HDMI framebuffer text console
 * Requests a 32-bit framebuffer via VideoCore mailbox,
 * renders text using a built-in 8x8 bitmap font.
 */

#include "fb.h"
#include "mailbox.h"
#include "mmio.h"
#include "dma.h"
#include "mmu.h"
#include "core_env.h"
#include "simd.h"

/* Framebuffer state */
static u32 *fb_ptr;
static u32  fb_width;
static u32  fb_height;
static u32  fb_pitch;   /* bytes per row */
static u32  fb_size;    /* total framebuffer bytes (from VideoCore) */
#ifndef PIOS_FB_NO_DOUBLE_BUFFER
static u32 *fb_back;    /* cached back buffer (rendering target when double-buffered) */
static bool fb_db;      /* double-buffering enabled */
/* Dirty tracking is per 8px row-band: fb_present() blits only the bands actually
 * touched since the last present, so a render that changes a few scattered text
 * rows no longer forces one big contiguous span through the (slow) scanout. */
#define FB_BAND_PX   8u
#define FB_MAX_BANDS 256u                       /* covers up to 2048px height */
static u64 fb_dirty_band[FB_MAX_BANDS / 64u];   /* 1 bit per band, set == dirty */
static volatile u64 g_fb_blit_ticks; /* CNTPCT ticks of the last blit */
#endif
static u32  fb_fg = 0x00FF9900;  /* amber */
static u32  fb_bg = 0x00000000;  /* black */

/* ── Text-cell shadow cache ──
 * Records the (code,fg,bg) last drawn at each text cell so re-rendering identical
 * content (e.g. the 1Hz HDMI dashboard, whose fields are mostly static) skips the
 * pixel writes and dirty marking. Display-only: a stale entry can affect nothing
 * but the diagnostic surface, and any direct-pixel write invalidates the cells it
 * touches. Sized to bound RAM; enabled only when the panel fits and DB is on. */
#define FB_SH_COLS 240u
#define FB_SH_ROWS 136u
#define FB_CELL_INVALID 0xFFFFFFFFu
struct fb_cell { u32 code; u32 fg; u32 bg; };
static struct fb_cell fb_shadow[FB_SH_ROWS * FB_SH_COLS];
static bool fb_shadow_on;
/* Deferred-clear state: fb_clear_text_row() marks cells to-be-blanked rather than
 * writing them, so a clear immediately followed by an identical redraw (static
 * dashboard rows) costs zero pixel writes; fb_present() flushes any cell not
 * redrawn this frame. */
static u8  fb_cell_pending[FB_SH_ROWS * FB_SH_COLS]; /* 1 = blank at flush unless redrawn */
static u8  fb_row_pending[FB_SH_ROWS];               /* 1 = row has >=1 pending cell */
static u32 fb_row_clear_bg[FB_SH_ROWS];              /* bg captured at clear time */

/* Text cursor */
static u32  cursor_x;
static u32  cursor_y;
static u32  cols;
static u32  rows;
static u32  reserved_rows;
static bool console_wrapped;

/* ── C64-style screen border ──
 * A gradient frame is painted around the whole panel and the text console +
 * status bars are inset inside it (an homage to Commodore loading screens).
 * The gradient sweeps green→blue→green around the perimeter, seamlessly.
 * Inset stays 0 on the tiny stage0 bootstrap (full-bleed, unchanged). */
#define FB_BORDER_PX 32u   /* thickness of the gradient frame */
#define FB_INSET_PX  40u   /* where content starts (border + ~8px gap) */
static u32 fb_inset_x;     /* content origin x; 0 = border disabled */
static u32 fb_inset_y;     /* content origin y; 0 = border disabled */
static void fb_draw_border(void);

#ifndef PIOS_FB_NO_DOUBLE_BUFFER
/* Render target: the cached back buffer when double-buffered, else the
 * VideoCore scanout buffer directly. */
static inline u32 *fb_rt(void) { return fb_db ? fb_back : fb_ptr; }

/* Mark the dirty bands overlapping pixel-row range [y0,y1) for the next blit. */
static inline void fb_mark_dirty(u32 y0, u32 y1) {
    if (!fb_db) return;
    if (y1 > fb_height) y1 = fb_height;
    if (y0 >= y1) return;
    u32 b0 = y0 / FB_BAND_PX;
    u32 b1 = (y1 - 1u) / FB_BAND_PX;
    if (b1 >= FB_MAX_BANDS) b1 = FB_MAX_BANDS - 1u;
    for (u32 b = b0; b <= b1; b++)
        fb_dirty_band[b >> 6] |= (1ULL << (b & 63u));
}
#else
/* Double-buffering disabled (e.g. the tiny stage0 bootstrap): render straight
 * to the scanout buffer and treat present/dirty as no-ops. */
static inline u32 *fb_rt(void) { return fb_ptr; }
static inline void fb_mark_dirty(u32 y0, u32 y1) { (void)y0; (void)y1; }
#endif

/* Shadow cell at text position (cx,cy), or NULL when caching is off/out of range
 * (callers fall back to drawing unconditionally). */
static inline struct fb_cell *fb_shadow_at(u32 cx, u32 cy) {
    if (!fb_shadow_on || cx >= FB_SH_COLS || cy >= FB_SH_ROWS)
        return (struct fb_cell *)0;
    return &fb_shadow[cy * FB_SH_COLS + cx];
}

/* Invalidate cached cells overlapping pixel rows [py0,py1) so a later identical
 * glyph there is redrawn. Used after direct-pixel writes that bypass the cell
 * path (fb_pixel, fb_status_block). Over-invalidating only costs a redraw. */
static void fb_shadow_invalidate_px(u32 py0, u32 py1) {
    if (!fb_shadow_on) return;
    u32 r0 = (py0 > fb_inset_y) ? (py0 - fb_inset_y) / 8u : 0;
    u32 r1 = (py1 > fb_inset_y) ? (py1 - fb_inset_y + 7u) / 8u : 0;
    if (r1 > FB_SH_ROWS) r1 = FB_SH_ROWS;
    for (u32 cy = r0; cy < r1; cy++)
        for (u32 cx = 0; cx < FB_SH_COLS; cx++) {
            fb_shadow[cy * FB_SH_COLS + cx].code = FB_CELL_INVALID;
            fb_cell_pending[cy * FB_SH_COLS + cx] = 0;  /* direct pixels own this cell now */
        }
}

/* Mailbox buffer - must be 16-byte aligned */
static volatile u32 __attribute__((aligned(16))) mbox_fb[36];

/* ---- Minimal 8x8 bitmap font (ASCII 32-126) ---- */
static const u8 font8x8[95][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 32 space */
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, /* 33 ! */
    {0x6C,0x6C,0x24,0x00,0x00,0x00,0x00,0x00}, /* 34 " */
    {0x6C,0xFE,0x6C,0x6C,0xFE,0x6C,0x00,0x00}, /* 35 # */
    {0x18,0x7E,0x06,0x3C,0x60,0x3E,0x18,0x00}, /* 36 $ */
    {0x46,0x66,0x30,0x18,0x0C,0x66,0x62,0x00}, /* 37 % */
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, /* 38 & */
    {0x18,0x18,0x08,0x00,0x00,0x00,0x00,0x00}, /* 39 ' */
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, /* 40 ( */
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, /* 41 ) */
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, /* 42 * */
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, /* 43 + */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x08}, /* 44 , */
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, /* 45 - */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, /* 46 . */
    {0x40,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, /* 47 / */
    {0x3C,0x66,0x76,0x7E,0x6E,0x66,0x3C,0x00}, /* 48 0 */
    {0x18,0x1C,0x18,0x18,0x18,0x18,0x7E,0x00}, /* 49 1 */
    {0x3C,0x66,0x60,0x30,0x18,0x0C,0x7E,0x00}, /* 50 2 */
    {0x3C,0x66,0x60,0x38,0x60,0x66,0x3C,0x00}, /* 51 3 */
    {0x30,0x38,0x34,0x32,0x7E,0x30,0x30,0x00}, /* 52 4 */
    {0x7E,0x06,0x3E,0x60,0x60,0x66,0x3C,0x00}, /* 53 5 */
    {0x3C,0x06,0x06,0x3E,0x66,0x66,0x3C,0x00}, /* 54 6 */
    {0x7E,0x60,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* 55 7 */
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, /* 56 8 */
    {0x3C,0x66,0x66,0x7C,0x60,0x60,0x3C,0x00}, /* 57 9 */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, /* 58 : */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x08}, /* 59 ; */
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, /* 60 < */
    {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, /* 61 = */
    {0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00}, /* 62 > */
    {0x3C,0x66,0x60,0x30,0x18,0x00,0x18,0x00}, /* 63 ? */
    {0x3C,0x66,0x76,0x76,0x06,0x46,0x3C,0x00}, /* 64 @ */
    {0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00}, /* 65 A */
    {0x3E,0x66,0x66,0x3E,0x66,0x66,0x3E,0x00}, /* 66 B */
    {0x3C,0x66,0x06,0x06,0x06,0x66,0x3C,0x00}, /* 67 C */
    {0x1E,0x36,0x66,0x66,0x66,0x36,0x1E,0x00}, /* 68 D */
    {0x7E,0x06,0x06,0x3E,0x06,0x06,0x7E,0x00}, /* 69 E */
    {0x7E,0x06,0x06,0x3E,0x06,0x06,0x06,0x00}, /* 70 F */
    {0x3C,0x66,0x06,0x76,0x66,0x66,0x3C,0x00}, /* 71 G */
    {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, /* 72 H */
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00}, /* 73 I */
    {0x78,0x30,0x30,0x30,0x30,0x36,0x1C,0x00}, /* 74 J */
    {0x66,0x36,0x1E,0x0E,0x1E,0x36,0x66,0x00}, /* 75 K */
    {0x06,0x06,0x06,0x06,0x06,0x06,0x7E,0x00}, /* 76 L */
    {0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0x00}, /* 77 M */
    {0x66,0x6E,0x7E,0x7E,0x76,0x66,0x66,0x00}, /* 78 N */
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, /* 79 O */
    {0x3E,0x66,0x66,0x3E,0x06,0x06,0x06,0x00}, /* 80 P */
    {0x3C,0x66,0x66,0x66,0x56,0x36,0x6C,0x00}, /* 81 Q */
    {0x3E,0x66,0x66,0x3E,0x36,0x66,0x66,0x00}, /* 82 R */
    {0x3C,0x66,0x06,0x3C,0x60,0x66,0x3C,0x00}, /* 83 S */
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, /* 84 T */
    {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, /* 85 U */
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, /* 86 V */
    {0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00}, /* 87 W */
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, /* 88 X */
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, /* 89 Y */
    {0x7E,0x60,0x30,0x18,0x0C,0x06,0x7E,0x00}, /* 90 Z */
    {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, /* 91 [ */
    {0x02,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, /* 92 \ */
    {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, /* 93 ] */
    {0x18,0x3C,0x66,0x00,0x00,0x00,0x00,0x00}, /* 94 ^ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00}, /* 95 _ */
    {0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00}, /* 96 ` */
    {0x00,0x00,0x3C,0x60,0x7C,0x66,0x7C,0x00}, /* 97 a */
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, /* 98 b */
    {0x00,0x00,0x3C,0x06,0x06,0x06,0x3C,0x00}, /* 99 c */
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, /*100 d */
    {0x00,0x00,0x3C,0x66,0x7E,0x06,0x3C,0x00}, /*101 e */
    {0x38,0x0C,0x0C,0x3E,0x0C,0x0C,0x0C,0x00}, /*102 f */
    {0x00,0x00,0x7C,0x66,0x7C,0x60,0x3C,0x00}, /*103 g */
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x00}, /*104 h */
    {0x18,0x00,0x1C,0x18,0x18,0x18,0x3C,0x00}, /*105 i */
    {0x30,0x00,0x38,0x30,0x30,0x30,0x1E,0x00}, /*106 j */
    {0x06,0x06,0x66,0x36,0x1E,0x36,0x66,0x00}, /*107 k */
    {0x1C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, /*108 l */
    {0x00,0x00,0x6C,0xFE,0xD6,0xC6,0xC6,0x00}, /*109 m */
    {0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x00}, /*110 n */
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, /*111 o */
    {0x00,0x00,0x3E,0x66,0x3E,0x06,0x06,0x00}, /*112 p */
    {0x00,0x00,0x7C,0x66,0x7C,0x60,0x60,0x00}, /*113 q */
    {0x00,0x00,0x36,0x1E,0x06,0x06,0x06,0x00}, /*114 r */
    {0x00,0x00,0x7C,0x06,0x3C,0x60,0x3E,0x00}, /*115 s */
    {0x0C,0x0C,0x3E,0x0C,0x0C,0x0C,0x38,0x00}, /*116 t */
    {0x00,0x00,0x66,0x66,0x66,0x66,0x7C,0x00}, /*117 u */
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, /*118 v */
    {0x00,0x00,0xC6,0xC6,0xD6,0xFE,0x6C,0x00}, /*119 w */
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, /*120 x */
    {0x00,0x00,0x66,0x66,0x7C,0x60,0x3C,0x00}, /*121 y */
    {0x00,0x00,0x7E,0x30,0x18,0x0C,0x7E,0x00}, /*122 z */
    {0x30,0x18,0x18,0x0C,0x18,0x18,0x30,0x00}, /*123 { */
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, /*124 | */
    {0x0C,0x18,0x18,0x30,0x18,0x18,0x0C,0x00}, /*125 } */
    {0x00,0x00,0x4C,0x7E,0x32,0x00,0x00,0x00}, /*126 ~ */
};

enum {
    FB_GLYPH_H = 95,
    FB_GLYPH_V,
    FB_GLYPH_TL,
    FB_GLYPH_TR,
    FB_GLYPH_BL,
    FB_GLYPH_BR,
    FB_GLYPH_LT,
    FB_GLYPH_RT,
    FB_GLYPH_TT,
    FB_GLYPH_BT,
    FB_GLYPH_CROSS,
    FB_GLYPH_BLOCK,
    FB_GLYPH_SHADE1,
    FB_GLYPH_SHADE2,
    FB_GLYPH_SHADE3,
};

static const u8 font8x8_box[][8] = {
    {0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00}, /* horizontal */
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}, /* vertical */
    {0x00,0x00,0x00,0xF8,0xF8,0x18,0x18,0x18}, /* top-left */
    {0x00,0x00,0x00,0x1F,0x1F,0x18,0x18,0x18}, /* top-right */
    {0x18,0x18,0x18,0xF8,0xF8,0x00,0x00,0x00}, /* bottom-left */
    {0x18,0x18,0x18,0x1F,0x1F,0x00,0x00,0x00}, /* bottom-right */
    {0x18,0x18,0x18,0xF8,0xF8,0x18,0x18,0x18}, /* tee-left */
    {0x18,0x18,0x18,0x1F,0x1F,0x18,0x18,0x18}, /* tee-right */
    {0x00,0x00,0x00,0xFF,0xFF,0x18,0x18,0x18}, /* tee-top */
    {0x18,0x18,0x18,0xFF,0xFF,0x00,0x00,0x00}, /* tee-bottom */
    {0x18,0x18,0x18,0xFF,0xFF,0x18,0x18,0x18}, /* cross */
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}, /* full block */
    {0x22,0x00,0x88,0x00,0x22,0x00,0x88,0x00}, /* light shade */
    {0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA}, /* medium shade */
    {0xDD,0x77,0xDD,0x77,0xDD,0x77,0xDD,0x77}, /* dark shade */
};

/* ---- Framebuffer init via mailbox ---- */

/*
 * Self-contained mailbox call — copied verbatim from the working canary
 * kernel (canary/canary_main.c).  Bypasses mailbox.c to eliminate any
 * differences in cache flushing, message construction, or timing.
 */
#define FB_MBOX_BASE       0x107C013880UL
#define FB_MBOX0_READ      (FB_MBOX_BASE + 0x00)
#define FB_MBOX0_STATUS    (FB_MBOX_BASE + 0x18)
#define FB_MBOX1_WRITE     (FB_MBOX_BASE + 0x20)
#define FB_MBOX1_STATUS    (FB_MBOX_BASE + 0x38)
#define FB_MBOX_FULL       0x80000000
#define FB_MBOX_EMPTY      0x40000000
#define FB_MBOX_CH         8
#define FB_MBOX_RESPONSE   0x80000000

static volatile u32 __attribute__((aligned(16))) fb_mbox[64];

static void fb_cache_flush(volatile void *addr, u32 size) {
    u64 p = (u64)addr;
    for (u32 j = 0; j < size; j += 64)
        __asm__ volatile("dc civac, %0" :: "r"(p + j));
    __asm__ volatile("dsb sy");
}

static int fb_mbox_call(void) {
    fb_cache_flush(fb_mbox, sizeof(fb_mbox));

    u32 addr = ((u32)(u64)fb_mbox) | FB_MBOX_CH;
    while (mmio_read(FB_MBOX1_STATUS) & FB_MBOX_FULL) {}
    mmio_write(FB_MBOX1_WRITE, addr);

    while (1) {
        while (mmio_read(FB_MBOX0_STATUS) & FB_MBOX_EMPTY) {}
        if (mmio_read(FB_MBOX0_READ) == addr) {
            fb_cache_flush(fb_mbox, sizeof(fb_mbox));
            return fb_mbox[1] == FB_MBOX_RESPONSE;
        }
    }
}

/* Ask the firmware for the ARM clock's maximum rate and set the ARM clock to
 * it. Bare-metal Pi 5 otherwise leaves the A76 at a low firmware default
 * frequency (no DVFS governor requests a higher one), which makes the whole
 * system run ~10-100x slower than it should. Returns the rate the firmware
 * reports it set (Hz), or 0 on failure. */
u32 fb_set_arm_clock_max(void) {
    int i = 0;
    fb_mbox[i++] = 0;            /* [0] total size */
    fb_mbox[i++] = 0;            /* [1] request */
    fb_mbox[i++] = 0x00030004;   /* GET_MAX_CLOCK_RATE */
    fb_mbox[i++] = 8;            /* value buffer size */
    fb_mbox[i++] = 4;            /* request size */
    fb_mbox[i++] = 3;            /* clock id: ARM */
    fb_mbox[i++] = 0;            /* [6] response: max rate */
    fb_mbox[i++] = 0;            /* end tag */
    fb_mbox[0] = (u32)(i * 4);
    if (!fb_mbox_call())
        return 0;
    u32 max_rate = fb_mbox[6];
    if (!max_rate)
        return 0;

    i = 0;
    fb_mbox[i++] = 0;            /* [0] total size */
    fb_mbox[i++] = 0;            /* [1] request */
    fb_mbox[i++] = 0x00038002;   /* SET_CLOCK_RATE */
    fb_mbox[i++] = 12;           /* value buffer size */
    fb_mbox[i++] = 8;            /* request size */
    fb_mbox[i++] = 3;            /* clock id: ARM */
    fb_mbox[i++] = max_rate;     /* [6] desired rate (response: actual) */
    fb_mbox[i++] = 0;            /* skip setting turbo = 0 */
    fb_mbox[i++] = 0;            /* end tag */
    fb_mbox[0] = (u32)(i * 4);
    if (!fb_mbox_call())
        return 0;
    return fb_mbox[6];
}

u32 fb_get_arm_clock(void) {
    int i = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0x00030002;   /* GET_CLOCK_RATE */
    fb_mbox[i++] = 8;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 3;            /* clock id: ARM */
    fb_mbox[i++] = 0;            /* [6] response: rate */
    fb_mbox[i++] = 0;
    fb_mbox[0] = (u32)(i * 4);
    if (!fb_mbox_call())
        return 0;
    return fb_mbox[6];
}

/* GET_CLOCK_RATE for any clock id (3=ARM, 4=CORE, etc.). */
u32 fb_get_clock_rate_id(u32 clock_id) {
    int i = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0x00030002;   /* GET_CLOCK_RATE */
    fb_mbox[i++] = 8;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = clock_id;
    fb_mbox[i++] = 0;            /* [6] response: rate */
    fb_mbox[i++] = 0;
    fb_mbox[0] = (u32)(i * 4);
    if (!fb_mbox_call())
        return 0;
    return fb_mbox[6];
}

/* GET_THROTTLED (0x00030046): firmware power/thermal status bitmap.
 * bit0=under-voltage now, bit1=arm freq capped now, bit2=currently throttled,
 * bit3=soft-temp-limit now; bits16..19 = same flags "occurred since boot". */
u32 fb_get_throttled(void) {
    int i = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0x00030046;   /* GET_THROTTLED */
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 0;            /* [5] response: throttled bits */
    fb_mbox[i++] = 0;
    fb_mbox[0] = (u32)(i * 4);
    if (!fb_mbox_call())
        return 0xFFFFFFFFU;
    return fb_mbox[5];
}

/* GET_TEMPERATURE (0x00030006): SoC temperature in millidegrees Celsius. */
u32 fb_get_temperature_mc(void) {
    int i = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0;
    fb_mbox[i++] = 0x00030006;   /* GET_TEMPERATURE */
    fb_mbox[i++] = 8;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 0;            /* [5] temperature id (input) */
    fb_mbox[i++] = 0;            /* [6] response: milli-Celsius */
    fb_mbox[i++] = 0;
    fb_mbox[0] = (u32)(i * 4);
    if (!fb_mbox_call())
        return 0xFFFFFFFFU;
    return fb_mbox[6];
}

bool fb_init(u32 width, u32 height) {
    /* Build tag buffer — matching canary_main.c exactly */
    int i = 0;
    fb_mbox[i++] = 0;           /* [0] total size (filled below) */
    fb_mbox[i++] = 0;           /* [1] request code */

    /* Set physical display size */
    fb_mbox[i++] = 0x00048003;
    fb_mbox[i++] = 8;
    fb_mbox[i++] = 8;
    fb_mbox[i++] = width;
    fb_mbox[i++] = height;

    /* Set virtual display size */
    fb_mbox[i++] = 0x00048004;
    fb_mbox[i++] = 8;
    fb_mbox[i++] = 8;
    fb_mbox[i++] = width;
    fb_mbox[i++] = height;

    /* Set depth */
    fb_mbox[i++] = 0x00048005;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 32;

    /* Set pixel order (0=BGR, 1=RGB) */
    fb_mbox[i++] = 0x00048006;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 0;           /* BGR — matches our 0x00RRGGBB color constants */

    /* Allocate framebuffer */
    int fb_idx = i + 3;
    fb_mbox[i++] = 0x00040001;
    fb_mbox[i++] = 8;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 16;          /* alignment */
    fb_mbox[i++] = 0;           /* size (response) */

    /* Get pitch */
    int pitch_idx = i + 3;
    fb_mbox[i++] = 0x00040008;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 4;
    fb_mbox[i++] = 0;

    /* End tag */
    fb_mbox[i++] = 0;
    fb_mbox[0] = (u32)(i * 4);

    if (!fb_mbox_call())
        return false;

    u32 fb_addr  = fb_mbox[fb_idx] & 0x3FFFFFFF;
    u32 raw_size = fb_mbox[fb_idx + 1];

    if (!fb_addr)
        return false;

    fb_ptr    = (u32 *)(u64)fb_addr;
    fb_width  = width;
    fb_height = height;
    fb_pitch  = fb_mbox[pitch_idx];
    fb_size   = raw_size;
    cursor_x  = 0;
    cursor_y  = 0;
    /* Inset the console inside the gradient border when the panel is large
     * enough to host it. Disabled (inset 0) on the stage0 bootstrap so the
     * boot-critical early console keeps its exact full-bleed behaviour. */
#ifndef PIOS_FB_NO_DOUBLE_BUFFER
    if (width > 2u * FB_INSET_PX + 64u && height > 2u * FB_INSET_PX + 64u) {
        fb_inset_x = FB_INSET_PX;
        fb_inset_y = FB_INSET_PX;
    } else {
        fb_inset_x = 0;
        fb_inset_y = 0;
    }
#endif
    cols      = (width  - 2u * fb_inset_x) / 8;
    rows      = (height - 2u * fb_inset_y) / 8;

    /* Fallback if pitch wasn't returned */
    if (fb_pitch == 0)
        fb_pitch = width * 4;

    /* Fallback if size wasn't returned */
    if (fb_size == 0)
        fb_size = fb_pitch * height;

    /* Set up the cached back buffer for double-buffered rendering. All fb_*
     * drawing targets this buffer; fb_present() DMA-blits dirty rows to the
     * VideoCore scanout buffer (fb_ptr). */
#ifndef PIOS_FB_NO_DOUBLE_BUFFER
    fb_back = (u32 *)(usize)FB_BACK_BASE;
    fb_db   = (fb_back != 0) && (fb_size != 0) && (fb_size <= FB_BACK_SIZE);
    for (u32 k = 0; k < FB_MAX_BANDS / 64u; k++)
        fb_dirty_band[k] = 0;
    /* Enable the text-cell shadow cache when the panel fits the fixed cache
     * dimensions; the upcoming fb_clear(fb_bg) seeds every cell to blank. */
    fb_shadow_on = (cols <= FB_SH_COLS && rows <= FB_SH_ROWS);
#endif

    fb_clear(fb_bg);   /* clears the render target and marks the full frame dirty */
    fb_present();      /* push the cleared frame to the scanout buffer (no-op if DB off) */
    return true;
}

/* Perimeter gradient colour for clockwise arc-length s (0..P, P=2*(W+H)).
 * Green at the top-left corner (s=0 and s=P), blue at the bottom-right
 * (s=P/2); the triangle blend makes adjacent edges meet seamlessly. */
static inline u32 fb_border_color(u32 s) {
    u32 P = 2u * (fb_width + fb_height);
    if (P == 0) return 0;
    u32 half = P / 2u;
    if (half == 0) return 0;
    u32 d = (s <= half) ? s : (P - s);              /* 0..half */
    u32 t = (u32)(((u64)d * 256u) / half);          /* 0..256, 0=purple 256=blue */
    if (t > 256u) t = 256u;
    u32 r = (0xA0u * (256u - t) + 0x10u * t) >> 8;  /* red   0xA0→0x10 */
    u32 g = (0x10u * (256u - t) + 0x40u * t) >> 8;  /* green 0x10→0x40 */
    u32 b = (0xE0u * (256u - t) + 0xF0u * t) >> 8;  /* blue  0xE0→0xF0 */
    return (r << 16) | (g << 8) | b;                /* 0x00RRGGBB: purple→blue */
}

/* Paint the gradient frame around the panel edges. Drawn by fb_clear; the
 * console and status bars never write into the border band, so it persists. */
static void fb_draw_border(void) {
    if (!fb_ptr || (fb_inset_x == 0 && fb_inset_y == 0))
        return;
    u32 W = fb_width, H = fb_height, B = FB_BORDER_PX;
    if (W < 2u * B || H < 2u * B)
        return;
    u32 *base = fb_rt();
    /* Top & bottom bands span the full width; colour depends on column x. */
    for (u32 x = 0; x < W; x++) {
        u32 ctop = fb_border_color(x);
        u32 cbot = fb_border_color(W + H + (W - 1 - x));
        for (u32 y = 0; y < B; y++) {
            *(u32 *)((u8 *)base + y * fb_pitch + x * 4) = ctop;
            *(u32 *)((u8 *)base + (H - 1 - y) * fb_pitch + x * 4) = cbot;
        }
    }
    /* Left & right bands fill between the corners; colour depends on row y. */
    for (u32 y = B; y < H - B; y++) {
        u32 cl = fb_border_color(2u * W + H + (H - 1 - y));
        u32 cr = fb_border_color(W + y);
        u32 *row = (u32 *)((u8 *)base + y * fb_pitch);
        for (u32 x = 0; x < B; x++) {
            row[x] = cl;
            row[W - 1 - x] = cr;
        }
    }
    fb_mark_dirty(0, H);
}

void fb_clear(u32 color) {
    /* Use VideoCore-reported size to cover the entire allocation */
    u32 *dst = fb_rt();
    u32 total = fb_size / 4;
    if (!total) total = (fb_pitch / 4) * fb_height;
    for (u32 i = 0; i < total; i++)
        dst[i] = color;
    cursor_x = 0;
    cursor_y = 0;
    console_wrapped = false;
    fb_draw_border();              /* repaint the frame over the cleared field */
    fb_mark_dirty(0, fb_height);
    /* The whole field now shows `color`; reset the shadow so identical content
     * drawn next can be skipped (and a real glyph differs in code -> redraws). */
    if (fb_shadow_on) {
        for (u32 k = 0; k < FB_SH_ROWS * FB_SH_COLS; k++) {
            fb_shadow[k].code = ' ';
            fb_shadow[k].fg = color;
            fb_shadow[k].bg = color;
            fb_cell_pending[k] = 0;     /* full repaint supersedes any deferred blank */
        }
        for (u32 r = 0; r < FB_SH_ROWS; r++)
            fb_row_pending[r] = 0;
    }
}

void fb_pixel(u32 x, u32 y, u32 color) {
    if (x < fb_width && y < fb_height) {
        u32 *row = (u32 *)((u8 *)fb_rt() + y * fb_pitch);
        row[x] = color;
        fb_mark_dirty(y, y + 1);
        fb_shadow_invalidate_px(y, y + 1);
    }
}

/* ── Network activity status strip ──
 * Paints a 14x14 block (with 2px gap) advancing across the bottom of the
 * screen, wrapping rows upward when full. Provides a visual liveness +
 * traffic indicator independent of the scrolling text console. */
#define FB_STAT_BLOCK   14
#define FB_STAT_STEP    16
static u32 fb_stat_x;
static u32 fb_stat_y_off;  /* offset from bottom in step units */

void fb_status_block(u32 color) {
    if (!fb_ptr || fb_width == 0 || fb_height == 0) return;

    u32 step = FB_STAT_STEP;
    u32 cx0  = fb_inset_x;                       /* stay inside the L/R border */
    u32 cw   = fb_width - 2u * fb_inset_x;        /* content width */
    u32 cols = cw / step;
    if (cols == 0) return;

    u32 bottom = fb_height - fb_inset_y;          /* just above the bottom border */
    u32 ceil_y = fb_inset_y + (fb_height - 2u * fb_inset_y) / 4u;
    u32 y_base = bottom - step - fb_stat_y_off * step;
    /* If we've climbed past the upper quarter of the content, wrap to bottom. */
    if (y_base < ceil_y) {
        fb_stat_y_off = 0;
        fb_stat_x = 0;
        y_base = bottom - step;
    }

    /* When starting a new horizontal line (first slot in the row), wipe the
     * content-width strip-row to black so old blocks from previous wraps don't
     * smear with the new line of indicators (the border band is left intact). */
    if (fb_stat_x == 0) {
        for (u32 dy = 0; dy < FB_STAT_BLOCK; dy++) {
            u32 *row = (u32 *)((u8 *)fb_rt() + (y_base + dy) * fb_pitch);
            for (u32 dx = 0; dx < cw; dx++)
                row[cx0 + dx] = 0x00000000;
        }
    }

    u32 x_base = cx0 + fb_stat_x * step;
    for (u32 dy = 0; dy < FB_STAT_BLOCK; dy++) {
        u32 *row = (u32 *)((u8 *)fb_rt() + (y_base + dy) * fb_pitch);
        for (u32 dx = 0; dx < FB_STAT_BLOCK; dx++)
            row[x_base + dx] = color;
    }
    fb_mark_dirty(y_base, y_base + FB_STAT_BLOCK);
    fb_shadow_invalidate_px(y_base, y_base + FB_STAT_BLOCK);

    fb_stat_x++;
    if (fb_stat_x >= cols) {
        fb_stat_x = 0;
        fb_stat_y_off++;
    }
}

void fb_set_color(u32 fg, u32 bg) {
    fb_fg = fg;
    fb_bg = bg;
}

void fb_set_cursor(u32 col, u32 row) {
    if (col < cols) cursor_x = col;
    if (row < rows) cursor_y = row;
}

void fb_set_reserved_rows(u32 r) {
    reserved_rows = (r < rows) ? r : 0;
    if (cursor_y < reserved_rows)
        cursor_y = reserved_rows;
}

u32 fb_cols(void) {
    return cols;
}

u32 fb_rows(void) {
    return rows;
}

u32 fb_reserved_rows(void) {
    return reserved_rows;
}

static void fb_clear_text_row(u32 cy) {
    if (!fb_ptr || cy >= rows) return;
    u32 py = fb_inset_y + cy * 8;
    /* Cell-aware clear: repaint only cells not already blank with this bg, so
     * clearing an already-blank row (the common dashboard case) costs nothing.
     * Safe because shadow_on implies cols <= FB_SH_COLS and cy < FB_SH_ROWS. */
    if (fb_shadow_on && cy < FB_SH_ROWS) {
        /* Deferred clear: glyphs are opaque (fb_draw_codepoint paints fg AND bg
         * for the whole 8x8 cell), so clearing a row then redrawing it writes
         * every cell twice. Instead, mark each non-blank cell pending-blank and
         * let fb_present() blank only those NOT redrawn this frame (the shrinking
         * tail). A static row redrawn identically then costs zero pixel writes. */
        bool any = false;
        for (u32 cx = 0; cx < cols; cx++) {
            struct fb_cell *sc = &fb_shadow[cy * FB_SH_COLS + cx];
            if (sc->code == ' ' && sc->fg == fb_bg && sc->bg == fb_bg)
                continue;                 /* already blank with this bg */
            fb_cell_pending[cy * FB_SH_COLS + cx] = 1;
            any = true;
        }
        if (any) {
            fb_row_clear_bg[cy] = fb_bg;
            fb_row_pending[cy] = 1;
        }
        return;
    }
    u32 x0 = fb_inset_x;
    u32 x1 = fb_inset_x + cols * 8;
    for (u32 row = 0; row < 8; row++) {
        u32 *scanline = (u32 *)((u8 *)fb_rt() + (py + row) * fb_pitch);
        for (u32 x = x0; x < x1; x++)
            scanline[x] = fb_bg;
    }
    fb_mark_dirty(py, py + 8);
}

void fb_clear_row(u32 row) {
    fb_clear_text_row(row);
}

static void fb_advance_row(void) {
    cursor_x = 0;
    cursor_y++;
    if (cursor_y >= rows) {
        cursor_y = reserved_rows;
        console_wrapped = true;
    }
    if (console_wrapped)
        fb_clear_text_row(cursor_y);
}

static const u8 *fb_glyph_for_code(u32 code)
{
    if (code >= 32 && code <= 126)
        return font8x8[code - 32];
    switch (code) {
    case 0x2500: case 0x2550: return font8x8_box[FB_GLYPH_H - 95];
    case 0x2502: case 0x2551: return font8x8_box[FB_GLYPH_V - 95];
    case 0x250C: case 0x2554: return font8x8_box[FB_GLYPH_TL - 95];
    case 0x2510: case 0x2557: return font8x8_box[FB_GLYPH_TR - 95];
    case 0x2514: case 0x255A: return font8x8_box[FB_GLYPH_BL - 95];
    case 0x2518: case 0x255D: return font8x8_box[FB_GLYPH_BR - 95];
    case 0x251C: case 0x2560: return font8x8_box[FB_GLYPH_LT - 95];
    case 0x2524: case 0x2563: return font8x8_box[FB_GLYPH_RT - 95];
    case 0x252C: case 0x2566: return font8x8_box[FB_GLYPH_TT - 95];
    case 0x2534: case 0x2569: return font8x8_box[FB_GLYPH_BT - 95];
    case 0x253C: case 0x256C: return font8x8_box[FB_GLYPH_CROSS - 95];
    case 0x2588: return font8x8_box[FB_GLYPH_BLOCK - 95];
    case 0x2591: return font8x8_box[FB_GLYPH_SHADE1 - 95];
    case 0x2592: return font8x8_box[FB_GLYPH_SHADE2 - 95];
    case 0x2593: return font8x8_box[FB_GLYPH_SHADE3 - 95];
    default: return font8x8[0];
    }
}

/* Draw one 8x8 glyph at text position (cx, cy) */
static void fb_draw_codepoint(u32 cx, u32 cy, u32 code) {
    if (!fb_ptr || cols == 0 || rows == 0 || cx >= cols || cy >= rows)
        return;
    struct fb_cell *sc = fb_shadow_at(cx, cy);
    if (sc)
        fb_cell_pending[cy * FB_SH_COLS + cx] = 0;  /* (re)drawn: cancel deferred blank */
    if (sc && sc->code == code && sc->fg == fb_fg && sc->bg == fb_bg)
        return;                       /* identical to what's already on screen */
    const u8 *glyph = fb_glyph_for_code(code);
    u32 px = fb_inset_x + cx * 8;
    u32 py = fb_inset_y + cy * 8;

    for (u32 row = 0; row < 8; row++) {
        u32 *scanline = (u32 *)((u8 *)fb_rt() + (py + row) * fb_pitch);
        u8 bits = glyph[row];
        for (u32 col = 0; col < 8; col++) {
            scanline[px + col] = (bits & (1 << col)) ? fb_fg : fb_bg;
        }
    }
    if (sc) { sc->code = code; sc->fg = fb_fg; sc->bg = fb_bg; }
    fb_mark_dirty(py, py + 8);
}

static void fb_put_codepoint(u32 code) {
    if (!fb_ptr || cols == 0 || rows == 0)
        return;
    if (code == '\n') {
        fb_advance_row();
    } else if (code == '\r') {
        cursor_x = 0;
    } else if (code == '\t') {
        cursor_x = (cursor_x + 4) & ~3;
    } else {
        if (console_wrapped && cursor_x == 0)
            fb_clear_text_row(cursor_y);
        fb_draw_codepoint(cursor_x, cursor_y, code);
        cursor_x++;
    }

    if (cursor_x >= cols) {
        fb_advance_row();
    }
}

void fb_putc(char c) {
    fb_put_codepoint((u8)c);
}

static u32 fb_utf8_next(const char **ps)
{
    const u8 *s = (const u8 *)*ps;
    u8 c = *s++;
    if (c < 0x80) {
        *ps = (const char *)s;
        return c;
    }
    if ((c & 0xE0) == 0xC0 && (s[0] & 0xC0) == 0x80) {
        u32 cp = ((u32)(c & 0x1F) << 6) | (u32)(s[0] & 0x3F);
        *ps = (const char *)(s + 1);
        return cp;
    }
    if ((c & 0xF0) == 0xE0 && (s[0] & 0xC0) == 0x80 && (s[1] & 0xC0) == 0x80) {
        u32 cp = ((u32)(c & 0x0F) << 12) |
                 ((u32)(s[0] & 0x3F) << 6) |
                 (u32)(s[1] & 0x3F);
        *ps = (const char *)(s + 2);
        return cp;
    }
    *ps = (const char *)s;
    return '?';
}

void fb_puts(const char *s) {
    while (s && *s)
        fb_put_codepoint(fb_utf8_next(&s));
}

void fb_hline(u32 n)
{
    for (u32 i = 0; i < n; i++)
        fb_put_codepoint(0x2500);
}

void fb_box(u32 width, u32 height, const char *title)
{
    if (width < 4 || height < 2)
        return;
    fb_put_codepoint(0x250C);
    fb_hline(width - 2);
    fb_put_codepoint(0x2510);
    fb_putc('\n');
    for (u32 row = 1; row + 1 < height; row++) {
        fb_put_codepoint(0x2502);
        if (row == 1 && title) {
            fb_putc(' ');
            u32 t = 0;
            while (title[t] && t + 3 < width) {
                fb_putc(title[t]);
                t++;
            }
            while (t + 3 < width) {
                fb_putc(' ');
                t++;
            }
            fb_putc(' ');
        } else {
            for (u32 col = 0; col < width - 2; col++)
                fb_putc(' ');
        }
        fb_put_codepoint(0x2502);
        fb_putc('\n');
    }
    fb_put_codepoint(0x2514);
    fb_hline(width - 2);
    fb_put_codepoint(0x2518);
    fb_putc('\n');
}

void fb_box_at(u32 col, u32 row, u32 width, u32 height, const char *title)
{
    if (width < 4 || height < 2)
        return;
    if (col >= cols || row >= rows)
        return;
    if (col + width > cols)
        width = cols - col;
    if (row + height > rows)
        height = rows - row;
    if (width < 4 || height < 2)
        return;

    fb_set_cursor(col, row);
    fb_put_codepoint(0x250C);
    fb_hline(width - 2);
    fb_put_codepoint(0x2510);

    if (title) {
        fb_set_cursor(col + 2U, row);
        fb_putc(' ');
        for (u32 i = 0; title[i] && i + 5U < width; i++)
            fb_putc(title[i]);
        fb_putc(' ');
    }

    for (u32 r = row + 1U; r + 1U < row + height; r++) {
        fb_set_cursor(col, r);
        fb_put_codepoint(0x2502);
        fb_set_cursor(col + width - 1U, r);
        fb_put_codepoint(0x2502);
    }

    fb_set_cursor(col, row + height - 1U);
    fb_put_codepoint(0x2514);
    fb_hline(width - 2);
    fb_put_codepoint(0x2518);
}

/* Minimal printf: %d %u %x %s %c %% */
static void fb_print_u32(u32 val) {
    char buf[11];
    int i = 0;
    if (val == 0) { fb_putc('0'); return; }
    while (val) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (--i >= 0) fb_putc(buf[i]);
}

static void fb_print_i32(i32 val) {
    if (val < 0) { fb_putc('-'); val = -val; }
    fb_print_u32((u32)val);
}

static void fb_print_hex(u64 val, int digits) {
    static const char hex[] = "0123456789ABCDEF";
    for (int i = (digits - 1) * 4; i >= 0; i -= 4)
        fb_putc(hex[(val >> i) & 0xF]);
}

void fb_printf(const char *fmt, ...) {
    __builtin_va_list args;
    __builtin_va_start(args, fmt);

    while (*fmt) {
        if (*fmt != '%') {
            fb_putc(*fmt++);
            continue;
        }
        fmt++;
        switch (*fmt) {
        case 'd': fb_print_i32(__builtin_va_arg(args, i32)); break;
        case 'u': fb_print_u32(__builtin_va_arg(args, u32)); break;
        case 'x': fb_print_hex(__builtin_va_arg(args, u32), 8); break;
        case 'X': fb_print_hex(__builtin_va_arg(args, u64), 16); break;
        case 's': fb_puts(__builtin_va_arg(args, const char *)); break;
        case 'c': fb_putc((char)__builtin_va_arg(args, int)); break;
        case '%': fb_putc('%'); break;
        default:  fb_putc('%'); fb_putc(*fmt); break;
        }
        fmt++;
    }

    __builtin_va_end(args);
}

u64 fb_get_phys_addr(void) {
    return (u64)(usize)fb_ptr;
}

/* Diagnostics: is double-buffering actually active, and the FB geometry. */
void fb_debug_info(u32 *db, u32 *size, u32 *pitch) {
    if (size)  *size = fb_size;
    if (pitch) *pitch = fb_pitch;
#ifndef PIOS_FB_NO_DOUBLE_BUFFER
    if (db) *db = fb_db ? 1U : 0U;
#else
    if (db) *db = 0U;
#endif
}

u64 fb_last_blit_ticks(void) {
#ifndef PIOS_FB_NO_DOUBLE_BUFFER
    return g_fb_blit_ticks;
#else
    return 0;
#endif
}

/* ── Double-buffer present ──
 * Blit the dirty pixel-row band from the cached back buffer to the VideoCore
 * scanout buffer. The back slice is cleaned to RAM so the DMA engine (or NEON
 * fallback before dma_init) reads current pixels; the front slice is then
 * cleaned to RAM so the scanout sees the result. Per the transfer rule, a
 * multi-row band is "bulk" and goes through DMA; a tiny band falls to NEON. */
#ifndef PIOS_FB_NO_DOUBLE_BUFFER
static void fb_blit(u32 off, u32 bytes) {
    u64 t0; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t0));
    u8 *src = (u8 *)fb_back + off;
    u8 *dst = (u8 *)fb_ptr  + off;
    /* Blit via CPU/NEON. This is the proven write path to the VideoCore
     * scanout buffer: the dma32 engine's view of the firmware framebuffer
     * address does not land on the scanned-out buffer, so a DMA blit leaves
     * the display frozen. The win is the cached back buffer (fast rendering);
     * the front copy is one bulk sequential NEON pass over only the dirty
     * rows. The clean pushes the band to RAM so the scanout sees it whether
     * the FB is mapped cacheable or not. */
    simd_memcpy(dst, src, bytes);
    dcache_clean_range((u64)(usize)dst, bytes);
    dsb();
    u64 t1; __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t1));
    g_fb_blit_ticks = t1 - t0;
}

/* Blank cells that were cleared (deferred) but not redrawn this frame — the
 * shrinking tail of shortened text — then mark their rows dirty for the blit.
 * Glyphs are opaque, so a redrawn cell already overwrote its old content and
 * cleared its pending flag in fb_draw_codepoint(); only true leftovers remain. */
static void fb_flush_pending(void) {
    if (!fb_shadow_on) return;
    for (u32 cy = 0; cy < FB_SH_ROWS; cy++) {
        if (!fb_row_pending[cy]) continue;
        fb_row_pending[cy] = 0;
        u32 py = fb_inset_y + cy * 8;
        u32 bg = fb_row_clear_bg[cy];
        bool any = false;
        for (u32 cx = 0; cx < FB_SH_COLS; cx++) {
            u32 idx = cy * FB_SH_COLS + cx;
            if (!fb_cell_pending[idx]) continue;
            fb_cell_pending[idx] = 0;
            u32 px = fb_inset_x + cx * 8;
            for (u32 r = 0; r < 8; r++) {
                u32 *scanline = (u32 *)((u8 *)fb_rt() + (py + r) * fb_pitch);
                for (u32 c = 0; c < 8; c++)
                    scanline[px + c] = bg;
            }
            struct fb_cell *sc = &fb_shadow[idx];
            sc->code = ' '; sc->fg = bg; sc->bg = bg;
            any = true;
        }
        if (any) fb_mark_dirty(py, py + 8);
    }
}

void fb_present(void) {
    if (!fb_db)
        return;

    fb_flush_pending();

    /* Blit each maximal run of consecutive dirty bands as one sequential pass,
     * clearing the bits as we go. With the cell cache, a typical dashboard tick
     * dirties only a few scattered rows, so this replaces one ~33-row span with
     * a handful of small blits. */
    u32 nbands = (fb_height + FB_BAND_PX - 1u) / FB_BAND_PX;
    if (nbands > FB_MAX_BANDS) nbands = FB_MAX_BANDS;
    u32 b = 0;
    while (b < nbands) {
        if (!(fb_dirty_band[b >> 6] & (1ULL << (b & 63u)))) { b++; continue; }
        u32 run0 = b;
        while (b < nbands && (fb_dirty_band[b >> 6] & (1ULL << (b & 63u)))) {
            fb_dirty_band[b >> 6] &= ~(1ULL << (b & 63u));
            b++;
        }
        u32 y0 = run0 * FB_BAND_PX;
        u32 y1 = b * FB_BAND_PX;
        if (y1 > fb_height) y1 = fb_height;
        u32 off = y0 * fb_pitch;
        if (off >= fb_size) break;
        u32 bytes = (y1 - y0) * fb_pitch;
        if (off + bytes > fb_size) bytes = fb_size - off;
        if (bytes)
            fb_blit(off, bytes);
    }
}
#else
void fb_present(void) { }
#endif
