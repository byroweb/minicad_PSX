/* memcard.c — PS1-ONLY on-console memory-card save/load.
 *
 * Uses the BIOS file API (open/read/write/close on the "bu00:"/"bu10:"
 * filesystem). The BIOS owns the on-card directory frame (block 0) and the
 * block-chain; we hand it a full single-block save IMAGE whose internal layout
 * is the standard PS1 save:
 *
 *   frame 0   (0x000): title frame  — "SC" magic, icon flag, block count,
 *                       Shift-JIS title, and the 16-colour icon CLUT at 0x60.
 *   frame 1   (0x080): 16x16 4bpp icon pixel data (one frame).
 *   frames 2-3(0x100): reserved/padding (kept zero).
 *   frames 4+ (0x200): the .mcad byte stream (this is what rip_mcad.py reads:
 *                       block_payload() returns frames 4..63 of the first block).
 *
 * The on-card directory NAME is the filename passed to open() (after "bu00:"),
 * which we build to start with the product code "BASLUS-MCAD" so rip_mcad.py
 * (PRODUCT_PREFIX = b"BASLUS-MCAD") recognises and extracts the save.
 *
 * Everything is integer-only and one block (8 KB); the .mcad payload is ~94 B.
 */
#ifdef MINICAD_PSX

#include <psxapi.h>
#include <stdint.h>
#include <stddef.h>
#include "minicad/memcard.h"

#ifndef FCREATE
#define FCREATE   0x200
#endif
#ifndef FREAD
#define FREAD     0x1
#endif
#ifndef FWRITE
#define FWRITE    0x2
#endif

#define FRAME      128
#define BLOCK      8192
#define PAYLOAD_OFF (4 * FRAME)              /* frames 4.. = .mcad payload */
#define PAYLOAD_CAP (BLOCK - PAYLOAD_OFF)

/* Product code the ripper keys on. Final on-card name = PRODUCT + suffix. */
#define MC_PRODUCT "BASLUS-MCAD"

/* The save icon, if the artist asset exists (CMake incbin's assets/icon.tim and
 * defines MINICAD_HAVE_ICON). We parse the small TIM at runtime to lift its
 * CLUT + 4bpp pixels into the save header. */
#ifdef MINICAD_HAVE_ICON
extern const uint8_t icon[];
extern const uint8_t icon_end[];
#endif

/* One static block-sized scratch buffer (8 KB) reused by save and load so we
 * never put 8 KB on the stack. */
static uint8_t s_block[BLOCK];

static void mc_memset(uint8_t *p, uint8_t v, int n) {
    for (int i = 0; i < n; ++i) p[i] = v;
}
static void mc_memcpy(uint8_t *d, const uint8_t *s, int n) {
    for (int i = 0; i < n; ++i) d[i] = s[i];
}

/* Build "bu00:" / "bu10:" + MC_PRODUCT + suffix into `path`. */
static void build_path(char *path, int port, const char *name) {
    const char *dev = (port == 0) ? "bu00:" : "bu10:";
    int o = 0;
    for (const char *p = dev;       *p; ++p) path[o++] = *p;
    for (const char *p = MC_PRODUCT; *p; ++p) path[o++] = *p;
    /* default suffix keeps the directory name well under 20 chars */
    const char *suf = (name && name[0]) ? name : "01PART01";
    for (const char *p = suf; *p && o < 5 + 20 - 1; ++p) path[o++] = *p;
    path[o] = '\0';
}

/* Fill the standard PS1 title frame (frame 0) + icon (frame 1) of s_block. */
static void build_header(const char *title) {
    /* --- frame 0: title frame --- */
    s_block[0] = 'S';
    s_block[1] = 'C';
    s_block[2] = 0x11;            /* icon display flag: 1 icon frame */
    s_block[3] = 1;               /* block count = 1 (single-block save) */

    /* Title at 0x04 (Shift-JIS). ASCII is a subset, so plain bytes are fine for
     * a browser to show. 64 bytes available. */
    int o = 4;
    for (const char *p = title; *p && o < 4 + 60; ++p) s_block[o++] = (uint8_t)*p;

#ifdef MINICAD_HAVE_ICON
    /* The TIM: u32 magic(0x10), u32 flags, CLUT block, pixel block.
     * CLUT block: u32 len, u16 x,y, u16 w,h, then 16 x u16 BGR555.
     * Pixel block: u32 len, u16 x,y, u16 w,h, then 4bpp pixels. */
    const uint8_t *t = icon;
    /* CLUT: 16 colours start at offset 8 (header) + 12 (clut block header). */
    const uint8_t *clut = t + 8 + 12;
    /* pixel block follows the CLUT block: its length is at t+8. */
    uint32_t clut_len = (uint32_t)t[8] | ((uint32_t)t[9] << 8) |
                        ((uint32_t)t[10] << 16) | ((uint32_t)t[11] << 24);
    const uint8_t *pixblk = t + 8 + clut_len;        /* start of pixel block */
    const uint8_t *pix = pixblk + 12;                /* skip its 12B header */

    /* Icon CLUT lives at frame-0 offset 0x60: 16 x u16. */
    mc_memcpy(s_block + 0x60, clut, 32);
    /* Icon image: 16x16 4bpp = 128 bytes -> frame 1 (offset 0x80). */
    mc_memcpy(s_block + 0x80, pix, 128);
#else
    /* No TIM asset: paint a minimal solid-blue icon so the browser still shows
     * a tile. CLUT entry 0 = a steel-blue BGR555; all pixels = index 0. */
    uint16_t blue = (uint16_t)((11 << 10) | (7 << 5) | 5);   /* ~steel blue */
    s_block[0x60] = (uint8_t)(blue & 0xFF);
    s_block[0x61] = (uint8_t)(blue >> 8);
    for (int i = 0x80; i < 0x80 + 128; ++i) s_block[i] = 0x00;
#endif
}

void mc_init(void) {
    /* InitCARD's arg: nonzero keeps the pad driver running alongside the card. */
    InitCARD(1);
    StartCARD();
    /* _bu_init wires up the "bu" filesystem driver for open()/read()/write(). */
    _bu_init();
}

int mc_save(int port, int slot, const char *name, const uint8_t *data, int len) {
    (void)slot;
    if (!data || len <= 0 || len > PAYLOAD_CAP) return -1;

    char path[40];
    build_path(path, port, name);

    /* Assemble the full single-block save image. */
    mc_memset(s_block, 0, BLOCK);
    build_header("MINICAD PART");
    mc_memcpy(s_block + PAYLOAD_OFF, data, len);

    /* Create (or truncate) a 1-block file. The block count rides the high word
     * of the open mode per the BIOS file API (FNBLOCKS / FCREATE). */
    int fd = open(path, FCREATE | (1 << 16));
    if (fd < 0) {
        /* maybe it already exists: open for writing without create */
        fd = open(path, FWRITE);
        if (fd < 0) return -2;
    }
    int wrote = write(fd, s_block, BLOCK);
    close(fd);
    if (wrote != BLOCK) return -3;
    return 0;
}

int mc_load(int port, int slot, const char *name, uint8_t *out, int cap) {
    (void)slot;
    if (!out || cap <= 0) return -1;

    char path[40];
    build_path(path, port, name);

    int fd = open(path, FREAD);
    if (fd < 0) return -2;
    int got = read(fd, s_block, BLOCK);
    close(fd);
    if (got < PAYLOAD_OFF + 12) return -3;     /* not even a header+payload */

    const uint8_t *p = s_block + PAYLOAD_OFF;
    /* sanity: must be an MCAD payload */
    if (p[0] != 'M' || p[1] != 'C' || p[2] != 'A' || p[3] != 'D') return -4;
    int plen = 12 + (p[6] | (p[7] << 8));      /* header + body length */
    if (plen > PAYLOAD_CAP) plen = PAYLOAD_CAP;
    if (plen > cap) plen = cap;
    mc_memcpy(out, p, plen);
    return plen;
}

#endif /* MINICAD_PSX */
