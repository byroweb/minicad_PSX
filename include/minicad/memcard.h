/* memcard.h — PS1-ONLY on-console memory-card save/load for MiniCAD parts.
 *
 * Writes a standard single-block PS1 save whose payload (frames 4..) is the
 * compact .mcad byte stream produced by mcad_encode(). The save carries a
 * proper title frame (SC magic + Shift-JIS title) and a 16x16 16-colour icon so
 * the BIOS/DuckStation save browser shows it, and the on-card directory name
 * starts with the MiniCAD product code so tools/rip_mcad.py extracts it.
 *
 * One block (8 KB) total — the .mcad payload is tiny (~94 B for the demo part).
 *
 * port : 0 = card slot 1, 1 = card slot 2.
 * slot : reserved for a future multi-file layout; pass 0 (uses one fixed name).
 * name : up to ~7 chars appended to the product code for the on-card filename
 *        (NULL -> a default). Keep it A-Z0-9.
 */
#ifndef MINICAD_MEMCARD_H
#define MINICAD_MEMCARD_H

#include <stdint.h>

#ifdef MINICAD_PSX

/* Initialise the BIOS memory-card driver. Safe to call once at boot (after
 * InitPAD/StartPAD — InitCARD takes a "pads also enabled" flag). */
void mc_init(void);

/* Write `len` bytes of .mcad payload to a single-block save on (port,slot).
 * Returns 0 on success, <0 on error. Blocks briefly during the card write. */
int  mc_save(int port, int slot, const char *name, const uint8_t *data, int len);

/* Read the .mcad payload from the save on (port,slot) into `out` (capacity
 * `cap`). Returns bytes read (>0) on success, <0 on error / not found. */
int  mc_load(int port, int slot, const char *name, uint8_t *out, int cap);

#endif /* MINICAD_PSX */
#endif /* MINICAD_MEMCARD_H */
