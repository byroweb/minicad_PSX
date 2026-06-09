/* save.h — compact .mcad codec. THE FEATURE TREE IS THE FILE.
 *
 * A parametric part is a recipe, not a mesh: serialize the feature tree's
 * parameters (myriometer integers, varint-coded) and regenerate geometry on
 * load. A flanged bearing is ~20 bytes. Layout is frame-aligned (128 B) so it
 * drops onto a memory-card block and the Python ripper can parse it exactly.
 */
#ifndef MINICAD_SAVE_H
#define MINICAD_SAVE_H

#include "minicad/feature.h"

#define MCAD_MAGIC0 'M'
#define MCAD_MAGIC1 'C'
#define MCAD_MAGIC2 'A'
#define MCAD_MAGIC3 'D'
#define MCAD_VERSION 2   /* v2: FEAT_SKETCH stores a parametric Sketch2 */

/* Encode a Document into `buf` (capacity `cap`). Returns bytes written, or
 * 0 on overflow. Header: magic[4], u8 version, u8 feature_count,
 * u16 payload_len, u32 crc32, then the varint feature stream. */
int  mcad_encode(const Document *d, uint8_t *buf, int cap);

/* Decode `buf` (len bytes) into `d`. Returns 1 on success (magic+crc ok). */
int  mcad_decode(Document *d, const uint8_t *buf, int len);

/* crc32 (IEEE) over a byte range — shared by encoder, decoder, and ripper. */
uint32_t mcad_crc32(const uint8_t *p, int len);

/* varint helpers (LEB128, unsigned) — small values cost 1 byte. */
int vu_write(uint8_t *p, uint32_t v);    /* returns bytes written */
int vu_read (const uint8_t *p, uint32_t *out);
/* zig-zag signed wrappers for myriometer params that may be negative. */
int vs_write(uint8_t *p, int32_t v);
int vs_read (const uint8_t *p, int32_t *out);

#endif /* MINICAD_SAVE_H */
