/* save.c — compact .mcad encode/decode + varint + crc32. */
#include "minicad/save.h"

/* ---------------- varint ---------------- */
int vu_write(uint8_t *p, uint32_t v) {
    int n = 0;
    do { uint8_t b = v & 0x7F; v >>= 7; if (v) b |= 0x80; p[n++] = b; } while (v);
    return n;
}
int vu_read(const uint8_t *p, uint32_t *out) {
    uint32_t v = 0; int n = 0, shift = 0; uint8_t b;
    do { b = p[n++]; v |= (uint32_t)(b & 0x7F) << shift; shift += 7; } while (b & 0x80);
    *out = v; return n;
}
int vs_write(uint8_t *p, int32_t v) {                 /* zig-zag */
    uint32_t z = ((uint32_t)v << 1) ^ (uint32_t)(v >> 31);
    return vu_write(p, z);
}
int vs_read(const uint8_t *p, int32_t *out) {
    uint32_t z; int n = vu_read(p, &z);
    *out = (int32_t)((z >> 1) ^ (~(z & 1) + 1));
    return n;
}

/* ---------------- crc32 (IEEE, no table to save RAM; small + fine) ---- */
uint32_t mcad_crc32(const uint8_t *p, int len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (int i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
    }
    return ~crc;
}

/* ---------------- encode ---------------- */
/* Header is 12 bytes: 'MCAD', ver, feat_count, payload_len(u16 LE), crc32(LE).
 * crc covers the payload only. */
int mcad_encode(const Document *d, uint8_t *buf, int cap) {
    if (cap < 12) return 0;
    uint8_t *payload = buf + 12;
    int o = 0;

    for (uint16_t i = 0; i < d->feat_count; ++i) {
        const Feature *f = &d->feat[i];
        if (o + 16 > cap - 12) return 0;           /* leave slack */
        payload[o++] = (uint8_t)f->kind;
        payload[o++] = (uint8_t)((f->suppressed ? 1 : 0) | (f->dep_count << 1));
        o += vu_write(payload + o, f->id);
        for (uint8_t k = 0; k < f->dep_count; ++k)
            o += vu_write(payload + o, f->depends_on[k]);
        switch (f->kind) {
        case FEAT_EXTRUDE:
            o += vu_write(payload + o, f->p.extrude.sketch_id);
            o += vs_write(payload + o, f->p.extrude.dist);
            payload[o++] = (uint8_t)((f->p.extrude.dir >= 0 ? 1 : 0)
                                     | ((uint8_t)f->p.extrude.op << 1)
                                     | ((uint8_t)f->p.extrude.end << 2));
            o += vu_write(payload + o, f->p.extrude.target_face);
            break;
        case FEAT_REVOLVE:
            o += vu_write(payload + o, f->p.revolve.sketch_id);
            o += vs_write(payload + o, f->p.revolve.sweep);
            o += vu_write(payload + o, (uint32_t)f->p.revolve.steps);
            payload[o++] = (uint8_t)f->p.revolve.op;
            break;
        case FEAT_REF_PLANE: case FEAT_REF_AXIS: case FEAT_REF_POINT:
            payload[o++] = (uint8_t)f->p.ref.method;
            o += vs_write(payload + o, f->p.ref.offset);
            break;
        case FEAT_SKETCH:
            /* sketch entities packed compactly */
            payload[o++] = f->sketch.count;
            for (uint8_t e = 0; e < f->sketch.count; ++e) {
                const SketchEntity *se = &f->sketch.ent[e];
                payload[o++] = (uint8_t)se->kind;
                o += vs_write(payload + o, se->a.u);
                o += vs_write(payload + o, se->a.v);
                o += vs_write(payload + o, se->b.u);
                o += vs_write(payload + o, se->b.v);
                o += vs_write(payload + o, se->radius);
            }
            break;
        default: break;
        }
    }

    uint32_t crc = mcad_crc32(payload, o);
    buf[0]=MCAD_MAGIC0; buf[1]=MCAD_MAGIC1; buf[2]=MCAD_MAGIC2; buf[3]=MCAD_MAGIC3;
    buf[4]=MCAD_VERSION;
    buf[5]=(uint8_t)d->feat_count;
    buf[6]=(uint8_t)(o & 0xFF); buf[7]=(uint8_t)((o >> 8) & 0xFF);
    buf[8]= (uint8_t)(crc & 0xFF);        buf[9]= (uint8_t)((crc>>8)&0xFF);
    buf[10]=(uint8_t)((crc>>16)&0xFF);    buf[11]=(uint8_t)((crc>>24)&0xFF);
    return 12 + o;
}

/* ---------------- decode ---------------- */
int mcad_decode(Document *d, const uint8_t *buf, int len) {
    if (len < 12) return 0;
    if (buf[0]!=MCAD_MAGIC0||buf[1]!=MCAD_MAGIC1||buf[2]!=MCAD_MAGIC2||buf[3]!=MCAD_MAGIC3)
        return 0;
    uint8_t count = buf[5];
    int payload_len = buf[6] | (buf[7] << 8);
    if (12 + payload_len > len) return 0;
    uint32_t crc = (uint32_t)buf[8] | ((uint32_t)buf[9]<<8) |
                   ((uint32_t)buf[10]<<16) | ((uint32_t)buf[11]<<24);
    const uint8_t *payload = buf + 12;
    if (mcad_crc32(payload, payload_len) != crc) return 0;

    doc_init(d, "Part1");
    int o = 0;
    for (uint8_t i = 0; i < count; ++i) {
        FeatureKind kind = (FeatureKind)payload[o++];
        uint8_t flags = payload[o++];
        Feature *f = doc_add(d, kind, "");
        if (!f) return 0;
        f->suppressed = flags & 1;
        f->dep_count  = (flags >> 1) & 0x7F;
        uint32_t tmp; o += vu_read(payload + o, &tmp); f->id = (uint16_t)tmp;
        for (uint8_t k = 0; k < f->dep_count; ++k) {
            o += vu_read(payload + o, &tmp); f->depends_on[k] = (uint16_t)tmp;
        }
        switch (kind) {
        case FEAT_EXTRUDE: {
            o += vu_read(payload + o, &tmp); f->p.extrude.sketch_id=(uint16_t)tmp;
            o += vs_read(payload + o, &f->p.extrude.dist);
            uint8_t bb = payload[o++];
            f->p.extrude.dir = (bb & 1) ? 1 : -1;
            f->p.extrude.op  = (OpType)((bb >> 1) & 1);
            f->p.extrude.end = (EndCond)((bb >> 2) & 3);
            o += vu_read(payload + o, &tmp); f->p.extrude.target_face=(uint16_t)tmp;
            break;
        }
        case FEAT_REVOLVE: {
            o += vu_read(payload + o, &tmp); f->p.revolve.sketch_id=(uint16_t)tmp;
            o += vs_read(payload + o, &f->p.revolve.sweep);
            o += vu_read(payload + o, &tmp); f->p.revolve.steps=(int16_t)tmp;
            f->p.revolve.op = (OpType)payload[o++];
            break;
        }
        case FEAT_REF_PLANE: case FEAT_REF_AXIS: case FEAT_REF_POINT:
            f->p.ref.method = (RefMethod)payload[o++];
            o += vs_read(payload + o, &f->p.ref.offset);
            break;
        case FEAT_SKETCH: {
            f->sketch.count = payload[o++];
            for (uint8_t e = 0; e < f->sketch.count; ++e) {
                SketchEntity *se = &f->sketch.ent[e];
                se->kind = (SketchKind)payload[o++];
                o += vs_read(payload + o, &se->a.u);
                o += vs_read(payload + o, &se->a.v);
                o += vs_read(payload + o, &se->b.u);
                o += vs_read(payload + o, &se->b.v);
                o += vs_read(payload + o, &se->radius);
            }
            break;
        }
        default: break;
        }
    }
    return 1;
}
