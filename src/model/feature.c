/* feature.c — document + feature tree management and regeneration. */
#include "minicad/feature.h"

static void str_copy(char *dst, const char *src, int cap) {
    int i = 0;
    if (src) for (; src[i] && i < cap - 1; ++i) dst[i] = src[i];
    dst[i] = '\0';
}

void doc_init(Document *d, const char *title) {
    d->feat_count = 0;
    d->next_id    = 1;
    str_copy(d->title, title ? title : "Part1", FEAT_NAME_LEN);
}

Feature *doc_add(Document *d, FeatureKind kind, const char *name) {
    if (d->feat_count >= DOC_MAX_FEATURES) return 0;
    Feature *f = &d->feat[d->feat_count++];
    f->kind       = kind;
    f->id         = d->next_id++;
    f->dep_count  = 0;
    f->dirty      = 1;
    f->suppressed = 0;
    f->result     = BREP_NONE;
    str_copy(f->name, name, FEAT_NAME_LEN);
    if (kind == FEAT_SKETCH) { sk_init(&f->sketch); f->plane = plane_xy(); }
    return f;
}

/* Return the prefix string used by SolidWorks-style auto naming for a given
 * (kind, op). `op` matters only for extrude/revolve. */
static const char *autoname_prefix(FeatureKind kind, OpType op) {
    switch (kind) {
    case FEAT_SKETCH:    return "Sketch";
    case FEAT_EXTRUDE:   return op == OP_CUT ? "Cut-Extrude" : "Boss-Extrude";
    case FEAT_REVOLVE:   return op == OP_CUT ? "Cut-Revolve" : "Boss-Revolve";
    case FEAT_REF_PLANE: return "Plane";
    case FEAT_REF_AXIS:  return "Axis";
    case FEAT_REF_POINT: return "Point";
    default:             return "Feature";
    }
}

/* If `name` is exactly `prefix` followed by one or more decimal digits (and
 * nothing else), return that integer (>= 0); otherwise return -1. */
static long name_index_for_prefix(const char *name, const char *prefix) {
    int i = 0;
    for (; prefix[i]; ++i)
        if (name[i] != prefix[i]) return -1;     /* prefix mismatch */
    if (name[i] == '\0') return -1;              /* prefix with no trailing digits */
    long v = 0;
    for (; name[i]; ++i) {
        if (name[i] < '0' || name[i] > '9') return -1;  /* non-digit suffix */
        v = v * 10 + (name[i] - '0');
    }
    return v;
}

/* Append the decimal `n` (>= 0) to the NUL-terminated string in `out` without
 * overflowing `cap`. */
static void append_uint(char *out, int cap, long n) {
    int len = 0;
    while (out[len]) ++len;
    char tmp[12];
    int t = 0;
    if (n == 0) tmp[t++] = '0';
    while (n > 0 && t < (int)sizeof tmp) { tmp[t++] = (char)('0' + (n % 10)); n /= 10; }
    while (t > 0 && len < cap - 1) out[len++] = tmp[--t];
    out[len] = '\0';
}

void feature_autoname(const Document *d, FeatureKind kind, OpType op,
                      char *out, int cap) {
    if (cap <= 0) return;
    out[0] = '\0';
    if (cap == 1) return;
    const char *prefix = autoname_prefix(kind, op);

    long maxidx = 0;          /* highest existing index for this prefix (0 = none) */
    if (d) {
        for (uint16_t i = 0; i < d->feat_count; ++i) {
            long idx = name_index_for_prefix(d->feat[i].name, prefix);
            if (idx > maxidx) maxidx = idx;
        }
    }

    str_copy(out, prefix, cap);
    append_uint(out, cap, maxidx + 1);
}

Feature *doc_find(Document *d, uint16_t id) {
    for (uint16_t i = 0; i < d->feat_count; ++i)
        if (d->feat[i].id == id) return &d->feat[i];
    return 0;
}

/* Mark a feature dirty, then any feature that (transitively) depends on it. */
void doc_mark_dirty(Document *d, uint16_t id) {
    Feature *f = doc_find(d, id);
    if (!f || f->dirty) return;
    f->dirty = 1;
    for (uint16_t i = 0; i < d->feat_count; ++i) {
        Feature *g = &d->feat[i];
        for (uint8_t k = 0; k < g->dep_count; ++k)
            if (g->depends_on[k] == id) { doc_mark_dirty(d, g->id); break; }
    }
}

int doc_regen(Document *d, Brep *b) {
    /* Features are kept in topological order by construction (you may only
     * depend on earlier features), so a single forward pass suffices. */
    for (uint16_t i = 0; i < d->feat_count; ++i) {
        Feature *f = &d->feat[i];
        if (f->suppressed) { f->result = BREP_NONE; continue; }
        if (!f->dirty) continue;

        switch (f->kind) {
        case FEAT_SKETCH:
            f->result = BREP_NONE;             /* sketches produce no solid */
            break;
        case FEAT_EXTRUDE: {
            Feature *skf = doc_find(d, f->p.extrude.sketch_id);
            if (!skf) return 0;
            OpParams pr;
            pr.op     = f->p.extrude.op;
            pr.end    = f->p.extrude.end;
            pr.dir    = f->p.extrude.dir;
            pr.target = f->p.extrude.target_face ? f->p.extrude.target_face : BREP_NONE;
            if (!resolve_end_condition(b, BREP_NONE, &skf->plane,
                                       &pr, f->p.extrude.dist)) return 0;
            f->result = op_extrude(b, &skf->sketch, &skf->plane, &pr,
                                   BREP_NONE, f->id);
            break;
        }
        case FEAT_REVOLVE: {
            Feature *skf = doc_find(d, f->p.revolve.sketch_id);
            if (!skf) return 0;
            OpParams pr; pr.op = f->p.revolve.op; pr.end = END_BLIND;
            pr.dir = 1; pr.target = BREP_NONE; pr.dist = 0;
            Vec3i ax_o = {0,0,0}, ax_d = {0,FX_ONE,0};   /* MVP: revolve about Y */
            f->result = op_revolve(b, &skf->sketch, &skf->plane, &pr, ax_o, ax_d,
                                   f->p.revolve.sweep, f->p.revolve.steps,
                                   BREP_NONE, f->id);
            break;
        }
        case FEAT_REF_PLANE:
        case FEAT_REF_AXIS:
        case FEAT_REF_POINT:
            /* Datums produce no B-rep; their geometry is derived on demand by
             * downstream features. TODO: store the computed datum transform. */
            f->result = BREP_NONE;
            break;
        default: break;
        }
        f->dirty = 0;
    }
    return 1;
}
