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
    return f;
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
            if (!resolve_end_condition(b, BREP_NONE, &skf->sketch.plane,
                                       &pr, f->p.extrude.dist)) return 0;
            f->result = op_extrude(b, &skf->sketch, &pr, BREP_NONE, f->id);
            break;
        }
        case FEAT_REVOLVE: {
            Feature *skf = doc_find(d, f->p.revolve.sketch_id);
            if (!skf) return 0;
            OpParams pr; pr.op = f->p.revolve.op; pr.end = END_BLIND;
            pr.dir = 1; pr.target = BREP_NONE; pr.dist = 0;
            Vec3i ax_o = {0,0,0}, ax_d = {0,FX_ONE,0};   /* MVP: revolve about Y */
            f->result = op_revolve(b, &skf->sketch, &pr, ax_o, ax_d,
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
