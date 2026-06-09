/* modeling.c — portable interactive-modeling core (host + PSX).
 * See modeling.h. Integer / fixed-point math only (no FPU). */
#include "minicad/modeling.h"

/* ---- small integer helpers (no float, no libm) ------------------------- */

/* Integer sqrt of a 64-bit value (floor). */
static mym2_t isqrt64(mym2_t x) {
    if (x <= 0) return 0;
    mym2_t lo = 0, hi = x < FX_ONE ? FX_ONE : x, r = 0;
    while (lo <= hi) {
        mym2_t m = lo + ((hi - lo) >> 1);
        if (m * m <= x) { r = m; lo = m + 1; } else { hi = m - 1; }
    }
    return r;
}

/* Scale a vector to magnitude ~FX_ONE (a 1.12 unit direction). Zero vector
 * stays zero. */
static Vec3i v3_unit_fx(Vec3i v) {
    mym2_t len = isqrt64((mym2_t)v.x * v.x + (mym2_t)v.y * v.y + (mym2_t)v.z * v.z);
    Vec3i r = { 0, 0, 0 };
    if (len == 0) return r;
    r.x = (mym_t)(((mym2_t)v.x * FX_ONE) / len);
    r.y = (mym_t)(((mym2_t)v.y * FX_ONE) / len);
    r.z = (mym_t)(((mym2_t)v.z * FX_ONE) / len);
    return r;
}

void model_init(ModelingState *m) {
    m->pending    = 0;
    m->sketch_id  = 0;
    m->extrude_id = 0;
    m->op         = OP_BOSS;
    m->dist       = MODEL_DEF_DIST;
}

/* ---- plane from face --------------------------------------------------- */

SketchPlane plane_from_face(Brep *b, FaceId f) {
    SketchPlane pl = plane_xy();   /* sane fallback */
    Face *face = brep_face(b, f);
    if (!face) return pl;

    /* Centroid = mean of the outer loop's vertex positions. */
    Loop *lp = (Loop *)pool_get(&b->loops, face->outer);
    if (!lp) return pl;
    HEdgeId h0 = lp->first_he, h = h0;
    mym2_t sx = 0, sy = 0, sz = 0;
    int n = 0, guard = 0;
    do {
        Vertex *v = brep_vert(b, brep_he(b, h)->origin);
        sx += v->p.x; sy += v->p.y; sz += v->p.z; n++;
        h = brep_he(b, h)->next;
    } while (h != h0 && guard++ < 256);
    if (n == 0) return pl;
    Vec3i ctr = { (mym_t)(sx / n), (mym_t)(sy / n), (mym_t)(sz / n) };

    /* Unit normal (1.12) from the face's exact integer normal. */
    Vec3i nrm = v3_unit_fx(face->normal);
    if (nrm.x == 0 && nrm.y == 0 && nrm.z == 0) return pl;

    /* Derived u/v basis orthogonal to the normal. Pick a reference axis least
     * parallel to the normal so the cross product is well-conditioned, then
     * Gram-Schmidt by cross products. Right-handed: u x v = normal. */
    Vec3i ax = mym_abs(nrm.x) <= mym_abs(nrm.y)
                   ? (mym_abs(nrm.x) <= mym_abs(nrm.z) ? (Vec3i){ FX_ONE, 0, 0 }
                                                       : (Vec3i){ 0, 0, FX_ONE })
                   : (mym_abs(nrm.y) <= mym_abs(nrm.z) ? (Vec3i){ 0, FX_ONE, 0 }
                                                       : (Vec3i){ 0, 0, FX_ONE });
    Vec3i u = v3_unit_fx(v3_cross(ax, nrm));   /* u perpendicular to normal   */
    Vec3i v = v3_unit_fx(v3_cross(nrm, u));    /* v completes the right-hand set */

    pl.origin = ctr;
    pl.u_axis = u;
    pl.v_axis = v;
    pl.normal = nrm;
    return pl;
}

/* ---- full B-rep rebuild (clean, for live preview) ---------------------- */

/* Re-thread a pool's free list over its existing backing (empties it). */
static void pool_reset(Pool *p) {
    p->count     = 0;
    p->free_head = 0;
    for (uint16_t i = 0; i < p->capacity; ++i) {
        PoolId next = (PoolId)((i + 1 < p->capacity) ? (i + 1) : POOL_NONE);
        *(PoolId *)(p->base + (size_t)i * p->elem_size) = next;
    }
    if (p->capacity == 0) p->free_head = POOL_NONE;
}

int model_regen_all(Document *d, Brep *b) {
    pool_reset(&b->verts);
    pool_reset(&b->hedges);
    pool_reset(&b->edges);
    pool_reset(&b->loops);
    pool_reset(&b->faces);
    pool_reset(&b->shells);
    pool_reset(&b->solids);
    b->etab_used  = 0;
    b->aabb_valid = 0;
    for (uint16_t i = 0; i < d->feat_count; ++i) d->feat[i].dirty = 1;
    return doc_regen(d, b);
}

/* ---- pending edit lifecycle ------------------------------------------- */

int model_begin_extrude(ModelingState *m, Document *d, Brep *b,
                        FaceId base, OpType op) {
    if (m->pending) return 0;
    if (!brep_face(b, base)) return 0;

    SketchPlane pl = plane_from_face(b, base);

    char nm[FEAT_NAME_LEN];
    feature_autoname(d, FEAT_SKETCH, op, nm, FEAT_NAME_LEN);
    Feature *sk = doc_add(d, FEAT_SKETCH, nm);
    if (!sk) return 0;
    sk->plane = pl;
    sk_init(&sk->sketch);
    sk_add_circle(&sk->sketch, 0, 0, MODEL_DEF_RADIUS);

    /* The sketch is already in `d`, but it uses the "Sketch" prefix so it won't
     * collide with the extrude's "Boss-Extrude"/"Cut-Extrude" namespace. */
    feature_autoname(d, FEAT_EXTRUDE, op, nm, FEAT_NAME_LEN);
    Feature *ex = doc_add(d, FEAT_EXTRUDE, nm);
    if (!ex) { d->feat_count--; return 0; }   /* roll back the sketch */
    ex->p.extrude.sketch_id   = sk->id;
    ex->p.extrude.dir         = 1;
    ex->p.extrude.op          = op;
    ex->p.extrude.target_face = 0;
    if (op == OP_CUT) {
        ex->p.extrude.end  = END_THROUGH_ALL;
        ex->p.extrude.dist = MODEL_DEF_DIST;   /* computed for through-all */
    } else {
        ex->p.extrude.end  = END_BLIND;
        ex->p.extrude.dist = MODEL_DEF_DIST;
    }
    ex->depends_on[0] = sk->id;
    ex->dep_count     = 1;

    m->pending    = 1;
    m->sketch_id  = sk->id;
    m->extrude_id = ex->id;
    m->op         = op;
    m->dist       = MODEL_DEF_DIST;

    return model_regen_all(d, b);
}

int model_set_distance(ModelingState *m, Document *d, Brep *b, mym_t dist) {
    if (!m->pending) return 0;
    if (dist < MODEL_MIN_DIST) dist = MODEL_MIN_DIST;
    m->dist = dist;
    Feature *ex = doc_find(d, m->extrude_id);
    if (!ex) return 0;
    ex->p.extrude.dist = dist;   /* ignored when end==THROUGH_ALL, harmless */
    return model_regen_all(d, b);
}

int model_nudge_distance(ModelingState *m, Document *d, Brep *b, mym_t delta) {
    if (!m->pending) return 0;
    return model_set_distance(m, d, b, m->dist + delta);
}

int model_confirm(ModelingState *m, Document *d, Brep *b, History *h) {
    if (!m->pending) return 0;
    if (!model_regen_all(d, b)) return 0;     /* leave pending on failure */
    if (h) hist_commit(h, d);
    m->pending    = 0;
    m->sketch_id  = 0;
    m->extrude_id = 0;
    return 1;
}

int model_cancel(ModelingState *m, Document *d, Brep *b) {
    if (!m->pending) return 0;
    /* The two pending features were appended last, so pop them back off. Guard
     * in case the document was otherwise mutated. */
    if (d->feat_count >= 2 &&
        d->feat[d->feat_count - 1].id == m->extrude_id &&
        d->feat[d->feat_count - 2].id == m->sketch_id) {
        d->feat_count = (uint16_t)(d->feat_count - 2);
    }
    m->pending    = 0;
    m->sketch_id  = 0;
    m->extrude_id = 0;
    model_regen_all(d, b);
    return 1;
}
