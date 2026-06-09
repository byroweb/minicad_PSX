/* ops.c — sketch planes, profile prep, end-conditions, extrude & revolve.
 *
 * Operations reduce any profile to an ordered ring of N points, then build a
 * prism (extrude) or solid-of-revolution (revolve) with full half-edge
 * stitching via brep_build_face (twins paired automatically). Boss makes a
 * standalone solid; cut carves a face-with-hole + inner wall on a target face
 * (the pragmatic cut: cube+hole, bearing bore — not general CSG).
 */
#include "minicad/ops.h"

/* ---------------- datum planes ---------------- */
SketchPlane plane_xy(void){ SketchPlane p={{0,0,0},{FX_ONE,0,0},{0,FX_ONE,0},{0,0,FX_ONE}}; return p; }
SketchPlane plane_xz(void){ SketchPlane p={{0,0,0},{FX_ONE,0,0},{0,0,FX_ONE},{0,FX_ONE,0}}; return p; }
SketchPlane plane_yz(void){ SketchPlane p={{0,0,0},{0,FX_ONE,0},{0,0,FX_ONE},{FX_ONE,0,0}}; return p; }

Vec3i sketch_to_3d(const SketchPlane *pl, Vec2i p){
    Vec3i r;
    r.x = pl->origin.x + (mym_t)(((mym2_t)pl->u_axis.x*p.u + (mym2_t)pl->v_axis.x*p.v) >> FX_SHIFT);
    r.y = pl->origin.y + (mym_t)(((mym2_t)pl->u_axis.y*p.u + (mym2_t)pl->v_axis.y*p.v) >> FX_SHIFT);
    r.z = pl->origin.z + (mym_t)(((mym2_t)pl->u_axis.z*p.u + (mym2_t)pl->v_axis.z*p.v) >> FX_SHIFT);
    return r;
}

/* ---------------- profile -> ring ---------------- */
int circle_segments(mym_t radius){
    /* low-poly LUT, no sqrt/float. radius in myriometers (10 = 1mm). */
    if (radius <  100) return 8;     /* < 10mm  */
    if (radius <  300) return 12;    /* < 30mm  */
    if (radius <  800) return 16;
    return 24;
}

int profile_to_ring(const Sketch *sk, Vec3i *ring, int cap){
    if (sk->count == 0) return 0;
    const SketchEntity *e0 = &sk->ent[0];

    if (e0->kind == SK_RECT) {
        if (cap < 4) return 0;
        Vec2i a = e0->a, c = e0->b;
        Vec2i q[4] = { {a.u,a.v}, {c.u,a.v}, {c.u,c.v}, {a.u,c.v} };
        for (int i=0;i<4;i++) ring[i] = sketch_to_3d(&sk->plane, q[i]);
        return 4;
    }
    if (e0->kind == SK_CIRCLE) {
        int n = circle_segments(e0->radius);
        if (cap < n) n = cap;
        /* additive angle step: one divide total, not one per segment (R3000
         * div is ~35 cyc; keep it out of the geometry-build loop). */
        int32_t step = SIN_LEN / n;
        int32_t ang  = 0;
        for (int i=0;i<n;i++){
            mym_t x = (mym_t)(((mym2_t)e0->radius * fx_icos(ang)) >> FX_SHIFT);
            mym_t y = (mym_t)(((mym2_t)e0->radius * fx_isin(ang)) >> FX_SHIFT);
            Vec2i p = { e0->a.u + x, e0->a.v + y };
            ring[i] = sketch_to_3d(&sk->plane, p);
            ang += step;
        }
        return n;
    }
    /* TODO: general line-chain profile (walk SK_LINE segments into a ring). */
    return 0;
}

int sketch_is_closed_profile(const Sketch *sk){
    if (sk->count == 0) return 0;
    if (sk->count == 1 && (sk->ent[0].kind==SK_RECT || sk->ent[0].kind==SK_CIRCLE)) return 1;
    for (uint8_t i=0;i<sk->count;i++){
        const SketchEntity *e=&sk->ent[i], *n=&sk->ent[(i+1)%sk->count];
        if (e->kind!=SK_LINE) continue;
        if (e->b.u!=n->a.u || e->b.v!=n->a.v) return 0;
    }
    return 1;
}

/* ---------------- end-condition resolution ---------------- */
/* Signed distance from a point along a unit (1.12) direction to a plane. */
static mym_t ray_plane_dist(Vec3i origin, Vec3i dir_unit, Vec3i n, mym2_t d){
    mym2_t denom = v3_dot(n, dir_unit);          /* n . dir (dir scaled 1.12) */
    if (denom == 0) return 0;                    /* parallel */
    mym2_t num = (d << FX_SHIFT) - (v3_dot(n, origin) << FX_SHIFT); /* keep 1.12 */
    /* t such that origin + (t/1)*dir_unit hits plane; dir_unit is 1.12 unit,
     * so t is already in myriometers after dividing by denom (also 1.12). */
    return mym_div_round(num, denom);
}

static mym_t solid_extent(Brep *b){
    /* Bound by max |coord| over the model, read straight from the running AABB
     * (maintained in brep_add_vertex). No pool walk, no chance of reading a
     * freed slot. Identical result to the old per-vert |coord| scan. */
    if (!b->aabb_valid) return 1000;            /* empty: just the margin */
    mym_t m = 0;
    mym_t c[6] = { mym_abs(b->aabb_min.x), mym_abs(b->aabb_max.x),
                   mym_abs(b->aabb_min.y), mym_abs(b->aabb_max.y),
                   mym_abs(b->aabb_min.z), mym_abs(b->aabb_max.z) };
    for (int i=0;i<6;i++) if (c[i] > m) m = c[i];
    return (mym_t)(m*2 + 1000);                 /* +100mm margin */
}

int resolve_end_condition(Brep *b, SolidId against, const SketchPlane *pl,
                          OpParams *params, mym_t blind_dist){
    (void)against;
    switch (params->end){
    case END_BLIND:
        params->dist = blind_dist;
        return 1;
    case END_THROUGH_ALL: {
        if (params->op != OP_CUT) return 0;       /* through-all is cut-only */
        /* Span exactly the body's extent along the cut normal (+ a small over-
         * cut so the tube clears both faces) rather than the whole-model
         * solid_extent, which made the hole shoot far past the part. */
        mym_t dist;
        if (b->aabb_valid) {
            Vec3i n = pl->normal;                 /* 1.12 unit dir */
            mym2_t e = ((mym2_t)mym_abs(n.x) * (b->aabb_max.x - b->aabb_min.x)
                      + (mym2_t)mym_abs(n.y) * (b->aabb_max.y - b->aabb_min.y)
                      + (mym2_t)mym_abs(n.z) * (b->aabb_max.z - b->aabb_min.z)) >> FX_SHIFT;
            dist = (mym_t)e + 20;                  /* +2mm over-cut */
        } else {
            dist = solid_extent(b);
        }
        params->dist = dist;
        return 1;
    }
    case END_UP_TO_SURFACE: {
        if (params->target == BREP_NONE) return 0;
        Face *tf = brep_face(b, params->target);
        if (!tf) return 0;
        mym_t d = ray_plane_dist(pl->origin, pl->normal, tf->normal, tf->plane_d);
        if (d < 0) d = (mym_t)(-d);
        params->dist = d;
        return 1;
    }
    }
    return 0;
}

/* ---------------- prism builder (shared) ---------------- */
/* Given a base ring of N verts, builds top ring translated by `vec`, then the
 * two caps and N side faces with full stitching. Returns the solid. */
/* Build a swept prism from a base ring + extrusion vector.
 * `caps`: 1 = closed solid (end caps), used by a boss. 0 = open tube (side
 * walls only), used by a through-cut: a hole has no end caps, and emitting them
 * here would sit coincident inside the target's faces as degenerate
 * zero-thickness surfaces. */
static SolidId build_prism(Brep *b, const Vec3i *base_pts, int n, Vec3i vec,
                           int caps, uint16_t feat){
    if (n < 3 || n > 64) return BREP_NONE;
    VertId base[64], top[64];
    for (int i=0;i<n;i++){
        base[i] = brep_add_vertex(b, base_pts[i]);
        Vec3i q = base_pts[i]; q.x+=vec.x; q.y+=vec.y; q.z+=vec.z;
        top[i]  = brep_add_vertex(b, q);
    }
    SolidId sid = pool_alloc(&b->solids);
    ShellId shid = pool_alloc(&b->shells);
    if (sid==BREP_NONE || shid==BREP_NONE) return BREP_NONE;
    Shell *sh=(Shell*)pool_get(&b->shells,shid); sh->face_count=0; sh->solid=sid;

    brep_begin_solid(b);

    /* bottom cap (reversed for outward normal), top cap — only for a solid */
    if (caps) {
        VertId rb[64]; for(int i=0;i<n;i++) rb[i]=base[n-1-i];
        FaceId fb = brep_build_face(b, rb, n, 0,0,0, feat);
        FaceId ft = brep_build_face(b, top, n, 0,0,0, feat);
        if (sh->face_count < 64) sh->faces[sh->face_count++]=fb;
        if (sh->face_count < 64) sh->faces[sh->face_count++]=ft;
    }

    /* side quads: base[i], base[i+1], top[i+1], top[i] */
    for (int i=0;i<n;i++){
        int j=(i+1)%n;
        VertId quad[4]={ base[i], base[j], top[j], top[i] };
        FaceId fs = brep_build_face(b, quad, 4, 0,0,0, feat);
        if (sh->face_count < 64) sh->faces[sh->face_count++]=fs;
    }
    ((Solid*)pool_get(&b->solids,sid))->outer = shid;
    return sid;
}

/* ---------------- through-cut fusion ----------------
 * Fuse a through-all cut INTO the target solid as a single body with
 * faces-with-holes, instead of emitting a separate open tube.
 *
 * Inputs: the profile ring (N points on the sketch plane) and the cut direction
 * `cdir` (1.12 unit, already sign-applied). Steps:
 *   1. pick the target solid (the last solid built) and its shell;
 *   2. find the two cap faces it pierces (normal parallel to the cut axis):
 *      the ENTRY cap (closest plane along -cdir from the profile) and the EXIT
 *      cap (closest along +cdir);
 *   3. project the profile ring onto each cap plane -> entry_ring / exit_ring,
 *      with EXACTLY the cap-plane coordinates so the tube ends are coincident;
 *   4. under ONE brep_begin_solid scope: add the projected ring as an inner
 *      (hole) loop on each cap, then build the N tube side-quads. Twin pairing is
 *      scoped across all of it, so the cap inner-loop half-edges pair with the
 *      tube end edges (no boundary edges remain);
 *   5. adopt the tube faces into the target shell. No new solid is allocated.
 *
 * Returns the target solid id, or BREP_NONE if the supported configuration
 * (two parallel cap faces straddling the profile) is not found.
 */

/* Project point P along unit direction u (1.12) onto plane (fn . X = pd).
 * Exact-ish integer: t = (pd - fn.P)*FX_ONE / (fn.u); result = P + t*u/FX_ONE. */
static Vec3i project_to_plane(Vec3i p, Vec3i u, Vec3i fn, mym2_t pd) {
    mym2_t denom = v3_dot(fn, u);                 /* scaled by FX_ONE (u is 1.12) */
    if (denom == 0) return p;                     /* parallel: shouldn't happen   */
    mym2_t t = mym_div_round((pd - v3_dot(fn, p)) << FX_SHIFT, denom); /* myriom. */
    Vec3i r;
    r.x = p.x + (mym_t)mym_div_round((mym2_t)u.x * t, FX_ONE);
    r.y = p.y + (mym_t)mym_div_round((mym2_t)u.y * t, FX_ONE);
    r.z = p.z + (mym_t)mym_div_round((mym2_t)u.z * t, FX_ONE);
    return r;
}

static SolidId cut_through_fuse(Brep *b, const Vec3i *ring, int n, Vec3i cdir,
                                uint16_t feat) {
    if (n < 3 || n > 64) return BREP_NONE;
    if (b->solids.count == 0) return BREP_NONE;

    /* target = last solid built (the body we cut into) */
    SolidId tsid = (SolidId)(b->solids.count - 1);
    Solid *ts = (Solid *)pool_get(&b->solids, tsid);
    if (!ts) return BREP_NONE;
    ShellId tshid = ts->outer;
    Shell *tsh = (Shell *)pool_get(&b->shells, tshid);
    if (!tsh) return BREP_NONE;

    /* Reference point: profile centroid (integer mean) to score cap distance. */
    mym2_t cx = 0, cy = 0, cz = 0;
    for (int i = 0; i < n; ++i) { cx += ring[i].x; cy += ring[i].y; cz += ring[i].z; }
    Vec3i ctr = { (mym_t)(cx / n), (mym_t)(cy / n), (mym_t)(cz / n) };

    /* Find the two cap faces: normal parallel to the cut axis, straddling the
     * profile along cdir. ENTRY = the cap on the -cdir side (smallest signed
     * distance), EXIT = the cap on the +cdir side (largest). */
    FaceId entry = BREP_NONE, exitf = BREP_NONE;
    mym2_t entry_s = 0, exit_s = 0;
    for (uint8_t k = 0; k < tsh->face_count; ++k) {
        FaceId fid = tsh->faces[k];
        Face *f = brep_face(b, fid);
        if (!f) continue;
        if (v3_cross(f->normal, cdir).x != 0 ||
            v3_cross(f->normal, cdir).y != 0 ||
            v3_cross(f->normal, cdir).z != 0) continue;   /* not a cap (not parallel) */
        if (f->inner_count != 0) continue;                /* already holed */
        /* signed distance of the cap plane from the profile centroid along cdir:
         * solve fn.(ctr + s*cdir) = pd  ->  s = (pd - fn.ctr)/(fn.cdir). */
        mym2_t denom = v3_dot(f->normal, cdir);
        if (denom == 0) continue;
        mym2_t s = mym_div_round((f->plane_d - v3_dot(f->normal, ctr)) << FX_SHIFT, denom);
        if (entry == BREP_NONE || s < entry_s) { entry = fid; entry_s = s; }
        if (exitf == BREP_NONE || s > exit_s)  { exitf = fid; exit_s = s; }
    }
    if (entry == BREP_NONE || exitf == BREP_NONE || entry == exitf) return BREP_NONE;

    Face *fe = brep_face(b, entry), *fx = brep_face(b, exitf);

    /* Project the profile ring onto each cap plane -> coincident tube end rings.
     * The tube wall vertices ARE these projected points (shared with the cap
     * inner loops), which is what lets the twins pair. */
    Vec3i ep[64], xp[64];
    for (int i = 0; i < n; ++i) {
        ep[i] = project_to_plane(ring[i], cdir, fe->normal, fe->plane_d);
        xp[i] = project_to_plane(ring[i], cdir, fx->normal, fx->plane_d);
    }
    VertId ev[64], xv[64];
    for (int i = 0; i < n; ++i) {
        ev[i] = brep_add_vertex(b, ep[i]);
        xv[i] = brep_add_vertex(b, xp[i]);
    }

    /* ---- one twin scope for the inner loops + the whole tube wall ---- */
    brep_begin_solid(b);

    /* INNER HOLE LOOP WINDING (load-bearing — see report to render agent):
     * The cap OUTER loop is CCW-from-outside. A hole loop must be wound the
     * OPPOSITE sense (CW-from-outside, i.e. reversed) so it bounds the same face
     * area with a consistent orientation and so its half-edges run anti-parallel
     * to the tube wall edges they twin with. We feed each cap the projected ring
     * REVERSED. The entry cap faces -cdir and the exit cap faces +cdir, so the
     * "reverse for a hole" direction is opposite on the two caps. The tube quad
     * {ev[i],xv[i],xv[j],ev[j]} traverses the entry ring as j->i and the exit
     * ring as i->j; for anti-parallel twins the entry inner loop must run i->j
     * (ev forward) and the exit inner loop j->i (xv reversed). This also yields
     * a CW-from-outside hole on each cap (verified by the antiparallel-twin +
     * outward-volume tests). */
    VertId ev_fwd[64], xv_rev[64];
    for (int i = 0; i < n; ++i) ev_fwd[i] = ev[i];
    for (int i = 0; i < n; ++i) xv_rev[i] = xv[n - 1 - i];
    if (brep_add_face_hole(b, entry, ev_fwd, n) == BREP_NONE) return BREP_NONE;
    if (brep_add_face_hole(b, exitf, xv_rev, n) == BREP_NONE) return BREP_NONE;

    /* Tube side quads spanning entry-ring -> exit-ring. The wall normals must
     * point INTO the hole (away from the remaining material). Order chosen so
     * each quad's edges twin anti-parallel with the cap inner-loop edges. */
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        VertId quad[4] = { ev[i], xv[i], xv[j], ev[j] };
        FaceId fs = brep_build_face(b, quad, 4, 0, 0, 0, feat);
        if (fs == BREP_NONE) return BREP_NONE;
        if (!brep_shell_add_face(b, tshid, fs)) return BREP_NONE;
    }
    return tsid;
}

/* ---------------- extrude ---------------- */
SolidId op_extrude(Brep *b, const Sketch *sk, const OpParams *params,
                   SolidId target_solid, uint16_t feature_id){
    if (!sketch_is_closed_profile(sk)) return BREP_NONE;

    Vec3i ring[64];
    int n = profile_to_ring(sk, ring, 64);
    if (n < 3) return BREP_NONE;

    /* translation vector = normal * (dir*dist), normal is 1.12 unit dir */
    mym_t d = (mym_t)(params->dir >= 0 ? params->dist : -params->dist);
    Vec3i nrm = sk->plane.normal, vec;
    vec.x = (mym_t)(((mym2_t)nrm.x*d)>>FX_SHIFT);
    vec.y = (mym_t)(((mym2_t)nrm.y*d)>>FX_SHIFT);
    vec.z = (mym_t)(((mym2_t)nrm.z*d)>>FX_SHIFT);

    if (params->op == OP_BOSS) {
        return build_prism(b, ring, n, vec, 1 /*caps*/, feature_id);
    }

    /* ---- CUT (pragmatic, profile-on-a-face) ----
     * Fuse the through-cut INTO the target solid as ONE body: the two pierced
     * cap faces become faces-with-holes and the N tube side-faces join the same
     * shell, with twins paired so no boundary edges remain. (Was: a separate
     * open-tube solid; that "one real limitation" is now closed for through-all.)
     *
     * cdir is the unit (1.12) cut direction. cut_through_fuse projects the
     * profile onto each cap plane, so the over-cut `dist` only matters for the
     * blind path; for through-all the tube ends land exactly on the caps. */
    (void)target_solid; (void)vec;
    if (params->end == END_THROUGH_ALL) {
        Vec3i cdir = sk->plane.normal;
        if (params->dir < 0) { cdir.x = -cdir.x; cdir.y = -cdir.y; cdir.z = -cdir.z; }
        SolidId fused = cut_through_fuse(b, ring, n, cdir, feature_id);
        if (fused != BREP_NONE) return fused;
        /* fall through to the legacy open-tube only if fusion's supported
         * configuration (two parallel straddling caps) was not found. */
    }
    /* Legacy / non-through cut: open tube (no caps) as its own wall solid. */
    SolidId wall = build_prism(b, ring, n, vec, 0 /*no caps*/, feature_id);
    return wall;
}

/* ---------------- revolve ---------------- */
SolidId op_revolve(Brep *b, const Sketch *sk, const OpParams *params,
                   Vec3i axis_origin, Vec3i axis_dir,
                   int32_t sweep, int steps,
                   SolidId target_solid, uint16_t feature_id){
    (void)params; (void)target_solid; (void)axis_dir;
    if (!sketch_is_closed_profile(sk)) return BREP_NONE;
    if (steps < 3) steps = 3;
    if (sweep == 0) sweep = SIN_LEN;          /* default full turn */

    Vec3i prof[64];
    int n = profile_to_ring(sk, prof, 64);
    if (n < 3) return BREP_NONE;

    int full = (sweep >= SIN_LEN);
    int rings = full ? steps : steps + 1;     /* open sweep needs an end ring */
    if (rings * n > 1024) return BREP_NONE;

    /* generate ring vertices: each step rotates the profile about Y through
     * (sweep/steps). MVP revolves about the Y axis through axis_origin. */
    VertId vid[64][64];
    for (int s=0; s<rings; s++){
        int32_t ang = (int32_t)(((mym2_t)sweep * s) / steps);
        Mat3 R = mat3_rot_y(ang);
        for (int i=0;i<n;i++){
            Vec3i p = prof[i];
            p.x -= axis_origin.x; p.y -= axis_origin.y; p.z -= axis_origin.z;
            p = mat3_apply(R, p);
            p.x += axis_origin.x; p.y += axis_origin.y; p.z += axis_origin.z;
            vid[s][i] = brep_add_vertex(b, p);
        }
    }

    SolidId sid = pool_alloc(&b->solids);
    ShellId shid = pool_alloc(&b->shells);
    if (sid==BREP_NONE||shid==BREP_NONE) return BREP_NONE;
    Shell *sh=(Shell*)pool_get(&b->shells,shid); sh->face_count=0; sh->solid=sid;
    brep_begin_solid(b);

    int ringcount = full ? steps : rings - 1;
    for (int s=0; s<ringcount; s++){
        int sn = full ? (s+1)%steps : s+1;
        for (int i=0;i<n;i++){
            int j=(i+1)%n;
            /* CCW-from-outside (see WINDING CONVENTION in brep.c). The naive
             * order vid[s][i],vid[s][j],vid[sn][j],vid[sn][i] winds the side
             * loops CW-as-seen-from-outside for this profile/sweep handedness,
             * giving inward normals (negative signed volume) — opposite to the
             * extrude prism. Reverse the loop so revolve faces match the single
             * outward convention the PS1 backface cull relies on. */
            VertId quad[4]={ vid[s][i], vid[sn][i], vid[sn][j], vid[s][j] };
            FaceId fs = brep_build_face(b, quad, 4, 0,0,0, feature_id);
            if (sh->face_count < 64) sh->faces[sh->face_count++]=fs;
        }
    }
    /* open sweep: cap the two end profiles */
    if (!full) {
        VertId start[64], end[64];
        /* caps flipped to match the reversed side winding above: the start cap
         * (at s=0) faces "back" along the sweep, the end cap faces "forward". */
        for (int i=0;i<n;i++){ start[i]=vid[0][i]; end[i]=vid[rings-1][n-1-i]; }
        FaceId f0=brep_build_face(b,start,n,0,0,0,feature_id);
        FaceId f1=brep_build_face(b,end,  n,0,0,0,feature_id);
        if (sh->face_count<64) sh->faces[sh->face_count++]=f0;
        if (sh->face_count<64) sh->faces[sh->face_count++]=f1;
    }
    ((Solid*)pool_get(&b->solids,sid))->outer = shid;
    return sid;
}
