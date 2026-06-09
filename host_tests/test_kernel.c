/* test_kernel.c — host-side unit tests (no PlayStation needed).
 * Build with MINICAD_HOST defined. Returns non-zero on first failure. */
#include "minicad/fixed.h"
#include "minicad/save.h"
#include "minicad/feature.h"
#include "minicad/sketch.h"
#include "minicad/history.h"
#include "minicad/modeling.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, msg) do { if(!(cond)){ printf("FAIL: %s\n", msg); fails++; } \
                              else { printf("ok:   %s\n", msg); } } while(0)

static void test_mm_str(void) {
    char b[16];
    CHECK(strcmp(mym_to_mm_str(12345, b), "1234.5")==0, "12345 -> 1234.5");
    CHECK(strcmp(mym_to_mm_str(7, b),     "0.7")==0,    "7 -> 0.7");
    CHECK(strcmp(mym_to_mm_str(-50, b),   "-5.0")==0,   "-50 -> -5.0");
    CHECK(strcmp(mym_to_mm_str(0, b),     "0.0")==0,    "0 -> 0.0");
    CHECK(strcmp(mym_to_mm_str(10, b),    "1.0")==0,    "10 -> 1.0");
    CHECK(strcmp(mym_to_mm_str(105, b),   "10.5")==0,   "105 -> 10.5");
}

static void test_sine(void) {
    CHECK(fx_isin(0) == 0,                      "sin(0)=0");
    CHECK(fx_icos(0) == FX_ONE,                 "cos(0)=1.0 (4096)");
    int s90 = fx_isin(SIN_LEN/4);
    CHECK(s90 > FX_ONE-40 && s90 <= FX_ONE,     "sin(90)~1.0");
}

static void test_save_roundtrip(void) {
    Document a, b;
    doc_init(&a, "Part1");
    Feature *sk = doc_add(&a, FEAT_SKETCH, "Sketch1");
    sk->plane = plane_xy();
    sk_init(&sk->sketch);
    sk_add_rect(&sk->sketch, -300,-300, 300,300);
    Feature *ex = doc_add(&a, FEAT_EXTRUDE, "Boss-Extrude1");
    ex->p.extrude.sketch_id = sk->id; ex->p.extrude.dist = 600;
    ex->p.extrude.dir = 1; ex->p.extrude.op = OP_BOSS;
    ex->p.extrude.end = END_BLIND; ex->p.extrude.target_face = 0;
    ex->depends_on[0] = sk->id; ex->dep_count = 1;

    uint8_t buf[512];
    int n = mcad_encode(&a, buf, sizeof buf);
    CHECK(n > 0,            "encode produced bytes");
    printf("      encoded part size: %d bytes\n", n);
    CHECK(n < 160,          "part encodes compactly (<160 bytes w/ Sketch2)");

    int ok = mcad_decode(&b, buf, n);
    CHECK(ok == 1,          "decode succeeded (magic+crc)");
    CHECK(b.feat_count == a.feat_count, "feature count round-trips");
    Feature *bex = doc_find(&b, ex->id);
    CHECK(bex && bex->p.extrude.dist == 600, "extrude distance round-trips");
    CHECK(bex && bex->p.extrude.op == OP_BOSS && bex->p.extrude.end == END_BLIND,
          "op/end round-trip");

    /* Sketch2 contents (points/entities) and the plane round-trip */
    Feature *bsk = doc_find(&b, sk->id);
    CHECK(bsk && bsk->sketch.pt_count == sk->sketch.pt_count,
          "Sketch2 point count round-trips");
    CHECK(bsk && bsk->sketch.ent_count == sk->sketch.ent_count,
          "Sketch2 entity count round-trips");
    CHECK(bsk && bsk->sketch.pt[0].u == sk->sketch.pt[0].u
              && bsk->sketch.pt[0].v == sk->sketch.pt[0].v,
          "Sketch2 first point coords round-trip");
    CHECK(bsk && bsk->sketch.ent[0].kind == sk->sketch.ent[0].kind,
          "Sketch2 first entity kind round-trips");
    CHECK(bsk && bsk->plane.normal.z == sk->plane.normal.z,
          "Sketch2 plane round-trips");
}

/* Round-trip a Sketch2 carrying a circle + a live constraint, end to end. */
static void test_save_roundtrip_sketch2(void) {
    Document a, b;
    doc_init(&a, "Part1");
    Feature *sk = doc_add(&a, FEAT_SKETCH, "Sketch1");
    sk->plane = plane_yz();
    sk_init(&sk->sketch);
    sk_add_circle(&sk->sketch, 10, 20, 150);
    SkEntId ln = sk_add_line(&sk->sketch,
                             sk_add_point(&sk->sketch, 0,0),
                             sk_add_point(&sk->sketch, 100,0));
    sk_add_constraint(&sk->sketch, SKC_DIMENSION, ln, SK_NONE,
                      SK_NONE, SK_NONE, 1000);

    uint8_t buf[512];
    int n = mcad_encode(&a, buf, sizeof buf);
    CHECK(n > 0, "sketch2 encode produced bytes");
    int ok = mcad_decode(&b, buf, n);
    CHECK(ok == 1, "sketch2 decode succeeded");
    Feature *bsk = doc_find(&b, sk->id);
    CHECK(bsk && bsk->sketch.con_count == 1, "constraint count round-trips");
    CHECK(bsk && bsk->sketch.con[0].kind == SKC_DIMENSION
              && bsk->sketch.con[0].value == 1000,
          "DIMENSION constraint (kind+value) round-trips");
    CHECK(bsk && bsk->sketch.ent[0].kind == SKE_CIRCLE
              && bsk->sketch.ent[0].radius == 150,
          "circle entity radius round-trips");
    CHECK(bsk && bsk->plane.u_axis.y == sk->plane.u_axis.y,
          "yz plane u_axis round-trips");
}

/* shared brep backing for op tests */
#define DECLARE_BREP(name) \
    static Vertex name##v[2048]; static HalfEdge name##he[8192]; \
    static Edge name##e[4096]; static Loop name##l[2048]; \
    static Face name##f[1024]; static Shell name##s[64]; static Solid name##so[64]; \
    Brep name; brep_init(&name, name##v,2048, name##he,8192, name##e,4096, \
                         name##l,2048, name##f,1024, name##s,64, name##so,64)

/* helpers: build a Sketch2 holding a single rect / circle profile */
static void mk_rect(Sketch2 *s, mym_t u0, mym_t v0, mym_t u1, mym_t v1) {
    sk_init(s); sk_add_rect(s, u0, v0, u1, v1);
}
static void mk_circle(Sketch2 *s, mym_t cu, mym_t cv, mym_t r) {
    sk_init(s); sk_add_circle(s, cu, cv, r);
}

static void test_extrude_boss(void) {
    DECLARE_BREP(brep);
    Sketch2 sk; mk_rect(&sk, -300,-300, 300,300);
    SketchPlane pl = plane_xy();
    OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
    SolidId sid = op_extrude(&brep, &sk, &pl, &pr, BREP_NONE, 1);
    CHECK(sid != BREP_NONE,        "boss extrude returned a solid");
    CHECK(brep.verts.count == 8,   "box has 8 vertices");
    CHECK(brep.faces.count == 6,   "box has 6 faces");
    CHECK(brep.edges.count == 12,  "box has 12 edges (twins paired)");
    CHECK(brep_check_euler(&brep, sid), "box satisfies Euler V-E+F=2");
}

static void test_extrude_circle(void) {
    DECLARE_BREP(brep);
    Sketch2 sk; mk_circle(&sk, 0,0, 150);
    SketchPlane pl = plane_xy();
    OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
    SolidId sid = op_extrude(&brep, &sk, &pl, &pr, BREP_NONE, 1);
    CHECK(sid != BREP_NONE,         "circle extrude (cylinder) returned a solid");
    /* Ø30 -> radius 150 -> 12 segments. 2 caps + 12 sides = 14 faces, 24 verts. */
    CHECK(brep.verts.count == 24,   "cylinder has 24 vertices (12-gon x2)");
    CHECK(brep.faces.count == 14,   "cylinder has 14 faces (12 sides + 2 caps)");
    CHECK(brep_check_euler(&brep, sid), "cylinder satisfies Euler");
}

static void test_through_all_is_cut_only(void) {
    DECLARE_BREP(brep);
    /* put a box in so solid_extent has something to measure */
    Sketch2 box; mk_rect(&box, -300,-300, 300,300);
    SketchPlane pl = plane_xy();
    OpParams bp={OP_BOSS,END_BLIND,600,1,BREP_NONE};
    op_extrude(&brep,&box,&pl,&bp,BREP_NONE,1);

    OpParams cut = { OP_CUT, END_THROUGH_ALL, 0, 1, BREP_NONE };
    int ok = resolve_end_condition(&brep, BREP_NONE, &pl, &cut, 0);
    CHECK(ok == 1 && cut.dist > 600, "through-all cut resolves to clear the solid");
    /* Through-all spans the body extent ALONG THE CUT NORMAL (+2mm over-cut),
     * not the whole-model solid_extent: box z is [0,600], normal +z, so
     * extent = 600 -> dist = 620. (Stops the hole shooting far past the part.) */
    CHECK(cut.dist == 620, "through-all spans body extent along normal + over-cut");

    OpParams bad = { OP_BOSS, END_THROUGH_ALL, 0, 1, BREP_NONE };
    int bok = resolve_end_condition(&brep, BREP_NONE, &pl, &bad, 0);
    CHECK(bok == 0, "through-all rejected for a boss (cut-only)");
}

static void test_revolve(void) {
    DECLARE_BREP(brep);
    Sketch2 sk; mk_rect(&sk, 200,-50, 300,50);
    SketchPlane pl = plane_xy();
    OpParams pr = { OP_BOSS, END_BLIND, 0, 1, BREP_NONE };
    Vec3i ax_o={0,0,0}, ax_d={0,FX_ONE,0};
    SolidId sid = op_revolve(&brep, &sk, &pl, &pr, ax_o, ax_d, SIN_LEN, 12, BREP_NONE, 1);
    CHECK(sid != BREP_NONE,        "full revolve returned a solid");
    CHECK(brep.verts.count == 48,  "revolve: 4 profile pts x 12 steps = 48 verts");
    CHECK(brep.faces.count == 48,  "revolve: 4 sides x 12 steps = 48 faces");
}

/* Winding-consistency / outward-orientation check (see WINDING CONVENTION in
 * brep.c). Works for non-convex solids (e.g. a revolved tube) by using the
 * divergence theorem rather than a centroid heuristic: triangulate every outer
 * face loop as a fan and accumulate the signed volume
 *     6V = Σ  dot(v0, cross(v1, v2))
 * over all triangles. If EVERY face winds CCW-from-outside, the normals all
 * point outward and 6V is strictly positive; a single inward (or inconsistent)
 * face would cancel or flip the sum. We assert 6V > 0, which is precisely the
 * global property the PS1 single-rule backface cull depends on. (We translate
 * verts by the solid centroid first so the int64 products keep headroom and the
 * result is origin-independent.) */
static mym2_t signed_volume6(Brep *b) {
    mym2_t sx=0, sy=0, sz=0;
    uint16_t nv = b->verts.count;
    if (nv == 0) return 0;
    for (uint16_t i=0;i<nv;i++){ Vertex *v=(Vertex*)pool_get(&b->verts,i);
        sx+=v->p.x; sy+=v->p.y; sz+=v->p.z; }
    Vec3i ctr = { (mym_t)(sx/nv), (mym_t)(sy/nv), (mym_t)(sz/nv) };

    mym2_t vol6 = 0;
    for (uint16_t fi=0; fi<b->faces.count; fi++){
        Face *f = brep_face(b, fi);
        Loop *lp = (Loop*)pool_get(&b->loops, f->outer);
        HEdgeId h0 = lp->first_he;
        Vec3i v0 = v3_sub(brep_vert(b, brep_he(b,h0)->origin)->p, ctr);
        /* fan from the first vertex over the remaining edges */
        HEdgeId ha = brep_he(b,h0)->next;
        HEdgeId hb = brep_he(b,ha)->next;
        int guard = 0;
        while (hb != h0 && guard++ < 256) {
            Vec3i va = v3_sub(brep_vert(b, brep_he(b,ha)->origin)->p, ctr);
            Vec3i vb = v3_sub(brep_vert(b, brep_he(b,hb)->origin)->p, ctr);
            vol6 += v3_dot(v0, v3_cross(va, vb));
            ha = hb; hb = brep_he(b,hb)->next;
        }
    }
    return vol6;
}
static int all_faces_wind_outward(Brep *b) { return signed_volume6(b) > 0; }

/* Every interior edge must have a twin, and the twin must run anti-parallel
 * (its origin == this half-edge's destination). That is the topological face
 * of the same orientation invariant. */
static int all_twins_antiparallel(Brep *b) {
    for (uint16_t hi=0; hi<b->hedges.count; hi++){
        HalfEdge *he = brep_he(b, hi);
        if (he->twin == BREP_NONE) continue;   /* boundary edge: skip */
        HalfEdge *tw = brep_he(b, he->twin);
        VertId dest = brep_he(b, he->next)->origin;
        if (tw->origin != dest) return 0;       /* twins not anti-parallel */
        if (tw->twin != hi) return 0;           /* twin link not symmetric */
    }
    return 1;
}

static void test_winding_consistency(void) {
    /* demo cube */
    {
        DECLARE_BREP(brep);
        Sketch2 sk; mk_rect(&sk, -300,-300, 300,300);
        SketchPlane pl = plane_xy();
        OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
        SolidId sid = op_extrude(&brep, &sk, &pl, &pr, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: cube built");
        CHECK(all_faces_wind_outward(&brep),  "winding: all cube faces point outward");
        CHECK(all_twins_antiparallel(&brep),  "winding: cube twins anti-parallel");
    }
    /* cylinder */
    {
        DECLARE_BREP(brep);
        Sketch2 sk; mk_circle(&sk, 0,0, 150);
        SketchPlane pl = plane_xy();
        OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
        SolidId sid = op_extrude(&brep, &sk, &pl, &pr, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: cylinder built");
        CHECK(all_faces_wind_outward(&brep), "winding: all cylinder faces point outward");
        CHECK(all_twins_antiparallel(&brep), "winding: cylinder twins anti-parallel");
    }
    /* full revolve (solid of revolution) */
    {
        DECLARE_BREP(brep);
        Sketch2 sk; mk_rect(&sk, 200,-50, 300,50);
        SketchPlane pl = plane_xy();
        OpParams pr = { OP_BOSS, END_BLIND, 0, 1, BREP_NONE };
        Vec3i ax_o={0,0,0}, ax_d={0,FX_ONE,0};
        SolidId sid = op_revolve(&brep, &sk, &pl, &pr, ax_o, ax_d, SIN_LEN, 12, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: revolve built");
        CHECK(all_faces_wind_outward(&brep), "winding: all revolve faces point outward");
        CHECK(all_twins_antiparallel(&brep), "winding: revolve twins anti-parallel");
    }
    /* open (partial-sweep) revolve exercises the end caps too */
    {
        DECLARE_BREP(brep);
        Sketch2 sk; mk_rect(&sk, 200,-50, 300,50);
        SketchPlane pl = plane_xy();
        OpParams pr = { OP_BOSS, END_BLIND, 0, 1, BREP_NONE };
        Vec3i ax_o={0,0,0}, ax_d={0,FX_ONE,0};
        SolidId sid = op_revolve(&brep, &sk, &pl, &pr, ax_o, ax_d, SIN_LEN/2, 6, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: open revolve built");
        CHECK(all_faces_wind_outward(&brep), "winding: all open-revolve faces point outward");
        CHECK(all_twins_antiparallel(&brep), "winding: open-revolve twins anti-parallel");
    }
}

/* ---- through-all cut fuses into the target as ONE body (faces-with-holes) ----
 * Builds the demo part (60mm cube + Ø30 through-hole) via doc_regen and asserts
 * the fused topology: 1 solid, both pierced caps holed, inner-loop vertex count
 * == N, generalized Euler residual 0 (genus 1), no unpaired half-edges. */
static int count_unpaired(Brep *b) {
    int u = 0;
    for (uint16_t hi=0; hi<b->hedges.count; hi++)
        if (brep_he(b, hi)->twin == BREP_NONE) u++;
    return u;
}
static int loop_len(Brep *b, LoopId lo) {
    Loop *L = (Loop*)pool_get(&b->loops, lo);
    HEdgeId h0 = L->first_he, h = h0; int n = 0, guard = 0;
    do { n++; h = brep_he(b,h)->next; } while (h != h0 && guard++ < 256);
    return n;
}
static void build_demo_doc(Document *d) {
    doc_init(d, "Part1");
    Feature *sk = doc_add(d, FEAT_SKETCH, "Sketch1");
    sk->plane = plane_xy(); sk_init(&sk->sketch);
    sk_add_rect(&sk->sketch, -300,-300, 300,300);
    Feature *ex = doc_add(d, FEAT_EXTRUDE, "Boss-Extrude1");
    ex->p.extrude.sketch_id = sk->id; ex->p.extrude.dist = 600;
    ex->p.extrude.dir = 1; ex->p.extrude.op = OP_BOSS;
    ex->p.extrude.end = END_BLIND; ex->p.extrude.target_face = 0;
    ex->depends_on[0] = sk->id; ex->dep_count = 1;
    Feature *sk2 = doc_add(d, FEAT_SKETCH, "Sketch2");
    sk2->plane = plane_xy(); sk_init(&sk2->sketch);
    sk_add_circle(&sk2->sketch, 0,0, 150);
    Feature *cut = doc_add(d, FEAT_EXTRUDE, "Cut-Extrude1");
    cut->p.extrude.sketch_id = sk2->id; cut->p.extrude.dist = 600;
    cut->p.extrude.dir = 1; cut->p.extrude.op = OP_CUT;
    cut->p.extrude.end = END_THROUGH_ALL; cut->p.extrude.target_face = 0;
    cut->depends_on[0] = sk2->id; cut->dep_count = 1;
}
static Document g_cutdoc;
static void test_through_all_fuses_one_solid(void) {
    DECLARE_BREP(brep);
    build_demo_doc(&g_cutdoc);
    int ok = doc_regen(&g_cutdoc, &brep);
    CHECK(ok == 1, "fuse: demo regen succeeded");
    CHECK(brep.solids.count == 1, "fuse: exactly ONE solid (cube+hole fused)");

    /* N for Ø30 (radius 150) -> 12 segments */
    int N = circle_segments(150);
    CHECK(N == 12, "fuse: cut ring is 12-gon");

    /* expected fused counts: V=8+2N, E=12+3N, F=6+N, R=2, S=1, G=1 */
    int V,E,F,R;
    int resid = brep_euler_residual(&brep, 1, &V,&E,&F,&R);
    CHECK(V == 8 + 2*N,  "fuse: V == 8 + 2N");
    CHECK(E == 12 + 3*N, "fuse: E == 12 + 3N");
    CHECK(F == 6 + N,    "fuse: F == 6 + N");
    CHECK(R == 2,        "fuse: R == 2 (two cap holes)");
    CHECK(resid == 0,    "fuse: generalized Euler residual == 0 (genus 1)");

    /* exactly two faces are holed, each with an inner loop of N edges */
    int holed = 0, inner_ok = 0;
    for (uint16_t fi=0; fi<brep.faces.count; fi++){
        Face *f = brep_face(&brep, fi);
        if (f->inner_count == 1) { holed++;
            if (loop_len(&brep, f->inner[0]) == N) inner_ok++;
        }
        CHECK(f->inner_count <= 1, "fuse: no face has more than one hole");
    }
    CHECK(holed == 2,    "fuse: exactly two pierced caps are faces-with-holes");
    CHECK(inner_ok == 2, "fuse: each inner loop has N half-edges");

    CHECK(count_unpaired(&brep) == 0, "fuse: no unpaired half-edges remain");
    CHECK(all_twins_antiparallel(&brep), "fuse: all twins anti-parallel");
    CHECK(all_faces_wind_outward(&brep),
          "fuse: outer-face signed volume positive (consistent outward winding)");
    CHECK(brep_check_euler(&brep, 0), "fuse: V-E+F is genus-1 (==0)");
}

/* ---- interactive modeling core --------------------------------------------
 * Starting from the demo part, model_begin_extrude on a side wall + confirm
 * grows feat_count by 2 (sketch+extrude), the B-rep face/solid count grows,
 * cancel restores the prior counts, and an undo after confirm reverts while a
 * redo restores. */
static Document g_mdoc;
static History  g_mhist;   /* large; keep off the stack */
/* pick a side-wall face of the demo (normal along +x: not a pierced cap). */
static FaceId find_side_face(Brep *b) {
    for (uint16_t fi = 0; fi < b->faces.count; fi++) {
        Face *f = brep_face(b, fi);
        if (f->normal.x != 0 && f->normal.y == 0 && f->normal.z == 0)
            return fi;
    }
    return BREP_NONE;
}
static void test_modeling_core(void) {
    DECLARE_BREP(brep);
    build_demo_doc(&g_mdoc);
    int ok = model_regen_all(&g_mdoc, &brep);
    CHECK(ok == 1, "model: demo regen via model_regen_all");

    uint16_t feat0   = g_mdoc.feat_count;
    uint16_t faces0  = brep.faces.count;
    uint16_t solids0 = brep.solids.count;

    /* history is initialised on the CLEAN baseline (mirrors main.c at boot). */
    History *hh = &g_mhist;
    hist_init(hh, &g_mdoc);

    FaceId side = find_side_face(&brep);
    CHECK(side != BREP_NONE, "model: found a side-wall face to build on");

    /* plane_from_face: axes are ~unit (1.12) and orthogonal to the normal. */
    SketchPlane pl = plane_from_face(&brep, side);
    mym2_t un = (mym2_t)pl.u_axis.x*pl.u_axis.x + (mym2_t)pl.u_axis.y*pl.u_axis.y
              + (mym2_t)pl.u_axis.z*pl.u_axis.z;
    CHECK(un > (mym2_t)(FX_ONE-200)*(FX_ONE-200)
            && un < (mym2_t)(FX_ONE+200)*(FX_ONE+200),
          "model: plane u_axis is ~unit length (1.12)");
    CHECK(v3_dot(pl.u_axis, pl.normal) == 0 || (v3_dot(pl.u_axis, pl.normal) < 64
            && v3_dot(pl.u_axis, pl.normal) > -64),
          "model: u_axis orthogonal to normal");

    ModelingState m; model_init(&m);
    ok = model_begin_extrude(&m, &g_mdoc, &brep, side, OP_BOSS);
    CHECK(ok == 1, "model: begin boss-extrude regenerated a preview");
    CHECK(m.pending == 1, "model: edit is pending");
    CHECK(g_mdoc.feat_count == feat0 + 2, "model: pending adds 2 features (sketch+extrude)");
    uint16_t faces_prev = brep.faces.count;
    CHECK(brep.faces.count > faces0, "model: preview grew the B-rep face count");
    CHECK(brep.solids.count > solids0, "model: preview added a solid (the boss)");

    /* live distance change re-regens cleanly (counts stable for a cylinder) */
    ok = model_set_distance(&m, &g_mdoc, &brep, 400);
    CHECK(ok == 1, "model: set_distance regenerated");
    CHECK(m.dist == 400, "model: distance updated to 400");
    CHECK(brep.faces.count == faces_prev, "model: face count stable across distance change");

    /* confirm: snapshot to history, clear pending */
    ok = model_confirm(&m, &g_mdoc, &brep, hh);
    CHECK(ok == 1, "model: confirm succeeded");
    CHECK(m.pending == 0, "model: no longer pending after confirm");
    CHECK(g_mdoc.feat_count == feat0 + 2, "model: confirmed features remain");

    /* undo removes the feature (counts revert), redo restores it */
    int moved = hist_undo(hh, &g_mdoc);
    CHECK(moved == 1, "model: undo moved");
    CHECK(g_mdoc.feat_count == feat0, "model: undo reverts feat_count");
    model_regen_all(&g_mdoc, &brep);
    CHECK(brep.faces.count == faces0, "model: undo reverts B-rep faces");

    moved = hist_redo(hh, &g_mdoc);
    CHECK(moved == 1, "model: redo moved");
    CHECK(g_mdoc.feat_count == feat0 + 2, "model: redo restores feat_count");
    model_regen_all(&g_mdoc, &brep);
    CHECK(brep.faces.count > faces0, "model: redo restores the boss faces");

    /* cancel path: begin then cancel restores prior counts */
    ModelingState m2; model_init(&m2);
    uint16_t featc = g_mdoc.feat_count, facesc = brep.faces.count;
    ok = model_begin_extrude(&m2, &g_mdoc, &brep, side, OP_BOSS);
    CHECK(ok == 1, "model: second begin for cancel test");
    CHECK(g_mdoc.feat_count == featc + 2, "model: cancel-test pending added 2");
    model_cancel(&m2, &g_mdoc, &brep);
    CHECK(m2.pending == 0, "model: cancel cleared pending");
    CHECK(g_mdoc.feat_count == featc, "model: cancel restored feat_count");
    CHECK(brep.faces.count == facesc, "model: cancel restored B-rep faces");
}

static void test_sketch_points_shared(void) {
    Sketch2 s; sk_init(&s);
    /* a rectangle should create 4 shared points + 4 lines */
    sk_add_rect(&s, -300,-300, 300,300);
    CHECK(s.pt_count == 4,  "rect makes 4 shared points");
    CHECK(s.ent_count == 4, "rect makes 4 lines");
    /* adding a line that reuses a corner coordinate shares the point */
    SkPointId a = sk_add_point(&s, 300, 300);   /* existing corner */
    CHECK(a < 4, "duplicate-coordinate point is shared, not added");
    CHECK(s.pt_count == 4, "no new point created for shared corner");
}

static void test_sketch_construction(void) {
    Sketch2 s; sk_init(&s);
    SkEntId c = sk_add_circle(&s, 0,0, 150);
    SkEntId guide = sk_add_line(&s, sk_add_point(&s,-500,0), sk_add_point(&s,500,0));
    sk_set_construction(&s, guide, 1);
    CHECK(s.ent[guide].construction == 1, "line flagged construction");
    CHECK(s.ent[c].construction == 0,     "circle stays real geometry");
    /* profile extraction skips construction, finds the circle */
    Vec2i ring[8];
    int r = sk_extract_profile(&s, ring, 8);
    CHECK(r == -150, "profile finds circle (r=150), skips construction line");
}

static void test_sketch_trim(void) {
    Sketch2 s; sk_init(&s);
    SkEntId l = sk_add_line(&s, sk_add_point(&s,0,0), sk_add_point(&s,100,0));
    sk_trim(&s, l, TRIM_TO_CONSTRUCTION);
    CHECK(s.ent[l].construction == 1, "trim-to-construction flips flag");
    SkEntId l2 = sk_add_line(&s, sk_add_point(&s,0,50), sk_add_point(&s,100,50));
    sk_trim(&s, l2, TRIM_DELETE);
    CHECK(s.ent[l2].alive == 0, "trim-delete removes the segment");
}

static void test_sketch_intersection(void) {
    Sketch2 s; sk_init(&s);
    SkEntId a = sk_add_line(&s, sk_add_point(&s,-100,0), sk_add_point(&s,100,0));
    SkEntId b = sk_add_line(&s, sk_add_point(&s,0,-100), sk_add_point(&s,0,100));
    Vec2i x;
    int hit = sk_line_line(&s, a, b, &x);
    CHECK(hit == 1, "perpendicular lines intersect");
    CHECK(x.u == 0 && x.v == 0, "intersection at origin");
}

static void test_sketch_constraints(void) {
    Sketch2 s; sk_init(&s);
    SkPointId p0 = sk_add_point(&s, 0, 0);
    SkPointId p1 = sk_add_point(&s, 100, 30);     /* not level */
    SkEntId   ln = sk_add_line(&s, p0, p1);
    sk_add_constraint(&s, SKC_HORIZONTAL, ln, SK_NONE, SK_NONE, SK_NONE, 0);
    int unsolved = sk_resolve_direct(&s);
    CHECK(s.pt[p1].v == s.pt[p0].v, "horizontal constraint levels the line (direct)");
    CHECK(unsolved == 0, "horizontal is a direct rule (0 unsolved)");

    /* a dimension is recorded but needs the future solver */
    sk_add_constraint(&s, SKC_DIMENSION, ln, SK_NONE, SK_NONE, SK_NONE, 1000);
    unsolved = sk_resolve_direct(&s);
    CHECK(unsolved == 1, "dimension recorded as unsolved (awaits Newton solver)");
}

/* helper: integer length of a Sketch2 line entity */
static mym_t line_length(const Sketch2 *s, SkEntId e) {
    const SkEntity *L = &s->ent[e];
    mym2_t dx = s->pt[L->p1].u - s->pt[L->p0].u;
    mym2_t dy = s->pt[L->p1].v - s->pt[L->p0].v;
    mym2_t l2 = dx*dx + dy*dy, lo=0, hi=l2<4096?4096:l2, r=0;
    while (lo<=hi){ mym2_t m=(lo+hi)/2; if(m*m<=l2){r=m;lo=m+1;}else hi=m-1; }
    return (mym_t)r;
}

/* PART 2: iterative solver drives a DIMENSION to the target length. */
static void test_solve_dimension(void) {
    Sketch2 s; sk_init(&s);
    SkPointId p0 = sk_add_point(&s, 0, 0);
    SkPointId p1 = sk_add_point(&s, 100, 0);      /* current length 100 */
    SkEntId   ln = sk_add_line(&s, p0, p1);
    /* anchor p0 so the solver moves p1 */
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, p0, SK_NONE, 0);
    sk_add_constraint(&s, SKC_DIMENSION, ln, SK_NONE, SK_NONE, SK_NONE, 1000);
    int unsolved = sk_solve(&s);
    mym_t L = line_length(&s, ln);
    printf("      solved length = %d (target 1000)\n", (int)L);
    CHECK(unsolved == 0, "dimension: solver reports 0 unsolved");
    CHECK(L >= 1000-4 && L <= 1000+4, "dimension: line length within +-4 of 1000");
    CHECK(s.pt[p0].u == 0 && s.pt[p0].v == 0, "dimension: fixed point unmoved");
}

/* PART 2: solver drives a DIMENSION between two free points symmetrically. */
static void test_solve_dimension_pts(void) {
    Sketch2 s; sk_init(&s);
    SkPointId a = sk_add_point(&s, -50, 0);
    SkPointId b = sk_add_point(&s,  50, 0);       /* distance 100 */
    sk_add_constraint(&s, SKC_DIMENSION, SK_NONE, SK_NONE, a, b, 600);
    int unsolved = sk_solve(&s);
    mym2_t dx = s.pt[b].u - s.pt[a].u, dy = s.pt[b].v - s.pt[a].v;
    mym2_t l2 = dx*dx+dy*dy, lo=0,hi=l2<4096?4096:l2,r=0;
    while(lo<=hi){mym2_t m=(lo+hi)/2; if(m*m<=l2){r=m;lo=m+1;}else hi=m-1;}
    printf("      point distance = %d (target 600)\n", (int)r);
    CHECK(unsolved == 0, "dim-pts: solver reports 0 unsolved");
    CHECK(r >= 600-4 && r <= 600+4, "dim-pts: distance within +-4 of 600");
}

/* PART 2: PARALLEL constraint converges (two lines end up parallel). */
static void test_solve_parallel(void) {
    Sketch2 s; sk_init(&s);
    /* reference line along +u, anchored */
    SkPointId a0 = sk_add_point(&s, 0, 0), a1 = sk_add_point(&s, 200, 0);
    SkEntId la = sk_add_line(&s, a0, a1);
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, a0, SK_NONE, 0);
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, a1, SK_NONE, 0);
    /* second line skewed; anchor one end so only the other rotates */
    SkPointId b0 = sk_add_point(&s, 0, 100), b1 = sk_add_point(&s, 150, 220);
    SkEntId lb = sk_add_line(&s, b0, b1);
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, b0, SK_NONE, 0);
    sk_add_constraint(&s, SKC_PARALLEL, la, lb, SK_NONE, SK_NONE, 0);
    int unsolved = sk_solve(&s);
    /* cross product of the two directions should be ~0 when parallel */
    mym2_t ax = s.pt[a1].u-s.pt[a0].u, ay = s.pt[a1].v-s.pt[a0].v;
    mym2_t bx = s.pt[b1].u-s.pt[b0].u, by = s.pt[b1].v-s.pt[b0].v;
    mym2_t cross = ax*by - ay*bx;
    printf("      parallel residual v = %d (b dir = %d,%d)\n",
           (int)(s.pt[b1].v - s.pt[b0].v), (int)bx, (int)by);
    CHECK(unsolved == 0, "parallel: solver reports 0 unsolved");
    /* parallel to +u axis means b's v-component ~ 0 */
    CHECK(cross >= -4*200 && cross <= 4*200, "parallel: cross product ~0");
}

/* PART 2: PERPENDICULAR constraint converges. */
static void test_solve_perpendicular(void) {
    Sketch2 s; sk_init(&s);
    SkPointId a0 = sk_add_point(&s, 0, 0), a1 = sk_add_point(&s, 200, 0);
    SkEntId la = sk_add_line(&s, a0, a1);
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, a0, SK_NONE, 0);
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, a1, SK_NONE, 0);
    SkPointId b0 = sk_add_point(&s, 50, 0), b1 = sk_add_point(&s, 250, 40);
    SkEntId lb = sk_add_line(&s, b0, b1);
    sk_add_constraint(&s, SKC_FIX, SK_NONE, SK_NONE, b0, SK_NONE, 0);
    sk_add_constraint(&s, SKC_PERPENDICULAR, la, lb, SK_NONE, SK_NONE, 0);
    int unsolved = sk_solve(&s);
    mym2_t ax = s.pt[a1].u-s.pt[a0].u, ay = s.pt[a1].v-s.pt[a0].v;
    mym2_t bx = s.pt[b1].u-s.pt[b0].u, by = s.pt[b1].v-s.pt[b0].v;
    mym2_t dot = ax*bx + ay*by;
    printf("      perp dot = %d (b dir = %d,%d)\n", (int)dot, (int)bx, (int)by);
    CHECK(unsolved == 0, "perpendicular: solver reports 0 unsolved");
    CHECK(dot >= -4*200 && dot <= 4*200, "perpendicular: dot product ~0");
}

/* Profile extraction from a migrated Sketch2 rect and circle. */
static void test_profile_extract_sketch2(void) {
    /* rect: 4-point ring matching the legacy winding */
    {
        Sketch2 s; sk_init(&s);
        sk_add_rect(&s, -300,-300, 300,300);
        Vec2i ring[8];
        int n = sk_extract_profile(&s, ring, 8);
        CHECK(n == 4, "extract: rect yields a 4-point ring");
        CHECK(ring[0].u==-300 && ring[0].v==-300, "extract: ring[0]=(-300,-300)");
        CHECK(ring[1].u== 300 && ring[1].v==-300, "extract: ring[1]=( 300,-300)");
        CHECK(ring[2].u== 300 && ring[2].v== 300, "extract: ring[2]=( 300, 300)");
        CHECK(ring[3].u==-300 && ring[3].v== 300, "extract: ring[3]=(-300, 300)");
    }
    /* circle: negative count encodes radius, ring[0] = center */
    {
        Sketch2 s; sk_init(&s);
        sk_add_circle(&s, 0,0, 150);
        Vec2i ring[8];
        int n = sk_extract_profile(&s, ring, 8);
        CHECK(n == -150, "extract: circle yields -radius (-150)");
        CHECK(ring[0].u==0 && ring[0].v==0, "extract: circle center at origin");
    }
    /* lift to 3D via profile_to_ring and confirm extrude builds a box */
    {
        DECLARE_BREP(brep);
        Sketch2 sk; mk_rect(&sk, -300,-300, 300,300);
        SketchPlane pl = plane_xy();
        OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
        SolidId sid = op_extrude(&brep, &sk, &pl, &pr, BREP_NONE, 1);
        CHECK(sid != BREP_NONE && brep.verts.count == 8,
              "extract: migrated rect extrudes to an 8-vertex box");
    }
}

/* helper: a document with one extrude whose distance we vary */
static void doc_with_dist(Document *d, mym_t dist) {
    doc_init(d, "Part1");
    Feature *sk = doc_add(d, FEAT_SKETCH, "Sketch1");
    sk->plane = plane_xy(); sk_init(&sk->sketch);
    sk_add_rect(&sk->sketch, -300,-300, 300,300);
    Feature *ex = doc_add(d, FEAT_EXTRUDE, "Boss-Extrude1");
    ex->p.extrude.sketch_id = sk->id; ex->p.extrude.dist = dist;
    ex->p.extrude.dir = 1; ex->p.extrude.op = OP_BOSS;
    ex->p.extrude.end = END_BLIND; ex->p.extrude.target_face = 0;
    ex->depends_on[0] = sk->id; ex->dep_count = 1;
}
static mym_t doc_dist(Document *d) {
    for (uint16_t i=0;i<d->feat_count;i++)
        if (d->feat[i].kind==FEAT_EXTRUDE) return d->feat[i].p.extrude.dist;
    return -1;
}

static History g_hist;  /* large; keep off the stack */

static void test_undo_redo(void) {
    Document d;
    doc_with_dist(&d, 100);
    hist_init(&g_hist, &d);

    /* edit: 100 -> 200, commit */
    d.feat[1].p.extrude.dist = 200; hist_commit(&g_hist, &d);
    /* edit: 200 -> 300, commit */
    d.feat[1].p.extrude.dist = 300; hist_commit(&g_hist, &d);

    CHECK(doc_dist(&d) == 300,             "current state is 300");
    CHECK(hist_can_undo(&g_hist),          "can undo after edits");

    hist_undo(&g_hist, &d);
    CHECK(doc_dist(&d) == 200,             "undo -> 200");
    hist_undo(&g_hist, &d);
    CHECK(doc_dist(&d) == 100,             "undo -> 100 (original)");
    CHECK(!hist_can_undo(&g_hist),         "cannot undo past the start");

    hist_redo(&g_hist, &d);
    CHECK(doc_dist(&d) == 200,             "redo -> 200");

    /* new edit after an undo discards the redo tail (300 is gone) */
    d.feat[1].p.extrude.dist = 250; hist_commit(&g_hist, &d);
    CHECK(!hist_can_redo(&g_hist),         "new edit clears redo tail");
    hist_undo(&g_hist, &d);
    CHECK(doc_dist(&d) == 200,             "undo after branch -> 200");
}

static void test_undo_depth_cap(void) {
    Document d;
    doc_with_dist(&d, 0);
    hist_init(&g_hist, &d);
    /* push 15 edits (0,10,20,...,150); only last 10 should be undoable */
    for (int i = 1; i <= 15; ++i) {
        d.feat[1].p.extrude.dist = (mym_t)(i * 10);
        hist_commit(&g_hist, &d);
    }
    CHECK(doc_dist(&d) == 150, "after 15 edits, current is 150");
    /* undo as far as possible and count steps */
    int steps = 0;
    while (hist_can_undo(&g_hist)) { hist_undo(&g_hist, &d); steps++; }
    CHECK(steps == HIST_DEPTH - 1, "exactly 9 undo steps from a full 10-deep ring");
    /* oldest reachable should be the 6th edit (60), since 0..50 were evicted */
    CHECK(doc_dist(&d) == 60, "oldest retained snapshot is 60 (older evicted)");
}

int main(void) {
    printf("== MiniCAD-PSX host kernel tests ==\n");
    test_mm_str();
    test_sine();
    test_save_roundtrip();
    test_save_roundtrip_sketch2();
    test_extrude_boss();
    test_extrude_circle();
    test_through_all_is_cut_only();
    test_revolve();
    test_winding_consistency();
    test_through_all_fuses_one_solid();
    test_modeling_core();
    test_sketch_points_shared();
    test_sketch_construction();
    test_sketch_trim();
    test_sketch_intersection();
    test_sketch_constraints();
    test_solve_dimension();
    test_solve_dimension_pts();
    test_solve_parallel();
    test_solve_perpendicular();
    test_profile_extract_sketch2();
    test_undo_redo();
    test_undo_depth_cap();
    printf("\n%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails;
}
