/* test_kernel.c — host-side unit tests (no PlayStation needed).
 * Build with MINICAD_HOST defined. Returns non-zero on first failure. */
#include "minicad/fixed.h"
#include "minicad/save.h"
#include "minicad/feature.h"
#include "minicad/sketch.h"
#include "minicad/history.h"
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
    sk->sketch.count = 1;
    sk->sketch.ent[0].kind = SK_RECT;
    sk->sketch.ent[0].a = (Vec2i){-300,-300};
    sk->sketch.ent[0].b = (Vec2i){ 300, 300};
    Feature *ex = doc_add(&a, FEAT_EXTRUDE, "Boss-Extrude1");
    ex->p.extrude.sketch_id = sk->id; ex->p.extrude.dist = 600;
    ex->p.extrude.dir = 1; ex->p.extrude.op = OP_BOSS;
    ex->p.extrude.end = END_BLIND; ex->p.extrude.target_face = 0;
    ex->depends_on[0] = sk->id; ex->dep_count = 1;

    uint8_t buf[512];
    int n = mcad_encode(&a, buf, sizeof buf);
    CHECK(n > 0,            "encode produced bytes");
    printf("      encoded part size: %d bytes\n", n);
    CHECK(n < 64,           "part encodes compactly (<64 bytes)");

    int ok = mcad_decode(&b, buf, n);
    CHECK(ok == 1,          "decode succeeded (magic+crc)");
    CHECK(b.feat_count == a.feat_count, "feature count round-trips");
    Feature *bex = doc_find(&b, ex->id);
    CHECK(bex && bex->p.extrude.dist == 600, "extrude distance round-trips");
    CHECK(bex && bex->p.extrude.op == OP_BOSS && bex->p.extrude.end == END_BLIND,
          "op/end round-trip");
}

/* shared brep backing for op tests */
#define DECLARE_BREP(name) \
    static Vertex name##v[2048]; static HalfEdge name##he[8192]; \
    static Edge name##e[4096]; static Loop name##l[2048]; \
    static Face name##f[1024]; static Shell name##s[64]; static Solid name##so[64]; \
    Brep name; brep_init(&name, name##v,2048, name##he,8192, name##e,4096, \
                         name##l,2048, name##f,1024, name##s,64, name##so,64)

static void test_extrude_boss(void) {
    DECLARE_BREP(brep);
    Sketch sk; sk.plane = plane_xy(); sk.count = 1;
    sk.ent[0].kind = SK_RECT;
    sk.ent[0].a = (Vec2i){-300,-300}; sk.ent[0].b = (Vec2i){300,300};
    OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
    SolidId sid = op_extrude(&brep, &sk, &pr, BREP_NONE, 1);
    CHECK(sid != BREP_NONE,        "boss extrude returned a solid");
    CHECK(brep.verts.count == 8,   "box has 8 vertices");
    CHECK(brep.faces.count == 6,   "box has 6 faces");
    CHECK(brep.edges.count == 12,  "box has 12 edges (twins paired)");
    CHECK(brep_check_euler(&brep, sid), "box satisfies Euler V-E+F=2");
}

static void test_extrude_circle(void) {
    DECLARE_BREP(brep);
    Sketch sk; sk.plane = plane_xy(); sk.count = 1;
    sk.ent[0].kind = SK_CIRCLE; sk.ent[0].a = (Vec2i){0,0}; sk.ent[0].radius = 150;
    OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
    SolidId sid = op_extrude(&brep, &sk, &pr, BREP_NONE, 1);
    CHECK(sid != BREP_NONE,         "circle extrude (cylinder) returned a solid");
    /* Ø30 -> radius 150 -> 12 segments. 2 caps + 12 sides = 14 faces, 24 verts. */
    CHECK(brep.verts.count == 24,   "cylinder has 24 vertices (12-gon x2)");
    CHECK(brep.faces.count == 14,   "cylinder has 14 faces (12 sides + 2 caps)");
    CHECK(brep_check_euler(&brep, sid), "cylinder satisfies Euler");
}

static void test_through_all_is_cut_only(void) {
    DECLARE_BREP(brep);
    /* put a box in so solid_extent has something to measure */
    Sketch box; box.plane=plane_xy(); box.count=1; box.ent[0].kind=SK_RECT;
    box.ent[0].a=(Vec2i){-300,-300}; box.ent[0].b=(Vec2i){300,300};
    OpParams bp={OP_BOSS,END_BLIND,600,1,BREP_NONE};
    op_extrude(&brep,&box,&bp,BREP_NONE,1);

    OpParams cut = { OP_CUT, END_THROUGH_ALL, 0, 1, BREP_NONE };
    int ok = resolve_end_condition(&brep, BREP_NONE, &box.plane, &cut, 0);
    CHECK(ok == 1 && cut.dist > 600, "through-all cut resolves to clear the solid");
    /* Through-all spans the body extent ALONG THE CUT NORMAL (+2mm over-cut),
     * not the whole-model solid_extent: box z is [0,600], normal +z, so
     * extent = 600 -> dist = 620. (Stops the hole shooting far past the part.) */
    CHECK(cut.dist == 620, "through-all spans body extent along normal + over-cut");

    OpParams bad = { OP_BOSS, END_THROUGH_ALL, 0, 1, BREP_NONE };
    int bok = resolve_end_condition(&brep, BREP_NONE, &box.plane, &bad, 0);
    CHECK(bok == 0, "through-all rejected for a boss (cut-only)");
}

static void test_revolve(void) {
    DECLARE_BREP(brep);
    Sketch sk; sk.plane = plane_xy(); sk.count = 1;
    sk.ent[0].kind = SK_RECT;
    sk.ent[0].a = (Vec2i){200, -50}; sk.ent[0].b = (Vec2i){300, 50};
    OpParams pr = { OP_BOSS, END_BLIND, 0, 1, BREP_NONE };
    Vec3i ax_o={0,0,0}, ax_d={0,FX_ONE,0};
    SolidId sid = op_revolve(&brep, &sk, &pr, ax_o, ax_d, SIN_LEN, 12, BREP_NONE, 1);
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
        Sketch sk; sk.plane = plane_xy(); sk.count = 1;
        sk.ent[0].kind = SK_RECT;
        sk.ent[0].a = (Vec2i){-300,-300}; sk.ent[0].b = (Vec2i){300,300};
        OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
        SolidId sid = op_extrude(&brep, &sk, &pr, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: cube built");
        CHECK(all_faces_wind_outward(&brep),  "winding: all cube faces point outward");
        CHECK(all_twins_antiparallel(&brep),  "winding: cube twins anti-parallel");
    }
    /* cylinder */
    {
        DECLARE_BREP(brep);
        Sketch sk; sk.plane = plane_xy(); sk.count = 1;
        sk.ent[0].kind = SK_CIRCLE; sk.ent[0].a = (Vec2i){0,0}; sk.ent[0].radius = 150;
        OpParams pr = { OP_BOSS, END_BLIND, 600, 1, BREP_NONE };
        SolidId sid = op_extrude(&brep, &sk, &pr, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: cylinder built");
        CHECK(all_faces_wind_outward(&brep), "winding: all cylinder faces point outward");
        CHECK(all_twins_antiparallel(&brep), "winding: cylinder twins anti-parallel");
    }
    /* full revolve (solid of revolution) */
    {
        DECLARE_BREP(brep);
        Sketch sk; sk.plane = plane_xy(); sk.count = 1;
        sk.ent[0].kind = SK_RECT;
        sk.ent[0].a = (Vec2i){200,-50}; sk.ent[0].b = (Vec2i){300,50};
        OpParams pr = { OP_BOSS, END_BLIND, 0, 1, BREP_NONE };
        Vec3i ax_o={0,0,0}, ax_d={0,FX_ONE,0};
        SolidId sid = op_revolve(&brep, &sk, &pr, ax_o, ax_d, SIN_LEN, 12, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: revolve built");
        CHECK(all_faces_wind_outward(&brep), "winding: all revolve faces point outward");
        CHECK(all_twins_antiparallel(&brep), "winding: revolve twins anti-parallel");
    }
    /* open (partial-sweep) revolve exercises the end caps too */
    {
        DECLARE_BREP(brep);
        Sketch sk; sk.plane = plane_xy(); sk.count = 1;
        sk.ent[0].kind = SK_RECT;
        sk.ent[0].a = (Vec2i){200,-50}; sk.ent[0].b = (Vec2i){300,50};
        OpParams pr = { OP_BOSS, END_BLIND, 0, 1, BREP_NONE };
        Vec3i ax_o={0,0,0}, ax_d={0,FX_ONE,0};
        SolidId sid = op_revolve(&brep, &sk, &pr, ax_o, ax_d, SIN_LEN/2, 6, BREP_NONE, 1);
        CHECK(sid != BREP_NONE, "winding: open revolve built");
        CHECK(all_faces_wind_outward(&brep), "winding: all open-revolve faces point outward");
        CHECK(all_twins_antiparallel(&brep), "winding: open-revolve twins anti-parallel");
    }
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

/* helper: a document with one extrude whose distance we vary */
static void doc_with_dist(Document *d, mym_t dist) {
    doc_init(d, "Part1");
    Feature *sk = doc_add(d, FEAT_SKETCH, "Sketch1");
    sk->sketch.count = 1; sk->sketch.ent[0].kind = SK_RECT;
    sk->sketch.ent[0].a = (Vec2i){-300,-300}; sk->sketch.ent[0].b = (Vec2i){300,300};
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
    test_extrude_boss();
    test_extrude_circle();
    test_through_all_is_cut_only();
    test_revolve();
    test_winding_consistency();
    test_sketch_points_shared();
    test_sketch_construction();
    test_sketch_trim();
    test_sketch_intersection();
    test_sketch_constraints();
    test_undo_redo();
    test_undo_depth_cap();
    printf("\n%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails;
}
