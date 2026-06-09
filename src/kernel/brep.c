/* brep.c — half-edge B-rep store: init, accessors, Euler check. */
#include "minicad/brep.h"

void brep_init(Brep *b,
               void *v_buf,  uint16_t v_cap,
               void *he_buf, uint16_t he_cap,
               void *e_buf,  uint16_t e_cap,
               void *l_buf,  uint16_t l_cap,
               void *f_buf,  uint16_t f_cap,
               void *s_buf,  uint16_t s_cap,
               void *so_buf, uint16_t so_cap) {
    pool_init(&b->verts,  v_buf,  (uint16_t)sizeof(Vertex),   v_cap,  "verts");
    pool_init(&b->hedges, he_buf, (uint16_t)sizeof(HalfEdge), he_cap, "hedges");
    pool_init(&b->edges,  e_buf,  (uint16_t)sizeof(Edge),     e_cap,  "edges");
    pool_init(&b->loops,  l_buf,  (uint16_t)sizeof(Loop),     l_cap,  "loops");
    pool_init(&b->faces,  f_buf,  (uint16_t)sizeof(Face),     f_cap,  "faces");
    pool_init(&b->shells, s_buf,  (uint16_t)sizeof(Shell),    s_cap,  "shells");
    pool_init(&b->solids, so_buf, (uint16_t)sizeof(Solid),    so_cap, "solids");
    b->aabb_valid = 0;
    b->aabb_min = b->aabb_max = (Vec3i){0,0,0};
}

VertId brep_add_vertex(Brep *b, Vec3i p) {
    VertId id = pool_alloc(&b->verts);
    if (id == BREP_NONE) return BREP_NONE;
    Vertex *v = (Vertex *)pool_get(&b->verts, id);
    v->p = p;
    /* fold into the running AABB (see Brep::aabb_* in brep.h) */
    if (!b->aabb_valid) { b->aabb_min = b->aabb_max = p; b->aabb_valid = 1; }
    else {
        if (p.x < b->aabb_min.x) b->aabb_min.x = p.x;
        if (p.y < b->aabb_min.y) b->aabb_min.y = p.y;
        if (p.z < b->aabb_min.z) b->aabb_min.z = p.z;
        if (p.x > b->aabb_max.x) b->aabb_max.x = p.x;
        if (p.y > b->aabb_max.y) b->aabb_max.y = p.y;
        if (p.z > b->aabb_max.z) b->aabb_max.z = p.z;
    }
    return id;
}

Vertex   *brep_vert (Brep *b, VertId id)  { return (Vertex   *)pool_get(&b->verts,  id); }
HalfEdge *brep_he   (Brep *b, HEdgeId id) { return (HalfEdge *)pool_get(&b->hedges, id); }
Face     *brep_face (Brep *b, FaceId id)  { return (Face     *)pool_get(&b->faces,  id); }

/* ---- twin-pairing table ----
 * An undirected edge between verts (a,b) gets the key (min<<16)|max. The first
 * half-edge to claim that key is stored; the second to arrive pairs as its twin
 * and creates the shared Edge record. Open-addressed linear probe. */
static uint32_t edge_key(VertId a, VertId b) {
    VertId lo = a < b ? a : b, hi = a < b ? b : a;
    return ((uint32_t)lo << 16) | (uint32_t)hi;
}

void brep_begin_solid(Brep *b) {
    b->etab_used = 0;
    for (int i = 0; i < BREP_EDGE_TABLE; ++i) { b->etab[i].key = 0xFFFFFFFFu; b->etab[i].he = BREP_NONE; }
}

/* Returns the twin half-edge if one was waiting, else BREP_NONE and stores
 * `he` as waiting for its future twin. */
static HEdgeId edge_pair(Brep *b, VertId a, VertId v, HEdgeId he) {
    uint32_t k = edge_key(a, v);
    int idx = (int)(k % BREP_EDGE_TABLE);
    for (int probe = 0; probe < BREP_EDGE_TABLE; ++probe) {
        int i = (idx + probe) % BREP_EDGE_TABLE;
        if (b->etab[i].key == k && b->etab[i].he != BREP_NONE) {
            HEdgeId twin = b->etab[i].he;
            b->etab[i].he = BREP_NONE;          /* consume the slot */
            return twin;
        }
        if (b->etab[i].key == 0xFFFFFFFFu) {
            b->etab[i].key = k; b->etab[i].he = he; b->etab_used++;
            return BREP_NONE;
        }
    }
    return BREP_NONE; /* table full -> twin left unpaired (boundary edge) */
}

/* ---- WINDING CONVENTION (load-bearing for the PS1 backface cull) ----
 * Every OUTER face loop is wound COUNTER-CLOCKWISE when viewed from OUTSIDE the
 * solid (i.e. looking at the face along -normal toward the solid). With that
 * convention, the exact integer normal computed in brep_face_plane as
 *     cross(p1 - p0, p2 - p0)
 * points OUTWARD from the solid (right-hand rule). Because adjacent faces share
 * an edge traversed in opposite directions, every interior half-edge twin pair
 * runs anti-parallel — the hallmark of a consistently oriented 2-manifold.
 *
 * The renderer relies on this: it can apply ONE GTE NCLIP sign rule to cull
 * back faces, because all front faces project with the same screen handedness.
 * The op builders (build_prism caps + sides, revolve ring stitch) MUST emit
 * rings in this CCW-from-outside order. host_tests/test_kernel.c asserts it
 * (test_winding_consistency) for the demo cube and cylinder. */

/* Build one loop's worth of half-edges around a ring; returns the first HE.
 * Wires next/prev within the loop and pairs twins across loops/faces. */
static HEdgeId build_loop(Brep *b, const VertId *ring, int n, LoopId loop) {
    HEdgeId first = BREP_NONE, prev = BREP_NONE;
    for (int i = 0; i < n; ++i) {
        HEdgeId hid = pool_alloc(&b->hedges);
        if (hid == BREP_NONE) return BREP_NONE;
        HalfEdge *he = brep_he(b, hid);
        he->origin = ring[i];
        he->loop   = loop;
        he->twin   = BREP_NONE;
        he->edge   = BREP_NONE;
        if (i == 0) first = hid;
        if (prev != BREP_NONE) { brep_he(b, prev)->next = hid; he->prev = prev; }
        prev = hid;

        /* pair twin on edge (ring[i] -> ring[i+1]) */
        VertId vnext = ring[(i + 1) % n];
        HEdgeId twin = edge_pair(b, ring[i], vnext, hid);
        if (twin != BREP_NONE) {
            he->twin = twin; brep_he(b, twin)->twin = hid;
            EdgeId eid = pool_alloc(&b->edges);
            if (eid != BREP_NONE) {
                Edge *e = (Edge *)pool_get(&b->edges, eid);
                e->he = hid; e->curve_kind = CURVE_LINE;
                he->edge = eid; brep_he(b, twin)->edge = eid;
            }
        }
    }
    /* close the loop */
    brep_he(b, prev)->next = first;
    brep_he(b, first)->prev = prev;
    return first;
}

LoopId brep_add_face_hole(Brep *b, FaceId fid, const VertId *ring, int n) {
    if (n < 3) return BREP_NONE;
    Face *f = brep_face(b, fid);
    if (!f || f->inner_count >= 4) return BREP_NONE;
    LoopId li = pool_alloc(&b->loops);
    if (li == BREP_NONE) return BREP_NONE;
    Loop *IL = (Loop *)pool_get(&b->loops, li);
    IL->face = fid;
    IL->first_he = build_loop(b, ring, n, li);
    if (IL->first_he == BREP_NONE) return BREP_NONE;
    f->inner[f->inner_count++] = li;
    return li;
}

int brep_shell_add_face(Brep *b, ShellId shid, FaceId fid) {
    Shell *sh = (Shell *)pool_get(&b->shells, shid);
    if (!sh || sh->face_count >= 64) return 0;
    sh->faces[sh->face_count++] = fid;
    Face *f = brep_face(b, fid);
    if (f) f->shell = shid;
    return 1;
}

void brep_face_plane(Brep *b, FaceId fid) {
    Face *f = brep_face(b, fid);
    Loop *lp = (Loop *)pool_get(&b->loops, f->outer);
    HEdgeId a = lp->first_he;
    Vec3i p0 = brep_vert(b, brep_he(b, a)->origin)->p;
    Vec3i p1 = brep_vert(b, brep_he(b, brep_he(b, a)->next)->origin)->p;
    Vec3i p2 = brep_vert(b, brep_he(b, brep_he(b, brep_he(b, a)->next)->next)->origin)->p;
    f->normal  = v3_cross(v3_sub(p1, p0), v3_sub(p2, p0));
    f->plane_d = v3_dot(f->normal, p0);
}

FaceId brep_build_face(Brep *b, const VertId *ring, int n,
                       const VertId *const *inner_rings, const int *inner_n,
                       int inner_count, uint16_t feature_id) {
    if (n < 3) return BREP_NONE;
    FaceId fid = pool_alloc(&b->faces);
    if (fid == BREP_NONE) return BREP_NONE;
    Face *f = brep_face(b, fid);
    f->inner_count = 0;
    f->feature_id  = feature_id;
    f->shell       = BREP_NONE;

    LoopId lo = pool_alloc(&b->loops);
    Loop *L = (Loop *)pool_get(&b->loops, lo);
    L->face = fid;
    L->first_he = build_loop(b, ring, n, lo);
    f->outer = lo;

    for (int k = 0; k < inner_count && k < 4; ++k) {
        LoopId li = pool_alloc(&b->loops);
        Loop *IL = (Loop *)pool_get(&b->loops, li);
        IL->face = fid;
        IL->first_he = build_loop(b, inner_rings[k], inner_n[k], li);
        f->inner[f->inner_count++] = li;
    }
    brep_face_plane(b, fid);
    return fid;
}

/* Count live entities by walking pool occupancy. For the MVP we count via the
 * pool's `count` field (live elements). Edges are counted as half the
 * half-edge count for a closed manifold (each edge = 2 half-edges). */
int brep_check_euler(const Brep *b, SolidId solid) {
    (void)solid; /* single-solid MVP: check the whole store */
    int V = b->verts.count;
    int E = b->edges.count;        /* full edges */
    int F = b->faces.count;
    /* V - E + F == 2 for a simple closed polyhedron (genus 0).
     * A cube-with-through-hole is genus 1 -> V - E + F == 0.
     * We accept either here and let the caller assert the expected genus. */
    int chi = V - E + F;
    return (chi == 2 || chi == 0);
}

int brep_euler_residual(const Brep *b, int genus,
                        int *Vout, int *Eout, int *Fout, int *Rout) {
    int V = b->verts.count;
    int E = b->edges.count;
    int F = b->faces.count;
    int S = b->solids.count;
    /* R = total inner (hole) rings over all faces. The MVP never frees faces,
     * so a linear walk of the pool matches the live set (same assumption the
     * test helpers make). */
    int R = 0;
    for (uint16_t fi = 0; fi < b->faces.count; ++fi) {
        const Face *f = (const Face *)pool_get((Pool *)&b->faces, fi);
        if (f) R += f->inner_count;
    }
    if (Vout) *Vout = V;
    if (Eout) *Eout = E;
    if (Fout) *Fout = F;
    if (Rout) *Rout = R;
    /* V - E + F - R = 2(S - G)  =>  residual = V - E + F - R - 2(S - G). */
    return V - E + F - R - 2 * (S - genus);
}
