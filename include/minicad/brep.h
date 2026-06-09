/* brep.h — integer half-edge boundary representation.
 *
 * Entities live in pools and reference each other by 16-bit PoolId handles
 * (not pointers): half the size, serialization-friendly, realloc-safe.
 * Faces are planar with an EXACT integer plane equation, so point-on-face
 * tests are pure integer arithmetic.
 *
 * Topology mirrors standard CAD kernels (FreeCAD/OCCT-style half-edge); the
 * only unusual part is that coordinates are myriometers and handles are 16-bit.
 */
#ifndef MINICAD_BREP_H
#define MINICAD_BREP_H

#include "minicad/ivec.h"
#include "minicad/mem.h"

typedef PoolId VertId, HEdgeId, EdgeId, LoopId, FaceId, ShellId, SolidId;
#define BREP_NONE POOL_NONE

typedef enum { CURVE_LINE = 0, CURVE_ARC = 1 } CurveKind;

typedef struct { Vec3i p; } Vertex;

typedef struct {
    VertId  origin;
    HEdgeId twin, next, prev;
    LoopId  loop;
    EdgeId  edge;
} HalfEdge;

typedef struct { HEdgeId he; uint8_t curve_kind; } Edge;
typedef struct { HEdgeId first_he; FaceId face; }  Loop;

typedef struct {
    LoopId  outer;        /* outer boundary loop                       */
    LoopId  inner[4];     /* up to 4 holes (e.g. the demo cube face)   */
    uint8_t inner_count;
    Vec3i   normal;       /* exact integer normal (not unit-length)    */
    mym2_t  plane_d;      /* plane: normal . X = plane_d               */
    ShellId shell;
    uint16_t feature_id;  /* which feature created this face (for select/highlight) */
} Face;

typedef struct {
    FaceId  faces[64];    /* small fixed cap per shell for the MVP     */
    uint8_t face_count;
    SolidId solid;
} Shell;

typedef struct { ShellId outer; } Solid;

/* The whole B-rep store. Backing memory is provided by the caller (arena),
 * so the kernel never mallocs. */
#define BREP_EDGE_TABLE 512   /* open-addressing twin-pairing scratch */
typedef struct {
    Pool verts, hedges, edges, loops, faces, shells, solids;
    /* twin-pairing table (scoped per solid via brep_begin_solid):
     * maps an undirected edge key -> the half-edge waiting for its twin. */
    struct { uint32_t key; HEdgeId he; } etab[BREP_EDGE_TABLE];
    int etab_used;
    /* Running AABB updated on every brep_add_vertex, so callers that need the
     * model extent (e.g. through-all cut depth) never have to re-scan the vert
     * pool or risk reading freed slots. `aabb_valid` is 0 until the first vert. */
    Vec3i aabb_min, aabb_max;
    int   aabb_valid;
} Brep;

/* Wire a Brep onto caller-supplied backing buffers (sized by the caller). */
void brep_init(Brep *b,
               void *v_buf,  uint16_t v_cap,
               void *he_buf, uint16_t he_cap,
               void *e_buf,  uint16_t e_cap,
               void *l_buf,  uint16_t l_cap,
               void *f_buf,  uint16_t f_cap,
               void *s_buf,  uint16_t s_cap,
               void *so_buf, uint16_t so_cap);

VertId brep_add_vertex(Brep *b, Vec3i p);

/* ---- face construction with automatic twin pairing ----
 * Build a face from an ordered ring of vertex ids (a closed loop). Creates the
 * half-edges, the loop, and the face; pairs twins via an internal edge table
 * keyed on (min_vert,max_vert) so adjacent faces sharing an edge get linked
 * automatically. This is the standard robust half-edge assembly approach.
 *
 * `inner`/`inner_count` add hole loops (face-with-hole) for cut results.
 * Returns the FaceId (BREP_NONE on failure). */
FaceId brep_build_face(Brep *b, const VertId *ring, int n,
                       const VertId *const *inner_rings, const int *inner_n,
                       int inner_count, uint16_t feature_id);

/* Reset the internal edge-pairing table. Call once before building the faces
 * of a single solid so twin lookup is scoped to that solid. */
void brep_begin_solid(Brep *b);

/* Compute the exact integer plane (normal + d) of a face from its loop. */
void brep_face_plane(Brep *b, FaceId fid);

/* Euler characteristic check for a closed 2-manifold solid: V - E + F = 2.
 * Returns 1 if satisfied. Used as a correctness assertion after every op. */
int brep_check_euler(const Brep *b, SolidId solid);

/* Convenience accessors. */
Vertex   *brep_vert (Brep *b, VertId id);
HalfEdge *brep_he   (Brep *b, HEdgeId id);
Face     *brep_face (Brep *b, FaceId id);

#endif /* MINICAD_BREP_H */
