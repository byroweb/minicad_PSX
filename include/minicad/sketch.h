/* sketch.h — the parametric sketcher.
 *
 * DESIGN (hybrid per the plan):
 *   - Geometry is split into shared POINTS (the degrees of freedom) and
 *     ENTITIES that reference points by id. Two lines sharing a corner share
 *     ONE point, so moving it moves both — coincidence is structural, free.
 *   - CONSTRAINTS are stored as data now and resolved by DIRECT INTEGER RULES
 *     (no iteration): horizontal/vertical/equal/coincident/fix. Constraints
 *     that truly need iteration (dimension, tangent) are recorded but flagged
 *     UNSOLVED until a fixed-point Newton solver slots in later. The data model
 *     is the solver's eventual input, so nothing here has to change then.
 *   - CONSTRUCTION geometry is a per-entity flag: real for referencing and
 *     constraining, excluded from profile extraction.
 *
 * All integer myriometers. This header supersedes the old Sketch in ops.h;
 * ops.h now includes this and profile extraction walks real entities.
 */
#ifndef MINICAD_SKETCH_H
#define MINICAD_SKETCH_H

#include "minicad/fixed.h"
#include "minicad/ivec.h"

typedef uint8_t SkPointId;     /* index into Sketch.pt[]  */
typedef uint8_t SkEntId;       /* index into Sketch.ent[] */
#define SK_NONE 0xFF

#define SK_MAX_POINTS      64
#define SK_MAX_ENTITIES    48
#define SK_MAX_CONSTRAINTS 48

/* ---- points: the degrees of freedom ---- */
typedef struct {
    mym_t u, v;        /* plane coordinates (myriometers) */
    uint8_t fixed;     /* 1 = immovable (anchored)        */
    uint8_t alive;
} SkPoint;

/* ---- entities reference points by id ---- */
typedef enum { SKE_LINE = 0, SKE_CIRCLE = 1, SKE_ARC = 2, SKE_RECT = 3 } SkEntKind;

typedef struct {
    SkEntKind kind;
    SkPointId p0, p1;      /* line: endpoints. rect: opposite corners.       */
    SkPointId center;      /* circle/arc center                               */
    mym_t     radius;      /* circle/arc (derived/anchored)                   */
    int32_t   ang0, ang1;  /* arc sweep (sine-table units)                    */
    uint8_t   construction;/* 1 = construction geometry (excluded from profile)*/
    uint8_t   alive;
} SkEntity;

/* ---- constraints (stored now; direct-rule subset resolved now) ---- */
typedef enum {
    SKC_COINCIDENT = 0,  /* two points coincide                     (direct) */
    SKC_HORIZONTAL,      /* a line is horizontal                    (direct) */
    SKC_VERTICAL,        /* a line is vertical                      (direct) */
    SKC_EQUAL,           /* two lines equal length / two circles eq (direct) */
    SKC_FIX,             /* anchor a point                          (direct) */
    SKC_PARALLEL,        /* two lines parallel              (needs solver*)   */
    SKC_PERPENDICULAR,   /* two lines perpendicular         (needs solver*)   */
    SKC_TANGENT,         /* line/arc tangent                (needs solver)    */
    SKC_DIMENSION,       /* a measured distance/length      (needs solver)    */
    SKC_KIND_COUNT
} SkConstraintKind;

typedef struct {
    SkConstraintKind kind;
    SkEntId   e0, e1;      /* entity refs (SK_NONE if N/A)        */
    SkPointId a, b;        /* point refs (SK_NONE if N/A)         */
    mym_t     value;       /* dimension value / equal length      */
    uint8_t   solved;      /* 1 once satisfied (direct rules)     */
    uint8_t   alive;
} SkConstraint;

typedef struct {
    /* plane is carried by the feature; see SketchPlane in ops.h */
    SkPoint      pt[SK_MAX_POINTS];      uint8_t pt_count;
    SkEntity     ent[SK_MAX_ENTITIES];   uint8_t ent_count;
    SkConstraint con[SK_MAX_CONSTRAINTS];uint8_t con_count;
} Sketch2;

/* ---- construction / editing API ---- */
void      sk_init(Sketch2 *s);
SkPointId sk_add_point(Sketch2 *s, mym_t u, mym_t v);
SkEntId   sk_add_line(Sketch2 *s, SkPointId a, SkPointId b);
SkEntId   sk_add_rect(Sketch2 *s, mym_t u0, mym_t v0, mym_t u1, mym_t v1);
SkEntId   sk_add_circle(Sketch2 *s, mym_t cu, mym_t cv, mym_t radius);
void      sk_set_construction(Sketch2 *s, SkEntId e, int on);

/* ---- trim ----
 * Mode TRIM_DELETE removes the entity (or the segment between the two nearest
 * intersections with other entities, when those exist).
 * Mode TRIM_TO_CONSTRUCTION flips the entity to construction instead. */
typedef enum { TRIM_DELETE = 0, TRIM_TO_CONSTRUCTION = 1 } TrimMode;
void sk_trim(Sketch2 *s, SkEntId e, TrimMode mode);

/* Integer intersection helpers (used by trim + future snapping). Return 1 and
 * write the point if they intersect within the segments. */
int sk_line_line(const Sketch2 *s, SkEntId a, SkEntId b, Vec2i *out);
int sk_line_circle(const Sketch2 *s, SkEntId line, SkEntId circle,
                   Vec2i *out0, Vec2i *out1, int *n);

/* ---- constraints ---- */
SkConstraint *sk_add_constraint(Sketch2 *s, SkConstraintKind kind,
                                SkEntId e0, SkEntId e1, SkPointId a, SkPointId b,
                                mym_t value);
/* Resolve all DIRECT-rule constraints in a single integer pass. Returns the
 * number left UNSOLVED (i.e. needing the iterative solver below). */
int sk_resolve_direct(Sketch2 *s);

/* Iterative fixed-point constraint solver (integer / fixed-point, NO floats).
 * First applies sk_resolve_direct(), then drives the iterative constraint kinds
 * — SKC_DIMENSION, SKC_PARALLEL, SKC_PERPENDICULAR, SKC_TANGENT — toward
 * satisfaction with a damped Gauss-Seidel sweep that moves only non-fixed
 * points. Caps iterations; deterministic. Returns the number of constraints
 * still unsatisfied (0 = fully solved). */
int sk_solve(Sketch2 *s);

/* ---- profile extraction ----
 * Collect non-construction entities into an ordered boundary ring for the 3D
 * ops. Returns point count, 0 if no closed profile. */
int sk_extract_profile(const Sketch2 *s, Vec2i *ring, int cap);

#endif /* MINICAD_SKETCH_H */
