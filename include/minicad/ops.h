/* ops.h — sketch geometry + the modelling operations (extrude, revolve).
 *
 * A sketch is a closed profile of 2D entities on a plane. Operations consume
 * a sketch and produce a Solid in the B-rep. Everything integer.
 *
 * Reference geometry (datum plane/axis/point) lives in the feature layer
 * (model/feature.h); ops.h is just the geometry-producing kernel ops.
 */
#ifndef MINICAD_OPS_H
#define MINICAD_OPS_H

#include "minicad/brep.h"
#include "minicad/sketch.h"

/* A feature either ADDS material (boss) or REMOVES it (cut). */
typedef enum { OP_BOSS = 0, OP_CUT = 1 } OpType;

/* End condition — orthogonal to boss/cut, mirrors SolidWorks.
 *   BLIND          : fixed distance (boss + cut).
 *   THROUGH_ALL    : pass entirely through the solid (CUT ONLY).
 *   UP_TO_SURFACE  : terminate at a referenced planar face/datum (boss + cut).
 * For THROUGH_ALL and UP_TO_SURFACE the distance is COMPUTED, not user-given. */
typedef enum { END_BLIND = 0, END_THROUGH_ALL = 1, END_UP_TO_SURFACE = 2 } EndCond;

/* Resolved parameters handed to the geometry op once end-condition math is done.
 * `dist` is the final signed extrude length in myriometers (always concrete by
 * the time it reaches op_extrude/op_revolve). */
typedef struct {
    OpType  op;
    EndCond end;
    mym_t   dist;          /* concrete distance (computed for non-BLIND)    */
    int8_t  dir;           /* +1 along plane normal, -1 opposite             */
    FaceId  target;        /* UP_TO_SURFACE: the face we stop at (else NONE)  */
} OpParams;

/* A sketch plane: 3D point = origin + u*u_axis + v*v_axis (all integer). */
typedef struct {
    Vec3i origin;
    Vec3i u_axis, v_axis, normal;   /* exact integer directions */
} SketchPlane;

/* Default datum planes at document creation. */
SketchPlane plane_xy(void);
SketchPlane plane_xz(void);
SketchPlane plane_yz(void);

/* Map a 2D sketch point to 3D (exact integer). */
Vec3i sketch_to_3d(const SketchPlane *pl, Vec2i p);

/* ---- profile preparation ----
 * Any Sketch2 profile is reduced to an ordered ring of N boundary points in 3D
 * (rectangle -> 4, circle -> tessellated N) on the given plane. Extrude and
 * revolve both operate on this ring, which unifies boss/cut and the two ops.
 * Bridges sk_extract_profile() (2D, in sketch.c) into 3D via the plane.
 * Returns N (>=3) on success, 0 on failure. Fills `ring` (caller provides cap). */
int profile_to_ring(const Sketch2 *sk, const SketchPlane *pl, Vec3i *ring, int cap);

/* Tessellation segment count for a circle of the given myriometer radius
 * (low-poly LUT, no float/sqrt). */
int circle_segments(mym_t radius);

/* ---- end-condition resolution ----
 * Turn a user end-condition into a concrete signed distance.
 *  - BLIND: returns the given distance.
 *  - THROUGH_ALL: returns enough to clear the solid's extent along dir (+margin).
 *  - UP_TO_SURFACE: ray from the profile plane to params->target's plane.
 * Writes the concrete distance into params->dist; returns 1 on success. */
int resolve_end_condition(Brep *b, SolidId against, const SketchPlane *pl,
                          OpParams *params, mym_t blind_dist);

/* ---- the unified operation ----
 * EXTRUDE a profile into a prism along the plane normal.
 *   - OP_BOSS: produces a new standalone solid (the prism).
 *   - OP_CUT : carves the prism out of `target_solid` AS A FACE-WITH-HOLE +
 *              inner wall, for the supported case where the profile lies on a
 *              planar face of the target (cube+hole, bearing bore). This is the
 *              pragmatic cut (not general CSG); see notes in ops.c.
 * `target_solid` is ignored for OP_BOSS (pass BREP_NONE). */
SolidId op_extrude(Brep *b, const Sketch2 *sk, const SketchPlane *pl,
                   const OpParams *params,
                   SolidId target_solid, uint16_t feature_id);

/* REVOLVE a profile about `axis_origin` + t*`axis_dir`, `sweep` in sine-table
 * units (4096 = full turn), `steps` segments. Same stitching as extrude around
 * an axis. OP_BOSS makes a solid of revolution; OP_CUT carves a groove/bore
 * where the profile lies on a target face (same pragmatic-cut rule). */
SolidId op_revolve(Brep *b, const Sketch2 *sk, const SketchPlane *pl,
                   const OpParams *params,
                   Vec3i axis_origin, Vec3i axis_dir,
                   int32_t sweep, int steps,
                   SolidId target_solid, uint16_t feature_id);

#endif /* MINICAD_OPS_H */
