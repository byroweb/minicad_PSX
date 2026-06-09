/* modeling.h — portable interactive-modeling core (host + PSX).
 *
 * This is the engine behind the on-console modeling loop: it lets the user
 * author a NEW feature (a boss/cut extrude) on a picked face, dial its
 * distance with live preview, and CONFIRM (committed to history) or CANCEL
 * (rolled back). It depends ONLY on Document / Brep / Sketch2 / History — NOT
 * on the PSX-only UiState — so it compiles into the host-tested kernel and the
 * PSX target alike. The PSX layer (input.c + main.c) maps controller intents
 * onto this API.
 *
 * MODEL: a "pending" edit is two appended features — a FEAT_SKETCH on the
 * picked face's plane (a centered circle profile) plus a FEAT_EXTRUDE that
 * depends on it. While pending, changing the distance re-regenerates the whole
 * B-rep so the preview tracks live. CONFIRM snapshots the document into history
 * and clears the pending state; CANCEL pops the two features back off and
 * regenerates the prior model. Integer / fixed-point math only.
 */
#ifndef MINICAD_MODELING_H
#define MINICAD_MODELING_H

#include "minicad/feature.h"
#include "minicad/brep.h"
#include "minicad/ops.h"
#include "minicad/history.h"

/* Default parameters for a freshly-started extrude (myriometers). */
#define MODEL_DEF_RADIUS   100   /* 10.0 mm profile circle radius */
#define MODEL_DEF_DIST     200   /* 20.0 mm default blind distance */
#define MODEL_MIN_DIST     10    /*  1.0 mm floor (never zero/neg)  */

/* The in-progress (pending) feature edit. Zero-initialise before first use, or
 * call model_init(). */
typedef struct {
    int      pending;       /* 1 while an edit is in progress            */
    uint16_t sketch_id;     /* the pending FEAT_SKETCH feature id        */
    uint16_t extrude_id;    /* the pending FEAT_EXTRUDE feature id        */
    OpType   op;            /* OP_BOSS / OP_CUT                           */
    mym_t    dist;          /* current blind distance (boss); cut is THROUGH_ALL */
} ModelingState;

void model_init(ModelingState *m);

/* Build an integer sketch plane from a B-rep face: origin = face centroid,
 * normal = face normal, with a derived integer u/v basis orthogonal to the
 * normal (right-handed: u x v = normal direction). */
SketchPlane plane_from_face(Brep *b, FaceId f);

/* Rebuild the ENTIRE B-rep from the document (resets the B-rep pools, marks
 * every feature dirty, regenerates). Returns doc_regen's result (1 = ok). Used
 * for every live preview so re-regen is clean (doc_regen otherwise accumulates
 * into the B-rep). */
int model_regen_all(Document *d, Brep *b);

/* Start a pending boss/cut extrude on `base`'s face plane: appends a Sketch
 * feature (centered circle, MODEL_DEF_RADIUS) + an Extrude feature (boss=BLIND
 * MODEL_DEF_DIST, cut=THROUGH_ALL) and regenerates for a live preview. No-op if
 * an edit is already pending or `base` is invalid. Returns 1 on success. */
int model_begin_extrude(ModelingState *m, Document *d, Brep *b,
                        FaceId base, OpType op);

/* Update the pending extrude's distance (clamped to >= MODEL_MIN_DIST) and
 * regenerate the live preview. No-op if nothing is pending. Returns 1 if the
 * preview regenerated. */
int model_set_distance(ModelingState *m, Document *d, Brep *b, mym_t dist);

/* Adjust the pending distance by `delta` (convenience for d-pad stepping). */
int model_nudge_distance(ModelingState *m, Document *d, Brep *b, mym_t delta);

/* Finalise the pending edit: ensure a clean regen, push a history snapshot,
 * clear the pending flag. Returns 1 on success, 0 if nothing pending / regen
 * failed (in which case the edit stays pending). */
int model_confirm(ModelingState *m, Document *d, Brep *b, History *h);

/* Abort the pending edit: remove the two appended features and regenerate the
 * prior model. No-op (returns 0) if nothing is pending. */
int model_cancel(ModelingState *m, Document *d, Brep *b);

#endif /* MINICAD_MODELING_H */
