/* camera.h — PS1-ONLY. Orbit/zoom camera that produces the GTE transform.
 *
 * Per the hardware review (§1): the model->render SCALE is baked into the GTE
 * rotation matrix, so vertices are loaded straight from the model with no
 * software scale pass. This file owns the camera state and emits a MATRIX +
 * translation each frame; render.c just loads verts and issues GTE ops.
 *
 * All state is integer. Angles are sine-table units (4096 = 360 deg). Zoom is
 * an integer 1.12 scale folded into the matrix.
 */
#ifdef MINICAD_PSX
#ifndef MINICAD_CAMERA_H
#define MINICAD_CAMERA_H

#include <psxgte.h>
#include "minicad/fixed.h"
#include "minicad/ivec.h"

typedef struct {
    int32_t  yaw, pitch;     /* orbit angles, sine-table units      */
    int32_t  zoom;           /* 1.12 scale (FX_ONE = 1.0x)          */
    Vec3i    pivot;          /* orbit center (myriometers)          */
    VECTOR   trans;          /* GTE translation (computed)          */
    MATRIX   m;              /* GTE rotation*scale (computed)       */
} Camera;

void cam_init(Camera *c);

/* Recompute c->m and c->trans from yaw/pitch/zoom/pivot. Call once per frame
 * after input. The zoom scale is multiplied into the rotation matrix so the
 * GTE applies model->render scaling for free during RTPT. */
void cam_update(Camera *c);

/* Snap to one of the 6 standard orthographic views (L3 view picker). */
typedef enum { VIEW_FRONT, VIEW_BACK, VIEW_LEFT, VIEW_RIGHT, VIEW_TOP, VIEW_BOTTOM } StdView;
void cam_set_view(Camera *c, StdView v);

/* Zoom-to-fit given a model half-extent (myriometers): pick a zoom that frames it. */
void cam_zoom_to_fit(Camera *c, mym_t half_extent);

#endif /* MINICAD_CAMERA_H */
#endif /* MINICAD_PSX */
