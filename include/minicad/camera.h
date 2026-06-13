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

/* Display width (must match SCREEN_W in render.c/input.c — this is the one knob).
 * 320x240 is square on a 4:3 NTSC screen; any wider mode at 240 lines has
 * narrower (taller) pixels, so camera.c scales the projected Y by 320/W to keep
 * geometry at a 1:1 on-screen aspect. 368 = a gentle bump (text ~13% smaller,
 * pixels ~0.87:1, barely condensed). */
#define MINICAD_SCREEN_W   368
#define MINICAD_YASPECT_FX ((320 << 12) / MINICAD_SCREEN_W)   /* 1.12; ==FX_ONE at 320 */

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

/* Pan the pivot in the camera's screen-aligned X/Y plane. dsx/dsy are screen-
 * space deltas (myriometers): +dsx moves the part right, +dsy moves it down.
 * The motion is rotated by the current yaw so panning always tracks the screen
 * axes, and scaled inversely with zoom so it feels uniform at any zoom level. */
void cam_pan_screen(Camera *c, int32_t dsx, int32_t dsy);

/* Recenter the orbit pivot on the part center (demo cube center). True
 * selection-pivot (centroid of the picked entity) needs renderer cooperation
 * and is a documented follow-up. */
void cam_recenter(Camera *c);

#endif /* MINICAD_CAMERA_H */
#endif /* MINICAD_PSX */
