/* camera.c — PS1-ONLY. Orbit camera -> GTE matrix with baked scale. */
#ifdef MINICAD_PSX

#include "minicad/camera.h"
#include <psxgte.h>
#include <inline_c.h>

void cam_init(Camera *c) {
    c->yaw = 360; c->pitch = 480;          /* a pleasant iso-ish start */
    /* Baked model->render downscale (HARDWARE_REVIEW §1): raw ±300 myriometer
     * coords project too large for the OT depth range, so shrink them into GTE
     * render space here rather than with a software pass. 0.5x frames the demo. */
    c->zoom = FX_ONE/5;                     /* model->render scale (tuning) */
    c->pivot.x = 0; c->pivot.y = 0; c->pivot.z = 300;  /* cube centre (z 0..600) */
}

/* Build a rotation matrix from yaw (about Y) then pitch (about X), each from
 * the integer sine table, then scale every entry by zoom (1.12 * 1.12 -> 1.12).
 * The result is a single MATRIX the GTE uses for the whole transform. */
void cam_update(Camera *c) {
    int32_t sy = fx_isin(c->yaw),  cy = fx_icos(c->yaw);
    int32_t sp = fx_isin(c->pitch),cp = fx_icos(c->pitch);

    /* Ry (yaw) */
    /* | cy  0  sy |
       |  0  1   0 |
       |-sy  0  cy | */
    /* Rx (pitch) */
    /* | 1   0    0 |
       | 0  cp  -sp |
       | 0  sp   cp | */
    /* M = Rx * Ry, entries in 1.12. Compose by hand to avoid an extra matrix. */
    int32_t r00 = cy;
    int32_t r01 = 0;
    int32_t r02 = sy;
    int32_t r10 = fx_mul(sp, sy);
    int32_t r11 = cp;
    int32_t r12 = -fx_mul(sp, cy);
    int32_t r20 = -fx_mul(cp, sy);
    int32_t r21 = sp;
    int32_t r22 = fx_mul(cp, cy);

    /* fold zoom scale into every entry (still 1.12) */
    #define Z(x) ((short)fx_mul((x), c->zoom))
    c->m.m[0][0]=Z(r00); c->m.m[0][1]=Z(r01); c->m.m[0][2]=Z(r02);
    c->m.m[1][0]=Z(r10); c->m.m[1][1]=Z(r11); c->m.m[1][2]=Z(r12);
    c->m.m[2][0]=Z(r20); c->m.m[2][1]=Z(r21); c->m.m[2][2]=Z(r22);
    #undef Z

    /* translation: push the model away from the eye along +Z so perspective
     * has positive SZ. Pivot offset is applied here so orbit centers on it. */
    c->trans.vx = -c->pivot.x;
    c->trans.vy = -c->pivot.y;
    c->trans.vz = -c->pivot.z + 1800;   /* eye distance (render units) */
}

void cam_set_view(Camera *c, StdView v) {
    switch (v) {
    case VIEW_FRONT:  c->yaw = 0;            c->pitch = 0;            break;
    case VIEW_BACK:   c->yaw = SIN_LEN/2;    c->pitch = 0;            break;
    case VIEW_LEFT:   c->yaw = SIN_LEN*3/4;  c->pitch = 0;            break;
    case VIEW_RIGHT:  c->yaw = SIN_LEN/4;    c->pitch = 0;            break;
    /* TOP looks straight down (+90 pitch), BOTTOM straight up (-90). BOTTOM uses
     * -SIN_LEN/4 rather than the equivalent +3*SIN_LEN/4 so it stays inside the
     * orbit pitch clamp [-90,+90] and the per-frame clamp won't pull it back. */
    case VIEW_TOP:    c->yaw = 0;            c->pitch = SIN_LEN/4;    break;
    case VIEW_BOTTOM: c->yaw = 0;            c->pitch = -SIN_LEN/4;   break;
    }
}

void cam_zoom_to_fit(Camera *c, mym_t half_extent) {
    /* Pick a baked model->render zoom that frames the given half-extent.
     * Calibrated empirically against the eye distance / GTE projection: a
     * half-extent of ~600 mym frames nicely at zoom ~ FX_ONE/5, i.e.
     * zoom ~ (120 * FX_ONE) / half_extent. Clamp to the matrix-safe range. */
    if (half_extent < 1) half_extent = 1;
    int32_t z = (int32_t)(((mym2_t)120 * FX_ONE) / ((mym2_t)half_extent));
    if (z < FX_ONE/8)  z = FX_ONE/8;
    if (z > FX_ONE*4)  z = FX_ONE*4;
    c->zoom = z;
}

/* Pan the pivot in screen-aligned X/Y. The pivot lives in model space, so we
 * rotate the screen-space delta by the current yaw (about Y) to keep the motion
 * locked to the screen axes regardless of orbit. Vertical screen motion maps to
 * the model's Y axis (pitch tilt is small enough that Y dominates; mapping to
 * pure Y keeps pan predictable and avoids gimbal weirdness near the poles).
 * Scale inversely with zoom: at high zoom the part fills the screen, so a given
 * pixel travel must move the pivot less. zoom is 1.12, so divide by it. */
void cam_pan_screen(Camera *c, int32_t dsx, int32_t dsy) {
    int32_t sy = fx_isin(c->yaw), cy = fx_icos(c->yaw);
    /* Inverse-zoom scale, in 1.12: bigger zoom -> smaller world step.
     * Reference zoom FX_ONE/5 gives factor ~5. Clamp factor to keep it sane. */
    int32_t zf = (c->zoom > 0) ? (FX_ONE * FX_ONE) / c->zoom : FX_ONE;
    int32_t wx = fx_mul(dsx, zf);   /* screen-right step in world units (1.12 folded) */
    int32_t wy = fx_mul(dsy, zf);
    /* Screen-right axis in model space = (cy, 0, -sy) for Ry yaw. Screen-down
     * maps to model -Y (screen Y grows downward; pivot subtract handled by sign).*/
    c->pivot.x += fx_mul(wx, cy);
    c->pivot.z += fx_mul(wx, -sy);
    c->pivot.y += wy;
}

void cam_recenter(Camera *c) {
    /* Recenter on the demo part center: the 60mm cube spans ±300 mym in X/Y and
     * 0..600 in Z, so its centroid is (0,0,300) — the same point cam_init uses. */
    c->pivot.x = 0;
    c->pivot.y = 0;
    c->pivot.z = 300;
}

#endif /* MINICAD_PSX */
