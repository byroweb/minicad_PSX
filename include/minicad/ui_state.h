/* ui_state.h — PS1-ONLY. Shared UI/input state, plus the camera binding glue
 * so main.c and input.c agree on the struct layout. */
#ifdef MINICAD_PSX
#ifndef MINICAD_UI_STATE_H
#define MINICAD_UI_STATE_H

#include "minicad/fixed.h"
#include "minicad/camera.h"
#include "minicad/feature.h"

typedef enum {
    FILT_VERTEX = 0, FILT_EDGE, FILT_FACE, FILT_PROFILE, FILT_DATUM, FILT_LOOP,
    FILT_COUNT
} SelFilter;

/* What KIND of entity a picked id refers to. KIND_NONE = nothing picked. */
typedef enum {
    KIND_NONE = 0, KIND_VERTEX, KIND_EDGE, KIND_FACE
} SelKind;

typedef struct UiState {
    int16_t  cursor_x, cursor_y;
    int32_t  d_yaw, d_pitch;     /* orbit deltas this frame      */
    int32_t  d_zoom;             /* zoom delta this frame        */
    int32_t  d_pan_x, d_pan_y;   /* pan deltas this frame        */
    SelFilter filter;
    int       moving;            /* camera in motion -> wireframe */
    uint16_t  hover_feat;        /* feature under cursor          */
    uint16_t  selected_feat;     /* current selection             */
    /* Generic cursor-picking state. The picker (render.c) writes hover_* each
     * frame; input.c commits hover_* -> sel_* on Cross and clears on Circle.
     * The id is a 16-bit PoolId whose meaning depends on the matching kind:
     *   KIND_VERTEX -> VertId, KIND_EDGE -> HEdgeId, KIND_FACE -> FaceId. */
    uint16_t  hover_id;          /* entity under cursor (by kind)  */
    SelKind   hover_kind;
    uint16_t  sel_id;            /* committed picked entity        */
    SelKind   sel_kind;
    int32_t   active_value;      /* value d-pad edits             */
    int32_t   value_step;
    int8_t    want_undo, want_redo;
    /* Interactive-modeling intents (consumed in main.c, driven by input.c).
     * want_new_boss/cut: start an extrude on the selected face (Triangle/Square).
     * want_confirm/cancel: finalise/abort the pending edit (Cross/Circle while
     * pending). want_dist_delta: live distance nudge from d-pad Up/Down. */
    int8_t    want_new_boss, want_new_cut;
    int8_t    want_confirm,  want_cancel;
    int32_t   want_dist_delta;
    int8_t    modeling_pending;  /* main.c writes 1 while an edit is in progress;
                                  * input.c reads it to route Cross/Circle to
                                  * confirm/cancel instead of select/deselect. */
    /* System / file intents (consumed in main.c). Chosen from the Start menu
     * (see menu_open/menu_index): Save / Load / New. */
    int8_t    want_save, want_load, want_new;
    /* Start opens a modal system menu; while open the d-pad moves the highlight
     * and Cross picks Save(0)/Load(1)/New(2), Circle/Start closes. render.c
     * draws it; input.c is modal while menu_open. */
    int8_t    menu_open;
    int8_t    menu_index;
    int8_t    want_zoom_fit;     /* L3 pressed                    */
    int8_t    want_view;         /* StdView+1 chosen, 0 = none    */
    int8_t    want_recenter;     /* R3 pressed -> recenter pivot   */
    /* Damped-manipulation velocities (owned by input_apply). These accumulate
     * the per-frame d_* intents and decay each frame so motion eases in/out
     * instead of snapping. Sub-unit precision (shifted) so slow drift survives
     * the decay; the camera consumes the high bits. Tuned for 60fps. */
    int32_t   v_yaw, v_pitch;    /* orbit velocity (angle units << VEL_SHIFT) */
    int32_t   v_zoom;            /* zoom velocity (1.12 << VEL_SHIFT)         */
} UiState;

#endif
#endif
