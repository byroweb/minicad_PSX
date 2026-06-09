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

typedef struct UiState {
    int16_t  cursor_x, cursor_y;
    int32_t  d_yaw, d_pitch;     /* orbit deltas this frame      */
    int32_t  d_zoom;             /* zoom delta this frame        */
    int32_t  d_pan_x, d_pan_y;   /* pan deltas this frame        */
    SelFilter filter;
    int       moving;            /* camera in motion -> wireframe */
    uint16_t  hover_feat;        /* feature under cursor          */
    uint16_t  selected_feat;     /* current selection             */
    int32_t   active_value;      /* value d-pad edits             */
    int32_t   value_step;
    int8_t    want_undo, want_redo;
    int8_t    want_zoom_fit;     /* L3 pressed                    */
    int8_t    want_view;         /* StdView+1 chosen, 0 = none    */
} UiState;

#endif
#endif
