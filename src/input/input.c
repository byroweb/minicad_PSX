/* input.c — PS1-ONLY. DualShock polling + the CAD interaction model.
 *
 * Control map (DESIGN §7):
 *   L stick : 3D cursor (snaps to nearest entity of active filter)
 *   R stick : orbit; (+R2) spins the contextual wheel
 *   L1/R1   : scroll selection FILTER
 *   L2/R2   : zoom out / in ; (+L1/R1) pan
 *   L3      : zoom-to-fit + 6-view picker ; R3 : recenter pivot on selection
 *   Cross   : select under cursor   Circle: deselect (hold = clear all)
 *   Triangle: execute wheel action  Square: step candidate (disambiguate)
 *   D-pad U/D : increment/decrement active value
 *   D-pad L/R : collapse/expand tree node (or value magnitude x10 / /10)
 *   Start   : system menu (save/load/new)
 *   Select  : undo modifier — Select+Triangle = undo, Select+Circle = redo
 *
 * input_poll() reads the pad into a UiState; input_apply() turns that into
 * camera motion + edit intents (undo/redo/view). Needs your emulator check.
 */
#ifdef MINICAD_PSX

#include <psxpad.h>
#include <psxapi.h>
#include <stdint.h>
#include "minicad/ui_state.h"
#include "minicad/camera.h"
#include "minicad/feature.h"
#include "minicad/history.h"

#define DZ 24   /* analog dead-zone */
#define SCREEN_W 320
#define SCREEN_H 240

static UiState  g_ui;
static uint8_t  g_padbuf[2][34];
static uint16_t g_prev = 0xFFFF;
static int      g_primed = 0;       /* seen one valid pad frame yet? */
static History  g_hist;
static int      g_hist_ready = 0;

void input_init(void) {
    InitPAD(g_padbuf[0], 34, g_padbuf[1], 34);
    StartPAD();
    ChangeClearPAD(0);
    g_ui.filter = FILT_FACE;
    g_ui.value_step = 10;            /* 1.0 mm */
    g_ui.selected_feat = 0;
    g_ui.cursor_x = SCREEN_W / 2;
    g_ui.cursor_y = SCREEN_H / 2;
    g_ui.hover_kind = KIND_NONE;
    g_ui.sel_kind   = KIND_NONE;
}

static int axis(uint8_t raw) {
    int v = (int)raw - 0x80;
    if (v > -DZ && v < DZ) return 0;
    return v;
}

void input_poll(UiState **out) {
    PADTYPE *pad = (PADTYPE *)g_padbuf[0];
    int connected = (pad->stat == 0);
    uint16_t btn = connected ? pad->btn : 0xFFFF;          /* active-low */
    /* On the first valid frame the pad buffer can read as all-pressed before
     * the pad IRQ fills it; priming g_prev from that frame avoids a phantom
     * burst of "presses" (which used to fire zoom-to-fit + view-snap at boot). */
    if (connected && !g_primed) { g_prev = btn; g_primed = 1; }
    uint16_t pressed = (uint16_t)(~btn & g_prev);          /* newly down */

    /* Analog sticks only exist on an analog pad (DualShock 0x7 / stick 0x5).
     * On a digital pad or with no controller the stick bytes are garbage, and
     * reading them unconditionally made the camera orbit on its own. */
    int has_sticks = connected &&
        (pad->type == PAD_ID_ANALOG || pad->type == PAD_ID_ANALOG_STICK);
    int lx = 0, ly = 0, rx = 0, ry = 0;
    if (has_sticks) {
        lx = axis(pad->ls_x); ly = axis(pad->ls_y);
        rx = axis(pad->rs_x); ry = axis(pad->rs_y);
    }
    int sel_held = !(~btn & PAD_SELECT) ? 0 : 1;           /* Select held? */
    /* active-low: bit clear = pressed. helper: */
    #define DOWN(b)    (!(btn & (b)))
    #define PRESS(b)   (pressed & (b))

    g_ui.moving = 0;
    g_ui.d_yaw = g_ui.d_pitch = g_ui.d_zoom = 0;
    g_ui.d_pan_x = g_ui.d_pan_y = 0;
    g_ui.want_undo = g_ui.want_redo = 0;
    g_ui.want_zoom_fit = 0;
    g_ui.want_view = 0;

    /* orbit (R stick), unless R2 held (wheel spin -> UI layer) */
    if (!DOWN(PAD_R2) && (rx || ry)) {
        g_ui.d_yaw   = rx >> 3;
        g_ui.d_pitch = -(ry >> 3);
        g_ui.moving  = 1;
    }
    /* cursor (L stick), clamped to the 320x240 viewport */
    if (lx || ly) { g_ui.cursor_x += (int16_t)(lx >> 5); g_ui.cursor_y += (int16_t)(ly >> 5); }
    if (g_ui.cursor_x < 0) g_ui.cursor_x = 0;
    if (g_ui.cursor_x > SCREEN_W - 1) g_ui.cursor_x = SCREEN_W - 1;
    if (g_ui.cursor_y < 0) g_ui.cursor_y = 0;
    if (g_ui.cursor_y > SCREEN_H - 1) g_ui.cursor_y = SCREEN_H - 1;

    /* filter scroll: L1 / R1 (unless used as pan modifier with L2/R2) */
    if (PRESS(PAD_R1) && !DOWN(PAD_R2)) g_ui.filter = (SelFilter)((g_ui.filter+1)%FILT_COUNT);
    if (PRESS(PAD_L1) && !DOWN(PAD_L2)) g_ui.filter = (SelFilter)((g_ui.filter+FILT_COUNT-1)%FILT_COUNT);

    /* zoom: L2 out / R2 in ; pan when combined with L1/R1 */
    if (DOWN(PAD_R2)) { if (DOWN(PAD_R1)) g_ui.d_pan_x += 4; else { g_ui.d_zoom += 6; g_ui.moving=1; } }
    if (DOWN(PAD_L2)) { if (DOWN(PAD_L1)) g_ui.d_pan_x -= 4; else { g_ui.d_zoom -= 6; g_ui.moving=1; } }

    /* selection verbs. The picker (render.c) fills hover_id/hover_kind each
     * frame from the cursor; Cross commits that to sel_id/sel_kind. We also
     * keep selected_feat in sync (face-feature highlight) for compatibility. */
    if (PRESS(PAD_CROSS)) {
        if (g_ui.hover_kind != KIND_NONE) {
            g_ui.sel_id   = g_ui.hover_id;
            g_ui.sel_kind = g_ui.hover_kind;
            g_ui.selected_feat = g_ui.hover_feat;
        }
    }
    if (PRESS(PAD_CIRCLE) && !sel_held) {   /* deselect */
        g_ui.sel_kind = KIND_NONE;
        g_ui.sel_id   = 0;
        g_ui.selected_feat = 0;
    }

    /* undo/redo: Select + Triangle / Select + Circle */
    if (sel_held && PRESS(PAD_TRIANGLE)) g_ui.want_undo = 1;
    else if (PRESS(PAD_TRIANGLE))        { /* execute wheel action */ }
    if (sel_held && PRESS(PAD_CIRCLE))   g_ui.want_redo = 1;

    /* d-pad value editing */
    if (PRESS(PAD_UP))    g_ui.active_value += g_ui.value_step;
    if (PRESS(PAD_DOWN))  g_ui.active_value -= g_ui.value_step;
    if (PRESS(PAD_RIGHT)) g_ui.value_step  *= 10;
    if (PRESS(PAD_LEFT))  g_ui.value_step   = (g_ui.value_step > 1) ? g_ui.value_step/10 : 1;

    /* L3 zoom-to-fit + view picker; R3 recenter pivot */
    if (PRESS(PAD_L3)) g_ui.want_zoom_fit = 1;
    /* d-pad while L3 latched could pick a view; simplified: Start cycles views */
    if (PRESS(PAD_START)) g_ui.want_view = (int8_t)(((g_ui.want_view) % 6) + 1);

    #undef DOWN
    #undef PRESS
    g_prev = btn;
    *out = &g_ui;
}

/* Apply UI intent to the camera + document. Owns the history binding so undo/
 * redo work end to end. `doc` is re-regenerated by the caller when edits land. */
void input_apply(UiState *ui, Camera *cam, Document *doc) {
    if (!g_hist_ready) { hist_init(&g_hist, doc); g_hist_ready = 1; }

    cam->yaw   += ui->d_yaw;
    cam->pitch += ui->d_pitch;
    cam->zoom  += ui->d_zoom;
    /* Clamp into a range that stays valid once folded into the 1.12 GTE matrix:
     * matrix entry = fx_mul(rot<=FX_ONE, zoom), so zoom*FX_ONE must fit int16.
     * FX_ONE*8 overflowed (8*4096 = 32768 -> -32768); cap at FX_ONE*4. */
    if (cam->zoom < FX_ONE/8) cam->zoom = FX_ONE/8;
    if (cam->zoom > FX_ONE*4) cam->zoom = FX_ONE*4;
    cam->pivot.x -= ui->d_pan_x;
    cam->pivot.y -= ui->d_pan_y;

    if (ui->want_zoom_fit) cam_zoom_to_fit(cam, 600);   /* demo half-extent */
    if (ui->want_view)     cam_set_view(cam, (StdView)(ui->want_view - 1));

    if (ui->want_undo) hist_undo(&g_hist, doc);
    if (ui->want_redo) hist_redo(&g_hist, doc);
    /* a real edit would call hist_commit(&g_hist, doc) after applying it */
}

int      input_moving(const UiState *ui)   { return ui->moving; }
uint16_t input_selected(const UiState *ui) { return ui->selected_feat; }

#endif /* MINICAD_PSX */
