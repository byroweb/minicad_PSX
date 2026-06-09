/* input.c — PS1-ONLY. DualShock polling + the CAD interaction model.
 *
 * Control map (DESIGN §7):
 *   L stick : 3D cursor (snaps to nearest entity of active filter)
 *   R stick : orbit (analog pad)
 *   D-pad   : orbit (digital-pad fallback); while editing: value / distance
 *   L1/R1   : scroll selection FILTER
 *   L2/R2   : zoom out / in ; (+L1/R1) pan
 *   L3      : zoom-to-fit ; R3 : recenter pivot
 *   Cross   : select under cursor (or confirm edit / menu pick)
 *   Circle  : deselect (or cancel edit / close menu)
 *   Triangle/Square : start boss / cut extrude on the selected face
 *   Start   : open system menu (Save / Load / New) ; Select+Start : cycle views
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
    g_ui.want_new_boss = g_ui.want_new_cut = 0;
    g_ui.want_confirm = g_ui.want_cancel = 0;
    g_ui.want_dist_delta = 0;
    g_ui.want_zoom_fit = 0;
    g_ui.want_view = 0;
    g_ui.want_recenter = 0;
    g_ui.want_save = g_ui.want_load = g_ui.want_new = 0;

    /* System menu (Start). MODAL: while open, the d-pad moves the highlight and
     * Cross picks Save(0)/Load(1)/New(2); Circle or Start closes. Nothing else
     * moves while it's open. Plain Start opens it; Select+Start still cycles the
     * standard views (handled below). */
    if (g_ui.menu_open) {
        if (PRESS(PAD_UP))   g_ui.menu_index = (int8_t)((g_ui.menu_index + 2) % 3);
        if (PRESS(PAD_DOWN)) g_ui.menu_index = (int8_t)((g_ui.menu_index + 1) % 3);
        if (PRESS(PAD_CROSS)) {
            if      (g_ui.menu_index == 0) g_ui.want_save = 1;
            else if (g_ui.menu_index == 1) g_ui.want_load = 1;
            else                           g_ui.want_new  = 1;
            g_ui.menu_open = 0;
        }
        if (PRESS(PAD_CIRCLE) || PRESS(PAD_START)) g_ui.menu_open = 0;
        g_prev = btn; *out = &g_ui; return;
    }
    if (!sel_held && PRESS(PAD_START)) {           /* open the system menu */
        g_ui.menu_open = 1; g_ui.menu_index = 0;
        g_prev = btn; *out = &g_ui; return;
    }

    /* orbit (R stick), unless R2 held (wheel spin -> UI layer). These feed the
     * damped velocity integrator in input_apply (see VEL_* there), so we only
     * report the raw impulse here; the easing/decay happens on apply. */
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

    /* filter scroll: L1 / R1 (unless used as pan modifier with L2/R2, or as the
     * Select+L1/R1 file combos below). */
    if (!sel_held && PRESS(PAD_R1) && !DOWN(PAD_R2)) g_ui.filter = (SelFilter)((g_ui.filter+1)%FILT_COUNT);
    if (!sel_held && PRESS(PAD_L1) && !DOWN(PAD_L2)) g_ui.filter = (SelFilter)((g_ui.filter+FILT_COUNT-1)%FILT_COUNT);

    /* Select+Start cycles the 6 standard views (plain Start opens the system
     * menu instead; save/load/new live there now). */
    if (sel_held && PRESS(PAD_START)) g_ui.want_view = (int8_t)(((g_ui.want_view) % 6) + 1);

    /* zoom: L2 out / R2 in ; pan when combined with L1/R1.
     *   R1+R2 -> pan right + up ; L1+L2 -> pan left + down.
     * Splitting the two diagonals across the two shoulder combos lets a digital
     * pad reach all four screen directions; an analog pad still pans via these
     * combos. Pan goes through cam_pan_screen (screen-aligned, zoom-scaled).
     * Plain L2/R2 (no shoulder modifier) feed the damped zoom integrator. */
    if (DOWN(PAD_R2)) {
        if (DOWN(PAD_R1)) { g_ui.d_pan_x += 4; g_ui.d_pan_y -= 4; g_ui.moving=1; }
        else              { g_ui.d_zoom += 6; g_ui.moving=1; }
    }
    if (DOWN(PAD_L2)) {
        if (DOWN(PAD_L1)) { g_ui.d_pan_x -= 4; g_ui.d_pan_y += 4; g_ui.moving=1; }
        else              { g_ui.d_zoom -= 6; g_ui.moving=1; }
    }

    /* --- Interactive modeling verbs -------------------------------------- *
     * The face-press flow (see modeling.c). With NO edit pending, Cross/Circle
     * keep their picking meaning (select/deselect) and Triangle/Square START a
     * new boss/cut extrude on the selected FACE. While an edit IS pending, Cross
     * CONFIRMS and Circle CANCELS instead — main.c drives the modeling core off
     * these intents. main.c writes g_ui.modeling_pending each frame. */
    int pending = g_ui.modeling_pending;

    /* selection verbs (only when NOT pending). The picker (render.c) fills
     * hover_id/hover_kind each frame from the cursor; Cross commits that to
     * sel_id/sel_kind. selected_feat stays in sync (face-feature highlight). */
    if (!pending && PRESS(PAD_CROSS)) {
        if (g_ui.hover_kind != KIND_NONE) {
            g_ui.sel_id   = g_ui.hover_id;
            g_ui.sel_kind = g_ui.hover_kind;
            g_ui.selected_feat = g_ui.hover_feat;
        }
    }
    if (!pending && PRESS(PAD_CIRCLE) && !sel_held) {   /* deselect */
        g_ui.sel_kind = KIND_NONE;
        g_ui.sel_id   = 0;
        g_ui.selected_feat = 0;
    }

    /* start a new feature on the selected face (only when NOT pending and a
     * face is selected): Triangle = boss, Square = cut. */
    if (!pending && !sel_held && g_ui.sel_kind == KIND_FACE) {
        if (PRESS(PAD_TRIANGLE)) g_ui.want_new_boss = 1;
        if (PRESS(PAD_SQUARE))   g_ui.want_new_cut  = 1;
    }

    /* confirm / cancel the pending edit */
    if (pending && !sel_held) {
        if (PRESS(PAD_CROSS))  g_ui.want_confirm = 1;
        if (PRESS(PAD_CIRCLE)) g_ui.want_cancel  = 1;
    }

    /* undo/redo: Select + Triangle / Select + Circle */
    if (sel_held && PRESS(PAD_TRIANGLE)) g_ui.want_undo = 1;
    if (sel_held && PRESS(PAD_CIRCLE))   g_ui.want_redo = 1;

    /* d-pad: ORBIT when idle, value/distance edit when a feature edit is pending.
     * The idle orbit is the digital-pad fallback so the model can be inspected
     * without an analog stick (the right stick still orbits when the pad is in
     * analog mode). Held d-pad feeds the same damped integrator as the stick. */
    if (pending) {
        if (PRESS(PAD_UP))    { g_ui.active_value += g_ui.value_step; g_ui.want_dist_delta += g_ui.value_step; }
        if (PRESS(PAD_DOWN))  { g_ui.active_value -= g_ui.value_step; g_ui.want_dist_delta -= g_ui.value_step; }
        if (PRESS(PAD_RIGHT)) g_ui.value_step  *= 10;
        if (PRESS(PAD_LEFT))  g_ui.value_step   = (g_ui.value_step > 1) ? g_ui.value_step/10 : 1;
    } else {
        if (DOWN(PAD_UP))    { g_ui.d_pitch += 6; g_ui.moving = 1; }
        if (DOWN(PAD_DOWN))  { g_ui.d_pitch -= 6; g_ui.moving = 1; }
        if (DOWN(PAD_LEFT))  { g_ui.d_yaw   -= 6; g_ui.moving = 1; }
        if (DOWN(PAD_RIGHT)) { g_ui.d_yaw   += 6; g_ui.moving = 1; }
    }

    /* L3 zoom-to-fit ; R3 recenter pivot */
    if (PRESS(PAD_L3)) g_ui.want_zoom_fit = 1;
    if (PRESS(PAD_R3)) g_ui.want_recenter = 1;

    #undef DOWN
    #undef PRESS
    g_prev = btn;
    *out = &g_ui;
}

/* --- Damped manipulation tuning (60fps) -----------------------------------
 * Orbit/zoom run through a first-order velocity filter so motion eases in when
 * you push the stick and coasts to a stop when you release it, instead of
 * snapping on/off. Each frame the velocity approaches a target derived from the
 * raw stick impulse:  v += (target - v) >> EASE_SHIFT.  With input held this is
 * an exponential ease-IN to the steady velocity; with input released target=0,
 * so the same formula is an exponential ease-OUT (decay). EASE_SHIFT=2 ->
 * ~1/4 of the gap closed per frame (time constant ~3.5 frames, ~60ms): snappy
 * but visibly smooth on hardware.
 *
 * Velocities are stored at VEL_SHIFT extra bits of precision so slow drift
 * survives the integer shift; the camera consumes (v >> VEL_SHIFT). The *_GAIN
 * constants convert a raw stick impulse (already >>3 in poll) into a target
 * velocity (ORBIT_GAIN for yaw/pitch, ZOOM_GAIN for zoom). */
#define VEL_SHIFT   6        /* fixed-point headroom for velocities          */
#define EASE_SHIFT  2        /* approach 1/(2^EASE_SHIFT) of the gap / frame  */
#define ORBIT_GAIN  12       /* stick impulse -> target orbit velocity        */
#define ZOOM_GAIN   8        /* impulse -> target zoom velocity               */

/* Largest orbit pitch: straight up/down (SIN_LEN/4 == 90 deg). Orbit can reach
 * the pole but not pass it (velocity is zeroed at the rail), so the model never
 * flips/inverts. The TOP/BOTTOM standard views sit exactly on +/-this limit. */
#define PITCH_LIMIT (SIN_LEN/4)

/* Apply UI intent to the camera + document. History is owned by main.c and
 * passed in (centralised so the modeling-core commit and undo/redo share one
 * ring). `doc` is re-regenerated by the caller when edits land. */
void input_apply(UiState *ui, Camera *cam, Document *doc, History *hist) {
    /* --- Damped orbit ------------------------------------------------------ */
    int32_t t_yaw   = (ui->d_yaw   * ORBIT_GAIN) << VEL_SHIFT;
    int32_t t_pitch = (ui->d_pitch * ORBIT_GAIN) << VEL_SHIFT;
    ui->v_yaw   += (t_yaw   - ui->v_yaw)   >> EASE_SHIFT;
    ui->v_pitch += (t_pitch - ui->v_pitch) >> EASE_SHIFT;
    cam->yaw   += ui->v_yaw   >> VEL_SHIFT;
    cam->pitch += ui->v_pitch >> VEL_SHIFT;

    /* Yaw wraps freely (sine table is periodic); keep it in range to avoid
     * unbounded growth. Pitch is clamped so you can't flip over the pole. */
    cam->yaw &= SIN_MASK;
    if (cam->pitch >  PITCH_LIMIT) { cam->pitch =  PITCH_LIMIT; if (ui->v_pitch > 0) ui->v_pitch = 0; }
    if (cam->pitch < -PITCH_LIMIT) { cam->pitch = -PITCH_LIMIT; if (ui->v_pitch < 0) ui->v_pitch = 0; }

    /* --- Damped zoom ------------------------------------------------------- */
    int32_t t_zoom = (ui->d_zoom * ZOOM_GAIN) << VEL_SHIFT;
    ui->v_zoom += (t_zoom - ui->v_zoom) >> EASE_SHIFT;
    cam->zoom  += ui->v_zoom >> VEL_SHIFT;
    /* Clamp into a range that stays valid once folded into the 1.12 GTE matrix:
     * matrix entry = fx_mul(rot<=FX_ONE, zoom), so zoom*FX_ONE must fit int16.
     * FX_ONE*8 overflowed (8*4096 = 32768 -> -32768); cap at FX_ONE*4. Kill the
     * velocity at the rails so it doesn't keep pushing against the clamp. */
    if (cam->zoom < FX_ONE/8) { cam->zoom = FX_ONE/8; if (ui->v_zoom < 0) ui->v_zoom = 0; }
    if (cam->zoom > FX_ONE*4) { cam->zoom = FX_ONE*4; if (ui->v_zoom > 0) ui->v_zoom = 0; }

    /* --- Pan (screen-aligned, zoom-scaled, undamped: it's button-stepped) --- */
    if (ui->d_pan_x || ui->d_pan_y)
        cam_pan_screen(cam, ui->d_pan_x, ui->d_pan_y);

    if (ui->want_zoom_fit) cam_zoom_to_fit(cam, 600);   /* demo half-extent */
    if (ui->want_view)     cam_set_view(cam, (StdView)(ui->want_view - 1));
    if (ui->want_recenter) cam_recenter(cam);

    /* Undo/redo through the centralised history. When an edit lands, main.c
     * (via model_confirm) does the hist_commit; after an undo/redo the doc is
     * a different snapshot, so main.c regenerates the B-rep this frame. */
    if (ui->want_undo) hist_undo(hist, doc);
    if (ui->want_redo) hist_redo(hist, doc);
}

int      input_moving(const UiState *ui)   { return ui->moving; }
uint16_t input_selected(const UiState *ui) { return ui->selected_feat; }

#endif /* MINICAD_PSX */
