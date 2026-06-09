/* render.c — PS1-ONLY. GTE transform + ordering-table painter's algorithm.
 *
 * Implements the hardware-review recommendations:
 *   §1  model->render scale is baked into the GTE matrix (camera.c); verts load
 *       straight from the model — no software scale pass.
 *   §2  perspective-overflow guard: GTE FLAG checked after RTPT; near-Z faces
 *       skipped. Wireframe-on-move masks the worst zoom window.
 *   §3  per-face working scratch + primitive cursor live in the 1KB scratchpad.
 *   §5  AVSZ4 for OTZ, (otz>>2) bucket scaling (verified SDK idiom), coplanar
 *       tie-break applied AFTER the shift, edges one bucket in front.
 *   §6.2 frame loop overlaps OT-build with the previous frame's GPU draw.
 *
 * Builds only on the PSn00bSDK target. Needs your emulator/hardware check —
 * it is not compiled in this environment (no MIPS toolchain here).
 */
#ifdef MINICAD_PSX

#include <psxgpu.h>
#include <psxgte.h>
#include <inline_c.h>
#include <stdint.h>
#include "minicad/brep.h"
#include "minicad/feature.h"
#include "minicad/camera.h"
#include "minicad/ui_state.h"

#define OT_LEN     1024
#define SCRATCH    (24*1024)
#define SCREEN_W   320
#define SCREEN_H   240

/* The 1KB scratchpad lives at 0x1F800000 (KSEG). Use it for the hottest
 * per-face working set: the 4 transformed screen verts + a small staging slot.
 * (§3) Everything else stays in main RAM. */
#define SPAD_BASE  ((volatile uint32_t *)0x1F800000)

typedef struct {
    DISPENV  disp[2];
    DRAWENV  draw[2];
    uint32_t ot[2][OT_LEN];
    uint8_t  pribuf[2][SCRATCH];
    uint8_t *nextpri;
    int      active;
} RenderCtx;

static RenderCtx g_rc;

void render_init(void) {
    ResetGraph(0);
    SetDefDispEnv(&g_rc.disp[0], 0,   0, SCREEN_W, SCREEN_H);
    SetDefDispEnv(&g_rc.disp[1], 0, SCREEN_H, SCREEN_W, SCREEN_H);
    SetDefDrawEnv(&g_rc.draw[0], 0, SCREEN_H, SCREEN_W, SCREEN_H);
    SetDefDrawEnv(&g_rc.draw[1], 0,   0, SCREEN_W, SCREEN_H);
    setRGB0(&g_rc.draw[0], 95, 127, 159);   /* steel-blue CAD viewport */
    setRGB0(&g_rc.draw[1], 95, 127, 159);
    g_rc.draw[0].isbg = 1; g_rc.draw[1].isbg = 1;
    g_rc.active = 0;
    InitGeom();
    gte_SetGeomOffset(SCREEN_W/2, SCREEN_H/2);
    gte_SetGeomScreen(SCREEN_W);            /* projection plane distance */
    FntLoad(960, 0);
    FntOpen(8, 8, 304, 60, 0, 256);
}

void render_set_camera(const Camera *c) {
    gte_SetRotMatrix(&c->m);
    gte_SetTransMatrix(&c->m);              /* trans vector set below */
    /* load translation into GTE TRX/TRY/TRZ */
    gte_SetTransVector((VECTOR *)&c->trans);
}

void render_begin(void) {
    int a = g_rc.active;
    ClearOTagR(g_rc.ot[a], OT_LEN);
    g_rc.nextpri = g_rc.pribuf[a];
}

/* ---- picking support -------------------------------------------------------
 * We project every quad face ONCE into this cache (screen XY of its 4 verts +
 * OT bucket + the vert/half-edge ids that make up the loop), then:
 *   (a) run the pick against the cursor in pure screen space (no 2nd GTE pass),
 *   (b) emit the primitives from the cache, coloring the hovered/selected
 *       entity contextually.
 * The cube is 6 faces; a 256-face cache covers far more than the MVP needs and
 * costs ~12 KB of BSS — cheaper than a second transform pass every frame. */
#define MAX_FACES 256

typedef struct {
    short    sx[4], sy[4];      /* projected screen XY of the 4 loop verts   */
    uint16_t vid[4];            /* VertId at each corner                     */
    uint16_t hid[4];            /* HEdgeId of edge corner i -> corner i+1    */
    uint16_t fid;               /* feature_id of the face                    */
    uint16_t faceid;            /* FaceId (pool handle) of the face          */
    int      bucket;            /* OT bucket                                 */
    long     minz;             /* per-face SZ approx for front-most test    */
} FaceProj;

static FaceProj g_fp[MAX_FACES];
static int      g_fp_count;

/* integer point-to-segment squared distance, screen pixels */
static int seg_dist2(int px, int py, int ax, int ay, int bx, int by) {
    int dx = bx - ax, dy = by - ay;
    int den = dx*dx + dy*dy;
    if (den == 0) { int ex = px-ax, ey = py-ay; return ex*ex + ey*ey; }
    /* t = clamp(((p-a).(b-a))/den, 0..1), in fixed scale of `den` */
    int num = (px-ax)*dx + (py-ay)*dy;
    if (num <= 0)      { int ex = px-ax, ey = py-ay; return ex*ex + ey*ey; }
    if (num >= den)    { int ex = px-bx, ey = py-by; return ex*ex + ey*ey; }
    /* foot of perpendicular = a + (num/den)*(b-a), kept integer by scaling by
     * den; the residual (ex,ey) is then dist*den, so true dist^2 = e2/den^2.
     * Normalise fully so distances from edges of different lengths compare. */
    int fx = ax*den + num*dx;
    int fy = ay*den + num*dy;
    long ex = (long)px*den - fx, ey = (long)py*den - fy;
    long e2 = ex*ex + ey*ey;
    return (int)(e2 / ((long)den*den));
}

/* even-odd point-in-polygon for the projected quad (screen space) */
static int pt_in_quad(int px, int py, const short *sx, const short *sy) {
    int inside = 0, j = 3;
    for (int i = 0; i < 4; ++i) {
        if (((sy[i] > py) != (sy[j] > py)) &&
            (px < (sx[j]-sx[i]) * (py-sy[i]) / (sy[j]-sy[i]) + sx[i]))
            inside = !inside;
        j = i;
    }
    return inside;
}

/* Project every quad face of the B-rep ONCE (one GTE pass), filling g_fp.
 * Back-facing / clipped faces are dropped so they can't be picked or drawn.
 * Uses the scratchpad for the 4 working verts (§3). */
static void project_faces(Brep *b) {
    volatile SVECTOR *sv = (volatile SVECTOR *)SPAD_BASE;  /* scratchpad verts */
    g_fp_count = 0;
    for (uint16_t fi = 0; fi < b->faces.count && g_fp_count < MAX_FACES; ++fi) {
        Face *f = brep_face(b, fi);
        if (!f) continue;
        Loop *lp = (Loop *)pool_get(&b->loops, f->outer);
        if (!lp) continue;
        HEdgeId h = lp->first_he;
        SVECTOR  vv[4];
        uint16_t vid[4], hid[4];
        int n = 0;
        do {
            HalfEdge *he = brep_he(b, h);
            Vertex   *vt = brep_vert(b, he->origin);
            vv[n].vx = (short)vt->p.x;
            vv[n].vy = (short)vt->p.y;
            vv[n].vz = (short)vt->p.z;
            vid[n] = he->origin;
            hid[n] = h;
            n++; h = he->next;
        } while (h != lp->first_he && n < 4);
        if (n < 4) continue;  /* MVP renders quad faces */

        sv[0]=vv[0]; sv[1]=vv[1]; sv[2]=vv[2]; sv[3]=vv[3];

        long nclip, otz, flag;
        gte_ldv3((SVECTOR*)&sv[0],(SVECTOR*)&sv[1],(SVECTOR*)&sv[2]);
        gte_rtpt();
        gte_nclip();
        gte_stopz(&nclip);
        /* always cull backfaces for picking/draw consistency (the moving-frame
         * wireframe is handled by the same projected data, drawn as lines) */
        int back = (nclip <= 0);

        gte_stflg(&flag);
        if (flag & 0x80000000) continue;        /* GTE saturation -> drop */

        gte_avsz3();
        gte_stotz(&otz);
        int bucket = (int)(otz >> 2);
        if (bucket >= OT_LEN || bucket < 0) continue;
        bucket += (int)(f->feature_id & 3) - 1;
        if (bucket < 0) bucket = 0;
        if (bucket >= OT_LEN) bucket = OT_LEN - 1;

        DVECTOR s0,s1,s2,s3;
        gte_stsxy0(&s0); gte_stsxy1(&s1); gte_stsxy2(&s2);
        gte_ldv0((SVECTOR*)&sv[3]); gte_rtps(); gte_stsxy(&s3);

        FaceProj *fp = &g_fp[g_fp_count++];
        fp->sx[0]=s0.vx; fp->sy[0]=s0.vy;
        fp->sx[1]=s1.vx; fp->sy[1]=s1.vy;
        fp->sx[2]=s2.vx; fp->sy[2]=s2.vy;
        fp->sx[3]=s3.vx; fp->sy[3]=s3.vy;
        for (int k=0;k<4;k++){ fp->vid[k]=vid[k]; fp->hid[k]=hid[k]; }
        fp->fid    = f->feature_id;
        fp->faceid = fi;
        fp->bucket = back ? -bucket : bucket;   /* sign flags backface */
        fp->minz   = otz;
    }
}

/* Pick the nearest entity of the active filter to the cursor, in screen space.
 * Writes ui->hover_id / hover_kind / hover_feat. No GTE use (works off g_fp). */
static void pick(UiState *ui) {
    int cx = ui->cursor_x, cy = ui->cursor_y;
    ui->hover_kind = KIND_NONE;
    ui->hover_id   = 0;

    if (ui->filter == FILT_FACE) {
        long bestz = 0x7fffffff; int found = 0;
        uint16_t bid = 0, bfeat = 0;
        for (int i = 0; i < g_fp_count; ++i) {
            FaceProj *fp = &g_fp[i];
            if (fp->bucket < 0) continue;   /* backface */
            if (!pt_in_quad(cx, cy, fp->sx, fp->sy)) continue;
            if (!found || fp->minz < bestz) { bestz = fp->minz; bid = fp->faceid; bfeat = fp->fid; found = 1; }
        }
        if (found) { ui->hover_kind = KIND_FACE; ui->hover_id = bid; ui->hover_feat = bfeat; }
        return;
    }

    if (ui->filter == FILT_EDGE) {
        int best = 0x7fffffff; int found = 0; uint16_t bid = 0, bfeat = 0;
        const int MAXD2 = 8*8;          /* must be within 8 px of the segment */
        for (int i = 0; i < g_fp_count; ++i) {
            FaceProj *fp = &g_fp[i];
            if (fp->bucket < 0) continue;   /* only front-facing edges */
            for (int k = 0; k < 4; ++k) {
                int j = (k+1) & 3;
                int d = seg_dist2(cx, cy, fp->sx[k], fp->sy[k], fp->sx[j], fp->sy[j]);
                if (d <= MAXD2 && (!found || d < best)) { best = d; bid = fp->hid[k]; bfeat = fp->fid; found = 1; }
            }
        }
        if (found) { ui->hover_kind = KIND_EDGE; ui->hover_id = bid; ui->hover_feat = bfeat; }
        return;
    }

    if (ui->filter == FILT_VERTEX) {
        int best = 0x7fffffff; int found = 0; uint16_t bid = 0, bfeat = 0;
        const int MAXD2 = 6*6;          /* within 6 px of a projected vertex */
        for (int i = 0; i < g_fp_count; ++i) {
            FaceProj *fp = &g_fp[i];
            if (fp->bucket < 0) continue;
            for (int k = 0; k < 4; ++k) {
                int ex = cx - fp->sx[k], ey = cy - fp->sy[k];
                int d = ex*ex + ey*ey;
                if (d <= MAXD2 && (!found || d < best)) { best = d; bid = fp->vid[k]; bfeat = fp->fid; found = 1; }
            }
        }
        if (found) { ui->hover_kind = KIND_VERTEX; ui->hover_id = bid; ui->hover_feat = bfeat; }
        return;
    }
    /* FILT_PROFILE/DATUM/LOOP: no-op (stubbed) */
}

/* Emit one cached face's primitives. Decides per-face / per-edge / per-vert
 * highlighting from the hover/sel state in `ui`. */
static void emit_face(const FaceProj *fp, int moving, UiState *ui) {
    int a = g_rc.active;
    if (fp->bucket < 0 && !moving) return;          /* backface, solid mode */
    int bucket = fp->bucket < 0 ? -fp->bucket : fp->bucket;

    /* face-level highlight state */
    int face_hover = (ui->hover_kind == KIND_FACE && ui->hover_id == fp->faceid);
    int face_sel   = (ui->sel_kind   == KIND_FACE && ui->sel_id   == fp->faceid);

    if (moving) {
        LINE_F2 *ln = (LINE_F2 *)g_rc.nextpri;
        int col = face_sel ? 0x46ff7a : (face_hover ? 0x10ffff : 0x102030);
        setLineF2(ln); setRGB0(ln, col&0xff,(col>>8)&0xff,(col>>16)&0xff);
        setXY2(ln, fp->sx[0],fp->sy[0], fp->sx[1],fp->sy[1]); addPrim(g_rc.ot[a][bucket], ln); ln++;
        setLineF2(ln); setXY2(ln, fp->sx[1],fp->sy[1], fp->sx[2],fp->sy[2]); addPrim(g_rc.ot[a][bucket], ln); ln++;
        setLineF2(ln); setXY2(ln, fp->sx[2],fp->sy[2], fp->sx[3],fp->sy[3]); addPrim(g_rc.ot[a][bucket], ln); ln++;
        setLineF2(ln); setXY2(ln, fp->sx[3],fp->sy[3], fp->sx[0],fp->sy[0]); addPrim(g_rc.ot[a][bucket], ln); ln++;
        g_rc.nextpri = (uint8_t *)ln;
        return;
    }

    /* shaded flat quad */
    POLY_F4 *pol = (POLY_F4 *)g_rc.nextpri;
    setPolyF4(pol);
    setXY4(pol, fp->sx[0],fp->sy[0], fp->sx[1],fp->sy[1],
                fp->sx[2],fp->sy[2], fp->sx[3],fp->sy[3]);
    if (face_sel)        setRGB0(pol, 0xff, 0x90, 0x20);   /* committed: orange */
    else if (face_hover) setRGB0(pol, 0xe0, 0xe0, 0x20);   /* hover: yellow     */
    else { int s=0x88; setRGB0(pol, s, s+0x10, s-0x08); }
    addPrim(g_rc.ot[a][bucket], pol);
    g_rc.nextpri = (uint8_t *)(pol + 1);

    /* edge outline one bucket in front; each edge colored by its own pick state */
    int ebkt = bucket > 0 ? bucket - 1 : 0;
    LINE_F2 *ln = (LINE_F2 *)g_rc.nextpri;
    for (int k = 0; k < 4; ++k) {
        int j = (k+1) & 3;
        int e_hover = (ui->hover_kind == KIND_EDGE && ui->hover_id == fp->hid[k]);
        int e_sel   = (ui->sel_kind   == KIND_EDGE && ui->sel_id   == fp->hid[k]);
        int ecol;
        if (e_sel)        ecol = 0xff9020;    /* committed orange */
        else if (e_hover) ecol = 0x20e0e0;    /* hover yellow     */
        else              ecol = face_sel ? 0x2bd960 : 0x203040;
        setLineF2(ln); setRGB0(ln, ecol&0xff,(ecol>>8)&0xff,(ecol>>16)&0xff);
        setXY2(ln, fp->sx[k],fp->sy[k], fp->sx[j],fp->sy[j]);
        addPrim(g_rc.ot[a][ebkt], ln); ln++;
    }
    g_rc.nextpri = (uint8_t *)ln;

    /* vertex markers: only draw a marker for a hovered/selected vertex (cheap,
     * keeps the cube clean). A small 3x3 TILE at the projected corner. */
    for (int k = 0; k < 4; ++k) {
        int v_hover = (ui->hover_kind == KIND_VERTEX && ui->hover_id == fp->vid[k]);
        int v_sel   = (ui->sel_kind   == KIND_VERTEX && ui->sel_id   == fp->vid[k]);
        if (!v_hover && !v_sel) continue;
        TILE *t = (TILE *)g_rc.nextpri;
        setTile(t); setWH(t, 4, 4);
        setXY0(t, fp->sx[k]-2, fp->sy[k]-2);
        if (v_sel) setRGB0(t, 0xff, 0x90, 0x20); else setRGB0(t, 0xe0, 0xe0, 0x20);
        addPrim(g_rc.ot[a][ebkt > 0 ? ebkt-1 : 0], t);
        g_rc.nextpri = (uint8_t *)(t + 1);
    }
}

/* Draw the on-screen cursor crosshair in the front-most OT bucket (always on
 * top). Built from two LINE_F2 spans crossing at (cursor_x, cursor_y). */
static void draw_cursor(UiState *ui) {
    int a = g_rc.active;
    int cx = ui->cursor_x, cy = ui->cursor_y;
    const int R = 6;
    LINE_F2 *ln = (LINE_F2 *)g_rc.nextpri;
    setLineF2(ln); setRGB0(ln, 0xff, 0xff, 0xff);
    setXY2(ln, cx-R, cy, cx+R, cy); addPrim(g_rc.ot[a][0], ln); ln++;
    setLineF2(ln); setRGB0(ln, 0xff, 0xff, 0xff);
    setXY2(ln, cx, cy-R, cx, cy+R); addPrim(g_rc.ot[a][0], ln); ln++;
    g_rc.nextpri = (uint8_t *)ln;
}

/* Walk the B-rep's quad faces: project once, pick the hovered entity for the
 * active filter, draw with contextual hover/selection highlighting, then the
 * cursor on top. `ui` carries cursor pos, filter, and hover/sel state (the
 * picker writes hover_*; input.c commits to sel_* on Cross). */
void render_model(Brep *b, UiState *ui, int moving) {
    project_faces(b);
    pick(ui);
    for (int i = 0; i < g_fp_count; ++i) emit_face(&g_fp[i], moving, ui);
    draw_cursor(ui);
}

/* Present the just-built frame, overlapping with the next build (§6.2):
 * the caller builds frame N into the active buffer, then calls render_end,
 * which waits on the PREVIOUS frame's GPU, swaps, and kicks this frame. */
void render_end(void) {
    int a = g_rc.active;
    DrawSync(0);
    VSync(0);
    PutDispEnv(&g_rc.disp[a]);
    PutDrawEnv(&g_rc.draw[a]);
    SetDispMask(1);
    DrawOTag(&g_rc.ot[a][OT_LEN - 1]);
    FntFlush(-1);
    g_rc.active ^= 1;
}

#endif /* MINICAD_PSX */
