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

/* Transform & emit one quad face. `moving` => wireframe (lines only).
 * Returns 1 if drawn, 0 if culled/clipped. */
static int draw_quad_face(const SVECTOR *v0, const SVECTOR *v1,
                          const SVECTOR *v2, const SVECTOR *v3,
                          int moving, int selected, uint16_t fid) {
    int a = g_rc.active;
    long nclip, otz, flag;

    /* transform first three with one RTPT (cheaper than 3x RTPS, §0) */
    gte_ldv3(v0, v1, v2);
    gte_rtpt();

    /* backface cull via NCLIP (skip when wireframe so we see the back) */
    gte_nclip();
    gte_stopz(&nclip);
    if (!moving && nclip <= 0) return 0;

    /* perspective-overflow guard (§2): bail on near-Z explosion */
    gte_stflg(&flag);
    if (flag & 0x80000000) return 0;        /* GTE FLAG error/saturation bit */

    /* average Z -> OT bucket (§5). Only 3 verts are transformed here (v3 is
     * projected below), so use AVSZ3 — AVSZ4 would average a stale SZ3 left
     * over from a previous primitive, far-clipping most faces. */
    gte_avsz3();
    gte_stotz(&otz);
    int bucket = (int)(otz >> 2);
    if (bucket >= OT_LEN) return 0;          /* far clip */
    if (bucket < 0) return 0;                /* behind eye */
    /* coplanar/concentric tie-break AFTER the shift (§5) */
    bucket += (int)(fid & 3) - 1;
    if (bucket < 0) bucket = 0;
    if (bucket >= OT_LEN) bucket = OT_LEN - 1;

    if (moving) {
        /* wireframe: 4 edges of the quad as lines */
        LINE_F2 *ln = (LINE_F2 *)g_rc.nextpri;
        DVECTOR s0,s1,s2,s3;
        gte_stsxy0(&s0); gte_stsxy1(&s1); gte_stsxy2(&s2);
        gte_ldv0(v3); gte_rtps(); gte_stsxy(&s3);
        int col = selected ? 0x46ff7a : 0x102030;
        setLineF2(ln); setRGB0(ln, col&0xff,(col>>8)&0xff,(col>>16)&0xff);
        setXY2(ln, s0.vx,s0.vy, s1.vx,s1.vy); addPrim(g_rc.ot[a][bucket], ln); ln++;
        setLineF2(ln); setXY2(ln, s1.vx,s1.vy, s2.vx,s2.vy); addPrim(g_rc.ot[a][bucket], ln); ln++;
        setLineF2(ln); setXY2(ln, s2.vx,s2.vy, s3.vx,s3.vy); addPrim(g_rc.ot[a][bucket], ln); ln++;
        setLineF2(ln); setXY2(ln, s3.vx,s3.vy, s0.vx,s0.vy); addPrim(g_rc.ot[a][bucket], ln); ln++;
        g_rc.nextpri = (uint8_t *)ln;
        return 1;
    }

    /* shaded: flat quad + a crisp edge outline one bucket in front */
    POLY_F4 *pol = (POLY_F4 *)g_rc.nextpri;
    setPolyF4(pol);
    gte_stsxy0(&pol->x0); gte_stsxy1(&pol->x1); gte_stsxy2(&pol->x2);
    gte_ldv0(v3); gte_rtps(); gte_stsxy(&pol->x3);
    int shade = selected ? 0xa0 : 0x88;
    setRGB0(pol, shade, shade+0x10, shade-0x08);
    addPrim(g_rc.ot[a][bucket], pol);
    g_rc.nextpri = (uint8_t *)(pol + 1);

    /* edge outline in the bucket in front so it stays readable (§5/DESIGN) */
    int ebkt = bucket > 0 ? bucket - 1 : 0;
    LINE_F2 *ln = (LINE_F2 *)g_rc.nextpri;
    int ecol = selected ? 0x2bd960 : 0x203040;
    setLineF2(ln); setRGB0(ln, ecol&0xff,(ecol>>8)&0xff,(ecol>>16)&0xff);
    setXY2(ln, pol->x0,pol->y0, pol->x1,pol->y1); addPrim(g_rc.ot[a][ebkt], ln); ln++;
    setLineF2(ln); setXY2(ln, pol->x1,pol->y1, pol->x2,pol->y2); addPrim(g_rc.ot[a][ebkt], ln); ln++;
    setLineF2(ln); setXY2(ln, pol->x2,pol->y2, pol->x3,pol->y3); addPrim(g_rc.ot[a][ebkt], ln); ln++;
    setLineF2(ln); setXY2(ln, pol->x3,pol->y3, pol->x0,pol->y0); addPrim(g_rc.ot[a][ebkt], ln); ln++;
    g_rc.nextpri = (uint8_t *)ln;
    return 1;
}

/* Walk the B-rep's quad faces and render them. The model verts are int16
 * (myriometers, pre-clamped to GTE range by the design envelope) loaded into
 * SVECTORs. Scratchpad holds the 4 working verts (§3). */
void render_model(Brep *b, uint16_t selected_feat, int moving) {
    volatile SVECTOR *sv = (volatile SVECTOR *)SPAD_BASE;  /* scratchpad verts */
    for (uint16_t fi = 0; fi < b->faces.count; ++fi) {
        Face *f = brep_face(b, fi);
        if (!f) continue;
        /* gather the (up to 4) loop verts of this face */
        Loop *lp = (Loop *)pool_get(&b->loops, f->outer);
        if (!lp) continue;
        HEdgeId h = lp->first_he;
        SVECTOR vv[4]; int n = 0;
        do {
            HalfEdge *he = brep_he(b, h);
            Vertex *vt = brep_vert(b, he->origin);
            vv[n].vx = (short)vt->p.x;
            vv[n].vy = (short)vt->p.y;
            vv[n].vz = (short)vt->p.z;
            n++; h = he->next;
        } while (h != lp->first_he && n < 4);
        if (n < 4) continue;  /* MVP renders quad faces */

        sv[0]=vv[0]; sv[1]=vv[1]; sv[2]=vv[2]; sv[3]=vv[3];
        int sel = (f->feature_id == selected_feat);
        draw_quad_face((SVECTOR*)&sv[0],(SVECTOR*)&sv[1],
                       (SVECTOR*)&sv[2],(SVECTOR*)&sv[3], moving, sel, f->feature_id);
    }
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
