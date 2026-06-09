/* main.c — entry point.
 *
 * On the PSX target this boots graphics+pad and runs the modeller loop on the
 * demo part (a 60mm cube with a 30mm through-hole, built from features).
 * On the host build it does nothing (host_tests/ drives the kernel instead).
 */
#include "minicad/feature.h"
#include "minicad/save.h"
#include "minicad/history.h"
#include "minicad/modeling.h"
#include "minicad/memcard.h"

#ifdef MINICAD_PSX
/* Backing memory for the B-rep pools (the Model DB arena, statically sized).
 * PSX-only: the host build's main() is a no-op and the kernel is exercised by
 * host_tests/ instead, so these would be unused there. */
static Vertex   s_verts [2048];
static HalfEdge s_hedges[8192];
static Edge     s_edges [4096];
static Loop     s_loops [2048];
static Face     s_faces [1024];
static Shell    s_shells[64];
static Solid    s_solids[64];

static Brep     g_brep;
static Document g_doc;
static History  g_hist;          /* centralised undo/redo (20KB) — keep static */
static ModelingState g_model;    /* in-progress interactive edit              */

/* Build the demo part purely from the feature recipe. */
static void build_demo(Document *d) {
    doc_init(d, "Part1");

    /* Sketch1: a 60mm square (±300 mym) on the XY plane. */
    Feature *sk = doc_add(d, FEAT_SKETCH, "Sketch1");
    sk->plane = plane_xy();
    sk_init(&sk->sketch);
    sk_add_rect(&sk->sketch, -300, -300, 300, 300);

    /* Boss-Extrude1: 60mm tall, blind. */
    Feature *ex = doc_add(d, FEAT_EXTRUDE, "Boss-Extrude1");
    ex->p.extrude.sketch_id = sk->id;
    ex->p.extrude.dist = 600;     /* 60.0 mm */
    ex->p.extrude.dir  = 1;
    ex->p.extrude.op   = OP_BOSS;
    ex->p.extrude.end  = END_BLIND;
    ex->p.extrude.target_face = 0;
    ex->depends_on[0] = sk->id; ex->dep_count = 1;

    /* Sketch2: a Ø30 circle centered, on the top face (datum TODO). */
    Feature *sk2 = doc_add(d, FEAT_SKETCH, "Sketch2");
    sk2->plane = plane_xy();
    sk_init(&sk2->sketch);
    sk_add_circle(&sk2->sketch, 0, 0, 150);     /* 15.0 mm radius */

    /* Cut-Extrude1: through-all hole. */
    Feature *cut = doc_add(d, FEAT_EXTRUDE, "Cut-Extrude1");
    cut->p.extrude.sketch_id = sk2->id;
    cut->p.extrude.dist = 600;
    cut->p.extrude.dir  = 1;
    cut->p.extrude.op   = OP_CUT;
    cut->p.extrude.end  = END_THROUGH_ALL;
    cut->p.extrude.target_face = 0;
    cut->depends_on[0] = sk2->id; cut->dep_count = 1;
}

#include "minicad/camera.h"
#include "minicad/ui_state.h"
#include <psxgpu.h>      /* FntPrint for the on-screen status line */
extern void render_init(void);
extern void render_begin(void);
extern void render_set_camera(const Camera *c);
extern void render_model(Brep *b, UiState *ui, int moving);
extern void render_panel(Document *doc, UiState *ui);
extern void render_end(void);
extern void input_init(void);
extern void input_poll(UiState **out);
extern void input_apply(UiState *ui, Camera *cam, Document *doc, History *hist);
extern int  input_moving(const UiState *ui);
extern uint16_t input_selected(const UiState *ui);

int main(void) {
    brep_init(&g_brep,
              s_verts,2048, s_hedges,8192, s_edges,4096, s_loops,2048,
              s_faces,1024, s_shells,64,   s_solids,64);
    build_demo(&g_doc);
    doc_regen(&g_doc, &g_brep);

    /* Centralised history (init once on the clean baseline) + the interactive
     * modeling state. main.c drives the modeling core off the UI intents. */
    hist_init(&g_hist, &g_doc);
    model_init(&g_model);

    render_init();
    input_init();
    mc_init();

    Camera cam; cam_init(&cam);

    /* .mcad scratch + a brief on-screen status line for file ops. */
    static uint8_t s_mcad[512];
    const char *status = "";
    int status_ttl = 0;        /* frames the status line stays up */

    /* Frame loop with build/draw overlap (§6.2): poll + camera + OT-build for
     * frame N happen while the GPU draws frame N-1; render_end syncs + swaps. */
    for (;;) {
        UiState *ui;
        input_poll(&ui);

        /* Tell input.c whether an edit is in progress so it routes Cross/Circle
         * to confirm/cancel instead of select/deselect this frame. */
        ui->modeling_pending = (int8_t)g_model.pending;

        input_apply(ui, &cam, &g_doc, &g_hist);  /* camera + undo/redo */

        /* --- Interactive modeling: drive the modeling core off UI intents --- */
        if (g_model.pending) {
            if (ui->want_dist_delta)
                model_nudge_distance(&g_model, &g_doc, &g_brep, ui->want_dist_delta);
            if (ui->want_confirm)
                model_confirm(&g_model, &g_doc, &g_brep, &g_hist);
            else if (ui->want_cancel)
                model_cancel(&g_model, &g_doc, &g_brep);
        } else {
            if (ui->want_new_boss && ui->sel_kind == KIND_FACE)
                model_begin_extrude(&g_model, &g_doc, &g_brep,
                                    (FaceId)ui->sel_id, OP_BOSS);
            else if (ui->want_new_cut && ui->sel_kind == KIND_FACE)
                model_begin_extrude(&g_model, &g_doc, &g_brep,
                                    (FaceId)ui->sel_id, OP_CUT);
        }

        /* After an undo/redo the document is a different snapshot: rebuild the
         * B-rep so the view reflects it. (A no-op cost on idle frames is fine.) */
        if (ui->want_undo || ui->want_redo) {
            model_regen_all(&g_doc, &g_brep);
            /* a pending edit can't survive a history jump */
            g_model.pending = 0;
        }

        /* --- File ops (memory card). Card I/O blocks briefly; this happens at
         * most once per button press, so the steady 60fps path is untouched. -- */
        if (ui->want_save) {
            int n = mcad_encode(&g_doc, s_mcad, (int)sizeof s_mcad);
            if (n > 0 && mc_save(0, 0, 0, s_mcad, n) == 0)
                status = "SAVED TO CARD";
            else
                status = "SAVE FAILED";
            status_ttl = 120;
        } else if (ui->want_load) {
            int n = mc_load(0, 0, 0, s_mcad, (int)sizeof s_mcad);
            if (n > 0 && mcad_decode(&g_doc, s_mcad, n)) {
                doc_regen(&g_doc, &g_brep);
                hist_init(&g_hist, &g_doc);     /* reset undo baseline to load */
                model_init(&g_model);
                status = "LOADED FROM CARD";
            } else {
                status = "LOAD FAILED";
            }
            status_ttl = 120;
        } else if (ui->want_new) {
            build_demo(&g_doc);
            doc_regen(&g_doc, &g_brep);
            hist_init(&g_hist, &g_doc);
            model_init(&g_model);
            status = "NEW PART";
            status_ttl = 120;
        }

        cam_update(&cam);

        render_begin();
        render_set_camera(&cam);
        render_model(&g_brep, ui, input_moving(ui));
        render_panel(&g_doc, ui);
        if (status_ttl > 0) { FntPrint(-1, "%s\n", status); status_ttl--; }
        render_end();
    }
    return 0;
}
#else
/* Host build: provide a no-op main so the library links for tooling. */
int main(void) { return 0; }
#endif
