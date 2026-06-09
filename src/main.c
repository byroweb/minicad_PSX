/* main.c — entry point.
 *
 * On the PSX target this boots graphics+pad and runs the modeller loop on the
 * demo part (a 60mm cube with a 30mm through-hole, built from features).
 * On the host build it does nothing (host_tests/ drives the kernel instead).
 */
#include "minicad/feature.h"
#include "minicad/save.h"

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

/* Build the demo part purely from the feature recipe. */
static void build_demo(Document *d) {
    doc_init(d, "Part1");

    /* Sketch1: a 60mm square (±300 mym) on the XY plane. */
    Feature *sk = doc_add(d, FEAT_SKETCH, "Sketch1");
    sk->sketch.plane = plane_xy();
    sk->sketch.count = 1;
    sk->sketch.ent[0].kind = SK_RECT;
    sk->sketch.ent[0].a = (Vec2i){ -300, -300 };
    sk->sketch.ent[0].b = (Vec2i){  300,  300 };

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
    sk2->sketch.plane = plane_xy();
    sk2->sketch.count = 1;
    sk2->sketch.ent[0].kind = SK_CIRCLE;
    sk2->sketch.ent[0].a = (Vec2i){ 0, 0 };
    sk2->sketch.ent[0].radius = 150;     /* 15.0 mm radius */

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
extern void render_init(void);
extern void render_begin(void);
extern void render_set_camera(const Camera *c);
extern void render_model(Brep *b, uint16_t selected_feat, int moving);
extern void render_end(void);
extern void input_init(void);
extern void input_poll(UiState **out);
extern void input_apply(UiState *ui, Camera *cam, Document *doc);
extern int  input_moving(const UiState *ui);
extern uint16_t input_selected(const UiState *ui);

int main(void) {
    brep_init(&g_brep,
              s_verts,2048, s_hedges,8192, s_edges,4096, s_loops,2048,
              s_faces,1024, s_shells,64,   s_solids,64);
    build_demo(&g_doc);
    doc_regen(&g_doc, &g_brep);

    render_init();
    input_init();

    Camera cam; cam_init(&cam);

    /* Frame loop with build/draw overlap (§6.2): poll + camera + OT-build for
     * frame N happen while the GPU draws frame N-1; render_end syncs + swaps. */
    for (;;) {
        UiState *ui;
        input_poll(&ui);
        input_apply(ui, &cam, &g_doc);     /* camera moves, edits -> regen flag */
        cam_update(&cam);

        render_begin();
        render_set_camera(&cam);
        render_model(&g_brep, input_selected(ui), input_moving(ui));
        render_end();
    }
    return 0;
}
#else
/* Host build: provide a no-op main so the library links for tooling. */
int main(void) { return 0; }
#endif
