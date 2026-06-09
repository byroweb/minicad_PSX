/* sketch.c — parametric sketcher: editing, trim, intersections, direct-rule
 * constraint resolution, profile extraction. All integer myriometers. */
#include "minicad/sketch.h"

/* ================= construction / editing ================= */
void sk_init(Sketch2 *s) {
    s->pt_count = s->ent_count = s->con_count = 0;
}

SkPointId sk_add_point(Sketch2 *s, mym_t u, mym_t v) {
    if (s->pt_count >= SK_MAX_POINTS) return SK_NONE;
    /* dedup: if a live point already sits here, share it (structural coincidence) */
    for (uint8_t i = 0; i < s->pt_count; ++i)
        if (s->pt[i].alive && s->pt[i].u == u && s->pt[i].v == v) return i;
    SkPointId id = s->pt_count++;
    s->pt[id].u = u; s->pt[id].v = v; s->pt[id].fixed = 0; s->pt[id].alive = 1;
    return id;
}

SkEntId sk_add_line(Sketch2 *s, SkPointId a, SkPointId b) {
    if (s->ent_count >= SK_MAX_ENTITIES) return SK_NONE;
    SkEntId id = s->ent_count++;
    SkEntity *e = &s->ent[id];
    e->kind = SKE_LINE; e->p0 = a; e->p1 = b; e->center = SK_NONE;
    e->radius = 0; e->construction = 0; e->alive = 1;
    return id;
}

SkEntId sk_add_rect(Sketch2 *s, mym_t u0, mym_t v0, mym_t u1, mym_t v1) {
    /* a rect is 4 shared points + 4 lines; corners auto-shared via sk_add_point */
    SkPointId a = sk_add_point(s, u0, v0);
    SkPointId b = sk_add_point(s, u1, v0);
    SkPointId c = sk_add_point(s, u1, v1);
    SkPointId d = sk_add_point(s, u0, v1);
    if (a==SK_NONE||b==SK_NONE||c==SK_NONE||d==SK_NONE) return SK_NONE;
    sk_add_line(s, a, b);
    sk_add_line(s, b, c);
    sk_add_line(s, c, d);
    SkEntId last = sk_add_line(s, d, a);
    return last;
}

SkEntId sk_add_circle(Sketch2 *s, mym_t cu, mym_t cv, mym_t radius) {
    if (s->ent_count >= SK_MAX_ENTITIES) return SK_NONE;
    SkPointId c = sk_add_point(s, cu, cv);
    if (c == SK_NONE) return SK_NONE;
    SkEntId id = s->ent_count++;
    SkEntity *e = &s->ent[id];
    e->kind = SKE_CIRCLE; e->p0 = e->p1 = SK_NONE; e->center = c;
    e->radius = radius; e->construction = 0; e->alive = 1;
    return id;
}

void sk_set_construction(Sketch2 *s, SkEntId e, int on) {
    if (e < s->ent_count && s->ent[e].alive) s->ent[e].construction = on ? 1 : 0;
}

/* ================= intersections (integer) ================= */
static SkPoint *pt(const Sketch2 *s, SkPointId id) {
    return (SkPoint *)&s->pt[id];
}

int sk_line_line(const Sketch2 *s, SkEntId ea, SkEntId eb, Vec2i *out) {
    const SkEntity *A = &s->ent[ea], *B = &s->ent[eb];
    if (A->kind != SKE_LINE || B->kind != SKE_LINE) return 0;
    SkPoint *p1 = pt(s, A->p0), *p2 = pt(s, A->p1);
    SkPoint *p3 = pt(s, B->p0), *p4 = pt(s, B->p1);
    mym2_t x1=p1->u,y1=p1->v, x2=p2->u,y2=p2->v, x3=p3->u,y3=p3->v, x4=p4->u,y4=p4->v;
    mym2_t den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (den == 0) return 0;                       /* parallel */
    mym2_t t_num = (x1-x3)*(y3-y4) - (y1-y3)*(x3-x4);
    mym2_t u_num = (x1-x3)*(y1-y2) - (y1-y3)*(x1-x2);
    /* require intersection within both segments: 0<=t<=1 and 0<=u<=1 */
    if (den > 0) { if (t_num<0||t_num>den||u_num<0||u_num>den) return 0; }
    else         { if (t_num>0||t_num<den||u_num>0||u_num<den) return 0; }
    out->u = (mym_t)(x1 + mym_div_round((x2-x1)*t_num, den));
    out->v = (mym_t)(y1 + mym_div_round((y2-y1)*t_num, den));
    return 1;
}

int sk_line_circle(const Sketch2 *s, SkEntId el, SkEntId ec,
                   Vec2i *o0, Vec2i *o1, int *n) {
    const SkEntity *L = &s->ent[el], *C = &s->ent[ec];
    if (L->kind != SKE_LINE || C->kind != SKE_CIRCLE) return 0;
    SkPoint *a = pt(s, L->p0), *b = pt(s, L->p1), *cc = pt(s, C->center);
    mym2_t dx=b->u-a->u, dy=b->v-a->v;
    mym2_t fx=a->u-cc->u, fy=a->v-cc->v, r=C->radius;
    mym2_t A=dx*dx+dy*dy, B=2*(fx*dx+fy*dy), Cc=fx*fx+fy*fy-r*r;
    mym2_t disc = B*B - 4*A*Cc;
    if (disc < 0 || A == 0) { *n=0; return 0; }
    /* integer sqrt of disc */
    mym2_t lo=0, hi=disc, root=0;
    while (lo<=hi){ mym2_t mid=(lo+hi)/2; if (mid*mid<=disc){root=mid;lo=mid+1;} else hi=mid-1; }
    mym2_t t0n = -B - root, t1n = -B + root, d2 = 2*A;
    int cnt=0;
    mym2_t ts[2]={t0n,t1n};
    Vec2i *outs[2]={o0,o1};
    for (int i=0;i<2;i++){
        if (ts[i]>=0 && ts[i]<=d2){            /* within segment */
            outs[cnt]->u=(mym_t)(a->u + mym_div_round(dx*ts[i], d2));
            outs[cnt]->v=(mym_t)(a->v + mym_div_round(dy*ts[i], d2));
            cnt++;
        }
    }
    *n=cnt; return cnt>0;
}

/* ================= trim ================= */
void sk_trim(Sketch2 *s, SkEntId e, TrimMode mode) {
    if (e >= s->ent_count || !s->ent[e].alive) return;
    if (mode == TRIM_TO_CONSTRUCTION) { s->ent[e].construction = 1; return; }
    /* TRIM_DELETE: MVP deletes the whole entity. Segment-between-intersections
     * uses sk_line_line/sk_line_circle to find the two nearest crossings and
     * splits the entity there — wired next; deleting the entity is the safe
     * fallback that always leaves a valid sketch. */
    s->ent[e].alive = 0;
}

/* ================= constraints ================= */
SkConstraint *sk_add_constraint(Sketch2 *s, SkConstraintKind kind,
                                SkEntId e0, SkEntId e1, SkPointId a, SkPointId b,
                                mym_t value) {
    if (s->con_count >= SK_MAX_CONSTRAINTS) return 0;
    SkConstraint *c = &s->con[s->con_count++];
    c->kind=kind; c->e0=e0; c->e1=e1; c->a=a; c->b=b; c->value=value;
    c->solved=0; c->alive=1;
    return c;
}

static mym_t line_len2(const Sketch2 *s, SkEntId e) {
    const SkEntity *L=&s->ent[e];
    SkPoint *p0=pt(s,L->p0),*p1=pt(s,L->p1);
    mym2_t dx=p1->u-p0->u, dy=p1->v-p0->v;
    return (mym_t)(dx*dx+dy*dy);   /* squared length (integer) */
}

int sk_resolve_direct(Sketch2 *s) {
    int unsolved = 0;
    for (uint8_t i = 0; i < s->con_count; ++i) {
        SkConstraint *c = &s->con[i];
        if (!c->alive) continue;
        c->solved = 0;
        switch (c->kind) {
        case SKC_FIX:
            if (c->a != SK_NONE) { s->pt[c->a].fixed = 1; c->solved = 1; }
            break;
        case SKC_COINCIDENT:
            /* snap b onto a (structural share is preferred at creation; this
             * handles after-the-fact coincidence). */
            if (c->a!=SK_NONE && c->b!=SK_NONE) {
                s->pt[c->b].u = s->pt[c->a].u;
                s->pt[c->b].v = s->pt[c->a].v;
                c->solved = 1;
            }
            break;
        case SKC_HORIZONTAL:
            if (c->e0!=SK_NONE && s->ent[c->e0].kind==SKE_LINE) {
                SkEntity *L=&s->ent[c->e0];
                mym_t v = s->pt[L->p0].v;            /* level both ends */
                if (!s->pt[L->p1].fixed) s->pt[L->p1].v = v;
                else s->pt[L->p0].v = s->pt[L->p1].v;
                c->solved = 1;
            }
            break;
        case SKC_VERTICAL:
            if (c->e0!=SK_NONE && s->ent[c->e0].kind==SKE_LINE) {
                SkEntity *L=&s->ent[c->e0];
                mym_t u = s->pt[L->p0].u;
                if (!s->pt[L->p1].fixed) s->pt[L->p1].u = u;
                else s->pt[L->p0].u = s->pt[L->p1].u;
                c->solved = 1;
            }
            break;
        case SKC_EQUAL:
            /* make e1 the same size as e0 (lines: length; circles: radius) */
            if (c->e0!=SK_NONE && c->e1!=SK_NONE) {
                SkEntity *A=&s->ent[c->e0], *B=&s->ent[c->e1];
                if (A->kind==SKE_CIRCLE && B->kind==SKE_CIRCLE) {
                    B->radius = A->radius; c->solved = 1;
                } else if (A->kind==SKE_LINE && B->kind==SKE_LINE) {
                    /* scale B's p1 about p0 to match A's length (squared cmp).
                     * Integer-exact only when lengths already match; otherwise
                     * record as a hint and leave for the solver. */
                    if (line_len2(s,c->e0)==line_len2(s,c->e1)) c->solved=1;
                }
            }
            break;
        /* --- need the iterative solver: store, count as unsolved --- */
        case SKC_PARALLEL: case SKC_PERPENDICULAR:
        case SKC_TANGENT:   case SKC_DIMENSION:
        default:
            c->solved = 0;
            break;
        }
        if (!c->solved) unsolved++;
    }
    return unsolved;
}

/* ================= profile extraction ================= */
int sk_extract_profile(const Sketch2 *s, Vec2i *ring, int cap) {
    /* MVP: a single non-construction circle, or a closed chain of
     * non-construction lines whose endpoints connect. Construction geometry is
     * skipped. Returns ring point count. */
    /* circle first */
    for (uint8_t i = 0; i < s->ent_count; ++i) {
        const SkEntity *e = &s->ent[i];
        if (!e->alive || e->construction) continue;
        if (e->kind == SKE_CIRCLE) {
            /* tessellation done in ops.c; here we just hand back center+radius
             * encoded as 1 "ring point" sentinel for the op to expand. For the
             * line path we return real points. Keep circle handling in ops. */
            if (cap < 1) return 0;
            ring[0].u = s->pt[e->center].u;
            ring[0].v = s->pt[e->center].v;
            return -e->radius;   /* negative count signals "circle r=|n|" */
        }
    }
    /* line chain: walk lines, collecting unique endpoints in order */
    int n = 0;
    SkPointId start = SK_NONE, cur = SK_NONE, prev_pt = SK_NONE;
    for (uint8_t i = 0; i < s->ent_count; ++i) {
        const SkEntity *e = &s->ent[i];
        if (!e->alive || e->construction || e->kind != SKE_LINE) continue;
        if (start == SK_NONE) { start = e->p0; cur = e->p0; }
        if (n >= cap) return 0;
        ring[n].u = s->pt[cur].u; ring[n].v = s->pt[cur].v; n++;
        prev_pt = cur;
        cur = (e->p0 == prev_pt) ? e->p1 : e->p0;
    }
    return n >= 3 ? n : 0;
}
