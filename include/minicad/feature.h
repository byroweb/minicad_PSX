/* feature.h — the feature tree. THE TREE IS THE PART FILE.
 *
 * A Document owns an ordered list of features. Regenerating the tree in
 * dependency order rebuilds the B-rep. Reference geometry (datum plane/axis/
 * point) are first-class features that produce datum results consumed by
 * downstream features — exactly the SolidWorks FeatureManager model.
 */
#ifndef MINICAD_FEATURE_H
#define MINICAD_FEATURE_H

#include "minicad/ops.h"

typedef enum {
    FEAT_SKETCH = 0,
    FEAT_EXTRUDE,
    FEAT_REVOLVE,
    FEAT_REF_PLANE,
    FEAT_REF_AXIS,
    FEAT_REF_POINT,
    FEAT_KIND_COUNT
} FeatureKind;

/* How a reference-geometry datum is constructed (its "method"). */
typedef enum {
    REFP_OFFSET = 0,     /* plane offset from a parent plane           */
    REFP_3POINTS,        /* plane through 3 points                     */
    REFA_TWO_PLANES,     /* axis = intersection of 2 planes            */
    REFA_TWO_POINTS,     /* axis through 2 points                      */
    REFPT_LINE_PLANE,    /* point = line ∩ plane                       */
    REFPT_VERTEX         /* point at an existing vertex                */
} RefMethod;

#define FEAT_MAX_DEPS 4
#define FEAT_NAME_LEN 24

typedef struct {
    FeatureKind kind;
    uint16_t    id;
    char        name[FEAT_NAME_LEN];   /* "Boss-Extrude1", "Axis1", ... */
    uint16_t    depends_on[FEAT_MAX_DEPS];
    uint8_t     dep_count;
    uint8_t     dirty;
    uint8_t     suppressed;

    /* parameters, by kind */
    union {
        struct {
            uint16_t sketch_id;
            mym_t    dist;        /* blind distance (computed for non-blind) */
            int8_t   dir;
            OpType   op;          /* boss or cut                              */
            EndCond  end;         /* blind / through-all / up-to-surface      */
            uint16_t target_face; /* up-to-surface reference (else 0)         */
        } extrude;
        struct {
            uint16_t sketch_id;
            int32_t  sweep;
            int16_t  steps;
            OpType   op;
        } revolve;
        struct { RefMethod method; mym_t offset; } ref;
    } p;

    Sketch  sketch;        /* valid when kind == FEAT_SKETCH        */
    SolidId result;        /* B-rep solid produced (BREP_NONE if datum/sketch) */
} Feature;

#define DOC_MAX_FEATURES 128

typedef struct {
    Feature  feat[DOC_MAX_FEATURES];
    uint16_t feat_count;
    uint16_t next_id;
    char     title[FEAT_NAME_LEN];     /* "Part1" */
} Document;

void     doc_init(Document *d, const char *title);
Feature *doc_add(Document *d, FeatureKind kind, const char *name);
Feature *doc_find(Document *d, uint16_t id);
void     doc_mark_dirty(Document *d, uint16_t id);   /* + transitive dependents */

/* Regenerate dirty features in dependency order into the given Brep + scratch.
 * Returns 1 on success. */
int doc_regen(Document *d, Brep *b);

#endif /* MINICAD_FEATURE_H */
