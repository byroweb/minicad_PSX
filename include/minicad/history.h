/* history.h — bounded undo/redo via compact document snapshots.
 *
 * DESIGN: a 10-deep ring of serialized .mcad images. Because the save codec is
 * already compact (a whole part is tens of bytes) and the depth is bounded,
 * snapshotting the entire document per commit is both tiny in RAM and totally
 * robust — it captures full state regardless of which edit happened, so there
 * is no per-edit inverse logic to get subtly wrong. Undo/redo just re-decode a
 * neighbouring snapshot.
 *
 *   hist_commit(h, doc)  -> call AFTER applying an edit; pushes a new snapshot,
 *                           clears the redo tail (standard undo semantics).
 *   hist_undo(h, doc)    -> step back one snapshot (1 = moved, 0 = nothing to undo)
 *   hist_redo(h, doc)    -> step forward one snapshot
 *
 * Memory: HIST_DEPTH * HIST_SLOT_BYTES, statically allocated (no heap).
 */
#ifndef MINICAD_HISTORY_H
#define MINICAD_HISTORY_H

#include "minicad/feature.h"

#define HIST_DEPTH       10        /* 10x undo/redo as specified            */
#define HIST_SLOT_BYTES  2048      /* per-snapshot cap (a big part is < 1KB) */

typedef struct {
    uint8_t  slot[HIST_DEPTH][HIST_SLOT_BYTES];
    uint16_t len [HIST_DEPTH];     /* bytes used in each slot               */
    int8_t   head;                 /* index of the current (live) snapshot  */
    int8_t   count;                /* total valid snapshots in the ring     */
    int8_t   cursor;               /* position within the undo stack        */
} History;

void hist_init(History *h, const Document *initial);

/* Push the document's current state as a new snapshot. Call after each edit.
 * Discards any redo states beyond the cursor. Returns 1 on success. */
int  hist_commit(History *h, const Document *doc);

/* Move back/forward; decode that snapshot into `doc`. Returns 1 if moved. */
int  hist_undo(History *h, Document *doc);
int  hist_redo(History *h, Document *doc);

int  hist_can_undo(const History *h);
int  hist_can_redo(const History *h);

#endif /* MINICAD_HISTORY_H */
