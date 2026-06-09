/* history.c — bounded undo/redo over compact document snapshots.
 *
 * Model: a stack of up to HIST_DEPTH snapshots stored in a ring buffer.
 *   base   = ring index of the OLDEST retained snapshot
 *   depth  = number of snapshots currently on the stack (<= HIST_DEPTH)
 *   cursor = 0-based position of the live snapshot within the stack
 *            (cursor == depth-1 means we're at the newest; undo decrements it)
 *
 * commit: drop everything above cursor (the redo tail), then push. If the stack
 *         is full, the oldest snapshot is evicted (base advances) — giving the
 *         "only the last 10 actions are undoable" behaviour.
 */
#include "minicad/history.h"
#include "minicad/save.h"

static int8_t s_base;     /* ring index of oldest snapshot */
static int8_t s_depth;    /* snapshots on the stack        */

static int ring_index(int8_t base, int pos) {
    return (base + pos) % HIST_DEPTH;
}

static int snapshot_into(History *h, int8_t ri, const Document *doc) {
    int n = mcad_encode(doc, h->slot[ri], HIST_SLOT_BYTES);
    if (n <= 0) return 0;
    h->len[ri] = (uint16_t)n;
    return 1;
}

void hist_init(History *h, const Document *initial) {
    s_base = 0; s_depth = 0;
    h->head = 0; h->count = 0; h->cursor = 0;
    for (int i = 0; i < HIST_DEPTH; ++i) h->len[i] = 0;
    if (initial) {
        if (snapshot_into(h, 0, initial)) {
            s_depth = 1; h->cursor = 0; h->count = 1;
        }
    }
}

int hist_commit(History *h, const Document *doc) {
    /* discard redo tail: anything above the cursor is no longer reachable */
    s_depth = (int8_t)(h->cursor + 1);

    if (s_depth >= HIST_DEPTH) {
        /* full: evict oldest by advancing base, keep depth at max */
        s_base = (int8_t)ring_index(s_base, 1);
        s_depth = HIST_DEPTH - 1;
    }

    int8_t ri = (int8_t)ring_index(s_base, s_depth);
    if (!snapshot_into(h, ri, doc)) return 0;
    s_depth++;
    h->cursor = (int8_t)(s_depth - 1);
    h->count  = s_depth;
    return 1;
}

int hist_can_undo(const History *h) { return h->cursor > 0; }
int hist_can_redo(const History *h) { return h->cursor < s_depth - 1; }

int hist_undo(History *h, Document *doc) {
    if (!hist_can_undo(h)) return 0;
    h->cursor--;
    int8_t ri = (int8_t)ring_index(s_base, h->cursor);
    return mcad_decode(doc, h->slot[ri], h->len[ri]);
}

int hist_redo(History *h, Document *doc) {
    if (!hist_can_redo(h)) return 0;
    h->cursor++;
    int8_t ri = (int8_t)ring_index(s_base, h->cursor);
    return mcad_decode(doc, h->slot[ri], h->len[ri]);
}
