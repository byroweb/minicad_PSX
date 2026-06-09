/* mem.h — arena (bump) + object-pool allocators with a global budget.
 *
 * No scattered malloc in kernel code. Two strategies:
 *   - Arena: forward-only bump allocation, freed wholesale by reset.
 *     Used for per-regen scratch and transient buffers.
 *   - Pool : fixed-size free-list for high-count B-rep entities.
 *     O(1) alloc/free, no fragmentation, hard per-pool cap.
 *
 * A MemBudget tracks bytes and refuses allocation past a ceiling, so a model
 * that is too large fails gracefully (returns NULL / ERR_BUDGET) rather than
 * crashing the 2 MB console.
 */
#ifndef MINICAD_MEM_H
#define MINICAD_MEM_H

#include <stddef.h>
#include <stdint.h>

/* ---- Arena ---- */
typedef struct {
    uint8_t *base;
    size_t   size;
    size_t   used;
    const char *name;
} Arena;

void  arena_init(Arena *a, void *backing, size_t size, const char *name);
void *arena_alloc(Arena *a, size_t bytes, size_t align);
void  arena_reset(Arena *a);                 /* free everything at once   */
size_t arena_used(const Arena *a);

/* ---- Object pool (fixed element size, 16-bit index handles) ---- */
typedef uint16_t PoolId;
#define POOL_NONE ((PoolId)0xFFFF)

typedef struct {
    uint8_t *base;       /* backing store                       */
    uint16_t elem_size;  /* bytes per element                   */
    uint16_t capacity;   /* max elements                        */
    uint16_t count;      /* live elements                       */
    PoolId   free_head;  /* head of free list (POOL_NONE = none) */
    const char *name;
} Pool;

void   pool_init(Pool *p, void *backing, uint16_t elem_size,
                 uint16_t capacity, const char *name);
PoolId pool_alloc(Pool *p);                  /* returns POOL_NONE if full */
void   pool_free(Pool *p, PoolId id);
void  *pool_get(Pool *p, PoolId id);         /* index -> pointer          */

/* ---- Global budget ---- */
void   budget_set_ceiling(size_t bytes);
int    budget_request(size_t bytes);         /* 1 = ok, 0 = over ceiling  */
void   budget_release(size_t bytes);
size_t budget_used(void);

#endif /* MINICAD_MEM_H */
