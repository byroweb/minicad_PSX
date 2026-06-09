/* mem.c — arena + pool allocators + global budget. */
#include "minicad/mem.h"

/* ---------------- budget ---------------- */
static size_t s_ceiling = (size_t)1536 * 1024; /* 1.5 MB default */
static size_t s_used    = 0;

void   budget_set_ceiling(size_t bytes) { s_ceiling = bytes; }
size_t budget_used(void)                { return s_used; }
int    budget_request(size_t bytes) {
    if (s_used + bytes > s_ceiling) return 0;
    s_used += bytes; return 1;
}
void   budget_release(size_t bytes) {
    s_used = (bytes > s_used) ? 0 : (s_used - bytes);
}

/* ---------------- arena ---------------- */
static size_t align_up(size_t v, size_t a) {
    if (a == 0) a = 1;
    return (v + (a - 1)) & ~(a - 1);
}

void arena_init(Arena *a, void *backing, size_t size, const char *name) {
    a->base = (uint8_t *)backing;
    a->size = size;
    a->used = 0;
    a->name = name;
}

void *arena_alloc(Arena *a, size_t bytes, size_t align) {
    size_t off = align_up(a->used, align ? align : 8);
    if (off + bytes > a->size) return 0;   /* out of arena */
    void *p = a->base + off;
    a->used = off + bytes;
    return p;
}

void   arena_reset(Arena *a)        { a->used = 0; }
size_t arena_used(const Arena *a)   { return a->used; }

/* ---------------- pool ----------------
 * Free elements form a singly-linked list; the first 2 bytes of a free slot
 * hold the next free index. Live elements use the full element_size. */
void pool_init(Pool *p, void *backing, uint16_t elem_size,
               uint16_t capacity, const char *name) {
    p->base      = (uint8_t *)backing;
    p->elem_size = elem_size;
    p->capacity  = capacity;
    p->count     = 0;
    p->name      = name;
    /* thread the free list */
    p->free_head = 0;
    for (uint16_t i = 0; i < capacity; ++i) {
        PoolId next = (PoolId)((i + 1 < capacity) ? (i + 1) : POOL_NONE);
        *(PoolId *)(p->base + (size_t)i * elem_size) = next;
    }
    if (capacity == 0) p->free_head = POOL_NONE;
}

PoolId pool_alloc(Pool *p) {
    if (p->free_head == POOL_NONE) return POOL_NONE;
    PoolId id = p->free_head;
    p->free_head = *(PoolId *)(p->base + (size_t)id * p->elem_size);
    p->count++;
    return id;
}

void pool_free(Pool *p, PoolId id) {
    if (id >= p->capacity) return;
    *(PoolId *)(p->base + (size_t)id * p->elem_size) = p->free_head;
    p->free_head = id;
    if (p->count) p->count--;
}

void *pool_get(Pool *p, PoolId id) {
    if (id >= p->capacity) return 0;
    return p->base + (size_t)id * p->elem_size;
}
