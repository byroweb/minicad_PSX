# MiniCAD-PSX — Hardware Optimization Review (R3000A / GTE / PSn00bSDK)

> Senior-developer pass over the plan and the code, judged against real PS1
> hardware: the MIPS I R3000A, the GTE (COP2), the cache/scratchpad model, and
> how PSn00bSDK actually boots and runs. Findings are ordered by impact, with
> concrete code changes. Where the current code is wrong for hardware, it says so.

---

## 0. The hardware facts that actually matter (verified)

| Fact | Source of truth | Why it changes our code |
|---|---|---|
| R3000A is **MIPS I**, 33.8688 MHz, ~30 MIPS, **no FPU (COP1 absent)** | psx-spx, copetti | Our integer-only rule is mandatory, confirmed. |
| **`mult`/`div` EXIST in hardware** (HI/LO regs), but `div` ≈ 35 cycles, non-interlocked | MIPS I ISA | We are NOT divide-free by necessity — we're divide-free for *speed*. Keep divides out of per-vertex/per-pixel loops, but they're fine in setup. |
| **Load delay slot**: the instruction *after* a load can't use the loaded reg | psx-spx, NGEmu | Compiler handles it, but tight pointer-chasing hot loops stall. Favors flat arrays over pointer chains (we already use index handles — good). |
| **Branch delay slot**, 5-stage pipeline ~1 IPC | R3000 refs | Minimize branches in inner loops; the GTE path is mostly branchless already. |
| **4 KB I-cache, 1 KB D-cache**, + **1 KB scratchpad** (fast on-chip RAM at 0x1F800000) | psx.pdf, PSn00bSDK | Hot data (sine table slice, OT, working verts) wants to fit cache; scratchpad is the single biggest lever for the GTE loop. |
| **GTE vertex input is 16-bit** (SX/SY/SZ); MAC are 44-bit, IR are 16-bit saturating | psx-spx GTE | Confirms the two-space split. Coordinates fed to RTPS/RTPT MUST be pre-scaled into int16. |
| GTE **divide (perspective) saturates / sets FLAG on overflow**; H is 16-bit unsigned with a read bug | psx-spx GTE | Near-plane / tiny-Z faces overflow the perspective divide — must clamp, exactly the case a CAD zoom hits. |
| GTE **RTPS ≈ 15 cyc, RTPT ≈ 23 cyc** (3 verts) | grokipedia/psx-spx | RTPT is ~5 cyc/vertex cheaper than 3× RTPS. Always batch triangles, never transform single verts in a loop except the 4th of a quad. |
| Memory mirrors: KUSEG `0x00..`, KSEG0 cached `0x80..`, KSEG1 uncached `0xA0..` | psx.pdf | DMA/GPU writes should target uncached or be cache-flushed; ordering-table build is cached. |

The headline: **our architecture is sound, but the render/render-prep code as written is naïve about the GTE and the scratchpad, and a few foundation pieces fight the cache.** Fixes below.

---

## 1. CRITICAL — the projection-prep pass is wrong for the GTE

**Current plan (DESIGN §3, render.c TODO):** a software "model→render" pass scales 32-bit
myriometers into 16-bit with a bit-shift, *then* feeds the GTE.

**Problem:** this does the scale on the R3000 (slow, per-vertex integer ops + a branch to clip)
and then *also* pays the GTE transform. That's double work. The GTE's `TRX/TRY/TRZ` translation
vector and the rotation matrix already do scale+rotate+translate in one `RTPT`. The right design:

- Bake the **model→render scale into the GTE rotation matrix** (multiply the rotation 1.12 matrix
  by an integer zoom factor, still 1.12 — the matrix entries are already `int16`-range). The GTE
  applies it for free during RTPT.
- Keep model coords as `int16`-clamped **at load time** with `gte_ldv3`, not via a separate pass.
  The only software step is choosing per-frame the matrix scale and the camera translation.

**Net:** the per-vertex software cost drops to ~zero; the GTE does all of it. This is the single
biggest speedup and it deletes code rather than adding it.

> Action: remove the separate scale/clip loop from the render-prep design. `camera.c` computes a
> scaled rotation matrix + translation each frame; `render.c` only loads verts and issues GTE ops.

---

## 2. CRITICAL — perspective-divide overflow on CAD zoom

A CAD user zooms *into* a face until it fills the screen. That drives projected Z toward the near
plane, and the GTE perspective divide (`H_SF / SZ`) overflows, setting the FLAG register and
saturating — producing the classic vertex "explosion." Games rarely hit this; a measurement tool
hits it constantly.

> Action: after `gte_rtpt`, the render loop must **check the GTE FLAG** (or pre-clamp SZ to a
> minimum) and fall back to wireframe / skip the primitive for that face. PSn00bSDK exposes the
> flag; the demo's `(p>>2) > OT_LEN` test already discards far faces — we add a near-clamp twin.
> This also dovetails with the existing wireframe-on-move mode: during a zoom gesture we're in
> wireframe anyway, so the worst overflow window is already masked.

---

## 3. HIGH — put the hot loop's data in the 1 KB scratchpad

The scratchpad (`0x1F800000`, 1 KB, single-cycle, never cache-misses) is the PS1's secret weapon
and our plan doesn't use it at all. The per-frame GTE loop touches: the current working vertices,
the active OT bucket pointers, and the primitive write cursor.

> Action: place the **per-face working vertex scratch** and the **primitive-packet write cursor**
> in scratchpad (PSn00bSDK lets you address it). Keep the big pools in main RAM. This removes
> D-cache pressure from the tightest loop. Measure with/without — typically a meaningful win on
> geometry-bound scenes, which ours is.

---

## 4. HIGH — foundation code that fights the hardware

### 4.1 `fx_div`/`mym_div_round` in hot paths
`div` is ~35 cycles and stalls. We use `mym_div_round` inside `sk_line_line`, `sk_line_circle`,
and `ray_plane_dist`. Those are **setup-time** (sketch edit, end-condition resolve), not
per-frame — so they're fine. But the **circle tessellation** in `profile_to_ring` does
`((mym2_t)i * SIN_LEN) / n` per segment: that's a divide in a geometry-build loop.

> Action: precompute the per-segment angle step once (`SIN_LEN / n`) and **add** it each iter
> instead of multiply-then-divide per i. One divide total instead of N.

### 4.2 The sine table build (`fixed.c`, target path)
The Bhaskara integer build does a 64-bit `mult` + `div` per entry × 4096 = 4096 divides at boot.
Acceptable once at startup, but the `mym2_t` (`int64`) math is emulated on the 32-bit R3000 via
runtime helper calls (`__divdi3` etc.) — slow and pulls in libgcc.

> Action: build the quarter-wave (1024 entries) only and mirror/negate for the other three
> quadrants (sin symmetry), cutting the work 4×. Better: **bake the table as a `const` array in
> ROM** (generate it host-side, `#include` it) so boot does zero math and the table lands in the
> read-only segment. Removes the `int64` divide path from the target binary entirely.

### 4.3 `int64` (`mym2_t`) usage
Every `mym2_t` multiply/divide on R3000 is a libgcc soft-routine. We use it correctly for
overflow safety in cross products and matrix math, but it's costly. For the **GTE-bound** math we
shouldn't be doing 64-bit at all — the GTE's MAC accumulators are 44-bit in hardware. So:

> Action: cross products / dot products that feed *rendering* should go through the GTE (`gte_op`
> MVMVA / NCLIP) not the C `v3_cross`. Reserve the C `int64` path for *kernel* geometry
> (Euler checks, plane equations, save) where it runs at setup time, not per frame.

---

## 5. MEDIUM — ordering table + DMA specifics

The current `render.c` is structurally right (double-buffered OT, `ClearOTagR`, `DrawOTag`) but:

- **OT depth scaling matches the SDK idiom** — the verified example uses `(otz >> 2)` into a
  1024-entry OT. Keep `>>2`; our DESIGN §6.1 nested-OT is an *enhancement*, not needed for v1.
  Recommend: **ship the single 1024-OT first**, add nesting only if depth-banding shows artifacts.
- **`AVSZ4`/`AVSZ3` give OTZ directly from the GTE** — we should use them (the example does) rather
  than averaging Z in C. Our DESIGN said this; the code TODO should call it out explicitly.
- **Coplanar tie-break** (DESIGN §6.1 item 4): doing `otz += (fid & 3) - 1` perturbs the bucket by
  ±1. Fine, but note it must happen *after* the `>>2` scale or it's lost in the shift. Document the
  ordering.
- **DMA chain ownership**: build the primitive packets contiguously (a bump pointer in the pri
  buffer) so the OT's linked list stays cache-friendly. Our `nextpri` cursor does this — good.

---

## 6. MEDIUM — boot path & memory layout

### 6.1 The static pools are too big for some segments as written
`main.c` declares `s_hedges[8192]` (≈ `8192 * sizeof(HalfEdge)`). With the current `HalfEdge`
(5 × `uint16` ≈ 12 bytes after padding) that's ~96 KB — fine. But `s_verts[2048]` of `Vertex`
(`Vec3i` = 12 bytes) is fine too. **The trap is `.bss` placement**: these are zero-init globals,
which PSn00bSDK clears at boot. Large `.bss` is free in ROM but costs boot time clearing.

> Action: confirm total `.bss` < ~512 KB (model arena budget). Keep the pool backing as `.bss`
> (zeroed once) rather than runtime `malloc` — matches our no-heap rule and the BIOS does the clear.

### 6.2 `main()` loop has no `VSync`/`DrawSync` pacing in the kernel logic
`render_end` calls `DrawSync(0); VSync(0)` — correct. But ensure the **input poll and regen happen
during the GPU's draw**, not serialized after `VSync`. Classic PS1 frame structure:
```
for(;;){
   poll_pad();                 // cheap
   update_camera_matrix();     // a few GTE/CPU ops
   build_ot(active);           // GTE transforms + OT inserts  <-- bulk of frame
   DrawSync(0); VSync(0);      // wait for prev frame's GPU + vblank
   swap_buffers();
   DrawOTag(prev);             // kick GPU on the just-built frame
}
```
> Action: restructure the loop so OT-build for frame N overlaps GPU draw of frame N−1
> (double buffer the pri/OT, which we already allocate `[2]`). This is the difference between
> 30 fps and 20.

### 6.3 `.exe` region string
PSn00bSDK's `elf2x` had a region-string quirk affecting DuckStation autodetection (fixed in
recent SDK). Pin a known-good SDK version in the toolchain file to avoid boot-detection surprises.

---

## 7. LOW — correctness/robustness nits found in review

- **`brep_face_plane` in brep.c** has a dead, convoluted `h0` computation (a leftover) before the
  real plane calc. Harmless but should be deleted; it also dereferences through `BREP_NONE` paths
  that only survive because the result is `(void)`-cast. Clean it.
- **`brep_check_euler`** counts `b->edges.count` as full edges, but the cut path currently leaves
  some half-edges unpaired (boundary edges), so `edges.count` undercounts. The check accepts
  χ∈{0,2}; tighten once the cut shell-merge lands so it asserts the real expected genus.
- **`solid_extent` loops over `pool->capacity`** probing every slot via `pool_get`; for 2048 verts
  that's fine at setup, but it reads freed slots too. Walk live count or track a running AABB at
  vertex-add instead. Cheap fix, avoids reading stale slots.
- **`-Wconversion` is clean today** — keep it. It's the guardrail that catches an accidental
  `int`/`int16` GTE-feed mismatch, which is the bug class most likely to corrupt geometry silently.

---

## 8. Recommended structural changes (summary)

1. **Delete the software scale/clip pass.** Fold model→render scale into the GTE matrix (`camera.c`).
   *(biggest win, removes code)*
2. **Bake the sine table as a ROM `const`** generated host-side; drop the boot-time `int64` divides.
3. **Use scratchpad** for the per-face working verts + pri cursor.
4. **Route render-time vector math through the GTE** (MVMVA/NCLIP/AVSZ), keep C `int64` math for
   setup-time kernel ops only.
5. **Add a near-Z clamp + FLAG check** after RTPT; lean on wireframe-on-move to mask zoom overflow.
6. **Restructure the frame loop** so OT-build overlaps GPU draw (true double buffering).
7. **Ship one flat 1024-OT first**; add nested OT only if artifacts demand it.
8. Clean the three nits in §7.

None of these touch the host-testable kernel (foundation/kernel/model stay portable and pass the
54 tests). They're confined to `render.c`, `camera.c`, `input.c`, `main.c`, and the `fixed.c`
target path — exactly the PS1-only layer, which is the right blast radius.

---

## 9. What the plan already gets RIGHT (don't change)

- Integer myriometers + 16-bit pool handles — cache-friendly, GTE-aligned, correct call.
- Feature-tree-as-file + regen-into-arena — no per-frame allocation, matches no-heap.
- Snapshot undo over the compact codec — tiny RAM, robust.
- Host/target dual build of the kernel — the reason we can unit-test at all; integer math makes it
  bit-identical, which §4.4 (GTE for render math) carefully preserves by keeping GTE use out of the
  portable kernel.
- Wireframe-on-move — turns out to also be the mitigation for §2 (perspective overflow). Nice
  accidental synergy; keep it.
