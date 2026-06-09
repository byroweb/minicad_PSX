# MiniCAD-PSX — Master Design & Prompting Document

> A parametric 3D solid modeller that runs as a **PlayStation 1 executable**.
> SolidWorks-style interface + FeatureManager tree, DualShock input, memory-card saves.
> Built with **PSn00bSDK** (open-source PSX SDK; PsyQ-compatible API).
>
> This document is the single source of truth for the project. Hand it to Claude Code
> (or any collaborator) alongside the source tree. It captures every decision made so far
> and the rationale behind each, so the *why* is never lost.

---

## 0. Elevator pitch

A feature-based parametric CAD modeller — sketch on planes, extrude/revolve, add reference
geometry — implemented to run on 1994 PlayStation hardware (2 MB RAM, 1 MB VRAM, no FPU).
The **feature tree is the file**: a part is the serialized recipe that regenerates its
geometry, which makes saves tiny (a part is dozens of bytes) and re-tessellatable at any
resolution. The PS1 is the *authoring* environment; a host-side toolchain (same integer
kernel, compiled for PC) handles export to real interchange formats (STEP/OBJ). Interop by
construction.

This is intentionally **not a research problem**. The rendering scheme is the well-trodden
PS1 path (GTE transform + ordering-table painter's algorithm), and the modelling kernel
follows established open-source approaches (think the half-edge B-rep + feature-tree model
seen in FreeCAD/OpenCASCADE, Solvespace's parametric sketcher, and the standard PS1 demos).
Lean on prior art; minimize novel invention.

---

## 1. Hard constraints (the machine dictates the architecture)

| Constraint | Consequence |
|---|---|
| **No FPU** | All math is integer. No floats *anywhere* — not even at boundaries. The GTE (integer fixed-point coprocessor) is the transform engine. Trig comes from an integer sine table. |
| **GTE transforms 16-bit vertex coords** | Two-space split: wide 32-bit **model space** (truth) vs. narrow 16-bit **render space** (fed to GTE after an integer scale/clip pass). |
| **2 MB main RAM total** | Statically-sized arenas, no heap growth, 16-bit pool-index handles (not pointers). A `MemBudget` ceiling fails gracefully. |
| **1 MB VRAM** | Two 320×240 framebuffers + font page; no model textures in MVP (flat/Gouraud only). |
| **No Z-buffer** | Painter's-algorithm via ordering table; per-face Z strategy (§7) + wireframe-on-move fallback. |

### The number system: myriometers
- `1 myriometer (mym) = 0.1 mm = 1/10,000 m`. **This is the native unit.** Minimum resolution = 1 mym.
- Model coordinates are `int32_t` (`mym_t`). Range ±214 km; real cap is the design envelope.
- **"mm" is a display-time string trick, never a float divide.** Format the integer, insert a
  decimal point one digit from the right. `12345 → "1234.5"`, `7 → "0.7"`, `-50 → "-5.0"`.
  (Implementation in `foundation/fixed_str.c`.)
- This was a *preference* on desktop; on PS1 it's mandated by hardware. It is also a
  **testability win**: integer code runs identically on the host, so the kernel is unit-tested off-target.

---

## 2. Layered architecture

```
L5  Input & UI      pad polling, 3D cursor, FeatureManager tree, contextual wheel
L4  App / command   undo-redo (parameter-delta), selection set
L3  Feature / regen feature tree (sketch/extrude/revolve/reference geom), dependency rebuild
L2  Geometry kernel integer half-edge B-rep, extrude/revolve, tessellation, save codec
L1  Foundation      arenas/pools, integer vec/mat, GTE wrappers, fixed->string
       (Renderer sits beside L2-L3, consuming render-space output.)
```

**Dependencies point downward only.** L1–L3 (the kernel + model) compile **twice**:
- MIPS/PSn00bSDK → the console `.exe`
- host GCC → unit tests + the memory-card ripper + future STEP/OBJ exporters

Only `render/` and `input/` are PS1-only. The integer kernel is portable *because* it's integer.

---

## 3. Data model (the feature tree IS the part)

A **Document** owns an ordered list of **Features**. Executing the tree in dependency order
regenerates the B-rep. The save file is the serialized tree; the B-rep is always derived.

Feature kinds (MVP):
- `FEAT_SKETCH` — 2D profile (lines, rectangles, circles, arcs) on a plane
- `FEAT_EXTRUDE` — sketch profile → prism (boss) or cut
- `FEAT_REVOLVE` — sketch profile revolved about an axis (the bearing case; same stitching machinery)
- **Reference geometry** (first-class tree nodes, participate in regen):
  - `FEAT_REF_PLANE` — from 3 points / offset a plane / plane+angle / etc.
  - `FEAT_REF_AXIS` — from 2 planes (intersection) / 2 points / edge
  - `FEAT_REF_POINT` — from line∩plane / 2 lines / vertex / center

Each feature stores: kind, id, `depends_on[]` (upstream feature ids), a dirty flag, its
parameters (integer myriometers), and its result handle. Reference-geometry features produce
datum results that other features consume — exactly the SolidWorks model.

**Regen:** edit a parameter → mark feature + transitive dependents dirty → recompute in
topological order into a fresh arena → swap. Full rebuild (no incremental mutation) keeps the
bug surface tiny and is well within budget for single-part models.

**Assemblies:** explicitly deferred. The data model must not foreclose later mates/drag-drop:
features produce independent solids referenced by id, which is assembly-friendly. Do not build
assembly code yet; just don't paint into a corner.

---

## 4. Geometry kernel
Half-edge B-rep, planar faces with **exact integer plane equations**, 16-bit pool handles.
Topology identical to standard CAD kernels; coordinates are `mym_t`.

- **Sketch primitives:** line, rectangle, circle, arc. Circles store the *ideal* center+radius
  (clean STEP later) but tessellate to integer-snapped points; segment count from a small LUT
  keyed by render-space radius (no sqrt/float).
- **Extrude:** validate closed profile (integer 2D segment-intersection, orientation via integer
  cross-product sign), build caps + side faces, stitch half-edges, **assert Euler V−E+F=2**.
- **Revolve:** same stitching around an axis instead of along a vector; profile rotated through
  N steps (integer sine table), caps closed if not full 360°.
- **No booleans in MVP** (the hole in the demo cube is modeled as a cut-extrude profile, not a
  CSG subtract). Robust integer boolean intersection is a large, deferred effort.

Reference: mirror the structure of established open-source kernels rather than inventing.
The half-edge mechanics, ear-clipping triangulation, and Euler checks are all textbook.

---

## 4b. The sketcher (parametric, hybrid constraint model)

The sketcher (`kernel/sketch.h`, `Sketch2`) is the deepest subsystem. Design follows a
**hybrid** path: useful direct-rule relations now, with a data model shaped so an iterative
solver slots in later without reshaping anything.

**Geometry is split into shared points and entities-by-reference:**
- **SkPoint** `{u, v, fixed}` — the *degrees of freedom*. A future solver perturbs only these.
- **SkEntity** — line/circle/arc/rect that references points *by id*. A line is two point-ids;
  a circle is a center-id + radius. Two lines sharing a corner share **one** point, so dragging
  it moves both — **coincidence is structural and free**, the cheapest, most robust relation.
- **construction flag** per entity — real geometry for referencing/constraining, excluded from
  profile extraction. Trim-to-construction just flips this flag.

**Editing API:** add line / rect / circle (rect auto-creates 4 shared points + 4 lines),
set-construction, and **trim** with a mode: `TRIM_DELETE` (remove, or split between the two
nearest intersections) vs `TRIM_TO_CONSTRUCTION` (flip the flag). Integer **intersection**
helpers (line-line, line-circle with integer sqrt) back trim and future snapping.

**Constraints — stored as data, resolved in two tiers:**
- **Direct integer rules (resolve now, single pass, no iteration):** coincident (snap/share),
  horizontal, vertical, equal (line length / circle radius), fix (anchor a point). Each is a
  direct assignment; conflicts are flagged, not iterated.
- **Needs the solver (recorded but flagged unsolved):** parallel, perpendicular, tangent,
  dimension. These live in the same `SkConstraint` list now; `sk_resolve_direct()` returns the
  count still unsolved. When the **fixed-point Newton solver** is added later, it consumes this
  exact list — the seam is already in place. (Newton on integer/fixed-point residuals is the one
  place we may need a bounded iterative scheme; it stays confined to the solver module.)

**Sketch on real geometry:** because entities reference points, you can sketch off existing
edges or construction lines by referencing/snapping to their points. Profile extraction
(`sk_extract_profile`) walks only non-construction entities into the boundary ring the 3D ops
consume.

*Integration status:* `Sketch2` is built and unit-tested as a standalone subsystem. Migrating
the feature/ops/save path from the simpler legacy `Sketch` to `Sketch2` (so the modeller uses
the full sketcher end to end) is the next integration step — see README status.

---

## 5. Memory map (static allocation, 2 MB)

```
~256 KB  code + rodata + libpsn00b + sine table + font glyphs
~512 KB  Model DB arena      (B-rep + sketches + features; pools)
~256 KB  Regen scratch arena (rebuilt B-rep, reset each pass)
~256 KB  Ordering tables + primitive packet pool (double-buffered)
~128 KB  Save staging buffer (one full save image)
~ 96 KB  UI / command / undo state + stack
```
Entity caps are fixed and documented as the model-complexity ceiling (e.g. 8192 verts,
16384 half-edges, 2048 faces). Tune against the 512 KB model arena.

VRAM (1 MB): framebuffer 0 + framebuffer 1 (320×240, double-buffered) + 16×16 font page.

---

## 6. Renderer (GTE + ordering table)

Per frame: camera→render transform (integer scale/offset, rotation matrix from sine table) →
`RTPT` transform (3 verts/call) → `NCLIP` backface cull → depth-sort into OT by average Z →
emit flat-shaded triangles + edge lines + UI sprites → DMA OT to GPU → swap buffers.

**Polygon budget:** ≤ ~1500 tris + ~2000 edge-lines/frame for stable 30 fps at 320×240.

### 6.1 Per-face Z strategy (no Z-buffer)
Layered, CAD-tuned (Sony's guidance is *subdivide, don't clip*; nested OTs add resolution):
1. Sort at **triangle** granularity using the GTE's `AVSZ3/AVSZ4` (small prims mis-sort less).
2. Adaptive subdivision of tall/near faces (same tess LUT, keyed on depth-span too).
3. **Nested ordering tables**: coarse OT feeds a fine OT over the populated depth band (CAD
   models cluster in depth — exactly what nested OTs are for).
4. **Deterministic tie-break on coplanar/concentric faces** (offset OT index by face id + normal
   sign). Prevents frame-to-frame flicker between coincident faces — the CAD-specific pain point.
5. **Edges always one bucket in front of their faces** → wireframe outline stays crisp even when
   a face sort is ambiguous; the edge graph carries the dimensional meaning.

Items 1, 4, 5 ship MVP; 2, 3 are tuning once real models stress it.

### 6.2 View modes (toggle)
- **Shaded** (faces + edge overlay) — default
- **Shaded, edges off**
- **Wireframe** — faces suppressed; *zero* painter's ambiguity, ~half the primitive load.
  This is both an aesthetic mode and the **guaranteed-correct fallback**.

### 6.3 Wireframe-while-moving
While the camera orbits/pans/zooms or a drag is active → render **wireframe**; on release →
settle back to the chosen solid mode after a short delay. Standard CAD "fast mode"; also
sidesteps the Z-sort cost during motion. (Demonstrated in the HTML mockup.)

---

## 7. Input — DualShock as a CAD interface

Central problem: precise selection without a mouse. Solved by **constraint-filtered snapping** —
the stick supplies intent; the system snaps to the nearest entity *of the active filter type*.

### Control map
| Control | Action |
|---|---|
| **Left stick** | Move 3D cursor; snaps to nearest entity of active filter |
| **Right stick** | Orbit about centroid (GTE rotation) |
| **Right stick ↕ + R2 held** | Spin the contextual-menu wheel (iOS-picker scroll) |
| **L1 / R1** | Scroll selection **filter** ↑/↓ (vertex→edge→face→profile→datum→loop, wrapping) |
| **L2 / R2** | Zoom out / in |
| **L2+L1 / R2+R1** | Pan (modifier combo) |
| **L3** (press left stick) | Zoom-to-fit + 6-view radial picker (d-pad selects view) |
| **R3** (press right stick) | Recenter orbit pivot on selection |
| **× (Cross)** | **Select** entity under cursor (add to set) |
| **○ (Circle)** | **Deselect** entity under cursor; **hold** = deselect all |
| **△ (Triangle)** | **Execute** highlighted contextual-wheel action |
| **□ (Square)** | Step through candidate entities under cursor (disambiguate ties) |
| **D-pad ↑ / ↓** | **Increment / decrement** the active value (e.g. extrude distance, hole Ø, wheel item) |
| **D-pad ← / →** | Collapse / expand the focused FeatureManager tree node (also: step value magnitude ×10 / ÷10) |
| **Start** | System menu (save to card, load, new) |
| **Select** | Modifier (combos); tap = cycle workspace mode (sketch ↔ model) |

The five selection verbs you specified: filter scroll = **L1/R1**, select = **×**,
deselect = **○**, deselect-all = **hold ○**, execute menu action = **△**.

### Contextual wheel
Fixed top-right panel, vertical rotating wheel (iOS picker). 3–5 context-relevant items;
centered item is highlighted (brightest/largest). Off-center items vertically foreshortened
via an integer scale LUT + brightness ramp (cheap on PS1: sprite-height scaling, no 3D/float).
Rebuilt whenever selection/filter changes, so it's always short. `△` executes the centered item.

### FeatureManager tree (left pane)
SolidWorks-style ordered tree, the canonical part view. Shows datum planes, origin, every
feature in order, with reference-geometry nodes inline (e.g. `Axis1 (ref: Plane1 ∩ Top)`).
Selecting a tree node selects its feature in the viewport and updates the wheel. Tree nodes
collapse/expand (d-pad ←/→). The tree *is* the part definition — editing it edits the file.

---

## 8. Save system — memory card, very compact

**A parametric file is a recipe, not a mesh.** Store the feature tree's parameters; regenerate
geometry on load. A flanged bearing = 5 numbers (~20 bytes), not hundreds of vertices.

- Card = 128 KB = 16 blocks × 8 KB; block 0 = directory, 15 usable. Block = 64 frames × 128 B.
- First block: frame 0 = title, frames 1–3 = icon (16×16 4-bit CLUT wireframe-cube), frames 4+ = data.
- A MiniCAD save almost always fits **one block** (often one frame of payload). Multi-block via
  directory linked-list is the rare safety valve, not the common case.
- **Encoding:** tagged stream `u8 kind | u8 flags | varint params`. Small dims = 1 byte. No padding.
- **Filenames:** `BASLUS-xxxxx` regional convention + model id so the BIOS manager lists it.
- **Checksums:** standard XOR check byte per directory frame; recompute on write.

### Python ripper (`tools/rip_mcad.py`, host)
Reads a raw `.mcr/.mcd/.bin` 131,072-byte card image (DuckStation/PCSX/hardware dumps), finds
MiniCAD saves by product code, follows the block chain, verifies crc32, emits a host `.mcad`.
Because the kernel is host-buildable, the ripper (and future exporters) reuse the *exact* integer
geometry code → a part authored on PS1 is bit-identical on PC and re-exportable to STEP/OBJ.

---

## 9. Build & test

- **PSn00bSDK + CMake toolchain** → PS-EXE; runs in DuckStation / PCSX-Redux / real hardware.
- **Host test target:** L1–L3 with system GCC under ASan/UBSan. UBSan guards integer-overflow
  surface (scaling intermediates, cross-product products promoted to `int64`).
- Tests: extrude/revolve topology (Euler), save round-trip (encode → Python ripper → compare),
  `mym_to_mm_str` (exhaustive over negatives/single-digit/boundaries), host↔console regen determinism diff.
- `-Wall -Wextra -Wconversion -Werror` — `-Wconversion` flags stray float/width bugs.

---

## 10. Build order (milestones)

1. Foundation + `mym` string formatter (host-tested). Sine table.
2. Integer B-rep + extrude + revolve, **host build only**: build the demo cube-with-hole in code,
   assert Euler, dump vertices.
3. Save encode + Python ripper round-trip (host) before any real card I/O.
4. **Boot on PS1:** clear screen, draw the cube via GTE/OT, orbit with right stick. First on-target milestone.
5. Cursor + snapping selection + filter scroll + □ disambiguation.
6. FeatureManager tree pane + contextual wheel + d-pad inc/dec.
7. Sketch tools + extrude/revolve UI + regen — close the parametric loop on-console.
8. Memory-card save/load (multi-block); verify ripper vs. an emulator card dump.
9. Reference geometry features, 6-view picker, undo/redo, wireframe-on-move polish.

Milestones 1–3 need no PlayStation at all.

---

## 11. Open decisions (settle as you reach them)

- Entity caps (verts/half-edges/faces) vs. the 512 KB model arena — set exact numbers.
- OT bucket count + nested-OT split point.
- Circle/revolve tessellation LUT values (chord error vs. polygon budget).
- Contextual-wheel item sets per context; wheel foreshortening LUT values.
- Memory-card product code string scheme.
- Stick dead-zone + response curve breakpoints (tune on hardware).
- d-pad ←/→ secondary behavior: confirm tree-collapse vs. value-magnitude stepping (or both, context-dependent).

---

## 12. Prior-art references (lean on these; minimize novel work)

- **PSn00bSDK** — GPU/GTE/OT APIs, examples (`n00bdemo`), memory-card lib. PsyQ-compatible.
- **nocash PSX specs (psxspx)** — authoritative hardware + memory-card data format.
- **Pikuma "How PS1 Graphics Work"** — painter's algorithm, OT, subdivision approach.
- **Psy-Q ordering-table technote** — nested OT pattern for depth resolution.
- **Solvespace** — compact parametric sketch/constraint model in C.
- **FreeCAD / OpenCASCADE** — feature-tree + B-rep structure (architecture reference, not code).
- **Standard half-edge / ear-clipping references** — textbook topology + triangulation.

The rendering and kernel are deliberately conventional. Novelty is confined to (a) the integer
myriometer discipline and (b) the DualShock CAD interaction model — everything else is prior art.
