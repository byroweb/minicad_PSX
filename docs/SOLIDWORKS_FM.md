# MiniCAD-PSX FeatureManager Design Tree — Layout Spec

Build spec for an on-console left-side panel that mimics the SolidWorks
**FeatureManager design tree (FM tree)**, rendered with the PS1 SDK 8px
monospace bitmap font at 320x240. This is implementation-ready; do not treat
it as prose.

---

## 1. SolidWorks reference (what we are copying)

### 1.1 Panel anatomy, top to bottom (part document)

The FM tree is a single scrollable vertical list, rooted at the part name.
Default order for a fresh part:

1. **Part name (root)** — e.g. `Part1`. Always row 0. Shows config/display
   state. Turns **red** when the part has a rebuild error anywhere below.
2. **History** (recent-features folder; optional, often collapsed).
3. **Sensors** (folder).
4. **Annotations** (folder).
5. **Surface Bodies** / **Solid Bodies** (folders; shown when bodies exist).
6. **Material** — e.g. `Material <not specified>`.
7. **Default reference planes**, in this order:
   - **Front Plane**
   - **Top Plane**
   - **Right Plane**
8. **Origin**.
9. **Feature history** — the ordered creation list: `Sketch1`,
   `Boss-Extrude1`, `Sketch2`, `Cut-Extrude1`, fillets, etc. Sketches are
   children nested **under** the feature that consumed them (expand the
   feature to reveal its sketch).
10. **Rollback bar** — a horizontal line that sits *between* feature rows.
    Everything below it is rolled back (temporarily suppressed). It rests at
    the very bottom by default.

### 1.2 Row format

`[triangle] [icon] [name]`

- **Expand/collapse triangle**: only on rows with children. Right-pointing =
  collapsed, down-pointing = expanded.
- **Icon**: per-type glyph (plane, sketch, extrude boss, extrude cut, origin,
  folder, material…).
- **Name**: editable text label.
- **Indentation**: each depth level indents the row by a fixed step; the
  triangle sits in the indent gutter to the left of the icon.

### 1.3 State indication (color/highlight)

| State        | SolidWorks convention                                            |
|--------------|-----------------------------------------------------------------|
| Normal       | Black text on light (near-white) background.                     |
| Selected     | Row highlighted (blue selection band), text inverted/white.      |
| Suppressed   | Text **grayed out** (light gray); item ignored by the model.     |
| Error        | Item + root part name **red**, red circle-X overlay on icon.     |
| Warning      | **Yellow** triangle/exclamation overlay on icon.                 |
| Rolled back  | Rows below the rollback bar are dimmed (treated as suppressed).  |

### 1.4 Sizing / fonts / colors (desktop)

- **Panel width**: ~15-25% of window, user-draggable (commonly ~250-300px on
  a 1080p+ window). It is the dominant left dock.
- **Row height**: ~18-20px (one text line plus padding); ~16px icons.
- **Font**: system UI font ~8-9pt, single line per row.
- **Colors**: white/very-light-gray background; black text; blue selection
  highlight; gray = suppressed; red = error; yellow = warning.

---

## 2. Mapping to MiniCAD-PSX constraints

### 2.1 Screen + font budget

- Framebuffer: **320 x 240**.
- Font: **8px monospace** → 40 cols x ~30 rows max.
- Left panel: **120px wide** → **15 chars** wide, **~28 rows** tall.

### 2.2 Panel rectangle and grid (exact pixels)

| Element                 | x   | y   | w   | h   | Notes                          |
|-------------------------|-----|-----|-----|-----|--------------------------------|
| Panel background        | 0   | 0   | 120 | 240 | Full-height left dock.         |
| Panel border (right)    | 119 | 0   | 1   | 240 | 1px divider vs. 3D viewport.   |
| Title bar (optional)    | 0   | 0   | 120 | 10  | "FeatureManager" / part name.  |
| Tree rows region        | 2   | 12  | 116 | 216 | Scrollable list area.          |
| Scroll indicator        | 116 | 12  | 2   | 216 | Optional thumb.                |
| Rollback bar            | 2   | var | 116 | 1   | Drawn between two rows.         |

- **Row pitch (height)**: **10px** (8px glyph + 2px lead). 216 / 10 ≈ **21
  visible rows**; scroll for more. (Use 9px pitch for 24 rows if denser is
  wanted.)
- **Indent per depth level**: **6px** (helps the 15-char width survive deep
  nesting; ~3 levels max before names truncate).
- **Per-row layout** (relative to row x0 = 2 + depth*6):
  - col 0: triangle glyph (1 char, 8px) — blank if no children.
  - col 1: type icon (1 char, 8px).
  - col 2..end: name, truncated with no ellipsis (hard clip at x=118).
- Net name budget at depth 0: 15 - 2 (triangle+icon) = **13 chars**; depth 1:
  ~12; depth 2: ~11.

### 2.3 Colors (RGB, pick from our palette)

| Use                 | Suggested RGB        |
|---------------------|----------------------|
| Panel background    | 200,200,200 (light)  |
| Row text (normal)   | 0,0,0                |
| Selection band fill | 40,80,200 (blue)     |
| Selected text       | 255,255,255          |
| Suppressed text     | 130,130,130 (gray)   |
| Error text          | 200,0,0 (red)        |
| Warning glyph       | 220,180,0 (yellow)   |
| Rollback bar        | 40,80,200 (blue)     |
| Divider/border      | 100,100,100          |

Selection is a filled rectangle `(0, rowY, 120, 10)` drawn before the text,
with text switched to white.

### 2.4 ASCII icon set (text-only font)

We have only a monospace font, so fake every glyph with one ASCII char.

| Item            | Triangle (children) | Icon char | Rationale                 |
|-----------------|---------------------|-----------|---------------------------|
| Collapsed node  | `+`                 | —         | "+" = can expand.         |
| Expanded node   | `-`                 | —         | "-" = can collapse.       |
| Leaf (no kids)  | ` ` (space)         | —         | aligns icon column.       |
| Part root       | `+`/`-`             | `@`       | the document.             |
| Folder          | `+`/`-`             | `/`       | Sensors/Annotations/etc.  |
| Material        | ` `                 | `=`       | layered/solid hint.       |
| Plane           | ` `                 | `:`       | flat reference.           |
| Origin          | ` `                 | `*`       | the datum point.          |
| Sketch          | `+`/`-`             | `#`       | grid of sketch lines.     |
| Boss-Extrude    | ` `/`+`             | `O`       | added solid.              |
| Cut-Extrude     | ` `/`+`             | `o`       | removed solid (lowercase).|
| Fillet/Chamfer  | ` `                 | `(`       | rounded edge.             |
| Error overlay   | (text in red)       | `!`       | replace icon w/ `!`.      |
| Warning overlay | (text in yellow)    | `?`       | replace icon w/ `?`.      |

Triangle and icon occupy **two distinct character cells** so columns stay
aligned. Suppressed rows keep their icon but render in gray; the rollback bar
is a horizontal line of `-` or a solid 1px rule across the row gutter.

### 2.5 ASCII mockup (demo part)

Demo part: `Part1` → `Sketch1`, `Boss-Extrude1`, `Sketch2`, `Cut-Extrude1`.
`Boss-Extrude1` is expanded to show its consumed `Sketch1`; `Cut-Extrude1` is
selected. 15-char-wide content area shown between the `|` rails:

```
|FeatureManager |   <- title bar (y=0..9)
|---------------|
|-@ Part1       |   <- root, expanded
| / Sensors     |
| / Annotations |
| = Material<no>|   <- "Material<not spec>" clipped to 13 chars
| : Front Plane |
| : Top Plane   |
| : Right Plane |
| * Origin      |
|-O Boss-Extru1 |   <- expanded boss; "Boss-Extrude1" clipped
|  # Sketch1    |   <-   consumed sketch (depth 2, indent 12px)
| # Sketch2     |
|#o Cut-Extrud1 |   <- SELECTED: blue band, white text, "Cut-Extrude1" clip
|---------------|   <- rollback bar (rests at bottom)
```

Notes on the mockup:
- Indent: depth 0 names start at x=2; depth 1 (root's children) at x=8;
  depth 2 (sketch under boss) at x=14. The triangle for the root sits at x=2.
- Long names hard-clip at the right rail (`Boss-Extrude1` -> `Boss-Extru1`,
  `Cut-Extrude1` -> `Cut-Extrud1`, `Material <not specified>` ->
  `Material<no>`). No ellipsis; just truncate.
- A suppressed `Sketch2` would render `# Sketch2` in gray (130,130,130).
- An errored `Cut-Extrude1` would render its row text in red with `!` in the
  icon cell instead of `o`.

### 2.6 Navigation / interaction (D-pad mapping, for implementer)

- Up/Down: move selection by one visible row (scroll when off-region).
- Cross/X: toggle expand/collapse on a node with children; else select.
- Triangle/△: move rollback bar up one feature; Circle/○: move it down.
- Selection drives the blue band + (later) highlight in the 3D viewport.

---

## 3. Minimal data model the renderer needs (informative)

Each row: `{ depth:u8, type:enum, name:char[16], state:enum,
hasChildren:bool, expanded:bool }`. The tree is flattened to a visible-row
array each frame (respecting collapse + rollback), then drawn top-down at
pitch 10px. Rollback index marks where the bar is drawn and below which rows
are dimmed.

---

## Sources

- https://help.solidworks.com/2026/English/SolidWorks/sldworks/c_featuremanager_design_tree_overview.htm
- https://help.solidworks.com/2023/English/SolidWorks/sldworks/c_featuremanager_design_tree.htm
- https://hawkridgesys.com/blog/solidworks-rollback-bar-just-price-cutter
- https://hawkridgesys.com/blog/whats-new-solidworks-2018-color-coded-folders-featuremanager-design-tree
- https://help.solidworks.com/2024/English/SWConnected/swdotworks/c_folder_icons_featuremanager.htm
- https://help.solidworks.com/2017/english/solidworks/sldworks/hidd_new_whats_wrong.htm
- https://help.solidworks.com/2021/english/SolidWorks/sldworks/c_Suppress_and_Unsuppress_Features.htm
- https://www.engineersrule.com/solidworks-featuremanager-management/
