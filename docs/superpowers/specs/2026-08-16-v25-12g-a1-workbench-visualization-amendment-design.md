# V25-12G-A1 Workbench Visualization Amendment

Date: 2026-08-16

Branch:

```text
feat/v25-12g-maximum-feasible-coverage
```

Status:

```text
DESIGN APPROVED IN DISCUSSION / WRITTEN AMENDMENT FROZEN / IMPLEMENTATION PENDING
```

## 1. Purpose

This amendment resolves one mismatch discovered before Task 5 implementation

The frozen A1 data contract gives active `VehicleFeasibleSegment` records explicit `centerline_xyz` geometry, while `RejectedFeasibleFragment` intentionally stores only longitudinal diagnostics:

```text
fragment_id
aisle_id
start_distance_m
end_distance_m
length_m
reason
```

Therefore the Workbench can render active segments in real map coordinates from the A1 asset alone, but it cannot reconstruct rejected-fragment XY/XYZ geometry without additional truth sources

This amendment preserves the already-tested A1 schema instead of inventing fragment geometry in the UI

If this document conflicts with Section 21 of `2026-08-15-v25-12g-maximum-feasible-coverage-design.md` or the original Task 5 wording in `2026-08-15-v25-12g-a1-vehicle-feasible-segments.md`, this amendment governs Task 5 visualization behavior

## 2. Frozen choice

Use the review-only **active-segment spatial visualization + rejected-fragment non-spatial diagnostics** design

The A1 schema remains unchanged

No geometry fields are added to `RejectedFeasibleFragment`

The Workbench must not load `aisle_graph.yaml` merely to reconstruct short-fragment geometry

The Workbench must not call `derive_vehicle_feasible_segment_plan`

The Workbench must not synthesize, interpolate, or guess XY positions for rejected fragments

## 3. Renderer contract

Freeze the renderer API as:

```text
VehicleFeasibleSegmentPreview(scene)
set_plan(plan-or-None)
set_visible(bool)
clear()
```

The renderer owns only graphics created from explicit geometry already present in `VehicleFeasibleSegmentPlan`

For every active segment:

```text
centerline_xyz -> QPainterPath in scene coordinates (x, -y)
```

Each active segment receives one non-editable `QGraphicsPathItem`

All A1 graphics must have both of these flags disabled:

```text
ItemIsMovable
ItemIsSelectable
```

The renderer must keep deterministic ownership of all created graphics so `clear()` removes every prior item from the scene

A fixed review z-order must place active-segment graphics above raster navigation overlays and below authoring handles

## 4. Rejected-fragment diagnostics

Rejected fragments remain visible to the operator as diagnostics, but not as map geometry

The Workbench may expose diagnostic state such as:

```text
rejected fragment count
rejected fragment total length
per-aisle rejected fragment count
fragment IDs / lengths when useful in status text
```

Task 5 does not require a `QGraphicsItem` for a rejected fragment because the frozen A1 asset contains no spatial geometry for it

This is intentional fail-honest behavior, not a loss of safety evidence

A future schema version may add rejected-fragment geometry only through a separately reviewed data-contract change

## 5. Workbench layer lifecycle

Add the navigation layer key:

```text
vehicle_feasible_segments
```

with user-facing label:

```text
12G-A1 Vehicle-Feasible Segments
```

On source/PCD change, the Workbench must:

```text
1. hide and clear the previous A1 preview
2. clear the previous loaded plan and A1 load error
3. look only for sibling vehicle_feasible_segments.yaml
4. load it with load_vehicle_feasible_segment_plan when present
```

Missing sibling YAML means the layer is unavailable with no exception

Invalid sibling YAML records a local A1 load error, leaves the A1 layer unavailable, and must not break legacy navigation, Site Boundary, 12F, 3D review, or Route Debug behavior

No A1 Workbench load path may mutate or overwrite the source YAML

## 6. Visibility semantics

When `vehicle_feasible_segments` is selected:

```text
raster NavigationPreviewItem -> hidden / cleared for this dispatch
A1 preview -> visible only when nav overlay visibility is enabled and a valid plan exists
```

When any other navigation layer is selected:

```text
A1 preview -> hidden
existing navigation overlay dispatch -> unchanged
```

Changing source must clear stale A1 graphics before any new sibling asset is accepted

## 7. Diagnostics surface

The first implementation should keep diagnostics narrow

Minimum state exposed by `VehicleFeasibleSegmentPreview` or `ReviewMapWorkbenchWindow` must be testable as:

```text
active graphics count
rejected fragment count
rejected fragment total length
```

No dedicated editor, table, modal, or new authoring workflow is required in Task 5

A status-bar or small local status string is sufficient if operator-facing text is added

## 8. TDD acceptance cases

Pure renderer tests must cover:

```text
plan with two active segments -> exactly two active path items
map coordinates use (x, -y)
all active graphics are non-movable and non-selectable
set_visible(False) hides all owned graphics
clear() removes all owned graphics from the scene
one or more rejected fragments -> diagnostic counts/lengths preserved without creating fake map graphics
```

Workbench integration tests must cover:

```text
missing sibling YAML -> no exception and no A1 graphics
valid sibling YAML -> plan loads and active segments render
invalid sibling YAML -> A1 unavailable while legacy layers still dispatch
source change -> previous A1 graphics cleared
switching to another nav layer -> A1 graphics hidden
switching back with nav overlay enabled -> A1 graphics visible
all A1 graphics remain non-editable/non-draggable
```

## 9. Non-goals

Task 5 explicitly does not add:

```text
route editing
segment editing
fragment editing
fragment geometry reconstruction
map mutation
A1 derivation inside Workbench
A2 reachability
A3 connectors
A4 optimizer behavior
```

The visualization remains review-only and diagnostic-only

## 10. Implementation boundary

Task 5 may change only the Workbench visualization/integration surface and its tests/CMake registration

The already-frozen A1 extraction, serialization, acceptance harness, and `vehicle_feasible_segments.yaml` schema must remain unchanged
