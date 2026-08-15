# AGT Navigation V25-12F — Traversability + Hard Boundary Repair Design

Date: 2026-08-15
Status: DESIGN APPROVED IN CHAT / IMPLEMENTATION NOT STARTED
Scope: greenhouse traversability semantics, site hard-boundary authoring, occlusion-aware aisle recovery, and before/after Route Debug validation

## 1. Purpose

V25-12E Route Debug exposed that the current dominant failure is upstream map semantics rather than insufficient local connector search

The real greenhouse review showed two independent problems:

```text
1. The final Navigation Map is too conservative in vegetation-occluded aisle regions
2. Greenhouse walls / site limits are not represented as an independent hard no-traverse invariant
```

The objective of V25-12F is therefore not to make R6B search more permissive

The objective is to repair the planning-space contract so that the system can distinguish:

```text
where the vehicle is expected to be able to drive
where traversability is directly observed
where traversability is inferred through bounded agricultural occlusion recovery
where physical/site constraints make traversal impossible regardless of map evidence
```

The implementation strategy is:

```text
A first: minimal engineering repair for immediate greenhouse validation
B architecture: preserve richer semantics so the implementation can evolve without another map-contract rewrite
```

No R6B gate is relaxed in this increment

## 2. Frozen decisions from operator review

The following decisions are fixed for V25-12F

### 2.1 Site boundary semantics

`site_boundary` represents the inner boundary of the vehicle-permitted site area

It does not represent the wall centerline

The polygon therefore already absorbs:

```text
wall thickness
unmodeled wall geometry uncertainty
desired wall-side safety stand-off
```

The planner must not estimate wall width again from this asset

Conceptual rule:

```text
vehicle footprint must remain inside site_boundary
```

Anything outside the polygon is hard blocked regardless of other traversability evidence

### 2.2 First implementation is manually authored

The first greenhouse hard boundary is authored once in the existing Workbench 2D map view

Automatic wall extraction is explicitly deferred

This is an engineering choice, not a claim that manual annotation is the final research method

### 2.3 Existing agricultural evidence is preserved

The existing Ground Confidence / Robust Slope / Hybrid Row Structure / Refined Aisle pipeline remains useful and must not be replaced by a learning model in this increment

The current `corridor.aisle_candidate` and `corridor.aisle_centerline` remain important agricultural-structure evidence

The 3D Review `Vehicle Corridor` layer is not treated as observed FREE space

It is an expected vehicle envelope around the current aisle centerline and is only supporting geometry

### 2.4 Unknown is not globally promoted to FREE

V25-12F must never implement:

```text
UNKNOWN -> FREE everywhere
```

Occlusion recovery is allowed only inside bounded, structurally supported agricultural aisle regions and must still respect hard blocking evidence

## 3. Root cause and semantic separation

The current pipeline compresses several different questions into one final occupancy decision too early

The repaired architecture separates them

```text
Question A
Does the terrain / agricultural structure support a drivable aisle here?

Question B
Is there direct evidence that the location is traversable?

Question C
Is the location only missing direct ground evidence because of bounded occlusion?

Question D
Is traversal prohibited by a hard physical/site constraint?
```

The system must stop treating these as interchangeable

## 4. Target architecture

The long-term B architecture is:

```text
                           processed.pcd
                                |
              +-----------------+------------------+
              |                                    |
              v                                    v
      Ground / terrain evidence              Physical structure evidence
      - ground_valid                         - raw obstacle
      - ground_height                        - geometry-bad evidence
      - ground_confidence                    - future wall / post classes
      - robust_slope                                  |
              |                                      |
              +---------------+----------------------+
                              |
                              v
                    Agricultural structure
                    - row model
                    - row structural band
                    - refined aisle candidate
                    - aisle centerline
                              |
                              v
                    Expected aisle envelope
                              |
                 +------------+-------------+
                 |                          |
                 v                          v
        Observed traversability      Bounded occlusion recovery
                 |                          |
                 +------------+-------------+
                              |
                              v
                   Rich Traversability Evidence
                              |
              +---------------+----------------+
              |                                |
              v                                v
       Hard site boundary                 Semantic NO_GO
              |                                |
              +---------------+----------------+
                              |
                              v
                    Final planning constraint
                              |
                              v
                  ROS/Nav2 compatibility export
                  FREE / OCCUPIED / UNKNOWN
```

The rich internal representation is authoritative for diagnostics

The ROS three-state raster is a compatibility product, not the only truth owner

## 5. Internal traversability states

V25-12F should introduce explicit internal evidence states instead of relying only on the final trinary raster

Minimum semantic states:

```text
OBSERVED_FREE
INFERRED_TRAVERSABLE
HARD_BLOCKED
SENSOR_OBSTACLE
UNKNOWN
```

### 5.1 OBSERVED_FREE

A location with direct terrain support that passes the current ground / slope criteria and is not prohibited by hard constraints

Typical evidence:

```text
ground_valid
sufficient ground confidence
robust slope within configured limit
no direct hard obstacle conflict
inside site_boundary
outside semantic NO_GO
```

### 5.2 INFERRED_TRAVERSABLE

A location without sufficient direct ground evidence, but with bounded evidence that it is part of a continuous agricultural aisle

This state must always retain provenance showing why it was inferred

It is not equivalent to generic FREE evidence

### 5.3 HARD_BLOCKED

A location forbidden regardless of local terrain confidence

V25-12F hard-block sources include at minimum:

```text
outside site_boundary
future explicit hard-structure mask when available
```

Semantic NO_GO remains a separate business exclusion source even when final compatibility export maps it to occupied/non-drivable

### 5.4 SENSOR_OBSTACLE

Observed obstacle evidence whose exact physical semantics may still be uncertain

This preserves the difference between:

```text
raw sensor obstacle evidence
site-level hard boundary
```

A vegetation return must not silently become equivalent to a greenhouse wall

### 5.5 UNKNOWN

Insufficient evidence and no justified structural recovery

UNKNOWN stays conservative outside the bounded recovery policy

## 6. Site boundary asset

Add a frozen run-directory asset:

```text
site_boundary.yaml
```

Schema:

```text
agt_site_boundary/v1
```

Required fields:

```yaml
schema: agt_site_boundary/v1
frame_id: map
status: READY
boundary_semantics: VEHICLE_PERMITTED_INNER_BOUNDARY
outer_boundary_xy:
  - [x0, y0]
  - [x1, y1]
  - [x2, y2]
  - [x3, y3]
source:
  authoring_mode: WORKBENCH_MANUAL_POLYGON
```

The exact source metadata may be extended, but the semantics above are fixed

Validation rules:

```text
same frame as Navigation Map
at least 3 unique vertices
finite coordinates
simple non-self-intersecting polygon
non-zero area
closed logically by the loader even if the serialized first point is not repeated
```

Invalid site-boundary evidence must fail closed for hard-boundary fusion

## 7. Workbench authoring behavior

The existing 2D Workbench remains the authoring surface

The user workflow is:

```text
open processed PCD / existing run
-> open Navigation Map page
-> start Site Boundary authoring
-> click polygon vertices around the vehicle-permitted inner perimeter
-> preview polygon fill / edge
-> finish polygon
-> validate
-> export site_boundary.yaml
```

The boundary is authored in the same map frame as the Navigation Map and route evidence

### 7.1 UI controls

Add a bounded Site Boundary control group, for example:

```text
Site Boundary
[开始绘制] [完成] [撤销顶点] [清除]
[导出 site_boundary.yaml]
status: 未定义 / 草稿 / READY / INVALID
```

No automatic wall-fit option is required in V25-12F

### 7.2 Visual semantics

Recommended review treatment:

```text
site boundary edge       high-contrast solid line
outside permitted area   translucent hard-block fill
polygon vertices         visible edit handles while authoring
```

The Workbench must visually distinguish site hard boundary from semantic NO_GO

## 8. Agricultural traversability base

V25-12F reuses the existing agricultural structure rather than regenerating idealized lanes

The core supporting evidence remains:

```text
navigation.ground_valid
structure.ground_confidence
structure.robust_slope_deg
corridor.row_structural_band
corridor.aisle_candidate
corridor.aisle_centerline
vehicle.required_envelope_mask
```

Important semantic distinction:

```text
corridor.aisle_candidate
= structurally and terrain-supported refined aisle evidence

vehicle.required_envelope_mask
= desired vehicle-width envelope around the current aisle centerline
```

The vehicle envelope does not by itself prove free space

## 9. Expected aisle envelope

Create a deterministic expected aisle support mask used by occlusion recovery

For the A-first implementation, the expected envelope should be derived from existing aisle geometry rather than introducing a new planner

Recommended construction:

```text
refined aisle candidate
UNION
bounded vehicle envelope around accepted aisle centerline
```

but only inside the valid agricultural pair geometry already produced by the corridor refinement

It must not spill through row structural bands or beyond the site boundary

This expected envelope answers:

> If direct ground evidence were temporarily missing, where does the existing agricultural structure say the aisle should continue?

It does not directly become final FREE

## 10. Bounded occlusion recovery

### 10.1 Purpose

Recover short holes in otherwise continuous greenhouse aisles where vegetation occludes direct ground observation

### 10.2 Required gates

A candidate cell may become `INFERRED_TRAVERSABLE` only when all required conditions hold

```text
1. cell is inside the expected agricultural aisle envelope
2. cell is inside site_boundary
3. cell is not in semantic NO_GO
4. cell is not in explicit hard-structure evidence
5. cell is not in row structural band
6. local aisle direction is known
7. observed traversable support exists before and after the gap along aisle direction
8. the unsupported longitudinal gap is below a bounded configured length
9. local robust slope / neighboring terrain evidence does not indicate a discontinuity
```

The A-first implementation may use a conservative subset of the final B gates if every inferred cell still carries provenance and the test cases lock the behavior

### 10.3 No cross-row bridging

Occlusion recovery must operate primarily along the aisle direction

It must not fill laterally through crop rows merely because two free areas are nearby in Euclidean distance

### 10.4 No wall crossing

Site boundary has higher precedence than all recovery logic

Conceptual priority:

```text
HARD_BLOCKED > NO_GO > SENSOR_OBSTACLE > inferred recovery > observed free
```

The exact conflict ordering for raw vegetation-like sensor obstacles may be tuned after real-data review, but site boundary cannot be overridden

## 11. Hard-boundary fusion

Rasterize `site_boundary` into the Navigation Map grid as a hard-permitted-area mask

Conceptually:

```text
inside_boundary = polygon containment
hard_blocked = not inside_boundary
```

Planning and validation must enforce the boundary at footprint level, not merely center-point level

For every candidate vehicle pose:

```text
all footprint cells / footprint polygon must remain inside site_boundary
```

This applies to:

```text
Vehicle-Safe Lane validation
connector preview footprint validation
R6B bounded primitive validation
future R7 planner validation
```

The boundary is a shared safety invariant

A stronger planner is never allowed to bypass it

## 12. Padding policy in V25-12F

V25-12F must expose map padding and vehicle footprint padding as separate uncertainty margins

The implementation must not silently remove both

The initial engineering goal is to eliminate accidental double-counting

The design keeps these meanings separate:

```text
map obstacle padding
= uncertainty around obstacle-map localization / source discretization

vehicle footprint padding
= body/control/localization margin around the vehicle footprint
```

If both are non-zero, their purpose must be explicit in frozen derivation metadata

A before/after diagnostic must report effective map padding cells and vehicle footprint padding used by the validation stage

## 13. Output assets

V25-12F should preserve the original frozen map and create explicit derived artifacts for comparison before promotion

Recommended first outputs:

```text
site_boundary.yaml
traversability_evidence.yaml
traversability_evidence.npz or deterministic sidecars
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

Exact filenames may be adjusted during implementation planning to fit existing asset conventions

The important rule is:

```text
do not overwrite the current frozen navigation_map.yaml/pgm during initial acceptance
```

Promotion to the canonical Navigation Map happens only after A/B validation

## 14. Traversability evidence contract

The rich evidence artifact should retain counts and provenance sufficient for Route Debug

Minimum summary fields:

```text
frame_id
source_navigation_map
source_site_boundary
observed_free_cells
inferred_traversable_cells
hard_blocked_cells
sensor_obstacle_cells
unknown_cells
recovery_gap_limit_m
site_boundary_area_m2
map_padding_m / effective_padding_cells
vehicle_footprint_padding_m when part of the evaluated profile
```

Per-cell or mask sidecars should distinguish at minimum:

```text
observed_free_mask
inferred_traversable_mask
hard_blocked_mask
sensor_obstacle_mask
unknown_mask
```

Masks must be mutually interpretable and their precedence documented

## 15. Route Debug integration

V25-12E Route Debug becomes the primary A/B verification surface for 12F

Add optional layers:

```text
BASE
- Current Navigation Map
- 12F Candidate Navigation Map

TRAVERSABILITY
- OBSERVED_FREE
- INFERRED_TRAVERSABLE
- HARD_BLOCKED
- SENSOR_OBSTACLE
- UNKNOWN

SEMANTICS
- Site Boundary
- NO_GO
```

A toggle or comparison preset should make it easy to answer:

```text
which cells were recovered
which cells became hard blocked
whether walls are now impossible to cross
whether agricultural aisles became continuous without globally freeing UNKNOWN
```

The Route Debug overlay remains render-only

## 16. Real-data acceptance cases

All acceptance uses the existing greenhouse run directory

```text
runtime/maps/agt_workbench_run
```

### 16.1 Site boundary authoring

Operator can draw the vehicle-permitted inner greenhouse perimeter and export a valid `site_boundary.yaml`

Reloading the Workbench reproduces the same polygon in the same frame

### 16.2 Wall hard-block acceptance

Select at least several candidate poses near the greenhouse wall / perimeter

Acceptance condition:

```text
any footprint that crosses site_boundary is rejected with a hard-boundary reason
```

This must remain true even if the underlying candidate Navigation Map would otherwise show local FREE cells

### 16.3 Occluded aisle recovery

Use visually identified aisle regions where the previous final Navigation Map contained fragmented UNKNOWN/OCCUPIED-like gaps but the 3D agricultural structure showed a continuous aisle

Acceptance requires:

```text
bounded gaps may become INFERRED_TRAVERSABLE
recovery remains inside agricultural aisle support
no lateral bridge through crop rows
no recovery outside site boundary
```

### 16.4 Preserve genuine obstacle probe

`aisle_003` remains a direct-obstacle diagnostic probe

12F must not blindly recover through strong direct obstacle evidence merely because the structural aisle is continuous

### 16.5 Preserve structural width rejection

`aisle_005` remains structurally rejected when its available width is below the required preview width

Traversability recovery must not convert a geometrically too-narrow aisle into a valid vehicle lane

### 16.6 Padding-dominant probe

`aisle_013` is used to inspect whether the repaired map semantics reduce artificial padding-only blockage without erasing real obstacle evidence

### 16.7 Vehicle-safe-lane before/after

Compare the frozen pre-12F and candidate-12F map with the same canonical MK-mini profile and same lane validator

Report:

```text
READY count
PARTIAL count
NO_VEHICLE_SAFE_LANE count
mean any-lateral footprint-free fraction
per-aisle coverage fraction
```

The acceptance target is improvement in structurally valid aisles without any hard-boundary violation

No fixed numeric success threshold is frozen before the first real A/B run

### 16.8 Connector regression

Re-evaluate the existing connector regression cases only after the candidate map is frozen

Required observations:

```text
connector_015 remains explainable as a known solved F/R/F regression or improves without semantic regression
connector_017 remains distinguishable as searched-but-unsolved if it still has no solution
```

The increment does not require changing R6B search parameters

## 17. Failure reasons

Add explicit reasons rather than collapsing all failures into occupancy conflict

At minimum reserve:

```text
SITE_BOUNDARY_CONFLICT
SEMANTIC_NO_GO_CONFLICT
DIRECT_SENSOR_OBSTACLE_CONFLICT
UNKNOWN_TRAVERSABILITY
STRUCTURAL_WIDTH_BLOCKED
OCCLUSION_RECOVERY_NOT_ADMITTED
```

The exact existing naming convention should be reused where an equivalent reason already exists

## 18. Error handling and fail-closed behavior

### 18.1 Missing site boundary

Before 12F is promoted to canonical production semantics:

```text
missing site_boundary
-> candidate 12F hard-boundary layer unavailable
-> no fake automatic boundary
-> Route Debug reports missing asset explicitly
```

### 18.2 Invalid site boundary

```text
invalid schema/frame/polygon
-> hard-boundary candidate map generation fails
-> no candidate map is promoted
```

### 18.3 Missing agricultural structure

Occlusion recovery requires agricultural structure evidence

If required aisle geometry is missing:

```text
no structural recovery
UNKNOWN remains UNKNOWN
```

### 18.4 Conflicting evidence

Hard boundary wins over all recovery evidence

No inferred cell may override a hard block

## 19. Explicit non-goals

V25-12F does not:

```text
automatically extract greenhouse walls from the point cloud
train or introduce a semantic segmentation neural network
rewrite Aisle Graph extraction
rewrite Coverage Ordering
relax structural width gates
increase R6B search budget
implement R7
convert every UNKNOWN cell to FREE
replace the canonical current map before A/B acceptance
use Vehicle Corridor as direct FREE truth
```

## 20. Code ownership and expected modules

The implementation plan should keep responsibilities bounded

Recommended ownership:

```text
agt_offline_assets/
  site_boundary.py
    schema, validation, rasterization

  traversability.py
    rich evidence states, bounded recovery, fusion

  navigation_map_derivation.py
    compatibility export integration only where necessary

agt_map_workbench/
  site_boundary authoring controls in existing 2D workflow
  Route Debug optional 12F evidence layers
```

Do not put production traversability logic in Qt widgets

The GUI authors/loads assets and renders evidence

Pure Python offline modules own the deterministic semantics

## 21. Testing strategy

### 21.1 Pure geometry tests

Test `site_boundary`:

```text
valid polygon
self-intersection rejection
frame mismatch
outside mask rasterization
point/footprint containment
edge-touch policy
```

The footprint-edge policy must be explicitly tested so boundary behavior is deterministic

### 21.2 Occlusion recovery tests

Synthetic row/aisle grids should verify:

```text
short longitudinal ground-evidence gap is recovered
large gap remains UNKNOWN
lateral gap through crop row is not recovered
hard-boundary intersection is never recovered
NO_GO is never recovered
strong direct obstacle remains blocked under the A-first policy
```

### 21.3 Compatibility export tests

Verify rich states map deterministically to final compatibility occupancy values

The mapping policy must be explicit and test-locked

### 21.4 Workbench tests

Headless Qt tests should verify:

```text
site boundary authoring state transitions
vertex undo / clear
export enabled only for valid polygon
reload reproduces polygon
Route Debug layer availability and toggles
```

Do not rely on screenshot pixel tests for semantic correctness

### 21.5 Real-data smoke

Use the operator machine and the existing greenhouse run directory

Record before/after summary metrics and Route Debug screenshots

No `REAL-DATA PASS` claim is allowed from synthetic tests alone

## 22. Promotion policy

V25-12F uses a candidate-map phase

```text
current frozen Navigation Map
        |
        +--> 12F candidate derivation
                 |
                 +--> Route Debug A/B
                 +--> Vehicle-Safe Lane A/B
                 +--> hard-boundary footprint tests
                 |
                 v
          operator acceptance
                 |
                 v
       explicit canonical promotion
```

Promotion must be explicit and reversible

The pre-12F map remains available for regression comparison

## 23. Success criteria

V25-12F is successful when all of the following are true:

```text
1. The operator can freeze a vehicle-permitted inner greenhouse boundary
2. Wall/perimeter crossing is impossible at vehicle-footprint level
3. Short vegetation-occluded gaps inside structurally supported aisles can be recovered without global UNKNOWN relaxation
4. Genuine direct obstacle and structural-width failures remain visible and conservative
5. Rich traversability provenance is inspectable in Route Debug
6. The candidate map can be compared against the old frozen map with the same vehicle-lane validator
7. R6B remains unchanged until the upstream map is accepted
```

The key design invariant is:

> Agricultural structure may justify bounded recovery of missing terrain evidence, but it may never override a hard site boundary
