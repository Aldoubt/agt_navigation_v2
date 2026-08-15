# AGT Navigation V25-12F — Traversability + Hard Boundary Repair Design

Date: 2026-08-15
Status: DESIGN APPROVED IN CHAT / IMPLEMENTATION NOT STARTED
Scope: greenhouse traversability semantics, site hard-boundary authoring, occlusion-aware aisle recovery, and Route Debug A/B validation

## 1. Purpose

V25-12E Route Debug exposed that the dominant current failure is upstream map semantics rather than insufficient local connector search

The real greenhouse review showed two independent problems:

```text
1. The final Navigation Map is too conservative in vegetation-occluded aisle regions
2. Greenhouse walls / site limits are not represented as an independent hard no-traverse invariant
```

V25-12F repairs the planning-space contract before any further R6B or R7 work

The implementation strategy is:

```text
A first
minimal deterministic engineering repair for immediate greenhouse validation

B architecture
preserve rich traversability semantics so the system can later add stronger perception without another map-contract rewrite
```

No R6B search parameter or admission gate is relaxed in this increment

## 2. Frozen design decisions

### 2.1 Site boundary semantics

`site_boundary` is the inner boundary of the vehicle-permitted site area

It is not a wall centerline

The polygon already absorbs:

```text
wall thickness
unmodeled wall geometry uncertainty
desired wall-side stand-off
```

The planner must not estimate wall width again from this asset

The continuous vehicle footprint must remain strictly inside the polygon

**Touching or crossing the boundary counts as `SITE_BOUNDARY_CONFLICT`**

This fail-closed edge policy is fixed for V25-12F

### 2.2 First boundary is manually authored

The first greenhouse boundary is drawn once in the existing Workbench 2D view and frozen to YAML

Automatic wall extraction is explicitly deferred

### 2.3 Existing agricultural evidence is preserved

The existing Ground Confidence / Robust Slope / Hybrid Row Structure / Corridor Refinement pipeline remains the primary agricultural geometry source

The 3D Review layers have different semantics:

```text
Ground
= navigation.ground_valid evidence

垄中心 / row_centerline
= crop-row center geometry

精炼行道 / aisle_candidate
= terrain-supported refined aisle evidence

行道中心 / aisle_centerline
= selected aisle reference line

Vehicle Corridor / required_envelope_mask
= expected vehicle-width envelope, not observed FREE truth
```

### 2.4 UNKNOWN is never globally freed

V25-12F must never implement:

```text
UNKNOWN -> FREE everywhere
```

Only bounded agricultural aisle gaps may be recovered

## 3. Root semantic separation

The repaired system separates four questions:

```text
A. Does agricultural geometry say an aisle should exist here?
B. Is traversable terrain directly observed here?
C. Is direct terrain evidence missing only across a bounded longitudinal occlusion gap?
D. Is traversal absolutely prohibited by a site or semantic constraint?
```

These questions must remain separately inspectable

## 4. Target B architecture

```text
                           processed.pcd
                                |
              +-----------------+------------------+
              |                                    |
              v                                    v
      Terrain evidence                      Obstacle evidence
      ground_valid                          raw obstacle
      ground_height                         geometry evidence
      ground_confidence                     future hard-structure classes
      robust_slope                                  |
              |                                      |
              +---------------+----------------------+
                              |
                              v
                    Agricultural structure
                    row model / row bands
                    structural aisle geometry
                    refined aisle / centerline
                              |
                              v
                    Expected aisle envelope
                              |
                 +------------+-------------+
                 |                          |
                 v                          v
          OBSERVED_FREE             bounded gap recovery
                 |                          |
                 +------------+-------------+
                              |
                              v
                   Rich Traversability Evidence
                              |
              +---------------+----------------+
              |                                |
              v                                v
       HARD site boundary                 semantic NO_GO
              |                                |
              +---------------+----------------+
                              |
                              v
                    Final planning constraint
                              |
                              v
                   ROS compatibility raster
                   FREE / OCCUPIED / UNKNOWN
```

The rich evidence is authoritative for diagnostics

The ROS trinary raster is a compatibility product

## 5. Internal traversability states

Minimum states:

```text
OBSERVED_FREE
INFERRED_TRAVERSABLE
HARD_BLOCKED
SENSOR_OBSTACLE
UNKNOWN
```

### 5.1 OBSERVED_FREE

Direct terrain support passes the current ground / confidence / slope criteria and is not prohibited by hard constraints

### 5.2 INFERRED_TRAVERSABLE

Direct ground evidence is missing, but a bounded longitudinal gap inside a structurally valid agricultural aisle satisfies the recovery gates in Section 10

Every inferred cell must preserve inference provenance

### 5.3 HARD_BLOCKED

A site-level physical constraint that cannot be overridden by agricultural inference or stronger planning

For the first implementation:

```text
outside / touching site_boundary at footprint level
-> HARD_BLOCKED / SITE_BOUNDARY_CONFLICT
```

Future wall/post masks may join this state without changing the contract

### 5.4 SENSOR_OBSTACLE

Direct obstacle evidence whose physical class may still be uncertain

In the A-first implementation direct sensor obstacle evidence remains blocked

A later B implementation may distinguish vegetation-like soft evidence from permanent hard structure, but that is not part of 12F

### 5.5 UNKNOWN

Insufficient evidence and no admitted structural recovery

UNKNOWN remains conservative

## 6. ROS compatibility mapping

The candidate 12F Navigation Map uses the following explicit mapping:

```text
OBSERVED_FREE          -> FREE
INFERRED_TRAVERSABLE   -> FREE
HARD_BLOCKED           -> OCCUPIED
SENSOR_OBSTACLE        -> OCCUPIED
UNKNOWN                -> UNKNOWN
semantic NO_GO         -> OCCUPIED in compatibility raster
```

`INFERRED_TRAVERSABLE` maps to FREE only after all recovery gates pass and remains separately visible through traversability sidecar evidence

NO_GO remains semantically separate even though the compatibility raster is OCCUPIED

## 7. Site boundary asset

Frozen asset:

```text
site_boundary.yaml
schema agt_site_boundary/v1
```

Required semantic form:

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

Validation:

```text
same frame as Navigation Map
>= 3 unique vertices
finite coordinates
simple polygon
non-zero area
logical closure handled by loader
```

Invalid boundary evidence fails closed

## 8. Boundary is mandatory for the 12F candidate map

A READY `site_boundary.yaml` is required before generating any 12F candidate Navigation Map

```text
missing site_boundary
-> no 12F candidate map
-> Route Debug reports MISSING

invalid site_boundary
-> no 12F candidate map
-> explicit contract error
```

There is no fallback to map bounds and no automatic fake perimeter

This keeps the A-first candidate semantics auditable

## 9. Workbench boundary authoring

Use the existing 2D Workbench map frame

Workflow:

```text
open run / processed PCD
-> Navigation Map authoring view
-> start Site Boundary
-> click permitted-inner-boundary vertices
-> preview polygon
-> finish
-> validate
-> export site_boundary.yaml
```

Controls:

```text
Site Boundary
[开始绘制] [完成] [撤销顶点] [清除]
[导出 site_boundary.yaml]
status: 未定义 / 草稿 / READY / INVALID
```

Visual semantics:

```text
boundary edge          high-contrast solid line
outside permitted area translucent HARD_BLOCKED fill
vertices               visible edit handles while authoring
```

Site Boundary and semantic NO_GO must not use the same visual encoding

## 10. Structural aisle envelope for recovery

A critical V25-12F change is to preserve **structural aisle geometry before ground-valid filtering**

The current corridor code computes a per-row-pair `geometry` mask before applying `safe_base`, but only the safe result is currently retained as `aisle_candidate`

V25-12F must expose the structurally valid geometry union as a separate deterministic mask, conceptually:

```text
aisle_geometric_envelope
```

It contains only row pairs that already pass structural checks such as:

```text
minimum available width
minimum longitudinal overlap
valid row-pair geometry
```

It does **not** require every cell to have direct ground evidence

This is the correct recovery domain

The relationship becomes:

```text
aisle_geometric_envelope
    structural expectation

      AND terrain gates
             |
             v
      aisle_candidate
      current refined safe aisle
```

`vehicle.required_envelope_mask` remains useful for review and footprint context, but it is not the primary truth used to justify recovery

## 11. Bounded occlusion recovery

### 11.1 Recovery direction

Recovery operates along the local aisle direction

It must not perform generic 2D morphological filling that can bridge laterally through crop rows

### 11.2 Required gates

A cell may become `INFERRED_TRAVERSABLE` only when all conditions are true:

```text
1. inside aisle_geometric_envelope
2. inside site_boundary
3. outside semantic NO_GO
4. outside row_structural_band
5. no direct SENSOR_OBSTACLE at that cell
6. local aisle direction is known
7. OBSERVED_FREE support exists on both longitudinal sides of the gap
8. unsupported longitudinal gap <= maximum_inferred_gap_m
9. endpoint terrain heights are mutually compatible with maximum slope / step limits
```

### 11.3 Initial bounded-gap configuration

Freeze an explicit configurable parameter:

```text
maximum_inferred_gap_m
```

Initial default for real-data A/B evaluation:

```text
0.60 m
```

The exact accepted production value may change only through frozen A/B evidence and must be written into derivation metadata

### 11.4 Terrain continuity across an occluded gap

The algorithm must not require slope values inside cells that have no ground observation

Instead it checks the observed support on both sides of the gap

Conceptually:

```text
left OBSERVED_FREE support
right OBSERVED_FREE support
height difference / longitudinal separation <= maximum slope bound
endpoint step consistency <= configured step bound
```

This allows occlusion recovery without inventing an unobserved ground surface arbitrarily

### 11.5 No direct-obstacle bypass

A-first policy:

```text
direct sensor obstacle at candidate cell
-> not recoverable
```

This preserves `aisle_003` as a direct-obstacle probe

### 11.6 Precedence

Fixed precedence for 12F candidate generation:

```text
HARD_BLOCKED
> semantic NO_GO
> SENSOR_OBSTACLE
> INFERRED_TRAVERSABLE
> OBSERVED_FREE
> UNKNOWN
```

Nothing overrides HARD_BLOCKED

## 12. Footprint-level site-boundary enforcement

Rasterization is useful for display and map export, but safety enforcement is footprint-level

For every evaluated vehicle pose:

```text
continuous footprint polygon must be strictly contained by site_boundary
boundary contact counts as conflict
```

This invariant applies to:

```text
Vehicle-Safe Lane validation
connector preview validation
R6B primitive validation
future R7 path validation
```

No planner may bypass it

## 13. Padding policy

12F does not silently change both map padding and vehicle footprint padding at once

Their meanings are kept distinct:

```text
map obstacle padding
= map/source localization and raster uncertainty

vehicle footprint padding
= body/control/localization safety margin
```

The 12F main candidate initially preserves the currently frozen map-padding setting unless a dedicated sensitivity result explicitly selects a different value

A diagnostic comparison must also evaluate the no-map-padding counterfactual using the same vehicle footprint so accidental double-margin can be measured

Required reporting:

```text
requested map padding m
effective map padding cells
vehicle footprint padding m
vehicle-safe-lane summary under each evaluated case
```

A padding change is not promoted merely because it creates more FREE cells

## 14. Frozen output filenames

Initial 12F outputs are fixed as:

```text
site_boundary.yaml
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

The current frozen files remain untouched during acceptance:

```text
navigation_map.yaml
navigation_map.pgm
derivation.yaml
```

Canonical promotion is a later explicit action

## 15. Traversability evidence contract

`traversability_evidence.yaml` stores summary/provenance

`traversability_evidence.npz` stores deterministic grid-aligned masks

Minimum masks:

```text
observed_free_mask
inferred_traversable_mask
hard_blocked_mask
sensor_obstacle_mask
unknown_mask
aisle_geometric_envelope_mask
```

Minimum summary fields:

```text
schema
frame_id
grid resolution / origin / width / height
source Navigation Map identity
source site_boundary identity
maximum_inferred_gap_m
observed_free_cells
inferred_traversable_cells
hard_blocked_cells
sensor_obstacle_cells
unknown_cells
requested/effective map padding
```

Masks use the fixed precedence in Section 11.6

## 16. Candidate Navigation Map derivation

The 12F candidate is derived without overwriting the frozen current map

High-level order:

```text
load current frozen Navigation evidence
load READY site_boundary
load agricultural structure/corridor evidence
build aisle_geometric_envelope
classify OBSERVED_FREE / SENSOR_OBSTACLE / UNKNOWN
run bounded longitudinal recovery
apply NO_GO
apply HARD site boundary with highest precedence
map rich states to FREE/OCCUPIED/UNKNOWN
write navigation_map_12f.* + traversability evidence
```

The candidate map must retain the same frame and compatible grid geometry unless the implementation plan identifies a concrete reason to rebuild the grid from the original PCD

## 17. Route Debug integration

V25-12E Route Debug becomes the main A/B review surface

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
- Aisle Geometric Envelope

SEMANTICS
- Site Boundary
- NO_GO
```

Required operator questions:

```text
Which cells were recovered?
Why were they recovered?
Which cells are hard blocked by the site boundary?
Can any footprint still cross the wall boundary?
Did recovery stay inside structural aisles?
Did crop-row obstacles remain blocked?
```

The Route Debug overlay remains render-only

## 18. Failure reasons

Reserve explicit reasons:

```text
SITE_BOUNDARY_CONFLICT
SEMANTIC_NO_GO_CONFLICT
DIRECT_SENSOR_OBSTACLE_CONFLICT
UNKNOWN_TRAVERSABILITY
STRUCTURAL_WIDTH_BLOCKED
OCCLUSION_RECOVERY_NOT_ADMITTED
```

Reuse an existing equivalent reason when the repository already has one with the same semantics

## 19. Real-data acceptance cases

All operator acceptance uses:

```text
runtime/maps/agt_workbench_run
```

### 19.1 Site boundary round trip

Draw the permitted inner greenhouse perimeter, export, reload, and confirm the same polygon in `map`

### 19.2 Hard wall invariant

Test multiple poses near the perimeter

Acceptance:

```text
any footprint touching/crossing site_boundary
-> SITE_BOUNDARY_CONFLICT
```

This remains true even where the raster below the footprint is otherwise FREE

### 19.3 Occluded aisle recovery

Use regions where the old trinary map is fragmented but agricultural geometry is visibly continuous

Acceptance:

```text
short longitudinal gaps may become INFERRED_TRAVERSABLE
large gaps remain UNKNOWN
no lateral bridge through crop rows
no recovery outside boundary
no recovery through NO_GO
```

### 19.4 `aisle_003`

Must remain a direct-obstacle regression probe

Strong direct obstacle evidence is not freed by structural inference

### 19.5 `aisle_005`

Must remain structurally width-blocked when available width is below the required preview width

Traversability repair cannot override vehicle geometry

### 19.6 `aisle_013`

Use as the padding-dominant probe to determine whether artificial padding blockage decreases without erasing real obstacle evidence

### 19.7 Vehicle-Safe Lane A/B

Run the same canonical MK-mini profile and same lane validator against current vs 12F candidate maps

Report:

```text
READY
PARTIAL
NO_VEHICLE_SAFE_LANE
mean any-lateral footprint-free fraction
per-aisle coverage
```

Required safety condition:

```text
zero site-boundary footprint violations
```

No fixed improvement percentage is claimed before the first real run

### 19.8 Connector regression

Only after candidate-map acceptance, re-evaluate existing connectors without changing R6B search settings

Keep `connector_015` and `connector_017` as positive/negative explainability regressions

## 20. Error handling

```text
missing/invalid site boundary
-> no 12F candidate map

missing agricultural structural geometry
-> no occlusion recovery
-> candidate generation fails rather than globally freeing UNKNOWN

conflicting evidence
-> fixed precedence applies

invalid frame/grid contract
-> fail closed
```

## 21. Explicit non-goals

V25-12F does not:

```text
automatically extract walls from point cloud
introduce a neural semantic segmentation model
rewrite Aisle Graph extraction
rewrite Coverage Ordering
relax structural width gates
increase R6B search budget
implement R7
convert generic UNKNOWN to FREE
use Vehicle Corridor as direct FREE truth
overwrite the canonical current map during acceptance
```

## 22. Code ownership

Recommended bounded modules:

```text
agt_offline_assets/
  site_boundary.py
    schema / validation / continuous footprint containment / rasterization

  traversability.py
    rich states / structural envelope / bounded recovery / fusion / sidecars

  navigation_corridor.py
    expose structurally valid aisle_geometric_envelope evidence

  navigation_map_derivation.py
    compatibility integration only where necessary

agt_map_workbench/
  existing 2D Navigation Map workflow
    site-boundary authoring controls

  Route Debug
    12F A/B layers and provenance inspector
```

Production semantics stay out of Qt widgets

## 23. Testing strategy

### 23.1 Site boundary pure tests

```text
valid polygon
<3 unique vertices rejected
self-intersection rejected
zero-area rejected
frame mismatch rejected
raster outside mask
continuous footprint fully inside accepted
footprint touching edge rejected
footprint crossing edge rejected
```

### 23.2 Structural envelope tests

Verify `aisle_geometric_envelope` exists independently of per-cell ground validity and still respects:

```text
row-pair width
longitudinal overlap
row structural bands
```

### 23.3 Occlusion recovery tests

```text
0.60 m-or-shorter longitudinal supported gap admitted under default config
over-limit gap rejected
lateral cross-row gap rejected
hard-boundary conflict never recovered
NO_GO never recovered
direct sensor obstacle never recovered
terrain endpoint discontinuity rejected
```

### 23.4 Compatibility mapping tests

Lock exactly:

```text
OBSERVED_FREE / INFERRED_TRAVERSABLE -> FREE
HARD_BLOCKED / SENSOR_OBSTACLE / NO_GO -> OCCUPIED
UNKNOWN -> UNKNOWN
```

### 23.5 Workbench tests

Headless Qt tests:

```text
start / finish / undo / clear boundary authoring
invalid polygon cannot export READY
valid polygon export/reload round trip
Route Debug 12F layer availability
```

### 23.6 Real-data smoke

Operator machine only

Record candidate-map summary, Vehicle-Safe Lane A/B, hard-boundary probes, and Route Debug screenshots

Synthetic tests alone cannot justify `REAL-DATA PASS`

## 24. Promotion policy

```text
current frozen Navigation Map
        |
        +--> V25-12F candidate derivation
                 |
                 +--> Route Debug A/B
                 +--> Vehicle-Safe Lane A/B
                 +--> site-boundary footprint probes
                 +--> padding sensitivity review
                 |
                 v
           operator acceptance
                 |
                 v
       explicit canonical promotion
```

Promotion is explicit and reversible

The old map remains available for regression

## 25. Success criteria

V25-12F is successful only when:

```text
1. a vehicle-permitted inner greenhouse boundary can be authored and frozen
2. any footprint touching/crossing that boundary is rejected
3. short vegetation-occluded longitudinal gaps inside structurally valid aisles can be recovered
4. recovery never globally frees UNKNOWN or bridges crop rows
5. direct obstacle and structural-width failures remain conservative
6. rich provenance is inspectable in Route Debug
7. current vs candidate maps can be evaluated with the same vehicle-lane validator
8. R6B remains unchanged until upstream map acceptance
```

Key invariant:

> Agricultural structure may justify bounded recovery of missing terrain evidence, but it may never override a hard site boundary
