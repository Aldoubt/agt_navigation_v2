# V25 Unified Map Authoring + Structure-Aware PGM Design

Date: 2026-08-22  
Branch: `refactor/v25-unified-map-authoring`  
Base: `fix/paper1-m154b-v25-map-authority-recovery@5f889c6f0e10d8e77955e25fb624f2a20ccfcda9`

## 1. Status and intent

This design unifies the current V25 Map Workbench map-production flow and the legacy standalone semantic editor into one authoritative authoring workflow.

The change has two coupled goals:

1. remove the second navigation-map authority created by direct raster editing in `semantic_editor_qt5.py`;
2. make the already-derived agricultural row/aisle structure participate in production of a usable Nav2 PGM instead of remaining review-only evidence.

The governing invariant is:

```text
One PCD lineage
  -> One AGT Map Workbench authoring workflow
  -> One immutable Map Revision
  -> One accepted Navigation Map authority
  -> One semantic task bound by hash to that accepted map
```

`agt_map_pipeline` remains a consumer/verifier/evidence producer and must not become a competing formal raster producer.

## 2. Problem statement

### 2.1 Competing map contracts

Today the repository has two practical authoring paths:

```text
A. V25 Map Workbench
PCD -> ground-relative map -> agricultural structure -> Site Boundary -> world-coordinate overrides -> generated/accepted Navigation Map

B. standalone semantic editor
Nav2 YAML/PGM -> direct pixel FREE/OCCUPIED/UNKNOWN painting -> semantic_map.geojson + coverage.yaml
```

The second path can mutate the base raster and then refresh `coverage.yaml.base_map_sha256`. The resulting files are internally consistent but no longer identical to the reviewed Workbench map. This violates the recovered V25 map-authority rule.

### 2.2 Automatic structure does not currently define the formal PGM

The current ground-relative derivation correctly produces conservative `FREE/OCCUPIED/UNKNOWN` cells from local ground, obstacle, slope and step evidence. The agricultural stack then separately derives row support, row structural bands, aisle geometric envelopes, refined aisle candidates and V25-12F traversability evidence.

However, the formal Paper I freeze path still writes the generated map from the ground-relative base result. Therefore:

```text
row / aisle segmentation success
!=
formal PGM uses row / aisle semantics
```

This leaves greenhouse aisles fragmented by UNKNOWN cells caused by occlusion or sparse ground returns, even when agricultural geometry already provides a strong explanation that the region is an aisle.

## 3. Scope

### 3.1 In scope

- integrate semantic authoring into the existing Workbench as one tab/panel;
- reuse the existing GUI-independent `agt_ui_bridge` semantic model, IO and validation logic;
- prohibit formal direct pixel painting of accepted PGM assets;
- add one deterministic structure-aware navigation-map materialization layer;
- classify Ground-only output as evidence, not final formal map authority;
- make row structural bands and valid aisle geometry participate in generated PGM materialization;
- preserve evidence-backed `FORCE_FREE` / `FORCE_OCCUPIED` world-coordinate overrides as the only formal manual raster correction path;
- bind `semantic_map.geojson` and `coverage.yaml` to the accepted map by exact SHA256 identity;
- export one immutable revision containing generated map, accepted map, semantic task, authoring inputs and validation reports;
- add planner-independent map QA before a revision can be considered accepted for Paper I use;
- keep the existing M1.5-4B `V25_MAP_WORKBENCH / BOUND_VERIFIED` ownership contract valid.

### 3.2 Explicit non-goals

- no semantic schema 2.0 redesign in this increment;
- no global `UNKNOWN -> FREE` policy;
- no generic morphological closing across crop rows;
- no direct route-dependent corridor painting into the map;
- no new autonomous headland detector in this increment;
- no planner tuning, route benchmark tuning or RPP tuning until map-authoring acceptance is restored;
- no change that turns V25-12F candidate files into formal authority merely by renaming them.

## 4. Final authoring architecture

```text
raw / cleaned PCD
        |
        v
AGT Map Workbench
        |
        +-- Point-cloud processing Recipe
        |      -> processed.pcd
        |
        +-- Map Frame calibration / canonical PCD
        |
        +-- Ground-relative evidence
        |      -> ground / obstacle / slope / step
        |      -> strict Ground-only occupancy evidence
        |
        +-- Agricultural structure
        |      -> row support
        |      -> row structural band
        |      -> aisle geometric envelope
        |      -> refined aisle / centerline
        |
        +-- Site Boundary
        |
        +-- Structure-aware Navigation Materializer
        |      -> generated/navigation_map.{pgm,yaml}
        |
        +-- Evidence-backed formal overrides
        |      -> accepted/navigation_map.{pgm,yaml}
        |
        +-- Semantic authoring
        |      -> semantic_map.geojson
        |      -> coverage.yaml
        |
        +-- Navigation + semantic QA
               -> immutable Map Revision
```

The standalone semantic editor ceases to be a map-authority tool.

## 5. Data ownership and semantic separation

The following concepts remain separate even when authored in one GUI:

| Asset | Meaning | Formal carrier |
| --- | --- | --- |
| Site Boundary | physical inner boundary the vehicle footprint may enter | `authoring/site_boundary.yaml` |
| Navigation Map | Nav2 FREE/OCCUPIED/UNKNOWN raster | `generated/` and `accepted/` |
| Formal raster override | evidence-backed correction of automatic raster | derivation metadata + accepted replay |
| field boundary | task/coverage region | `semantic/semantic_map.geojson` |
| exclusion / keepout | semantic no-go/task exclusion | `semantic/semantic_map.geojson` |
| row centerline | crop-row semantic geometry | `semantic/semantic_map.geojson` |
| access lane | explicit semantic road | `semantic/semantic_map.geojson` |
| entry pose | task entry pose | `semantic/semantic_map.geojson` |
| work direction | task direction | `semantic/semantic_map.geojson` |
| coverage parameters | task/planning configuration | `semantic/coverage.yaml` |

`Site Boundary` and `field_boundary` are not aliases. A Workbench convenience action may create a field-boundary candidate from the Site Boundary, but the operator must explicitly accept it as a semantic object.

## 6. Structure-aware Navigation Map materialization

### 6.1 New layer

Add a focused offline core, tentatively:

```text
src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py
```

The module receives immutable evidence objects and returns a new structure-aware navigation result. It does not own Qt and does not write files by itself.

Inputs:

```text
NavigationMapResult              # Ground-only conservative evidence
NavigationStructureResult       # row/terrain evidence
CorridorRefinementResult         # row structural band + aisle geometry
SiteBoundary                     # mandatory for formal mode
TraversabilityEvidence optional  # bounded evidence reuse where appropriate
```

Output:

```text
StructureAwareNavigationResult
- occupancy
- observed_free_mask
- structure_inferred_free_mask
- row_structural_blocked_mask
- sensor_obstacle_mask
- site_boundary_blocked_mask
- unresolved_unknown_mask
- provenance / counts
- same grid geometry as the Ground-only input
```

### 6.2 State precedence

Formal materialization uses deterministic precedence. Later states may not weaken earlier hard constraints.

```text
Priority 1: FORCE_OCCUPIED during accepted-map replay
Priority 2: outside Site Boundary -> OCCUPIED
Priority 3: sensor obstacle -> OCCUPIED
Priority 4: row structural band -> OCCUPIED
Priority 5: observed Ground FREE -> FREE
Priority 6: structure-supported aisle FREE -> FREE
Priority 7: otherwise -> UNKNOWN
Priority 8: FORCE_FREE during accepted-map replay, subject to formal override validation policy
```

The generated map contains no manual override replay. Manual overrides only produce the accepted map.

### 6.3 Observed FREE

Observed FREE keeps the current conservative meaning:

```text
ground valid
AND direct ground support >= configured threshold
AND slope <= threshold
AND step <= threshold
AND no occupied evidence
-> OBSERVED_FREE
```

### 6.4 Row structural blocking

`CorridorRefinementResult.row_structural_band` becomes formal structural obstacle evidence.

A cell inside a validated row structural band is OCCUPIED in the generated map unless a future separately reviewed design explicitly changes that rule.

This prevents vegetation sparsity or missing returns from turning a crop row into traversable white space.

### 6.5 Structure-supported aisle FREE

A Ground-only UNKNOWN cell may become `STRUCTURE_INFERRED_FREE` only when all of the following are true:

```text
inside Site Boundary
AND inside aisle_geometric_envelope
AND outside row_structural_band
AND no current sensor obstacle
AND not semantically hard blocked by inputs available to the materializer
AND agricultural pair geometry is already accepted by the existing corridor refinement
```

The materializer must not require direct Ground support for these cells, because that is exactly the greenhouse occlusion failure being addressed.

The agricultural geometry is accepted only after the existing corridor logic has already checked row support, longitudinal overlap and geometric width. This makes structure-supported FREE a semantic/geometric inference, not an absence-of-obstacle heuristic.

### 6.6 UNKNOWN preservation

Cells outside validated agricultural aisle geometry that lack direct evidence remain UNKNOWN.

The implementation must not perform:

```text
all UNKNOWN -> FREE
map-wide flood fill
cross-row closing
free-space inference from image background
```

### 6.7 Relation to V25-12F

V25-12F remains useful evidence, especially its explicit masks and bounded short-gap reasoning. However, this design does not promote the existing `navigation_map_12f.*` DRAFT candidate directly into formal authority.

The new materializer may reuse compatible evidence concepts, but formal generated output must have its own deterministic contract and provenance.

## 7. Formal manual correction

Formal manual map repair remains world-coordinate metadata, never raw pixel mutation.

Allowed formal modes for Paper I:

```text
FORCE_FREE
FORCE_OCCUPIED
```

Each override must carry the current Paper I metadata requirements, including non-empty reason and evidence category.

The accepted map is always reproduced as:

```text
generated structure-aware map
  + ordered formal overrides
  -> accepted map
```

Changing map resolution or re-materializing the grid therefore replays geometry rather than preserving hand-painted pixels.

The generic Workbench may continue to understand non-Paper modes such as `UNKNOWN` or `NO_GO`, but Paper I formal freeze remains restricted.

## 8. Semantic authoring integration

### 8.1 UI placement

The existing Workbench right-side control tabs become:

```text
点云编辑
坐标系标定
导航地图
语义与任务
```

No second top-level semantic editor window is required for the formal workflow.

### 8.2 Reuse existing semantic core

Reuse without duplicating contracts:

```text
agt_ui_bridge.semantic_model
agt_ui_bridge.semantic_io
agt_ui_bridge.semantic_validation
agt_ui_bridge.semantic_scene
agt_ui_bridge.semantic_rasterizer
```

The Workbench semantic panel owns only interaction and rendering glue.

### 8.3 Supported semantic objects

The integrated panel supports the existing 1.0 feature types:

```text
field_boundary
exclusion_zone
row_centerline
access_lane
entry_pose
work_direction
headland_zone
keepout_zone
```

It must preserve existing validation rules, object IDs, world-coordinate GeoJSON geometry and footprint-aware validation.

### 8.4 Automatic structure promotion

Automatic agricultural results are candidates, not silently committed semantics.

The Workbench supports an explicit promotion workflow:

```text
automatic candidate
  -> accept
  -> edit then accept
  -> reject
```

Promoted GeoJSON features record provenance in `properties`, for example:

```yaml
source: auto_row_detection
authoring_state: accepted
```

or after operator adjustment:

```yaml
source: auto_row_detection
authoring_state: manually_edited
```

Purely manual features use:

```yaml
source: manual
authoring_state: accepted
```

This provenance is intended to support later Paper I reporting of direct accepts, edits and manual additions.

## 9. Legacy semantic editor migration

The standalone `semantic_editor_qt5.py` is no longer allowed to mutate formal navigation rasters.

The following formal capabilities are retired from that path:

```text
map_free
map_occupied
map_unknown
freehand pixel painting
line pixel painting
_save_map_in_place() for formal map production
```

Migration may be staged:

1. disable formal raster mutation and show a clear deprecation message;
2. keep the standalone editor temporarily for legacy semantic-only inspection if needed;
3. remove the standalone formal entry point after Workbench acceptance.

No accepted map may be modified in place by the legacy editor.

## 10. Immutable Map Revision contract

The existing M1.5-4B minimum paths remain compatible:

```text
<revision>/generated/navigation_map.yaml
<revision>/generated/navigation_map.pgm
<revision>/accepted/navigation_map.yaml
<revision>/accepted/navigation_map.pgm
<revision>/derivation.yaml
```

The revision is extended, not replaced:

```text
<revision>/
├── evidence/
│   ├── ground_only_navigation_map.pgm
│   ├── ground_only_navigation_map.yaml
│   ├── ground_height.npy
│   ├── slope_deg.npy
│   ├── step_m.npy
│   ├── obstacle_count.npy
│   └── ground_support_count.npy
├── generated/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── accepted/
│   ├── navigation_map.pgm
│   └── navigation_map.yaml
├── semantic/
│   ├── semantic_map.geojson
│   ├── coverage.yaml
│   ├── keepout_mask.pgm
│   └── keepout_mask.yaml
├── authoring/
│   ├── processing_recipe.yaml
│   ├── map_frame.yaml
│   └── site_boundary.yaml
├── validation/
│   ├── navigation_validation.json
│   ├── semantic_validation.json
│   └── coverage_preview.json
└── derivation.yaml
```

`coverage.yaml.base_map` points to the accepted Nav2 YAML by relative path, and `base_map_sha256` must exactly match that accepted YAML file.

The revision writer is atomic: if any required component or validation step fails, no READY final revision directory is published.

## 11. Generated vs accepted semantics

The terms are frozen as follows:

```text
Evidence
= what the sensors / deterministic low-level derivation directly observed or calculated

Generated
= automatic structure-aware formal navigation map, with no manual override

Accepted
= Generated + validated formal human overrides
```

The semantic task binds to `accepted/`, never to Evidence and never to an independent raster.

## 12. Navigation Map QA gate

A map is not considered usable merely because a PGM file exists.

Add planner-independent QA before Paper I acceptance. At minimum report:

```text
outside_site_boundary_free_count
row_structural_band_free_leak_count
accepted_aisle_count
per_aisle_free_fraction
per_aisle_unknown_fraction
per_aisle_occupied_conflict_fraction
per_aisle_grid_connectivity
map_unknown_fraction
structure_inferred_free_fraction
manual_force_free_area_m2
manual_force_occupied_area_m2
```

Required hard failures:

```text
outside_site_boundary_free_count > 0
row_structural_band_free_leak_count > 0
accepted map cannot be deterministically replayed from generated + overrides
coverage.yaml hash does not bind exactly to accepted map YAML
semantic validation ERROR exists
map/frame/grid identity mismatch
```

The QA layer should also expose, but not necessarily hard-fail in the first implementation, aisle connectivity and unknown statistics so the operator can see why a map remains unusable.

A later planner dry-run can consume the same revision, but the materialization contract itself must not depend on one planner.

## 13. Headland policy for this increment

No new automatic headland detector is introduced.

Headland/open-area cells become FREE only through:

```text
observed Ground FREE
or
formal evidence-backed FORCE_FREE
```

This keeps the current increment focused on the known aisle occlusion failure and avoids inventing a second broad free-space inference policy.

A semantic `headland_zone` may still be authored for task meaning, but it does not automatically whiten UNKNOWN raster cells in this increment.

## 14. Resource bundle integration

The existing `Save Map Resource Bundle As...` workflow remains an export/transport mechanism, not authority.

Add an optional semantic group containing:

```text
semantic_map.geojson
coverage.yaml
keepout_mask.pgm
keepout_mask.yaml
semantic_validation.json
```

Recommended bundle export uses an already-frozen revision when available. It must not independently regenerate a different formal map.

## 15. Error handling and fail-closed behavior

Formal freeze fails closed when any of these conditions hold:

- no valid Site Boundary;
- structure arrays do not match the navigation grid;
- frame IDs differ;
- grid geometry changes between Evidence and Generated;
- generated occupancy contains invalid values;
- formal override metadata is invalid;
- accepted map replay does not match serialized accepted PGM;
- semantic task is invalid;
- semantic task cannot bind to the accepted map hash;
- required output path already exists;
- any staged writer fails.

The previous valid revision remains untouched.

## 16. Testing strategy

Implementation follows strict RED -> GREEN.

### 16.1 Offline-core unit tests

Add tests for the structure-aware materializer covering:

- observed FREE preserved;
- row structural band becomes OCCUPIED even with sparse obstacle returns;
- Ground-only UNKNOWN inside valid aisle geometry becomes structure-inferred FREE;
- UNKNOWN outside aisle geometry remains UNKNOWN;
- sensor OCCUPIED is never automatically freed;
- outside Site Boundary is OCCUPIED;
- output grid geometry is byte-for-byte/equality consistent with the source grid contract;
- deterministic repeated materialization;
- no overlap among incompatible provenance masks.

### 16.2 Override / freeze tests

- generated contains no manual override;
- accepted exactly equals generated + ordered override replay;
- invalid Paper override metadata blocks freeze;
- existing revision directory is never overwritten;
- writer failure leaves no final partial revision.

### 16.3 Semantic integration tests

- semantic feature authoring uses map-frame coordinates;
- promoted automatic row/lane features carry provenance;
- semantic ERROR blocks formal freeze;
- `coverage.base_map` targets accepted map YAML;
- `coverage.base_map_sha256` equals accepted YAML SHA256;
- direct formal PGM mutation is unavailable from the legacy editor.

### 16.4 QA tests

- outside-boundary FREE leak is detected;
- row-band FREE leak is detected;
- aisle fractions and connectivity are deterministic;
- accepted replay mismatch is detected;
- semantic/hash mismatch is detected.

### 16.5 Regression gates

Preserve:

- V25-12C ground-relative derivation tests;
- V25-12F traversability tests;
- Site Boundary tests;
- vehicle-safe-lane tests;
- M1.5-4B map-authority tests;
- resource-bundle atomicity tests.

## 17. Target-machine real-map acceptance

After software tests pass, use the intended greenhouse PCD and require one full Workbench run:

```text
PCD loaded
-> Ground-only evidence generated
-> agricultural rows/aisles reviewed
-> Site Boundary READY
-> structure-aware generated map produced
-> operator reviews unresolved UNKNOWN / conflicts
-> only evidence-backed formal overrides added where necessary
-> semantic task authored/validated
-> immutable revision frozen
-> map-only QA passes hard invariants
-> human visual review passes
-> Paper project binds revision as V25_MAP_WORKBENCH / BOUND_VERIFIED
```

Record at least:

```text
Ground-only FREE/OCCUPIED/UNKNOWN counts
Generated FREE/OCCUPIED/UNKNOWN counts
Accepted FREE/OCCUPIED/UNKNOWN counts
structure-inferred FREE cell count / area
manual FORCE_FREE area
manual FORCE_OCCUPIED area
accepted aisle connectivity summary
semantic validation summary
all output hashes
```

These records become Paper I experiment evidence rather than ad hoc screenshots only.

## 18. Paper I experiment interpretation

The architecture naturally supports an ablation without changing map contracts:

```text
M0 = Ground-only evidence map
M1 = structure-aware Generated map
M2 = human-reviewed Accepted map
```

Candidate metrics include:

```text
UNKNOWN ratio
accepted-aisle connectivity
per-aisle passable fraction
planner success on fixed route queries
manual correction area
real vehicle route completion in later stages
```

This allows the paper to distinguish sensor evidence, agricultural structural inference and human review instead of presenting one opaque final PGM.

## 19. Implementation order

Once this written design is approved, the implementation plan should stage work in this order:

```text
A. structure-aware offline materializer + tests
B. formal revision writer / QA + tests
C. Workbench generated/accepted wiring
D. integrated semantic authoring panel using existing semantic core
E. automatic row/aisle promotion provenance
F. legacy semantic-editor raster mutation deprecation
G. resource-bundle semantic group
H. real greenhouse target-machine acceptance
```

This order deliberately solves the unusable-PGM problem before expanding GUI behavior, while keeping each layer testable without Qt where possible.

## 20. Frozen design decisions

The following decisions are considered frozen for this increment unless a new design amendment is explicitly approved:

```text
V25 Workbench accepted revision is the only formal Navigation Map authority.
Ground-only occupancy is Evidence, not the final formal PGM.
Generated formal PGM is structure-aware.
Accepted formal PGM is Generated + auditable world-coordinate overrides.
Row structural bands are formal blocked evidence.
Validated aisle geometry may promote Ground-only UNKNOWN to structure-inferred FREE.
UNKNOWN outside supported geometry remains UNKNOWN.
Site Boundary and field_boundary remain distinct concepts.
Semantic task binds to accepted map by SHA256.
Standalone semantic editor may not directly mutate formal accepted PGM assets.
No global UNKNOWN fill, no cross-row morphology, and no new headland detector in this increment.
```