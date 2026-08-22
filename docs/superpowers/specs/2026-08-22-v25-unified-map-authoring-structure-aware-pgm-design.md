# V25 Unified Map Authoring + Structure-Aware PGM Design

Date: 2026-08-22  
Branch: `refactor/v25-unified-map-authoring`  
Base: `fix/paper1-m154b-v25-map-authority-recovery@5f889c6f0e10d8e77955e25fb624f2a20ccfcda9`

## 1. Intent

This increment removes the repository's competing map-authoring paths and closes the remaining gap between successful greenhouse structure extraction and a usable formal Nav2 PGM.

The frozen ownership rule is:

```text
One PCD lineage
  -> One AGT Map Workbench authoring workflow
  -> One immutable Map Revision
  -> One accepted Navigation Map authority
  -> One semantic task bound by SHA256 to that accepted map
```

`agt_map_pipeline` remains a consumer/verifier/evidence producer. It must not create a competing formal Navigation Map.

## 2. Current problem

### 2.1 Two practical map authorities

The repository currently has two authoring paths:

```text
A. V25 Map Workbench
PCD -> Ground-relative map -> agricultural structure -> Site Boundary
    -> world-coordinate overrides -> generated/accepted Navigation Map

B. standalone semantic editor
Nav2 YAML/PGM -> direct FREE/OCCUPIED/UNKNOWN pixel painting
    -> semantic_map.geojson + coverage.yaml
```

Path B can mutate the raster and then update `coverage.yaml.base_map_sha256`. The result is internally consistent but is no longer necessarily the reviewed Workbench map.

The standalone semantic editor therefore must cease to own formal raster production.

### 2.2 Structure extraction is not yet the formal PGM producer

The current Ground-relative derivation correctly creates conservative `FREE/OCCUPIED/UNKNOWN` evidence from local ground, obstacle, slope and step information.

The agricultural stack then derives:

```text
row support
row structural band
aisle geometric envelope
refined aisle / centerline
V25-12F traversability evidence
```

However, formal Paper I freeze still starts from the Ground-only base result. Therefore:

```text
row / aisle segmentation success
!=
formal PGM uses row / aisle structure
```

In greenhouse data this leaves valid aisles fragmented by UNKNOWN cells caused by occlusion or sparse ground returns.

## 3. Scope

### 3.1 In scope

- integrate semantic authoring into the existing Workbench as one `语义与任务` tab/panel;
- reuse existing GUI-independent `agt_ui_bridge` semantic model/IO/validation logic;
- prohibit direct formal pixel painting of accepted PGM assets;
- add one deterministic Structure-Aware Navigation Materializer;
- classify Ground-only occupancy as Evidence rather than the final formal map;
- use row structural bands as formal blocked evidence;
- allow structurally valid aisle geometry to resolve Ground-only UNKNOWN cells;
- keep evidence-backed world-coordinate `FORCE_FREE` / `FORCE_OCCUPIED` as the formal human raster-correction mechanism;
- bind `semantic_map.geojson` and `coverage.yaml` to the accepted map by exact SHA256 identity;
- freeze one immutable revision containing Evidence, Generated, Accepted, semantics, authoring inputs and QA;
- preserve the M1.5-4B `V25_MAP_WORKBENCH / BOUND_VERIFIED` ownership contract.

### 3.2 Non-goals

- no semantic schema 2.0 redesign;
- no global `UNKNOWN -> FREE` conversion;
- no cross-row morphology or generic flood fill;
- no route-dependent whitening of the PGM;
- no new automatic headland detector in this increment;
- no planner/RPP tuning until the map-authoring gate is restored;
- no promotion of existing `navigation_map_12f.*` to formal authority by renaming files.

## 4. Final data flow

```text
raw / cleaned PCD
        |
        v
AGT Map Workbench
        |
        +-- Point-cloud Recipe -> processed.pcd
        |
        +-- Map Frame calibration / canonical PCD
        |
        +-- Ground-relative Evidence
        |      -> ground / obstacle / slope / step
        |      -> Ground-only trinary occupancy
        |
        +-- Agricultural structure
        |      -> row support
        |      -> row structural band
        |      -> aisle geometric envelope
        |      -> aisle/refined centerline
        |
        +-- Site Boundary
        |
        +-- Structure-Aware Navigation Materializer
        |      -> generated/navigation_map.{pgm,yaml}
        |
        +-- Formal evidence-backed overrides
        |      -> accepted/navigation_map.{pgm,yaml}
        |
        +-- Semantic authoring
        |      -> semantic_map.geojson
        |      -> coverage.yaml
        |      -> keepout mask
        |
        +-- Navigation + semantic QA
               -> immutable Map Revision
```

The standalone semantic editor is no longer a formal map-authority application.

## 5. Data ownership remains separated

One GUI does not mean one overloaded file.

| Asset | Meaning | Carrier |
| --- | --- | --- |
| Site Boundary | physical inner boundary the vehicle footprint may enter | `authoring/site_boundary.yaml` |
| Navigation Map | Nav2 FREE/OCCUPIED/UNKNOWN raster | `generated/`, `accepted/` |
| Formal raster override | evidence-backed correction of automatic raster | derivation metadata + replay |
| field boundary | task/coverage region | `semantic/semantic_map.geojson` |
| exclusion / keepout | semantic task/no-go region | `semantic/semantic_map.geojson` + keepout mask |
| row centerline | crop-row semantic geometry | `semantic/semantic_map.geojson` |
| access lane | explicit semantic road | `semantic/semantic_map.geojson` |
| entry pose | task entry pose | `semantic/semantic_map.geojson` |
| work direction | task direction | `semantic/semantic_map.geojson` |
| coverage parameters | task/planning configuration | `semantic/coverage.yaml` |

`Site Boundary` and `field_boundary` are deliberately not aliases. A convenience action may create a field-boundary candidate from Site Boundary, but an operator must explicitly accept it as semantic data.

Semantic `keepout_zone` remains a separate Nav2 semantic mask. It does not silently rewrite the generated or accepted base PGM in this increment.

## 6. Structure-Aware Navigation Materializer

### 6.1 Offline core

Add a focused offline module, planned as:

```text
src/agt_offline_assets/agt_offline_assets/formal_navigation_map.py
```

It contains no Qt code and does not directly own file dialogs or GUI state.

Inputs:

```text
NavigationMapResult        # Ground-only conservative Evidence
NavigationStructureResult  # terrain/row evidence
CorridorRefinementResult   # row band + aisle geometry
SiteBoundary               # mandatory in formal mode
```

Output:

```text
StructureAwareNavigationResult
- occupancy
- observed_free_mask
- structure_inferred_free_mask
- base_hard_occupied_mask
- row_structural_blocked_mask
- site_boundary_blocked_mask
- unresolved_unknown_mask
- counts / provenance
- exact source grid geometry
```

The materializer must preserve resolution, origin, width and height exactly.

### 6.2 Generated-map precedence

Generated materialization contains no manual override.

For each cell:

```text
1. outside Site Boundary
   -> OCCUPIED

2. Ground-only base occupancy == OCCUPIED
   -> OCCUPIED
   # includes current obstacle, slope, step and obstacle-padding decisions

3. row_structural_band
   -> OCCUPIED

4. Ground-only base occupancy == FREE
   -> OBSERVED_FREE -> FREE

5. Ground-only base occupancy == UNKNOWN
   AND inside structurally admissible aisle geometry
   -> STRUCTURE_INFERRED_FREE -> FREE

6. otherwise
   -> UNKNOWN
```

The materializer never automatically changes a current Ground-only OCCUPIED cell to FREE. This is important: structure inference solves missing evidence, not explicit negative evidence.

### 6.3 Structurally admissible aisle geometry

The inference source is `CorridorRefinementResult.aisle_geometric_envelope` with its existing structural gates.

The existing corridor code only emits non-empty geometry after the corresponding row/boundary pair has passed the prerequisites required to define that corridor, including row support, geometric width and longitudinal overlap.

For this design, the term **structurally admissible** refers to those geometric prerequisites, not necessarily to a final diagnostic status of `ACCEPTED`.

This distinction is intentional. A corridor can have valid agricultural geometry while having no current safe cells because Ground evidence is missing. That exact condition is the greenhouse failure this materializer is intended to repair.

Safety is retained because:

```text
base OCCUPIED is never auto-freed
row structural band is blocked
outside Site Boundary is blocked
only base UNKNOWN can be promoted
```

Therefore a slope/step/obstacle cell already classified OCCUPIED remains OCCUPIED even if it lies inside the aisle geometry.

### 6.4 Row structural blocking

`CorridorRefinementResult.row_structural_band` becomes formal blocked evidence.

This prevents sparse vegetation returns from turning a known crop row into navigable white space.

If the row structural model itself is wrong, the correct workflow is to fix/review the structure and regenerate the map rather than silently paint through the row band.

### 6.5 UNKNOWN preservation

Outside structurally admissible aisle geometry, cells without direct positive/negative Evidence remain UNKNOWN.

The implementation must not perform:

```text
all UNKNOWN -> FREE
map-wide flood fill
cross-row closing
free-space inference from image background
```

### 6.6 Relation to V25-12F

V25-12F remains evidence/candidate logic and continues to be useful for bounded occlusion reasoning and diagnostics.

The new formal materializer does not simply rename `navigation_map_12f.pgm`. It owns a separate deterministic contract whose output is the automatic Generated map.

## 7. Formal manual correction and Accepted map

Paper I formal modes remain:

```text
FORCE_FREE
FORCE_OCCUPIED
```

Each override must retain the current audit metadata, including non-empty reason and evidence category.

Accepted is produced only by deterministic replay:

```text
Generated structure-aware map
  + ordered validated formal overrides
  -> Accepted map
```

### 7.1 FORCE_OCCUPIED

`FORCE_OCCUPIED` may mark any in-grid area OCCUPIED and always wins for its rasterized cells according to ordered replay.

### 7.2 FORCE_FREE safety rule

`FORCE_FREE` is intended for independently evidenced false obstacle/UNKNOWN cases such as severe occlusion, sparse returns or a known false sensor obstacle.

However, formal `FORCE_FREE` is rejected wherever it would create FREE cells:

```text
outside Site Boundary
or
inside row_structural_band
```

Thus it may correct automatic sensor/terrain evidence when independently justified, but it cannot punch through the physical permitted boundary or the current accepted crop-row structural model.

If the row structural model is wrong, review/fix the structure and regenerate instead.

This rule removes ambiguity between human correction and hard geometric invariants.

## 8. Semantic authoring integration

### 8.1 Workbench UI

The existing right-side tabs become:

```text
点云编辑
坐标系标定
导航地图
语义与任务
```

The formal workflow does not open a second top-level semantic editor.

### 8.2 Reuse the existing semantic core

Reuse rather than duplicate:

```text
agt_ui_bridge.semantic_model
agt_ui_bridge.semantic_io
agt_ui_bridge.semantic_validation
agt_ui_bridge.semantic_scene
agt_ui_bridge.semantic_rasterizer
```

The new Workbench panel owns only Qt interaction/rendering glue.

### 8.3 Existing semantic schema remains

Support the existing 1.0 feature types:

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

Existing world-coordinate geometry, ID rules and footprint-aware validation remain authoritative.

### 8.4 Automatic structure promotion

Automatic rows/lanes are candidates and are never silently committed to GeoJSON.

Workflow:

```text
automatic candidate
  -> accept
  -> edit then accept
  -> reject
```

Promoted features record provenance in `properties`:

```yaml
source: auto_row_detection
authoring_state: accepted
```

After operator editing:

```yaml
source: auto_row_detection
authoring_state: manually_edited
```

Pure manual objects use:

```yaml
source: manual
authoring_state: accepted
```

## 9. Legacy semantic-editor migration

The standalone `semantic_editor_qt5.py` may no longer mutate formal navigation rasters.

Formal retirement includes:

```text
map_free
map_occupied
map_unknown
freehand pixel painting
line pixel painting
_save_map_in_place() as a formal map-production path
```

Migration may be staged: disable raster mutation first, retain legacy semantic-only inspection temporarily if useful, and later remove the standalone formal entry point after Workbench acceptance.

No accepted revision may be modified in place.

## 10. Immutable Map Revision

The existing M1.5-4B minimum paths remain valid:

```text
<revision>/generated/navigation_map.yaml
<revision>/generated/navigation_map.pgm
<revision>/accepted/navigation_map.yaml
<revision>/accepted/navigation_map.pgm
<revision>/derivation.yaml
```

The revision is extended as follows:

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

`coverage.yaml.base_map` points by relative path to `accepted/navigation_map.yaml` and `base_map_sha256` must equal the exact SHA256 of that accepted YAML.

The writer is atomic: a failed required writer/validation step must not publish a final revision directory.

## 11. Evidence / Generated / Accepted terminology

These meanings are frozen:

```text
Evidence
= sensor-derived / deterministic low-level observation and Ground-only occupancy

Generated
= automatic structure-aware formal Navigation Map with no human raster override

Accepted
= Generated + validated evidence-backed human overrides
```

The semantic task binds to Accepted, never to Evidence and never to an independently edited raster.

## 12. Navigation Map QA gate

A PGM file existing is not sufficient acceptance.

At minimum report:

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

`per_aisle_grid_connectivity` is a planner-independent diagnostic computed as 8-connected FREE-cell connectivity within each structurally admissible aisle envelope.

Hard failures:

```text
outside_site_boundary_free_count > 0
row_structural_band_free_leak_count > 0
Accepted cannot be replayed deterministically from Generated + overrides
coverage.yaml hash does not match accepted map YAML
semantic validation contains ERROR
map/frame/grid identity mismatch
serialized PGM/YAML do not match the in-memory accepted grid
```

Aisle connectivity and UNKNOWN percentages are reported even when they are not yet hard-fail thresholds, so the operator can diagnose an unusable map before planner experiments.

A later Nav2 dry-run may consume the same revision, but map materialization itself must remain planner-independent.

## 13. Headland policy

No new headland detector is introduced in this increment.

Headland/open-area cells become FREE only through:

```text
Observed Ground FREE
or
validated evidence-backed FORCE_FREE
```

A semantic `headland_zone` may describe task meaning but does not automatically whiten UNKNOWN cells in the base PGM.

## 14. Resource bundle integration

`Save Map Resource Bundle As...` remains a persistence/transport feature, not a map-authority producer.

Add one optional semantic group:

```text
semantic_map.geojson
coverage.yaml
keepout_mask.pgm
keepout_mask.yaml
semantic_validation.json
```

When a frozen revision exists, bundle export should package that revision rather than independently regenerate a formal raster.

## 15. Fail-closed conditions

Formal freeze fails when any of the following is true:

- no valid Site Boundary;
- Evidence/structure arrays do not share one grid shape;
- frame IDs differ;
- grid geometry changes between Evidence and Generated;
- Generated/Accepted occupancy contains values outside the AGT trinary contract;
- formal override metadata is invalid;
- `FORCE_FREE` would free outside Site Boundary or inside row structural band;
- Accepted replay differs from serialized Accepted PGM;
- semantic validation fails;
- semantic task cannot bind exactly to Accepted SHA256;
- required output path already exists;
- any staging writer fails.

The previous valid revision remains untouched.

## 16. Testing strategy

Implementation follows strict RED -> GREEN.

### 16.1 Structure-aware core

Tests must cover:

- Ground-only FREE remains FREE;
- Ground-only OCCUPIED remains OCCUPIED even inside aisle geometry;
- row structural band becomes OCCUPIED;
- Ground-only UNKNOWN inside structurally admissible aisle geometry becomes structure-inferred FREE;
- UNKNOWN outside aisle geometry remains UNKNOWN;
- outside Site Boundary becomes OCCUPIED;
- exact grid geometry preservation;
- deterministic repeated materialization;
- provenance masks do not claim contradictory states.

### 16.2 Formal override/freeze

Tests must cover:

- Generated contains no manual override;
- Accepted exactly equals Generated + ordered validated replay;
- `FORCE_FREE` outside Site Boundary is rejected;
- `FORCE_FREE` inside row band is rejected;
- independently evidenced `FORCE_FREE` may correct eligible Generated UNKNOWN/OCCUPIED cells;
- invalid override metadata blocks freeze;
- an existing revision is never overwritten;
- writer failure leaves no final partial revision.

### 16.3 Semantic integration

Tests must cover:

- map-frame coordinates are preserved;
- automatic row/lane promotion records provenance;
- semantic ERROR blocks formal freeze;
- coverage binds to Accepted YAML path/hash;
- legacy editor formal raster mutation is unavailable.

### 16.4 QA

Tests must cover:

- outside-boundary FREE leak detection;
- row-band FREE leak detection;
- deterministic aisle fractions/connectivity;
- Accepted replay mismatch detection;
- semantic/hash mismatch detection.

### 16.5 Regression

Preserve the existing Ground-relative, V25-12F, Site Boundary, vehicle-safe-lane, M1.5-4B map-authority and resource-bundle tests.

## 17. Real greenhouse acceptance

After software gates pass, run one full target-machine workflow on the intended greenhouse PCD:

```text
PCD loaded
-> Ground-only Evidence generated
-> rows/aisles reviewed
-> Site Boundary READY
-> Structure-aware Generated map produced
-> unresolved UNKNOWN/conflicts reviewed
-> only evidence-backed formal overrides added where necessary
-> semantic task authored/validated
-> immutable revision frozen
-> map-only QA hard invariants pass
-> human visual review passes
-> Paper project binds revision as V25_MAP_WORKBENCH / BOUND_VERIFIED
```

Record at least:

```text
Ground-only FREE/OCCUPIED/UNKNOWN counts
Generated FREE/OCCUPIED/UNKNOWN counts
Accepted FREE/OCCUPIED/UNKNOWN counts
structure-inferred FREE cells / area
manual FORCE_FREE area
manual FORCE_OCCUPIED area
per-aisle connectivity summary
semantic validation summary
all formal output hashes
```

## 18. Paper I ablation enabled by this architecture

Without changing map contracts, Paper I can compare:

```text
M0 = Ground-only Evidence map
M1 = structure-aware Generated map
M2 = human-reviewed Accepted map
```

Useful metrics include:

```text
UNKNOWN ratio
per-aisle passable fraction
per-aisle connectivity
fixed-query planner success
manual correction area
later real-vehicle route completion
```

This cleanly separates sensor evidence, agricultural structural inference and human review.

## 19. Implementation order after written-spec approval

```text
A. Structure-Aware Navigation Materializer + unit tests
B. formal revision writer + map QA + tests
C. Workbench Generated/Accepted wiring
D. integrated semantic authoring panel using existing semantic core
E. automatic row/aisle promotion provenance
F. legacy semantic-editor raster-mutation deprecation
G. resource-bundle semantic group
H. target-machine real greenhouse acceptance
```

This order solves the unusable-PGM problem before expanding GUI behavior and keeps most early work testable without Qt.

## 20. Frozen decisions

```text
V25 Workbench Accepted revision is the only formal Navigation Map authority.
Ground-only occupancy is Evidence, not the final formal PGM.
Generated formal PGM is structure-aware.
Accepted formal PGM is Generated + auditable world-coordinate overrides.
Ground-only OCCUPIED is never auto-freed by structure inference.
Row structural bands are formal blocked evidence.
Only Ground-only UNKNOWN inside structurally admissible aisle geometry is auto-promoted to FREE.
Outside supported geometry, UNKNOWN remains UNKNOWN.
FORCE_FREE cannot violate Site Boundary or row structural band.
Site Boundary and field_boundary remain distinct.
Semantic keepout remains a separate semantic mask and does not silently rewrite the base PGM.
Semantic task binds to Accepted by exact SHA256.
Standalone semantic editor may not directly mutate formal accepted PGM assets.
No global UNKNOWN fill, cross-row morphology, route-dependent whitening or new headland detector is introduced here.
```