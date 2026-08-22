# Paper I Map Semantics Freeze Design

## Status

DESIGN FREEZE / PAPER I MAP-SEMANTICS SCOPE

## Research question

Given a globally consistent greenhouse point cloud, recover a conservative, structure-aware vehicle-traversability map from vegetation occlusion and incomplete ground observations, and expose the result as auditable agricultural semantic assets for downstream navigation.

The paper does **not** propose a new SLAM frontend. Mapping-source quality is evaluated independently in `lio_benchmark_tools` and imported as evidence.

## Inputs

Primary site snapshot:

- `greenhouse_01_map_assets_v01/map_resource_manifest.yaml`
- `pointcloud/processed.pcd`
- `navigation/generated/*`
- `navigation/accepted/*`
- `navigation/derivation.yaml`
- terrain / obstacle / structure evidence layers
- `authoring/site_boundary.yaml`
- `planning/vehicle_feasible_segments.yaml`

Mapping-source evidence may include:

- FAST-LIVO2 LIO-only global map
- Kilo-Map global map
- handheld 3D scanner map produced with FAST-LIO

These sources are compared in `lio_benchmark_tools`; Paper I consumes frozen metrics and representative maps rather than reimplementing LIO evaluation here.

## Core method scope

1. Ground-relative terrain representation.
2. Conservative obstacle / unknown handling.
3. Agricultural row structural recovery.
4. Aisle geometric envelope and centerline extraction.
5. Site Boundary as a hard vehicle-permitted gate.
6. Bounded recovery of vegetation-occluded traversable space.
7. Structured agricultural semantic output and asset lineage.

Traversability semantics remain:

- `OBSERVED_FREE`
- `INFERRED_TRAVERSABLE`
- `HARD_BLOCKED`
- `SENSOR_OBSTACLE`
- `UNKNOWN`

The method must never globally convert UNKNOWN to FREE.

## Required paper-facing artifacts

### Formal map identity

The paper records:

- map-resource bundle schema and bundle id
- bundle manifest SHA256
- processed PCD SHA256
- accepted map YAML/PGM SHA256
- map frame and exact grid geometry
- source git commit

Any changed asset creates a new bundle/revision; `v01` is immutable.

### Semantic representation

Freeze a paper-facing semantic asset contract that can represent at least:

- site boundary
- crop-row centerlines / structural bands
- aisle centerlines / aisle polygons or envelopes
- traversability class / confidence where applicable
- stable IDs and source lineage

The formal semantic asset should use `frame_id: map` and must be verified against the exact accepted navigation grid.

### Paper figures

Generate reproducible figures from frozen assets, not screenshots as the sole evidence:

1. raw/global point cloud overview
2. ground-relative terrain / confidence
3. raw obstacle and UNKNOWN failure case under vegetation
4. recovered row structure
5. aisle geometric envelope / centerline
6. traversability-class map
7. generated vs accepted map and audited overrides
8. vehicle-feasible region / segments
9. representative failure cases
10. mapping-source comparison figure imported from `lio_benchmark_tools`

## Experiments

### E1 — Representation ablation

Compare the same site/grid with progressively stronger evidence:

- A: conventional / raw occupancy projection baseline
- B: ground-relative representation
- C: B + agricultural row structure
- D: C + aisle envelope + bounded occlusion recovery + Site Boundary

Primary metrics:

- traversable precision
- traversable recall
- traversable IoU
- false-free rate
- false-blocked rate
- UNKNOWN ratio
- largest connected traversable component
- reachable aisle count

False-free rate is safety-critical and must be reported explicitly.

### E2 — Mapping-source robustness

Import frozen comparison results from `lio_benchmark_tools` for FAST-LIVO2 LIO-only, Kilo-Map, and handheld FAST-LIO map where available.

Paper-facing metrics focus on downstream structural consistency, for example:

- row direction deviation
- row-spacing deviation
- aisle-centerline deviation
- traversable-area agreement
- row/aisle topology agreement

Trajectory / APE / RPE and mapping-runtime details remain owned by `lio_benchmark_tools`.

### E3 — Minimal downstream navigation validation

Use one fixed route and one fixed controller configuration only to show that the accepted semantic map can produce an executable navigation path.

This is **not** a global planner comparison.

The validation path is converted to `nav_msgs/Path` and tracked with Nav2 Regulated Pure Pursuit (RPP). Report path completion, cross-track error, heading error, stop/failure reasons, and safety intervention count where available.

## Explicit non-goals for Paper I

Do not add as paper contributions in this branch:

- A* vs Theta* vs Hybrid-A* vs RRT* comparison
- complete / maximum coverage planning
- Fields2Cover comparison
- Ackermann global route optimization
- BT / Mission orchestration contributions
- GNSS fusion contributions
- new SLAM frontend/back-end algorithms

Those belong to later work.

## Completion gate

Paper I map-semantics freeze is complete only when:

1. `greenhouse_01_map_assets_v01` is immutable and hash-recorded.
2. accepted-map replay QA passes and human map review is recorded.
3. semantic asset is frozen and in-bounds on the accepted grid.
4. representation ablation protocol and reference labels are frozen.
5. mapping-source comparison results are imported by hash/commit from `lio_benchmark_tools`.
6. one fixed route can be exported into the runtime route contract.
7. paper figures/tables can be regenerated from frozen inputs.

Only after this gate should the maximum-coverage project branch from the frozen semantic-map commit.
