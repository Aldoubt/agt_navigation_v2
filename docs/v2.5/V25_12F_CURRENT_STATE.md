# V25-12F Current State

Date: 2026-08-15

Status:

```text
CODE IMPLEMENTED / LOCAL AUTOMATED VERIFICATION PENDING / REAL-DATA A/B PENDING
```

This checkpoint continues from `V25_12E_CURRENT_STATE.md`

The frozen V25-12E real-data baselines remain regression evidence and are not replaced by this document

## Why V25-12F exists

V25-12E Route Debug real greenhouse review exposed that the dominant problem is upstream planning-space semantics rather than an insufficient R6B local search budget

Observed operator findings:

```text
1. the current final Navigation Map is too conservative in vegetation-occluded aisle regions
2. agricultural Ground Confidence / Robust Slope / refined-aisle evidence preserves much more realistic aisle continuity
3. greenhouse walls / site limits are not an independent hard no-traverse invariant
```

Therefore V25-12F repairs traversability and hard-boundary semantics before further R6B/R7 work

## Frozen safety semantics

```text
site_boundary
= vehicle-permitted inner perimeter
!= wall centerline
```

The polygon already includes wall thickness / wall uncertainty / desired wall-side stand-off

Continuous footprint policy:

```text
strictly inside boundary → may continue to other gates
touch boundary           → SITE_BOUNDARY_CONFLICT
cross boundary           → SITE_BOUNDARY_CONFLICT
```

A FREE raster cell can never override this continuous hard-boundary rule

A READY `site_boundary.yaml` is mandatory for a 12F candidate

No map-bounds fallback and no automatic wall extraction are allowed in this increment

## Structural aisle semantics

`CorridorRefinementResult` now preserves:

```text
aisle_geometric_envelope
```

This mask is formed only for structurally valid corridor pairs that already pass width / row-support / longitudinal-overlap checks, but it is retained before Ground / confidence / slope / obstacle filtering

Relationship:

```text
aisle_geometric_envelope
        ↓ terrain / ground / obstacle gates
aisle_candidate / refined aisle
        ↓ centerline selection
aisle_centerline
```

This is the bounded domain where missing direct terrain evidence may later be recovered

`Vehicle Corridor / required_envelope_mask` remains expected vehicle geometry and is not direct FREE truth

## Rich traversability contract

Internal states:

```text
OBSERVED_FREE
INFERRED_TRAVERSABLE
HARD_BLOCKED
SENSOR_OBSTACLE
UNKNOWN
```

A-first recovery policy:

```text
candidate cell must currently be UNKNOWN
inside aisle_geometric_envelope
inside site_boundary
outside semantic NO_GO
outside row_structural_band
local row direction known
OBSERVED_FREE support on both longitudinal sides
unsupported gap <= 0.60 m
endpoint height / slope / step continuity accepted
```

Current OCCUPIED is not recoverable in the A-first candidate

No generic 2D morphological fill and no global UNKNOWN→FREE conversion are permitted

Compatibility export:

```text
OBSERVED_FREE        → FREE
INFERRED_TRAVERSABLE → FREE
HARD_BLOCKED         → OCCUPIED
SENSOR_OBSTACLE      → OCCUPIED
semantic NO_GO       → OCCUPIED
UNKNOWN              → UNKNOWN
```

## Candidate artifacts

Required frozen Site Boundary:

```text
site_boundary.yaml
schema agt_site_boundary/v1
```

Candidate outputs:

```text
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml
```

The candidate writer must not overwrite:

```text
navigation_map.yaml
navigation_map.pgm
derivation.yaml
```

Workbench candidate generation uses a separate `_navigation_12f_result` and does not replace the current `_navigation_result`

## Shared footprint-level hard-boundary gate

Site Boundary is now an optional shared invariant for:

```text
Vehicle-Safe Aisle Lane
Forward Connector Navigation Gate
R6B bounded reverse primitive search
```

When supplied, each transformed preview vehicle footprint must be strictly inside the boundary before/alongside the existing Navigation Grid FREE check

R6B defaults remain frozen:

```text
primitive_length_m          0.30
collision_sample_step_m     0.10
state_xy_resolution_m       0.15
state_yaw_resolution_deg   15.0
goal_position_tolerance_m   0.18
goal_yaw_tolerance_deg     12.0
goal_shot_distance_m        3.0
max_cusps                   2
max_expansions          30000
max_path_length_m          18.0
reverse_cost_multiplier     1.15
cusp_penalty_m              0.75
steering_change_penalty_m   0.05
longitudinal_zone_padding_m 0.80
lateral_pair_padding_m      1.25
preview_footprint_padding_m 0.05
```

No search-budget increase or R6A policy relaxation is part of 12F

## Route Debug 12F A/B

The stable V25-12E `RouteDebugDataset` is intentionally not expanded into the truth owner for 12F

A separate optional `RouteDebug12FBundle` loads:

```text
site_boundary.yaml
navigation_map_12f.yaml
traversability_evidence.yaml / npz
```

Missing 12F assets degrade to unavailable layers

Invalid 12F evidence fails closed for that optional layer and does not make the legacy 12E debug dataset unusable

New Route Debug preset:

```text
12F A/B
```

Available visual layers include:

```text
Current Navigation Map
12F Candidate Navigation Map
Site Boundary
OBSERVED_FREE
INFERRED_TRAVERSABLE
HARD_BLOCKED
SENSOR_OBSTACLE
12F UNKNOWN
Aisle Geometric Envelope
```

Route Debug remains render-only and does not invoke candidate derivation or route planning

## Frozen V25-12E regression probes

The following real-data probes remain important during 12F acceptance:

```text
aisle_003
V25-12E classification: RAW_OBSTACLE_DIRECT_DOMINANT
12F expectation: strong direct obstacle evidence must not be blindly recovered

aisle_005
structural width 0.658 m < required preview width 0.700 m
12F expectation: remains STRUCTURAL_WIDTH_BLOCKED

aisle_013
padding-dominant probe
V25-12E selected-pose occupied hit padding fraction approximately 0.761
12F expectation: inspect whether candidate semantics improve artificial blockage without erasing real obstacle evidence

connector_015
V25-12E solved baseline:
FORWARD → REVERSE → FORWARD
2 cusps
4.499 m total
3.899 m forward
0.600 m reverse
1490 expansions
0.154 m / 8.11 deg goal error

connector_017
V25-12E baseline:
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
67 expansions
```

12F does not require those connector metrics to remain numerically identical because the candidate map and hard boundary deliberately change the planning constraints

The requirement is that any change is explainable from the 12F evidence while the R6B configuration itself remains unchanged

## Repeatable A/B harness

Tool:

```text
tools/v25_12f_acceptance.py
```

Run after `site_boundary.yaml` and candidate outputs have been frozen:

```bash
python3 tools/v25_12f_acceptance.py \
  --run-dir /home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run \
  --vehicle-profile /home/yangxuan/agt_navigation_v2/profiles/platforms/mk_mini.yaml \
  --pretty
```

It compares current vs candidate with the same VehicleSafeLaneConfig and replays `connector_015` / `connector_017` with one unchanged default `ReversePrimitiveConnectorConfig`

The output schema is:

```text
agt_v25_12f_acceptance_report/v1
validation_scope OFFLINE_A_B_REVIEW_NOT_ROUTE_READY
```

The harness does not regenerate the candidate map and does not promote any asset

## Verification boundary

The implementation has been written through the GitHub connector in this conversation

The assistant execution environment could not resolve GitHub through git and therefore could not create a runnable local worktree or execute ROS2/PyQt5 pytest / colcon against these commits

Therefore this document must not claim:

```text
AUTOMATED TEST PASS
BUILD PASS
REAL-DATA A/B PASS
REAL-DATA OPERATOR ACCEPTANCE PASS
```

until the operator machine provides those results

## Immediate local gate

On the operator machine:

```bash
cd ~/agt_navigation_v2
git fetch origin
git switch feat/v25-12f-traversability-hard-boundary
git pull --ff-only

source /opt/ros/humble/setup.bash

rm -rf \
  build/agt_offline_assets \
  build/agt_map_workbench \
  install/agt_offline_assets \
  install/agt_map_workbench

colcon build \
  --symlink-install \
  --packages-select agt_offline_assets agt_map_workbench

source install/setup.bash
```

Then run the focused automated contracts before any real-data promotion

## Real-data operator gate

After automated tests pass:

```text
1. open processed.pcd in the existing Workbench
2. draw the vehicle-permitted inner greenhouse Site Boundary
3. export site_boundary.yaml into runtime/maps/agt_workbench_run
4. generate and export the 12F candidate
5. open Route Debug → 12F A/B
6. inspect wall/perimeter behavior and inferred gaps
7. inspect aisle_003 / aisle_005 / aisle_013
8. run tools/v25_12f_acceptance.py
9. inspect connector_015 / connector_017 current-vs-candidate results
10. only then decide whether the candidate is eligible for canonical promotion
```

## Current next gate

```text
LOCAL AUTOMATED VERIFICATION
        ↓
fix any compile/test regressions
        ↓
REAL Site Boundary authoring
        ↓
12F candidate freeze
        ↓
Route Debug A/B + acceptance harness
        ↓
operator decision on candidate semantics
        ↓ only after acceptance
canonical Navigation Map promotion decision
        ↓
resume Vehicle-Safe Lane / Connector / R6B / R7 progression
```

Do not tune R6B to compensate for a failed 12F map
