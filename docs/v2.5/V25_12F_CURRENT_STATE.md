# V25-12F Current State

Date: 2026-08-15

Status:

```text
CODE IMPLEMENTED / FOCUSED AUTOMATED VERIFICATION PASS / PACKAGE-LEVEL COLCON TEST PASS / REAL-DATA A/B OBSERVED / NOT PROMOTED TO CANONICAL NAVIGATION MAP
```

This checkpoint continues from `V25_12E_CURRENT_STATE.md`

The frozen V25-12E real-data baselines remain regression evidence and are not replaced by this document

## Scope completed

V25-12F implemented and verified the following safety / traversability work:

```text
site_boundary.yaml
= vehicle-permitted inner greenhouse perimeter
= hard footprint-level safety invariant

CorridorRefinementResult.aisle_geometric_envelope
= structurally valid aisle geometry retained before Ground filtering

rich traversability evidence
OBSERVED_FREE
INFERRED_TRAVERSABLE
HARD_BLOCKED
SENSOR_OBSTACLE
UNKNOWN

12F candidate artifacts
traversability_evidence.yaml
traversability_evidence.npz
navigation_map_12f.yaml
navigation_map_12f.pgm
navigation_map_12f_derivation.yaml

shared Site Boundary gate
Vehicle-Safe Lane
Forward Connector Navigation Gate
R6B bounded reverse primitive search

Route Debug
12F A/B preset
Site Boundary / rich-state / aisle-geometric-envelope overlays
```

The canonical `navigation_map.yaml`, `navigation_map.pgm`, and `derivation.yaml` are not overwritten by candidate export

## Frozen safety semantics

```text
site_boundary
= vehicle-permitted inner perimeter
!= wall centerline
```

The polygon already includes wall thickness / wall uncertainty / desired wall-side stand-off

Continuous footprint policy:

```text
strictly inside boundary -> may continue to other gates
touch boundary           -> SITE_BOUNDARY_CONFLICT
cross boundary           -> SITE_BOUNDARY_CONFLICT
```

A FREE raster cell can never override this continuous hard-boundary rule

A READY `site_boundary.yaml` is mandatory for a 12F candidate

## A-first traversability rule

The first 12F experiment deliberately remains conservative:

```text
current FREE       -> may remain OBSERVED_FREE
current UNKNOWN    -> may be recovered only by bounded longitudinal inference
current OCCUPIED   -> remains blocked
```

Recovery requires the candidate cell to remain inside the structural aisle domain, inside Site Boundary, outside semantic NO_GO and row structure, with longitudinal OBSERVED_FREE support and bounded missing span

No generic 2D morphology and no global UNKNOWN -> FREE conversion are allowed

## Real-data A/B conclusion

The corrected real-data rerun is recorded in:

```text
docs/v2.5/V25_12F_REAL_AB_2026-08-15.md
```

Observed Vehicle-Safe Lane summary remained unchanged between Current and Candidate:

```text
READY          1 -> 1
PARTIAL        4 -> 4
UNAVAILABLE   14 -> 14
mean coverage delta = 0
```

Corrected Site Boundary diagnostics showed:

```text
current fully-blocked aisles   = 0
candidate fully-blocked aisles = 0
```

Therefore Site Boundary is not the dominant cause of low lane feasibility on this greenhouse dataset

The first UNKNOWN-only recovery also does not materially improve Vehicle-Safe Lane feasibility

The dominant remaining limitation is still upstream configuration-space semantics:

```text
Navigation Grid OCCUPIED
+ obstacle padding
+ preview footprint interaction
```

The candidate is therefore not promoted to the canonical Navigation Map

R6B search-budget tuning is not justified by this result

## Regression probes

The frozen connector probes remain behaviorally stable under the first 12F candidate:

```text
connector_015
REVERSE_PRIMITIVE_PREVIEW_FREE
FORWARD -> REVERSE -> FORWARD
2 cusps
approximately 4.499 m total
approximately 0.600 m reverse

connector_017
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
```

No R6B default search parameter was relaxed for this result

## Verification evidence

Operator-machine focused regression:

```text
26 passed
12 passed
focused total = 38 passed
```

Package-level verification was then run through `colcon test` for:

```text
agt_offline_assets
agt_map_workbench
```

Initial package-level execution exposed two legacy test fixtures that had not supplied the new required `aisle_geometric_envelope` field:

```text
test_vehicle_corridor
test_agricultural_aisle_graph
```

Only the synthetic test fixtures were updated; production planning / traversability code was not changed for that fix

After the fixture update, the operator reran the focused failures and scoped package-level tests and reported all selected tests passing

Therefore the V25-12F package-level gate is accepted as PASS for the two affected packages

Historical failures from unrelated packages present in the workspace-wide `build/` tree are not part of this scoped V25-12F gate

## Final V25-12F decision

```text
Site Boundary hard safety invariant       KEEP
Aisle geometric envelope                  KEEP
Rich traversability sidecar               KEEP
Route Debug 12F A/B                       KEEP
UNKNOWN-only recovery candidate           EXPERIMENT COMPLETE / NOT PROMOTED
R6B parameter tuning                      DO NOT USE AS COMPENSATION
```

V25-12F is complete as a safety/traversability experiment and implementation increment

Next design increment:

```text
V25-12G Maximum Feasible Coverage
```

Its objective is not mandatory full-aisle coverage

It should operate on vehicle-feasible aisle segments and connector feasibility, allow bounded forward/reverse motion, explicitly skip infeasible segments with reasons, and maximize useful executable coverage under hard vehicle / map / Site Boundary constraints
