# V25-12G-A1 Current State

Date: 2026-08-16

Status:

```text
CODE IMPLEMENTED / FOCUSED AUTOMATED VERIFICATION PASS / PACKAGE-LEVEL COLCON TEST PASS / REAL-DATA A1 OBSERVED / SUPPORTED WITH MAP/FOOTPRINT CAVEATS / NOT ROUTE-READY
```

This checkpoint starts the V25-12G Maximum Feasible Coverage implementation series after the frozen V25-12F safety / traversability experiment

The immutable runtime extraction baseline remains:

```text
v2.5-runtime-extraction-base
-> 0b2a0f1947f66629dcbdf623206ea398dde3e676
```

The tag is a historical runtime-extraction checkpoint only; V25-12G-A1 continues on the active feature branch

## A1 objective

V25-12G-A1 replaces whole-aisle executability as the only useful coverage primitive with explicit vehicle-feasible contiguous aisle segments

The A1 objective is deliberately narrower than the full V25-12G optimizer:

```text
structural aisle evidence
+ frozen vehicle / map / Site Boundary safety semantics
-> shared lane-feasibility trace
-> every maximal contiguous feasible run
-> active VehicleFeasibleSegment when length >= minimum_contiguous_span_m
-> rejected short fragment diagnostics otherwise
```

A1 does not choose a global route, generate headland connectors, optimize coverage order, or promote any navigation asset to canonical status

## Frozen A1 data contract

The serialized schema remains:

```text
agt_vehicle_feasible_segment_plan/v1
```

No schema change was introduced by Workbench visualization

For each active segment, the frozen artifact preserves explicit geometry and endpoint semantics including:

```text
segment_id
aisle_id
ordinal_in_aisle
start_distance_m
end_distance_m
length_m
coverage_fraction_of_aisle
low_endpoint_type
high_endpoint_type
centerline_xyz
lateral_offsets_m
maximum_used_lateral_shift_m
low_endpoint_pose
high_endpoint_pose
```

Endpoint classes remain:

```text
LOW_U_HEADLAND
HIGH_U_HEADLAND
INTERIOR_BLOCKED_END
```

Rejected feasible fragments preserve only non-spatial diagnostics such as IDs, longitudinal distances, length, and rejection reason

The Workbench therefore does not invent, interpolate, or reconstruct rejected-fragment XY / XYZ geometry

## Shared feasibility trace

The legacy `VehicleSafeLanePlan` longest-only behavior remains backward compatible

V25-12G-A1 extracts the common vehicle lane-feasibility trace into a shared module and derives all useful contiguous runs from that same trace

The active threshold remains:

```text
minimum_contiguous_span_m = 1.0 m
```

A contiguous feasible run at or above the threshold becomes an active `VehicleFeasibleSegment`

Shorter feasible runs remain diagnostic-only `RejectedFeasibleFragment`s

The extraction remains fail-closed for invalid frame / boundary / geometry / structural-width / navigation-grid safety conditions

## Deterministic YAML artifact

A1 provides deterministic serialization and strict loading through:

```text
vehicle_feasible_segment_plan_to_dict(plan)
write_vehicle_feasible_segment_plan(plan, path, overwrite=False)
load_vehicle_feasible_segment_plan(path)
```

The serialized artifact includes the frozen vehicle-safe-lane configuration snapshot and deterministic summary fields

The strict loader rejects malformed schema, missing fields, duplicate IDs, invalid segment ordering / overlap, invalid endpoint types, invalid threshold semantics, invalid point dimensions, and inconsistent summary data

## Real-run acceptance harness

The A1 diagnostic acceptance harness is:

```text
tools/v25_12g_a1_acceptance.py
```

Its validation scope is explicitly:

```text
A1_SEGMENT_EXTRACTION_DIAGNOSTIC_NOT_ROUTE_READY
```

It compares the old longest-only Vehicle-Safe Lane interpretation and the new segment-preserving interpretation from the same input evidence

The harness can report:

```text
structural aisle length
raw feasible fragment count
active segment count
active feasible length
old longest-only span
recoverable additional feasible length
segment IDs
endpoint classes
```

Optional write mode creates only:

```text
vehicle_feasible_segments.yaml
```

It must not modify canonical Navigation Map / derivation files

## Workbench review-only visualization

The Workbench layer is:

```text
key   = vehicle_feasible_segments
label = 12G-A1 Vehicle-Feasible Segments
```

The renderer consumes only the loaded `VehicleFeasibleSegmentPlan`

For every active segment it creates one review-only path from explicit `centerline_xyz` using scene coordinates:

```text
(x, -y)
```

A1 graphics use fixed z-order:

```text
12.0
```

which remains above raster navigation preview and below Site Boundary / authoring graphics

All A1 graphics are non-movable and non-selectable

Rejected fragments remain non-spatial diagnostics only

Workbench behavior is frozen as:

```text
missing vehicle_feasible_segments.yaml
-> quiet / no A1 error

invalid vehicle_feasible_segments.yaml
-> local A1 error only
-> legacy Workbench remains usable

source PCD changes
-> stale A1 plan / graphics cleared before the next sibling result

A1 selected + Navigation Overlay enabled + valid plan
-> A1 graphics visible
-> raster NavigationPreview hidden for this dispatch

switch to another navigation layer
-> A1 graphics hidden

A1 selected + Navigation Overlay disabled
-> A1 graphics hidden

enter Route Debug
-> A1 graphics hidden
-> loaded A1 plan retained

leave Route Debug
-> normal navigation dispatch re-evaluated from current layer / overlay / plan state
-> no stale A1 visibility snapshot restoration
```

Workbench never calls `derive_vehicle_feasible_segment_plan`

Workbench also does not load `aisle_graph.yaml` merely to reconstruct rejected fragments

## Automated verification evidence

Operator-machine focused Workbench regression on 2026-08-16:

```text
22 passed in 1.02s
```

The focused set covered:

```text
test_vehicle_feasible_segment_preview.py
test_vehicle_feasible_segment_workbench.py
test_site_boundary_workbench.py
test_route_debug_view.py
test_route_debug_panel.py
```

Package-level registered verification for `agt_map_workbench` then reported:

```text
8 / 8 registered tests passed
100% tests passed
0 tests failed
```

The registered A1 Workbench test itself reported:

```text
9 passed in 0.81s
```

Workspace-wide historical failures from unrelated packages, including third-party `livox_ros_driver2` lint / formatting results, are not part of this package-scoped A1 gate

Therefore the focused and package-level automated verification gates for V25-12G-A1 Workbench integration are accepted as PASS

## Real-data evidence checkpoint

The real greenhouse A1 experiment is frozen in:

```text
docs/v2.5/V25_12G_A1_REAL_DATA_2026-08-16.md
```

The five diagnostic aisles `aisle_016` through `aisle_020` produced:

```text
structural length total                131.096874 m
legacy longest-only selected span      69.718350 m
A1 active segment length               97.356674 m
recoverable additional segment length  27.638324 m
active segments                        14
rejected short fragments               23
A1 segment recovery fraction            0.742632
```

The 23 rejected feasible fragments total approximately 2.893816 m and are predominantly very short feasible islands below the frozen 1.0 m threshold

The narrow real-run write created only:

```text
vehicle_feasible_segments.yaml
```

Operator SHA256 verification reported all frozen canonical/input assets unchanged after the write:

```text
navigation_map.yaml  OK
navigation_map.pgm   OK
derivation.yaml      OK
aisle_graph.yaml     OK
site_boundary.yaml   OK
```

Workbench review showed the active A1 geometry aligned with the real greenhouse aisle structure without gross transform, cross-row, or boundary-scale errors

`aisle_020` remained the clean one-segment LOW_U_HEADLAND -> HIGH_U_HEADLAND control case

`aisle_016`, `aisle_017`, `aisle_018`, and especially `aisle_019` retained spatially credible active geometry but exposed substantial fragmentation in the frozen configuration-space evidence

A diagnostic-only sensitivity probe over 277 blocked longitudinal samples found:

```text
recovered when Site Boundary omitted          45 / 277
recovered when extra 0.05 m padding removed 148 / 277
```

These counterfactual probes overlap and are not independent causal categories

They do show that padding / Navigation Grid / base-footprint interaction is more broadly influential than Site Boundary in this diagnostic subset, while Site Boundary remains a valid hard safety invariant and is locally material in some aisle regions

One provenance caveat remains recorded:

```text
derivation.yaml frame_id = source_map
aisle_graph / site_boundary / A1 frame_id = map
```

Code-path review and the real Workbench overlay indicate shared numeric geometry with inconsistent metadata naming rather than an observed rigid-transform error

The real-data classification is therefore:

```text
A1 SEGMENT REPRESENTATION SUPPORTED WITH MAP/FOOTPRINT CAVEATS
```

This does not authorize removing Site Boundary, changing the frozen 0.05 m padding, or treating A1 as route-ready

## Current A1 decision

```text
shared lane-feasibility trace                  KEEP
all contiguous feasible segment extraction     KEEP
1.0 m active-segment threshold                 KEEP
short-fragment non-spatial diagnostics         KEEP
deterministic vehicle_feasible_segments.yaml   KEEP
A1 diagnostic acceptance harness               KEEP
Workbench review-only segment layer             KEEP
real-data segment representation                ACCEPT WITH MAP/FOOTPRINT CAVEATS
Site Boundary hard invariant                    KEEP
0.05 m preview padding                          KEEP FROZEN AT A1 CHECKPOINT
frame metadata inconsistency                    RECORD FOR SEPARATE CORRECTION
rejected-fragment geometry reconstruction       DO NOT ADD
Workbench A1 derivation                         DO NOT ADD
canonical map mutation                          DO NOT ADD
route / connector / ordering semantics          NOT IN A1
```

## A1 gate status

A1 is accepted as a real greenhouse segment-representation checkpoint with explicit map/footprint caveats

The evidence supports preserving all useful contiguous feasible runs rather than retaining only one longest span per aisle

The evidence does not prove that separate segments can reach one another or that every active segment can be serviced from the vehicle start state

That connectivity question belongs to the next design increment

## Next step

```text
V25-12G-A2 Service / Connectivity Graph Design
```

A2 may now use A1 active segments as service primitives, but it must independently model reachable endpoints and legal service transitions

In particular:

```text
HEADLAND endpoints
-> may become cross-aisle connector candidates subject to later feasibility gates

INTERIOR_BLOCKED_END
-> cannot imply cross-row connectivity
-> may terminate service
-> requires separately validated dead-end return semantics if used by future routing
```

No A2 implementation is part of the A1 checkpoint
