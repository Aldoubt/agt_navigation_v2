# Paper I Route Benchmark: Synthetic-to-Real Map Design

Date: 2026-08-17
Branch: `feat/paper1-agri-route-benchmark`
Status: Approved design

## 1. Goal

This design freezes the next phase of Paper I route-planning validation after the Nav2 runtime smoke chain passed for A*, Theta*, and Hybrid A*.

The implementation must separate two concerns:

1. A controlled synthetic greenhouse map used only to validate planner behavior and the benchmark/evaluator pipeline.
2. A real `greenhouse_01` map used for formal experiments after planner-independent map curation and asset freezing.

The scientific principle is:

> Collision-free does not imply kinematically feasible, and kinematic feasibility does not imply agriculturally executable.

The synthetic stage demonstrates that the benchmark can observe these distinctions under controlled geometry. The real stage then measures them on a frozen greenhouse map without modifying the map in response to planner outcomes.

## 2. Scope

This phase includes:

- scenario-map preflight validation
- endpoint discretization metrics
- `synthetic_greenhouse_v1` diagnostic map
- synthetic S01-S05 diagnostic scenarios
- planner-independent real-map curation contract
- auditable manual override layer for real maps
- regression tests and output contracts for the above

This phase does not include:

- formal 20-cell P2P benchmark execution
- formal 3-cell mission benchmark execution
- final State Lattice control-set generation
- final real greenhouse scenario coordinates
- vehicle field execution
- RPP tuning changes

Those remain downstream of the accepted real-map snapshot.

## 3. Two-Level Experimental Architecture

### 3.1 Synthetic diagnostic layer

Asset name:

`synthetic_greenhouse_v1`

Resolution:

`0.10 m/cell`

Purpose:

- validate the planner/evaluator/export chain under controlled geometry
- expose differences between grid-geometric paths and Ackermann-feasible paths
- exercise reverse-capable planning and obstacle detours
- debug benchmark logic without real-map noise

Synthetic results are development/diagnostic evidence only and are not formal Paper I headline results.

### 3.2 Real formal layer

Asset name:

`greenhouse_01`

Purpose:

- produce the formal P2P and mission benchmark results
- use one accepted map and one accepted semantic/topology description for every planner
- preserve planner independence of all map edits

The real formal layer begins only after the map, semantic assets, vehicle profile, scenario coordinates, and site snapshot are frozen.

## 4. Synthetic Map Geometry

`synthetic_greenhouse_v1` shall be a deterministic occupancy map large enough that all scenarios remain away from map borders by more than the MKmini footprint circumscribed radius plus a safety margin.

It shall contain five controlled structures:

1. **Straight aisle region** for S01
   - wide, obstacle-free corridor
   - no intended curvature challenge

2. **Ninety-degree aisle entry region** for S02
   - a headland-to-row transition requiring a 90-degree heading change
   - geometry shall permit a collision-free grid path
   - geometry shall allow evaluator discrimination between sharp grid corners and Ackermann-feasible curvature

3. **Wide headland U-turn region** for S03
   - adjacent aisle geometry
   - enough space for a nominal Ackermann U-turn with `R_min = 1.5 m`

4. **Narrow headland region** for S04
   - deliberately too narrow for a simple forward-only U-turn
   - enough free space for a reverse/multi-stage maneuver if the planner supports it

5. **Blocked-row / obstacle region** for S05
   - a permanent obstacle represented directly in occupancy
   - geometry permits either a legal detour or an explicit no-path outcome depending on the chosen start/goal pair

The map must not be tuned to force a predetermined planner ranking. It is designed to expose different constraint representations, not to manufacture failures.

## 5. Scenario-Map Preflight Contract

Before starting a live Nav2 P2P request, the benchmark shall validate the scenario against the selected map and platform footprint.

Required hard preflight error codes:

- `START_OUT_OF_MAP`
- `GOAL_OUT_OF_MAP`
- `START_OCCUPIED`
- `GOAL_OCCUPIED`
- `START_FOOTPRINT_COLLISION`
- `GOAL_FOOTPRINT_COLLISION`

A scenario failing any hard preflight check must be classified as an invalid scenario/input condition, not as a planner failure.

This distinction is required so benchmark statistics cannot count fixture mistakes as algorithm failures.

The preflight result shall be stored in the experiment manifest or planner report so a failed run remains auditable.

## 6. Endpoint Discretization Metrics

The benchmark shall report how far the returned planner path endpoints differ from the requested continuous poses.

Required metrics for successful P2P paths:

- `start_pose_deviation_m`
- `goal_pose_deviation_m`
- `goal_heading_deviation_rad`

Optional but permitted:

- `start_heading_deviation_rad`

Angular errors must use wrapped angular distance.

These metrics are descriptive only. They must not alter the path, snap the requested scenario pose, or change planner behavior.

Purpose:

- make grid-center quantization visible
- avoid mixing map discretization error with path geometry/kinematic quality
- enable fair interpretation across A*, Theta*, Hybrid A*, and State Lattice

## 7. Real Map Curation: Approved Scheme B

The real-map pipeline is frozen as:

```text
raw PCD
  -> ground-relative projection
  -> raw occupancy/traversability
  -> deterministic filtering
  -> manual override layer
  -> accepted navigation map
  -> semantic map
  -> agricultural topology
  -> site snapshot + hashes
  -> formal benchmark
```

Manual correction is allowed because real point-cloud projection can contain raster artifacts, sparse-return holes, vegetation-boundary noise, and jagged occupancy edges. However, corrections must be independent of planner outputs.

### 7.1 Override model

The manual correction system shall be an explicit overlay, not an undocumented destructive edit of the raw/generated map.

At minimum, each override record shall preserve:

- unique override id
- edit type: `FORCE_FREE` or `FORCE_OCCUPIED`
- geometry of the edited region
- operator-provided reason
- source evidence category, e.g. PCD inspection / site photo / measured structure / known permanent obstacle
- creation timestamp if available

The implementation may reuse the existing V25 map-workbench override mechanism where compatible.

### 7.2 Prohibited behavior

The following is prohibited for formal data preparation:

- editing the map because A*, Theta*, Hybrid A*, State Lattice, Fields2Cover, or the proposed method produced an undesirable path
- changing map geometry separately for different planners
- overwriting the generated map without retaining the override record
- hiding failed or unreachable areas by deleting them after benchmark execution

### 7.3 Accepted-map evidence

Before formal benchmarking, the accepted real map shall preserve:

- source PCD identity/hash where available
- generated pre-override map identity/hash
- override layer identity/hash
- final accepted PGM/YAML identity/hash
- semantic-map identity/hash
- platform-profile identity/hash
- site snapshot hash binding all formal assets

The accepted map is immutable for one formal benchmark revision.

## 8. Real-Map Smoothing and Jagged-Edge Policy

Automatic filtering may be used before manual override, but it must be deterministic and parameterized.

Permitted operations include conservative morphology or topology-preserving cleanup intended to remove isolated raster artifacts.

Automatic filtering must not silently erase narrow permanent obstacles, posts, or structural boundaries.

Manual smoothing/de-jagging is acceptable only when supported by scene evidence and recorded in the override layer.

The benchmark paper should describe this as planner-independent map curation, not as planner-specific tuning.

## 9. Planner Diagnostic Sequence

After implementation, diagnostic execution order is:

1. S01 smoke validation on synthetic map
2. S02 with A*, Theta*, Hybrid A*
3. S03 with A*, Theta*, Hybrid A*
4. inspect path geometry, feasibility, endpoint deviations, and failure codes
5. only after S02/S03 are stable, run S04 and S05
6. add State Lattice after the final lattice/control-set contract is ready

No synthetic result is promoted to a formal Paper I result.

## 10. Metrics and Interpretation

Existing metrics remain authoritative:

- planning success/error code
- planning time
- path length
- max curvature
- footprint collisions
- min clearance
- kinematic feasibility
- execution feasibility
- reverse distance
- contiguous reverse maneuver count
- semantic hard violations when a semantic map is supplied

This phase adds endpoint deviations and preflight status.

For failed scenarios, reporting must distinguish:

- invalid scenario/input
- planner no-path
- planner runtime/action failure
- post-planning execution infeasibility

A planner-generated collision-free path that violates the MKmini turning-radius bound remains a successful planning output but an execution-infeasible result. The benchmark must not rewrite such a result into planner failure.

## 11. Output and Reproducibility Requirements

Successful diagnostic runs continue to emit:

- `experiment_manifest.json`
- `planner_report.json`
- `metrics.json`
- `path.csv`
- `path.geojson`
- `figure.svg`
- `figure.png`
- `figure.pdf`

Invalid preflight runs may omit path and figure artifacts, but must emit sufficient manifest/report data to identify the exact invalid-input reason.

All diagnostic assets shall be clearly marked development/non-formal.

## 12. Testing Requirements

Required regression tests:

1. preflight detects goal outside map
2. preflight detects occupied start/goal
3. preflight detects footprint collision while reference point remains in a free cell
4. valid scenario passes preflight
5. endpoint position deviation is correct for grid-center quantization
6. angular deviation uses wraparound correctly
7. invalid preflight is not counted as planner algorithm failure in result classification
8. synthetic map generation is deterministic
9. synthetic S01-S05 scenario coordinates lie inside the synthetic map and pass basic reference-point preflight

Tests shall not require live Nav2 unless explicitly added as a separate target-machine integration test.

## 13. Already Observed Runtime Findings to Preserve

The implementation plan must preserve the following findings from target-machine smoke validation:

- live Nav2 planning-only runtime works for A*, Theta*, and Hybrid A*
- benchmark runner launched through `launch_ros.actions.Node` receives ROS arguments and therefore must remove ROS-specific arguments before strict benchmark CLI parsing
- the original S01 development goal `(6.0, 0.0)` was outside the `offline_test` map extent and must not be treated as an A* failure
- corrected S01 smoke target `(4.0, 0.0)` produced successful A*, Theta*, and Hybrid A* runs
- grid-based planner output on the 0.5 m fixture exposed endpoint quantization, motivating explicit endpoint-deviation metrics

These are implementation constraints, not paper conclusions.

## 14. Completion Gate for This Phase

This design phase is complete when:

- preflight and endpoint-deviation contracts are implemented and tested
- synthetic greenhouse assets are deterministic and versioned
- S02 and S03 produce complete diagnostic outputs for A*, Theta*, and Hybrid A*
- any observed infeasibility is reported by the independent evaluator rather than inferred from screenshots
- the real-map override/audit format is defined well enough that the current V25 workbench can export a frozen accepted-map revision

Only then should the project proceed to final real-map acceptance, State Lattice generation, and the formal 23-cell matrix.
