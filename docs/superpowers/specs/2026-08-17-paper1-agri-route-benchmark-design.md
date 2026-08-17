# Paper I Agricultural Route Benchmark Design

Date: 2026-08-17
Branch: `feat/paper1-agri-route-benchmark`
Base: `feat/v25-12g-maximum-feasible-coverage`

## 1. Goal

Build a reproducible research benchmark for real-greenhouse agricultural route planning on one canonical Ackermann platform (`mk_mini`) and one canonical site package (`greenhouse_01`). The benchmark must separate point-to-point path planning from agricultural mission route planning, reuse the existing V2.5 map/route/coverage contracts, and produce both simulator-visible routes and paper-ready artifacts.

The research question is not whether A* is generally inadequate. It is whether free-space shortest-path formulations remain sufficient after progressively adding Ackermann kinematics, footprint feasibility, agricultural semantics, reachable coverage, and task-level visit ordering.

## 2. Ownership split

### User-owned inputs and acceptance

The user provides and validates:

1. the real greenhouse PCD;
2. the derived 2D occupancy grid reliability;
3. the correctness of site semantic annotations.

The benchmark treats these accepted assets as immutable experiment inputs for a given site revision.

### Benchmark-owned responsibilities

This branch owns:

1. baseline task definitions;
2. unified planner/scenario contracts;
3. metric computation;
4. path CSV export;
5. RViz/simulation visualization;
6. paper figure export inputs;
7. experiment manifests and result lineage;
8. the later RPP execution handoff for real-vehicle validation.

## 3. Scope boundary

### In scope

- PCD-derived occupancy/traversability assets already produced by V2.5 tools;
- semantic row/headland/access/keepout geometry;
- canonical MKmini platform profile;
- point-to-point global planning baselines;
- Fields2Cover/OpenNav coverage baseline;
- proposed maximum-feasible agricultural route planner;
- static feasibility validation;
- CSV/JSON/GeoJSON/SVG/PDF/PNG export;
- simulation/RViz visualization;
- later Nav2 `FollowPath` / RPP execution handoff.

### Explicitly out of scope for Paper I

- SLAM/localization algorithm novelty;
- relocalization/fusion/GNSS;
- rolling ESDF/local obstacle avoidance novelty;
- BT mission architecture novelty;
- chassis-control novelty;
- perception or manipulation.

These may be used as infrastructure but are not Paper I contributions.

## 4. Canonical site representation

`greenhouse_01` is represented through existing V2.5 site/map contracts rather than a second asset registry.

Required logical layers:

1. **Geometry**: occupancy map derived from the accepted PCD;
2. **Traversability**: body/clearance-aware navigable region;
3. **Agricultural semantics**: rows, headlands, access lanes, keepouts, entrances/exits, permanent obstacles;
4. **Agricultural topology**: row entrances/exits and legal transitions used by mission planning.

The benchmark must bind exact source asset identities/hashes in each experiment manifest.

## 5. Canonical robot

All formal experiments use `profiles/platforms/mk_mini.yaml` as the single source of geometry and kinematic truth.

The formal benchmark must not copy wheelbase, footprint, minimum-turning-radius, reverse policy, or safety margins into planner-specific independent constants.

If a planner needs a derived parameter, it must be deterministically derived from the profile and recorded in the result manifest.

## 6. Two experiment levels

### 6.1 Point-to-point planning

Purpose: isolate what progressively stronger generic path planners can solve when start and goal are already supplied.

Formal baselines:

- `astar`
- `theta_star`
- `hybrid_astar`
- `state_lattice`

These planners receive the same accepted map, robot profile, start/goal, footprint policy, unknown-space policy, timeout, and resolution.

### 6.2 Agricultural mission planning

Purpose: evaluate who determines what regions to visit, in what order, through which legal transitions, and with what task coverage.

Formal baselines:

- `manual_waypoints_best_p2p`
- `fields2cover`
- `ours`

`manual_waypoints_best_p2p` is the control representing the common engineering practice of manually choosing task waypoints and repeatedly invoking a generic P2P planner.

`ours` performs reachable-task selection, semantic/topological ordering, and kinematically feasible connection generation.

## 7. Frozen scenario matrix

The canonical scenario IDs are:

- `S01_straight_row`
- `S02_90deg_entry`
- `S03_headland_uturn`
- `S04_narrow_headland`
- `S05_blocked_row`
- `S06_full_mission`

The first five primarily expose isolated constraints. `S06_full_mission` is the complete agricultural mission benchmark.

Each scenario file must contain explicit start/goal or mission intent, semantic expectations, expected reachable task set, and evaluation tolerances. Scenario files may reference site semantic object IDs but must not encode a second copy of map geometry.

## 8. Frozen planner matrix

The paper matrix remains six named planners/methods:

1. A*
2. Theta*
3. Hybrid A*
4. State Lattice
5. Fields2Cover
6. Proposed method

The implementation may additionally use Reeds-Shepp/Dubins as connectors or diagnostics, but they are not added as formal matrix columns unless the paper design is explicitly revised.

## 9. Common result contract

Every planner run emits one result directory containing at minimum:

- `experiment_manifest.json`
- `metrics.json`
- `path.csv`
- `path.geojson`
- `planner_report.json`
- `figure.svg`
- `figure.png`

PDF figure export is required when the plotting backend supports it.

`path.csv` uses a stable schema:

```text
index,x_m,y_m,yaw_rad,direction,segment_type,semantic_ref
```

Allowed `direction`: `F`, `R`, `UNKNOWN`.

Allowed `segment_type` includes at least `P2P`, `SWATH`, `CONNECTION`, `ACCESS`, `TURN`, `UNKNOWN`.

## 10. Metrics

### 10.1 Common path metrics

- planner success/failure and stable error code;
- planning time;
- path length;
- minimum clearance;
- maximum absolute curvature;
- kinematic violation count;
- footprint collision count;
- reverse distance/count;
- semantic violation count.

### 10.2 Mission metrics

- required task coverage ratio;
- reachable task coverage ratio;
- missed reachable rows/segments;
- mission distance;
- deadhead distance;
- turn/transition count;
- reverse count/distance;
- semantic violations;
- execution eligibility.

### 10.3 Real-vehicle discussion metrics

Later RPP tests additionally record:

- execution success rate;
- lateral tracking RMSE;
- maximum lateral error;
- heading error;
- stop/abort count;
- completion time.

These real-vehicle metrics are not fabricated by simulation and remain empty/null before field tests.

## 11. Fairness rules

1. Same accepted map and semantic revision for all methods in a scenario.
2. Same canonical `mk_mini` profile.
3. Same start/goal or mission intent.
4. Same footprint and obstacle semantics where supported.
5. No planner-specific hand-edited map.
6. No hidden waypoint insertion for algorithmic methods.
7. Manual-waypoint baseline must store the exact manually authored points as an input artifact.
8. Unsupported capabilities are reported as limitations, not silently compensated by changing another method's inputs.
9. Every formal run records parameters and repository revision.

## 12. Proposed-method architecture

The proposed method is hierarchical rather than a new low-level A* variant:

```text
accepted site assets
  -> feasible agricultural task regions
  -> reachable row/segment set
  -> semantic/topological task graph
  -> route ordering
  -> legal transition selection
  -> Ackermann-feasible connector/path generation
  -> full-footprint/semantic validation
  -> export
```

Primary objective: maximize feasible agricultural task completion.

Secondary objective: among equal-coverage solutions minimize nonproductive travel and execution burden (distance, reversals, curvature/turn cost, semantic risk).

## 13. Reuse from V2.5

Extend rather than replace:

- `agt_coverage_planning` for OpenNav/Fields2Cover adaptation, semantic reconstruction, validation, repair and simulation reporting;
- existing route asset lineage and map manifest contracts;
- existing point-cloud/map-workbench/traversability assets;
- existing Hybrid-A*/Reeds-Shepp connector work;
- existing evaluation/experiment-manager patterns;
- existing map-route visualization work.

Do not fork a second site/map registry.

## 14. New benchmark layer

Add one focused package/module boundary for paper experiments, tentatively `agt_route_benchmark`, whose responsibilities are:

1. load scenario + selected planner adapter;
2. resolve canonical site/profile inputs;
3. run one deterministic experiment;
4. normalize planner output to the common path contract;
5. call shared feasibility/metric evaluators;
6. export artifacts;
7. never command chassis velocity.

Planner-specific logic remains behind adapters. Existing production packages are not rewritten to depend on paper code.

## 15. Visualization

Formal plots use one consistent site background and coordinate frame.

Required views:

- single-run route overlay;
- six-method comparison panel/input set;
- collision/curvature/semantic violation markers;
- reachable versus unreachable task regions;
- full-mission route with row IDs and transition semantics.

RViz remains an interactive engineering view. Paper export is deterministic and does not rely on screenshots.

## 16. One-week acceptance target

The implementation is considered ready for the first simulation discussion when:

1. one command selects `site + scenario + planner`;
2. S01-S06 scenario contracts validate;
3. the four P2P baselines have adapters/configs even if an external Nav2 dependency is required at runtime;
4. Fields2Cover uses the existing coverage stack;
5. `ours` can produce a maximum-feasible mission route from accepted semantics;
6. every successful run exports the stable path CSV plus metrics;
7. RViz or offline renderer displays the route on the canonical map;
8. batch execution produces a comparison summary;
9. real-vehicle RPP execution consumes the same exported/validated route asset without manually retyping waypoints.

## 17. Failure behavior

- Missing site/profile/scenario assets fail closed with explicit error codes.
- Invalid semantic references fail before planning.
- Empty/invalid planner paths are still recorded as failed experiment results.
- Formal export never marks a path execution-eligible unless full footprint and configured semantic checks pass.
- Results are immutable per experiment ID; reruns create a new result identity or explicit overwrite only in non-formal development mode.

## 18. Paper claim boundary

The intended evidence chain is:

1. 2D/free-space search solves supplied A-to-B geometry but does not define the agricultural mission;
2. any-angle planning removes grid-direction artifacts but not kinematic/task limitations;
3. Hybrid A*/State Lattice improve Ackermann feasibility;
4. coverage planning adds agricultural coverage structure;
5. real greenhouse operation additionally requires extracting/accepting traversable structure, selecting the feasible task set, enforcing semantic/legal transitions, and ordering a complete mission;
6. the proposed hierarchy addresses that final formulation and is evaluated without altering the accepted site map per planner.

The paper must not claim that A*, Hybrid A*, State Lattice, or Fields2Cover are generally incapable algorithms. Claims are restricted to the frozen formulation, inputs, constraints, and measured scenarios above.
