# Paper I Agricultural Route Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reproducible, paper-oriented benchmark layer that runs the frozen greenhouse route-planning matrix on one canonical MKmini profile, exports normalized paths/metrics/figures, and hands validated CSV routes to later RPP field execution.

**Architecture:** Add a focused `agt_route_benchmark` ROS2/Python package. It does not replace the existing V2.5 asset system or production planners; instead it resolves existing site/profile assets, dispatches planner adapters, normalizes every result into one path/result schema, evaluates it, and exports deterministic artifacts. P2P planning and agricultural mission planning remain separate experiment levels.

**Tech Stack:** ROS2 Humble, Python 3, `ament_cmake_python`, PyYAML, NumPy, matplotlib for deterministic paper rendering, Nav2 planner interfaces for generic P2P baselines, existing `agt_coverage_planning` / OpenNav / Fields2Cover stack for coverage, existing V2.5 map/semantic/profile contracts.

## Global Constraints

- Formal site: `greenhouse_01`.
- Formal vehicle: `profiles/platforms/mk_mini.yaml`.
- Formal planner matrix: `astar`, `theta_star`, `hybrid_astar`, `state_lattice`, `fields2cover`, `ours`.
- Formal scenario matrix: `S01_straight_row`, `S02_90deg_entry`, `S03_headland_uturn`, `S04_narrow_headland`, `S05_blocked_row`, `S06_full_mission`.
- Reuse V2.5 map manifest, semantic-map, route-asset and Site Package lineage; do not create a second site/map registry.
- Formal experiments must not silently alter accepted PCD-derived occupancy or semantic assets per planner.
- Vehicle geometry/kinematics must come from the canonical platform profile rather than planner-local duplicated constants.
- Successful normalized path CSV schema is exactly `index,x_m,y_m,yaw_rad,direction,segment_type,semantic_ref`.
- Failed planner runs still emit manifest/report/metrics with a stable failure code.
- No benchmark node publishes chassis velocity.
- Real-vehicle RPP tracking metrics remain null until field data exists.

---

## File Structure

Create package:

```text
src/agt_route_benchmark/
├── CMakeLists.txt
├── package.xml
├── README.md
├── agt_route_benchmark/
│   ├── __init__.py
│   ├── contracts.py
│   ├── scenario.py
│   ├── path_io.py
│   ├── metrics.py
│   ├── experiment.py
│   ├── renderer.py
│   ├── batch.py
│   └── adapters/
│       ├── __init__.py
│       ├── base.py
│       ├── nav2_p2p.py
│       ├── fields2cover.py
│       ├── manual_waypoints.py
│       └── proposed.py
├── scripts/
│   ├── route_benchmark_run.py
│   ├── route_benchmark_batch.py
│   └── route_csv_to_path.py
├── config/
│   └── benchmark.yaml
├── scenarios/
│   ├── S01_straight_row.yaml
│   ├── S02_90deg_entry.yaml
│   ├── S03_headland_uturn.yaml
│   ├── S04_narrow_headland.yaml
│   ├── S05_blocked_row.yaml
│   └── S06_full_mission.yaml
├── launch/
│   ├── benchmark_run.launch.py
│   └── benchmark_visualize.launch.py
├── rviz/
│   └── route_benchmark.rviz
└── test/
    ├── test_contracts.py
    ├── test_scenario.py
    ├── test_path_io.py
    ├── test_metrics.py
    ├── test_experiment.py
    ├── test_renderer.py
    └── test_batch.py
```

The package may consume accepted site assets from existing runtime/site-package paths but must not embed a duplicate copy of the real map.

---

### Task 1: Common contracts and scenario validation

**Files:**
- Create: `src/agt_route_benchmark/CMakeLists.txt`
- Create: `src/agt_route_benchmark/package.xml`
- Create: `src/agt_route_benchmark/agt_route_benchmark/__init__.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/contracts.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/scenario.py`
- Create: `src/agt_route_benchmark/config/benchmark.yaml`
- Create: `src/agt_route_benchmark/scenarios/S01_straight_row.yaml`
- Create: `src/agt_route_benchmark/scenarios/S02_90deg_entry.yaml`
- Create: `src/agt_route_benchmark/scenarios/S03_headland_uturn.yaml`
- Create: `src/agt_route_benchmark/scenarios/S04_narrow_headland.yaml`
- Create: `src/agt_route_benchmark/scenarios/S05_blocked_row.yaml`
- Create: `src/agt_route_benchmark/scenarios/S06_full_mission.yaml`
- Test: `src/agt_route_benchmark/test/test_contracts.py`
- Test: `src/agt_route_benchmark/test/test_scenario.py`

**Interfaces:**
- Produces: `ExperimentSpec`, `ScenarioSpec`, `PathPoint`, `PlannerResult`, `load_scenario(path: Path) -> ScenarioSpec`, `validate_matrix_name(name: str, allowed: tuple[str, ...]) -> str`.
- Consumes: canonical site/profile paths supplied at runtime.

- [ ] **Step 1: Write failing contract tests**

```python
from agt_route_benchmark.contracts import PathPoint, P2P_PLANNERS, SCENARIO_IDS


def test_frozen_matrix_names_are_stable():
    assert P2P_PLANNERS == ("astar", "theta_star", "hybrid_astar", "state_lattice")
    assert SCENARIO_IDS[-1] == "S06_full_mission"


def test_path_point_rejects_bad_direction():
    try:
        PathPoint(0.0, 0.0, 0.0, "SIDEWAYS", "P2P", "")
    except ValueError as exc:
        assert "direction" in str(exc)
    else:
        raise AssertionError("invalid direction accepted")
```

- [ ] **Step 2: Run tests and verify failure**

Run: `colcon test --packages-select agt_route_benchmark --event-handlers console_direct+`
Expected: package/tests unavailable or import failure.

- [ ] **Step 3: Implement the minimal dataclasses/constants and YAML loader**

`contracts.py` must define frozen planner/scenario identifiers and fail-closed validation. `scenario.py` must distinguish `level: p2p` from `level: mission`, require `start`/`goal` for P2P, require `mission.required_semantic_ids` for mission scenarios, and preserve semantic IDs without inventing geometry.

- [ ] **Step 4: Add six schema-valid scenario templates**

The templates encode intent and semantic references only. Coordinates may be explicitly marked `development_fixture: true` until the user accepts the real `greenhouse_01` map/semantics; formal batch mode must reject development fixtures.

- [ ] **Step 5: Run tests and commit**

Run: `colcon test --packages-select agt_route_benchmark --event-handlers console_direct+`
Expected: contract/scenario tests PASS.

Commit: `feat(route-benchmark): add frozen experiment contracts`

---

### Task 2: Stable path/result export

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/path_io.py`
- Test: `src/agt_route_benchmark/test/test_path_io.py`

**Interfaces:**
- Consumes: `Sequence[PathPoint]`.
- Produces: `write_path_csv(points, path)`, `read_path_csv(path)`, `write_path_geojson(points, path)`, `write_json_atomic(payload, path)`.

- [ ] **Step 1: Write failing round-trip tests**

```python
from pathlib import Path
from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.path_io import read_path_csv, write_path_csv


def test_csv_schema_and_round_trip(tmp_path: Path):
    src = [PathPoint(1.0, 2.0, 0.5, "F", "SWATH", "row_01")]
    out = tmp_path / "path.csv"
    write_path_csv(src, out)
    assert out.read_text().splitlines()[0] == "index,x_m,y_m,yaw_rad,direction,segment_type,semantic_ref"
    assert read_path_csv(out) == src
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest -q src/agt_route_benchmark/test/test_path_io.py`
Expected: import/function failure.

- [ ] **Step 3: Implement CSV/GeoJSON/atomic JSON export**

Use deterministic key ordering and UTF-8. Reject NaN/Inf coordinates and empty successful paths.

- [ ] **Step 4: Run tests and commit**

Run: `pytest -q src/agt_route_benchmark/test/test_path_io.py`
Expected: PASS.

Commit: `feat(route-benchmark): add stable path artifact export`

---

### Task 3: Geometry-independent benchmark metrics

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/metrics.py`
- Test: `src/agt_route_benchmark/test/test_metrics.py`

**Interfaces:**
- Consumes: normalized `PathPoint` sequences plus optional externally computed `clearance_m`, `footprint_collision_count`, `semantic_violation_count`, `required_semantic_ids`, `visited_semantic_ids`.
- Produces: `compute_path_metrics(...) -> dict`, `compute_mission_metrics(...) -> dict`.

- [ ] **Step 1: Write failing metric tests**

```python
import math
from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.metrics import compute_path_metrics


def test_length_reverse_and_curvature_metrics():
    pts = [
        PathPoint(0.0, 0.0, 0.0, "F", "P2P", ""),
        PathPoint(1.0, 0.0, 0.0, "F", "P2P", ""),
        PathPoint(2.0, 0.0, math.pi / 4, "R", "TURN", ""),
    ]
    m = compute_path_metrics(pts)
    assert abs(m["path_length_m"] - 2.0) < 1e-9
    assert m["reverse_segment_count"] == 1
    assert m["reverse_distance_m"] == 1.0
```

- [ ] **Step 2: Verify failure, then implement**

Curvature must use heading change over traveled distance with angle wrapping; zero-length segments must not divide by zero and must be reported in diagnostics.

- [ ] **Step 3: Add mission coverage/deadhead tests**

Require separate `required_task_coverage_ratio` and `reachable_task_coverage_ratio` so physically unreachable rows do not disappear from the discussion.

- [ ] **Step 4: Run tests and commit**

Run: `pytest -q src/agt_route_benchmark/test/test_metrics.py`
Expected: PASS.

Commit: `feat(route-benchmark): add paper metric computation`

---

### Task 4: Planner adapter boundary and P2P baselines

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/adapters/__init__.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/adapters/base.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/adapters/nav2_p2p.py`
- Create: `src/agt_route_benchmark/scripts/route_benchmark_run.py`
- Create: `src/agt_route_benchmark/launch/benchmark_run.launch.py`
- Test: `src/agt_route_benchmark/test/test_experiment.py`

**Interfaces:**
- Produces: `PlannerAdapter.plan(spec: ExperimentSpec) -> PlannerResult` and `Nav2P2PAdapter(planner_id, nav2_plugin_id)`.
- P2P adapter mapping is explicit:
  - `astar` -> Nav2 2D A*/NavFn-compatible configured baseline;
  - `theta_star` -> ThetaStar planner plugin;
  - `hybrid_astar` -> Smac Hybrid-A* plugin;
  - `state_lattice` -> Smac State Lattice plugin.

- [ ] **Step 1: Write a fake-adapter experiment test**

The orchestrator test must run without ROS/Nav2 by injecting a fake adapter and assert that planner failure still creates a result report.

- [ ] **Step 2: Implement adapter protocol and experiment orchestrator**

Do not embed ROS messages in `contracts.py`. ROS conversion stays inside adapters/scripts.

- [ ] **Step 3: Implement Nav2 adapter configuration mapping**

The adapter must record plugin ID, timeout, start/goal, and received path. It must never silently fall back from one baseline to another.

- [ ] **Step 4: Add launch-time arguments**

Required arguments: `site`, `scenario`, `planner`, `platform_profile`, `result_root`, `formal`.

- [ ] **Step 5: Run unit tests and commit**

Run: `pytest -q src/agt_route_benchmark/test/test_experiment.py`
Expected: PASS without Nav2 runtime.

Commit: `feat(route-benchmark): add P2P planner adapter boundary`

---

### Task 5: Mission-level baselines and proposed planner skeleton

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/adapters/manual_waypoints.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/adapters/fields2cover.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/adapters/proposed.py`
- Test: extend `src/agt_route_benchmark/test/test_experiment.py`

**Interfaces:**
- `ManualWaypointAdapter` consumes an explicit waypoint artifact and a selected P2P adapter.
- `Fields2CoverAdapter` wraps existing `agt_coverage_planning` output and preserves SWATH/CONNECTION semantics.
- `ProposedAdapter` consumes accepted semantic/topological task objects and a connector callback; produces normalized segments with semantic references.

- [ ] **Step 1: Write failing mission tests with tiny synthetic semantic graphs**

Test cases must include: all rows reachable, one blocked/unreachable row, and a legal-headland-only transition rule.

- [ ] **Step 2: Implement manual-waypoint baseline**

The exact waypoint list must be stored in the experiment manifest; no generated waypoint may be mislabeled as manual.

- [ ] **Step 3: Implement Fields2Cover result normalizer**

Reuse the existing coverage semantic reconstruction rather than parsing visualization markers. Preserve F2C/OpenNav failure details.

- [ ] **Step 4: Implement proposed maximum-feasible route ordering core**

First version deliberately stays interpretable: construct the reachable task set from accepted semantic graph constraints, maximize visited reachable task elements, then minimize a deterministic secondary cost over legal row/headland transitions. Low-level connectors remain replaceable.

- [ ] **Step 5: Verify synthetic blocked-row behavior**

Expected: the unreachable row remains listed under `required_but_unreachable`, while reachable coverage can still equal 1.0.

- [ ] **Step 6: Run tests and commit**

Run: `pytest -q src/agt_route_benchmark/test/test_experiment.py`
Expected: PASS.

Commit: `feat(route-benchmark): add mission planning baselines`

---

### Task 6: Deterministic renderer and batch matrix

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/renderer.py`
- Create: `src/agt_route_benchmark/agt_route_benchmark/batch.py`
- Create: `src/agt_route_benchmark/scripts/route_benchmark_batch.py`
- Create: `src/agt_route_benchmark/launch/benchmark_visualize.launch.py`
- Create: `src/agt_route_benchmark/rviz/route_benchmark.rviz`
- Test: `src/agt_route_benchmark/test/test_renderer.py`
- Test: `src/agt_route_benchmark/test/test_batch.py`

**Interfaces:**
- Produces deterministic `figure.svg`, `figure.png`, optional `figure.pdf`, and `comparison.csv/json`.
- Batch key: `(site_id, site_revision, platform_profile_hash, scenario_id, planner_id, run_id)`.

- [ ] **Step 1: Write renderer tests**

Test that a normalized path plus occupancy image metadata produces SVG/PNG files and that axis limits remain map-derived rather than autoscaled differently per planner.

- [ ] **Step 2: Implement renderer**

Do not use RViz screenshots as formal paper figures. Render the same occupancy background, route, start/goal, row labels, unavailable task marks, and failure markers deterministically.

- [ ] **Step 3: Write batch completeness test**

Development mode may skip unavailable external planner runtimes but must emit explicit `SKIPPED_DEPENDENCY` results; formal mode must fail if any required cell is missing.

- [ ] **Step 4: Implement matrix runner and summary exporter**

Generate P2P and mission summaries separately; do not compare F2C path length as if it were a direct A-to-B equivalent without labeling experiment level.

- [ ] **Step 5: Run tests and commit**

Run: `pytest -q src/agt_route_benchmark/test/test_renderer.py src/agt_route_benchmark/test/test_batch.py`
Expected: PASS.

Commit: `feat(route-benchmark): add matrix runner and paper renderer`

---

### Task 7: CSV-to-NavPath/RPP handoff and operator documentation

**Files:**
- Create: `src/agt_route_benchmark/scripts/route_csv_to_path.py`
- Create: `src/agt_route_benchmark/README.md`
- Modify only if required: existing RPP/Nav2 configuration to add a benchmark-specific profile reference without changing validated defaults.

**Interfaces:**
- Consumes: a validated exported `path.csv` plus its manifest.
- Produces: `nav_msgs/Path` on a benchmark preview/execution handoff topic; execution remains separately gated by existing project safety/navigation capability.

- [ ] **Step 1: Add CSV conversion unit test**

Reject unknown frame, missing manifest compatibility, invalid quaternion/yaw, and non-execution-eligible formal results.

- [ ] **Step 2: Implement converter/publisher**

Default behavior is preview-only. Any field execution path must require an explicit execution-eligible result and existing project safety gates.

- [ ] **Step 3: Document simulation sequence**

README must include exact commands for: single planner run, six-scenario batch, artifact inspection, RViz preview, and CSV handoff.

- [ ] **Step 4: Document field-test record sheet**

Record per run: route/result ID, map/semantic/profile hashes, RPP parameters, success/failure, lateral RMSE, max lateral error, heading error, completion time, abort reason, and operator notes.

- [ ] **Step 5: Run package tests and build**

Run:

```bash
colcon build --packages-select agt_route_benchmark --symlink-install
colcon test --packages-select agt_route_benchmark --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests.

Commit: `feat(route-benchmark): add RPP route handoff and docs`

---

### Task 8: Integration acceptance on `greenhouse_01`

**Files:**
- No duplicate real map files added to the benchmark package.
- Result artifacts are generated under the existing runtime/results or configured experiment result root and remain untracked unless intentionally committed as paper evidence.

**Interfaces:**
- Consumes: user-accepted PCD-derived occupancy, semantic map, `mk_mini.yaml`.
- Produces: complete benchmark result set and paper discussion inputs.

- [ ] **Step 1: Validate user-supplied site revision**

Check map YAML/image, semantic object references, map frame consistency, profile compatibility, and absence of development fixtures.

- [ ] **Step 2: Run S01-S05 P2P matrix**

Run four P2P planners on identical scenario inputs and export all result cells.

- [ ] **Step 3: Run S06 mission matrix**

Run `manual_waypoints_best_p2p`, `fields2cover`, and `ours` on the same mission intent and accepted semantics.

- [ ] **Step 4: Generate comparison artifacts**

Produce separate P2P and mission tables/figures plus failure-case zoom figures.

- [ ] **Step 5: Simulation validation**

Preview the exported route in RViz/Gazebo and confirm the exact CSV consumed later by RPP is the validated route, not a manually reconstructed waypoint list.

- [ ] **Step 6: Field validation after site access**

Run repeated RPP tracking trials and append measured tracking/execution metrics. Do not backfill real metrics from simulation.

- [ ] **Step 7: Final verification commit**

Commit only source/config/docs and intentionally selected evidence. Preserve raw experiment results according to the repository's dataset/site lineage policy.

Commit: `test(route-benchmark): record greenhouse benchmark acceptance`
