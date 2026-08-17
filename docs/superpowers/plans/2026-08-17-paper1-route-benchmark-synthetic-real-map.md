# Paper I Route Benchmark Synthetic-to-Real Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a controlled synthetic greenhouse diagnostic benchmark, hard scenario-map preflight, endpoint discretization metrics, auditable real-map curation handoff, and paper-ready figure/claim outputs that can later be reused unchanged on the accepted `greenhouse_01` assets.

**Architecture:** Planner execution, independent path evaluation, and paper interpretation remain separate. A deterministic `synthetic_greenhouse_v1` supplies controlled S01-S05 geometry; preflight rejects invalid scenario inputs before Nav2; successful planner paths receive endpoint and feasibility metrics; a paper-bundle layer reads immutable outputs and generates vector figures plus evidence-conditioned narrative without changing planner results. The real map uses a planner-independent override layer and curation manifest before site-snapshot freezing.

**Tech Stack:** Python 3.10, ROS 2 Humble, Nav2 SmacPlanner2D / ThetaStar / SmacPlannerHybrid, NumPy, Matplotlib, Pillow, PyYAML, Shapely, GeoJSON/JSON, pytest, ament/colcon.

## Global Constraints

- Synthetic diagnostic map resolution is exactly `0.10 m/cell`.
- Canonical platform is MKmini Ackermann with `min_turning_radius = 1.500 m` from `profiles/platforms/mk_mini.yaml`.
- Synthetic results are development evidence only and must never satisfy a formal matrix cell.
- Formal map edits are planner-independent and represented only as `FORCE_FREE` / `FORCE_OCCUPIED` overrides.
- Invalid scenario input is not planner failure.
- Collision-free planner success may still be execution-infeasible after independent curvature/footprint/semantic evaluation.
- Formal assets are immutable within one benchmark revision and are bound by hashes/site snapshot.
- Paper figures export SVG/PDF/PNG from result files; RViz screenshots are never authoritative evidence.
- Paper prose is derived from observed metrics and never pre-asserts a planner ranking.
- Single-run synthetic planning time is descriptive only.
- Final acceptance requires figures and a claim-evidence narrative that are directly traceable to result metrics.

---

## File Structure

**Create core modules**

- `src/agt_route_benchmark/agt_route_benchmark/preflight.py`
- `src/agt_route_benchmark/agt_route_benchmark/endpoint_metrics.py`
- `src/agt_route_benchmark/agt_route_benchmark/synthetic_greenhouse.py`
- `src/agt_route_benchmark/agt_route_benchmark/paper_bundle.py`
- `src/agt_route_benchmark/agt_route_benchmark/map_curation.py`

**Create scripts**

- `src/agt_route_benchmark/scripts/route_benchmark_generate_synthetic.py`
- `src/agt_route_benchmark/scripts/route_benchmark_paper_bundle.py`
- `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`

**Create versioned assets**

- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.pgm`
- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.yaml`
- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/semantic.geojson`
- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/fixture_manifest.json`
- `src/agt_route_benchmark/scenarios/synthetic/S01_straight_row.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S02_90deg_entry.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S03_headland_uturn.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S04_narrow_headland.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S05_blocked_row.yaml`

**Create tests**

- `src/agt_route_benchmark/test/test_runner_ros_args.py`
- `src/agt_route_benchmark/test/test_preflight.py`
- `src/agt_route_benchmark/test/test_endpoint_metrics.py`
- `src/agt_route_benchmark/test/test_synthetic_greenhouse.py`
- `src/agt_route_benchmark/test/test_paper_bundle.py`
- `src/agt_route_benchmark/test/test_map_curation.py`

**Modify existing files**

- `src/agt_route_benchmark/scripts/route_benchmark_run.py`
- `src/agt_route_benchmark/agt_route_benchmark/experiment.py`
- `src/agt_route_benchmark/CMakeLists.txt`
- `src/agt_route_benchmark/README.md`
- `src/agt_route_benchmark/scenarios/S01_straight_row.yaml`
- `.github/workflows/paper1-route-benchmark.yml`

---

### Task 1: Preserve Target-Machine Runtime Fixes

**Files:**
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_run.py`
- Modify: `src/agt_route_benchmark/scenarios/S01_straight_row.yaml`
- Create: `src/agt_route_benchmark/test/test_runner_ros_args.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Consumes: launch_ros-added `--ros-args -r __node:=agt_route_benchmark_runner`.
- Produces: strict benchmark CLI parsing after ROS arguments are removed; legacy smoke goal `(4.0, 0.0, 0.0)`.

- [ ] **Step 1: Write the failing ROS-argument regression test**

Test a helper with:

```python
argv = [
    "route_benchmark_run.py",
    "--site", "greenhouse_01",
    "--scenario", "scenario.yaml",
    "--planner", "astar",
    "--ros-args", "-r", "__node:=agt_route_benchmark_runner",
]
assert _benchmark_cli_args(argv) == [
    "--site", "greenhouse_01",
    "--scenario", "scenario.yaml",
    "--planner", "astar",
]
```

Add a second test where `--platfrom-profile` appears before `--ros-args`; require it to remain in returned argv so normal `argparse` rejects it.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_runner_ros_args.py
```

Expected: FAIL because the branch runner still directly parses process argv.

- [ ] **Step 3: Implement narrow ROS argument removal**

Add `import sys` and:

```python
def _benchmark_cli_args(argv: list[str]) -> list[str]:
    try:
        from rclpy.utilities import remove_ros_args
        return list(remove_ros_args(args=argv)[1:])
    except ImportError:
        if "--ros-args" in argv:
            return list(argv[1:argv.index("--ros-args")])
        return list(argv[1:])
```

Parse with:

```python
args = parser.parse_args(_benchmark_cli_args(sys.argv))
```

Do not use `parse_known_args()`.

- [ ] **Step 4: Correct the legacy S01 fixture**

Change:

```yaml
goal: {x: 6.0, y: 0.0, yaw: 0.0}
```

to:

```yaml
goal: {x: 4.0, y: 0.0, yaw: 0.0}
```

Keep `development_fixture: true`.

- [ ] **Step 5: Register and verify**

Add `test_runner_ros_args` to CMake and run:

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_runner_ros_args.py
python3 -m pytest -q src/agt_route_benchmark/test
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/agt_route_benchmark/scripts/route_benchmark_run.py \
        src/agt_route_benchmark/scenarios/S01_straight_row.yaml \
        src/agt_route_benchmark/test/test_runner_ros_args.py \
        src/agt_route_benchmark/CMakeLists.txt
git commit -m "fix(route-benchmark): preserve target-machine launch fixes"
```

---

### Task 2: Add Scenario-Map Preflight and Invalid-Input Classification

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/preflight.py`
- Create: `src/agt_route_benchmark/test/test_preflight.py`
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_run.py`
- Modify: `src/agt_route_benchmark/agt_route_benchmark/experiment.py`
- Modify: `src/agt_route_benchmark/agt_route_benchmark/batch.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Produces: `PreflightResult(valid: bool, error_codes: tuple[str, ...], metadata: dict[str, object])`.
- Produces: `evaluate_p2p_preflight(scenario: ScenarioSpec, nav_map: Nav2Map, profile: PlatformProfile) -> PreflightResult`.

- [ ] **Step 1: Write failing tests**

Require these outcomes:

```python
assert goal_outside.error_codes == ("GOAL_OUT_OF_MAP",)
assert start_occupied.error_codes == ("START_OCCUPIED",)
assert footprint_collision.error_codes == ("START_FOOTPRINT_COLLISION",)
assert valid.valid is True
```

The footprint test keeps the pose reference point free while the rotated footprint polygon intersects an occupied cell.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_preflight.py
```

- [ ] **Step 3: Implement coordinate and occupancy queries**

Use `nav2_map_occupancy_data(nav_map)` and:

```python
ix = math.floor((x - nav_map.origin[0]) / nav_map.resolution_m)
iy = math.floor((y - nav_map.origin[1]) / nav_map.resolution_m)
```

Upper extent bounds are exclusive. Unknown cells are invalid under current `allow_unknown: false` policy.

Use exact hard codes:

```text
START_OUT_OF_MAP
GOAL_OUT_OF_MAP
START_OCCUPIED
GOAL_OCCUPIED
START_FOOTPRINT_COLLISION
GOAL_FOOTPRINT_COLLISION
```

- [ ] **Step 4: Implement full rotated footprint checking**

Transform each footprint vertex with:

```python
wx = x + math.cos(yaw) * fx - math.sin(yaw) * fy
wy = y + math.sin(yaw) * fx + math.cos(yaw) * fy
```

Use Shapely polygon intersection against occupied/unknown raster-cell polygons. Do not reduce the check to corners only.

- [ ] **Step 5: Run preflight before live Nav2 P2P**

In `route_benchmark_run.py`, evaluate after map/profile/scenario loading and before `_nav2_call()`.

Invalid input writes `experiment_manifest.json`, `planner_report.json`, and `metrics.json` with:

```json
{
  "success": false,
  "error_code": "INVALID_SCENARIO",
  "preflight_error_codes": ["GOAL_OUT_OF_MAP"]
}
```

No Nav2 planning request is sent.

- [ ] **Step 6: Exclude invalid scenarios from planner denominators**

Update batch classification so `INVALID_SCENARIO` is retained as an audit row but is not counted as planner success or algorithm failure.

- [ ] **Step 7: Verify**

```bash
python3 -m pytest -q \
  src/agt_route_benchmark/test/test_preflight.py \
  src/agt_route_benchmark/test/test_experiment.py \
  src/agt_route_benchmark/test/test_batch.py \
  src/agt_route_benchmark/test/test_result_selection.py
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add src/agt_route_benchmark/agt_route_benchmark/preflight.py \
        src/agt_route_benchmark/agt_route_benchmark/experiment.py \
        src/agt_route_benchmark/agt_route_benchmark/batch.py \
        src/agt_route_benchmark/scripts/route_benchmark_run.py \
        src/agt_route_benchmark/test/test_preflight.py \
        src/agt_route_benchmark/CMakeLists.txt
git commit -m "feat(route-benchmark): add scenario map preflight"
```

---

### Task 3: Add Endpoint Discretization Metrics

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/endpoint_metrics.py`
- Create: `src/agt_route_benchmark/test/test_endpoint_metrics.py`
- Modify: `src/agt_route_benchmark/agt_route_benchmark/experiment.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Produces: `compute_endpoint_deviation(start, goal, points) -> dict[str, float]`.

- [ ] **Step 1: Write failing position and angle tests**

Use the observed 0.5 m-cell smoke values:

```python
start = (0.0, 0.0, 0.0)
goal = (4.0, 0.0, 0.0)
returned_start = (0.25, 0.25, 0.0)
returned_goal = (4.25, 0.25, 0.0)
```

Require both position deviations to equal `sqrt(0.125)` within pytest tolerance.

Compare `math.pi - 0.01` with `-math.pi + 0.01` and require angular deviation approximately `0.02`.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_endpoint_metrics.py
```

- [ ] **Step 3: Implement pure metrics**

Return:

```python
{
    "start_pose_deviation_m": start_position_error,
    "goal_pose_deviation_m": goal_position_error,
    "start_heading_deviation_rad": start_heading_error,
    "goal_heading_deviation_rad": goal_heading_error,
}
```

Use:

```python
abs(math.atan2(math.sin(a - b), math.cos(a - b)))
```

for angular distance.

- [ ] **Step 4: Integrate only on successful non-empty P2P paths**

Add metrics in `ExperimentRunner.run()` after path metrics. Do not snap or rewrite returned path points.

- [ ] **Step 5: Verify and commit**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_endpoint_metrics.py
python3 -m pytest -q src/agt_route_benchmark/test

git add src/agt_route_benchmark/agt_route_benchmark/endpoint_metrics.py \
        src/agt_route_benchmark/agt_route_benchmark/experiment.py \
        src/agt_route_benchmark/test/test_endpoint_metrics.py \
        src/agt_route_benchmark/CMakeLists.txt
git commit -m "feat(route-benchmark): report endpoint discretization"
```

---

### Task 4: Generate and Freeze `synthetic_greenhouse_v1`

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/synthetic_greenhouse.py`
- Create: `src/agt_route_benchmark/scripts/route_benchmark_generate_synthetic.py`
- Create: `src/agt_route_benchmark/test/test_synthetic_greenhouse.py`
- Create: `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.pgm`
- Create: `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.yaml`
- Create: `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/semantic.geojson`
- Create: `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/fixture_manifest.json`
- Create: five files under `src/agt_route_benchmark/scenarios/synthetic/`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Produces deterministic PGM/YAML/GeoJSON and diagnostic scenarios.

- [ ] **Step 1: Write deterministic-generation tests**

Generate twice into separate temporary directories and require byte-identical SHA256 for all four generated assets. Require resolution `0.10` and all S01-S05 start/goal reference points inside map bounds.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_synthetic_greenhouse.py
```

- [ ] **Step 3: Implement fixed geometry**

Create a `260 x 200` image at `0.10 m/cell`, origin `[0.0, 0.0, 0.0]`, total `26 m x 20 m`. Use a 1.0 m occupied outer boundary.

Add occupied crop beds:

```text
bed_1 x=[5.0,7.0],   y=[4.0,16.0]
bed_2 x=[10.0,12.0], y=[4.0,16.0]
bed_3 x=[15.0,17.0], y=[4.0,16.0]
```

Add permanent obstacle:

```text
x=[12.6,14.4], y=[9.3,10.7]
```

Add the S04 narrow-headland constriction only in the right-side diagnostic region so it cannot alter S02/S03 geometry.

- [ ] **Step 4: Freeze first-pass scenario coordinates**

```yaml
# synthetic S01
start: {x: 8.5, y: 5.0, yaw: 1.57079632679}
goal: {x: 8.5, y: 13.0, yaw: 1.57079632679}

# synthetic S02
start: {x: 3.0, y: 2.0, yaw: 0.0}
goal: {x: 8.5, y: 7.0, yaw: 1.57079632679}

# synthetic S03
start: {x: 8.5, y: 15.0, yaw: 1.57079632679}
goal: {x: 13.5, y: 15.0, yaw: -1.57079632679}

# synthetic S04
start: {x: 19.0, y: 5.0, yaw: -1.57079632679}
goal: {x: 19.0, y: 7.5, yaw: 1.57079632679}

# synthetic S05
start: {x: 13.5, y: 6.0, yaw: 1.57079632679}
goal: {x: 13.5, y: 14.0, yaw: 1.57079632679}
```

Every file contains `development_fixture: true`.

- [ ] **Step 5: Generate semantic GeoJSON**

Use feature IDs:

```text
field_boundary
bed_1
bed_2
bed_3
headland_south
headland_north
permanent_obstacle_01
```

Occupancy remains authoritative for P2P collision planning; semantics support visualization/audit.

- [ ] **Step 6: Generate manifest with real hashes**

Implement:

```python
assets = {
    "map.pgm": sha256_file(output_dir / "map.pgm"),
    "map.yaml": sha256_file(output_dir / "map.yaml"),
    "semantic.geojson": sha256_file(output_dir / "semantic.geojson"),
}
manifest = {
    "schema_version": "1.0",
    "fixture_id": "synthetic_greenhouse_v1",
    "resolution_m": 0.1,
    "formal": False,
    "generator": "agt_route_benchmark.synthetic_greenhouse",
    "assets": assets,
}
```

- [ ] **Step 7: Install assets/script and verify reproducibility**

Add `maps` to installed share directories and the generator to installed programs. Then run:

```bash
python3 src/agt_route_benchmark/scripts/route_benchmark_generate_synthetic.py \
  --output src/agt_route_benchmark/maps/synthetic_greenhouse_v1
python3 -m pytest -q src/agt_route_benchmark/test/test_synthetic_greenhouse.py
```

Run the generator a second time and require `git diff --exit-code` for the generated fixture directory.

- [ ] **Step 8: Commit**

```bash
git add src/agt_route_benchmark/agt_route_benchmark/synthetic_greenhouse.py \
        src/agt_route_benchmark/scripts/route_benchmark_generate_synthetic.py \
        src/agt_route_benchmark/maps/synthetic_greenhouse_v1 \
        src/agt_route_benchmark/scenarios/synthetic \
        src/agt_route_benchmark/test/test_synthetic_greenhouse.py \
        src/agt_route_benchmark/CMakeLists.txt
git commit -m "feat(route-benchmark): add synthetic greenhouse fixture"
```

---

### Task 5: Run S02/S03 Diagnostic Cells

**Files:**
- Modify: `src/agt_route_benchmark/README.md`
- Modify implementation only if target-machine execution exposes a reproducible defect.

**Interfaces:**
- Consumes frozen synthetic map plus A*/Theta*/Hybrid live Nav2.
- Produces six diagnostic result cells.

- [ ] **Step 1: Build and test on ROS2 Humble target machine**

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_navigation agt_route_benchmark
source install/setup.bash
rm -rf /tmp/agt_route_benchmark_test_results
colcon test --packages-select agt_route_benchmark \
  --test-result-base /tmp/agt_route_benchmark_test_results \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/agt_route_benchmark_test_results --verbose
```

Expected: zero errors/failures.

- [ ] **Step 2: Run exactly six cells**

Run scenarios `S02_90deg_entry` and `S03_headland_uturn` for planners `astar`, `theta_star`, `hybrid_astar` using:

```text
map=$PWD/src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.yaml
platform_profile=$PWD/profiles/platforms/mk_mini.yaml
result_root=$PWD/runtime/results/paper1_route_benchmark_synthetic
formal=false
```

Use run IDs:

```text
diag_s02_astar_001
diag_s02_theta_001
diag_s02_hybrid_001
diag_s03_astar_001
diag_s03_theta_001
diag_s03_hybrid_001
```

- [ ] **Step 3: Verify result completeness**

Each successful output requires:

```text
experiment_manifest.json
planner_report.json
metrics.json
path.csv
path.geojson
figure.svg
figure.png
figure.pdf
```

A planner no-path still requires manifest/report/metrics and must not be relabeled as post-planning infeasibility.

- [ ] **Step 4: Record evidence fields**

For each cell retain:

```text
success
planning_time_s
path_length_m
collision_free
kinematic_feasible
execution_feasible
max_abs_curvature_1pm
required_max_curvature_1pm
min_clearance_m
reverse_distance_m
reverse_segment_count
start_pose_deviation_m
goal_pose_deviation_m
goal_heading_deviation_rad
```

Do not change the synthetic map to obtain a preferred ranking. A geometry revision must be a new fixture ID and justified by scenario validity, not planner outcome.

- [ ] **Step 5: Document commands and commit**

```bash
git add src/agt_route_benchmark/README.md
git commit -m "docs(route-benchmark): add synthetic diagnostic workflow"
```

---

### Task 6: Generate Paper-Ready Figures and Evidence-Conditioned Narrative

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/paper_bundle.py`
- Create: `src/agt_route_benchmark/scripts/route_benchmark_paper_bundle.py`
- Create: `src/agt_route_benchmark/test/test_paper_bundle.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`
- Modify: `src/agt_route_benchmark/README.md`

**Interfaces:**
- Consumes immutable result directories and map/semantic assets.
- Produces `comparison.csv`, `comparison.json`, `claims.md`, `figure_manifest.json`, and four vector figure families.

- [ ] **Step 1: Write claim-safety tests**

Test four metric cases: collision-free but kinematically infeasible; fully feasible; planner no-path; invalid scenario.

Require:

```python
assert "collision-free but not kinematically feasible" in claims
assert "A* cannot solve" not in claims
assert "Theta* is always slower" not in claims
assert "our method is superior" not in claims
```

Invalid scenarios must not contribute to algorithm-failure statements.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_paper_bundle.py
```

- [ ] **Step 3: Normalize result evidence**

Create one comparison row per run with scenario/planner, all required metrics, run ID, formal flag, and SHA256 for source `metrics.json` plus `path.csv` when present. Export deterministic CSV and JSON.

- [ ] **Step 4: Render Figure D1 — synthetic problem formulation**

Export:

```text
fig_d1_synthetic_problem.svg
fig_d1_synthetic_problem.pdf
fig_d1_synthetic_problem.png
```

Show occupancy, crop beds, headlands, permanent obstacle, and S01-S05 start/goal markers. Label it as controlled diagnostic geometry.

- [ ] **Step 5: Render Figure D2 — S02 same-map comparison**

Overlay A*, Theta*, Hybrid paths on one map. Include requested start/goal orientation arrows and an observed metric box containing path length, max curvature, frozen curvature bound `1/1.5 = 0.6666666667 1/m`, and execution-feasible status.

- [ ] **Step 6: Render Figure D3 — S03 U-turn comparison**

Use the same visual grammar as D2. Render reverse-direction markers where present. If a planner returns no path, show `NO PATH` rather than fabricating a trajectory.

- [ ] **Step 7: Render Figure D4 — feasibility evidence matrix**

Rows are S02/S03; columns are A*/Theta*/Hybrid. Each cell reports planner success, collision-free, kinematic-feasible, execution-feasible. This is the direct visual support for the distinction between geometric planning and executability.

- [ ] **Step 8: Generate evidence-conditioned `claims.md`**

Use exact conditional templates. When `collision_free` is true and `kinematic_feasible` is false:

```text
In <scenario>, <planner> returned a collision-free path, but the independent footprint/curvature evaluator classified it as kinematically infeasible under the frozen MKmini minimum turning radius.
```

When all tested planners are feasible:

```text
<scenario> did not produce a kinematic-feasibility divergence among the tested planners; this diagnostic case therefore does not support a planner-family separation claim.
```

When planning fails:

```text
<planner> did not return a valid path in <scenario>; this is reported as a no-path outcome rather than post-planning infeasibility.
```

- [ ] **Step 9: Bind figures to source hashes**

`figure_manifest.json` records figure ID and SHA256 of every input `metrics.json`, `path.csv` when present, map YAML/image, and semantic GeoJSON.

- [ ] **Step 10: Verify and commit**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_paper_bundle.py

git add src/agt_route_benchmark/agt_route_benchmark/paper_bundle.py \
        src/agt_route_benchmark/scripts/route_benchmark_paper_bundle.py \
        src/agt_route_benchmark/test/test_paper_bundle.py \
        src/agt_route_benchmark/CMakeLists.txt \
        src/agt_route_benchmark/README.md
git commit -m "feat(route-benchmark): add paper evidence bundle"
```

---

### Task 7: Define Auditable Real-Map Curation Handoff

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/map_curation.py`
- Create: `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`
- Create: `src/agt_route_benchmark/test/test_map_curation.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`
- Modify: `src/agt_route_benchmark/README.md`

**Interfaces:**
- Consumes generated map YAML/PGM, workbench override JSON, accepted map YAML/PGM, optional source PCD.
- Produces `map_curation_manifest.json` with hashes and override audit records.

- [ ] **Step 1: Write schema tests**

A valid record has:

```python
record = {
    "override_id": "override_0001",
    "edit_type": "FORCE_FREE",
    "geometry": {
        "type": "Polygon",
        "coordinates": [[[1.0, 1.0], [1.2, 1.0], [1.2, 1.2], [1.0, 1.2], [1.0, 1.0]]],
    },
    "reason": "remove raster artifact verified against source PCD",
    "evidence_category": "PCD_INSPECTION",
}
```

Allowed evidence categories are exactly:

```text
PCD_INSPECTION
SITE_PHOTO
MEASURED_STRUCTURE
KNOWN_PERMANENT_OBSTACLE
```

Reject empty reason, unknown edit type, malformed polygon, and duplicate IDs.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_map_curation.py
```

- [ ] **Step 3: Implement curation manifest with actual file hashes**

Use:

```python
manifest = {
    "schema_version": "1.0",
    "site_id": "greenhouse_01",
    "planner_independent": True,
    "source_pcd": source_pcd_record,
    "generated_map": generated_map_hash_record,
    "override_layer": override_hash_record,
    "accepted_map": accepted_map_hash_record,
}
```

Every hash record is generated from file bytes at runtime with `hashlib.sha256(path.read_bytes()).hexdigest()`. If no PCD path is supplied, `source_pcd` is `None`.

- [ ] **Step 4: Freeze CLI contract**

Require:

```bash
ros2 run agt_route_benchmark route_benchmark_map_curation.py \
  --site greenhouse_01 \
  --generated-map-yaml runtime/maps/agt_workbench_run/generated_map.yaml \
  --override-json runtime/maps/agt_workbench_run/overrides.json \
  --accepted-map-yaml runtime/maps/agt_workbench_run/accepted_map.yaml \
  --output runtime/maps/agt_workbench_run/map_curation_manifest.json
```

Support optional `--source-pcd runtime/maps/agt_workbench_run/processed.pcd`.

- [ ] **Step 5: Enforce same-raster geometry by default**

Generated and accepted maps must have identical resolution, origin, width, and height. Scheme B changes occupancy cells through an overlay; it does not silently resize/reproject the map.

- [ ] **Step 6: Verify and commit**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_map_curation.py
python3 -m pytest -q src/agt_route_benchmark/test

git add src/agt_route_benchmark/agt_route_benchmark/map_curation.py \
        src/agt_route_benchmark/scripts/route_benchmark_map_curation.py \
        src/agt_route_benchmark/test/test_map_curation.py \
        src/agt_route_benchmark/CMakeLists.txt \
        src/agt_route_benchmark/README.md
git commit -m "feat(route-benchmark): add auditable map curation manifest"
```

---

### Task 8: CI, Target-Machine Gate, and Scientific Completion Check

**Files:**
- Modify: `.github/workflows/paper1-route-benchmark.yml`
- Modify: `src/agt_route_benchmark/README.md`

**Interfaces:**
- Produces green CI/ROS tests, six diagnostic cells, and a paper bundle whose prose/figures are traceable to metrics.

- [ ] **Step 1: Run pure-Python verification**

```bash
cd ~/agt_navigation_v2
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets \
python3 -m pytest -q src/agt_route_benchmark/test

python3 -m compileall -q \
  src/agt_route_benchmark/agt_route_benchmark \
  src/agt_route_benchmark/scripts
```

Expected: zero failures/errors.

- [ ] **Step 2: Run ROS2 target-machine verification**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agt_navigation agt_route_benchmark
source install/setup.bash
rm -rf /tmp/agt_route_benchmark_test_results
colcon test --packages-select agt_route_benchmark \
  --test-result-base /tmp/agt_route_benchmark_test_results \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/agt_route_benchmark_test_results --verbose
```

Expected: zero failures/errors.

- [ ] **Step 3: Re-run synthetic S01 plus six S02/S03 cells**

Synthetic S01 must report successful, collision-free, kinematically feasible, execution-feasible straight traversal with path length approximately `8.0 m` and curvature approximately zero. Re-run all six Task 5 cells.

- [ ] **Step 4: Build the paper evidence bundle**

```bash
ros2 run agt_route_benchmark route_benchmark_paper_bundle.py \
  --result-root $PWD/runtime/results/paper1_route_benchmark_synthetic \
  --map-yaml $PWD/src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.yaml \
  --semantic-map $PWD/src/agt_route_benchmark/maps/synthetic_greenhouse_v1/semantic.geojson \
  --output $PWD/runtime/results/paper1_route_benchmark_synthetic/paper_bundle
```

Require:

```text
comparison.csv
comparison.json
claims.md
figure_manifest.json
fig_d1_synthetic_problem.svg
fig_d1_synthetic_problem.pdf
fig_d1_synthetic_problem.png
fig_d2_s02_planner_comparison.svg
fig_d2_s02_planner_comparison.pdf
fig_d2_s02_planner_comparison.png
fig_d3_s03_planner_comparison.svg
fig_d3_s03_planner_comparison.pdf
fig_d3_s03_planner_comparison.png
fig_d4_feasibility_matrix.svg
fig_d4_feasibility_matrix.pdf
fig_d4_feasibility_matrix.png
```

- [ ] **Step 5: Perform scientific self-consistency review**

Every substantive sentence in `claims.md` must map to named rows/metrics in `comparison.csv`. Reject statements that claim A* is categorically incapable, Theta* is categorically slower, Hybrid A* never violates semantics, or the proposed method is superior without direct formal evidence.

The allowed core interpretation is:

```text
A collision-free 2D geometric path is not sufficient evidence of Ackermann executability; executability must be evaluated against footprint, curvature/turning-radius, semantic, and task constraints.
```

- [ ] **Step 6: Document synthetic-to-real substitution**

README states that final paper generation reuses the same metric/figure/claim pipeline with site-snapshot-bound `greenhouse_01` results. Synthetic figures are method/diagnostic illustrations; headline quantitative results come from the accepted real map.

- [ ] **Step 7: Commit integration**

```bash
git add .github/workflows/paper1-route-benchmark.yml \
        src/agt_route_benchmark/README.md
git commit -m "test(route-benchmark): close synthetic diagnostic gate"
```

---

## Completion Gate

Do not start the formal 23-cell benchmark until all conditions are true:

- target-machine A*/Theta*/Hybrid S01 smoke chain passes
- hard preflight rejects out-of-map/occupied/footprint-collision inputs before Nav2
- endpoint discretization metrics are present on successful P2P results
- `synthetic_greenhouse_v1` regenerates deterministically at `0.10 m/cell`
- S02 and S03 each have A*/Theta*/Hybrid diagnostic results with independent feasibility evidence
- paper bundle emits SVG/PDF/PNG figures and evidence-conditioned `claims.md`
- synthetic interpretation remains descriptive and does not manufacture planner ranking
- Scheme B curation manifest binds generated/override/accepted-map hashes and optional PCD hash
- real workbench retains override records rather than destructively hiding manual edits

After this gate, the next plan freezes the actual `greenhouse_01` map/semantics, generates the State Lattice control set for the frozen resolution and `R_min`, executes 20 formal P2P cells plus 3 mission cells, generates the real paper bundle, then runs selected accepted paths on MKmini with the existing tracking recorder/evaluator.
