# Paper I Route Benchmark Synthetic-to-Real Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a controlled synthetic greenhouse diagnostic benchmark, hard scenario-map preflight, endpoint discretization metrics, auditable real-map curation handoff, and paper-ready figure/claim outputs that can later be reused unchanged on the accepted `greenhouse_01` assets.

**Architecture:** Keep planner execution, independent path evaluation, and paper interpretation separate. A deterministic `synthetic_greenhouse_v1` provides controlled S01-S05 geometry; preflight rejects invalid inputs before Nav2; successful planner paths receive descriptive endpoint and feasibility metrics; a paper-bundle layer reads immutable result artifacts and generates vector figures plus evidence-conditioned narrative without changing planner outputs. Real-map manual edits remain an explicit overlay/curation manifest that is frozen before formal planner execution.

**Tech Stack:** Python 3.10, ROS 2 Humble, Nav2 SmacPlanner2D / ThetaStar / SmacPlannerHybrid, NumPy, Pillow/Matplotlib, PyYAML, Shapely, GeoJSON/JSON, pytest, ament/colcon.

## Global Constraints

- Synthetic diagnostic map resolution is exactly `0.10 m/cell`.
- Canonical execution platform is MKmini Ackermann with `min_turning_radius = 1.500 m` from `profiles/platforms/mk_mini.yaml`.
- Synthetic results are development/diagnostic evidence only and must never satisfy a formal matrix cell.
- Formal map edits must be planner-independent and represented as an explicit `FORCE_FREE` / `FORCE_OCCUPIED` override layer.
- A scenario input failure must not be counted as a planner algorithm failure.
- A collision-free planner output that violates the turning-radius bound remains a planner success and an execution-infeasible result.
- The accepted real map and semantic assets are immutable within one formal benchmark revision and bound by hashes/site snapshot.
- Paper figures must be reproducible from result files, export vector PDF/SVG plus PNG, and never rely on RViz screenshots as the authoritative figure source.
- Paper narrative must be generated from observed metrics; it must not pre-assert that a baseline fails or that the proposed method wins.
- Single-run synthetic planning time is descriptive only; no runtime-performance conclusion is allowed until repeated formal trials are available.

---

## File Structure

### New benchmark core files

- `src/agt_route_benchmark/agt_route_benchmark/preflight.py` — scenario/map/footprint preflight and typed error codes.
- `src/agt_route_benchmark/agt_route_benchmark/endpoint_metrics.py` — requested-vs-returned endpoint deviation metrics.
- `src/agt_route_benchmark/agt_route_benchmark/synthetic_greenhouse.py` — deterministic `synthetic_greenhouse_v1` map and semantic fixture generator.
- `src/agt_route_benchmark/agt_route_benchmark/paper_bundle.py` — result selection, evidence table, figure inputs, claim-safe descriptive text.
- `src/agt_route_benchmark/agt_route_benchmark/map_curation.py` — real-map override and accepted-map curation manifest contract.

### New scripts

- `src/agt_route_benchmark/scripts/route_benchmark_generate_synthetic.py`
- `src/agt_route_benchmark/scripts/route_benchmark_paper_bundle.py`
- `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`

### New versioned assets

- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.pgm`
- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.yaml`
- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/semantic.geojson`
- `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/fixture_manifest.json`
- `src/agt_route_benchmark/scenarios/synthetic/S01_straight_row.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S02_90deg_entry.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S03_headland_uturn.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S04_narrow_headland.yaml`
- `src/agt_route_benchmark/scenarios/synthetic/S05_blocked_row.yaml`

### New tests

- `src/agt_route_benchmark/test/test_preflight.py`
- `src/agt_route_benchmark/test/test_endpoint_metrics.py`
- `src/agt_route_benchmark/test/test_synthetic_greenhouse.py`
- `src/agt_route_benchmark/test/test_paper_bundle.py`
- `src/agt_route_benchmark/test/test_map_curation.py`
- `src/agt_route_benchmark/test/test_runner_ros_args.py`

### Existing files to modify

- `src/agt_route_benchmark/scripts/route_benchmark_run.py`
- `src/agt_route_benchmark/agt_route_benchmark/experiment.py`
- `src/agt_route_benchmark/CMakeLists.txt`
- `src/agt_route_benchmark/README.md`
- `src/agt_route_benchmark/scenarios/S01_straight_row.yaml`
- `.github/workflows/paper1-route-benchmark.yml`

---

### Task 1: Preserve Target-Machine Runtime Fixes in the Branch

**Files:**
- Modify: `src/agt_route_benchmark/scripts/route_benchmark_run.py`
- Modify: `src/agt_route_benchmark/scenarios/S01_straight_row.yaml`
- Create: `src/agt_route_benchmark/test/test_runner_ros_args.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Consumes: launch_ros-added arguments such as `--ros-args -r __node:=agt_route_benchmark_runner`.
- Produces: strict benchmark CLI parsing after ROS arguments are removed; corrected legacy smoke fixture goal `(4.0, 0.0, 0.0)`.

- [ ] **Step 1: Write the failing ROS-argument regression test**

Create a test that imports the runner module by path, replaces `sys.argv` with:

```python
[
    "route_benchmark_run.py",
    "--site", "greenhouse_01",
    "--scenario", "scenario.yaml",
    "--planner", "astar",
    "--ros-args", "-r", "__node:=agt_route_benchmark_runner",
]
```

Test the extracted benchmark argv helper and require:

```python
[
    "--site", "greenhouse_01",
    "--scenario", "scenario.yaml",
    "--planner", "astar",
]
```

Also test that an unknown benchmark option before `--ros-args` remains present so `argparse` still rejects misspelled benchmark arguments.

- [ ] **Step 2: Run the regression test and verify RED**

Run:

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_runner_ros_args.py
```

Expected: FAIL because the current branch still calls `parser.parse_args()` directly.

- [ ] **Step 3: Implement a narrow ROS-argument removal helper**

In `route_benchmark_run.py`, add:

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

and replace direct parsing with:

```python
args = parser.parse_args(_benchmark_cli_args(sys.argv))
```

Import `sys`. Do not use `parse_known_args()`.

- [ ] **Step 4: Correct the legacy S01 smoke fixture**

Change only:

```yaml
goal: {x: 6.0, y: 0.0, yaw: 0.0}
```

to:

```yaml
goal: {x: 4.0, y: 0.0, yaw: 0.0}
```

Keep `development_fixture: true`.

- [ ] **Step 5: Register and run the test suite**

Add `test_runner_ros_args` to `CMakeLists.txt`, then run:

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
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Consumes: `ScenarioSpec`, `Nav2Map`, platform navigation footprint.
- Produces: `PreflightResult(valid: bool, error_codes: tuple[str, ...], metadata: dict[str, object])` and `evaluate_p2p_preflight(...)`.

- [ ] **Step 1: Write four failing preflight tests**

Cover exactly:

```python
assert result.error_codes == ("GOAL_OUT_OF_MAP",)
assert result.error_codes == ("START_OCCUPIED",)
assert result.error_codes == ("START_FOOTPRINT_COLLISION",)
assert valid_result.valid is True
```

The footprint-collision fixture must place the reference point in a free cell while one rotated footprint corner intersects occupied space.

- [ ] **Step 2: Run tests and verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_preflight.py
```

Expected: FAIL because `preflight.py` does not exist.

- [ ] **Step 3: Implement deterministic occupancy queries**

Use `nav2_map_occupancy_data(nav_map)` and map coordinates:

```python
ix = math.floor((x - nav_map.origin[0]) / nav_map.resolution_m)
iy = math.floor((y - nav_map.origin[1]) / nav_map.resolution_m)
```

Treat `x == extent[1]` or `y == extent[3]` as outside because upper bounds are exclusive.

Define hard codes exactly:

```python
START_OUT_OF_MAP
GOAL_OUT_OF_MAP
START_OCCUPIED
GOAL_OCCUPIED
START_FOOTPRINT_COLLISION
GOAL_FOOTPRINT_COLLISION
```

Unknown cells are invalid for start/goal under the current `allow_unknown: false` benchmark policy.

- [ ] **Step 4: Implement rotated full-footprint preflight**

Transform each footprint vertex `(fx, fy)` by pose yaw:

```python
wx = x + math.cos(yaw) * fx - math.sin(yaw) * fy
wy = y + math.sin(yaw) * fx + math.cos(yaw) * fy
```

Rasterize/check the footprint polygon with Shapely against occupied/unknown cell boxes, not only its corners.

- [ ] **Step 5: Integrate preflight before `_nav2_call()`**

In `route_benchmark_run.py`, after `nav_map` and profile load but before starting live P2P planning:

```python
preflight = evaluate_p2p_preflight(scenario, nav_map, profile)
```

When invalid, do not call Nav2. Write a result directory with:

```json
{
  "success": false,
  "error_code": "INVALID_SCENARIO",
  "preflight_error_codes": ["GOAL_OUT_OF_MAP"]
}
```

and manifest metadata containing the same preflight record.

- [ ] **Step 6: Protect statistics from invalid inputs**

Update result classification so `INVALID_SCENARIO` is excluded from planner success/failure denominators in formal summary code. It remains visible as an audit/error row.

- [ ] **Step 7: Run tests**

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
- Consumes: requested `start`, requested `goal`, normalized returned `PathPoint` sequence.
- Produces: `compute_endpoint_deviation(...) -> dict[str, float]`.

- [ ] **Step 1: Write failing position and wraparound tests**

Use the observed 0.5 m-cell smoke behavior:

```python
start = (0.0, 0.0, 0.0)
goal = (4.0, 0.0, 0.0)
returned_start = (0.25, 0.25, 0.0)
returned_goal = (4.25, 0.25, 0.0)
```

Require:

```python
start_pose_deviation_m == pytest.approx(math.sqrt(0.125))
goal_pose_deviation_m == pytest.approx(math.sqrt(0.125))
```

For yaw, compare `math.pi - 0.01` against `-math.pi + 0.01` and require an error of approximately `0.02`, not `~2*pi`.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_endpoint_metrics.py
```

- [ ] **Step 3: Implement the pure metric function**

Return exactly:

```python
{
    "start_pose_deviation_m": ...,
    "goal_pose_deviation_m": ...,
    "start_heading_deviation_rad": ...,
    "goal_heading_deviation_rad": ...,
}
```

Use wrapped angular distance:

```python
abs(math.atan2(math.sin(a - b), math.cos(a - b)))
```

- [ ] **Step 4: Integrate only for successful P2P runs**

In `ExperimentRunner.run()`, after path metrics and before writing `metrics.json`, add endpoint metrics when `scenario.level == "p2p"` and returned path is non-empty.

Do not snap or rewrite the path.

- [ ] **Step 5: Run focused and full tests**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_endpoint_metrics.py
python3 -m pytest -q src/agt_route_benchmark/test
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
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
- Create: `src/agt_route_benchmark/maps/synthetic_greenhouse_v1/*`
- Create: `src/agt_route_benchmark/scenarios/synthetic/S01_straight_row.yaml`
- Create: `src/agt_route_benchmark/scenarios/synthetic/S02_90deg_entry.yaml`
- Create: `src/agt_route_benchmark/scenarios/synthetic/S03_headland_uturn.yaml`
- Create: `src/agt_route_benchmark/scenarios/synthetic/S04_narrow_headland.yaml`
- Create: `src/agt_route_benchmark/scenarios/synthetic/S05_blocked_row.yaml`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`

**Interfaces:**
- Consumes: no runtime data; fixed geometry constants.
- Produces: deterministic PGM/YAML/semantic GeoJSON/fixture manifest and five `development_fixture: true` scenarios.

- [ ] **Step 1: Write deterministic-generation tests**

Generate twice into two temporary directories and assert byte-identical SHA256 for:

```text
map.pgm
map.yaml
semantic.geojson
fixture_manifest.json
```

Assert map resolution is exactly `0.10` and each S01-S05 start/goal is inside the map.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_synthetic_greenhouse.py
```

- [ ] **Step 3: Implement fixed map geometry**

Use a `260 x 200` image at `0.10 m/cell`, origin `[0.0, 0.0, 0.0]`, giving `26 m x 20 m`.

Reserve a one-meter occupied outer boundary. Build three long occupied crop-bed rectangles:

```text
bed_1: x=[5.0, 7.0],   y=[4.0, 16.0]
bed_2: x=[10.0, 12.0], y=[4.0, 16.0]
bed_3: x=[15.0, 17.0], y=[4.0, 16.0]
```

This creates aisles centered near `x=3.0, 8.5, 13.5, 19.0` with open headlands above/below the beds.

Add a permanent obstacle in the `x=12.0..15.0` aisle:

```text
obstacle: x=[12.6, 14.4], y=[9.3, 10.7]
```

Add a narrow-headland constriction only in the left diagnostic zone so S04 can be tuned independently without changing S02/S03 geometry.

- [ ] **Step 4: Define diagnostic scenarios**

Freeze the first-pass coordinates as:

```yaml
# S01
start: {x: 8.5, y: 5.0, yaw: 1.57079632679}
goal:  {x: 8.5, y: 13.0, yaw: 1.57079632679}

# S02: bottom headland into aisle
start: {x: 3.0, y: 2.0, yaw: 0.0}
goal:  {x: 8.5, y: 7.0, yaw: 1.57079632679}

# S03: wide upper-headland U-turn between adjacent aisles
start: {x: 8.5, y: 15.0, yaw: 1.57079632679}
goal:  {x: 13.5, y: 15.0, yaw: -1.57079632679}

# S04: narrow-headland diagnostic region
start: {x: 19.0, y: 5.0, yaw: -1.57079632679}
goal:  {x: 19.0, y: 7.5, yaw: 1.57079632679}

# S05: blocked-row route requiring exit/detour or no-path
start: {x: 13.5, y: 6.0, yaw: 1.57079632679}
goal:  {x: 13.5, y: 14.0, yaw: 1.57079632679}
```

All files must include `development_fixture: true` and a description stating that planner ranking is not predetermined.

- [ ] **Step 5: Generate semantic fixture**

Write `semantic.geojson` with feature IDs for:

```text
field_boundary
bed_1
bed_2
bed_3
headland_south
headland_north
permanent_obstacle_01
```

Use semantics for visualization/audit; occupancy remains authoritative for P2P collision planning.

- [ ] **Step 6: Add fixture manifest hashes**

`fixture_manifest.json` must include:

```json
{
  "schema_version": "1.0",
  "fixture_id": "synthetic_greenhouse_v1",
  "resolution_m": 0.1,
  "formal": false,
  "generator": "agt_route_benchmark.synthetic_greenhouse",
  "assets": {"map.pgm": "<sha256>", "map.yaml": "<sha256>", "semantic.geojson": "<sha256>"}
}
```

The generator writes real hashes; the string above is the contract shape, not a literal fixture file.

- [ ] **Step 7: Register installation and tests**

Install `maps` with existing package share assets and install the generator script. Add `test_synthetic_greenhouse` to CMake.

- [ ] **Step 8: Generate committed assets and verify reproducibility**

```bash
python3 src/agt_route_benchmark/scripts/route_benchmark_generate_synthetic.py \
  --output src/agt_route_benchmark/maps/synthetic_greenhouse_v1
python3 -m pytest -q src/agt_route_benchmark/test/test_synthetic_greenhouse.py
```

Expected: PASS and no diff after a second generation.

- [ ] **Step 9: Commit**

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

### Task 5: Run S02/S03 Diagnostics Without Manufacturing a Ranking

**Files:**
- Modify: `src/agt_route_benchmark/README.md`
- Modify only if runtime exposes a real defect: benchmark implementation files covered by Tasks 1-4.

**Interfaces:**
- Consumes: frozen synthetic map, S02/S03 scenarios, A*/Theta*/Hybrid live Nav2.
- Produces: six complete development result cells and a machine-readable comparison table.

- [ ] **Step 1: Build and test on ROS2 Humble target machine**

```bash
colcon build --symlink-install --packages-select agt_route_benchmark
source install/setup.bash
rm -rf /tmp/agt_route_benchmark_test_results
colcon test --packages-select agt_route_benchmark \
  --test-result-base /tmp/agt_route_benchmark_test_results \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/agt_route_benchmark_test_results --verbose
```

Expected: zero failures.

- [ ] **Step 2: Run exactly six diagnostic cells**

For each scenario in `S02_90deg_entry`, `S03_headland_uturn` and planner in `astar`, `theta_star`, `hybrid_astar`, invoke `benchmark_run.launch.py` with:

```text
formal:=false
map:=.../maps/synthetic_greenhouse_v1/map.yaml
platform_profile:=.../profiles/platforms/mk_mini.yaml
```

Use unique run IDs `diag_s02_astar_001`, `diag_s02_theta_001`, `diag_s02_hybrid_001`, `diag_s03_astar_001`, `diag_s03_theta_001`, `diag_s03_hybrid_001`.

- [ ] **Step 3: Verify artifact completeness**

For each successful planner output require:

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

If a planner reports no path, require manifest/report/metrics and classify it as planner no-path, not invalid scenario.

- [ ] **Step 4: Inspect evidence, not screenshots**

Record for each cell:

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

Do not change map geometry because a planner result is inconvenient. A geometry change requires a new fixture version such as `synthetic_greenhouse_v2` with a written reason unrelated to desired planner ranking.

- [ ] **Step 5: Commit README execution instructions**

Document the exact six commands and the non-formal interpretation rule.

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
- Consumes: existing immutable result directories and fixture/real-map assets.
- Produces: comparison CSV/JSON, vector figures, `claims.md`, `figure_manifest.json`.

- [ ] **Step 1: Write claim-safety tests before rendering code**

Create synthetic metrics fixtures that cover:

1. planner success + collision-free + `kinematic_feasible=false`
2. planner success + `kinematic_feasible=true`
3. planner no-path
4. invalid scenario

Require narrative behavior such as:

```python
assert "collision-free but not kinematically feasible" in claims
assert "A* cannot solve" not in claims
assert "Theta* is slower" not in claims  # single-run diagnostic timing
assert "invalid scenario" not in algorithm_failure_claims
```

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_paper_bundle.py
```

- [ ] **Step 3: Implement result normalization**

Create a row per result containing planner/scenario plus all relevant metrics and source file SHA256 values. Write:

```text
comparison.csv
comparison.json
```

- [ ] **Step 4: Implement Figure D1 — map/problem formulation**

Render `synthetic_greenhouse_v1` occupancy plus labeled beds/headlands/obstacle/start-goal markers into:

```text
fig_d1_synthetic_problem.svg
fig_d1_synthetic_problem.pdf
fig_d1_synthetic_problem.png
```

The caption source text must state that this is controlled diagnostic geometry, not the real greenhouse result.

- [ ] **Step 5: Implement Figure D2 — S02 same-map planner overlay**

Overlay A*, Theta*, Hybrid A* paths on the same occupancy map. Add start/goal arrows and a small metric panel containing only observed values:

```text
length
max curvature
R_min bound
execution feasible
```

Export SVG/PDF/PNG.

- [ ] **Step 6: Implement Figure D3 — S03 U-turn comparison**

Use the same visual grammar as D2. Include direction markers where reverse segments exist. Do not hide planner no-path; render a labeled `NO PATH` panel entry instead.

- [ ] **Step 7: Implement Figure D4 — feasibility evidence matrix**

Create a compact matrix with rows `(S02, S03)` and columns `(A*, Theta*, Hybrid A*)`; cells show:

```text
planner success
collision-free
kinematic feasible
execution feasible
```

This figure is the direct visual evidence for the distinction between geometric planning and executable planning.

- [ ] **Step 8: Generate `claims.md` from evidence**

Use conditional templates. Examples:

```text
If collision_free == true and kinematic_feasible == false:
"In S02, <planner> returned a collision-free path, but the independent full-footprint/curvature evaluator classified it as kinematically infeasible under the frozen MKmini minimum turning radius."
```

```text
If all three are feasible:
"S02 did not produce a kinematic-feasibility divergence among the tested planners; this diagnostic case therefore does not support a claim of planner-family separation and should be reported as such."
```

```text
If planner success == false:
"<planner> did not return a valid path in this scenario; this is reported as a no-path outcome rather than post-planning infeasibility."
```

Never emit a superiority claim unless the metrics support it.

- [ ] **Step 9: Write `figure_manifest.json`**

Bind every figure to input run directories and SHA256 of each input `metrics.json`, `path.csv`, map YAML/image, and semantic GeoJSON used to render it.

- [ ] **Step 10: Test deterministic narrative and figure manifest**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_paper_bundle.py
```

Require byte-stable `claims.md` and stable source hashes. Do not require PNG byte identity across Matplotlib versions; require identical data/manifest inputs instead.

- [ ] **Step 11: Commit**

```bash
git add src/agt_route_benchmark/agt_route_benchmark/paper_bundle.py \
        src/agt_route_benchmark/scripts/route_benchmark_paper_bundle.py \
        src/agt_route_benchmark/test/test_paper_bundle.py \
        src/agt_route_benchmark/CMakeLists.txt \
        src/agt_route_benchmark/README.md
git commit -m "feat(route-benchmark): add paper evidence bundle"
```

---

### Task 7: Define the Auditable Real-Map Override / Curation Handoff

**Files:**
- Create: `src/agt_route_benchmark/agt_route_benchmark/map_curation.py`
- Create: `src/agt_route_benchmark/scripts/route_benchmark_map_curation.py`
- Create: `src/agt_route_benchmark/test/test_map_curation.py`
- Modify: `src/agt_route_benchmark/CMakeLists.txt`
- Modify: `src/agt_route_benchmark/README.md`

**Interfaces:**
- Consumes: generated pre-override map YAML/PGM, override JSON exported by the V25 workbench, accepted final YAML/PGM, optional source PCD path.
- Produces: immutable `map_curation_manifest.json` with audit records and SHA256 lineage.

- [ ] **Step 1: Write schema tests**

Require one override record to have exactly these core fields:

```json
{
  "override_id": "override_0001",
  "edit_type": "FORCE_FREE",
  "geometry": {"type": "Polygon", "coordinates": []},
  "reason": "remove raster artifact verified against source PCD",
  "evidence_category": "PCD_INSPECTION"
}
```

Allowed evidence categories:

```text
PCD_INSPECTION
SITE_PHOTO
MEASURED_STRUCTURE
KNOWN_PERMANENT_OBSTACLE
```

Reject empty reason, unknown edit type, malformed geometry, and duplicate IDs.

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q src/agt_route_benchmark/test/test_map_curation.py
```

- [ ] **Step 3: Implement curation manifest**

Write:

```json
{
  "schema_version": "1.0",
  "site_id": "greenhouse_01",
  "planner_independent": true,
  "source_pcd": {"path": "...", "sha256": "..."},
  "generated_map": {"yaml_sha256": "...", "image_sha256": "..."},
  "override_layer": {"sha256": "...", "records": []},
  "accepted_map": {"yaml_sha256": "...", "image_sha256": "..."}
}
```

If source PCD is omitted, record `source_pcd: null`; do not invent a hash.

- [ ] **Step 4: Enforce planner-independent wording at the contract boundary**

The schema must contain no planner ID field. The CLI shall require:

```text
--site greenhouse_01
--generated-map-yaml ...
--override-json ...
--accepted-map-yaml ...
--output .../map_curation_manifest.json
```

and optionally `--source-pcd`.

- [ ] **Step 5: Add accepted-map consistency check**

Load both generated and accepted map YAML. Require identical resolution/origin dimensions unless the override export explicitly declares `geometry_change_allowed: true`; Scheme B uses same raster geometry by default, so normal manual de-jagging changes cell values only.

- [ ] **Step 6: Run tests and commit**

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
- Consumes: all Tasks 1-7.
- Produces: green pure-Python CI, green target-machine ROS package tests, six diagnostic cells, and paper bundle whose claims trace directly to metrics.

- [ ] **Step 1: Extend pure-Python CI coverage**

Keep existing dependency install. Ensure the full test command naturally includes all new tests:

```bash
python -m pytest -q src/agt_route_benchmark/test
```

Extend compile step to include all new package/script files, which the existing directory-level `compileall` already covers.

- [ ] **Step 2: Run local pure-Python verification**

```bash
MPLBACKEND=Agg \
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets \
python3 -m pytest -q src/agt_route_benchmark/test

python3 -m compileall -q \
  src/agt_route_benchmark/agt_route_benchmark \
  src/agt_route_benchmark/scripts
```

Expected: zero failures/errors.

- [ ] **Step 3: Run ROS2 target-machine build/test**

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

- [ ] **Step 4: Re-run S01 smoke and the six S02/S03 cells**

S01 must still produce the observed straight-path properties without requiring exact runtime equality:

```text
success=true
collision_free=true
kinematic_feasible=true
execution_feasible=true
path_length_m approximately 8.0 on synthetic S01
max_abs_curvature_1pm approximately 0
```

Run six S02/S03 cells as defined in Task 5.

- [ ] **Step 5: Build paper bundle**

Run:

```bash
ros2 run agt_route_benchmark route_benchmark_paper_bundle.py \
  --result-root runtime/results/paper1_route_benchmark_synthetic \
  --map-yaml src/agt_route_benchmark/maps/synthetic_greenhouse_v1/map.yaml \
  --semantic-map src/agt_route_benchmark/maps/synthetic_greenhouse_v1/semantic.geojson \
  --output runtime/results/paper1_route_benchmark_synthetic/paper_bundle
```

Require:

```text
comparison.csv
comparison.json
claims.md
figure_manifest.json
fig_d1_synthetic_problem.svg/pdf/png
fig_d2_s02_planner_comparison.svg/pdf/png
fig_d3_s03_planner_comparison.svg/pdf/png
fig_d4_feasibility_matrix.svg/pdf/png
```

- [ ] **Step 6: Scientific self-consistency review**

Read `claims.md` next to `comparison.csv` and require every substantive sentence to be traceable to one or more named metrics. Specifically reject any generated/manual text that says:

```text
A* cannot solve agricultural planning
Theta* is always slower
Hybrid A* never violates semantics
our method is superior
```

unless a later formal experiment actually supports a narrower, explicitly scoped statement.

The allowed core interpretation is:

```text
A collision-free 2D geometric path is not sufficient evidence of Ackermann executability; executability must be evaluated against footprint, curvature/turning-radius, semantic, and task constraints.
```

- [ ] **Step 7: Document the synthetic-to-real figure substitution rule**

README must state that final paper figures reuse the same renderer/metrics/claim pipeline but replace synthetic inputs with frozen `greenhouse_01` site-snapshot-bound results. Synthetic figures may appear only as method/diagnostic illustrations, while headline quantitative tables/figures come from the accepted real map.

- [ ] **Step 8: Commit final integration**

```bash
git add .github/workflows/paper1-route-benchmark.yml \
        src/agt_route_benchmark/README.md
git commit -m "test(route-benchmark): close synthetic diagnostic gate"
```

---

## Completion Gate

Do not start the formal 23-cell benchmark until all conditions below are true:

- target-machine A*/Theta*/Hybrid S01 smoke chain passes
- hard preflight rejects invalid/out-of-map/occupied/footprint-collision scenarios before Nav2
- endpoint discretization metrics are present on successful P2P results
- `synthetic_greenhouse_v1` regenerates deterministically at `0.10 m/cell`
- S02 and S03 each have A*/Theta*/Hybrid diagnostic results with independent feasibility evidence
- paper bundle emits vector figures and evidence-conditioned `claims.md`
- synthetic interpretation remains descriptive and does not manufacture planner ranking
- real-map Scheme B curation manifest can bind source/generated/override/accepted-map hashes
- the real workbench exports/retains override records instead of destructively hiding them

After this gate, the next plan is: accept/freeze the actual `greenhouse_01` map and semantics, generate the State Lattice control set for the frozen resolution/Rmin, execute 20 formal P2P cells + 3 formal mission cells, generate the real paper bundle, then execute selected accepted routes on MKmini and add tracking evidence.
