# agt_route_benchmark

Paper-oriented benchmark and execution layer for the first agricultural route-planning study in `agt_navigation_v2`.
It reuses the V2.5 map, semantic, maximum-feasible-coverage and Route Asset contracts instead of creating a parallel navigation stack.

## 1. Frozen study definition

- Site: `greenhouse_01`
- Vehicle: `profiles/platforms/mk_mini.yaml`
- P2P baselines: A*, Theta*, Hybrid A*, State Lattice
- Mission baselines: manual waypoints + frozen best P2P, Fields2Cover/OpenNav, proposed V2.5 maximum-feasible route
- Scenarios: S01 straight row, S02 90-degree entry, S03 headland U-turn, S04 narrow headland, S05 blocked row, S06 full mission

The formal matrix has **23 meaningful cells**, not a blind 6 x 6 Cartesian product:

```text
S01-S05 x {A*, Theta*, Hybrid A*, State Lattice} = 20 P2P cells
S06 x {manual+best-P2P, Fields2Cover, ours}       =  3 mission cells
                                                        -----
                                                         23
```

Checked-in scenario values are development fixtures. Formal mode rejects them until the real PCD-derived map, semantic IDs and reference-reachable set are accepted.

## 2. Responsibility boundary

```text
real PCD
  -> occupancy / traversability                 user accepts geometry
  -> agricultural semantics                    user accepts semantic correctness
  -> maximum feasible coverage / task route    V2.5 route pipeline
  -> canonical Route Asset CSV
  -> benchmark normalization + independent validation
  -> paper CSV / GeoJSON / figures / metrics
  -> RPP tracking simulation
  -> same accepted route for site tracking test
```

The benchmark never republishes velocity during planning experiments and never silently substitutes a missing baseline.

## 3. Build and unit verification

```bash
cd ~/agt_navigation_v2
colcon build --symlink-install --packages-select agt_route_benchmark
source install/setup.bash
colcon test --packages-select agt_route_benchmark --event-handlers console_direct+
colcon test-result --verbose
```

Pure-Python development checks can also be run with:

```bash
PYTHONPATH=src/agt_route_benchmark:src/agt_coverage_planning:src/agt_offline_assets \
python3 -m pytest -q src/agt_route_benchmark/test
```

## 4. Freeze the accepted `greenhouse_01` revision

After the PCD-derived 2D map and semantic annotations are accepted, copy `config/site_acceptance_template.yaml` into the runtime site directory, set both acceptance flags to `true`, and record `accepted_by` / `accepted_at`.
Then bind exact asset hashes:

```bash
ros2 run agt_route_benchmark route_benchmark_accept_site.py \
  --pcd runtime/maps/greenhouse_01/source/greenhouse_01.pcd \
  --map-yaml runtime/maps/greenhouse_01/greenhouse_01.yaml \
  --semantic-map runtime/maps/greenhouse_01/semantic/semantic_map.geojson \
  --coverage-yaml runtime/maps/greenhouse_01/semantic/coverage.yaml \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --acceptance runtime/maps/greenhouse_01/benchmark/acceptance.yaml \
  --output runtime/maps/greenhouse_01/benchmark/site_snapshot.json
```

Formal runs re-hash the bound assets before planning, so map/semantic/profile changes cannot silently mix experiment revisions.

## 5. Inspect the canonical 23-cell execution plan

```bash
ros2 run agt_route_benchmark route_benchmark_matrix_plan.py \
  --site greenhouse_01 \
  --result-root runtime/results/paper1_route_benchmark \
  --map runtime/maps/greenhouse_01/greenhouse_01.yaml \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --semantic-map runtime/maps/greenhouse_01/semantic/semantic_map.geojson \
  --manual-waypoints runtime/maps/greenhouse_01/benchmark/manual_waypoints.yaml \
  --ours-route-csv runtime/maps/greenhouse_01/routes/maximum_feasible_route.csv \
  --lattice-filepath runtime/maps/greenhouse_01/benchmark/mkmini_lattice.json \
  --site-snapshot runtime/maps/greenhouse_01/benchmark/site_snapshot.json \
  --formal --show-commands \
  --output runtime/results/paper1_route_benchmark/execution_plan.json
```

Each cell is reported as `READY`, `COMPLETE`, or `BLOCKED_INPUT`. Only the five State Lattice P2P cells require a lattice control set. The formal lattice file must be generated after the final map resolution is frozen because the motion-control set is resolution- and vehicle-specific.

## 6. P2P baseline runtime

The four P2P baselines use the same accepted map, footprint and minimum turning radius. Formal P2P cells must run live Nav2; fixture-only adapters are rejected.

Example:

```bash
ros2 launch agt_route_benchmark benchmark_run.launch.py \
  site:=greenhouse_01 \
  scenario:=$(pwd)/src/agt_route_benchmark/scenarios/S04_narrow_headland.yaml \
  planner:=hybrid_astar \
  map:=$(pwd)/runtime/maps/greenhouse_01/greenhouse_01.yaml \
  platform_profile:=$(pwd)/profiles/platforms/mk_mini.yaml \
  site_snapshot:=$(pwd)/runtime/maps/greenhouse_01/benchmark/site_snapshot.json \
  formal:=true
```

The generated path is then independently checked with the same full MKmini footprint and `R_min`, rather than trusting each planner's own success flag.

## 7. Mission baselines

### 7.1 Manual waypoints + frozen best P2P

Manual waypoints are an explicit evaluation asset, not hidden operator knowledge:

```yaml
schema_version: '1.0'
frame_id: map
p2p_planner: hybrid_astar
waypoints:
  - {x: 0.0, y: 0.0, yaw: 0.0, semantic_ref: row_01}
  - {x: 8.0, y: 0.0, yaw: 0.0, semantic_ref: row_01}
  - {x: 8.0, y: 2.0, yaw: 3.14159, semantic_ref: row_02}
```

Run:

```bash
ros2 launch agt_route_benchmark manual_mission_benchmark.launch.py \
  scenario:=$(pwd)/src/agt_route_benchmark/scenarios/S06_full_mission.yaml \
  manual_waypoints:=$(pwd)/runtime/maps/greenhouse_01/benchmark/manual_waypoints.yaml \
  map:=$(pwd)/runtime/maps/greenhouse_01/greenhouse_01.yaml \
  platform_profile:=$(pwd)/profiles/platforms/mk_mini.yaml \
  site_snapshot:=$(pwd)/runtime/maps/greenhouse_01/benchmark/site_snapshot.json \
  formal:=true
```

### 7.2 Fields2Cover / OpenNav

The benchmark reuses the existing `agt_coverage_planning` runtime. Formal F2C experiments must use the live coverage service; precomputed component JSON is development-only.

```bash
ros2 launch agt_route_benchmark fields2cover_mission_benchmark.launch.py \
  scenario:=$(pwd)/src/agt_route_benchmark/scenarios/S06_full_mission.yaml \
  semantic_map:=$(pwd)/runtime/maps/greenhouse_01/semantic/semantic_map.geojson \
  map:=$(pwd)/runtime/maps/greenhouse_01/greenhouse_01.yaml \
  platform_profile:=$(pwd)/profiles/platforms/mk_mini.yaml \
  site_snapshot:=$(pwd)/runtime/maps/greenhouse_01/benchmark/site_snapshot.json \
  formal:=true
```

### 7.3 Proposed method

Formal `ours` does **not** use the sparse development graph polyline. V2.5/12g first produces the canonical continuous Route Asset CSV; the benchmark consumes that exact geometry without smoothing or replanning:

```bash
ros2 run agt_route_benchmark route_benchmark_run.py \
  --site greenhouse_01 \
  --scenario src/agt_route_benchmark/scenarios/S06_full_mission.yaml \
  --planner ours \
  --ours-route-csv runtime/maps/greenhouse_01/routes/maximum_feasible_route.csv \
  --map-yaml runtime/maps/greenhouse_01/greenhouse_01.yaml \
  --semantic-map runtime/maps/greenhouse_01/semantic/semantic_map.geojson \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --site-snapshot runtime/maps/greenhouse_01/benchmark/site_snapshot.json \
  --result-root runtime/results/paper1_route_benchmark \
  --formal
```

## 8. Independent evaluation contract

Every successful normalized route can be evaluated against the same accepted assets:

- path length
- planning time
- reverse distance
- contiguous reverse-maneuver count
- maximum curvature
- full-footprint collision count
- minimum clearance
- minimum-turning-radius feasibility
- hard semantic violations: leaving `field_boundary`, entering `exclusion_zone` or `keepout_zone`
- mission required-task coverage
- reachable-task coverage
- deadhead distance

Hard semantic violations are counted as contiguous episodes; sample counts are emitted separately so metrics do not depend on route discretization density.

`headland_zone` / row-topology transition legality is intentionally kept as a separate mission-topology metric and should only be frozen after the final real semantic map is accepted.

## 9. Output contract

A successful experiment cell contains:

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

Stable paper CSV schema:

```text
index,x_m,y_m,yaw_rad,direction,segment_type,semantic_ref
```

The exact validated `path.csv` is the immutable handoff to tracking evaluation.

## 10. Canonical batch summary

After the 23 valid cells have been run:

```bash
ros2 run agt_route_benchmark route_benchmark_batch.py \
  --site greenhouse_01 \
  --site-snapshot runtime/maps/greenhouse_01/benchmark/site_snapshot.json \
  --result-root runtime/results/paper1_route_benchmark \
  --output runtime/results/paper1_route_benchmark/summary \
  --formal
```

Formal summary rejects a missing canonical cell, dependency-skipped cell, duplicate or unexpected cell, and ignores development or old-site-revision runs that do not match the selected `site_snapshot_sha256`.

## 11. MKmini RPP tracking profile

Generate the ROS2 Humble controller profile from the accepted MKmini geometry:

```bash
ros2 run agt_route_benchmark route_benchmark_rpp_params.py \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --output runtime/config/mkmini_rpp_humble.yaml
```

The initial profile is deliberately low-speed (`0.30 m/s`) and freezes Ackermann-critical behavior:

- `use_rotate_to_heading: false`
- `allow_reversing: true`
- curvature regulation begins at the accepted `min_turning_radius`
- reference path is not globally replanned by the controller stage

This is an initial simulation / low-speed tuning profile, not yet a measured final vehicle-controller calibration.

## 12. Lightweight Ackermann + RPP simulation

Until a validated MKmini Gazebo vehicle model is available, use the curvature-limited kinematic simulator rather than the existing BUNKER differential-drive simulator:

```bash
ros2 launch agt_route_benchmark rpp_tracking_sim.launch.py \
  reference_path:=$(pwd)/runtime/results/paper1_route_benchmark/greenhouse_01/S06_full_mission/<ours-run>/path.csv \
  map:=$(pwd)/runtime/maps/greenhouse_01/greenhouse_01.yaml \
  platform_profile:=$(pwd)/profiles/platforms/mk_mini.yaml \
  output_dir:=$(pwd)/runtime/results/paper1_rpp_tracking/ours_run_001 \
  rviz:=true
```

This launch runs:

```text
accepted 2D map
  + immutable path.csv
  + Nav2 Humble RPP
  + curvature-limited Ackermann/bicycle integration
  -> executed.csv
  -> tracking_metrics.json
  -> RViz reference path + executed path + Humble RPP lookahead collision arc
```

The simulator enforces `|curvature| <= 1/R_min` and refuses rotate-in-place commands. It is a controller/path feasibility layer, not a full tire/dynamics/Gazebo validation.

## 13. Tracking metric contract

For simulation or a later real run, store the executed trajectory in the same reference frame:

```text
stamp_s,x_m,y_m,yaw_rad
```

Then evaluate:

```bash
ros2 run agt_route_benchmark route_tracking_evaluate.py \
  --reference runtime/results/.../path.csv \
  --executed runtime/results/.../executed.csv \
  --output runtime/results/.../tracking_metrics.json
```

Metrics include lateral RMSE / mean / maximum error, heading RMSE, final position / yaw error, duration and along-path completion ratio. Lateral error is computed against the nearest line segment, not the nearest stored vertex.

## 14. Field-test handoff

Do not re-click or retype waypoints for the site test. The same accepted Route Asset / normalized `path.csv` identity must be retained. The existing V2.5 `Nav2FollowPathTrackerAdapter` is the intended controller boundary: it tracks provided Route segments and does not request a new global path.

Start the real trajectory recorder in a separate terminal before enabling the route task:

```bash
ros2 run agt_route_benchmark route_tracking_tf_recorder.py \
  --reference runtime/results/paper1_route_benchmark/greenhouse_01/S06_full_mission/<ours-run>/path.csv \
  --executed runtime/results/paper1_real_tracking/run_001/executed.csv \
  --metrics runtime/results/paper1_real_tracking/run_001/tracking_metrics.json \
  --fixed-frame map \
  --base-frame base_footprint \
  --rate 20
```

The recorder samples the real `map -> base_footprint` TF directly. Stop it with `Ctrl-C` after the route trial; it then writes the same executed-trajectory schema and tracking metrics used by the lightweight simulation. This keeps simulation and field evaluation on one metric implementation.

Before enabling physical execution, the MKmini platform profile must be updated from preview-only to execution-ready only after the actual `base_footprint`, mounted envelope and real steering/turning limit have been measured and accepted.

Record at minimum:

- site snapshot / route SHA256
- controller parameter revision
- start pose and task identity
- execution success/failure and abort reason
- lateral RMSE / maximum error
- heading error
- completion ratio and elapsed time
- reverse maneuver behavior

That dataset is the direct input to the Paper I real-world discussion section.
