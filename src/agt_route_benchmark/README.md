# agt_route_benchmark

Paper-oriented benchmark layer for the first agricultural-route-planning study in `agt_navigation_v2`.
It **extends** existing V2.5 map/site/semantic/coverage contracts; it does not create a second map registry.

## Frozen experiment definition

- Site: `greenhouse_01`
- Vehicle: `profiles/platforms/mk_mini.yaml`
- P2P baselines: A*, Theta*, Hybrid A*, State Lattice
- Mission baselines: manual waypoints + best P2P, Fields2Cover/OpenNav, proposed maximum-feasible route planner
- Scenarios: S01 straight row, S02 90-degree entry, S03 headland U-turn, S04 narrow headland, S05 blocked row, S06 full mission

The checked-in scenarios are **development fixtures only**. Formal mode rejects them until the real PCD-derived map and semantic IDs are accepted and the scenario files are replaced with the accepted `greenhouse_01` values.

## Freeze the real `greenhouse_01` input revision

Formal experiments are gated by a human-accepted site snapshot. After the user has accepted the PCD-derived 2D map and the semantic annotations, copy `config/site_acceptance_template.yaml` into the runtime site directory, set both acceptance flags to `true`, and record `accepted_by` / `accepted_at`. Then freeze the exact PCD, map, map image, semantic map, coverage YAML and MK-mini profile hashes:

```bash
PYTHONPATH=src/agt_route_benchmark \
python3 src/agt_route_benchmark/scripts/route_benchmark_accept_site.py \
  --pcd runtime/maps/greenhouse_01/source/greenhouse_01.pcd \
  --map-yaml runtime/maps/greenhouse_01/greenhouse_01.yaml \
  --semantic-map runtime/maps/greenhouse_01/semantic/semantic_map.geojson \
  --coverage-yaml runtime/maps/greenhouse_01/semantic/coverage.yaml \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --acceptance runtime/maps/greenhouse_01/benchmark/acceptance.yaml \
  --output runtime/maps/greenhouse_01/benchmark/site_snapshot.json
```

`--formal` runs require this snapshot and re-hash every bound asset before planning, so changing the PCD/map/semantics/profile invalidates the frozen revision rather than silently mixing experiment versions.

## What is already runnable without ROS

The proposed route-ordering core, CSV/GeoJSON export, metrics, batch summary and paper renderer have no ROS dependency.

```bash
cd ~/agt_navigation_v2
python3 -m pytest -q src/agt_route_benchmark/test

PYTHONPATH=src/agt_route_benchmark \
python3 src/agt_route_benchmark/scripts/route_benchmark_run.py \
  --scenario src/agt_route_benchmark/scenarios/S06_full_mission.yaml \
  --planner ours \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --result-root runtime/results/paper1_route_benchmark \
  --run-id run_001
```

The output cell contains:

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

Stable CSV schema:

```text
index,x_m,y_m,yaw_rad,direction,segment_type,semantic_ref
```

## P2P Nav2 baseline runtime

Start a `planner_server` and global costmap on the accepted map with the four benchmark plugins. Humble plugin definitions are provided in `config/nav2_planners_humble.yaml`.
Then run one cell with `--nav2-live`:

```bash
PYTHONPATH=src/agt_route_benchmark \
python3 src/agt_route_benchmark/scripts/route_benchmark_run.py \
  --scenario src/agt_route_benchmark/scenarios/S01_straight_row.yaml \
  --planner hybrid_astar \
  --platform-profile profiles/platforms/mk_mini.yaml \
  --result-root runtime/results/paper1_route_benchmark \
  --nav2-live
```

`astar`, `theta_star`, `hybrid_astar`, `state_lattice` map to the explicit planner namespaces in the provided Nav2 config. The State Lattice entry is **development-only** until a minimum-control-set file is generated for the accepted map resolution and MK-mini's 1.5 m minimum turning radius.

## Fields2Cover baseline

The existing `agt_coverage_planning` package remains the authoritative OpenNav/Fields2Cover adapter and validator. The benchmark adapter consumes reconstructed SWATH/CONNECTION components rather than RViz markers. Until the direct ROS bridge is added, a component JSON can be imported with `--coverage-components-json`; lack of the dependency is reported as `SKIPPED_DEPENDENCY`, never silently replaced by another planner.

## Reference reachable truth

Mission-level `reachable_task_coverage_ratio` is evaluated against a scenario-level `reference_reachable_semantic_ids` set, not against each planner's own claimed reachable set. This reference is an evaluation truth artifact accepted with the site/scenario and is not generated per planner. Formal mission scenarios require it.

## Proposed method contract

The current first-stage implementation is deliberately interpretable:

```text
accepted agricultural task semantics
  -> explicit reachable task set
  -> legal row/headland topology
  -> maximize visited reachable tasks
  -> minimize deterministic transition cost among equal-coverage routes
  -> normalized SWATH/CONNECTION route
  -> downstream Ackermann connector + full-footprint validation
```

`scenarios/fixtures/S06_development_graph.yaml` is a tiny CI/development graph, not paper evidence. The real graph must be derived from / bound to the accepted semantic map revision.

## MK-mini execution gate

The current canonical profile has preview planning enabled but `route_acceptance.enabled: false`. That means simulation and offline paper experiments are allowed, while a route must **not** be promoted as vehicle-execution READY until the actual `base_footprint` reference and final mounted navigation envelope are measured and accepted in the platform profile.

## Preview exported CSV in RViz

```bash
ros2 run agt_route_benchmark route_csv_to_path.py \
  --path-csv runtime/results/paper1_route_benchmark/.../path.csv
```

This only publishes `/agt/benchmark/path_preview` as `nav_msgs/Path`; it publishes no velocity and does not bypass the existing navigation/safety capability boundary.

## Batch summary

```bash
ros2 run agt_route_benchmark route_benchmark_batch.py \
  --result-root runtime/results/paper1_route_benchmark \
  --output runtime/results/paper1_route_benchmark/summary \
  --scenario S01_straight_row --scenario S02_90deg_entry \
  --planner astar --planner theta_star
```

Formal batch mode requires every requested matrix cell and rejects dependency-skipped cells.

## Field-test handoff

The exact validated `path.csv` is the handoff artifact for later RPP tests. Do not re-click/retype waypoints for the field run. Record at least route/result identity, map/semantic/profile identities, RPP tuning, success/failure, lateral RMSE, max lateral error, heading error, completion time and abort reason.
