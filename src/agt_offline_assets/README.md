# agt_offline_assets

Pure offline asset-preparation primitives for V25-09A. The package itself owns no ROS graph state and does not publish TF or velocity. ROS replay orchestration lives in `agt_system_manager`; finalized MappingSession artifacts are handed back into this package for immutable asset preparation.

The package turns the frozen asset contracts into executable file workflows:

```text
Dataset + Calibration + Platform + Recipe
  -> PROCESSING map workspace
  -> MappingSession evidence (session frame)
  -> alignment / cleaning / canonical materialization
  -> derived map products + quality reports
  -> READY map manifest
  -> Semantic Map + Route Policy
  -> DRAFT Route Asset
  -> full-footprint / kinematic feasibility
  -> preview.geojson
  -> READY or INVALID route revision
```

## CLI

The executable is intentionally named `agt_offline_assets_cli.py` rather than `agt_offline_assets.py` so it cannot shadow the Python package `agt_offline_assets` when launched through `ros2 run`.

Create a versioned workspace:

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py init-map \
  --maps-root runtime/maps \
  --map-id greenhouse_a \
  --dataset dataset_binding.yaml \
  --recipe recipe.yaml \
  --site-frame site_frame.yaml \
  --alignment alignment.yaml \
  --platform-profile profiles/platforms/bunker.yaml \
  --calibration calibration.yaml
```

For a real recorded bag, the recommended next command is the system-manager replay orchestrator:

```bash
ros2 run agt_system_manager replay_mapping_to_workspace.py \
  --workspace-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --source-bag /absolute/path/to/rosbag2 \
  --platform-profile profiles/platforms/bunker.yaml
```

It starts the existing managed MappingSession with `start_sensor=false` and `use_sim_time=true`, replays only mapping-input topics, finalizes the managed static candidate, and calls the same ingestion primitive exposed below. The result remains `PROCESSING`: session-frame products are evidence, not canonical map assets.

A finalized MappingSession may also be ingested manually:

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py ingest-mapping-session \
  --manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --session-file <session.yaml> \
  --session-id <session_id> \
  --candidate-map-yaml <ground_temporal.yaml> \
  --candidate-map-image <ground_temporal.pgm> \
  --localization-pcd <localization_map.pcd> \
  --processing-record <localization_map.processing.yaml> \
  --derived-bag <mapping_session_bag> \
  --source-bag <original_frozen_bag>
```

The copied evidence is stored under:

```text
processing/mapping_session/
  handoff.yaml
  evidence/session.yaml
  candidate/...
  localization/...
```

`navigation/map.*` and `pointcloud/localization_map.pcd` are intentionally not created by ingestion. A later alignment/materialization stage must transform the session-frame products into the canonical `map` frame first.

After alignment/cleaning writes canonical products, hash and promote them:

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py refresh-map \
  --manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --state READY
```

Derive a semantic route candidate:

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py derive-route \
  --map-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --semantic runtime/maps/greenhouse_a/versions/<version>/semantic/semantic_map.geojson \
  --coverage runtime/maps/greenhouse_a/versions/<version>/semantic/coverage.yaml \
  --policy route_policy.yaml \
  --platform-profile profiles/platforms/bunker.yaml \
  --route-id inspection_main --revision 1
```

Run vehicle feasibility and preview export:

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-route \
  --route-dir runtime/maps/greenhouse_a/versions/<version>/routes/inspection_main/1 \
  --map-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --platform-profile profiles/platforms/bunker.yaml
```

The first route generator is intentionally a deterministic annotated-row/boustrophedon MVP. Its inter-lane connector is a straight candidate. If the selected vehicle cannot execute that connector, the existing full-footprint/minimum-turning-radius validator rejects it. A Hybrid-A*/State-Lattice/Reeds-Shepp connector backend can replace that internal generator later without changing the Route Asset contract.
