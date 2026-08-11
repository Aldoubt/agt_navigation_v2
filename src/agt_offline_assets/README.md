# agt_offline_assets

Pure offline asset-preparation primitives for AGT Navigation V2.5

The package itself owns no ROS graph state and does not publish TF or velocity

V25-09A established reproducible Map Version and Route Asset preparation; V25-12A extends that existing lineage with a Site Package root identity rather than creating a second map/route schema

The current offline chain is

```text
Dataset + Calibration + Platform + Recipe
  -> PROCESSING Map Version workspace
  -> MappingSession evidence (session frame)
  -> alignment / cleaning / canonical materialization
  -> derived map products + quality reports
  -> READY Map Version
  -> Semantic Map + Route Policy
  -> DRAFT Route Asset
  -> full-footprint / kinematic feasibility
  -> preview.geojson
  -> READY or INVALID Route revision
  -> READY map + selected execution vehicle + optional READY routes
  -> DRAFT Site Package
  -> read-only cross-binding validation
  -> READY Site Package
```

The Site Package is the future deployment identity root

It binds existing stable identities and hashes; it does not copy full map/route content or redefine their internal contracts

## CLI

The executable is intentionally named `agt_offline_assets_cli.py` rather than `agt_offline_assets.py` so it cannot shadow the Python package `agt_offline_assets` when launched through `ros2 run`

### Map Version

Create a versioned workspace

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

For a real recorded bag, the recommended next command is the system-manager replay orchestrator

```bash
ros2 run agt_system_manager replay_mapping_to_workspace.py \
  --workspace-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --source-bag /absolute/path/to/rosbag2 \
  --platform-profile profiles/platforms/bunker.yaml
```

It starts the existing managed MappingSession with `start_sensor=false` and `use_sim_time=true`, replays only mapping-input topics, finalizes the managed static candidate, and calls the same ingestion primitive exposed below

The result remains `PROCESSING`: session-frame products are evidence, not canonical map assets

A finalized MappingSession may also be ingested manually

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

The copied evidence is stored under

```text
processing/mapping_session/
  handoff.yaml
  evidence/session.yaml
  candidate/...
  localization/...
```

`navigation/map.*` and `pointcloud/localization_map.pcd` are intentionally not created by ingestion

A later alignment/materialization stage must transform the session-frame products into the canonical `map` frame first

After alignment/cleaning writes canonical products, hash and promote them

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py refresh-map \
  --manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --state READY
```

Audit without mutation

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-map \
  --manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml
```

### Route Asset

Derive a semantic route candidate

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py derive-route \
  --map-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --semantic runtime/maps/greenhouse_a/versions/<version>/semantic/semantic_map.geojson \
  --coverage runtime/maps/greenhouse_a/versions/<version>/semantic/coverage.yaml \
  --policy route_policy.yaml \
  --platform-profile profiles/platforms/bunker.yaml \
  --route-id inspection_main --revision 1
```

Run vehicle feasibility and preview export

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-route \
  --route-dir runtime/maps/greenhouse_a/versions/<version>/routes/inspection_main/1 \
  --map-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --platform-profile profiles/platforms/bunker.yaml
```

The first route generator is intentionally a deterministic annotated-row/boustrophedon MVP

Its inter-lane connector is a straight candidate

If the selected vehicle cannot execute that connector, the existing full-footprint/minimum-turning-radius validator rejects it

A Hybrid-A*/State-Lattice/Reeds-Shepp connector backend can replace that internal generator later without changing the Route Asset contract

### V25-12A Site Package

Create a DRAFT root binding from one compliant READY Map Version, one execution-vehicle profile, and optional READY Route revisions

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py create-site-package \
  --sites-root runtime/sites \
  --map-manifest runtime/maps/greenhouse_a/versions/<version>/manifest.yaml \
  --platform-profile profiles/platforms/bunker.yaml \
  --route-dir runtime/maps/greenhouse_a/versions/<version>/routes/inspection_main/1
```

The result is

```text
runtime/sites/<site_id>/packages/<site_package_id>/manifest.yaml
```

The manifest stores stable identities/hashes rather than arbitrary paths to map and Route files

Promote only after the cross-binding validator passes

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py refresh-site-package \
  --manifest runtime/sites/<site_id>/packages/<site_package_id>/manifest.yaml \
  --maps-root runtime/maps \
  --platform-profile profiles/platforms/bunker.yaml \
  --state READY
```

Read-only audit

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-site-package \
  --manifest runtime/sites/<site_id>/packages/<site_package_id>/manifest.yaml \
  --maps-root runtime/maps \
  --platform-profile profiles/platforms/bunker.yaml
```

V25-12A validation checks

- Site Package schema / identity / content hash
- selected Map Version is still compliant READY
- exact map ID / version / `map_content_sha256`
- inherited Calibration binding
- Localization Map + processing-record hashes
- Navigation YAML + PGM hashes
- Semantic Map/Coverage hashes when present
- selected execution-vehicle profile/hash
- every declared Route is canonical READY and matches the same map + vehicle
- Route CSV / policy / feasibility / preview hashes
- `feasibility_report.status=PASS`

A READY Site Package is immutable from the offline preparation perspective

Binding changes create a new `site_package_id`

## Architecture boundary

`agt_offline_assets` remains a pure offline owner

It may prepare/validate

```text
Dataset
Map Version
Localization Map
future Localization Prior
Navigation Map
Semantic Map
Route Asset
Site Package
Benchmark evidence
```

It does not

- publish runtime TF
- publish vehicle velocity
- own Mission state
- become Localization Authority
- become runtime map registry owner

Future `LoadSitePackage` and navigation BT nodes must consume project-level identity/capability interfaces rather than parsing these files directly inside BehaviorTree nodes

See

- `docs/interfaces/site_package_manifest.md`
- `docs/v2.5/V25_12_SITE_WORKFLOW_REQUIREMENTS.md`
- `docs/architecture/offline_asset_pipeline.md`
- `docs/architecture/behavior_tree_execution.md`
