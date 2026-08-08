# agt_offline_assets

Pure offline tooling for V25-09A. It owns no ROS graph state and does not publish TF or velocity.

The package turns the frozen asset contracts into executable file workflows:

```text
Dataset + Calibration + Platform + Recipe
  -> PROCESSING map workspace
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

After the mapping/cleaning pipeline writes canonical products, hash and promote them:

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
