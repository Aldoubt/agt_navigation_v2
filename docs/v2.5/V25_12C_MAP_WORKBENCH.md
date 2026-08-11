# V25-12C AGT Map Workbench MVP

Status: IMPLEMENTED, LOCAL ACCEPTANCE PENDING

## Goal

Provide a visual authoring client over the frozen V25-12B point-cloud processing contract

The Workbench must make scene-boundary and object-removal authoring easier without creating a second point-cloud editing truth

## Architecture

```text
agt_map_workbench
  Qt5 visual authoring only
        ↓
agt_pointcloud_processing_recipe/v1
        ↓
agt_offline_assets V25-12B
        ↓
immutable processing run
```

The Workbench is an Offline Plane package

It does not publish ROS topics, own TF, command the vehicle or modify a READY Map Version

## MVP renderer

The first renderer is intentionally 2.5D rather than a full 3D CAD environment

```text
full PCD
  ↓
deterministic display sample
  ↓
XY top view
+ height colour
+ explicit Z window
```

This directly supports the most important first authoring primitive

```text
XY polygon × [z_min, z_max]
```

which maps to V25-12B `crop_polygon` or `delete_polygon`

A later true 3D renderer may replace the view without changing recipe semantics

## Implemented

Package

```text
src/agt_map_workbench
```

Capabilities

- open ASCII, binary and PCL binary_compressed PCD through `agt_offline_assets`
- deterministic sampled display for multi-million-point maps
- XY zoom/pan
- Z visibility window
- map-coordinate polygon vertex authoring
- 3D polygon-volume crop/delete authoring
- recipe operation history
- undo/clear operations
- export `agt_pointcloud_processing_recipe/v1`
- run the canonical V25-12B processing core in a Qt worker thread
- preserve the original source PCD
- optionally load the processed PCD for visual review

Formal processing always uses the full source cloud; display sampling is never used as processing input

## Real greenhouse design decision

The V25-12B profile of `greenhouse_aligned_full2.pcd` shows that a global Z crop cannot be justified safely from statistics alone

Therefore V25-12C focuses first on visually authored site/object boundaries rather than imposing one automatic height threshold

The master/source PCD also retains CloudCompare analysis fields

Localization-product field normalization is a later derived-product decision, not a destructive Workbench edit

## Acceptance gate

Build

```bash
source /opt/ros/humble/setup.bash
rm -rf build/agt_map_workbench install/agt_map_workbench
colcon build --symlink-install --packages-up-to agt_map_workbench
source install/setup.bash
```

Model tests

```bash
python3 -m pytest -q \
  src/agt_map_workbench/test/test_workbench_model.py
```

Repository contract

```bash
python3 -m pytest -q \
  tests/test_v25_12c_map_workbench_contract.py
```

Regression with V25-12B

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_pcd_compressed.py \
  src/agt_offline_assets/test/test_pointcloud_profile.py \
  src/agt_offline_assets/test/test_pointcloud_processing.py \
  tests/test_v25_12b_pointcloud_contract.py
```

Launch

```bash
ros2 run agt_map_workbench agt_map_workbench.py
```

Real-data smoke

1. open `runtime/maps/greenhouse_ground/pcd/greenhouse_aligned_full2.pcd`
2. verify the 5.19M-point compressed PCD opens and the view remains interactive
3. adjust the visible Z range and verify display filtering
4. start a polygon and mark a visually obvious outside/temporary region
5. choose delete or crop
6. finish the polygon and verify an operation appears in the recipe list
7. export YAML and confirm `polygon_xy`, `z_min`, `z_max` are in map coordinates
8. run immutable processing into a new directory
9. confirm V25-12B produces `processing.yaml`, `processing_report.json`, `processed.pcd`, `recipe.yaml`
10. confirm the original PCD SHA remains unchanged

## Not yet frozen in this MVP

- voxel/SOR/radius parameter widgets
- fast non-formal operation preview
- recipe import/replay UI
- semantic overlay
- Localization Prior overlay
- true free-rotation 3D renderer
- field-projection policy for derived Localization Map
- automatic stable-structure classification

These features may be added incrementally without bypassing the V25-12B recipe executor
