# agt_map_workbench

Offline visual point-cloud authoring client for AGT Navigation V2.5

The Workbench is a client of the V25-12B `agt_offline_assets` processing contract

It does not publish ROS topics, does not own TF, does not command the vehicle, and does not modify READY map assets in place

## V25-12C MVP

The first MVP uses the existing project Qt5/PyQt5 toolchain and a scalable 2.5D point-cloud view

```text
PCD (ascii / binary / PCL binary_compressed)
        ↓
full cloud loaded by agt_offline_assets
        ↓
deterministic display sampling only
        ↓
XY top view + height colour
        ↓
polygon authoring + explicit Z interval
        ↓
crop_polygon / delete_polygon
        ↓
agt_pointcloud_processing_recipe/v1
        ↓
V25-12B immutable full-resolution processing
```

Display sampling never changes the formal processing input

The full-resolution source PCD is always passed to `process_pointcloud`

## Start

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_map_workbench agt_map_workbench
```

The installed executable is intentionally named `agt_map_workbench` while its build-tree launcher is `map_workbench_launcher.py`; the launcher filename must differ from the Python package name to avoid Python import shadowing under `--symlink-install`

## Current tools

- open PCD including PCL `binary_compressed`
- deterministic sampled point display for multi-million-point maps
- XY zoom/pan
- Z visibility window
- polygon authoring in map coordinates
- `delete_polygon` volume authoring
- `crop_polygon` volume authoring
- operation history
- undo/clear operations
- export canonical V25-12B recipe YAML
- run immutable full-resolution processing in a Qt worker thread
- optionally load the processed PCD for visual review

A polygon edit is not stored as a private GUI file

For example, deleting one 3D selection produces

```yaml
- type: delete_polygon
  parameters:
    polygon_xy:
      - [1.0, 2.0]
      - [4.0, 2.0]
      - [4.0, 5.0]
      - [1.0, 5.0]
    z_min: -1.5
    z_max: 4.5
```

This is directly replayable by V25-12B

## Product boundary

The Workbench edits a processing draft, not a READY map

```text
source/master PCD
   ↓ visual authoring
recipe draft
   ↓ V25-12B executor
immutable processing run
   ↓ review / quality gate
later explicit Map Version materialization
```

The source/master PCD should retain useful analysis fields

Localization-product field normalization such as selecting only `x/y/z/intensity` is a separate derived-product policy and must not silently destroy the master asset

## Next MVP increments

- numerical filter panel for voxel/SOR/radius/height operations
- fast operation preview without producing a formal run
- original/processed visual toggle
- selection list visibility and editing
- recipe import/replay
- semantic-map overlay
- Localization Prior overlay reserved for V25-12D
- optional true 3D renderer after the 2.5D authoring contract is stable
