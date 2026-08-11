# V25-12B Point Cloud Processing CLI

Status: IMPLEMENTED, LOCAL ACCEPTANCE PENDING

## Goal

Provide the deterministic non-destructive point-cloud processing kernel that V25-12C AGT Map Workbench will call

The stage is complete only when the same input PCD and recipe produce a validated immutable processing run and the existing V25-12A/offline contracts remain green

## Implemented

### PCD I/O

`src/agt_offline_assets/agt_offline_assets/pcd_io.py`

Supports

- PCD v0.7-style schema parsing
- arbitrary scalar/vector field preservation
- `DATA ascii`
- uncompressed `DATA binary`
- PCL `DATA binary_compressed` input through bounds-checked LZF decoding and field-major/SoA restoration
- deterministic ASCII/BINARY writing
- required scalar x/y/z geometry fields

Formal processing output remains normalized to ASCII or uncompressed binary; V25-12B does not emit `binary_compressed`

### Recipe executor

`src/agt_offline_assets/agt_offline_assets/pointcloud_processing.py`

Recipe schema

```text
agt_pointcloud_processing_recipe/v1
```

Processing record

```text
agt_pointcloud_processing/v1
```

Report

```text
agt_pointcloud_processing_report/v1
```

Supported operations

```text
remove_nonfinite
crop_box
crop_polygon
delete_polygon
height_range
voxel_downsample
sor
radius_outlier
ground_separation / plane_ransac
```

### CLI

```text
inspect-pcd
process-pointcloud
validate-pointcloud-processing
```

### Dependencies

- numpy
- scipy for KD-tree SOR/radius queries

No Open3D dependency is required by the canonical processing core

V25-12C may use Open3D or another renderer for visualization without changing the processing contract

## Processing artifact

```text
<run_dir>/
├── recipe.yaml
├── processed.pcd
├── processing.yaml
└── processing_report.json
```

The source is never overwritten

Existing run directories are not reused

The processing artifact is not automatically copied into a Map Version and does not become READY navigation/localization truth by itself

## Ground backend boundary

The first ground backend is seeded near-horizontal plane RANSAC

It is intentionally a baseline only

The agricultural production pipeline may later add PMF, CSF, terrain-adaptive or learned ground backends behind the same operation/evidence boundary

## V25-12B Acceptance Gate

### Build

```bash
rm -rf build/agt_offline_assets install/agt_offline_assets
colcon build --symlink-install --packages-up-to agt_offline_assets
source install/setup.bash
```

If numpy/scipy are missing, resolve package dependencies first with the project rosdep workflow

### Package tests

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_pointcloud_processing.py
```

Then

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test
```

### Repository contracts

```bash
python3 -m pytest -q \
  tests/test_v25_12b_pointcloud_contract.py \
  tests/test_navigation_offline_contracts.py \
  tests/test_offline_asset_contract.py \
  tests/test_navigation_architecture_contract.py
```

### CLI smoke

Use a real mapping PCD or small fixture

PCL `binary_compressed` input is supported directly, so a CloudCompare/PCL-produced map does not need a manual conversion step before inspection

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py inspect-pcd \
  --input <input.pcd>
```

Then copy and tune the example recipe rather than editing a READY map

```bash
cp docs/interfaces/examples/pointcloud_processing/recipe.yaml /tmp/agt_pc_recipe.yaml
```

Before processing, adjust `crop_box`, `height_range`, and other scene-dependent values according to the bounds reported by `inspect-pcd`

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py process-pointcloud \
  --input <input.pcd> \
  --recipe /tmp/agt_pc_recipe.yaml \
  --output-dir /tmp/agt_pc_run
```

```bash
ros2 run agt_offline_assets agt_offline_assets_cli.py validate-pointcloud-processing \
  --run-dir /tmp/agt_pc_run
```

Required

```text
valid = true
processing_report.status = PASS
source SHA256 unchanged
processing output SHA256 stable across repeated run with same input/recipe
```

## Not yet implemented

The following are intentionally outside V25-12B

- AGT Map Workbench GUI
- interactive 3D picking/brush rendering
- Localization Prior weight authoring
- PMF/CSF production ground backend
- automatic stable-structure classification
- automatic map-quality/relocalization acceptance from processing PASS
- direct modification of READY Map Versions
- tiled/out-of-core point-cloud processing

Large-map out-of-core/chunked processing belongs to later V25-16 work; V25-12B currently processes one PCD in memory

## Next stage

After this Gate passes, V25-12C builds AGT Map Workbench as a visual client over the frozen V25-12B recipe executor
